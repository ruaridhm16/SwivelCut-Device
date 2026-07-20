#include "TeachReplay.h"

#include <Arduino.h>
#include <math.h>

#include "Blade.h"
#include "CommandLine.h"
#include "Config.h"
#include "Diagnostics.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "Panel.h"
#include "StepperMotion.h"

ProductState productState = ProductState::IDLE;
bool productReady = false;
bool stabilizationEnabled = false;
bool repeatCutActive = false;
bool productCutActive = false;
bool productAbortRequested = false;
bool productHasLastCut = false;
bool tracerReplayActive = false;
int taughtCount = 0;

namespace {

TeachPoint taught[MAX_TEACH_POINTS];
TeachPoint rawTaught[MAX_TEACH_POINTS];
bool taughtJ1Only = false;
unsigned long productTeachStartedMs = 0;
unsigned long nextProductTeachSampleMs = 0;

// Smooths each taught point in Cartesian (XY) space rather than joint space,
// which keeps the tip path itself smooth regardless of joint geometry. Falls
// back to a shrinking time window, and finally the raw point, if a smoothed
// XY position turns out to be unreachable.
void stabilizeTeachPointsXY(float smoothingMs, float maxDeviationDeg) {
  if (taughtCount < 3 || smoothingMs <= 0.0f) return;

  float *xyScratch =
      static_cast<float *>(malloc(taughtCount * 2 * sizeof(float)));
  if (xyScratch == nullptr) {
    Serial.println("XY_SMOOTH_SKIPPED insufficient memory");
    return;
  }
  float *scratchX = xyScratch;
  float *scratchY = xyScratch + taughtCount;
  for (int i = 0; i < taughtCount; ++i) {
    forwardKinematics(rawTaught[i].j1Deg, rawTaught[i].j2Deg, scratchX[i], scratchY[i]);
  }

  for (int i = 0; i < taughtCount; ++i) {
    const float rawX = scratchX[i];
    const float rawY = scratchY[i];
    const float reachMm = max(hypotf(rawX, rawY), 20.0f);
    const float maxDeviationMm = radians(maxDeviationDeg) * reachMm;
    const float windowsMs[] = {smoothingMs, smoothingMs * 0.5f, smoothingMs * 0.25f, 0.0f};
    const bool elbowDown = rawTaught[i].j2Deg < 0.0f;
    bool solved = false;

    for (size_t attempt = 0; attempt < sizeof(windowsMs) / sizeof(windowsMs[0]); ++attempt) {
      const float windowMs = windowsMs[attempt];
      float smoothX = rawX;
      float smoothY = rawY;
      if (windowMs > 0.0f) {
        float sumX = 0.0f;
        float sumY = 0.0f;
        int count = 0;
        for (int j = 0; j < taughtCount; ++j) {
          if (fabsf((rawTaught[j].seconds - rawTaught[i].seconds) * 1000.0f) <= windowMs) {
            sumX += scratchX[j];
            sumY += scratchY[j];
            ++count;
          }
        }
        smoothX = sumX / count;
        smoothY = sumY / count;
      }

      if (maxDeviationDeg > 0.0f) {
        smoothX = constrain(smoothX, rawX - maxDeviationMm, rawX + maxDeviationMm);
        smoothY = constrain(smoothY, rawY - maxDeviationMm, rawY + maxDeviationMm);
      }

      float smoothJ1 = 0.0f;
      float smoothJ2 = 0.0f;
      if (inverseKinematics(smoothX, smoothY, elbowDown, smoothJ1, smoothJ2)) {
        taught[i].j1Deg = smoothJ1;
        taught[i].j2Deg = smoothJ2;
        solved = true;
        if (attempt > 0) {
          Serial.print("XY_SMOOTH_REDUCED point=");
          Serial.print(i);
          Serial.print(" window_ms=");
          Serial.println(windowMs);
        }
        break;
      }
    }

    if (!solved) {
      taught[i].j1Deg = rawTaught[i].j1Deg;
      taught[i].j2Deg = rawTaught[i].j2Deg;
      Serial.print("XY_SMOOTH_FALLBACK point=");
      Serial.println(i);
    }
  }
  free(xyScratch);
}

void prepareTaughtPath(float smoothingMs, float maxDeviationDeg) {
  memcpy(taught, rawTaught, taughtCount * sizeof(TeachPoint));
  stabilizeTeachPointsXY(smoothingMs, maxDeviationDeg);
  if (taughtCount > 0) {
    taught[0] = rawTaught[0];
    taught[taughtCount - 1] = rawTaught[taughtCount - 1];
  }
}

bool compensateTaughtPathForCutter() {
  if (fabsf(CUTTER_EXTRA_LENGTH_MM) < 0.0001f) return true;

  for (int i = 0; i < taughtCount; ++i) {
    float tracedX = 0.0f;
    float tracedY = 0.0f;
    forwardKinematicsForLink2(taught[i].j1Deg, taught[i].j2Deg, LINK_2_MM, tracedX, tracedY);
    const bool elbowDown = taught[i].j2Deg < 0.0f;
    float cutterJ1 = 0.0f;
    float cutterJ2 = 0.0f;
    if (!inverseKinematicsForLink2(
            tracedX, tracedY, elbowDown, CUTTER_LINK_2_MM, cutterJ1, cutterJ2)) {
      Serial.print("ERROR_CUTTER_LENGTH_COMPENSATION POINT=");
      Serial.print(i);
      Serial.print(" X=");
      Serial.print(tracedX, 2);
      Serial.print(" Y=");
      Serial.println(tracedY, 2);
      return false;
    }
    taught[i].j1Deg = cutterJ1;
    taught[i].j2Deg = cutterJ2;
  }

  Serial.print("CUTTER_LENGTH_COMPENSATION_MM=");
  Serial.println(CUTTER_EXTRA_LENGTH_MM, 2);
  return true;
}

bool validateRawTeachPath(bool j1Only) {
  for (int i = 0; i < taughtCount; ++i) {
    const bool j1NearRange =
        rawTaught[i].j1Deg >= J1_MIN_DEG - TEACH_LIMIT_NOISE_MARGIN_DEG &&
        rawTaught[i].j1Deg <= J1_MAX_DEG + TEACH_LIMIT_NOISE_MARGIN_DEG;
    const bool j2NearRange =
        j1Only ||
        (rawTaught[i].j2Deg >= J2_MIN_DEG - TEACH_LIMIT_NOISE_MARGIN_DEG &&
         rawTaught[i].j2Deg <= J2_MAX_DEG + TEACH_LIMIT_NOISE_MARGIN_DEG);
    if (!ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS && (!j1NearRange || !j2NearRange)) {
      Serial.print("TEACH_REJECTED_OUT_OF_RANGE POINT=");
      Serial.print(i);
      Serial.print(" J1=");
      Serial.print(rawTaught[i].j1Deg, 2);
      Serial.print(" J2=");
      Serial.println(rawTaught[i].j2Deg, 2);
      taughtCount = 0;
      return false;
    }
    if (!ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS) {
      rawTaught[i].j1Deg = constrain(rawTaught[i].j1Deg, J1_MIN_DEG, J1_MAX_DEG);
      if (!j1Only) {
        rawTaught[i].j2Deg = constrain(rawTaught[i].j2Deg, J2_MIN_DEG, J2_MAX_DEG);
      }
    }
  }

  for (int i = 1; i < taughtCount; ++i) {
    const float jumpJ1 = fabsf(rawTaught[i].j1Deg - rawTaught[i - 1].j1Deg);
    const float jumpJ2 =
        fabsf(shortestJointDelta(rawTaught[i].j2Deg, rawTaught[i - 1].j2Deg));
    if (jumpJ1 > 5.0f || (!j1Only && jumpJ2 > 5.0f)) {
      Serial.print("TEACH_REJECTED_ENCODER_JUMP POINT=");
      Serial.print(i);
      Serial.print(" J1=");
      Serial.print(jumpJ1, 2);
      Serial.print(" J2=");
      Serial.println(jumpJ2, 2);
      taughtCount = 0;
      return false;
    }
  }
  if (ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS) {
    Serial.println(
        "TEACH_LIMIT_BYPASS_ACTIVE: recorded path retained outside "
        "software joint limits");
  }
  return true;
}

long trajectorySegmentStepEvents(int pointIndex) {
  if (pointIndex <= 0 || pointIndex >= taughtCount) return 0;
  const long deltaJ1 = lroundf(
      (taught[pointIndex].j1Deg - taught[pointIndex - 1].j1Deg) * J1_STEPS_PER_DEG);
  const long deltaJ2 = lroundf(
      shortestJointDelta(taught[pointIndex].j2Deg, taught[pointIndex - 1].j2Deg) *
      J2_STEPS_PER_DEG);
  return max(labs(deltaJ1), labs(deltaJ2));
}

bool trajectoryDirectionReversesAfter(int pointIndex) {
  if (pointIndex <= 0 || pointIndex >= taughtCount - 1) return false;
  const float currentJ1 = taught[pointIndex].j1Deg - taught[pointIndex - 1].j1Deg;
  const float nextJ1 = taught[pointIndex + 1].j1Deg - taught[pointIndex].j1Deg;
  const float currentJ2 =
      shortestJointDelta(taught[pointIndex].j2Deg, taught[pointIndex - 1].j2Deg);
  const float nextJ2 =
      shortestJointDelta(taught[pointIndex + 1].j2Deg, taught[pointIndex].j2Deg);
  return (currentJ1 * nextJ1 < 0.0f) || (currentJ2 * nextJ2 < 0.0f);
}

// Duration for one replay segment under a trapezoidal (accelerate/coast/
// decelerate) speed profile, given the step rate the arm is already moving
// at (entryRateHz, updated in place to become the next segment's entry rate).
unsigned long replaySegmentDurationUs(
    int pointIndex, long remainingSteps, float &entryRateHz) {
  const long stepEvents = trajectorySegmentStepEvents(pointIndex);
  if (stepEvents <= 0) {
    return static_cast<unsigned long>(1000000.0f / PRODUCT_TEACH_HZ);
  }

  const float acceleration = max(REPLAY_MAX_ACCEL_STEPS_PER_S2, 1.0f);
  const float reachableExitRate =
      sqrtf(entryRateHz * entryRateHz + 2.0f * acceleration * stepEvents);
  const float stoppingExitRate = sqrtf(2.0f * acceleration * max(remainingSteps, 0L));
  float exitRateHz = min(REPLAY_STEP_RATE_HZ, min(reachableExitRate, stoppingExitRate));
  if (trajectoryDirectionReversesAfter(pointIndex) || pointIndex == taughtCount - 1) {
    exitRateHz = 0.0f;
  }

  float durationSeconds = 0.0f;
  const float rateSum = entryRateHz + exitRateHz;
  if (rateSum > 0.001f) {
    durationSeconds = 2.0f * stepEvents / rateSum;
  } else {
    // A single segment starting and ending at rest uses a triangular profile.
    durationSeconds = 2.0f * sqrtf(stepEvents / acceleration);
  }
  entryRateHz = exitRateHz;
  return max(
      static_cast<unsigned long>(ceilf(durationSeconds * 1000000.0f)),
      static_cast<unsigned long>(stepEvents) * STEPPER_MIN_STEP_PERIOD_US);
}

bool serviceContinuousTrajectory() {
  serviceControlInputs();
  serviceEncoderStream();
  if (productCutActive && productAbortRequested) {
    Serial.println("CUT_ABORTED");
    return false;
  }
  return true;
}

bool executeContinuousTrajectory(int &stoppedPoint) {
  Serial.print("PLAY MODE: CONSTANT_PACE RATE_HZ=");
  Serial.print(REPLAY_STEP_RATE_HZ, 1);
  Serial.print(" MAX_ACCEL_STEPS_S2=");
  Serial.println(REPLAY_MAX_ACCEL_STEPS_PER_S2, 1);

  motorsMoving = true;
  long stepsSinceFeedback = 0;
  long remainingReplaySteps = 0;
  for (int i = 1; i < taughtCount; ++i) {
    remainingReplaySteps += trajectorySegmentStepEvents(i);
  }
  float replayRateHz = 0.0f;
  for (int i = 1; i < taughtCount; ++i) {
    const long targetJ1 = lroundf(taught[i].j1Deg * J1_STEPS_PER_DEG);
    const long deltaJ1 = targetJ1 - atomicReadSteps(j1PositionSteps);
    const long deltaJ2 = lroundf(
        shortestJointDelta(taught[i].j2Deg, currentJ2Deg()) * J2_STEPS_PER_DEG);
    const long total = max(labs(deltaJ1), labs(deltaJ2));
    remainingReplaySteps = max(0L, remainingReplaySteps - total);

    const unsigned long durationUs =
        replaySegmentDurationUs(i, remainingReplaySteps, replayRateHz);

    if (!startSegment(deltaJ1, deltaJ2, durationUs)) {
      stoppedPoint = i;
      return false;
    }
    long lastSegmentFeedbackSteps = 0;
    while (segmentInProgress()) {
      if (!serviceContinuousTrajectory()) {
        stopMotionSegment();
        stoppedPoint = i;
        return false;
      }
      const long emittedSteps = segmentEmittedStepEvents();
      stepsSinceFeedback += emittedSteps - lastSegmentFeedbackSteps;
      lastSegmentFeedbackSteps = emittedSteps;
      if (stepsSinceFeedback >= CONTINUOUS_FEEDBACK_STEP_INTERVAL) {
        stepsSinceFeedback %= CONTINUOUS_FEEDBACK_STEP_INTERVAL;
        if (!checkFeedback()) {
          stopMotionSegment();
          stoppedPoint = i;
          return false;
        }
      }
      delay(0);
    }
    stopMotionSegment();
  }
  motorsMoving = false;
  return true;
}

void printReplayEndpointReport(int targetPoint, bool completed) {
  if (taughtCount == 0) return;
  targetPoint = constrain(targetPoint, 0, taughtCount - 1);
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  Serial.print("REPLAY_ENDPOINT status=");
  Serial.print(completed ? "COMPLETE" : "STOPPED");
  Serial.print(" point=");
  Serial.print(targetPoint);
  Serial.print("/");
  Serial.print(taughtCount - 1);
  Serial.print(" target_J1=");
  Serial.print(taught[targetPoint].j1Deg, 2);
  Serial.print(" target_J2=");
  Serial.print(taught[targetPoint].j2Deg, 2);
  if (encoderJointAngles(measuredJ1, measuredJ2)) {
    Serial.print(" measured_J1=");
    Serial.print(measuredJ1, 2);
    Serial.print(" measured_J2=");
    Serial.print(measuredJ2, 2);
    Serial.print(" error_J1=");
    Serial.print(taught[targetPoint].j1Deg - measuredJ1, 2);
    Serial.print(" error_J2=");
    Serial.print(shortestJointDelta(taught[targetPoint].j2Deg, measuredJ2), 2);
  } else {
    Serial.print(" measured=READ_ERROR");
  }
  Serial.println();
}

void reversePreparedTaughtPath() {
  const float totalSeconds = taught[taughtCount - 1].seconds;
  for (int i = 0; i < taughtCount / 2; ++i) {
    const int opposite = taughtCount - 1 - i;
    const TeachPoint first = taught[i];
    const TeachPoint last = taught[opposite];
    taught[i] = {totalSeconds - last.seconds, last.j1Deg, last.j2Deg};
    taught[opposite] = {totalSeconds - first.seconds, first.j1Deg, first.j2Deg};
  }
  if (taughtCount % 2 == 1) {
    const int middle = taughtCount / 2;
    taught[middle].seconds = totalSeconds - taught[middle].seconds;
  }
}

bool prepareTracerReplayFromNearestEndpoint() {
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  if (!encoderJointAngles(measuredJ1, measuredJ2)) {
    feedbackFault("AS5600 read failed before tracer replay");
    return false;
  }
  const float firstDistance = hypotf(
      measuredJ1 - taught[0].j1Deg, shortestJointDelta(measuredJ2, taught[0].j2Deg));
  const float lastDistance = hypotf(
      measuredJ1 - taught[taughtCount - 1].j1Deg,
      shortestJointDelta(measuredJ2, taught[taughtCount - 1].j2Deg));
  if (lastDistance < firstDistance) {
    reversePreparedTaughtPath();
    Serial.println("TRACER_REPLAY_DIRECTION=REVERSE_FROM_NEAREST_END");
  } else {
    Serial.println("TRACER_REPLAY_DIRECTION=FORWARD_FROM_NEAREST_END");
  }
  return true;
}

bool sampleProductTeach(unsigned long now) {
  if (taughtCount >= MAX_TEACH_POINTS) {
    Serial.println("TEACHING_STOPPED_MEMORY_LIMIT");
    return false;
  }
  float j1 = 0.0f;
  float j2 = 0.0f;
  if (!encoderJointAngles(j1, j2)) {
    feedbackFault("AS5600 read failed while teaching");
    taughtCount = 0;
    return false;
  }
  rawTaught[taughtCount++] = {(now - productTeachStartedMs) / 1000.0f, j1, j2};
  return true;
}

void startProductTeach(unsigned long now) {
  if (!productReady || !encodersCalibrated || encoderMode != AxisMode::DUAL) {
    Serial.println("ERROR_PRODUCT_NOT_READY_USE_ARM_FOLDED");
    return;
  }
  if (!headTypeKnown() || currentHeadType() != HeadType::TRACING) {
    Serial.println("ERROR_TRACER_HEAD_REQUIRED");
    return;
  }

  disableDrivers();
  taughtJ1Only = false;
  taughtCount = 0;
  productHasLastCut = false;
  productTeachStartedMs = now;
  nextProductTeachSampleMs = now;
  productState = ProductState::TEACHING;
  if (!sampleProductTeach(now)) {
    productState = ProductState::IDLE;
    return;
  }
  nextProductTeachSampleMs = now + static_cast<unsigned long>(1000.0f / PRODUCT_TEACH_HZ);
  Serial.println("TRACING_STARTED_PRESS_AGAIN_TO_STOP");
}

void stopProductTeach(unsigned long now) {
  if (productState != ProductState::TEACHING) return;
  if (taughtCount == 0 ||
      rawTaught[taughtCount - 1].seconds < (now - productTeachStartedMs) / 1000.0f) {
    sampleProductTeach(now);
  }
  productState = ProductState::IDLE;
  if (taughtCount < 2) {
    taughtCount = 0;
    Serial.println("ERROR_TRACED_PATH_TOO_SHORT");
    printOperationReport("TRACING_STOPPED_ERROR");
    return;
  }
  if (!validateRawTeachPath(false)) {
    printOperationReport("TRACING_REJECTED");
    return;
  }
  prepareTaughtPath(0.0f, 0.0f);

  float j1 = 0.0f;
  float j2 = 0.0f;
  if (!encoderJointAngles(j1, j2)) {
    feedbackFault("AS5600 read failed after teaching");
    taughtCount = 0;
    return;
  }
  atomicWriteSteps(j1PositionSteps, lroundf(j1 * J1_STEPS_PER_DEG));
  atomicWriteSteps(j2PositionSteps, lroundf(j2 * J2_STEPS_PER_DEG));
  Serial.print("TRACING_STOPPED POINTS=");
  Serial.println(taughtCount);
  printOperationReport("TRACING_COMPLETE");
}

}  // namespace

void armAtFoldedPose(AxisMode mode, bool enableAfterCalibration) {
  disableDrivers();
  atomicWriteSteps(j1PositionSteps, 0);
  atomicWriteSteps(j2PositionSteps, lroundf(180.0f * J2_STEPS_PER_DEG));
  encoderFault = false;
  armMode = mode;
  encodersCalibrated = false;
  encoderMode = mode;
  productReady = false;
  productState = ProductState::IDLE;
  if (USE_ENCODERS) {
    const bool j1Ok = mode == AxisMode::J2_ONLY || j1Encoder.calibrate();
    const bool j2Ok = mode == AxisMode::J1_ONLY || j2Encoder.calibrate();
    if (!j1Ok || !j2Ok) {
      Serial.println("ERROR: encoder or magnet check failed");
      return;
    }
    encodersCalibrated = true;
  } else if (mode != AxisMode::DUAL) {
    Serial.println("ERROR: single-axis ARM requires an encoder branch");
    return;
  }
  productReady = mode == AxisMode::DUAL;
  if (enableAfterCalibration) {
    enableDrivers();
    armed = true;
  } else {
    disableDrivers();
  }
  if (mode == AxisMode::J1_ONLY) {
    Serial.println("ARMED J1 TEST: only J1 motion is allowed");
  } else if (mode == AxisMode::J2_ONLY) {
    Serial.println("ARMED J2 TEST: J2 homed at 180; only J2 motion is allowed");
  } else if (!enableAfterCalibration) {
    Serial.println("HOME CALIBRATED at J1=0, J2=180; motor drivers remain disabled");
  } else {
    Serial.println("ARMED at J1=0, J2=180; physical product buttons enabled");
  }
  printOperationReport("ARM_COMPLETE");
}

void recordTeach(
    float seconds, float hz, bool j1Only, float smoothingMs, float maxDeviationDeg) {
  if (!USE_ENCODERS) {
    Serial.println("ERROR: this branch has no AS5600 teach support");
    return;
  }
  if ((j1Only && armMode != AxisMode::J1_ONLY) ||
      (!j1Only && armMode != AxisMode::DUAL)) {
    Serial.println(
        armMode == AxisMode::J2_ONLY ? "ERROR: TEACH is unavailable in ARM J2 mode"
                                      : "ERROR: use ARM J1 for TEACH J1 or ARM FOLDED for TEACH");
    return;
  }
  if (!armed || seconds <= 0.0f || seconds > 60.0f || hz < 1.0f || hz > 50.0f) {
    Serial.println("ERROR: TEACH needs 0-60 seconds and 1-50 Hz");
    return;
  }
  const int requested = static_cast<int>(ceilf(seconds * hz));
  if (requested > MAX_TEACH_POINTS) {
    Serial.println("ERROR: too many teach points");
    return;
  }

  disableDrivers();
  taughtCount = 0;
  taughtJ1Only = j1Only;
  productHasLastCut = false;
  const unsigned long started = millis();
  const unsigned long interval = static_cast<unsigned long>(1000.0f / hz);
  for (int i = 0; i < requested; ++i) {
    while (millis() - started < static_cast<unsigned long>(i) * interval) {
      serviceControlInputs();
      serviceEncoderStream();
      delay(1);
    }
    float j1 = 0.0f;
    float j2 = 0.0f;
    if (!encoderJointAngles(j1, j2)) {
      feedbackFault("AS5600 read failed while teaching");
      taughtCount = 0;
      return;
    }
    rawTaught[taughtCount++] = {(millis() - started) / 1000.0f, j1, j1Only ? 180.0f : j2};
  }
  for (int i = 1; i < taughtCount; ++i) {
    const float jumpJ1 = fabsf(rawTaught[i].j1Deg - rawTaught[i - 1].j1Deg);
    const float jumpJ2 =
        fabsf(shortestJointDelta(rawTaught[i].j2Deg, rawTaught[i - 1].j2Deg));
    if (jumpJ1 > 5.0f || (!j1Only && jumpJ2 > 5.0f)) {
      Serial.print("TEACH REJECTED: encoder jump at point ");
      Serial.print(i);
      Serial.print(" J1=");
      Serial.print(jumpJ1, 2);
      if (!j1Only) {
        Serial.print(" J2=");
        Serial.print(jumpJ2, 2);
      }
      Serial.println(" deg");
      taughtCount = 0;
      return;
    }
  }
  prepareTaughtPath(smoothingMs, maxDeviationDeg);
  Serial.print("TAUGHT: ");
  Serial.print(taughtCount);
  Serial.print(" points, J1 ");
  Serial.print(taught[0].j1Deg, 2);
  Serial.print(" -> ");
  Serial.print(taught[taughtCount - 1].j1Deg, 2);
  Serial.println("; type PLAY");
  printOperationReport("TEACH_COMPLETE");
}

bool replayTeach(bool operateBlade) {
  if (!USE_ENCODERS || taughtCount == 0) {
    Serial.println("ERROR: no taught movement");
    return false;
  }
  armMode = taughtJ1Only ? AxisMode::J1_ONLY : AxisMode::DUAL;
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  if (!encoderJointAngles(measuredJ1, measuredJ2)) {
    feedbackFault("AS5600 read failed before replay");
    return false;
  }
  atomicWriteSteps(j1PositionSteps, lroundf(measuredJ1 * J1_STEPS_PER_DEG));
  atomicWriteSteps(j2PositionSteps, lroundf(measuredJ2 * J2_STEPS_PER_DEG));
  enableDrivers();
  armed = true;
  Serial.print("PLAY RETURN: J1 ");
  Serial.print(measuredJ1, 2);
  Serial.print(" -> ");
  Serial.print(taught[0].j1Deg, 2);
  Serial.print("; J2 ");
  Serial.print(measuredJ2, 2);
  Serial.print(" -> ");
  Serial.println(taught[0].j2Deg, 2);
  const bool allowOutsideLimits = ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS;
  if (allowOutsideLimits) {
    Serial.println(
        "WARNING: taught-path software joint limits are bypassed; "
        "feedback and encoder-jump protection remain active");
  }
  if (operateBlade) bladeRetracted(true);
  if (!moveToAngles(taught[0].j1Deg, taught[0].j2Deg, false, allowOutsideLimits)) {
    disableDrivers();
    if (operateBlade && bladeIsDown()) bladeRetracted();
    return false;
  }
  if (operateBlade) bladeDown();

  int stoppedPoint = -1;
  if (CONTINUOUS_TRAJECTORY_REPLAY) {
    if (!executeContinuousTrajectory(stoppedPoint) ||
        !moveToAngles(
            taught[taughtCount - 1].j1Deg, taught[taughtCount - 1].j2Deg, false,
            allowOutsideLimits)) {
      if (stoppedPoint < 0) stoppedPoint = taughtCount - 1;
    }
  } else {
    for (int i = 1; i < taughtCount; ++i) {
      if (!moveToAngles(taught[i].j1Deg, taught[i].j2Deg, false, allowOutsideLimits)) {
        stoppedPoint = i;
        break;
      }
    }
  }
  if (stoppedPoint >= 0) {
    Serial.print("PLAY STOPPED AT POINT ");
    Serial.print(stoppedPoint);
    Serial.print("/");
    Serial.println(taughtCount - 1);
    printReplayEndpointReport(stoppedPoint, false);
    disableDrivers();
    if (bladeIsDown()) bladeRetracted();
    if (!operateBlade) printOperationReport("PLAY_STOPPED");
    return false;
  }

  printReplayEndpointReport(taughtCount - 1, true);
  disableDrivers();
  if (bladeIsDown()) bladeRetracted();
  Serial.print("PLAYED: ");
  Serial.print(taughtCount);
  Serial.println(" points");
  if (!operateBlade) printOperationReport("PLAY_COMPLETE");
  return true;
}

void runProductCut(bool repeat) {
  if (!productReady || !encodersCalibrated || encoderMode != AxisMode::DUAL) {
    Serial.println("ERROR_PRODUCT_NOT_READY_USE_ARM_FOLDED");
    return;
  }
  if (!headTypeKnown() || currentHeadType() != HeadType::CUTTING) {
    Serial.println("ERROR_CUTTER_HEAD_REQUIRED");
    return;
  }
  if (taughtCount < 2) {
    Serial.println("ERROR_NO_TRACED_PATH");
    return;
  }
  if (!repeat && productHasLastCut) {
    Serial.println("ERROR_CUT_ALREADY_COMPLETED_TEACH_ANOTHER_MOVEMENT");
    return;
  }
  if (repeat && !productHasLastCut) {
    Serial.println("ERROR_NO_COMPLETED_CUT_TO_REPEAT");
    return;
  }

  prepareTaughtPath(
      stabilizationEnabled ? PRODUCT_SMOOTHING_MS : 0.0f,
      stabilizationEnabled ? PRODUCT_MAX_DEVIATION_DEG : 0.0f);
  if (!compensateTaughtPathForCutter()) return;
  taughtJ1Only = false;
  productState = ProductState::CUTTING;
  repeatCutActive = repeat;
  productCutActive = true;
  productAbortRequested = false;
  refreshButtonLeds();
  Serial.print(repeat ? "REPEATING_LAST_CUT" : "CUTTING_STARTED");
  Serial.println(stabilizationEnabled ? "_WITH_STABILIZATION" : "");
  const bool completed = replayTeach(true);
  productCutActive = false;
  productState = ProductState::IDLE;
  repeatCutActive = false;
  refreshButtonLeds();
  disableDrivers();
  if (bladeIsDown()) bladeRetracted();
  if (completed) {
    productHasLastCut = true;
    Serial.println(repeat ? "REPEAT_COMPLETE" : "CUT_COMPLETE");
  } else {
    Serial.println(repeat ? "REPEAT_STOPPED" : "CUT_STOPPED");
  }
  printOperationReport(
      completed ? (repeat ? "REPEAT_COMPLETE" : "CUT_COMPLETE")
                : (repeat ? "REPEAT_STOPPED" : "CUT_STOPPED"));
}

void runTracerReplay() {
  if (!productReady || !encodersCalibrated || encoderMode != AxisMode::DUAL) {
    Serial.println("ERROR_PRODUCT_NOT_READY_USE_ARM_FOLDED");
    return;
  }
  if (!headTypeKnown() || currentHeadType() != HeadType::TRACING) {
    Serial.println("ERROR_TRACER_HEAD_REQUIRED");
    return;
  }
  if (taughtCount < 2) {
    Serial.println("ERROR_NO_TRACED_PATH");
    return;
  }

  prepareTaughtPath(
      stabilizationEnabled ? PRODUCT_SMOOTHING_MS : 0.0f,
      stabilizationEnabled ? PRODUCT_MAX_DEVIATION_DEG : 0.0f);
  if (!prepareTracerReplayFromNearestEndpoint()) return;
  taughtJ1Only = false;
  productState = ProductState::CUTTING;
  tracerReplayActive = true;
  repeatCutActive = true;
  productCutActive = true;
  productAbortRequested = false;
  refreshButtonLeds();
  Serial.println(
      stabilizationEnabled ? "TRACER_REPLAY_STARTED_WITH_STABILIZATION"
                            : "TRACER_REPLAY_STARTED");
  const bool completed = replayTeach(false);
  productCutActive = false;
  tracerReplayActive = false;
  productState = ProductState::IDLE;
  repeatCutActive = false;
  refreshButtonLeds();
  disableDrivers();
  Serial.println(completed ? "TRACER_REPLAY_COMPLETE" : "TRACER_REPLAY_STOPPED");
  printOperationReport(completed ? "TRACER_REPLAY_COMPLETE" : "TRACER_REPLAY_STOPPED");
}

void loadPointsFromSerial(long requestedCount) {
  if (requestedCount <= 1 || requestedCount > MAX_TEACH_POINTS) {
    Serial.print("ERROR: LOAD POINTS count out of range (1-");
    Serial.print(MAX_TEACH_POINTS);
    Serial.println(")");
    return;
  }
  if (!armed || armMode != AxisMode::DUAL) {
    Serial.println("ERROR: LOAD POINTS requires ARM FOLDED before reading any point lines");
    return;
  }

  inputLine = "";
  if (commandTerminator == '\r') {
    delay(1);
    if (Serial.available() > 0 && Serial.peek() == '\n') Serial.read();
  }
  const int pointCount = static_cast<int>(requestedCount);
  for (int i = 0; i < pointCount; ++i) {
    String pointLine = Serial.readStringUntil('\n');
    pointLine.trim();
    float j1Deg = 0.0f;
    float j2Deg = 0.0f;
    char trailing = '\0';
    if (sscanf(pointLine.c_str(), "%f %f %c", &j1Deg, &j2Deg, &trailing) != 2) {
      taughtCount = 0;
      inputLine = "";
      Serial.print("ERROR: LOAD POINTS bad point at line ");
      Serial.println(i);
      return;
    }
    rawTaught[i] = {i / PRODUCT_TEACH_HZ, j1Deg, j2Deg};
  }
  inputLine = "";

  taughtCount = pointCount;
  taughtJ1Only = false;
  for (int i = 0; i < taughtCount; ++i) {
    if (!angleInRange(rawTaught[i].j1Deg, rawTaught[i].j2Deg)) {
      Serial.print("ERROR: LOAD POINTS point ");
      Serial.print(i);
      Serial.print(" out of range J1=");
      Serial.print(rawTaught[i].j1Deg, 2);
      Serial.print(" J2=");
      Serial.println(rawTaught[i].j2Deg, 2);
      taughtCount = 0;
      return;
    }
  }
  if (!validateRawTeachPath(false)) return;

  productHasLastCut = false;
  prepareTaughtPath(0.0f, 0.0f);
  Serial.print("LOADED: ");
  Serial.print(taughtCount);
  Serial.print(" points, J1 ");
  Serial.print(taught[0].j1Deg, 2);
  Serial.print(" -> ");
  Serial.print(taught[taughtCount - 1].j1Deg, 2);
  Serial.println("; type PLAY or CUT LOADED");
  printOperationReport("LOAD_COMPLETE");
}

void cutLoadedPath() {
  if (!productReady || !encodersCalibrated || encoderMode != AxisMode::DUAL) {
    Serial.println("ERROR_PRODUCT_NOT_READY_USE_ARM_FOLDED");
    return;
  }
  if (!headTypeKnown() || currentHeadType() != HeadType::CUTTING) {
    Serial.println("ERROR_CUTTER_HEAD_REQUIRED");
    return;
  }
  if (taughtCount < 2) {
    Serial.println("ERROR_NO_TRACED_PATH");
    return;
  }

  taughtJ1Only = false;
  productState = ProductState::CUTTING;
  productCutActive = true;
  productAbortRequested = false;
  Serial.println("CUTTING_STARTED");
  const bool completed = replayTeach(true);
  productCutActive = false;
  productState = ProductState::IDLE;
  disableDrivers();
  if (bladeIsDown()) bladeRetracted();
  if (completed) {
    productHasLastCut = true;
    Serial.println("CUT_COMPLETE");
  } else {
    Serial.println("CUT_STOPPED");
  }
  printOperationReport(completed ? "CUT_COMPLETE" : "CUT_STOPPED");
}

void handleProductButtonChange(const ButtonInput &button) {
  const bool pressed = button.stableState == LOW;
  const unsigned long now = millis();

  if (button.number == 4) {
    if (!pressed) return;
    setMachinePower(!relayConnected);
    return;
  }

  if (button.number == 1) {
    if (!pressed) return;
    if (productState == ProductState::IDLE) {
      if (headTypeKnown() && currentHeadType() == HeadType::TRACING) {
        startProductTeach(now);
      } else if (headTypeKnown() && currentHeadType() == HeadType::CUTTING) {
        runProductCut(false);
      } else {
        Serial.println("ERROR_RECOGNIZED_HEAD_REQUIRED");
      }
    } else if (productState == ProductState::TEACHING) {
      stopProductTeach(now);
    } else if (productState == ProductState::CUTTING) {
      productAbortRequested = true;
      Serial.println("CUT_ABORT_REQUESTED_BUTTON_PRESSED");
    }
    return;
  }

  if (!pressed) return;
  if (button.number == 2) {
    if (productState != ProductState::IDLE) {
      Serial.println("STABILIZATION_CHANGE_IGNORED_ACTIVE_OPERATION");
      return;
    }
    stabilizationEnabled = !stabilizationEnabled;
    Serial.println(stabilizationEnabled ? "STABILIZATION_ON" : "STABILIZATION_OFF");
  } else if (button.number == 3) {
    if (productState != ProductState::IDLE) {
      Serial.println("REPEAT_IGNORED_ACTIVE_OPERATION");
      return;
    }
    if (headTypeKnown() && currentHeadType() == HeadType::TRACING) {
      runTracerReplay();
    } else if (headTypeKnown() && currentHeadType() == HeadType::CUTTING) {
      runProductCut(true);
    } else {
      Serial.println("ERROR_RECOGNIZED_HEAD_REQUIRED");
    }
  }
}

void serviceProductWorkflow() {
  if (anyPanelTestActive()) return;
  if (productState != ProductState::TEACHING) return;

  const unsigned long now = millis();
  if (now - productTeachStartedMs >=
      static_cast<unsigned long>(PRODUCT_TEACH_MAX_SECONDS * 1000.0f)) {
    stopProductTeach(now);
    Serial.println("TRACING_STOPPED_MAXIMUM_DURATION");
    return;
  }
  if (static_cast<long>(now - nextProductTeachSampleMs) >= 0) {
    if (!sampleProductTeach(now)) {
      productState = ProductState::IDLE;
      printOperationReport("TRACING_STOPPED_ERROR");
      return;
    }
    nextProductTeachSampleMs += static_cast<unsigned long>(1000.0f / PRODUCT_TEACH_HZ);
  }
}

bool moveToXY(float x, float y, bool elbowDown) {
  float j1 = 0.0f;
  float j2 = 0.0f;
  if (!inverseKinematics(x, y, elbowDown, j1, j2)) {
    Serial.println("ERROR: XY point unreachable or outside joint limits");
    return false;
  }
  return moveToAngles(j1, j2, false);
}

bool moveToXYAuto(float x, float y) {
  const bool preferredElbowDown = x < 0.0f;
  float j1 = 0.0f;
  float j2 = 0.0f;
  bool elbowDown = preferredElbowDown;
  if (!inverseKinematics(x, y, elbowDown, j1, j2)) {
    elbowDown = !elbowDown;
    if (!inverseKinematics(x, y, elbowDown, j1, j2)) {
      Serial.println("ERROR: XY point unreachable or outside joint limits");
      return false;
    }
  }
  Serial.print("XY AUTO: ");
  Serial.print(elbowDown ? "DOWN" : "UP");
  Serial.print(" J1=");
  Serial.print(j1, 2);
  Serial.print(" J2=");
  Serial.println(j2, 2);
  return moveToAngles(j1, j2, false);
}

bool cutLine(float x0, float y0, float x1, float y1, bool elbowDown) {
  const float distance = hypotf(x1 - x0, y1 - y0);
  const int segments = max(1, static_cast<int>(ceilf(distance / 2.0f)));
  for (int i = 0; i <= segments; ++i) {
    const float fraction = static_cast<float>(i) / segments;
    float j1 = 0.0f;
    float j2 = 0.0f;
    if (!inverseKinematics(
            x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction, elbowDown, j1, j2)) {
      Serial.println("ERROR: cut crosses unreachable workspace");
      return false;
    }
  }

  bladeRetracted(true);
  if (!moveToXY(x0, y0, elbowDown)) return false;
  bladeDown();
  for (int i = 1; i <= segments; ++i) {
    const float fraction = static_cast<float>(i) / segments;
    if (!moveToXY(x0 + (x1 - x0) * fraction, y0 + (y1 - y0) * fraction, elbowDown)) {
      if (bladeIsDown()) bladeRetracted();
      return false;
    }
  }
  if (bladeIsDown()) bladeRetracted();
  Serial.println("OK");
  printOperationReport("CUT_LINE_COMPLETE");
  return true;
}
