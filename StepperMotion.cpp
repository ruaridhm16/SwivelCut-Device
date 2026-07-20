#include "StepperMotion.h"

#include <driver/gpio.h>
#include <math.h>

#include "Blade.h"
#include "Config.h"
#include "Diagnostics.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "Panel.h"
#include "TeachReplay.h"

namespace {

// Per-axis state for the hardware-timer step generator. Every field the ISR
// touches is volatile and updated with explicit atomics so the main loop can
// read consistent values while the ISR fires concurrently.
struct StepAxisRuntime {
  int stepPin;
  hw_timer_t *timer;
  volatile long remainingSteps;
  volatile long emittedSteps;
  volatile int direction;
  volatile bool active;
  volatile bool pulseActive;
  bool timerRunning;
  volatile long *positionSteps;
};

StepAxisRuntime j1StepAxis = {
    J1_PUL_PIN, nullptr, 0, 0, 1, false, false, false, &j1PositionSteps};
StepAxisRuntime j2StepAxis = {
    J2_PUL_PIN, nullptr, 0, 0, 1, false, false, false, &j2PositionSteps};

bool motionSegmentActive = false;
unsigned long motionSegmentStartedUs = 0;
unsigned long motionSegmentDurationUs = 0;

void setDirection(int pin, long delta, bool invert) {
  bool forward = delta >= 0;
  if (invert) forward = !forward;
  digitalWrite(pin, forward ? HIGH : LOW);
}

void stopStepAxis(StepAxisRuntime &axis) {
  __atomic_store_n(&axis.active, false, __ATOMIC_RELEASE);
  if (axis.timerRunning) {
    timerStop(axis.timer);
    axis.timerRunning = false;
  }
  if (__atomic_load_n(&axis.pulseActive, __ATOMIC_ACQUIRE)) {
    // Finish at least the TB6600 minimum active width before forcing idle.
    delayMicroseconds(STEPPER_MIN_HALF_PERIOD_US);
  }
  digitalWrite(axis.stepPin, STEP_IDLE);
  __atomic_store_n(&axis.pulseActive, false, __ATOMIC_RELEASE);
}

bool configureStepAxis(
    StepAxisRuntime &axis, long deltaSteps, unsigned long durationUs) {
  const long stepCount = labs(deltaSteps);
  __atomic_store_n(&axis.remainingSteps, stepCount, __ATOMIC_RELEASE);
  __atomic_store_n(&axis.emittedSteps, 0L, __ATOMIC_RELEASE);
  __atomic_store_n(&axis.direction, deltaSteps >= 0 ? 1 : -1, __ATOMIC_RELEASE);
  __atomic_store_n(&axis.pulseActive, false, __ATOMIC_RELEASE);
  if (stepCount == 0) {
    __atomic_store_n(&axis.active, false, __ATOMIC_RELEASE);
    return true;
  }

  const uint64_t denominator = static_cast<uint64_t>(stepCount) * 2ULL;
  unsigned long halfPeriodUs = static_cast<unsigned long>(
      (static_cast<uint64_t>(durationUs) + denominator - 1ULL) / denominator);
  halfPeriodUs = max(halfPeriodUs, STEPPER_MIN_HALF_PERIOD_US);
  if (axis.timerRunning) timerStop(axis.timer);
  timerRestart(axis.timer);
  timerAlarm(axis.timer, halfPeriodUs, true, 0);
  __atomic_store_n(&axis.active, true, __ATOMIC_RELEASE);
  timerStart(axis.timer);
  axis.timerRunning = true;
  return true;
}

// Arduino-ESP32 3.x timerBegin(frequency) configures a general-purpose timer
// at the requested tick rate. Each axis uses a separate 1 MHz timer, so one
// alarm tick is 1 us. The ISR alternates STEP active/idle phases; changing
// STEPPER_TIMER_HZ or STEPPER_MIN_HALF_PERIOD_US retunes pulse resolution and
// the maximum allowed step rate without changing the trajectory code.
void ARDUINO_ISR_ATTR stepAxisTimerIsr(void *argument) {
  StepAxisRuntime *axis = static_cast<StepAxisRuntime *>(argument);
  if (!__atomic_load_n(&axis->active, __ATOMIC_ACQUIRE)) {
    gpio_set_level(static_cast<gpio_num_t>(axis->stepPin), STEP_IDLE);
    return;
  }

  if (!__atomic_load_n(&axis->pulseActive, __ATOMIC_RELAXED)) {
    gpio_set_level(static_cast<gpio_num_t>(axis->stepPin), STEP_ACTIVE);
    __atomic_store_n(&axis->pulseActive, true, __ATOMIC_RELEASE);
    __atomic_add_fetch(
        axis->positionSteps,
        __atomic_load_n(&axis->direction, __ATOMIC_RELAXED),
        __ATOMIC_RELAXED);
    __atomic_add_fetch(&axis->emittedSteps, 1L, __ATOMIC_RELAXED);
    __atomic_sub_fetch(&axis->remainingSteps, 1L, __ATOMIC_RELAXED);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(axis->stepPin), STEP_IDLE);
    __atomic_store_n(&axis->pulseActive, false, __ATOMIC_RELEASE);
    if (__atomic_load_n(&axis->remainingSteps, __ATOMIC_ACQUIRE) <= 0) {
      __atomic_store_n(&axis->active, false, __ATOMIC_RELEASE);
    }
  }
}

bool initializeStepperTimers() {
  j1StepAxis.timer = timerBegin(STEPPER_TIMER_HZ);
  j2StepAxis.timer = timerBegin(STEPPER_TIMER_HZ);
  if (j1StepAxis.timer == nullptr || j2StepAxis.timer == nullptr) {
    Serial.println("ERROR: failed to allocate stepper hardware timers");
    return false;
  }
  timerAttachInterruptArg(j1StepAxis.timer, stepAxisTimerIsr, &j1StepAxis);
  timerAttachInterruptArg(j2StepAxis.timer, stepAxisTimerIsr, &j2StepAxis);
  return true;
}

}  // namespace

volatile long j1PositionSteps = 0;
volatile long j2PositionSteps = lroundf(180.0f * J2_STEPS_PER_DEG);
bool armed = false;
AxisMode armMode = AxisMode::DUAL;
bool motorsMoving = false;
bool stepperTimersReady = false;
int lastCommandDirectionJ1 = 0;
int lastCommandDirectionJ2 = 0;

long atomicReadSteps(volatile long &steps) {
  return __atomic_load_n(&steps, __ATOMIC_ACQUIRE);
}

void atomicWriteSteps(volatile long &steps, long value) {
  __atomic_store_n(&steps, value, __ATOMIC_RELEASE);
}

float currentJ1Deg() { return atomicReadSteps(j1PositionSteps) / J1_STEPS_PER_DEG; }
float currentJ2Deg() { return atomicReadSteps(j2PositionSteps) / J2_STEPS_PER_DEG; }

void stopMotionSegment() {
  stopStepAxis(j1StepAxis);
  stopStepAxis(j2StepAxis);
  motionSegmentActive = false;
  motorsMoving = false;
}

bool startSegment(long deltaJ1, long deltaJ2, unsigned long requestedDurationUs) {
  if (!stepperTimersReady || motionSegmentActive) {
    Serial.println("ERROR: stepper timer engine unavailable or busy");
    return false;
  }
  const long countJ1 = labs(deltaJ1);
  const long countJ2 = labs(deltaJ2);
  const unsigned long minimumDurationUs =
      static_cast<unsigned long>(max(countJ1, countJ2)) * STEPPER_MIN_STEP_PERIOD_US;
  const unsigned long durationUs = max(requestedDurationUs, minimumDurationUs);

  if (deltaJ1 != 0) lastCommandDirectionJ1 = deltaJ1 > 0 ? 1 : -1;
  if (deltaJ2 != 0) lastCommandDirectionJ2 = deltaJ2 > 0 ? 1 : -1;
  setDirection(J1_DIR_PIN, deltaJ1, INVERT_J1);
  setDirection(J2_DIR_PIN, deltaJ2, INVERT_J2);
  delayMicroseconds(DIR_SETUP_US);

  motionSegmentDurationUs = durationUs;
  motionSegmentStartedUs = micros();
  motionSegmentActive = true;
  motorsMoving = true;
  if (!configureStepAxis(j1StepAxis, deltaJ1, durationUs) ||
      !configureStepAxis(j2StepAxis, deltaJ2, durationUs)) {
    stopMotionSegment();
    return false;
  }
  return true;
}

bool segmentInProgress() {
  if (!motionSegmentActive) return false;
  const bool j1Active = __atomic_load_n(&j1StepAxis.active, __ATOMIC_ACQUIRE);
  const bool j2Active = __atomic_load_n(&j2StepAxis.active, __ATOMIC_ACQUIRE);
  if (!j1Active && j1StepAxis.timerRunning) stopStepAxis(j1StepAxis);
  if (!j2Active && j2StepAxis.timerRunning) stopStepAxis(j2StepAxis);
  const bool durationPending =
      static_cast<unsigned long>(micros() - motionSegmentStartedUs) <
      motionSegmentDurationUs;
  if (j1Active || j2Active || durationPending) return true;
  motionSegmentActive = false;
  motorsMoving = false;
  return false;
}

long segmentEmittedStepEvents() {
  return max(
      __atomic_load_n(&j1StepAxis.emittedSteps, __ATOMIC_ACQUIRE),
      __atomic_load_n(&j2StepAxis.emittedSteps, __ATOMIC_ACQUIRE));
}

void stepperMotionSetup() {
  pinMode(J1_PUL_PIN, OUTPUT);
  pinMode(J1_DIR_PIN, OUTPUT);
  pinMode(J2_PUL_PIN, OUTPUT);
  pinMode(J2_DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(J1_PUL_PIN, STEP_IDLE);
  digitalWrite(J2_PUL_PIN, STEP_IDLE);
  digitalWrite(J1_DIR_PIN, LOW);
  digitalWrite(J2_DIR_PIN, LOW);
  digitalWrite(ENA_PIN, OUTPUTS_DISABLED);
  stepperTimersReady = initializeStepperTimers();
}

void enableDrivers() {
  digitalWrite(ENA_PIN, OUTPUTS_ENABLED);
  delay(2);
}

void disableDrivers() {
  if (motionSegmentActive) stopMotionSegment();
  digitalWrite(ENA_PIN, OUTPUTS_DISABLED);
  armed = false;
}

void feedbackFault(const char *message) {
  disableDrivers();
  if (bladeIsDown()) bladeRetracted();
  encoderFault = true;
  productReady = false;
  Serial.print("FEEDBACK FAULT: ");
  Serial.println(message);
}

bool checkFeedback() {
  if (!USE_ENCODERS || !encoderFeedbackEnabled) return true;
  float measuredJ1 = 0.0f;
  float measuredJ2 = 0.0f;
  if (!encoderJointAngles(measuredJ1, measuredJ2)) {
    feedbackFault("AS5600 read failed");
    return false;
  }
  const float errorJ1 = measuredJ1 - currentJ1Deg();
  const float errorJ2 = shortestJointDelta(measuredJ2, currentJ2Deg());
  if (MOTION_DIAGNOSTICS) {
    printMotionDiagnostic(
        "FEEDBACK", -1, currentJ1Deg(), currentJ2Deg(), measuredJ1, measuredJ2);
  }
  if ((armMode != AxisMode::J2_ONLY && fabsf(errorJ1) > FEEDBACK_MAX_ERROR_DEG) ||
      (armMode != AxisMode::J1_ONLY && fabsf(errorJ2) > FEEDBACK_MAX_ERROR_DEG)) {
    disableDrivers();
    encoderFault = true;
    Serial.print("FEEDBACK FAULT:");
    if (armMode != AxisMode::J2_ONLY) {
      Serial.print(" expected J1=");
      Serial.print(currentJ1Deg(), 2);
      Serial.print(" measured J1=");
      Serial.print(measuredJ1, 2);
      Serial.print(" error=");
      Serial.print(errorJ1, 2);
    }
    if (armMode != AxisMode::J1_ONLY) {
      Serial.print("; expected J2=");
      Serial.print(currentJ2Deg(), 2);
      Serial.print(" measured J2=");
      Serial.print(measuredJ2, 2);
      Serial.print(" error=");
      Serial.print(errorJ2, 2);
    }
    Serial.println();
    return false;
  }
  return true;
}

bool executeSteps(long deltaJ1, long deltaJ2) {
  const long countJ1 = labs(deltaJ1);
  const long countJ2 = labs(deltaJ2);
  const long total = max(countJ1, countJ2);
  if (total == 0) return true;
  const unsigned long durationUs =
      static_cast<unsigned long>(ceilf(total * 1000000.0f / DEFAULT_STEP_RATE_HZ));
  if (!startSegment(deltaJ1, deltaJ2, durationUs)) return false;

  long lastFeedbackSteps = 0;
  while (segmentInProgress()) {
    serviceControlInputs();
    serviceEncoderStream();
    if (productCutActive && productAbortRequested) {
      stopMotionSegment();
      Serial.println("CUT_ABORTED");
      return false;
    }
    const long emittedSteps = segmentEmittedStepEvents();
    if (emittedSteps - lastFeedbackSteps >= CONTINUOUS_FEEDBACK_STEP_INTERVAL) {
      lastFeedbackSteps = emittedSteps;
      if (!checkFeedback()) {
        stopMotionSegment();
        return false;
      }
    }
    delay(0);
  }
  stopMotionSegment();
  return checkFeedback();
}

bool moveToAngles(
    float j1Deg, float j2Deg, bool report, bool allowOutsideLimits) {
  if (!armed) {
    Serial.println("ERROR: type ARM FOLDED first");
    return false;
  }
  if (armMode == AxisMode::J1_ONLY &&
      fabsf(shortestJointDelta(j2Deg, currentJ2Deg())) > 0.001f) {
    Serial.println("ERROR: ARM J1 mode blocks J2 motion");
    return false;
  }
  if (armMode == AxisMode::J2_ONLY && fabsf(j1Deg - currentJ1Deg()) > 0.001f) {
    Serial.println("ERROR: ARM J2 mode blocks J1 motion");
    return false;
  }
  if (!allowOutsideLimits && !angleInRange(j1Deg, j2Deg)) {
    Serial.println("ERROR: angle outside software limits");
    return false;
  }

  for (int correction = 0; correction <= FEEDBACK_MAX_CORRECTIONS; ++correction) {
    long targetJ1 = lroundf(j1Deg * J1_STEPS_PER_DEG);
    const float equivalentJ2 = equivalentJointTargetNear(j2Deg, currentJ2Deg());
    long targetJ2 = lroundf(equivalentJ2 * J2_STEPS_PER_DEG);
    if (!executeSteps(
            targetJ1 - atomicReadSteps(j1PositionSteps),
            targetJ2 - atomicReadSteps(j2PositionSteps))) {
      return false;
    }
    if (!USE_ENCODERS || !encoderFeedbackEnabled) break;

    float measuredJ1 = 0.0f;
    float measuredJ2 = 0.0f;
    if (!encoderJointAngles(measuredJ1, measuredJ2)) {
      feedbackFault("AS5600 read failed");
      return false;
    }
    const float errorJ1 = j1Deg - measuredJ1;
    const float errorJ2 = shortestJointDelta(j2Deg, measuredJ2);
    if (MOTION_DIAGNOSTICS) {
      printMotionDiagnostic(
          "CORRECTION", correction, j1Deg, j2Deg, measuredJ1, measuredJ2);
    }
    const bool j1Settled =
        armMode == AxisMode::J2_ONLY || fabsf(errorJ1) <= FEEDBACK_TOLERANCE_DEG;
    const bool j2Settled =
        armMode == AxisMode::J1_ONLY || fabsf(errorJ2) <= FEEDBACK_TOLERANCE_DEG;
    if (j1Settled && j2Settled) break;
    if (correction == FEEDBACK_MAX_CORRECTIONS) {
      feedbackFault("target did not settle");
      return false;
    }
    if (armMode != AxisMode::J2_ONLY) {
      atomicWriteSteps(j1PositionSteps, lroundf(measuredJ1 * J1_STEPS_PER_DEG));
    }
    if (armMode != AxisMode::J1_ONLY) {
      atomicWriteSteps(j2PositionSteps, lroundf(measuredJ2 * J2_STEPS_PER_DEG));
    }
  }
  if (report) {
    Serial.println("OK");
    printOperationReport("MOVE_COMPLETE");
  }
  return true;
}
