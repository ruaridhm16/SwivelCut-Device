#include "Diagnostics.h"

#include <Arduino.h>

#include "Encoders.h"
#include "Kinematics.h"
#include "StepperMotion.h"

void printMotionDiagnostic(
    const char *stage, int correction, float targetJ1, float targetJ2,
    float measuredJ1, float measuredJ2) {
  float targetX = 0.0f;
  float targetY = 0.0f;
  float measuredX = 0.0f;
  float measuredY = 0.0f;
  forwardKinematics(targetJ1, targetJ2, targetX, targetY);
  forwardKinematics(measuredJ1, measuredJ2, measuredX, measuredY);
  Serial.print("MOTION_DIAG stage=");
  Serial.print(stage);
  if (correction >= 0) {
    Serial.print(" correction=");
    Serial.print(correction);
  }
  Serial.print(" dir_J1=");
  Serial.print(lastCommandDirectionJ1);
  Serial.print(" dir_J2=");
  Serial.print(lastCommandDirectionJ2);
  Serial.print(" target_J1=");
  Serial.print(targetJ1, 3);
  Serial.print(" target_J2=");
  Serial.print(targetJ2, 3);
  Serial.print(" measured_J1=");
  Serial.print(measuredJ1, 3);
  Serial.print(" measured_J2=");
  Serial.print(measuredJ2, 3);
  Serial.print(" error_J1=");
  Serial.print(targetJ1 - measuredJ1, 3);
  Serial.print(" error_J2=");
  Serial.print(shortestJointDelta(targetJ2, measuredJ2), 3);
  Serial.print(" target_X=");
  Serial.print(targetX, 2);
  Serial.print(" target_Y=");
  Serial.print(targetY, 2);
  Serial.print(" measured_X=");
  Serial.print(measuredX, 2);
  Serial.print(" measured_Y=");
  Serial.print(measuredY, 2);
  Serial.print(" error_X=");
  Serial.print(targetX - measuredX, 2);
  Serial.print(" error_Y=");
  Serial.println(targetY - measuredY, 2);
}

void printOperationReport(const char *label) {
  const float softwareJ1 = currentJ1Deg();
  const float softwareJ2 = currentJ2Deg();
  float x = 0.0f;
  float y = 0.0f;
  forwardKinematics(softwareJ1, softwareJ2, x, y);
  Serial.print("REPORT ");
  Serial.print(label);
  Serial.print(" SW_J1=");
  Serial.print(softwareJ1, 2);
  Serial.print(" SW_J2=");
  Serial.print(softwareJ2, 2);
  Serial.print(" X=");
  Serial.print(x, 1);
  Serial.print(" Y=");
  Serial.print(y, 1);

  if (encodersCalibrated) {
    float encoderJ1 = 0.0f;
    float encoderJ2 = 0.0f;
    int16_t rawJ1 = 0;
    int16_t rawJ2 = 0;
    const bool anglesOk = encoderJointAngles(encoderJ1, encoderJ2);
    const bool raw1Ok = encoderMode == AxisMode::J2_ONLY || j1Encoder.rawValue(rawJ1);
    const bool raw2Ok = encoderMode == AxisMode::J1_ONLY || j2Encoder.rawValue(rawJ2);
    if (anglesOk) {
      Serial.print(" ENC_J1=");
      Serial.print(encoderJ1, 2);
      Serial.print(" ENC_J2=");
      Serial.print(encoderJ2, 2);
    } else {
      Serial.print(" ENC_ANGLES=READ_ERROR");
    }
    if (raw1Ok && encoderMode != AxisMode::J2_ONLY) {
      Serial.print(" RAW1=");
      Serial.print(rawJ1);
    }
    if (raw2Ok && encoderMode != AxisMode::J1_ONLY) {
      Serial.print(" RAW2=");
      Serial.print(rawJ2);
    }
    if (!raw1Ok || !raw2Ok) Serial.print(" RAW=READ_ERROR");
  } else {
    Serial.print(" ENC=UNCALIBRATED");
  }
  Serial.println();
}

void printPosition() { printOperationReport("POSITION"); }
