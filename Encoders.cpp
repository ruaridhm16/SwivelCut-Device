#include "Encoders.h"

#include <Arduino.h>

#include "Config.h"
#include "Kinematics.h"
#include "StepperMotion.h"

namespace {
TwoWire j1Wire(0);
TwoWire j2Wire(1);
}  // namespace

AS5600Tracker j1Encoder(j1Wire, ENCODER_J1_SIGN);
AS5600Tracker j2Encoder(j2Wire, ENCODER_J2_SIGN);

bool encoderFault = false;
bool encodersCalibrated = false;
AxisMode encoderMode = AxisMode::DUAL;
bool encoderFeedbackEnabled = true;
bool encoderStreamEnabled = false;
float encoderStreamHz = 10.0f;
unsigned long nextEncoderStreamMs = 0;

bool AS5600Tracker::begin(int sda, int scl) {
  wire_.begin(sda, scl, AS5600_I2C_HZ);
  return probe();
}

bool AS5600Tracker::probe() {
  wire_.beginTransmission(AS5600_ADDRESS);
  return wire_.endTransmission() == 0;
}

const char *AS5600Tracker::magnetState(bool &found) {
  uint8_t status = 0;
  if (!readRegister(0x0B, &status, 1)) {
    found = false;
    return "UNREADABLE";
  }
  found = true;
  if (status & 0x08) return "TOO STRONG";
  if (status & 0x10) return "TOO WEAK";
  if (status & 0x20) return "OK";
  return "MISSING";
}

bool AS5600Tracker::magnetOk() {
  bool found = false;
  return strcmp(magnetState(found), "OK") == 0;
}

bool AS5600Tracker::calibrate() {
  uint16_t raw = 0;
  if (!readRaw(raw) || !magnetOk()) {
    return false;
  }
  lastRaw_ = raw;
  positionCounts_ = 0;
  calibrated_ = true;
  return true;
}

bool AS5600Tracker::angleDegrees(float &degreesOut) {
  if (!calibrated_) {
    return false;
  }
  int deltas[5] = {};
  for (int i = 0; i < 5; ++i) {
    uint16_t raw = 0;
    if (!readRaw(raw)) return false;
    int delta = static_cast<int>(raw) - static_cast<int>(lastRaw_);
    if (delta > 2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    deltas[i] = delta;
  }
  // Insertion-sort five samples in place and take the median; cheaper than
  // pulling in <algorithm> for a fixed five-element window.
  for (int i = 1; i < 5; ++i) {
    const int value = deltas[i];
    int j = i - 1;
    while (j >= 0 && deltas[j] > value) {
      deltas[j + 1] = deltas[j];
      --j;
    }
    deltas[j + 1] = value;
  }
  const int medianDelta = deltas[2];
  positionCounts_ += medianDelta;
  lastRaw_ = static_cast<uint16_t>(
      (static_cast<int>(lastRaw_) + medianDelta + 4096) % 4096);
  degreesOut = sign_ * positionCounts_ * (360.0f / 4096.0f);
  return true;
}

bool AS5600Tracker::rawValue(int16_t &raw) {
  uint16_t unsignedRaw = 0;
  if (!readRaw(unsignedRaw)) return false;
  raw = unsignedRaw > 2047 ? static_cast<int16_t>(unsignedRaw) - 4096
                           : static_cast<int16_t>(unsignedRaw);
  return true;
}

bool AS5600Tracker::readRaw(uint16_t &raw) {
  uint8_t data[2] = {};
  if (!readRegister(0x0C, data, 2)) {
    return false;
  }
  raw = (static_cast<uint16_t>(data[0]) << 8 | data[1]) & 0x0FFF;
  return true;
}

bool AS5600Tracker::readRegister(uint8_t reg, uint8_t *data, size_t length) {
  for (int attempt = 0; attempt < AS5600_READ_ATTEMPTS; ++attempt) {
    wire_.beginTransmission(AS5600_ADDRESS);
    wire_.write(reg);
    if (wire_.endTransmission(false) == 0 &&
        wire_.requestFrom(AS5600_ADDRESS, length) == length) {
      for (size_t i = 0; i < length; ++i) {
        data[i] = wire_.read();
      }
      return true;
    }
    while (wire_.available()) wire_.read();
    delayMicroseconds(AS5600_RETRY_DELAY_US);
  }
  return false;
}

void encodersSetup() {
  const bool j1Found = j1Encoder.begin(J1_SDA_PIN, J1_SCL_PIN);
  const bool j2Found = j2Encoder.begin(J2_SDA_PIN, J2_SCL_PIN);
  if (!j1Found || !j2Found) {
    Serial.println("WARNING: one or both AS5600 encoders were not found");
  }
}

bool encoderJointAngles(float &j1Deg, float &j2Deg) {
  if (encoderMode == AxisMode::J2_ONLY) {
    j1Deg = currentJ1Deg();
  } else {
    float motor1Deg = 0.0f;
    if (!j1Encoder.angleDegrees(motor1Deg)) return false;
    j1Deg = motor1Deg / J1_GEAR_RATIO;
  }
  if (encoderMode == AxisMode::J1_ONLY) {
    j2Deg = currentJ2Deg();
    return true;
  }
  float motor2Deg = 0.0f;
  if (!j2Encoder.angleDegrees(motor2Deg)) return false;
  // The folded pose is both +180 and -180. If the manually moved arm opens
  // through the negative branch, the continuous encoder can read above +180
  // (for example 196 degrees). Canonicalize that to the equivalent -164
  // degrees so valid traces are not rejected at the folded boundary.
  j2Deg = normalizeJointDegrees(180.0f + motor2Deg / J2_GEAR_RATIO);
  return true;
}

void serviceEncoderStream() {
  if (!encoderStreamEnabled || !encodersCalibrated) return;
  const unsigned long now = millis();
  if (static_cast<long>(now - nextEncoderStreamMs) < 0) return;
  nextEncoderStreamMs = now + static_cast<unsigned long>(1000.0f / encoderStreamHz);

  float j1Deg = 0.0f;
  float j2Deg = 0.0f;
  if (!encoderJointAngles(j1Deg, j2Deg)) {
    encoderStreamEnabled = false;
    Serial.println("ENC_STREAM ERROR: read failed; stream stopped");
    return;
  }
  int16_t j1Raw = 0;
  int16_t j2Raw = 0;
  if ((encoderMode != AxisMode::J2_ONLY && !j1Encoder.rawValue(j1Raw)) ||
      (encoderMode != AxisMode::J1_ONLY && !j2Encoder.rawValue(j2Raw))) {
    encoderStreamEnabled = false;
    Serial.println("ENC_STREAM ERROR: raw read failed; stream stopped");
    return;
  }
  Serial.print("ENC_STREAM");
  if (encoderMode != AxisMode::J2_ONLY) {
    Serial.print(" J1=");
    Serial.print(j1Deg, 2);
    Serial.print(" RAW1=");
    Serial.print(j1Raw);
  }
  if (encoderMode != AxisMode::J1_ONLY) {
    Serial.print(" J2=");
    Serial.print(j2Deg, 2);
    Serial.print(" RAW2=");
    Serial.print(j2Raw);
  }
  Serial.println();
}
