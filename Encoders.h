// AS5600 magnetic joint encoders: raw I2C driver plus the joint-angle
// feedback used to correct commanded motor position.
#pragma once

#include <Wire.h>

#include "Types.h"

// Tracks one AS5600's continuous (unwrapped) angle by accumulating the
// median of several consecutive raw-count deltas each sample, which rejects
// single-sample I2C glitches without adding a lag-inducing filter.
class AS5600Tracker {
 public:
  AS5600Tracker(TwoWire &wire, int sign)
      : wire_(wire), sign_(sign), calibrated_(false), lastRaw_(0),
        positionCounts_(0) {}

  bool begin(int sda, int scl);
  bool probe();
  const char *magnetState(bool &found);
  bool magnetOk();
  bool calibrate();
  bool angleDegrees(float &degreesOut);
  bool rawValue(int16_t &raw);

 private:
  bool readRaw(uint16_t &raw);
  bool readRegister(uint8_t reg, uint8_t *data, size_t length);

  TwoWire &wire_;
  int sign_;
  bool calibrated_;
  uint16_t lastRaw_;
  long positionCounts_;
};

extern AS5600Tracker j1Encoder;
extern AS5600Tracker j2Encoder;

extern bool encoderFault;
extern bool encodersCalibrated;
extern AxisMode encoderMode;
extern bool encoderFeedbackEnabled;
extern bool encoderStreamEnabled;
extern float encoderStreamHz;
extern unsigned long nextEncoderStreamMs;

// Initializes both encoder I2C buses. Prints a warning (does not halt) if
// either AS5600 fails to ACK, since the machine may still be usable in a
// single-axis ARM mode.
void encodersSetup();

// Reads both joints' angles, respecting encoderMode for single-axis arming.
bool encoderJointAngles(float &j1Deg, float &j2Deg);

// Streams live encoder angles over Serial at encoderStreamHz when enabled.
void serviceEncoderStream();
