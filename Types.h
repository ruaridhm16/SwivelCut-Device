// Shared enums and plain-data structs used across the firmware modules.
#pragma once

#include <Arduino.h>

struct TeachPoint {
  float seconds;
  float j1Deg;
  float j2Deg;
};

struct ButtonInput {
  int pin;
  int number;
  const char *name;
  int rawState;
  int stableState;
  unsigned long rawChangedMs;
};

enum class LedColor {
  OFF,
  RED,
  GREEN,
  WHITE,
};

struct ButtonLed {
  int pixelIndex;
  int number;
  const char *name;
  LedColor color;
};

enum class HeadType {
  UNKNOWN,
  CUTTING,
  TRACING,
  DISCONNECTED,
};

enum class ProductState {
  IDLE,
  TEACHING,
  CUTTING,
};

enum class BladePosition {
  RETRACTED,
  DOWN,
};

enum class AxisMode {
  DUAL,
  J1_ONLY,
  J2_ONLY,
};
