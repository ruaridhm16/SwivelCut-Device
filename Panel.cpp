#include "Panel.h"

#include <Arduino.h>
#include <FastLED.h>

#include "Blade.h"
#include "Config.h"
#include "Diagnostics.h"
#include "Encoders.h"
#include "StepperMotion.h"
#include "TeachReplay.h"

namespace {

CRGB buttonLedPixels[BUTTON_LED_COUNT];

ButtonInput buttons[] = {
    {START_STOP_BUTTON_PIN, 1, "START_STOP", HIGH, HIGH, 0},
    {STABILIZATION_BUTTON_PIN, 2, "STABILIZATION", HIGH, HIGH, 0},
    {REPEAT_BUTTON_PIN, 3, "REPEAT", HIGH, HIGH, 0},
    {RELAY_BUTTON_PIN, 4, "RELAY", HIGH, HIGH, 0},
};
constexpr size_t BUTTON_COUNT = sizeof(buttons) / sizeof(buttons[0]);
static_assert(BUTTON_LED_COUNT == BUTTON_COUNT,
              "The WS2812 strip needs one pixel per button");

ButtonLed buttonLeds[] = {
    {1, 1, "START_STOP", LedColor::OFF},
    {2, 2, "STABILIZATION", LedColor::OFF},
    {3, 3, "REPEAT", LedColor::OFF},
    {0, 4, "RELAY", LedColor::OFF},
};

HeadType stableHeadType = HeadType::UNKNOWN;
HeadType candidateHeadType = HeadType::UNKNOWN;
int candidateHeadSamples = 0;
int latestHeadAdc = 0;
bool headTypeInitialized = false;
unsigned long nextHeadSampleMs = 0;

bool controlTestEnabled = false;
bool stateTestEnabled = false;
bool testTeachingActive = false;
bool testStabilizationEnabled = false;
bool testHasLastCut = false;
bool controlTestButtonOn[BUTTON_COUNT] = {};
HeadType testActiveHead = HeadType::UNKNOWN;

const char *ledColorName(LedColor color) {
  switch (color) {
    case LedColor::RED: return "RED";
    case LedColor::GREEN: return "GREEN";
    case LedColor::WHITE: return "WHITE";
    default: return "OFF";
  }
}

const char *headTypeName(HeadType type) {
  switch (type) {
    case HeadType::CUTTING: return "CUTTER";
    case HeadType::TRACING: return "TRACER";
    case HeadType::DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

HeadType classifyHeadAdc(int adc) {
  if (adc >= TRACING_HEAD_ADC_MIN && adc <= TRACING_HEAD_ADC_MAX) {
    return HeadType::TRACING;
  }
  if (ASSUME_CUTTER_UNLESS_TRACER) return HeadType::CUTTING;
  if (adc >= HEAD_DISCONNECTED_ADC_MIN) return HeadType::DISCONNECTED;
  if (adc >= CUTTING_HEAD_ADC_MIN && adc <= CUTTING_HEAD_ADC_MAX) {
    return HeadType::CUTTING;
  }
  return HeadType::UNKNOWN;
}

void writeButtonLed(ButtonLed &led, LedColor color) {
  buttonLedPixels[led.pixelIndex] =
      color == LedColor::RED
          ? CRGB::Red
          : (color == LedColor::GREEN
                 ? CRGB::Green
                 : (color == LedColor::WHITE ? CRGB::White : CRGB::Black));
  FastLED.show();
  led.color = color;
}

void setRelayConnected(bool connected, bool report = true) {
  relayConnected = connected;
  if (controlTestEnabled) {
    controlTestButtonOn[3] = connected;
  }
  digitalWrite(RELAY_PIN, connected ? RELAY_CONNECTED_LEVEL : RELAY_DISCONNECTED_LEVEL);
  if (report) {
    Serial.print("RELAY ");
    Serial.print(connected ? "CONNECTED" : "DISCONNECTED");
    Serial.print(" GPIO");
    Serial.print(RELAY_PIN);
    Serial.print("=");
    Serial.println(connected ? "HIGH" : "LOW");
  }
}

void printButtonEvent(const ButtonInput &button) {
  Serial.print("PIN ");
  Serial.print(button.number);
  Serial.print(button.stableState == LOW ? " PRESSED" : " RELEASED");
  Serial.print(" - ");
  Serial.print(button.name);
  Serial.print(" (GPIO");
  Serial.print(button.pin);
  Serial.println(")");
  if (button.stableState != LOW) return;

  const size_t index = static_cast<size_t>(button.number - 1);
  controlTestButtonOn[index] = !controlTestButtonOn[index];
  if (button.number == 4) {
    setRelayConnected(controlTestButtonOn[index]);
  }
  refreshButtonLeds();
  Serial.print("LED ");
  Serial.print(button.number);
  Serial.print(" ");
  Serial.println(ledColorName(buttonLeds[index].color));

  Serial.print("TEST ONLY - ");
  if (button.number == 1) {
    testTeachingActive = !testTeachingActive;
    Serial.println(
        testTeachingActive ? "STARTING TEACHING REQUESTED; NO FUNCTION STARTED"
                            : "STOPPING TEACHING REQUESTED; NO FUNCTION STARTED");
  } else if (button.number == 2) {
    testStabilizationEnabled = !testStabilizationEnabled;
    Serial.println(
        testStabilizationEnabled ? "STABILIZATION ON REQUESTED; NO FUNCTION STARTED"
                                  : "STABILIZATION OFF REQUESTED; NO FUNCTION STARTED");
  } else if (button.number == 3) {
    Serial.println("REPEAT REQUESTED; NO FUNCTION STARTED");
  } else if (button.number == 4) {
    Serial.println(relayConnected ? "RELAY CONNECTED" : "RELAY DISCONNECTED");
  }
}

void printStateTestEvent(const ButtonInput &button) {
  if (button.stableState != LOW) return;

  if (button.number == 1) {
    if (testTeachingActive) {
      Serial.println(
          testActiveHead == HeadType::TRACING ? "TRACING_STOPPED" : "CUTTING_STOPPED");
      if (testActiveHead == HeadType::CUTTING) testHasLastCut = true;
      testTeachingActive = false;
      testActiveHead = HeadType::UNKNOWN;
      return;
    }

    if (!headTypeInitialized ||
        (stableHeadType != HeadType::CUTTING && stableHeadType != HeadType::TRACING)) {
      Serial.println("NOT_DOING_ANYTHING");
      return;
    }

    testTeachingActive = true;
    testActiveHead = stableHeadType;
    if (testActiveHead == HeadType::TRACING) {
      Serial.println("TRACING_STARTED");
    } else if (testStabilizationEnabled) {
      Serial.println("CUTTING_STARTED_WITH_STABILIZATION");
    } else {
      Serial.println("CUTTING_STARTED");
    }
  } else if (button.number == 2) {
    if (testTeachingActive) {
      Serial.println("STABILIZATION_CHANGE_IGNORED_ACTIVE_EVENT");
      return;
    }
    testStabilizationEnabled = !testStabilizationEnabled;
    Serial.println(testStabilizationEnabled ? "STABILIZATION_ON" : "STABILIZATION_OFF");
  } else if (button.number == 3) {
    if (testTeachingActive || !testHasLastCut) {
      Serial.println("NOT_DOING_ANYTHING");
    } else if (testStabilizationEnabled) {
      Serial.println("REPEATING_LAST_CUT_EVENT_WITH_STABILIZATION");
    } else {
      Serial.println("REPEATING_LAST_CUT_EVENT");
    }
  } else if (button.number == 4) {
    setRelayConnected(!relayConnected);
  }
  refreshButtonLeds();
}

}  // namespace

bool relayConnected = false;

void panelSetup() {
  pinMode(START_STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STABILIZATION_BUTTON_PIN, INPUT);  // External pull-up on this ADC-only pin.
  pinMode(REPEAT_BUTTON_PIN, INPUT);         // External pull-up on this ADC-only pin.
  pinMode(RELAY_BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, RELAY_DISCONNECTED_LEVEL);
  pinMode(RELAY_PIN, OUTPUT);

  FastLED.addLeds<WS2812, BUTTON_LED_DATA_PIN, GRB>(buttonLedPixels, BUTTON_LED_COUNT);
  FastLED.setBrightness(BUTTON_LED_BRIGHTNESS);
  fill_solid(buttonLedPixels, BUTTON_LED_COUNT, CRGB::Black);
  FastLED.show();

  pinMode(HEAD_ID_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(HEAD_ID_PIN, ADC_11db);

  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    buttons[i].rawState = digitalRead(buttons[i].pin);
    buttons[i].stableState = buttons[i].rawState;
    buttons[i].rawChangedMs = now;
  }
}

void refreshButtonLeds() {
  if (!relayConnected) {
    bool changed = false;
    for (size_t i = 0; i < BUTTON_COUNT; ++i) {
      if (buttonLeds[i].color != LedColor::OFF) {
        buttonLedPixels[buttonLeds[i].pixelIndex] = CRGB::Black;
        buttonLeds[i].color = LedColor::OFF;
        changed = true;
      }
    }
    if (changed) FastLED.show();
    return;
  }

  bool on[BUTTON_COUNT] = {};
  if (controlTestEnabled) {
    for (size_t i = 0; i < BUTTON_COUNT; ++i) on[i] = controlTestButtonOn[i];
  } else if (stateTestEnabled) {
    on[0] = testTeachingActive;
    on[1] = testStabilizationEnabled;
    on[2] = testHasLastCut;
    on[3] = relayConnected;
  } else {
    on[0] = productState != ProductState::IDLE;
    on[1] = stabilizationEnabled;
    on[2] = repeatCutActive;
    on[3] = relayConnected;
  }

  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    const LedColor requested =
        on[i] ? (i == 3 ? LedColor::WHITE : LedColor::GREEN) : LedColor::OFF;
    if (buttonLeds[i].color != requested) {
      writeButtonLed(buttonLeds[i], requested);
    }
  }
}

void setMachinePower(bool enabled) {
  if (!enabled) {
    disableDrivers();
    armMode = AxisMode::DUAL;
    productReady = false;
    productState = ProductState::IDLE;
    productAbortRequested = false;
    repeatCutActive = false;
    tracerReplayActive = false;
    if (bladeIsDown()) bladeRetracted();
    setRelayConnected(false);
    refreshButtonLeds();
    Serial.println("MACHINE_OFF ARMS_DISABLED");
    return;
  }

  Serial.println(
      "MACHINE_ON: current arm pose must be physically folded; "
      "calibrating home with motor drivers disabled");
  setRelayConnected(true);
  delay(100);
  armAtFoldedPose(AxisMode::DUAL, false);
  if (!encodersCalibrated || !productReady) {
    disableDrivers();
    setRelayConnected(false);
    refreshButtonLeds();
    Serial.println("MACHINE_ON_FAILED RELAY_OFF ARMS_DISABLED");
    return;
  }
  refreshButtonLeds();
  Serial.println("MACHINE_ON ARMS_HOMED_AND_DISARMED");
}

void serviceControlInputs() {
  const unsigned long now = millis();
  refreshButtonLeds();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    ButtonInput &button = buttons[i];
    const int raw = digitalRead(button.pin);
    if (raw != button.rawState) {
      button.rawState = raw;
      button.rawChangedMs = now;
    }
    if (raw != button.stableState && now - button.rawChangedMs >= BUTTON_DEBOUNCE_MS) {
      button.stableState = raw;
      if (controlTestEnabled) printButtonEvent(button);
      if (stateTestEnabled) printStateTestEvent(button);
      if (!controlTestEnabled && !stateTestEnabled) {
        if (button.number == 4 ||
            (productState != ProductState::CUTTING && !motorsMoving)) {
          handleProductButtonChange(button);
        }
      }
      refreshButtonLeds();
    }
  }

  if (static_cast<long>(now - nextHeadSampleMs) < 0) return;
  nextHeadSampleMs = now + HEAD_SAMPLE_INTERVAL_MS;
  latestHeadAdc = analogRead(HEAD_ID_PIN);
  const HeadType measuredHead = classifyHeadAdc(latestHeadAdc);
  if (measuredHead == candidateHeadType) {
    if (candidateHeadSamples < HEAD_STABLE_SAMPLE_COUNT) ++candidateHeadSamples;
  } else {
    candidateHeadType = measuredHead;
    candidateHeadSamples = 1;
  }

  if (candidateHeadSamples >= HEAD_STABLE_SAMPLE_COUNT &&
      (!headTypeInitialized || candidateHeadType != stableHeadType)) {
    stableHeadType = candidateHeadType;
    headTypeInitialized = true;
    Serial.print("HEAD ");
    Serial.print(headTypeName(stableHeadType));
    Serial.print(" ADC=");
    Serial.println(latestHeadAdc);
    if (!controlTestEnabled && !stateTestEnabled) {
      if (productState == ProductState::TEACHING && stableHeadType != HeadType::TRACING) {
        taughtCount = 0;
        productState = ProductState::IDLE;
        Serial.println("TEACHING_STOPPED_HEAD_CHANGED PATH_DISCARDED");
        printOperationReport("TRACING_STOPPED_HEAD_CHANGED");
      }
      if (productState == ProductState::CUTTING &&
          stableHeadType != (tracerReplayActive ? HeadType::TRACING : HeadType::CUTTING)) {
        productAbortRequested = true;
        Serial.println(
            tracerReplayActive ? "TRACER_REPLAY_ABORT_REQUESTED_HEAD_CHANGED"
                                : "CUT_ABORT_REQUESTED_HEAD_CHANGED");
      }
    }
  }
}

void setControlTest(bool enabled) {
  controlTestEnabled = enabled;
  stateTestEnabled = false;
  if (!enabled) {
    refreshButtonLeds();
    Serial.println("CONTROL TEST OFF");
    return;
  }

  disableDrivers();
  stopBlade();
  encoderStreamEnabled = false;
  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    buttons[i].rawState = digitalRead(buttons[i].pin);
    buttons[i].stableState = buttons[i].rawState;
    buttons[i].rawChangedMs = now;
  }
  candidateHeadSamples = 0;
  headTypeInitialized = false;
  testTeachingActive = false;
  testStabilizationEnabled = false;
  for (size_t i = 0; i < BUTTON_COUNT; ++i) controlTestButtonOn[i] = false;
  controlTestButtonOn[3] = relayConnected;
  nextHeadSampleMs = 0;
  Serial.println(
      "CONTROL TEST ON: motors and blade outputs disabled; "
      "printing only when a button, LED, relay, or head state changes");
  refreshButtonLeds();
  printControlStatus();
}

void setStateTest(bool enabled) {
  stateTestEnabled = enabled;
  controlTestEnabled = false;
  if (!enabled) {
    refreshButtonLeds();
    Serial.println("STATE_TEST_OFF");
    return;
  }

  disableDrivers();
  stopBlade();
  encoderStreamEnabled = false;
  const unsigned long now = millis();
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    buttons[i].rawState = digitalRead(buttons[i].pin);
    buttons[i].stableState = buttons[i].rawState;
    buttons[i].rawChangedMs = now;
  }
  candidateHeadSamples = 0;
  headTypeInitialized = false;
  testTeachingActive = false;
  testStabilizationEnabled = false;
  testHasLastCut = false;
  testActiveHead = HeadType::UNKNOWN;
  nextHeadSampleMs = 0;
  refreshButtonLeds();
  Serial.println("NOT_DOING_ANYTHING");
}

bool anyPanelTestActive() { return controlTestEnabled || stateTestEnabled; }

void printControlStatus() {
  Serial.print("CONTROLS");
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    Serial.print(" B");
    Serial.print(buttons[i].number);
    Serial.print("_");
    Serial.print(buttons[i].name);
    Serial.print("=");
    Serial.print(buttons[i].stableState == LOW ? "PRESSED" : "RELEASED");
    Serial.print("(GPIO");
    Serial.print(buttons[i].pin);
    Serial.print("=");
    Serial.print(buttons[i].stableState == HIGH ? "HIGH" : "LOW");
    Serial.print(")");
  }
  Serial.print(" RELAY=");
  Serial.print(relayConnected ? "CONNECTED" : "DISCONNECTED");
  Serial.print("(GPIO");
  Serial.print(RELAY_PIN);
  Serial.print("=");
  Serial.print(digitalRead(RELAY_PIN) == HIGH ? "HIGH" : "LOW");
  Serial.print(")");
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    Serial.print(" LED");
    Serial.print(buttonLeds[i].number);
    Serial.print("=");
    Serial.print(ledColorName(buttonLeds[i].color));
    Serial.print("(PIXEL");
    Serial.print(buttonLeds[i].pixelIndex);
    Serial.print(")");
  }
  latestHeadAdc = analogRead(HEAD_ID_PIN);
  const HeadType measuredHead = classifyHeadAdc(latestHeadAdc);
  Serial.print(" HEAD=");
  Serial.print(headTypeName(headTypeInitialized ? stableHeadType : measuredHead));
  Serial.print(" ADC=");
  Serial.println(latestHeadAdc);
}

void printLedStatus() {
  Serial.print("LEDS");
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    Serial.print(" LED");
    Serial.print(buttonLeds[i].number);
    Serial.print("_");
    Serial.print(buttonLeds[i].name);
    Serial.print("=");
    Serial.print(ledColorName(buttonLeds[i].color));
    Serial.print(" PIXEL=");
    Serial.print(buttonLeds[i].pixelIndex);
  }
  Serial.print(" DATA_GPIO=");
  Serial.print(BUTTON_LED_DATA_PIN);
  Serial.println();
}

void printPanelPinMap() {
  Serial.println("PANEL PIN MAP (edit constants near the top to rewire):");
  for (size_t i = 0; i < BUTTON_COUNT; ++i) {
    Serial.print("  B");
    Serial.print(buttons[i].number);
    Serial.print(" ");
    Serial.print(buttons[i].name);
    Serial.print(" button=GPIO");
    Serial.print(buttons[i].pin);
    Serial.print(" pressed=LOW input=");
    Serial.print(
        buttons[i].pin == 36 || buttons[i].pin == 39 ? "EXTERNAL_PULLUP"
                                                       : "INPUT_PULLUP");
    Serial.print(" led=WS2812_PIXEL");
    Serial.println(buttonLeds[i].pixelIndex);
  }
  Serial.print("  WS2812 data=GPIO");
  Serial.print(BUTTON_LED_DATA_PIN);
  Serial.println(" count=4 order=GRB brightness=64");
  Serial.print("  Relay=GPIO");
  Serial.print(RELAY_PIN);
  Serial.println(" connected=HIGH disconnected=LOW");
}

bool headTypeKnown() { return headTypeInitialized; }
HeadType currentHeadType() { return stableHeadType; }
