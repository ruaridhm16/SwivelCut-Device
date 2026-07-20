// SwivelCut firmware for an ESP32 and two TB6600 stepper drivers.
//
// The two-link arm is driven by a hardware-timer step generator so motion
// runs independently of the main loop (see StepperMotion), with AS5600
// magnetic encoders closing the loop on commanded position (see Encoders).
// A serial command line and a four-button operator panel share the same
// underlying teach/replay workflow (see CommandLine, Panel, TeachReplay).
//
// See Config.h for pin assignments and tuning constants.
#include "Blade.h"
#include "CommandLine.h"
#include "Config.h"
#include "Encoders.h"
#include "Panel.h"
#include "StepperMotion.h"
#include "TeachReplay.h"

void setup() {
  stepperMotionSetup();
  bladeSetup();
  panelSetup();

  Serial.begin(115200);
  Serial.setTimeout(50);
  if (USE_ENCODERS) encodersSetup();

  Serial.println();
  Serial.println("SwivelCut Arduino controller ready");
  if (!bladePwmReady) {
    Serial.println("ERROR: blade PWM channel failed to initialize");
  }
  Serial.println("Fold the arm, then type ARM FOLDED");
  Serial.println("Product buttons become active after ARM FOLDED.");
  Serial.println("Type CONTROL TEST ON to test buttons, LEDs, relay, and head ID.");
  if (ALLOW_TAUGHT_PATH_OUTSIDE_SOFTWARE_LIMITS) {
    Serial.println("WARNING: encoder-taught paths may replay outside software limits");
  }
  if (MOTION_DIAGNOSTICS) {
    Serial.println("WARNING: MOTION_DIAGNOSTICS active; serial output may affect timing");
  }
  if (ASSUME_CUTTER_UNLESS_TRACER) {
    Serial.println("WARNING: head override active; every non-tracer reading is CUTTER");
  }
  printPanelPinMap();
  refreshButtonLeds();
  printHelp();
}

void loop() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      if (inputLine.length() > 0) {
        commandTerminator = incoming;
        handleCommand(inputLine);
        inputLine = "";
      }
    } else if (inputLine.length() < 120) {
      inputLine += incoming;
    }
  }
  serviceControlInputs();
  serviceProductWorkflow();
  serviceEncoderStream();
}
