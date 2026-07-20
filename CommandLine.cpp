#include "CommandLine.h"

#include <Arduino.h>

#include "Blade.h"
#include "Config.h"
#include "Diagnostics.h"
#include "Encoders.h"
#include "Panel.h"
#include "StepperMotion.h"
#include "TeachReplay.h"

String inputLine;
char commandTerminator = '\0';

namespace {

bool parseElbow(const char *text) {
  return text != nullptr && strcmp(text, "DOWN") == 0;
}

void printEncoderStatus() {
  if (!USE_ENCODERS) {
    Serial.println("ENC unavailable on this branch");
    return;
  }
  bool j1Found = false;
  bool j2Found = false;
  const char *j1Magnet = j1Encoder.magnetState(j1Found);
  const char *j2Magnet = j2Encoder.magnetState(j2Found);
  Serial.print("ENC J1=[found=");
  Serial.print(j1Found ? "YES" : "NO");
  Serial.print(" magnet=");
  Serial.print(j1Magnet);
  Serial.print("] J2=[found=");
  Serial.print(j2Found ? "YES" : "NO");
  Serial.print(" magnet=");
  Serial.print(j2Magnet);
  Serial.print("]");
  if (encodersCalibrated) {
    float j1 = 0.0f;
    float j2 = 0.0f;
    if (encoderJointAngles(j1, j2)) {
      Serial.print(" J1=");
      Serial.print(j1, 2);
      Serial.print(" J2=");
      Serial.print(j2, 2);
    } else {
      Serial.print(" angles=READ_ERROR");
    }
  } else {
    Serial.print(" angles=UNCALIBRATED");
  }
  Serial.println();
}

}  // namespace

void printHelp() {
  Serial.println("Physical product controls after ARM FOLDED:");
  Serial.println("  Start/Stop + tracer: press to start/stop recording");
  Serial.println("  Start/Stop + cutter: press once to run the full cut");
  Serial.println("  Stabilization: toggle while idle");
  Serial.println(
      "  Repeat + cutter: repeat last cut; + tracer: redraw traced path");
  Serial.println(
      "  On/Off (button 4): relay ON homes/enables folded arms; OFF disables");
  Serial.println(
      "  WS2812 LEDs (GPIO4): green=on, power button=white; one pixel per button");
  Serial.println("  Product buttons are ignored while motors are moving");
  Serial.println("Commands:");
  Serial.println("  ARM FOLDED | ARM J1 | ARM J2 | DISARM");
  Serial.println("  TEST J1 <steps> | TEST J2 <steps>");
  Serial.println("  J1 <deg> | J2 <deg> | ANGLES <j1> <j2>");
  Serial.println("  XY <x> <y> [UP|DOWN] (omitted = AUTO by X side)");
  Serial.println("  CUT <x0> <y0> <x1> <y1> [UP|DOWN] (omitted = UP)");
  Serial.println("  LOAD POINTS <N> (then N lines: <j1Deg> <j2Deg>)");
  Serial.println("  CUT LOADED (Start/Stop press aborts)");
  Serial.println("  ENC | TEACH [J1] <seconds> [Hz] [smooth_ms] [max_dev]");
  Serial.println("  STREAM ON | STREAM OFF | STREAM RATE <1-50 Hz>");
  Serial.println("  FEEDBACK ON | FEEDBACK OFF | FEEDBACK STATUS");
  Serial.println("  CONTROLS | LEDS | PINS | RELAY ON/OFF/STATUS");
  Serial.println("  CONTROL TEST ON/OFF | STATE TEST ON/OFF");
  Serial.println("  PLAY | CLEAR | POS | HELP");
  Serial.println("  BLADE RETRACTED | BLADE DOWN | BLADE STATUS");
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0) return;
  command.toUpperCase();

  if (command == "CONTROL TEST ON") return setControlTest(true);
  if (command == "CONTROL TEST OFF") return setControlTest(false);
  if (command == "STATE TEST ON") return setStateTest(true);
  if (command == "STATE TEST OFF") return setStateTest(false);
  if (command == "CONTROL TEST STATUS") {
    Serial.println(anyPanelTestActive() ? "CONTROL TEST ON" : "CONTROL TEST OFF");
    return;
  }
  if (command == "CONTROLS") return printControlStatus();
  if (command == "LEDS") return printLedStatus();
  if (command == "PINS") return printPanelPinMap();
  if (command == "RELAY ON") return setMachinePower(true);
  if (command == "RELAY OFF") return setMachinePower(false);
  if (command == "RELAY STATUS") {
    Serial.println(relayConnected ? "RELAY CONNECTED" : "RELAY DISCONNECTED");
    return;
  }
  if (command == "HELP") return printHelp();
  if (anyPanelTestActive()) {
    Serial.println("ERROR: input test is active; turn the test OFF first");
    return;
  }
  if (command == "BLADE RETRACTED") {
    bladeRetracted(true);
    return;
  }
  if (command == "BLADE DOWN") {
    bladeDown(true);
    return;
  }
  if (command == "BLADE STATUS") {
    printBladeStatus();
    return;
  }
  if (command.startsWith("BLADE ")) {
    Serial.println("ERROR: use BLADE RETRACTED or BLADE DOWN");
    return;
  }

  if (command == "ARM FOLDED") return armAtFoldedPose(AxisMode::DUAL);
  if (command == "ARM J1") return armAtFoldedPose(AxisMode::J1_ONLY);
  if (command == "ARM J2") return armAtFoldedPose(AxisMode::J2_ONLY);
  if (command == "DISARM") {
    disableDrivers();
    armMode = AxisMode::DUAL;
    productReady = false;
    productState = ProductState::IDLE;
    if (bladeIsDown()) bladeRetracted();
    Serial.println("DISARMED");
    printOperationReport("DISARMED");
    return;
  }
  if (command == "POS") return printPosition();
  if (command == "CLEAR") {
    taughtCount = 0;
    productHasLastCut = false;
    Serial.println("TAUGHT MOVEMENT CLEARED");
    return;
  }
  if (command == "PLAY") {
    replayTeach();
    return;
  }
  if (command == "CUT LOADED") {
    cutLoadedPath();
    return;
  }
  long loadPointCount = 0;
  char loadTrailing = '\0';
  if (sscanf(command.c_str(), "LOAD POINTS %ld %c", &loadPointCount, &loadTrailing) == 1) {
    loadPointsFromSerial(loadPointCount);
    return;
  }
  if (command == "FEEDBACK ON") {
    encoderFeedbackEnabled = true;
    Serial.println("FEEDBACK ON: correction and position faults enabled");
    return;
  }
  if (command == "FEEDBACK OFF") {
    encoderFeedbackEnabled = false;
    Serial.println("WARNING: FEEDBACK OFF; motor motion is open-loop");
    return;
  }
  if (command == "FEEDBACK STATUS") {
    Serial.println(
        encoderFeedbackEnabled ? "FEEDBACK ON: correction and position faults enabled"
                                : "FEEDBACK OFF: motor motion is open-loop");
    return;
  }
  if (command == "STREAM ON") {
    if (!encodersCalibrated) {
      Serial.println("ERROR: type ARM FOLDED, ARM J1, or ARM J2 before streaming");
    } else {
      encoderStreamEnabled = true;
      nextEncoderStreamMs = 0;
      Serial.println("ENC_STREAM ON");
    }
    return;
  }
  if (command == "STREAM OFF") {
    encoderStreamEnabled = false;
    Serial.println("ENC_STREAM OFF");
    return;
  }
  float streamRate = 0.0f;
  if (sscanf(command.c_str(), "STREAM RATE %f", &streamRate) == 1) {
    if (streamRate < 1.0f || streamRate > 50.0f) {
      Serial.println("ERROR: STREAM RATE must be 1-50 Hz");
    } else {
      encoderStreamHz = streamRate;
      nextEncoderStreamMs = 0;
      Serial.print("ENC_STREAM RATE ");
      Serial.print(encoderStreamHz, 1);
      Serial.println(" Hz");
    }
    return;
  }
  if (command == "ENC") return printEncoderStatus();

  float argA = 0.0f, argB = 0.0f, argC = 0.0f, argD = 0.0f;
  char optionText[8] = {};
  if (sscanf(command.c_str(), "ANGLES %f %f", &argA, &argB) == 2) {
    moveToAngles(argA, argB);
    return;
  }
  if (sscanf(command.c_str(), "J1 %f", &argA) == 1) {
    moveToAngles(argA, currentJ2Deg());
    return;
  }
  if (sscanf(command.c_str(), "J2 %f", &argA) == 1) {
    moveToAngles(currentJ1Deg(), argA);
    return;
  }
  const int xyFields = sscanf(command.c_str(), "XY %f %f %7s", &argA, &argB, optionText);
  if (xyFields >= 2) {
    if (!armed || armMode != AxisMode::DUAL) {
      Serial.println("ERROR: XY requires ARM FOLDED");
    } else if (xyFields == 3 ? moveToXY(argA, argB, parseElbow(optionText))
                              : moveToXYAuto(argA, argB)) {
      printPosition();
    }
    return;
  }
  const int cutFields =
      sscanf(command.c_str(), "CUT %f %f %f %f %7s", &argA, &argB, &argC, &argD, optionText);
  if (cutFields >= 4) {
    if (!armed || armMode != AxisMode::DUAL) {
      Serial.println("ERROR: CUT requires ARM FOLDED");
    } else {
      cutLine(argA, argB, argC, argD, cutFields == 5 && parseElbow(optionText));
    }
    return;
  }

  char axis[3] = {};
  long rawSteps = 0;
  if (sscanf(command.c_str(), "TEST %2s %ld", axis, &rawSteps) == 2) {
    if (!armed) {
      Serial.println("ERROR: type ARM FOLDED first");
    } else if (strcmp(axis, "J1") == 0 && armMode != AxisMode::J2_ONLY) {
      executeSteps(rawSteps, 0);
      Serial.println("OK");
      printOperationReport("TEST_J1_COMPLETE");
    } else if (strcmp(axis, "J2") == 0 && armMode != AxisMode::J1_ONLY) {
      executeSteps(0, rawSteps);
      Serial.println("OK");
      printOperationReport("TEST_J2_COMPLETE");
    } else {
      Serial.println("ERROR: invalid or blocked test axis");
    }
    return;
  }

  float teachSeconds = 0.0f, teachHz = 20.0f, smoothMs = 150.0f, maxDeviationDeg = 1.0f;
  int fields = sscanf(
      command.c_str(), "TEACH J1 %f %f %f %f", &teachSeconds, &teachHz, &smoothMs,
      &maxDeviationDeg);
  if (fields >= 1) {
    recordTeach(teachSeconds, teachHz, true, smoothMs, maxDeviationDeg);
    return;
  }
  fields = sscanf(
      command.c_str(), "TEACH %f %f %f %f", &teachSeconds, &teachHz, &smoothMs,
      &maxDeviationDeg);
  if (fields >= 1) {
    recordTeach(teachSeconds, teachHz, false, smoothMs, maxDeviationDeg);
    return;
  }
  Serial.println("ERROR: unknown command; type HELP");
}
