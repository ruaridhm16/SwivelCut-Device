// Operator panel: four debounced buttons, one WS2812 status LED per button,
// the power relay, and the head-type ID sensor (cutter/tracer/disconnected).
// Also hosts CONTROL TEST / STATE TEST, which repurpose the same buttons and
// LEDs to verify wiring without touching motors or the blade.
#pragma once

#include "Types.h"

extern bool relayConnected;

void panelSetup();

// Debounces buttons, drives product-button actions, and updates the sampled
// head type. Call every loop() iteration and from any blocking wait so
// button/relay/head state stays live during motion.
void serviceControlInputs();

void refreshButtonLeds();

// Relay ON re-homes the folded arm with drivers disabled, then arms it if
// calibration succeeds. Relay OFF disables drivers and retracts the blade.
void setMachinePower(bool enabled);

void setControlTest(bool enabled);
void setStateTest(bool enabled);
bool anyPanelTestActive();

void printControlStatus();
void printLedStatus();
void printPanelPinMap();

bool headTypeKnown();
HeadType currentHeadType();
