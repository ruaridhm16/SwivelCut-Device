// Blade motor control: a single PWM+direction driver that only ever runs
// timed down/retract strokes, so there's no need for closed-loop position.
#pragma once

#include "Types.h"

extern BladePosition bladePosition;
extern bool bladePwmReady;

void bladeSetup();

void stopBlade();
bool bladeIsDown();

// Drives the blade down/retracted for its fixed stroke time. force=true
// re-runs the stroke even if the tracked position already matches, which
// setup() and abort paths use to guarantee a known physical position.
void bladeDown(bool force = false);
void bladeRetracted(bool force = false);

void printBladeStatus();
