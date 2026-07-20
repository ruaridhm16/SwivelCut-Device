// Hardware-timer stepper engine: generates STEP pulses for both joints in
// the background via ESP32 hardware timer ISRs, independent of the main
// loop, and layers closed-loop encoder correction on top of open-loop moves.
#pragma once

#include <Arduino.h>

#include "Types.h"

extern volatile long j1PositionSteps;
extern volatile long j2PositionSteps;
extern bool armed;
extern AxisMode armMode;
extern bool motorsMoving;
extern bool stepperTimersReady;
extern int lastCommandDirectionJ1;
extern int lastCommandDirectionJ2;

long atomicReadSteps(volatile long &steps);
void atomicWriteSteps(volatile long &steps, long value);

float currentJ1Deg();
float currentJ2Deg();

// Configures step/dir/enable pins and starts the per-axis hardware timers.
// Must run before any motion command.
void stepperMotionSetup();

void enableDrivers();
void disableDrivers();

// Disables drivers, retracts the blade if needed, and flags encoderFault so
// the next command surfaces the mismatch instead of silently continuing.
void feedbackFault(const char *message);

// Compares commanded position against encoder-measured position and raises
// feedbackFault() if either joint has drifted past FEEDBACK_MAX_ERROR_DEG.
// A no-op (returns true) when USE_ENCODERS or feedback is disabled.
bool checkFeedback();

// Blocking open-loop move of both axes by the given step counts, run over
// requestedDurationUs (stretched to the drivers' minimum step period).
// Services controls/encoder streaming and product-cut abort while waiting.
bool executeSteps(long deltaJ1, long deltaJ2);

// Lower-level segment control used by the continuous-trajectory replay path,
// which needs to plan its own per-segment duration (velocity profile) rather
// than the constant-rate timing executeSteps() applies.
bool startSegment(long deltaJ1, long deltaJ2, unsigned long durationUs);
bool segmentInProgress();
void stopMotionSegment();
long segmentEmittedStepEvents();

// Moves to an absolute joint pose with closed-loop settling: commands the
// step delta, checks encoder error, and re-issues a correction move up to
// FEEDBACK_MAX_CORRECTIONS times if the tolerance isn't met.
bool moveToAngles(
    float j1Deg, float j2Deg, bool report = true,
    bool allowOutsideLimits = false);
