// Teach-by-hand path recording, playback, and the product-facing
// teach/trace/cut workflow driven by the panel buttons and head sensor.
#pragma once

#include "Types.h"

extern ProductState productState;
extern bool productReady;
extern bool stabilizationEnabled;
extern bool repeatCutActive;
extern bool productCutActive;
extern bool productAbortRequested;
extern bool productHasLastCut;
extern bool tracerReplayActive;
extern int taughtCount;

// Re-homes to the folded pose (J1=0, J2=180), recalibrates the encoder(s)
// for `mode`, and arms the drivers only if enableAfterCalibration is set.
void armAtFoldedPose(AxisMode mode, bool enableAfterCalibration = true);

// Records a hand-guided move for `seconds` at `hz`, then smooths it in
// Cartesian space (smoothingMs window, maxDeviationDeg clamp) into the
// active taught path. Used by the TEACH serial command.
void recordTeach(
    float seconds, float hz, bool j1Only, float smoothingMs,
    float maxDeviationDeg);

// Replays the current taught path. operateBlade lowers the blade after
// reaching the first point and retracts it at the end/on stop.
bool replayTeach(bool operateBlade = false);

// Full product cut: validates head/readiness, re-smooths and cutter-length
// compensates the path, then replays it with the blade down.
void runProductCut(bool repeat);

// Replays the taught path with the tracer head, starting from whichever
// endpoint is physically closer to the arm's current position.
void runTracerReplay();

// Loads an explicit J1/J2 point list from Serial (LOAD POINTS command).
void loadPointsFromSerial(long requestedCount);

// Cuts whatever path is currently loaded/taught without re-teaching.
void cutLoadedPath();

// Routes a debounced panel button press to the right teach/cut/replay action.
void handleProductButtonChange(const ButtonInput &button);

// Ticks in-progress product teaching (auto-sampling at PRODUCT_TEACH_HZ).
// Call every loop() iteration.
void serviceProductWorkflow();

bool moveToXY(float x, float y, bool elbowDown);
bool moveToXYAuto(float x, float y);
bool cutLine(float x0, float y0, float x1, float y1, bool elbowDown);
