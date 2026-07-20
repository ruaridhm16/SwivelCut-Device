// Serial status reporting: joint/Cartesian position reports used after every
// command, plus an opt-in verbose trace for motion tuning.
#pragma once

// Prints commanded and (if calibrated) encoder-measured joint angles plus
// forward-kinematic XY, tagged with label. The standard "operation done"
// line most commands print on completion.
void printOperationReport(const char *label);

// Verbose target-vs-measured trace, gated behind Config.h's
// MOTION_DIAGNOSTICS since it adds Serial I/O inside motion timing loops.
void printMotionDiagnostic(
    const char *stage, int correction, float targetJ1, float targetJ2,
    float measuredJ1, float measuredJ2);

void printPosition();
