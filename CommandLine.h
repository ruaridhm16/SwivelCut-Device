// Serial command parser: the text protocol used for bring-up, calibration,
// and manual jogging alongside the panel's physical product workflow.
#pragma once

#include <Arduino.h>

// The line currently being assembled from Serial, and the terminator ('\n'
// or '\r') that most recently ended a command. Exposed because LOAD POINTS
// reads its point list directly off Serial and needs to know how the
// previous line ended to correctly skip a following '\n' in "\r\n" input.
extern String inputLine;
extern char commandTerminator;

void handleCommand(String command);
void printHelp();
