#pragma once
#include "main.h"

struct ObstacleClearanceResult {
    bool  isClear;
    // Signed lateral offset from robot centreline (mm): negative = left, positive = right
    float closestObstacleOffsetMm;
    float closestObstacleDistanceMm;
};

// Call once per loop() tick while in STATE_FIRE.
// Returns NEXT_STEP when done, CURRENT_STEP while working.
STEP runFireRoutine();

// Restart the fire routine from the beginning.
void resetFireRoutine();

// Blocking forward-clearance scan (sweeps turret 30–150°, returns immediately).
ObstacleClearanceResult checkForwardClearance();

// Non-blocking scanner: call clearanceScanStart() once, then call
// clearanceScanStep(out) every loop tick until it returns true.
void clearanceScanStart();
bool clearanceScanStep(ObstacleClearanceResult &out);
