#pragma once
#include "main.h"

struct ObstacleClearanceResult {
	bool isClear;
	float closestObstacleAngleDeg;
	float closestObstacleDistanceMm;
};

// Call once per loop() tick while in STATE_FIRE_FIGHTING.
// Returns NEXT_STEP when both fires are extinguished.
// Returns CURRENT_STEP while still working.
STEP runFireRoutine();

// Call this if you ever need to restart the fire routine from scratch.
void resetFireRoutine();

// Sweep the front-mounted ultrasonic sensor across 0..180 degrees and check
// whether the clearance box in front of the robot is free of obstacles.
ObstacleClearanceResult checkForwardClearance();


