#pragma once
#include "main.h"

// Call once per loop() tick while in STATE_FIRE_FIGHTING.
// Returns NEXT_STEP when both fires are extinguished.
// Returns CURRENT_STEP while still working.
STEP runFireRoutine();

// Call this if you ever need to restart the fire routine from scratch.
void resetFireRoutine();


