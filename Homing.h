#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include "main.h"

struct MinResult;

struct homing_dir;

STEP homing();

bool findNextMin(homing_dir *dir);

#endif