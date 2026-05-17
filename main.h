#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

enum STEP {
    INITIALISING,
    IDLE,
    FIRE,
    STOPPED,
    ERROR,
    CURRENT_STEP,   // returned by runFireRoutine() while working
    NEXT_STEP,      // returned by runFireRoutine() when done
    AVOID_STEP
};

STEP initialising();
STEP idle();
STEP fire();
STEP stopped();
STEP error();

extern HardwareSerial *SerialCom;

#define DUAL_PRINT(x)   do { Serial.print(x);   if(SerialCom) SerialCom->print(x);   } while(0)
#define DUAL_PRINTLN(x) do { Serial.println(x); if(SerialCom) SerialCom->println(x); } while(0)

#endif