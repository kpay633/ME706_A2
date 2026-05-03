#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

enum STEP {
  INITIALISING,
  IDLE,
  HOMING,
  TILLING,
  STOPPED,
  ERROR,
  DEBUG
};

STEP initialising();
STEP idle();
STEP homing();
STEP tilling();
STEP stopped();
STEP error();



// Add this line FIRST so the macros know what SerialCom is
extern HardwareSerial *SerialCom;

#define DUAL_PRINT(x)   do { Serial.print(x);   if(SerialCom) SerialCom->print(x);   } while(0)
#define DUAL_PRINTLN(x) do { Serial.println(x); if(SerialCom) SerialCom->println(x); } while(0)


#endif