/*
  MechEng 706 Base Code
  Hardware: Arduino Mega2560, BNO085, HC-SR04, Sharp IR, Servo,
            Vex Motor Controller 29, Turnigy nano-tech 2200mah 2S, HC-12
  Modified: 18/02/2026  Author: Trishit Ghatak
*/

/*
        DONT CHANGE HEADERS
-----------------------------
*/

#include "main.h"
#include <Servo.h>
#include "Motor.h"
#include "Sensors.h"
#include "fire.h"

//#define NO_READ_GYRO
//#define NO_HC_SR04
//#define NO_BATTERY_V_OK

char read_serial_command();
void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();

Servo turret_motor;
Motor motors;
Sensors sensors;

// SerialCom → Serial (USB) for monitoring; swap to &Serial2 for HC-12 wireless
HardwareSerial *SerialCom;

unsigned long main_previous_millis = 0;
float gyro_vel;
float gyro_angle = 0.0f;
static STEP machine_step = STEP::INITIALISING;
bool started = false;

void setup(void) {
    turret_motor.attach(5);
    turret_motor.write(90);
    pinMode(LED_BUILTIN, OUTPUT);
    main_previous_millis = 0;

    Serial.begin(9600);
    Serial.println(F("USB Serial active - monitoring only"));

    Serial2.begin(115200);
    SerialCom = &Serial;   // change to &Serial2 when running wireless

    SerialCom->println(F("MECHENG706_Base_Code"));
    delay(1000);
    SerialCom->println(F("Setup...."));
    delay(1000);
}

void loop(void) {
    switch (machine_step) {
        case INITIALISING: machine_step = initialising(); break;
        case IDLE:         machine_step = idle();         break;
        case FIRE:         machine_step = fire();         break;
        case STOPPED:      machine_step = stopped();      break;
        case ERROR:        machine_step = error();        break;
        default: break;
    }
}

STEP initialising() {
    static unsigned long next_retry_ms = 50;

    if (!started) {
        SerialCom->println(F("INITIALISING...."));
        SerialCom->println(F("Enabling Motors..."));
        motors.initialise(SerialCom);
        sensors.setSerial(SerialCom);
        started = true;
        Serial.println(F("A"));
    }

    if (millis() < next_retry_ms) {
        Serial.println(F("B"));
        return STEP::INITIALISING;
    }

    SerialCom->println(F("Enabling Sensors..."));
    if (!sensors.initialise()) {
        SerialCom->println(F("Sensor init failed - retrying..."));
        next_retry_ms = millis() + 1000;
        Serial.println(F("C"));
        return STEP::INITIALISING;
    }

    SerialCom->println(F("RUNNING STATE..."));
    Serial.println(F("D"));
    return STEP::IDLE;
}

STEP idle() {
    static unsigned long previous_millis;
    Serial.println(F("IDLE"));

#ifndef NO_BATTERY_V_OK
    if (!sensors.isBatteryVoltageOK()) return STEP::ERROR;
#endif

    if (millis() - previous_millis > 200) {
        previous_millis = millis();
        // Uncomment for diagnostics:
        // SerialCom->print("Heading = "); SerialCom->println(sensors.getGyroHeading());
    }
    return STEP::FIRE;
}

// ─── fire() — delegates entirely to runFireRoutine() ────────────────────────
// The fire-finding sweep, PID turn, gyro-locked approach, and inline obstacle
// avoidance are all managed inside fire.cpp.  No parallel avoidance FSM here.
STEP fire() {
#ifndef NO_BATTERY_V_OK
    if (!sensors.isBatteryVoltageOK()) return STEP::ERROR;
#endif
    if (runFireRoutine() == NEXT_STEP) return STEP::STOPPED;
    return STEP::FIRE;
}

STEP stopped() {
    static byte counter_lipo_voltage_ok;
    static unsigned long previous_millis;
    motors.disable();
    slow_flash_LED_builtin();
    if (millis() - previous_millis > 500) {
        previous_millis = millis();
        SerialCom->println(F("STOPPED---------"));
#ifndef NO_BATTERY_V_OK
        if (sensors.isBatteryVoltageOK()) {
            SerialCom->print(F("Lipo OK waiting Counter 10 < "));
            SerialCom->println(counter_lipo_voltage_ok);
            counter_lipo_voltage_ok++;
            if (counter_lipo_voltage_ok > 10) {
                counter_lipo_voltage_ok = 0;
                motors.enable();
                SerialCom->println(F("Lipo OK returning to RUN STATE"));
                return STEP::IDLE;
            }
        } else {
            counter_lipo_voltage_ok = 0;
        }
#endif
    }
    return STEP::STOPPED;
}

STEP error() {
    return stopped();
}

void fast_flash_double_LED_builtin() {
    static byte indexer = 0;
    static unsigned long fast_flash_millis;
    if (millis() > fast_flash_millis) {
        indexer++;
        if (indexer > 4) {
            fast_flash_millis = millis() + 700;
            digitalWrite(LED_BUILTIN, LOW);
            indexer = 0;
        } else {
            fast_flash_millis = millis() + 100;
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }
    }
}

void slow_flash_LED_builtin() {
    static unsigned long slow_flash_millis;
    if (millis() - slow_flash_millis > 2000) {
        slow_flash_millis = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

char read_serial_command() {
    if (SerialCom->available()) {
        char val = SerialCom->read();
        SerialCom->print(F("Speed:")); SerialCom->print(motors.getSpeed()); SerialCom->print(F(" ms "));
        switch (val) {
            case 'w': case 'W':
                SerialCom->println(F("Forward"));
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, sensors.getGyroHeading(), 15, 0);
                return 'w';
            case 's': case 'S':
                motors.ResetDriveStraightTarget(); return 's';
            case 'q': case 'Q':
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, sensors.getGyroHeading() + 90, 0, 15);
                return 'q';
            case 'e': case 'E':
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, sensors.getGyroHeading() - 90, 0, 15);
                return 'e';
            case 'a': case 'A':
                motors.SetTurnTarget(sensors.getGyroHeading() + 45); return 'a';
            case 'd': case 'D':
                motors.SetTurnTarget(sensors.getGyroHeading() - 45); return 'd';
            case 'h': case 'H': return 'h';
            case 't': case 'T': return 't';
            case '-': case '_': motors.SetTurnTarget(-90); return '-';
            case '=': case '+': motors.SetTurnTarget(90);  return '=';
            case ' ': motors.stop(); return ' ';
        }
    }
    return '\0';
}
