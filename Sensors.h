#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <SharpDistSensor.h>
#include <NewPing.h>

#define US_FILTER_SIZE 7

class Sensors {
public:
    Sensors();
    void setSerial(HardwareSerial *serialCom);
    bool initialise();

    // ── Front short-range IR (forward-facing, front corners) ─────────────────
    float readIRFrontLeft();    // irShort1 A13 — front-left  corner, fwd-facing
    float readIRFrontRight();   // irShort2 A15 — front-right corner, fwd-facing

    // ── Rear long-range IR (sideways-facing, rear corners) ───────────────────
    float readIRRearLeft();     // irLong1  A8  — rear-left  corner, faces left
    float readIRRearRight();    // irLong2  A10 — rear-right corner, faces right

    // ── Legacy raw accessors (kept for backwards compat) ─────────────────────
    uint16_t readShortRangeIR1();
    uint16_t readShortRangeIR2();
    uint16_t readLongRangeIR1();
    uint16_t readLongRangeIR2();

    // ── Ultrasonic ────────────────────────────────────────────────────────────
    float readUltrasonicCm();
    float pingNowCm();
    void  WarmUSFilter(uint8_t samples = 10);
    void  setUSStrictFilter(bool enabled) { _usStrictFilter = enabled; }

    // ── IMU ───────────────────────────────────────────────────────────────────
    float getGyroHeading();
    void  ZeroGyroHeading();

    // ── Battery ───────────────────────────────────────────────────────────────
    bool isBatteryVoltageOK();

    // ── Debug ─────────────────────────────────────────────────────────────────
    void printLongRangeIR2();
    void printGyroZ();
    void printUltrasonicRange();

private:
    HardwareSerial   *_serial;
    Adafruit_BNO08x   _bno08x;
    sh2_SensorValue_t _sensorValue;
    byte              _lowVoltageCounter;

    float _currentAbsoluteYaw;
    float _yawOffset;

    unsigned long _usLastPollMs;
    unsigned int  _usBuf[US_FILTER_SIZE];
    uint8_t       _usBufIdx;
    bool          _usBufFull;
    float         _usFiltered;
    bool          _usStrictFilter = false;

    // Front short-range: A13 (front-left), A15 (front-right)
    // Rear  long-range:  A8  (rear-left),  A10 (rear-right)
    // Constructor second arg = median window size (built-in averaging)
    SharpDistSensor _irShort1{A13, 5};   // front-left,  forward-facing
    SharpDistSensor _irShort2{A15, 5};   // front-right, forward-facing
    SharpDistSensor _irLong1 {A8,  5};   // rear-left,   faces left
    SharpDistSensor _irLong2 {A10, 5};   // rear-right,  faces right

    NewPing _sonar{48, 49, 400};

    static const uint8_t BATTERY_PIN = A0;
};