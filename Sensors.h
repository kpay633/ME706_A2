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

	uint16_t readShortRangeIR1();
	uint16_t readShortRangeIR2();
	uint16_t readLongRangeIR1();
	uint16_t readLongRangeIR2();

	float readUltrasonicCm();
	// Immediate raw ping (no filtering, no internal rate limiting). Returns 0.0 if out of range.
	float pingNowCm();
	void WarmUSFilter(uint8_t samples = 10);
	void setUSStrictFilter(bool enabled) { _usStrictFilter = enabled; }
	float getGyroHeading();
	bool isBatteryVoltageOK();
	void ZeroGyroHeading();

	void printLongRangeIR2();
	void printGyroZ();
	void printUltrasonicRange();

private:
	HardwareSerial *_serial;
	Adafruit_BNO08x _bno08x;
	sh2_SensorValue_t _sensorValue;
	byte _lowVoltageCounter;

	// --- Gyro Variables ---
	float _currentAbsoluteYaw;
	float _yawOffset;

	// --- Ultrasonic Filter Variables ---
	unsigned long _usLastPollMs;
	unsigned int  _usBuf[US_FILTER_SIZE];
	uint8_t       _usBufIdx;
	bool          _usBufFull;
	float  _usFiltered;
    bool _usStrictFilter = false;

	// SharpDistSensor (pin, median window size = 5)
	SharpDistSensor _irShort1{A5, 5};
	SharpDistSensor _irShort2{A3, 5};
	SharpDistSensor _irLong1 {A2, 5};
	SharpDistSensor _irLong2 {A15, 5};

	NewPing _sonar{48, 49, 400};

	static const uint8_t BATTERY_PIN = A0;
};
