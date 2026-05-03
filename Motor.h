#pragma once

#include <Arduino.h>
#include <Servo.h>

enum DIRECTION { FORWARD, REVERSE };

class Motor {
 public:
	Motor(uint8_t leftFrontPin = 46, uint8_t leftRearPin = 47, uint8_t rightRearPin = 50, uint8_t rightFrontPin = 51);

	void initialise(HardwareSerial *serialCom = nullptr);
	void setSerial(HardwareSerial *serialCom);
	void enable();
	void disable();
	void setSpeed(int speed);
	void changeSpeed(int delta);
	int getSpeed() const;
	void driveForward();
	void driveReverse();
	void strafeLeft();
	void strafeRight();
	void rotateClockwise();
	void rotateCounterClockwise();
	void SetTurnTarget(float turnAngle);
	void ResetTurnTarget();
	bool isTurnComplete(float gyro_angle);
	void setDriveStraightTarget(float targetHeadingDegrees);
	void SetDriveStraightTarget(DIRECTION direction, float targetHeadingDegrees, float targetUSDist, float targetIRDist);
	void ResetDriveStraightTarget();
	void driveStraight(DIRECTION, float gyro_angle);
	bool driveStraight(float gyro_angle, float usDist, float irDIst);
	bool strafeToUSDist(float targetDist, float usDist, float gyroAngle, float irDist);
	void stop();
	void PrintDetails();

 private:
	void log(const char *message) const;
	void writeAll(int leftFrontPulse, int leftRearPulse, int rightRearPulse, int rightFrontPulse);
	float wrapAngleError(float target, float current) const;

	uint8_t _leftFrontPin;
	uint8_t _leftRearPin;
	uint8_t _rightRearPin;
	uint8_t _rightFrontPin;
	Servo _leftFrontMotor;
	Servo _leftRearMotor;
	Servo _rightRearMotor;
	Servo _rightFrontMotor;
	int _speed;
	int speed;
	float _driveStraightTargetAng;
	float _driveStraightTargetSideDist;
	float _driveStraightTargetEndDist;
	DIRECTION _driveStraightDirection;
	float _driveStraightKp;
	float _driveStraightKi;
	float _driveStraightKd;
	float _driveStraightKff;
	float _driveStraightIntegral;
	float _driveStraightPrevError;
	unsigned long _driveStraightPrevMs;
	bool _driveStraightTargetSet;
	int _turnTargetAngle;
	HardwareSerial *_serial;
	float wrappedGyro;

	float _driveStraightAtDistKp;

	//drive straight variables
	float driveStraightAngError;
	float driveStraightDistError;
	int leftCommand;
	int rightCommand;
	int leftFrontCommand;
	int rightFrontCommand;
	int leftRearCommand;
	int rightRearCommand;
	bool initialErrorSet;
	float startTime;
	float pidAngCorrection;
	float pidDistCorrection;

	//turn rel variables
	float turn_rel_target;
	float turn_integral;
	float turn_prev_error;
	float turn_start_ms;
	bool turnTargetSet;
	float turn_error;
	float turn_kp_large;
	float turn_exit_time;

	float _irDist;
	float strafeIRKp;
	float irCorrection;

};
