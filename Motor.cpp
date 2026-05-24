#include "Motor.h"

#define MotorOffset 60
#define TurnTolerance 0.5 //deg

Motor::Motor(uint8_t leftFrontPin, uint8_t leftRearPin, uint8_t rightRearPin, uint8_t rightFrontPin)
	: _leftFrontPin(leftFrontPin),
	  _leftRearPin(leftRearPin),
	  _rightRearPin(rightRearPin),
	  _rightFrontPin(rightFrontPin),
	  _speed(100),
	  _driveStraightTargetAng(0.0f),
	  _driveStraightTargetSideDist(0.0f),
	  _driveStraightTargetEndDist(0.0f),
	  _driveStraightKp(10.0f),
	  //_driveStraightKi(0.1f),
	  //_driveStraightKd(0.1f),
	  _driveStraightKff(0.0f),
	  _driveStraightIntegral(0.0f),
	  _driveStraightPrevError(0.0f),
	  _driveStraightPrevMs(0),
	  _driveStraightTargetSet(false),
	  _turnTargetAngle(0),
	  _serial(&Serial),
	  turnTargetSet(false),
	  turn_kp_large(6),
	  _driveStraightAtDistKp(120), 
	  strafeIRKp(5) {}

void Motor::initialise(HardwareSerial *serialCom) {
	if (serialCom) _serial = serialCom;

	enable(); // Attaches the servos to pins
	stop();   // Sets all motors to 1500ms (neutral)

	// Reset Drive Straight (PID) variables to clean state
	_driveStraightTargetSet = false;
	_driveStraightIntegral = 0.0f;
	_driveStraightPrevError = 0.0f;
	_driveStraightPrevMs = 0;

	// Ensure direction is neutral
	_driveStraightTargetAng = 0.0f;
	_driveStraightTargetSideDist = 0.0f;
	_driveStraightTargetEndDist = 0.0f;

	_serial->println(F("Motor System Initialised."));
}

void Motor::setSerial(HardwareSerial *serialCom) {
	if (serialCom != nullptr) {
		_serial = serialCom;
	}
}

void Motor::enable() {
	_leftFrontMotor.attach(_leftFrontPin);
	_leftRearMotor.attach(_leftRearPin);
	_rightRearMotor.attach(_rightRearPin);
	_rightFrontMotor.attach(_rightFrontPin);
}

void Motor::disable() {
	_leftFrontMotor.detach();
	_leftRearMotor.detach();
	_rightRearMotor.detach();
	_rightFrontMotor.detach();

	pinMode(_leftFrontPin, INPUT);
	pinMode(_leftRearPin, INPUT);
	pinMode(_rightRearPin, INPUT);
	pinMode(_rightFrontPin, INPUT);
}

void Motor::setSpeed(int speed) {
	_speed = constrain(speed, 0, 1000);
}

void Motor::changeSpeed(int delta) {
	setSpeed(_speed + delta);
}

int Motor::getSpeed() const {
	return _speed;
}

void Motor::driveForward() {
	writeAll(1500 + _speed, 1500 + _speed, 1500 - _speed, 1500 - _speed);
}

void Motor::driveReverse() {
	writeAll(1500 - _speed, 1500 - _speed, 1500 + _speed, 1500 + _speed);
}

void Motor::strafeLeft() {
	writeAll(1500 - _speed, 1500 + _speed, 1500 + _speed, 1500 - _speed);

}

void Motor::strafeRight() {
	writeAll(1500 + _speed, 1500 - _speed, 1500 - _speed, 1500 + _speed);
}

void Motor::rotateClockwise() {
	float speed = 150;
	writeAll(1500 + speed, 1500 + speed, 1500 + speed, 1500 + speed);
}	

void Motor::rotateCounterClockwise() {
	writeAll(1500 - _speed, 1500 - _speed, 1500 - _speed, 1500 - _speed);
}

void Motor::SetTurnTarget(float turnAngle) {
	float wrapped = fmod(turnAngle, 360.0f);
	if (wrapped < 0.0f) {
		wrapped += 360.0f;
	}
	turn_rel_target = wrapped;
	turn_error = 0;
	turn_start_ms = millis();
	turnTargetSet = true;
	turn_exit_time = 0;
	pidAngCorrection = 0;
	ResetDriveStraightTarget();
	// _serial->println("TURN TARGET SET");
}

void Motor::ResetTurnTarget(){
	turn_rel_target = 0;
	turnTargetSet = false;

}

bool Motor::isTurnComplete(float gyro_angle){
	if (!turnTargetSet) {
		return true;
	}

	wrappedGyro = fmod(gyro_angle, 360.0f);
	if (wrappedGyro < 0.0f) {
		wrappedGyro += 360.0f;
	}

	const unsigned long now = millis();
	// float dt = 0.0f;
	// if (turn_prev_ms > 0 && now >= turn_prev_ms) {
	// 	dt = (now - turn_prev_ms) / 1000.0f;
	// }

	turn_error = wrapAngleError(turn_rel_target, wrappedGyro);
	const float errorAbs = fabsf(turn_error);

	if (errorAbs < 8){
		pidAngCorrection = 0;
	} else if (errorAbs >= 8){
		pidAngCorrection = turn_error * turn_kp_large;
	} 
	
	if (turn_error >= TurnTolerance){
			pidAngCorrection = pidAngCorrection + MotorOffset;
		} else if (turn_error <= -TurnTolerance){
			pidAngCorrection = pidAngCorrection - MotorOffset;
		}

	if ((now - turn_start_ms) < 1000){
		pidAngCorrection = pidAngCorrection * (now - turn_start_ms)/1000;
	}

	leftCommand = constrain(1500 - pidAngCorrection, 1200, 1800);
	rightCommand = constrain(1500 - pidAngCorrection, 1200, 1800);
	writeAll(leftCommand, leftCommand, rightCommand, rightCommand);

	if (fabsf(turn_error) > TurnTolerance){
		turn_exit_time = millis();
	}

	if ((now - turn_exit_time) > 400){
		// _serial->println("Turn complete");
		stop();
		ResetTurnTarget();
		return true;
	}

	return false;
}


void Motor::setDriveStraightTarget(float targetHeadingDegrees) {
	SetDriveStraightTarget(DIRECTION::FORWARD, targetHeadingDegrees, 0.0f, 0.0f);
}

void Motor::SetDriveStraightTarget(DIRECTION direction, float targetHeadingDegrees, float targetUSDistance, float targetIRDistance) {
	float wrapped = fmod(targetHeadingDegrees, 360.0f);
	if (wrapped < 0.0f) {
		wrapped += 360.0f;
	}
	_driveStraightTargetAng = wrapped;
	_driveStraightTargetSideDist = targetUSDistance;
	_driveStraightTargetEndDist = targetIRDistance;
	_driveStraightIntegral = 0.0f;
	_driveStraightPrevError = 0.0f;
	_driveStraightPrevMs = millis();
	_driveStraightTargetSet = true;
	_driveStraightDirection = direction;

	ResetTurnTarget();
}

void Motor::ResetDriveStraightTarget(){
	_driveStraightTargetAng = 0;
	_driveStraightTargetSideDist = 0;
	_driveStraightTargetEndDist = 0;
	_driveStraightTargetSet = false;
}

bool Motor::driveStraight(float gyro_angle, float usDist, float irDist) {
	if (!_driveStraightTargetSet){
		ResetDriveStraightTarget();
		return false;
	}
	_irDist = irDist;

	if((irDist < _driveStraightTargetEndDist) && (irDist > 5)){
		// _serial->println("STOPPED DRIVING");
		ResetDriveStraightTarget();
		stop();
		return 1;
	}

	wrappedGyro = fmod(gyro_angle, 360.0f);
	if (wrappedGyro < 0.0f) {
		wrappedGyro += 360.0f;
	}

	const unsigned long now = millis();
	driveStraightAngError = wrapAngleError(_driveStraightTargetAng, wrappedGyro);
	pidAngCorrection = (_driveStraightKp * driveStraightAngError);

	if(_driveStraightTargetSideDist == 0){
		pidDistCorrection = 0;
	} else{
		driveStraightDistError = _driveStraightTargetSideDist - usDist;
		pidDistCorrection = _driveStraightAtDistKp * driveStraightDistError;
	}

	if(_driveStraightDirection == DIRECTION::REVERSE){
		speed = -_speed;
	} else if (_driveStraightDirection == DIRECTION::FORWARD){
		speed = _speed;
	}

	leftFrontCommand  = constrain(1500 + speed - pidAngCorrection - pidDistCorrection, 1100, 1900);
	leftRearCommand   = constrain(1500 + speed - pidAngCorrection + pidDistCorrection, 1100, 1900);
	rightRearCommand  = constrain(1500 - speed - pidAngCorrection + pidDistCorrection, 1100, 1900);
	rightFrontCommand = constrain(1500 - speed - pidAngCorrection - pidDistCorrection, 1100, 1900);

	writeAll(leftFrontCommand, leftRearCommand, rightRearCommand, rightFrontCommand);

	_driveStraightPrevError = driveStraightAngError;
	_driveStraightPrevMs = now;

	if (!initialErrorSet){
		initialErrorSet = true;
		startTime = millis();
	}
	return true;
}


bool Motor::strafeToUSDist(float targetDist, float usDist, float gyroAngle, float irDist) {
	float distError = targetDist - usDist;
	float angError  = wrapAngleError(_driveStraightTargetAng, fmod(gyroAngle, 360.0f));
	float irError = 50 - irDist;
	float irCorrection = 0;

	if (fabsf(distError) <= 3.0f) {
		stop();
		return false;
	}

	if (irDist != 0){ irCorrection = constrain(strafeIRKp * irError, -150.0f, 150.0f); }

	float distCorrection = constrain(_driveStraightAtDistKp * distError, -120.0f, 120.0f);

	// Apply minimum offset to overcome motor deadband, preserving sign
	if (distCorrection > 0)       distCorrection = max(distCorrection, (float)MotorOffset);
	else if (distCorrection < 0)  distCorrection = min(distCorrection, -(float)MotorOffset);

	float angCorrection = 0;
	// float angCorrection = constrain(_driveStraightKp * angError, -50.0f, 50.0f);

	// _serial->print("Ir Dist = ");
	// _serial->println(irDist);
	// _serial->print("Control Effort = ");
	// _serial->println(irCorrection);

	leftFrontCommand  = constrain(1500 + irCorrection - distCorrection - angCorrection, 1200, 1800);
	leftRearCommand   = constrain(1500 + irCorrection + distCorrection - angCorrection, 1200, 1800);
	rightRearCommand  = constrain(1500 - irCorrection + distCorrection + angCorrection, 1200, 1800);
	rightFrontCommand = constrain(1500 - irCorrection - distCorrection + angCorrection, 1200, 1800);

	writeAll(leftFrontCommand, leftRearCommand, rightRearCommand, rightFrontCommand);
	return true;
}


void Motor::PrintDetails() {
	//_serial->print("Current angle = ");
	//_serial->println(wrappedGyro);
	//_serial->print("Error = ");
	//_serial->println(turn_error);
	// _serial->println();

return;
}

float Motor::wrapAngleError(float target, float current) const {
	float error = target - current;
	while (error > 180.0f) {
		error -= 360.0f;
	}
	while (error < -180.0f) {
		error += 360.0f;
	}
	return error;
}

void Motor::writeAll(int leftFrontPulse, int leftRearPulse, int rightRearPulse, int rightFrontPulse) {
	_leftFrontMotor.writeMicroseconds(leftFrontPulse);
	_leftRearMotor.writeMicroseconds(leftRearPulse);
	_rightRearMotor.writeMicroseconds(rightRearPulse);
	_rightFrontMotor.writeMicroseconds(rightFrontPulse);
}

void Motor::stop() {
	writeAll(1500, 1500, 1500, 1500);
}

void Motor::log(const char *message) const {
	if (_serial != nullptr && message != nullptr) {
		_serial->println(message);
	}
}

