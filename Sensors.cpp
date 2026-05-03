// NOTE TO CAHNGE SOME STATIC VARS BACK TO ZERO AT END OF FNCT, IN CASE HOMING IS CALLED MULTIPLE TIMES IN A RUN (error)

#include "Sensors.h"
#include <Wire.h>

#define TRIG_PIN 48
#define ECHO_PIN 49

Sensors::Sensors()
	: _serial(&Serial), _bno08x(-1), _lowVoltageCounter(0),
	  _currentAbsoluteYaw(0.0f), _yawOffset(0.0f),
	  _usLastPollMs(0), _usBufIdx(0), _usBufFull(false), _usFiltered(0) {
	memset(_usBuf, 0, sizeof(_usBuf));
}

void Sensors::setSerial(HardwareSerial *serialCom) {
	if (serialCom != nullptr) {
		_serial = serialCom;
	}
}

bool Sensors::initialise() {
	pinMode(BATTERY_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	digitalWrite(TRIG_PIN, LOW);

	_irShort1.setPowerFitCoeffs(38329.8809, -1.0894, 86, 438);
	_irShort2.setPowerFitCoeffs(9033.3337, -0.8135, 45, 421);
	_irLong1.setPowerFitCoeffs(179006.0322, -1.2062, 88, 488);
	_irLong2.setPowerFitCoeffs(148265.9356, -1.1997, 81, 464);

	Wire.begin();
	delay(100); // Give I2C bus time to settle

	bool imuReady = false;
	// Attempt to find the hardware
	for (uint8_t attempt = 0; attempt < 5; ++attempt) {
		if (_bno08x.begin_I2C()) {
			imuReady = true;
			break; // Found it!
		}
		if (_serial) _serial->println(F("BNO085 not found, retrying..."));
		delay(200);
	}

	if (!imuReady) {
		if (_serial) _serial->println(F("IMU Hardware Check Failed"));
		return false;
	}

	// Now enable the reports
	if (!_bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
		if (_serial) _serial->println(F("BNO085 Rotation Report failed!"));
		return false;
	}

	// Optional: Only enable if you actually use Raw Gyro data elsewhere
	if (!_bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000)) {
		if (_serial) _serial->println(F("BNO085 Gyro Report failed!"));
		return false;
	}

	_lowVoltageCounter = 0;
	if (_serial) _serial->println(F("Sensors Initialised Successfully"));
	return true;
}

uint16_t Sensors::readShortRangeIR1() { return _irShort1.getDist(); }
uint16_t Sensors::readShortRangeIR2() { return _irShort2.getDist(); }
uint16_t Sensors::readLongRangeIR1()  { return _irLong1.getDist();  }
uint16_t Sensors::readLongRangeIR2()  { return _irLong2.getDist();  }







float Sensors::readUltrasonicCm() {
    unsigned long now = millis();

    if (now - _usLastPollMs >= 35) {
        _usLastPollMs = now;
        unsigned long pingTime = _sonar.ping();

        // ping_us() returns 0 if out of range
        float rawCm = pingTime / 57.0f;  // 57 µs per cm round-trip

        if (rawCm >= 3.0f) {
			uint8_t count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
			
		    if (_usStrictFilter) {
		        if (count > 0 && fabsf(rawCm - _usFiltered) > 10.0f) {
		            return _usFiltered;
		        }
		    }

            // Store as scaled integer (x10) to keep unsigned int buffer
            _usBuf[_usBufIdx] = (unsigned int)(rawCm * 10.0f);
            _usBufIdx = (_usBufIdx + 1) % US_FILTER_SIZE;
            if (_usBufIdx == 0) _usBufFull = true;

            count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
            if (count > 0) {
                unsigned int tmp[US_FILTER_SIZE];
				memset(tmp, 0, sizeof(tmp)); 
                memcpy(tmp, _usBuf, count * sizeof(unsigned int));

                for (uint8_t i = 1; i < count; i++) {
                    unsigned int key = tmp[i];
                    int8_t j = i - 1;
                    while (j >= 0 && tmp[j] > key) {
                        tmp[j+1] = tmp[j];
                        j--;
                    }
                    tmp[j+1] = key;
                }
                _usFiltered = tmp[count / 2] / 10.0f;  // convert back to cm
            }
        }
    }

    return _usFiltered;
}


// float Sensors::readUltrasonicCm() {
//   unsigned long now = millis();
//
//   // Only ping every 20ms to prevent acoustic echoes
//   if (now - _usLastPollMs >= 35) {
//     _usLastPollMs = now;
//     unsigned int raw = _sonar.ping_cm();
//
//     // Ignore dead values (0) or values too close (<3cm)
//     if (raw >= 3) {
// 			
//       //
//       uint8_t count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
//       bool filterWarm = (count > 0);
//       if (filterWarm && abs((int)raw - (int)_usFiltered) > 10) {
//         return (float)_usFiltered;
//       }
//       // 
//
//       _usBuf[_usBufIdx] = raw;
//       _usBufIdx = (_usBufIdx + 1) % US_FILTER_SIZE;
//       if (_usBufIdx == 0) _usBufFull = true;
//
//       count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
//       if (count > 0) {
//         unsigned int tmp[US_FILTER_SIZE];
//         memcpy(tmp, _usBuf, count * sizeof(unsigned int));
//
//         // Insertion sort for median calculation
//         for (uint8_t i = 1; i < count; i++) {
//           unsigned int key = tmp[i];
//           int8_t j = i - 1;
//           while (j >= 0 && tmp[j] > key) {
//             tmp[j+1] = tmp[j];
//             j--;
//           }
//           tmp[j+1] = key;
//         }
//         _usFiltered = tmp[count / 2];
//       }
//     }
//   }
//
//   return (float)_usFiltered;
// }


void Sensors::WarmUSFilter(uint8_t samples) {
    if (_serial) _serial->println(F("Warming US filter..."));
    uint8_t collected = 0;
    while (collected < samples) {
        unsigned long pingTime = _sonar.ping();
        float rawCm = pingTime / 57.0f;

        if (rawCm >= 3.0f) {
            _usBuf[_usBufIdx] = (unsigned int)(rawCm * 10.0f); 
            _usBufIdx = (_usBufIdx + 1) % US_FILTER_SIZE;
            if (_usBufIdx == 0) _usBufFull = true;
            collected++; 

            uint8_t count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
            unsigned int tmp[US_FILTER_SIZE];
			memset(tmp, 0, sizeof(tmp)); 
            memcpy(tmp, _usBuf, count * sizeof(unsigned int));
            for (uint8_t i = 1; i < count; i++) {
                unsigned int key = tmp[i];
                int8_t j = i - 1;
                while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
                tmp[j+1] = key;
            }
            _usFiltered = tmp[count / 2] / 10.0f;
        }
        delay(40);
    }
    if (_serial) {
        _serial->print(F("US warmed. Reading: "));
        _serial->print(_usFiltered, 1);
        _serial->println(F(" cm"));
    }
}


// void Sensors::WarmUSFilter(uint8_t samples) {
//   if (_serial) _serial->println(F("Warming US filter..."));
//   uint8_t collected = 0;
//   while (collected < samples) {
//     unsigned int raw = _sonar.ping_cm();
//     if (raw >= 3) {
//       _usBuf[_usBufIdx] = raw;
//       _usBufIdx = (_usBufIdx + 1) % US_FILTER_SIZE;
//       if (_usBufIdx == 0) _usBufFull = true;
//       collected++;
//
//       // Recompute median
//       uint8_t count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
//       unsigned int tmp[US_FILTER_SIZE];
//       memcpy(tmp, _usBuf, count * sizeof(unsigned int));
//       for (uint8_t i = 1; i < count; i++) {
//         unsigned int key = tmp[i];
//         int8_t j = i - 1;
//         while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
//         tmp[j+1] = key;
//       }
//       _usFiltered = tmp[count / 2];
//     }
//     delay(25); // slightly longer than the 20ms poll gate
//   }
//   if (_serial) {
//     _serial->print(F("US warmed. Reading: "));
//     _serial->print(_usFiltered);
//     _serial->println(F(" cm"));
//   }
// }





void Sensors::ZeroGyroHeading() {
	// Force an update to get the latest absolute yaw before zeroing
	getGyroHeading();
	_yawOffset = _currentAbsoluteYaw;
	if (_serial) _serial->println("Gyro Zeroed.");
}

float Sensors::getGyroHeading() {
	// Re-enable if reset
	if (_bno08x.wasReset()) {
		_bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
	}

	// Read new events if available
	if (_bno08x.getSensorEvent(&_sensorValue)) {
		if (_sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
			float qw = _sensorValue.un.gameRotationVector.real;
			float qx = _sensorValue.un.gameRotationVector.i;
			float qy = _sensorValue.un.gameRotationVector.j;
			float qz = _sensorValue.un.gameRotationVector.k;

			float yaw = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * 180.0f / PI;
			if (yaw < 0.0f) yaw += 360.0f;
			_currentAbsoluteYaw = yaw;
		}
	}

	// Calculate relative heading based on the zero offset
	float relativeYaw = _currentAbsoluteYaw - _yawOffset;
	if (relativeYaw < 0.0f) relativeYaw += 360.0f;
	if (relativeYaw >= 360.0f) relativeYaw -= 360.0f;

	return relativeYaw;
}

bool Sensors::isBatteryVoltageOK() {
	static unsigned long lastPrintMs = 0;
	const bool canPrint = (_serial != nullptr) && (millis() - lastPrintMs >= 1000);

	int raw_lipo = analogRead(BATTERY_PIN);
	int lipo_level_cal = (raw_lipo - 717);
	lipo_level_cal = lipo_level_cal * 100;
	lipo_level_cal = lipo_level_cal / 143;

	if (_serial != nullptr) {
		if (lipo_level_cal > 0 && lipo_level_cal < 160) {
			if (canPrint) {
				//_serial->print("Lipo level:");
				//_serial->print(lipo_level_cal);
				//_serial->println("%");
				lastPrintMs = millis();
			}
			_lowVoltageCounter = 0;
			return true;
		}

		if (lipo_level_cal < 0) {
			if (canPrint) {
				_serial->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
				lastPrintMs = millis();
			}
		} else if (lipo_level_cal > 160) {
			if (canPrint) {
				_serial->println("!Lipo is Overchanged!!!");
				lastPrintMs = millis();
			}
		} else {
			if (canPrint) {
				_serial->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
				_serial->print("Please Re-charge Lipo:");
				_serial->print(lipo_level_cal);
				_serial->println("%");
				lastPrintMs = millis();
			}
		}
	} else {
		if (lipo_level_cal > 0 && lipo_level_cal < 160) {
			_lowVoltageCounter = 0;
			return true;
		}
	}

	_lowVoltageCounter++;
	return _lowVoltageCounter <= 5;
}

void Sensors::printLongRangeIR2() {
	if (_serial == nullptr) {
		return;
	}
	_serial->print("Analog Range A4:");
	_serial->println(readLongRangeIR2());
}

void Sensors::printGyroZ() {
	if (_serial == nullptr) {
		return;
	}
	_serial->print("Yaw/Heading: ");
	_serial->println(getGyroHeading());
}


void Sensors::printUltrasonicRange() {
	if (_serial == nullptr) {
		return;
	}
	float cm = readUltrasonicCm();
	if (cm < 0.0f) {
		_serial->println("HC-SR04: Out of range");
		return;
	}

	_serial->print("HC-SR04:");
	_serial->print(cm);
	_serial->println("cm");
}
