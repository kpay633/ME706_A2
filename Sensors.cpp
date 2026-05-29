#include "Sensors.h"
#include <Wire.h>

#define TRIG_PIN 48
#define ECHO_PIN 49

Sensors::Sensors()
    : _serial(&Serial), _bno08x(-1), _lowVoltageCounter(0),
      _currentAbsoluteYaw(0.0f), _yawOffset(0.0f),
      _usLastPollMs(0), _usBufIdx(0), _usBufFull(false), _usFiltered(0)
{
    memset(_usBuf, 0, sizeof(_usBuf));
}

void Sensors::setSerial(HardwareSerial *serialCom) {
    if (serialCom != nullptr) _serial = serialCom;
}

bool Sensors::initialise() {
    pinMode(BATTERY_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);

    // NOTE: Coefficients below are placeholders matched to previously used
    // Sharp models. If sensors have been swapped, re-calibrate these values.
    // Short-range (GP2Y0A41SK0F approx): front-left A10, front-right A11
    _irShort1.setPowerFitCoeffs(38329.8809f, -1.0894f, 86, 438);
    _irShort2.setPowerFitCoeffs(38329.8809f, -1.0894f, 86, 438);
    // Long-range (GP2Y0A02YK0F approx): rear-left A8, rear-right A9
    _irLong1.setPowerFitCoeffs(179006.0322f, -1.2062f, 88, 488);
    _irLong2.setPowerFitCoeffs(148265.9356f, -1.1997f, 81, 464);

    Wire.begin();
    delay(100);

    bool imuReady = false;
    for (uint8_t attempt = 0; attempt < 5; ++attempt) {
        if (_bno08x.begin_I2C()) { imuReady = true; break; }
        if (_serial) _serial->println(F("BNO085 not found, retrying..."));
        delay(200);
    }
    if (!imuReady) {
        if (_serial) _serial->println(F("IMU init failed"));
        return false;
    }
    if (!_bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000)) {
        if (_serial) _serial->println(F("BNO085 rotation report failed"));
        return false;
    }
    if (!_bno08x.enableReport(SH2_GYROSCOPE_UNCALIBRATED, 10000)) {
        if (_serial) _serial->println(F("BNO085 gyro report failed"));
        return false;
    }

    _lowVoltageCounter = 0;
    if (_serial) _serial->println(F("Sensors initialised"));
    return true;
}

// ── IR named accessors ────────────────────────────────────────────────────────
// SharpDistSensor::getDist() returns mm; divide by 10 for cm.
// The median window (size 5 in constructor) handles noise internally —
// no additional averaging needed at this layer.

float Sensors::readIRFrontLeft()  { return _irShort1.getDist() / 10.0f; }
float Sensors::readIRFrontRight() { return _irShort2.getDist() / 10.0f; }
float Sensors::readIRRearLeft()   { return _irLong1.getDist()  / 10.0f; }
float Sensors::readIRRearRight()  { return _irLong2.getDist()  / 10.0f; }

// ── Legacy raw accessors ──────────────────────────────────────────────────────
uint16_t Sensors::readShortRangeIR1() { return _irShort1.getDist(); }
uint16_t Sensors::readShortRangeIR2() { return _irShort2.getDist(); }
uint16_t Sensors::readLongRangeIR1()  { return _irLong1.getDist();  }
uint16_t Sensors::readLongRangeIR2()  { return _irLong2.getDist();  }

// ── Ultrasonic ────────────────────────────────────────────────────────────────

float Sensors::readUltrasonicCm() {
    unsigned long now = millis();
    if (now - _usLastPollMs >= 35) {
        _usLastPollMs = now;
        unsigned long pingTime = _sonar.ping();
        float rawCm = pingTime / 57.0f;

        if (rawCm >= 3.0f) {
            uint8_t count = _usBufFull ? US_FILTER_SIZE : _usBufIdx;
            if (_usStrictFilter) {
                if (count > 0 && fabsf(rawCm - _usFiltered) > 10.0f)
                    return _usFiltered;
            }
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
                    while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
                    tmp[j+1] = key;
                }
                _usFiltered = tmp[count / 2] / 10.0f;
            }
        }
    }
    return _usFiltered;
}

float Sensors::pingNowCm() {
    unsigned long pingTime = _sonar.ping();
    float rawCm = pingTime / 57.0f;
    return (rawCm >= 3.0f) ? rawCm : 0.0f;
}

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
        _serial->print(F("US warmed: "));
        _serial->print(_usFiltered, 1);
        _serial->println(F(" cm"));
    }
}

// ── IMU ───────────────────────────────────────────────────────────────────────

void Sensors::ZeroGyroHeading() {
    getGyroHeading();
    _yawOffset = _currentAbsoluteYaw;
    if (_serial) _serial->println(F("Gyro zeroed"));
}

float Sensors::getGyroHeading() {
    if (_bno08x.wasReset())
        _bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);

    if (_bno08x.getSensorEvent(&_sensorValue)) {
        if (_sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float qw = _sensorValue.un.gameRotationVector.real;
            float qx = _sensorValue.un.gameRotationVector.i;
            float qy = _sensorValue.un.gameRotationVector.j;
            float qz = _sensorValue.un.gameRotationVector.k;
            float yaw = atan2f(2.0f*(qw*qz + qx*qy),
                               1.0f - 2.0f*(qy*qy + qz*qz)) * 180.0f / PI;
            if (yaw < 0.0f) yaw += 360.0f;
            _currentAbsoluteYaw = yaw;
        }
    }
    float rel = _currentAbsoluteYaw - _yawOffset;
    if (rel <    0.0f) rel += 360.0f;
    if (rel >= 360.0f) rel -= 360.0f;
    return rel;
}

// ── Battery ───────────────────────────────────────────────────────────────────

bool Sensors::isBatteryVoltageOK() {
    static unsigned long lastPrintMs = 0;
    const bool canPrint = (_serial != nullptr) && (millis() - lastPrintMs >= 1000);
    int raw = analogRead(BATTERY_PIN);
    int cal = ((raw - 717) * 100) / 143;

    if (_serial != nullptr) {
        if (cal > 0 && cal < 160) {
            if (canPrint) lastPrintMs = millis();
            _lowVoltageCounter = 0;
            return true;
        }
        if (canPrint) {
            if      (cal < 0)   _serial->println(F("Lipo disconnected or off"));
            else if (cal > 160) _serial->println(F("Lipo overcharged"));
            else {
                _serial->println(F("Lipo voltage too low"));
                _serial->print(F("Level: ")); _serial->print(cal); _serial->println(F("%"));
            }
            lastPrintMs = millis();
        }
    } else {
        if (cal > 0 && cal < 160) { _lowVoltageCounter = 0; return true; }
    }
    _lowVoltageCounter++;
    return _lowVoltageCounter <= 5;
}

// ── Debug prints ─────────────────────────────────────────────────────────────

void Sensors::printLongRangeIR2() {
    if (!_serial) return;
    _serial->print(F("Rear-right IR: "));
    _serial->print(readIRRearRight(), 1);
    _serial->println(F(" cm"));
}

void Sensors::printGyroZ() {
    if (!_serial) return;
    _serial->print(F("Heading: "));
    _serial->println(getGyroHeading());
}

void Sensors::printUltrasonicRange() {
    if (!_serial) return;
    float cm = readUltrasonicCm();
    _serial->print(F("US: "));
    _serial->print(cm);
    _serial->println(F(" cm"));
}
