#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"

#include <Servo.h>
#include <math.h>

extern Motor motors;
extern Sensors sensors;
extern Servo turret_motor;

// ═════════════════════════════════════════════════════════════════════════════
// PINS & CONSTANTS
// ═════════════════════════════════════════════════════════════════════════════
#define PT_CENTRE_PIN A3

// PT tuning:
// - PHOTO_THRESHOLD: PT value at or below which the fire is considered seen
//   and close enough to stop/extinguish.
// - PHOTO_CONFIRM_TICKS: number of consecutive low readings required to avoid
//   reacting to noise.
// - PT_FILTER_SAMPLES: moving-average window for smoothing the PT signal.
static constexpr int   PHOTO_THRESHOLD               = 35;
static constexpr uint8_t PHOTO_CONFIRM_TICKS         = 3;
static constexpr float  PHOTO_GAIN                   = 1.0f;

static constexpr int    TURRET_FWD                   = 90;
static constexpr int    TURRET_LEFT                  = 180;
static constexpr int    TURRET_RIGHT                 = 0;
static constexpr int    SWEEP_BINS                   = 360;
static constexpr uint16_t SWEEP_UNSAMPLED           = 0xFFFFu;
static constexpr int    PT_FILTER_SAMPLES           = 10;
static constexpr float  CENTROID_THRESH             = 0.7f;
static constexpr float  HEADING_OFFSET              = -2.0f;

static constexpr float   APPROACH_SPEED_CM_HINT      = 1.0f;
static constexpr float   SEARCH_SPEED_CM_HINT        = 0.5f;
static constexpr float   FRONT_OBSTACLE_CM           = 17.0f;
static constexpr float   FRONT_CLEAR_CM              = 23.0f;
static constexpr float   SIDE_WALL_CM                = 20.0f;
static constexpr float   US_BRAKE_CM                 = 15.0f;
static constexpr float   SIDE_IR_EFFECT_CM           = 10.0f;
static constexpr int     MOTION_BASE_SPEED          = 200;
static constexpr int     MOTION_MIN_SPEED            = 120;
static constexpr int     MOTION_MAX_SPEED            = 240;
static constexpr float   FRONT_IR_GAIN               = 0.5f;
static constexpr float   SIDE_IR_GAIN                = 1.0f;
static constexpr unsigned long FAN_DURATION_MS       = 1800;
static constexpr unsigned long RECENT_VISIBLE_MS     = 700;

// ═════════════════════════════════════════════════════════════════════════════
// STATE
// ═════════════════════════════════════════════════════════════════════════════
struct SensorSnapshot {
    float currentHeading;
    float frontLeftIR;
    float frontRightIR;
    float rearLeftIR;
    float rearRightIR;
    float ultrasonicCm;
    int   photo;
};

struct PhotoFilter {
    static constexpr uint8_t SIZE = 10;
    float    buf[SIZE];
    uint8_t  idx;
    float    sum;
    bool     seeded;

    PhotoFilter() : buf{0.0f}, idx(0), sum(0.0f), seeded(false) {}
};

static PhotoFilter photoFilter;

struct Hotspot {
    float angle;
    int   intensity;
    bool  valid;
    Hotspot() : angle(0.0f), intensity(1023), valid(false) {}
    Hotspot(float a, int i, bool v) : angle(a), intensity(i), valid(v) {}
};

static Hotspot hotspot;
static uint16_t sweepSamples[SWEEP_BINS];
static float    ptBuf[PT_FILTER_SAMPLES];
static uint8_t  ptBufIdx;
static float    ptBufSum;

enum FireMode {
    SEARCHING,
    APPROACHING,
    EXTINGUISHING,
    DONE
};

enum ServoMode {
    SERVO_SWEEPING,
    SERVO_HOLD
};

static FireMode  mode                 = SEARCHING;
static ServoMode servoMode            = SERVO_SWEEPING;
static uint8_t   firesExtinguished     = 0;
static bool      routineStarted       = false;
static bool      fireVisible          = false;
static bool      fireWasRecentlyVisible = false;
static float     fireHeading          = 0.0f;
static float     photoErrorDeg        = 0.0f;
static uint8_t   photoConfirmCount     = 0;
static unsigned long lastVisibleMs    = 0;
static unsigned long fanStartMs       = 0;
static bool      sweepInited          = false;
static bool      sweepStarted         = false;
static float     driveTargetHeading   = 0.0f;
static bool      driveTargetActive    = false;
static bool      fanActive            = false;

// ═════════════════════════════════════════════════════════════════════════════
// Forward declarations
// ═════════════════════════════════════════════════════════════════════════════
static void readAllSensors(SensorSnapshot &snapshot);
static float filteredPhoto(int rawPhoto);
static float normalizeAngle(float angle);
static float normalizeAngleError(float target, float current);
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid);
static void doSweep(const SensorSnapshot &snapshot);
static int   computeApproachSpeed(const SensorSnapshot &snapshot);
static void  computeObstaclePressure(const SensorSnapshot &snapshot, float &leftPressure, float &rightPressure);
static void activateFan(bool on);
static void startApproach(float heading);
static void stopAndHoldForward();

// ═════════════════════════════════════════════════════════════════════════════
void resetFireRoutine() {
    mode                   = SEARCHING;
    servoMode              = SERVO_SWEEPING;
    firesExtinguished      = 0;
    routineStarted         = false;
    fireVisible            = false;
    fireWasRecentlyVisible = false;
    fireHeading            = 0.0f;
    photoErrorDeg          = 0.0f;
    photoConfirmCount      = 0;
    lastVisibleMs          = 0;
    fanStartMs             = 0;
    motors.setSpeed(MOTION_BASE_SPEED);
    hotspot                = Hotspot(0.0f, 1023, false);
    for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;
    ptBufIdx               = 0;
    ptBufSum               = 0.0f;
    for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
        float v = (float)analogRead(PT_CENTRE_PIN);
        ptBuf[i] = v;
        ptBufSum += v;
        delay(5);
    }
    sweepInited            = false;
    sweepStarted           = false;
    driveTargetHeading     = 0.0f;
    driveTargetActive      = false;
    fanActive              = false;
    turret_motor.write(TURRET_FWD);
    motors.stop();
}

// ═════════════════════════════════════════════════════════════════════════════
STEP runFireRoutine() {
    SensorSnapshot snapshot;
    readAllSensors(snapshot);

    if (!routineStarted) {
        resetFireRoutine();
        routineStarted = true;
    }

    fireVisible = (snapshot.photo <= PHOTO_THRESHOLD);
    if (fireVisible) {
        photoConfirmCount++;
        lastVisibleMs = millis();
    } else {
        photoConfirmCount = 0;
    }

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 250) {
        lastLog = millis();
        Serial.print(F("[FIRE] mode=")); Serial.print((int)mode);
        Serial.print(F(" photo="));      Serial.print(snapshot.photo);
        Serial.print(F(" hdg="));        Serial.print(snapshot.currentHeading, 1);
        Serial.print(F(" fireH="));      Serial.print(fireHeading, 1);
        Serial.print(F(" visible="));    Serial.println(fireVisible);
    }

    if (mode == SEARCHING) {
        doSweep(snapshot);
        return CURRENT_STEP;
    }

    float frontClearance = min(snapshot.frontLeftIR, snapshot.frontRightIR);

    switch (mode) {
        case SEARCHING: {
            break;
        }

        case APPROACHING: {
            if (!fireVisible && !fireWasRecentlyVisible) {
                driveTargetActive = false;
                mode = SEARCHING;
                    motors.setSpeed(MOTION_BASE_SPEED);
                break;
            }

            if (fireVisible && photoConfirmCount >= PHOTO_CONFIRM_TICKS) {
                motors.stop();
                activateFan(true);
                fanStartMs = millis();
                mode = EXTINGUISHING;
                break;
            }

            float leftPressure = 0.0f;
            float rightPressure = 0.0f;
            computeObstaclePressure(snapshot, leftPressure, rightPressure);

            if (leftPressure > 0.0f || rightPressure > 0.0f) {
                int obstacleSpeed = computeApproachSpeed(snapshot);
                motors.setSpeed(obstacleSpeed);

                if (leftPressure > rightPressure + 0.05f) {
                    motors.strafeRight();
                    break;
                }

                if (rightPressure > leftPressure + 0.05f) {
                    motors.strafeLeft();
                    break;
                }

                motors.stop();
                break;
            }

            if (snapshot.ultrasonicCm > 0.5f && snapshot.ultrasonicCm < US_BRAKE_CM) {
                motors.stop();
                break;
            }

            motors.setSpeed(computeApproachSpeed(snapshot));
            if (!driveTargetActive || fabsf(normalizeAngleError(fireHeading, driveTargetHeading)) > 3.0f) {
                startApproach(fireHeading);
            }

            if (!motors.driveStraight(snapshot.currentHeading, snapshot.ultrasonicCm, frontClearance)) {
                startApproach(fireHeading);
            }
            break;
        }

        case EXTINGUISHING: {
            motors.stop();
            turret_motor.write(TURRET_FWD);

            if (millis() - fanStartMs >= FAN_DURATION_MS) {
                activateFan(false);
                firesExtinguished++;
                fireVisible = false;
                fireWasRecentlyVisible = false;
                photoConfirmCount = 0;
                driveTargetActive = false;

                if (firesExtinguished >= 2) {
                    mode = DONE;
                } else {
                    mode = SEARCHING;
                    motors.setSpeed(MOTION_BASE_SPEED);
                }
            }
            break;
        }

        case DONE: {
            stopAndHoldForward();
            return NEXT_STEP;
        }
    }

    return CURRENT_STEP;
}

// ═════════════════════════════════════════════════════════════════════════════
// Helpers
// ═════════════════════════════════════════════════════════════════════════════
static void readAllSensors(SensorSnapshot &snapshot) {
    snapshot.currentHeading = sensors.getGyroHeading();
    snapshot.frontLeftIR    = sensors.readIRFrontLeft();
    snapshot.frontRightIR   = sensors.readIRFrontRight();
    snapshot.rearLeftIR     = sensors.readIRRearLeft();
    snapshot.rearRightIR    = sensors.readIRRearRight();
    snapshot.ultrasonicCm   = sensors.pingNowCm();
    snapshot.photo          = (int)filteredPhoto(analogRead(PT_CENTRE_PIN));
}

static float filteredPhoto(int rawPhoto) {
    float value = (float)rawPhoto;
    if (!photoFilter.seeded) {
        photoFilter.seeded = true;
        photoFilter.sum = 0.0f;
        for (uint8_t i = 0; i < PhotoFilter::SIZE; i++) {
            photoFilter.buf[i] = value;
            photoFilter.sum += value;
        }
        return value;
    }

    photoFilter.sum -= photoFilter.buf[photoFilter.idx];
    photoFilter.buf[photoFilter.idx] = value;
    photoFilter.sum += value;
    photoFilter.idx = (photoFilter.idx + 1) % PhotoFilter::SIZE;
    return photoFilter.sum / (float)PhotoFilter::SIZE;
}

static float normalizeAngle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float normalizeAngleError(float target, float current) {
    float error = normalizeAngle(target) - normalizeAngle(current);
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid) {
    uint16_t globalMin = 1023;
    uint16_t globalMax = 0;
    int minBin = -1;
    int sampledCount = 0;

    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        sampledCount++;
        if (sweepSamples[i] < globalMin) {
            globalMin = sweepSamples[i];
            minBin = i;
        }
        if (sweepSamples[i] > globalMax) {
            globalMax = sweepSamples[i];
        }
    }

    outMin = globalMin;
    outMax = globalMax;
    if (sampledCount == 0) {
        outValid = false;
        return 0.0f;
    }

    float range = (float)(globalMax - globalMin);
    float sumCos = 0.0f;
    float sumSin = 0.0f;
    float bestNorm = 0.0f;
    int bestBin = -1;
    int binsAbove = 0;

    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        float norm = (range > 0.01f) ? (float)(globalMax - sweepSamples[i]) / range : 0.0f;
        if (norm > CENTROID_THRESH) {
            binsAbove++;
            float rad = (float)i * (float)PI / 180.0f;
            sumCos += norm * cosf(rad);
            sumSin += norm * sinf(rad);
        }
        if (norm > bestNorm) {
            bestNorm = norm;
            bestBin = i;
        }
    }

    Serial.print(F("[CENTROID] n=")); Serial.print(sampledCount);
    Serial.print(F(" peak="));        Serial.print(minBin);
    Serial.print(F(" above="));       Serial.print(binsAbove);
    Serial.print(F(" norm="));        Serial.println(bestNorm, 3);

    if (sumCos != 0.0f || sumSin != 0.0f) {
        float deg = atan2f(sumSin, sumCos) * 180.0f / (float)PI;
        if (deg < 0.0f) deg += 360.0f;
        outValid = true;
        return deg;
    }

    if (bestBin >= 0) {
        outValid = true;
        return (float)bestBin;
    }

    outValid = false;
    return 0.0f;
}

static void doSweep(const SensorSnapshot &snapshot) {
    if (!sweepInited) {
        Serial.println(F("[SWEEP] Init — full 360"));
        turret_motor.write(TURRET_FWD);
        hotspot = Hotspot(0.0f, 1023, false);
        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;
        ptBufIdx = 0;
        ptBufSum = 0.0f;
        for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
            float v = (float)analogRead(PT_CENTRE_PIN);
            ptBuf[i] = v;
            ptBufSum += v;
            delay(5);
        }
        sweepStarted = false;
        sweepInited = true;
        sensors.ZeroGyroHeading();
        motors.setSpeed(MOTION_BASE_SPEED);
        motors.rotateCounterClockwise();
        return;
    }

    float heading = snapshot.currentHeading;
    if (!sweepStarted && heading > 10.0f && heading < 350.0f) {
        sweepStarted = true;
        Serial.println(F("[SWEEP] Rotating"));
    }

    float newVal = (float)analogRead(PT_CENTRE_PIN);
    ptBufSum -= ptBuf[ptBufIdx];
    ptBuf[ptBufIdx] = newVal;
    ptBufSum += newVal;
    ptBufIdx = (ptBufIdx + 1) % PT_FILTER_SAMPLES;
    float avg = ptBufSum / (float)PT_FILTER_SAMPLES;

    int bin = (int)heading;
    if (bin < 0) bin = 0;
    if (bin >= SWEEP_BINS) bin = SWEEP_BINS - 1;
    sweepSamples[bin] = (uint16_t)avg;

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print(F("[SWEEP] hdg=")); Serial.print(heading);
        Serial.print(F(" avg="));        Serial.println(avg, 1);
    }

    if (sweepStarted && heading >= 358.0f) {
        motors.stop();
        sweepInited = false;
        sweepStarted = false;

        uint16_t gMin = 0, gMax = 0;
        bool valid = false;
        float rawAngle = computeHotspotAngle(gMin, gMax, valid);
        float target = rawAngle + HEADING_OFFSET;
        while (target >= 360.0f) target -= 360.0f;
        while (target < 0.0f) target += 360.0f;

        hotspot = Hotspot(target, (int)gMin, valid);
        fireHeading = target;
        fireVisible = true;
        fireWasRecentlyVisible = true;
        lastVisibleMs = millis();

        Serial.print(F("[SWEEP] hotspot=")); Serial.print(target);
        Serial.print(F(" valid="));          Serial.println(valid);

        if (!valid) {
            Serial.println(F("[SWEEP] No hotspot confidence — using best estimate"));
        }

        startApproach(target);
        mode = APPROACHING;
    }
}

static void activateFan(bool on) {
    fanActive = on;
    if (on) {
        
        Serial.println(F("[FAN] on"));
    } else {
        Serial.println(F("[FAN] off"));
    }
}

static void startApproach(float heading) {
    driveTargetHeading = normalizeAngle(heading);
    driveTargetActive = true;
    motors.SetDriveStraightTarget(FORWARD, driveTargetHeading, 0.0f, 0.0f);
}

static void computeObstaclePressure(const SensorSnapshot &snapshot, float &leftPressure, float &rightPressure) {
    leftPressure = 0.0f;
    rightPressure = 0.0f;

    if (snapshot.frontLeftIR > 0.5f && snapshot.frontLeftIR < FRONT_OBSTACLE_CM) {
        leftPressure += (FRONT_OBSTACLE_CM - snapshot.frontLeftIR) * FRONT_IR_GAIN;
    }
    if (snapshot.frontRightIR > 0.5f && snapshot.frontRightIR < FRONT_OBSTACLE_CM) {
        rightPressure += (FRONT_OBSTACLE_CM - snapshot.frontRightIR) * FRONT_IR_GAIN;
    }

    if (snapshot.rearLeftIR > 0.5f && snapshot.rearLeftIR < SIDE_IR_EFFECT_CM) {
        leftPressure += (SIDE_IR_EFFECT_CM - snapshot.rearLeftIR) * SIDE_IR_GAIN;
    }
    if (snapshot.rearRightIR > 0.5f && snapshot.rearRightIR < SIDE_IR_EFFECT_CM) {
        rightPressure += (SIDE_IR_EFFECT_CM - snapshot.rearRightIR) * SIDE_IR_GAIN;
    }
}

static int computeApproachSpeed(const SensorSnapshot &snapshot) {
    float speed = (float)MOTION_BASE_SPEED;

    if (snapshot.frontLeftIR > 0.5f && snapshot.frontLeftIR < FRONT_OBSTACLE_CM) {
        speed -= (FRONT_OBSTACLE_CM - snapshot.frontLeftIR) * FRONT_IR_GAIN;
    }
    if (snapshot.frontRightIR > 0.5f && snapshot.frontRightIR < FRONT_OBSTACLE_CM) {
        speed -= (FRONT_OBSTACLE_CM - snapshot.frontRightIR) * FRONT_IR_GAIN;
    }

    if (snapshot.rearLeftIR > 0.5f && snapshot.rearLeftIR < SIDE_IR_EFFECT_CM) {
        speed -= (SIDE_IR_EFFECT_CM - snapshot.rearLeftIR) * SIDE_IR_GAIN;
    }
    if (snapshot.rearRightIR > 0.5f && snapshot.rearRightIR < SIDE_IR_EFFECT_CM) {
        speed -= (SIDE_IR_EFFECT_CM - snapshot.rearRightIR) * SIDE_IR_GAIN;
    }

    return constrain((int)speed, MOTION_MIN_SPEED, MOTION_MAX_SPEED);
}

static void stopAndHoldForward() {
    activateFan(false);
    motors.stop();
    turret_motor.write(TURRET_FWD);
}

// ═════════════════════════════════════════════════════════════════════════════
// Blocking clearance scan (utility — not used by runFireRoutine)
// ═════════════════════════════════════════════════════════════════════════════
ObstacleClearanceResult checkForwardClearance() {
    static constexpr float CLEARANCE_DEPTH_MM  = 400.0f;
    static constexpr float SENSOR_OFFSET_MM    = 50.0f;
    static constexpr float CLEARANCE_HALF_W_MM = 125.0f;
    static constexpr int   SCAN_START_DEG      = 50;
    static constexpr int   SCAN_END_DEG        = 130;
    static constexpr int   SCAN_STEP_DEG       = 6;
    static constexpr uint16_t SERVO_SETTLE_MS  = 13;
    static constexpr float DEG2RAD             = 3.14159265f / 180.0f;

    ObstacleClearanceResult result;
    result.isClear = true;
    result.closestObstacleOffsetMm = 0.0f;
    result.closestObstacleDistanceMm = -1.0f;

    float closestMm = 1.0e9f;
    for (int deg = SCAN_START_DEG; deg <= SCAN_END_DEG; deg += SCAN_STEP_DEG) {
        turret_motor.write(deg);
        delay(SERVO_SETTLE_MS);
        float cm = sensors.pingNowCm();
        if (cm <= 0.0f) continue;

        float mm  = cm * 10.0f;
        float brg = ((float)deg - 90.0f) * DEG2RAD;
        float fwd = SENSOR_OFFSET_MM + mm * cosf(brg);
        float lat = mm * sinf(brg);
        if (fwd >= 0.0f && fwd <= CLEARANCE_DEPTH_MM &&
            fabsf(lat) <= CLEARANCE_HALF_W_MM && mm < closestMm) {
            closestMm = mm;
            result.isClear = false;
            result.closestObstacleOffsetMm = lat;
            result.closestObstacleDistanceMm = mm;
        }
    }

    turret_motor.write(TURRET_FWD);
    delay(SERVO_SETTLE_MS);
    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Non-blocking clearance scanner (centre arc 50–130°) — kept for compatibility
// ═════════════════════════════════════════════════════════════════════════════
static int           nb_scanAngleDeg    = 50;
static unsigned long nb_lastMs          = 0;
static bool          nb_scanning        = false;
static float         nb_closestDistMm   = 1.0e9f;
static float         nb_closestOffsetMm = 0.0f;

void clearanceScanStart() {
    nb_scanAngleDeg    = 50;
    nb_scanning        = true;
    nb_lastMs          = 0;
    nb_closestDistMm   = 1.0e9f;
    nb_closestOffsetMm = 0.0f;
    turret_motor.write(nb_scanAngleDeg);
}

bool clearanceScanStep(ObstacleClearanceResult &out) {
    static constexpr float CLEARANCE_DEPTH_MM  = 400.0f;
    static constexpr float SENSOR_OFFSET_MM    = 50.0f;
    static constexpr float CLEARANCE_HALF_W_MM = 125.0f;
    static constexpr int   SCAN_END_DEG        = 130;
    static constexpr int   SCAN_STEP_DEG       = 6;
    static constexpr uint16_t SERVO_SETTLE_MS  = 13;
    static constexpr float DEG2RAD             = 3.14159265f / 180.0f;

    if (!nb_scanning) {
        out.isClear = true;
        out.closestObstacleOffsetMm = 0.0f;
        out.closestObstacleDistanceMm = -1.0f;
        return true;
    }

    unsigned long now = millis();
    if (nb_lastMs == 0) {
        turret_motor.write(nb_scanAngleDeg);
        nb_lastMs = now;
        return false;
    }

    if (now - nb_lastMs < SERVO_SETTLE_MS) {
        return false;
    }

    float cm = sensors.pingNowCm();
    if (cm > 0.0f) {
        float mm  = cm * 10.0f;
        float brg = ((float)nb_scanAngleDeg - 90.0f) * DEG2RAD;
        float fwd = SENSOR_OFFSET_MM + mm * cosf(brg);
        float lat = mm * sinf(brg);
        if (fwd >= 0.0f && fwd <= CLEARANCE_DEPTH_MM &&
            fabsf(lat) <= CLEARANCE_HALF_W_MM && mm < nb_closestDistMm) {
            nb_closestDistMm = mm;
            nb_closestOffsetMm = lat;
        }
    }

    nb_scanAngleDeg += SCAN_STEP_DEG;
    nb_lastMs = 0;

    if (nb_scanAngleDeg > SCAN_END_DEG) {
        nb_scanning = false;
        turret_motor.write(TURRET_FWD);
        if (nb_closestDistMm < 1.0e8f) {
            out.isClear = false;
            out.closestObstacleOffsetMm = nb_closestOffsetMm;
            out.closestObstacleDistanceMm = nb_closestDistMm;
        } else {
            out.isClear = true;
            out.closestObstacleOffsetMm = 0.0f;
            out.closestObstacleDistanceMm = -1.0f;
        }
        return true;
    }

    turret_motor.write(nb_scanAngleDeg);
    return false;
}
