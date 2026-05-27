#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"
#include <Servo.h>

extern Motor   motors;
extern Sensors sensors;
extern Servo   turret_motor;

// ─── PT sensor pins ───────────────────────────────────────────────────────────
#define PT_LEFT_PIN    A2
#define PT_CENTRE_PIN  A3
#define PT_RIGHT_PIN   A4

// ─── Sweep / turn / approach tuning ──────────────────────────────────────────
#define SWEEP_SPEED          60
#define TURN_SPEED           60
#define TURN_TOLERANCE        3.0f
#define APPROACH_SPEED       70
#define APPROACH_IR_STOP_CM  10.0f   // stop when front IR reads ≤ this many cm

#define SWEEP_BINS           360
#define SWEEP_UNSAMPLED      0xFFFFu
#define PT_FILTER_SAMPLES    10
#define CENTROID_THRESH       0.7f
#define TURRET_LOCK_DEG      90

// PT mounting offset (deg, CCW positive).
//   With the servo locked at TURRET_LOCK_DEG, this is the angle the PT
//   physically faces relative to the robot's FRONT.
//     PT points forward  → 0    (normal case)
//     PT points right    → 90
//     PT points rear     → 180
//     PT points left     → 270
#define HEADING_OFFSET  -2.0f

// ─── Obstacle-avoidance clearance geometry ───────────────────────────────────
static constexpr float    CLEARANCE_WIDTH_MM       = 250.0f;
static constexpr float    CLEARANCE_DEPTH_MM       = 400.0f;
static constexpr float    SENSOR_OFFSET_MM         = 50.0f;
static constexpr float    CLEARANCE_HALF_WIDTH_MM  = CLEARANCE_WIDTH_MM * 0.5f;
static constexpr int      SCAN_START_DEG           = 30;
static constexpr int      SCAN_END_DEG             = 150;
static constexpr int      SCAN_STEP_DEG            = 6;
static constexpr uint16_t SERVO_SETTLE_MS          = 13;
static constexpr float    DEG_TO_RAD_SCALE         = 3.14159265f / 180.0f;

// Thresholds used by the approach obstacle-avoidance logic
static constexpr float    OBS_DETECT_THRESH_MM     = 300.0f;
static constexpr float    OBS_AVOID_THRESH_MM      = 200.0f;
static constexpr unsigned long STRAFE_MIN_MS       = 500;
static constexpr unsigned long STRAFE_MAX_MS       = 2400;

// ─── Hotspot & sweep state ───────────────────────────────────────────────────
struct Hotspot { float angle; int intensity; bool valid = false; };
static Hotspot hotspot;

static uint16_t sweepSamples[SWEEP_BINS];
static float    ptBuf[PT_FILTER_SAMPLES];
static uint8_t  ptBufIdx;
static float    ptBufSum;
static uint8_t  ptBufCount;

// ─── Fire FSM ────────────────────────────────────────────────────────────────
enum FireSubState {
    FS_SWEEP, FS_TURN, FS_APPROACH,
    FS_AVOID, FS_ALIGN, FS_EXTINGUISH, FS_DONE
};
static FireSubState subState     = FS_SWEEP;
static int          fireIndex    = 0;
static bool         sweepInited  = false;
static bool         sweepStarted = false;
static bool         targetSet    = false;

// ─── Approach sub-state (obstacle avoidance inline) ──────────────────────────
enum ApproachSubState { AP_DRIVE, AP_SCANNING, AP_STRAFING };
static ApproachSubState approachSub = AP_DRIVE;
static unsigned long    strafeUntilMs = 0;

static void doSweep();
static void doTurn();
static void doApproach();
static void doAvoid();
static void doAlign();
static void doExtinguish();
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid);

// ─────────────────────────────────────────────────────────────────────────────
void resetFireRoutine() {
    subState      = FS_SWEEP;
    fireIndex     = 0;
    sweepInited   = false;
    sweepStarted  = false;   // BUG FIX: must be reset or next run skips sweep
    targetSet     = false;
    approachSub   = AP_DRIVE;
}

// ─────────────────────────────────────────────────────────────────────────────
STEP runFireRoutine() {
    static unsigned long lastStateprint = 0;
    if (millis() - lastStateprint > 500) {
        lastStateprint = millis();
        Serial.print(F("[FSM] subState = "));
        switch (subState) {
            case FS_SWEEP:      Serial.println(F("FS_SWEEP"));      break;
            case FS_TURN:       Serial.println(F("FS_TURN"));       break;
            case FS_APPROACH:   Serial.println(F("FS_APPROACH"));   break;
            case FS_AVOID:      Serial.println(F("FS_AVOID"));      break;
            case FS_ALIGN:      Serial.println(F("FS_ALIGN"));      break;
            case FS_EXTINGUISH: Serial.println(F("FS_EXTINGUISH")); break;
            case FS_DONE:       Serial.println(F("FS_DONE"));       break;
        }
    }

    switch (subState) {
        case FS_SWEEP:      doSweep();      break;
        case FS_TURN:       doTurn();       break;
        case FS_APPROACH:   doApproach();   break;
        case FS_AVOID:      doAvoid();      break;
        case FS_ALIGN:      doAlign();      break;
        case FS_EXTINGUISH: doExtinguish(); break;
        case FS_DONE:
            Serial.println(F("[FSM] FS_DONE reached — returning NEXT_STEP"));
            return NEXT_STEP;
    }
    return CURRENT_STEP;
}

// ─────────────────────────────────────────────────────────────────────────────
// doSweep — lock turret straight ahead, rotate robot 360°, locate fire via
//           normalised circular-mean centroid on the centre PT signal.
// ─────────────────────────────────────────────────────────────────────────────
static void doSweep() {
    if (!sweepInited) {
        Serial.println(F("[SWEEP] Init — locking turret @ 90°, zeroing gyro, starting rotation"));

        turret_motor.write(TURRET_LOCK_DEG);

        hotspot.angle     = 0.0f;
        hotspot.intensity = 1023;
        hotspot.valid     = false;

        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;

        ptBufIdx   = 0;
        ptBufSum   = 0.0f;
        ptBufCount = 0;
        for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
            float v = (float)analogRead(PT_CENTRE_PIN);
            ptBuf[i] = v;
            ptBufSum += v;
            delay(5);
        }
        ptBufCount = PT_FILTER_SAMPLES;

        sweepStarted = false;
        sweepInited  = true;
        sensors.ZeroGyroHeading();
        motors.rotateCounterClockwise();
        return;
    }

    float heading = sensors.getGyroHeading();

    if (!sweepStarted && heading > 10.0f && heading < 350.0f) {
        sweepStarted = true;
        Serial.println(F("[SWEEP] Rotation confirmed, now watching for 358°"));
    }

    // Update rolling-average PT reading
    float newVal = (float)analogRead(PT_CENTRE_PIN);
    ptBufSum -= ptBuf[ptBufIdx];
    ptBuf[ptBufIdx] = newVal;
    ptBufSum += newVal;
    ptBufIdx = (ptBufIdx + 1) % PT_FILTER_SAMPLES;
    float avg = ptBufSum / (float)PT_FILTER_SAMPLES;

    int bin = (int)heading;
    if (bin < 0)           bin = 0;
    if (bin >= SWEEP_BINS) bin = SWEEP_BINS - 1;
    sweepSamples[bin] = (uint16_t)avg;

    static unsigned long lastSweepPrint = 0;
    if (millis() - lastSweepPrint > 200) {
        lastSweepPrint = millis();
        Serial.print(F("[SWEEP] heading="));
        Serial.print(heading);
        Serial.print(F("  ptAvg="));
        Serial.println(avg, 1);
    }

    if (sweepStarted && heading >= 358.0f) {
        motors.stop();
        sweepInited  = false;
        sweepStarted = false;

        uint16_t gMin = 0, gMax = 0;
        bool valid = false;
        float rawAngle = computeHotspotAngle(gMin, gMax, valid);

        float targetAngle = rawAngle + HEADING_OFFSET;
        while (targetAngle >= 360.0f) targetAngle -= 360.0f;
        while (targetAngle <    0.0f) targetAngle += 360.0f;

        hotspot.angle     = targetAngle;
        hotspot.intensity = (int)gMin;
        hotspot.valid     = valid;

        Serial.println(F("[SWEEP] Complete."));
        Serial.print(F("[SWEEP] globalMin=")); Serial.print(gMin);
        Serial.print(F("  globalMax="));      Serial.println(gMax);
        Serial.print(F("[SWEEP] PT-brightest @ ")); Serial.print(rawAngle);
        Serial.print(F(" + offset "));              Serial.print(HEADING_OFFSET);
        Serial.print(F(" → target heading "));      Serial.println(targetAngle);
        Serial.print(F("[SWEEP] Hotspot valid="));  Serial.println(hotspot.valid);
        subState = FS_TURN;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Circular-mean centroid over sweep bins above CENTROID_THRESH.
// Falls back to single-brightest bin if no region passes threshold.
// Wrap-safe: uses atan2(Σnorm·sin, Σnorm·cos).
// ─────────────────────────────────────────────────────────────────────────────
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid) {
    uint16_t globalMin = 1023, globalMax = 0;
    int minBin = -1, sampledCount = 0;

    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        sampledCount++;
        if (sweepSamples[i] < globalMin) { globalMin = sweepSamples[i]; minBin = i; }
        if (sweepSamples[i] > globalMax) { globalMax = sweepSamples[i]; }
    }

    outMin = globalMin; outMax = globalMax;
    if (sampledCount == 0) { outValid = false; return 0.0f; }

    float range    = (float)(globalMax - globalMin);
    float sumCos   = 0.0f, sumSin = 0.0f;
    float bestNorm = 0.0f;
    int   bestBin  = -1, binsAboveThresh = 0;

    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        float norm = (range > 0.01f)
                     ? (float)(globalMax - sweepSamples[i]) / range
                     : 0.0f;
        if (norm > CENTROID_THRESH) {
            binsAboveThresh++;
            float rad = (float)i * (float)PI / 180.0f;
            sumCos += norm * cosf(rad);
            sumSin += norm * sinf(rad);
        }
        if (norm > bestNorm) { bestNorm = norm; bestBin = i; }
    }

    Serial.print(F("[CENTROID] samples=")); Serial.print(sampledCount);
    Serial.print(F("  brightestBin="));    Serial.print(minBin);
    Serial.print(F(" (raw="));             Serial.print(globalMin); Serial.print(F(")"));
    Serial.print(F("  binsAboveThresh=")); Serial.print(binsAboveThresh);
    Serial.print(F("  peakNormBin="));     Serial.print(bestBin);
    Serial.print(F(" (norm="));            Serial.print(bestNorm, 3); Serial.println(F(")"));

    if (sumCos != 0.0f || sumSin != 0.0f) {
        float deg = atan2f(sumSin, sumCos) * 180.0f / (float)PI;
        if (deg < 0.0f) deg += 360.0f;
        outValid = true;
        return deg;
    }
    if (bestBin >= 0) { outValid = true; return (float)bestBin; }
    outValid = false;
    return 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// doTurn — PID turn to hotspot heading.
// ─────────────────────────────────────────────────────────────────────────────
static void doTurn() {
    if (!hotspot.valid) {
        Serial.println(F("[TURN] No valid hotspot — skipping to DONE"));
        targetSet = false;
        subState  = FS_DONE;
        return;
    }

    if (!targetSet) {
        Serial.print(F("[TURN] Setting PID target = "));
        Serial.println(hotspot.angle);
        motors.SetTurnTarget(hotspot.angle);
        targetSet = true;
    }

    float current = sensors.getGyroHeading();

    static unsigned long lastTurnPrint = 0;
    if (millis() - lastTurnPrint > 200) {
        lastTurnPrint = millis();
        Serial.print(F("[TURN] target="));  Serial.print(hotspot.angle);
        Serial.print(F("  current="));      Serial.println(current);
    }

    if (motors.isTurnComplete(current)) {
        Serial.println(F("[TURN] Complete — arming driveStraight → APPROACH"));
        targetSet = false;
        approachSub = AP_DRIVE;
        // Arm gyro-hold drive; IR front stop at APPROACH_IR_STOP_CM.
        motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, APPROACH_IR_STOP_CM);
        subState = FS_APPROACH;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// doApproach — gyro-locked straight drive with inline obstacle avoidance.
//
//  AP_DRIVE    → drives straight; kicks off a non-blocking clearance scan
//  AP_SCANNING → keeps driving while scan accumulates; on complete:
//                  blocked → AP_STRAFING
//                  clear   → back to AP_DRIVE (starts next scan)
//  AP_STRAFING → strafe for a timed duration, then re-arm driveStraight
//                and return to AP_DRIVE
//
// Front IR stop terminates the whole approach at any sub-state.
// ─────────────────────────────────────────────────────────────────────────────
static void doApproach() {
    float heading = sensors.getGyroHeading();
    float irFront = (float)sensors.readLongRangeIR1();

    // Global exit condition — fire reached
    if (irFront > 5.0f && irFront <= APPROACH_IR_STOP_CM) {
        motors.stop();
        Serial.println(F("[APPROACH] IR stop — moving to DONE"));
        approachSub = AP_DRIVE;
        subState    = FS_DONE;
        return;
    }

    static unsigned long lastApproachPrint = 0;
    if (millis() - lastApproachPrint > 200) {
        lastApproachPrint = millis();
        Serial.print(F("[APPROACH] sub="));
        Serial.print(approachSub);
        Serial.print(F("  heading="));  Serial.print(heading);
        Serial.print(F("  irFront="));  Serial.print(irFront);
        Serial.print(F(" cm  ptC="));   Serial.println(analogRead(PT_CENTRE_PIN));
    }

    switch (approachSub) {

        case AP_DRIVE:
            // (Re-)arm driveStraight if needed, then kick off a fresh scan
            if (!motors.driveStraight(heading, 0.0f, irFront)) {
                // driveStraight returned false — target wasn't set; re-arm
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, APPROACH_IR_STOP_CM);
            }
            clearanceScanStart();
            approachSub = AP_SCANNING;
            break;

        case AP_SCANNING: {
            // Keep driving while the scan step accumulates
            motors.driveStraight(heading, 0.0f, irFront);

            ObstacleClearanceResult scanResult;
            if (clearanceScanStep(scanResult)) {
                // Scan complete — evaluate
                if (!scanResult.isClear &&
                    fabsf(scanResult.closestObstacleOffsetMm) < OBS_AVOID_THRESH_MM &&
                    scanResult.closestObstacleDistanceMm      < OBS_DETECT_THRESH_MM)
                {
                    motors.stop();

                    if (scanResult.closestObstacleOffsetMm > 0.0f) {
                        motors.strafeRight();
                        Serial.print(F("[APPROACH] OBS → strafe RIGHT  offset="));
                    } else {
                        motors.strafeLeft();
                        Serial.print(F("[APPROACH] OBS → strafe LEFT   offset="));
                    }
                    Serial.print(scanResult.closestObstacleOffsetMm);
                    Serial.print(F("  dist="));
                    Serial.println(scanResult.closestObstacleDistanceMm);

                    float ratio = fabsf(scanResult.closestObstacleOffsetMm) / OBS_AVOID_THRESH_MM;
                    if (ratio > 1.0f) ratio = 1.0f;
                    unsigned long ms = (unsigned long)(STRAFE_MAX_MS - (STRAFE_MAX_MS - STRAFE_MIN_MS) * ratio);
                    if (ms < STRAFE_MIN_MS) ms = STRAFE_MIN_MS;

                    strafeUntilMs = millis() + ms;
                    approachSub   = AP_STRAFING;
                } else {
                    // Path clear — go back to driving and start the next scan
                    approachSub = AP_DRIVE;
                }
            }
            break;
        }

        case AP_STRAFING:
            if (millis() >= strafeUntilMs) {
                motors.stop();
                Serial.println(F("[APPROACH] Strafe done — re-arming drive"));
                // Re-arm driveStraight on the same hotspot heading after the strafe
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, APPROACH_IR_STOP_CM);
                approachSub = AP_DRIVE;
            }
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Stub states — implement as needed
// ─────────────────────────────────────────────────────────────────────────────
static void doAvoid()      { Serial.println(F("[AVOID] stub")); }
static void doAlign()      { Serial.println(F("[ALIGN] stub")); }
static void doExtinguish() { Serial.println(F("[EXTINGUISH] stub")); }

// ─────────────────────────────────────────────────────────────────────────────
// Blocking clearance scan (convenience helper — not used by doApproach)
// ─────────────────────────────────────────────────────────────────────────────
ObstacleClearanceResult checkForwardClearance() {
    ObstacleClearanceResult result;
    result.isClear                   = true;
    result.closestObstacleOffsetMm   = 0.0f;
    result.closestObstacleDistanceMm = -1.0f;

    float closestDistanceMm = 1.0e9f;

    for (int servoAngleDeg = SCAN_START_DEG; servoAngleDeg <= SCAN_END_DEG; servoAngleDeg += SCAN_STEP_DEG) {
        turret_motor.write(servoAngleDeg);
        delay(SERVO_SETTLE_MS);

        float distanceCm = sensors.pingNowCm();
        if (distanceCm <= 0.0f) continue;

        float distanceMm  = distanceCm * 10.0f;
        float bearingDeg  = (float)servoAngleDeg - 90.0f;
        float bearingRad  = bearingDeg * DEG_TO_RAD_SCALE;
        float fwdMm       = SENSOR_OFFSET_MM + distanceMm * cosf(bearingRad);
        float lateralMm   = distanceMm * sinf(bearingRad);

        bool withinBox = (fwdMm >= 0.0f && fwdMm <= CLEARANCE_DEPTH_MM &&
                          fabsf(lateralMm) <= CLEARANCE_HALF_WIDTH_MM);

        if (withinBox && distanceMm < closestDistanceMm) {
            closestDistanceMm                = distanceMm;
            result.isClear                   = false;
            result.closestObstacleOffsetMm   = lateralMm;
            result.closestObstacleDistanceMm = distanceMm;
        }
    }

    turret_motor.write(90);
    delay(SERVO_SETTLE_MS);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Non-blocking clearance scanner (used by doApproach via AP_SCANNING)
// ─────────────────────────────────────────────────────────────────────────────
static int           nb_scanAngleDeg    = SCAN_START_DEG;
static unsigned long nb_lastMs          = 0;
static bool          nb_scanning        = false;
static float         nb_closestDistMm   = 1.0e9f;
static float         nb_closestOffsetMm = 0.0f;

void clearanceScanStart() {
    nb_scanAngleDeg    = SCAN_START_DEG;
    nb_scanning        = true;
    nb_lastMs          = 0;
    nb_closestDistMm   = 1.0e9f;
    nb_closestOffsetMm = 0.0f;
    turret_motor.write(nb_scanAngleDeg);
}

bool clearanceScanStep(ObstacleClearanceResult &out) {
    if (!nb_scanning) {
        out.isClear                   = true;
        out.closestObstacleOffsetMm   = 0.0f;
        out.closestObstacleDistanceMm = -1.0f;
        return true;
    }

    unsigned long now = millis();

    if (nb_lastMs == 0) {
        turret_motor.write(nb_scanAngleDeg);
        nb_lastMs = now;
        return false;
    }

    if (now - nb_lastMs < SERVO_SETTLE_MS) return false;

    float distanceCm = sensors.pingNowCm();
    if (distanceCm > 0.0f) {
        float distanceMm  = distanceCm * 10.0f;
        float bearingDeg  = (float)nb_scanAngleDeg - 90.0f;
        float bearingRad  = bearingDeg * DEG_TO_RAD_SCALE;
        float fwdMm       = SENSOR_OFFSET_MM + distanceMm * cosf(bearingRad);
        float lateralMm   = distanceMm * sinf(bearingRad);

        bool withinBox = (fwdMm >= 0.0f && fwdMm <= CLEARANCE_DEPTH_MM &&
                          fabsf(lateralMm) <= CLEARANCE_HALF_WIDTH_MM);

        if (withinBox && distanceMm < nb_closestDistMm) {
            nb_closestDistMm   = distanceMm;
            nb_closestOffsetMm = lateralMm;
        }
    }

    nb_scanAngleDeg += SCAN_STEP_DEG;
    nb_lastMs = 0;

    if (nb_scanAngleDeg > SCAN_END_DEG) {
        nb_scanning = false;
        turret_motor.write(90);

        if (nb_closestDistMm < 1.0e8f) {
            out.isClear                   = false;
            out.closestObstacleOffsetMm   = nb_closestOffsetMm;
            out.closestObstacleDistanceMm = nb_closestDistMm;
        } else {
            out.isClear                   = true;
            out.closestObstacleOffsetMm   = 0.0f;
            out.closestObstacleDistanceMm = -1.0f;
        }
        return true;
    }

    return false;
}
