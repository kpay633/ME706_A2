#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"
#include <Servo.h>

extern Motor   motors;
extern Sensors sensors;
extern Servo   turret_motor;

// ═════════════════════════════════════════════════════════════════════════════
// PINS & CONSTANTS
// ═════════════════════════════════════════════════════════════════════════════
#define PT_CENTRE_PIN  A3

// ─── Fire detection ───────────────────────────────────────────────────────────
// PT raw ADC: LOWER = brighter = closer to flame. Fire reached when the centre
// PT reads below this for several consecutive ticks WHILE driving at the flame.
#define PT_FIRE_THRESH        100
#define PT_FIRE_CONFIRM_TICKS  5     // consecutive ticks below threshold to confirm

// ─── Sweep ────────────────────────────────────────────────────────────────────
#define SWEEP_BINS          360
#define SWEEP_UNSAMPLED     0xFFFFu
#define PT_FILTER_SAMPLES   10
#define CENTROID_THRESH      0.7f
#define TURRET_LOCK_DEG     90
#define HEADING_OFFSET      -2.0f
#define PARTIAL_SWEEP_ARC_DEG  90.0f

// ─── Turret angles ────────────────────────────────────────────────────────────
//   90° = forward.   Servo: 0° points to the robot's RIGHT, 180° to the LEFT.
//   (If your servo is mounted the other way, swap TURRET_LEFT and TURRET_RIGHT.)
#define TURRET_FWD    90
#define TURRET_LEFT  180
#define TURRET_RIGHT   0

// ─── Ultrasonic forward-scan geometry (centre arc; corners handled by IR) ─────
static constexpr float    CLEARANCE_DEPTH_MM   = 400.0f;
static constexpr float    SENSOR_OFFSET_MM     =  50.0f;
static constexpr float    CLEARANCE_HALF_W_MM  = 125.0f;
static constexpr int      SCAN_START_DEG       = 50;
static constexpr int      SCAN_END_DEG         = 130;
static constexpr int      SCAN_STEP_DEG        = 6;
static constexpr uint16_t SERVO_SETTLE_MS      = 13;
static constexpr float    DEG2RAD           = 3.14159265f / 180.0f;

// ─── Obstacle thresholds (cm) ─────────────────────────────────────────────────
static constexpr float FRONT_IR_TRIGGER_CM = 12.0f;  // front IR: obstacle ahead
static constexpr float FRONT_IR_CLEAR_CM   = 18.0f;  // front IR: considered clear (hysteresis)
static constexpr float SIDE_US_BLOCKED_CM  = 20.0f;  // turret US side-check: corridor blocked
static constexpr float REAR_IR_BLOCKED_CM  = 20.0f;  // rear side IR: obstacle in strafe path
static constexpr float US_FWD_TRIGGER_CM   = 20.0f;  // forward US scan: obstacle ahead

// ─── Motion timing / odometry ─────────────────────────────────────────────────
static constexpr unsigned long SIDE_CHECK_SETTLE_MS = 120;   // turret settle before US read
static constexpr unsigned long STRAFE_TIMEOUT_MS     = 4000;  // safety cap on a strafe
static constexpr unsigned long STRAFE_EXTRA_MS       = 400;   // extra strafe after front clears
static constexpr unsigned long POST_STRAFE_FWD_MS    = 3000;  // forward drive after strafe
static constexpr float         REVERSE_DIST_CM       = 30.0f; // reverse distance (US odometry)
static constexpr unsigned long REVERSE_TIMEOUT_MS    = 3000;

// ═════════════════════════════════════════════════════════════════════════════
// STATE
// ═════════════════════════════════════════════════════════════════════════════
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

// Top-level FSM
enum FireSubState {
    FS_SWEEP, FS_PARTIAL_SWEEP, FS_TURN, FS_APPROACH, FS_DONE
};
static FireSubState subState     = FS_SWEEP;
static bool         sweepInited  = false;
static bool         sweepStarted = false;
static bool         targetSet    = false;

static float partialSweepStartDeg = 0.0f;
static float partialSweepEndDeg   = 90.0f;

// Approach FSM
enum ApproachSubState {
    AP_DRIVE,        // driving at flame; front IR + fire-reached checked
    AP_SCANNING,     // forward US scan running while driving
    AP_SIDE_CHECK,   // turret pointed to candidate strafe side, taking US reading
    AP_STRAFING,     // strafing; front IR (clear?) + rear IR + turret US watched
    AP_STRAFE_EXTRA, // front cleared — strafe a little more to fully pass obstacle
    AP_POST_STRAFE,  // drive forward past the obstacle before re-seeking flame
    AP_REVERSING     // both sides blocked — back up, then re-check sides
};
static ApproachSubState approachSub = AP_DRIVE;

// Avoidance working vars
static bool          strafeRight       = false;  // current / intended strafe direction
static bool          sideCheckFallback = false;  // true once we've flipped to 2nd side
static unsigned long sideCheckSettleMs = 0;
static unsigned long strafeStartMs     = 0;
static unsigned long strafeExtraMs     = 0;
static unsigned long postStrafeMs      = 0;
static unsigned long reverseStartMs    = 0;
static float         reverseStartCm    = 0.0f;
static uint8_t       ptFireCounter     = 0;       // consecutive fire-confirm ticks

// Forward declarations
static void  doSweep();
static void  doPartialSweep();
static void  doTurn();
static void  doApproach();
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid);
static int   evalFrontIR();
static bool  frontIsClear();
static void  pointTurret(bool toRight);
static void  beginStrafe(bool goRight);

// ═════════════════════════════════════════════════════════════════════════════
void resetFireRoutine() {
    subState      = FS_SWEEP;
    sweepInited   = false;
    sweepStarted  = false;
    targetSet     = false;
    approachSub   = AP_DRIVE;
    ptFireCounter = 0;
    turret_motor.write(TURRET_FWD);
}

// ═════════════════════════════════════════════════════════════════════════════
STEP runFireRoutine() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        Serial.print(F("[FSM] "));
        switch (subState) {
            case FS_SWEEP:         Serial.println(F("SWEEP"));         break;
            case FS_PARTIAL_SWEEP: Serial.println(F("PARTIAL_SWEEP")); break;
            case FS_TURN:          Serial.println(F("TURN"));          break;
            case FS_APPROACH:      Serial.println(F("APPROACH"));      break;
            case FS_DONE:          Serial.println(F("DONE"));          break;
        }
    }
    switch (subState) {
        case FS_SWEEP:         doSweep();        break;
        case FS_PARTIAL_SWEEP: doPartialSweep(); break;
        case FS_TURN:          doTurn();         break;
        case FS_APPROACH:      doApproach();     break;
        case FS_DONE:
            Serial.println(F("[FSM] DONE — NEXT_STEP"));
            return NEXT_STEP;
    }
    return CURRENT_STEP;
}

// ═════════════════════════════════════════════════════════════════════════════
// doSweep — full 360° rotation, centroid fire detection
// ═════════════════════════════════════════════════════════════════════════════
static void doSweep() {
    if (!sweepInited) {
        Serial.println(F("[SWEEP] Init — full 360"));
        turret_motor.write(TURRET_LOCK_DEG);
        hotspot = Hotspot(0.0f, 1023, false);
        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;
        ptBufIdx = 0; ptBufSum = 0.0f;
        for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
            float v = (float)analogRead(PT_CENTRE_PIN);
            ptBuf[i] = v; ptBufSum += v; delay(5);
        }
        sweepStarted = false; sweepInited = true;
        sensors.ZeroGyroHeading();
        motors.rotateCounterClockwise();
        return;
    }

    float heading = sensors.getGyroHeading();
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
        sweepInited = sweepStarted = false;
        uint16_t gMin = 0, gMax = 0; bool valid = false;
        float rawAngle = computeHotspotAngle(gMin, gMax, valid);
        float target = rawAngle + HEADING_OFFSET;
        while (target >= 360.0f) target -= 360.0f;
        while (target <    0.0f) target += 360.0f;
        hotspot = Hotspot(target, (int)gMin, valid);
        Serial.print(F("[SWEEP] hotspot=")); Serial.print(target);
        Serial.print(F(" valid="));          Serial.println(valid);
        subState = FS_TURN;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// doPartialSweep — sweep only a 90° arc after an avoidance manoeuvre.
// Gyro is re-zeroed here so heading starts at 0 and increases CCW.
//   Strafed RIGHT → obstacle was on LEFT → flame is on our LEFT
//                 → CCW reaches it immediately → arc [0, 90]
//   Strafed LEFT  → obstacle was on RIGHT → flame is on our RIGHT
//                 → CCW reaches it at the end → arc [270, 358]
// (Arc is set by the caller before transitioning here.)
// ═════════════════════════════════════════════════════════════════════════════
static void doPartialSweep() {
    if (!sweepInited) {
        Serial.print(F("[PSWEEP] Init — arc "));
        Serial.print(partialSweepStartDeg); Serial.print(F("→"));
        Serial.println(partialSweepEndDeg);
        turret_motor.write(TURRET_LOCK_DEG);
        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;
        ptBufIdx = 0; ptBufSum = 0.0f;
        for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
            float v = (float)analogRead(PT_CENTRE_PIN);
            ptBuf[i] = v; ptBufSum += v; delay(5);
        }
        hotspot = Hotspot(0.0f, 1023, false);
        sweepStarted = false; sweepInited = true;
        sensors.ZeroGyroHeading();
        motors.rotateCounterClockwise();
        return;
    }

    float heading = sensors.getGyroHeading();
    if (!sweepStarted) {
        bool entered = (partialSweepStartDeg < 5.0f) ? (heading > 5.0f)
                                                      : (heading >= partialSweepStartDeg);
        if (entered) { sweepStarted = true; Serial.println(F("[PSWEEP] Arc entered")); }
    }

    float newVal = (float)analogRead(PT_CENTRE_PIN);
    ptBufSum -= ptBuf[ptBufIdx];
    ptBuf[ptBufIdx] = newVal;
    ptBufSum += newVal;
    ptBufIdx = (ptBufIdx + 1) % PT_FILTER_SAMPLES;
    float avg = ptBufSum / (float)PT_FILTER_SAMPLES;

    if (heading >= partialSweepStartDeg && heading <= partialSweepEndDeg) {
        int bin = (int)heading;
        if (bin >= 0 && bin < SWEEP_BINS) sweepSamples[bin] = (uint16_t)avg;
    }

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print(F("[PSWEEP] hdg=")); Serial.print(heading);
        Serial.print(F(" avg="));         Serial.println(avg, 1);
    }

    if (sweepStarted && heading >= partialSweepEndDeg) {
        motors.stop();
        sweepInited = sweepStarted = false;
        uint16_t gMin = 0, gMax = 0; bool valid = false;
        float rawAngle = computeHotspotAngle(gMin, gMax, valid);
        float target = rawAngle + HEADING_OFFSET;
        while (target >= 360.0f) target -= 360.0f;
        while (target <    0.0f) target += 360.0f;
        hotspot = Hotspot(target, (int)gMin, valid);
        Serial.print(F("[PSWEEP] hotspot=")); Serial.print(target);
        Serial.print(F(" valid="));           Serial.println(valid);
        if (!valid) {
            Serial.println(F("[PSWEEP] Nothing in arc — full sweep"));
            subState = FS_SWEEP;
        } else {
            subState = FS_TURN;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid) {
    uint16_t globalMin = 1023, globalMax = 0;
    int minBin = -1, sampledCount = 0;
    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        sampledCount++;
        if (sweepSamples[i] < globalMin) { globalMin = sweepSamples[i]; minBin = i; }
        if (sweepSamples[i] > globalMax)   globalMax = sweepSamples[i];
    }
    outMin = globalMin; outMax = globalMax;
    if (sampledCount == 0) { outValid = false; return 0.0f; }

    float range = (float)(globalMax - globalMin);
    float sumCos = 0.0f, sumSin = 0.0f, bestNorm = 0.0f;
    int bestBin = -1, binsAbove = 0;
    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        float norm = (range > 0.01f) ? (float)(globalMax - sweepSamples[i]) / range : 0.0f;
        if (norm > CENTROID_THRESH) {
            binsAbove++;
            float rad = (float)i * (float)PI / 180.0f;
            sumCos += norm * cosf(rad);
            sumSin += norm * sinf(rad);
        }
        if (norm > bestNorm) { bestNorm = norm; bestBin = i; }
    }
    Serial.print(F("[CENTROID] n=")); Serial.print(sampledCount);
    Serial.print(F(" peak="));        Serial.print(minBin);
    Serial.print(F(" above="));       Serial.print(binsAbove);
    Serial.print(F(" norm="));        Serial.println(bestNorm, 3);

    if (sumCos != 0.0f || sumSin != 0.0f) {
        float deg = atan2f(sumSin, sumCos) * 180.0f / (float)PI;
        if (deg < 0.0f) deg += 360.0f;
        outValid = true; return deg;
    }
    if (bestBin >= 0) { outValid = true; return (float)bestBin; }
    outValid = false; return 0.0f;
}

// ═════════════════════════════════════════════════════════════════════════════
// doTurn — PID turn to hotspot heading
// ═════════════════════════════════════════════════════════════════════════════
static void doTurn() {
    if (!hotspot.valid) {
        Serial.println(F("[TURN] No hotspot → DONE"));
        targetSet = false; subState = FS_DONE; return;
    }
    if (!targetSet) {
        motors.SetTurnTarget(hotspot.angle);
        targetSet = true;
        Serial.print(F("[TURN] target=")); Serial.println(hotspot.angle);
    }
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print(F("[TURN] cur=")); Serial.println(sensors.getGyroHeading());
    }
    if (motors.isTurnComplete(sensors.getGyroHeading())) {
        Serial.print(F("[TURN] Done — heading="));
        Serial.print(sensors.getGyroHeading());
        Serial.println(F(" → APPROACH"));
        targetSet     = false;
        approachSub   = AP_DRIVE;
        ptFireCounter = 0;
        turret_motor.write(TURRET_FWD);
        motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
        subState = FS_APPROACH;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Front IR evaluation.
//   0 = clear,  +1 = left blocked (→ strafe right),  -1 = right blocked (→ left)
//   +2 = both blocked, left closer (→ strafe right after reverse)
//   -2 = both blocked, right closer (→ strafe left after reverse)
// ═════════════════════════════════════════════════════════════════════════════
static int evalFrontIR() {
    float fl = sensors.readIRFrontLeft();
    float fr = sensors.readIRFrontRight();
    bool leftBlocked  = (fl > 0.5f && fl < FRONT_IR_TRIGGER_CM);
    bool rightBlocked = (fr > 0.5f && fr < FRONT_IR_TRIGGER_CM);
    if (leftBlocked && rightBlocked) return (fl <= fr) ? 2 : -2;
    if (leftBlocked)  return  1;
    if (rightBlocked) return -1;
    return 0;
}

// True when BOTH front IRs read clear (above the hysteresis clear threshold,
// or out of range). Used to decide when a strafe has cleared the obstacle.
static bool frontIsClear() {
    float fl = sensors.readIRFrontLeft();
    float fr = sensors.readIRFrontRight();
    bool flClear = (fl < 0.5f || fl > FRONT_IR_CLEAR_CM);
    bool frClear = (fr < 0.5f || fr > FRONT_IR_CLEAR_CM);
    return flClear && frClear;
}

// Point the turret to a side. toRight=true → robot's right, false → left.
static void pointTurret(bool toRight) {
    int angle = toRight ? TURRET_RIGHT : TURRET_LEFT;
    turret_motor.write(angle);
    Serial.print(F("[TURRET] → ")); Serial.print(toRight ? F("RIGHT") : F("LEFT"));
    Serial.print(F(" (")); Serial.print(angle); Serial.println(F("°)"));
}

// Begin strafing. Turret should already point to the strafe side.
static void beginStrafe(bool goRight) {
    strafeRight   = goRight;
    strafeStartMs = millis();
    Serial.print(F("[STRAFE] Begin ")); Serial.println(goRight ? F("RIGHT") : F("LEFT"));
    if (goRight) motors.strafeRight();
    else         motors.strafeLeft();
}

// ═════════════════════════════════════════════════════════════════════════════
// doApproach — drive at flame with layered obstacle avoidance.
//
// FIRE-REACHED is checked ONLY in AP_DRIVE / AP_SCANNING (when pointed at the
// flame) and must be sustained for PT_FIRE_CONFIRM_TICKS to avoid false stops
// from transient bright readings during manoeuvres.
// ═════════════════════════════════════════════════════════════════════════════
static void doApproach() {
    float heading = sensors.getGyroHeading();

    // Per-tick sensor log
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print(F("[AP] sub=")); Serial.print((int)approachSub);
        Serial.print(F(" FL="));      Serial.print(sensors.readIRFrontLeft(),  1);
        Serial.print(F(" FR="));      Serial.print(sensors.readIRFrontRight(), 1);
        Serial.print(F(" RL="));      Serial.print(sensors.readIRRearLeft(),   1);
        Serial.print(F(" RR="));      Serial.print(sensors.readIRRearRight(),  1);
        Serial.print(F(" PT="));      Serial.println(analogRead(PT_CENTRE_PIN));
    }

    switch (approachSub) {

        // ── AP_DRIVE: drive at flame, watch front IR + fire-reached ──────────
        case AP_DRIVE: {
            // Fire-reached (only while driving straight at the flame)
            int pt = analogRead(PT_CENTRE_PIN);
            if (pt < PT_FIRE_THRESH) {
                ptFireCounter++;
                if (ptFireCounter >= PT_FIRE_CONFIRM_TICKS) {
                    motors.stop();
                    turret_motor.write(TURRET_FWD);
                    Serial.print(F("[AP_DRIVE] Fire confirmed (PT="));
                    Serial.print(pt); Serial.println(F(") → DONE"));
                    subState = FS_DONE;
                    break;
                }
            } else {
                ptFireCounter = 0;
            }

            if (!motors.driveStraight(heading, 0.0f, 0.0f)) {
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
            }

            int action = evalFrontIR();
            if (action != 0) {
                float fl = sensors.readIRFrontLeft();
                float fr = sensors.readIRFrontRight();
                motors.stop();
                ptFireCounter = 0;
                bool preferRight = (action == 1 || action == 2);
                Serial.print(F("[AP_DRIVE] Front IR: action=")); Serial.print(action);
                Serial.print(F(" FL=")); Serial.print(fl, 1);
                Serial.print(F(" FR=")); Serial.print(fr, 1);
                Serial.print(F(" → check ")); Serial.println(preferRight ? F("RIGHT") : F("LEFT"));
                sideCheckFallback = false;
                strafeRight       = preferRight;
                sideCheckSettleMs = millis();
                pointTurret(preferRight);
                approachSub = AP_SIDE_CHECK;
                break;
            }

            clearanceScanStart();
            approachSub = AP_SCANNING;
            break;
        }

        // ── AP_SCANNING: forward US scan while driving ───────────────────────
        case AP_SCANNING: {
            // Fire-reached still valid here (still pointed at flame)
            int pt = analogRead(PT_CENTRE_PIN);
            if (pt < PT_FIRE_THRESH) {
                ptFireCounter++;
                if (ptFireCounter >= PT_FIRE_CONFIRM_TICKS) {
                    motors.stop();
                    turret_motor.write(TURRET_FWD);
                    Serial.print(F("[AP_SCAN] Fire confirmed (PT="));
                    Serial.print(pt); Serial.println(F(") → DONE"));
                    subState = FS_DONE;
                    break;
                }
            } else {
                ptFireCounter = 0;
            }

            motors.driveStraight(heading, 0.0f, 0.0f);

            int action = evalFrontIR();
            if (action != 0) {
                motors.stop();
                ptFireCounter = 0;
                bool preferRight = (action == 1 || action == 2);
                Serial.print(F("[AP_SCAN] Front IR: action=")); Serial.print(action);
                Serial.print(F(" → check ")); Serial.println(preferRight ? F("RIGHT") : F("LEFT"));
                sideCheckFallback = false;
                strafeRight       = preferRight;
                sideCheckSettleMs = millis();
                pointTurret(preferRight);
                approachSub = AP_SIDE_CHECK;
                break;
            }

            ObstacleClearanceResult scan;
            if (clearanceScanStep(scan)) {
                if (!scan.isClear &&
                    scan.closestObstacleDistanceMm < (US_FWD_TRIGGER_CM * 10.0f) &&
                    fabsf(scan.closestObstacleOffsetMm) < CLEARANCE_HALF_W_MM)
                {
                    motors.stop();
                    ptFireCounter = 0;
                    // obstacle right of centre → strafe left, and vice-versa
                    bool preferRight = (scan.closestObstacleOffsetMm < 0.0f);
                    Serial.print(F("[AP_SCAN] US obstacle dist="));
                    Serial.print(scan.closestObstacleDistanceMm, 0);
                    Serial.print(F("mm off="));
                    Serial.print(scan.closestObstacleOffsetMm, 0);
                    Serial.print(F("mm → check ")); Serial.println(preferRight ? F("RIGHT") : F("LEFT"));
                    sideCheckFallback = false;
                    strafeRight       = preferRight;
                    sideCheckSettleMs = millis();
                    pointTurret(preferRight);
                    approachSub = AP_SIDE_CHECK;
                } else {
                    approachSub = AP_DRIVE;
                }
            }
            break;
        }

        // ── AP_SIDE_CHECK: turret points to candidate side, one US reading ───
        case AP_SIDE_CHECK: {
            if (millis() - sideCheckSettleMs < SIDE_CHECK_SETTLE_MS) break;

            float usSide = sensors.pingNowCm();
            bool blocked = (usSide > 0.5f && usSide < SIDE_US_BLOCKED_CM);

            Serial.print(F("[SIDE_CHECK] ")); Serial.print(strafeRight ? F("RIGHT") : F("LEFT"));
            Serial.print(F(" US=")); Serial.print(usSide, 1);
            Serial.println(blocked ? F(" BLOCKED") : F(" CLEAR"));

            if (!blocked) {
                beginStrafe(strafeRight);
                approachSub = AP_STRAFING;
            } else if (!sideCheckFallback) {
                sideCheckFallback = true;
                strafeRight       = !strafeRight;
                sideCheckSettleMs = millis();
                Serial.println(F("[SIDE_CHECK] Blocked — trying other side"));
                pointTurret(strafeRight);
            } else {
                Serial.println(F("[SIDE_CHECK] Both blocked → REVERSING"));
                turret_motor.write(TURRET_FWD);
                reverseStartCm = sensors.pingNowCm();
                reverseStartMs = millis();
                motors.driveReverse();
                approachSub = AP_REVERSING;
            }
            break;
        }

        // ── AP_STRAFING: strafe until front clears (turret watches corridor) ──
        case AP_STRAFING: {
            float usSide = sensors.pingNowCm();   // turret still on strafe side
            float rearIR = strafeRight ? sensors.readIRRearRight()
                                       : sensors.readIRRearLeft();

            bool usBlocked  = (usSide > 0.5f && usSide < SIDE_US_BLOCKED_CM);
            bool rearBlocked = (rearIR > 0.5f && rearIR < REAR_IR_BLOCKED_CM);
            bool timedOut   = (millis() - strafeStartMs >= STRAFE_TIMEOUT_MS);

            static unsigned long lastStr = 0;
            if (millis() - lastStr > 200) {
                lastStr = millis();
                Serial.print(F("[STRAFE] usSide=")); Serial.print(usSide, 1);
                Serial.print(F(" rearIR="));         Serial.print(rearIR, 1);
                Serial.print(F(" frontClear="));     Serial.println(frontIsClear());
            }

            // New obstacle appeared in the strafe corridor — back to drive/reassess
            if (usBlocked || rearBlocked) {
                motors.stop();
                Serial.print(F("[STRAFE] Corridor blocked (US="));
                Serial.print(usSide, 1); Serial.print(F(" rear="));
                Serial.print(rearIR, 1); Serial.println(F(") → reassess"));
                turret_motor.write(TURRET_FWD);
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_DRIVE;
                break;
            }

            // Obstacle cleared from front → strafe a little extra then move on
            if (frontIsClear()) {
                Serial.println(F("[STRAFE] Front clear → extra strafe"));
                strafeExtraMs = millis();
                approachSub = AP_STRAFE_EXTRA;
                break;
            }

            if (timedOut) {
                motors.stop();
                Serial.println(F("[STRAFE] Timeout → post-strafe drive"));
                turret_motor.write(TURRET_FWD);
                postStrafeMs = millis();
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_POST_STRAFE;
            }
            break;
        }

        // ── AP_STRAFE_EXTRA: keep strafing a bit to fully pass the obstacle ──
        case AP_STRAFE_EXTRA: {
            // Still watch the corridor during the extra strafe
            float usSide = sensors.pingNowCm();
            float rearIR = strafeRight ? sensors.readIRRearRight()
                                       : sensors.readIRRearLeft();
            bool corridorBlocked = (usSide > 0.5f && usSide < SIDE_US_BLOCKED_CM) ||
                                   (rearIR > 0.5f && rearIR < REAR_IR_BLOCKED_CM);
            if (corridorBlocked) {
                motors.stop();
                Serial.println(F("[EXTRA] Corridor blocked → reassess"));
                turret_motor.write(TURRET_FWD);
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_DRIVE;
                break;
            }

            if (millis() - strafeExtraMs >= STRAFE_EXTRA_MS) {
                motors.stop();
                Serial.println(F("[EXTRA] Done → post-strafe drive"));
                turret_motor.write(TURRET_FWD);
                postStrafeMs = millis();
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_POST_STRAFE;
            }
            break;
        }

        // ── AP_POST_STRAFE: drive forward past obstacle, then re-seek flame ──
        case AP_POST_STRAFE: {
            // Front IR avoidance stays active during the forward drive
            int action = evalFrontIR();
            if (action != 0) {
                motors.stop();
                bool preferRight = (action == 1 || action == 2);
                Serial.print(F("[POST] Front IR mid-drive → check "));
                Serial.println(preferRight ? F("RIGHT") : F("LEFT"));
                sideCheckFallback = false;
                strafeRight       = preferRight;
                sideCheckSettleMs = millis();
                pointTurret(preferRight);
                approachSub = AP_SIDE_CHECK;
                break;
            }

            motors.driveStraight(heading, 0.0f, 0.0f);

            if (millis() - postStrafeMs >= POST_STRAFE_FWD_MS) {
                motors.stop();
                Serial.println(F("[POST] Forward done → partial sweep"));
                // Strafed right → flame LEFT  → arc [0,90]
                // Strafed left  → flame RIGHT → arc [270,358]
                partialSweepStartDeg = strafeRight ?   0.0f : 270.0f;
                partialSweepEndDeg   = strafeRight ?  90.0f : 358.0f;
                Serial.print(F("[POST] Arc ")); Serial.print(partialSweepStartDeg);
                Serial.print(F("→")); Serial.println(partialSweepEndDeg);
                approachSub = AP_DRIVE;
                subState    = FS_PARTIAL_SWEEP;
            }
            break;
        }

        // ── AP_REVERSING: back up 30cm, then re-check sides ──────────────────
        case AP_REVERSING: {
            float usFwd = sensors.pingNowCm();   // turret forward
            bool distReached = (usFwd > 0.5f && (usFwd - reverseStartCm) >= REVERSE_DIST_CM);
            bool timedOut    = (millis() - reverseStartMs >= REVERSE_TIMEOUT_MS);

            static unsigned long lastRev = 0;
            if (millis() - lastRev > 200) {
                lastRev = millis();
                Serial.print(F("[REV] US=")); Serial.print(usFwd, 1);
                Serial.print(F(" delta="));   Serial.println(usFwd - reverseStartCm, 1);
            }

            if (distReached || timedOut) {
                motors.stop();
                Serial.println(timedOut ? F("[REV] Timeout") : F("[REV] 30cm done"));
                sideCheckFallback = false;
                strafeRight       = true;            // re-check right first
                sideCheckSettleMs = millis();
                pointTurret(true);
                approachSub = AP_SIDE_CHECK;
            }
            break;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Blocking clearance scan (utility — not used by doApproach)
// ═════════════════════════════════════════════════════════════════════════════
ObstacleClearanceResult checkForwardClearance() {
    ObstacleClearanceResult result;
    result.isClear = true; result.closestObstacleOffsetMm = 0.0f;
    result.closestObstacleDistanceMm = -1.0f;
    float closestMm = 1.0e9f;
    for (int deg = SCAN_START_DEG; deg <= SCAN_END_DEG; deg += SCAN_STEP_DEG) {
        turret_motor.write(deg); delay(SERVO_SETTLE_MS);
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
            result.closestObstacleOffsetMm   = lat;
            result.closestObstacleDistanceMm = mm;
        }
    }
    turret_motor.write(TURRET_FWD); delay(SERVO_SETTLE_MS);
    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Non-blocking clearance scanner (centre arc 50–130°)
// ═════════════════════════════════════════════════════════════════════════════
static int           nb_scanAngleDeg    = SCAN_START_DEG;
static unsigned long nb_lastMs          = 0;
static bool          nb_scanning        = false;
static float         nb_closestDistMm   = 1.0e9f;
static float         nb_closestOffsetMm = 0.0f;

void clearanceScanStart() {
    nb_scanAngleDeg  = SCAN_START_DEG;
    nb_scanning      = true;
    nb_lastMs        = 0;
    nb_closestDistMm   = 1.0e9f;
    nb_closestOffsetMm = 0.0f;
    turret_motor.write(nb_scanAngleDeg);
}

bool clearanceScanStep(ObstacleClearanceResult &out) {
    if (!nb_scanning) {
        out.isClear = true; out.closestObstacleOffsetMm = 0.0f;
        out.closestObstacleDistanceMm = -1.0f;
        return true;
    }
    unsigned long now = millis();
    if (nb_lastMs == 0) { turret_motor.write(nb_scanAngleDeg); nb_lastMs = now; return false; }
    if (now - nb_lastMs < SERVO_SETTLE_MS) return false;

    float cm = sensors.pingNowCm();
    if (cm > 0.0f) {
        float mm  = cm * 10.0f;
        float brg = ((float)nb_scanAngleDeg - 90.0f) * DEG2RAD;
        float fwd = SENSOR_OFFSET_MM + mm * cosf(brg);
        float lat = mm * sinf(brg);
        if (fwd >= 0.0f && fwd <= CLEARANCE_DEPTH_MM &&
            fabsf(lat) <= CLEARANCE_HALF_W_MM && mm < nb_closestDistMm) {
            nb_closestDistMm   = mm;
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
            out.closestObstacleOffsetMm   = nb_closestOffsetMm;
            out.closestObstacleDistanceMm = nb_closestDistMm;
        } else {
            out.isClear = true; out.closestObstacleOffsetMm = 0.0f;
            out.closestObstacleDistanceMm = -1.0f;
        }
        return true;
    }
    return false;
}