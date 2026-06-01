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
#define PT_FIRE_THRESH        20
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
#define TURRET_FWD    90
#define TURRET_LEFT  180
#define TURRET_RIGHT   0

// ─── Ultrasonic forward-scan geometry (used by utility scan funcs only) ───────
static constexpr float    CLEARANCE_DEPTH_MM   = 400.0f;
static constexpr float    SENSOR_OFFSET_MM     =  50.0f;
static constexpr float    CLEARANCE_HALF_W_MM  = 125.0f;
static constexpr int      SCAN_START_DEG       = 50;
static constexpr int      SCAN_END_DEG         = 130;
static constexpr int      SCAN_STEP_DEG        = 6;
static constexpr uint16_t SERVO_SETTLE_MS      = 13;
static constexpr float    DEG2RAD              = 3.14159265f / 180.0f;

// ─── Obstacle thresholds (cm) ─────────────────────────────────────────────────
static constexpr float FRONT_IR_TRIGGER_CM = 17.0f;  // front IR: obstacle ahead
static constexpr float FRONT_IR_CLEAR_CM   = 23.0f;  // front IR: considered clear (hysteresis)
static constexpr float SIDE_US_BLOCKED_CM  = 20.0f;  // turret US side-check: corridor blocked
static constexpr float REAR_IR_BLOCKED_CM  = 12.0f;  // rear side IR: obstacle in strafe path
static constexpr float US_FWD_TRIGGER_CM   = 15.0f;  // forward US: obstacle straight ahead

// ─── Motion timing / odometry ─────────────────────────────────────────────────
static constexpr unsigned long SIDE_CHECK_SETTLE_MS = 120;
static constexpr unsigned long STRAFE_TIMEOUT_MS     = 4000;
static constexpr uint8_t       CORRIDOR_BLOCK_TICKS  = 3;
static constexpr unsigned long STRAFE_US_SETTLE_MS   = 200;
static constexpr float         SENSOR_JUMP_REJECT_CM = 20.0f;
static constexpr unsigned long STRAFE_EXTRA_MS       = 400;
static constexpr unsigned long POST_STRAFE_FWD_MS    = 1800;
static constexpr unsigned long REVERSE_TIMEOUT_MS    = 1000;

// ─── Both-sides-blocked recovery (spin to find a gap) ────────────────────────
static constexpr float    GAP_MARGIN_CM       = 20.0f;  // all 3 front sensors must exceed this = gap
static constexpr uint8_t  GAP_CONFIRM_TICKS   = 3;      // consecutive clear ticks to confirm a gap
static constexpr float    GAP_SPIN_MAX_DEG    = 90.0f;  // max CW spin while searching for a gap
static constexpr float    NOGAP_REVERSE_CM    = 30.0f;  // reverse distance if no gap found (easy to change)
static constexpr unsigned long NOGAP_STRAFE_MS = 2500;  // long strafe after the no-gap reverse

// ─── Inter-state settle (pause + confirm clear before driving forward) ───────
static constexpr unsigned long SETTLE_PAUSE_MS    = 250;  // motor-stop pause before confirming
static constexpr uint8_t       SETTLE_CLEAR_TICKS = 3;    // consecutive all-clear ticks required

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
    FS_SWEEP, FS_TURRET_SEEK, FS_TURN, FS_APPROACH, FS_DONE
};
static FireSubState subState     = FS_SWEEP;
static bool         sweepInited  = false;
static bool         sweepStarted = false;
static bool         targetSet    = false;

// ── Turret-seek (servo sweep to re-acquire flame without rotating body) ──────
static constexpr int      TSEEK_STEP_DEG   = 3;
static constexpr uint16_t TSEEK_SETTLE_MS  = 20;
static int          tseekServoDeg    = TURRET_FWD;
static int          tseekDir         = +1;
static int          tseekEndDeg      = TURRET_LEFT;
static unsigned long tseekStepMs     = 0;
static int          tseekBestPT      = 1023;
static int          tseekBestServo   = TURRET_FWD;
static bool         tseekInited      = false;

// Approach FSM
enum ApproachSubState {
    AP_DRIVE,          // driving at flame; front IR + forward US + fire-reached
    AP_SIDE_CHECK,     // turret pointed to candidate strafe side, taking US reading
    AP_STRAFING,       // strafing; front sensors (clear?) + rear IR + turret US
    AP_STRAFE_EXTRA,   // front cleared — strafe a little more to fully pass obstacle
    AP_SETTLE,         // motors stopped; wait + confirm front clear before driving
    AP_POST_STRAFE,    // drive forward past the obstacle before re-seeking flame
    AP_SPIN_GAP,       // both sides blocked — spin CW up to 90° to find a gap
    AP_GAP_FWD,        // gap found — drive forward through it
    AP_RETURN_HEADING, // turn body back to a target heading
    AP_NOGAP_REVERSE,  // no gap found — reverse NOGAP_REVERSE_CM
    AP_NOGAP_STRAFE    // then long strafe left
};
static ApproachSubState approachSub = AP_DRIVE;

// Avoidance working vars
static bool          strafeRight        = false;
static bool          sideCheckFallback  = false;
static unsigned long sideCheckSettleMs  = 0;
static unsigned long strafeStartMs      = 0;
static uint8_t       corridorBlockCount = 0;
static float         prevUsSide         = -1.0f;
static float         prevRearIR         = -1.0f;
static unsigned long strafeExtraMs      = 0;
static unsigned long postStrafeMs       = 0;
static uint8_t       ptFireCounter      = 0;

// Both-sides-blocked recovery working vars
static float         detectHeading       = 0.0f;   // heading when obstacle detected
static uint8_t       gapClearCount       = 0;
static float         returnTargetHeading = 0.0f;
static bool          afterGapFwd         = false;   // true = seek flame after return; false = reverse+strafe
static unsigned long nogapStrafeMs       = 0;
static float         nogapReverseStartCm = 0.0f;
static unsigned long nogapReverseMs      = 0;
static unsigned long settleStartMs       = 0;     // settle pause start
static uint8_t       settleClearCount    = 0;     // consecutive all-clear ticks during settle

// Forward declarations
static void  doSweep();
static void  doTurretSeek();
static void  doTurn();
static void  doApproach();
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid);
static bool  frontIsClear();
static bool  allFrontClearMargin();
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
            case FS_SWEEP:       Serial.println(F("SWEEP"));       break;
            case FS_TURRET_SEEK: Serial.println(F("TURRET_SEEK")); break;
            case FS_TURN:        Serial.println(F("TURN"));        break;
            case FS_APPROACH:    Serial.println(F("APPROACH"));    break;
            case FS_DONE:        Serial.println(F("DONE"));        break;
        }
    }
    switch (subState) {
        case FS_SWEEP:       doSweep();      break;
        case FS_TURRET_SEEK: doTurretSeek(); break;
        case FS_TURN:        doTurn();       break;
        case FS_APPROACH:    doApproach();   break;
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
// doTurretSeek — sweep the SERVO across a 90° arc on the side opposite the
// strafe, find the brightest PT angle, then turn the body to face it.
//   strafeRight==true  → flame on our LEFT  → sweep servo 90°→180°
//   strafeRight==false → flame on our RIGHT → sweep servo 90°→0°
// ═════════════════════════════════════════════════════════════════════════════
static void doTurretSeek() {
    if (!tseekInited) {
        tseekDir       = strafeRight ? +1 : -1;
        tseekEndDeg    = strafeRight ? TURRET_LEFT : TURRET_RIGHT;
        tseekServoDeg  = TURRET_FWD;
        tseekBestPT    = 1023;
        tseekBestServo = TURRET_FWD;
        tseekStepMs    = 0;
        tseekInited    = true;
        motors.stop();
        turret_motor.write(tseekServoDeg);
        Serial.print(F("[TSEEK] Init — sweeping servo 90→"));
        Serial.print(tseekEndDeg);
        Serial.println(strafeRight ? F(" (flame LEFT)") : F(" (flame RIGHT)"));
        return;
    }

    if (tseekStepMs == 0) { tseekStepMs = millis(); return; }
    if (millis() - tseekStepMs < TSEEK_SETTLE_MS) return;

    int pt = analogRead(PT_CENTRE_PIN);
    if (pt < tseekBestPT) { tseekBestPT = pt; tseekBestServo = tseekServoDeg; }

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 150) {
        lastLog = millis();
        Serial.print(F("[TSEEK] servo=")); Serial.print(tseekServoDeg);
        Serial.print(F(" PT="));           Serial.print(pt);
        Serial.print(F(" best="));         Serial.print(tseekBestPT);
        Serial.print(F("@"));              Serial.println(tseekBestServo);
    }

    tseekServoDeg += tseekDir * TSEEK_STEP_DEG;
    tseekStepMs = 0;
    turret_motor.write(tseekServoDeg);

    bool done = (tseekDir > 0) ? (tseekServoDeg >= tseekEndDeg)
                               : (tseekServoDeg <= tseekEndDeg);
    if (done) {
        turret_motor.write(TURRET_FWD);
        Serial.print(F("[TSEEK] done — brightest servo="));
        Serial.print(tseekBestServo);
        Serial.print(F(" PT=")); Serial.println(tseekBestPT);

        float servoOffset = (float)(tseekBestServo - TURRET_FWD);   // +ve = left
        float curHeading  = sensors.getGyroHeading();
        float targetHeading = curHeading + servoOffset;             // CCW positive
        while (targetHeading >= 360.0f) targetHeading -= 360.0f;
        while (targetHeading <    0.0f) targetHeading += 360.0f;

        hotspot = Hotspot(targetHeading, tseekBestPT, true);
        tseekInited = false;

        Serial.print(F("[TSEEK] offset=")); Serial.print(servoOffset);
        Serial.print(F(" → targetHdg="));   Serial.println(targetHeading);

        if (fabsf(servoOffset) < 4.0f) {
            Serial.println(F("[TSEEK] flame ~ahead — resume approach"));
            approachSub = AP_DRIVE;
            ptFireCounter = 0;
            turret_motor.write(TURRET_FWD);
            motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
            subState = FS_APPROACH;
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
// Helpers
// ═════════════════════════════════════════════════════════════════════════════
static bool frontIsClear() {
    float fl = sensors.readIRFrontLeft();
    float fr = sensors.readIRFrontRight();
    bool flClear = (fl < 0.5f || fl > FRONT_IR_CLEAR_CM);
    bool frClear = (fr < 0.5f || fr > FRONT_IR_CLEAR_CM);
    return flClear && frClear;
}

// All three front sensors (FL, FR, forward US) clear with GAP_MARGIN_CM of room.
// Turret must be pointing forward for the US reading to be meaningful.
static bool allFrontClearMargin() {
    float fl = sensors.readIRFrontLeft();
    float fr = sensors.readIRFrontRight();
    float us = sensors.pingNowCm();
    bool flOk = (fl < 0.5f || fl > GAP_MARGIN_CM);
    bool frOk = (fr < 0.5f || fr > GAP_MARGIN_CM);
    bool usOk = (us < 0.5f || us > GAP_MARGIN_CM);
    return flOk && frOk && usOk;
}

static void pointTurret(bool toRight) {
    int angle = toRight ? TURRET_RIGHT : TURRET_LEFT;
    turret_motor.write(angle);
    Serial.print(F("[TURRET] → ")); Serial.print(toRight ? F("RIGHT") : F("LEFT"));
    Serial.print(F(" (")); Serial.print(angle); Serial.println(F("°)"));
}

static void beginStrafe(bool goRight) {
    strafeRight        = goRight;
    strafeStartMs      = millis();
    corridorBlockCount = 0;
    prevUsSide         = -1.0f;
    prevRearIR         = -1.0f;
    Serial.print(F("[STRAFE] Begin ")); Serial.println(goRight ? F("RIGHT") : F("LEFT"));
    if (goRight) motors.strafeRight();
    else         motors.strafeLeft();
}

// ═════════════════════════════════════════════════════════════════════════════
// doApproach — drive at flame with layered obstacle avoidance.
// ═════════════════════════════════════════════════════════════════════════════
static void doApproach() {
    float heading = sensors.getGyroHeading();

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

        // ── AP_DRIVE: turret forward, front IR + forward US watched ──────────
        case AP_DRIVE: {
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

            float fl = sensors.readIRFrontLeft();
            float fr = sensors.readIRFrontRight();
            float us = sensors.pingNowCm();
            bool flBlk = (fl > 0.5f && fl < FRONT_IR_TRIGGER_CM);
            bool frBlk = (fr > 0.5f && fr < FRONT_IR_TRIGGER_CM);
            bool usBlk = (us > 0.5f && us < US_FWD_TRIGGER_CM);

            if (flBlk || frBlk || usBlk) {
                motors.stop();
                ptFireCounter = 0;
                detectHeading = sensors.getGyroHeading();

                // FL blocked → strafe RIGHT; FR blocked → strafe LEFT;
                // US-only → default LEFT.
                bool preferRight;
                if (flBlk)      preferRight = true;
                else if (frBlk) preferRight = false;
                else            preferRight = false;

                Serial.print(F("[AP_DRIVE] Obstacle FL=")); Serial.print(fl, 1);
                Serial.print(F(" FR=")); Serial.print(fr, 1);
                Serial.print(F(" US=")); Serial.print(us, 1);
                Serial.print(F(" → check ")); Serial.println(preferRight ? F("RIGHT") : F("LEFT"));

                sideCheckFallback = false;
                strafeRight       = preferRight;
                sideCheckSettleMs = millis();
                pointTurret(preferRight);
                approachSub = AP_SIDE_CHECK;
            }
            break;
        }

        // ── AP_SIDE_CHECK: turret at candidate side, one US reading ──────────
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
                strafeRight       = !strafeRight;     // flip to other side (full 180°)
                sideCheckSettleMs = millis();
                Serial.println(F("[SIDE_CHECK] Blocked — trying other side"));
                pointTurret(strafeRight);
            } else {
                Serial.println(F("[SIDE_CHECK] Both blocked → SPIN to find gap"));
                turret_motor.write(TURRET_FWD);
                gapClearCount = 0;
                motors.rotateClockwise();
                approachSub = AP_SPIN_GAP;
            }
            break;
        }

        // ── AP_STRAFING: strafe until front clears (debounced) ───────────────
        case AP_STRAFING: {
            float usSide = sensors.pingNowCm();
            float rearIR = strafeRight ? sensors.readIRRearRight()
                                       : sensors.readIRRearLeft();

            bool usSettled = (millis() - strafeStartMs >= STRAFE_US_SETTLE_MS);

            bool usValid = true, rearValid = true;
            if (prevUsSide > 0.5f && usSide > 0.5f &&
                fabsf(usSide - prevUsSide) > SENSOR_JUMP_REJECT_CM) usValid = false;
            if (prevRearIR > 0.5f && rearIR > 0.5f &&
                fabsf(rearIR - prevRearIR) > SENSOR_JUMP_REJECT_CM) rearValid = false;

            bool usBlocked   = usSettled && usValid &&
                               (usSide > 0.5f && usSide < SIDE_US_BLOCKED_CM);
            bool rearBlocked = rearValid &&
                               (rearIR > 0.5f && rearIR < REAR_IR_BLOCKED_CM);

            if (usValid)   prevUsSide = usSide;
            if (rearValid) prevRearIR = rearIR;

            bool timedOut = (millis() - strafeStartMs >= STRAFE_TIMEOUT_MS);

            static unsigned long lastStr = 0;
            if (millis() - lastStr > 200) {
                lastStr = millis();
                Serial.print(F("[STRAFE] usSide=")); Serial.print(usSide, 1);
                Serial.print(F(" rearIR="));         Serial.print(rearIR, 1);
                Serial.print(F(" blkCnt="));         Serial.print(corridorBlockCount);
                Serial.print(F(" frontClear="));     Serial.println(frontIsClear());
            }

            if (usBlocked || rearBlocked) {
                corridorBlockCount++;
                if (corridorBlockCount >= CORRIDOR_BLOCK_TICKS) {
                    motors.stop();
                    Serial.print(F("[STRAFE] Corridor blocked x"));
                    Serial.print(corridorBlockCount);
                    Serial.println(F(" → reassess"));
                    turret_motor.write(TURRET_FWD);
                    motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                    approachSub = AP_DRIVE;
                    break;
                }
            } else {
                corridorBlockCount = 0;
            }

            if (frontIsClear()) {
                Serial.println(F("[STRAFE] Front clear → extra strafe"));
                strafeExtraMs = millis();
                approachSub = AP_STRAFE_EXTRA;
                break;
            }

            if (timedOut) {
                motors.stop();
                Serial.println(F("[STRAFE] Timeout → settle"));
                turret_motor.write(TURRET_FWD);
                settleStartMs    = millis();
                settleClearCount = 0;
                approachSub = AP_SETTLE;
            }
            break;
        }

        // ── AP_STRAFE_EXTRA: keep strafing a bit to fully pass the obstacle ──
        case AP_STRAFE_EXTRA: {
            float usSide = sensors.pingNowCm();
            float rearIR = strafeRight ? sensors.readIRRearRight()
                                       : sensors.readIRRearLeft();

            bool usValid = true, rearValid = true;
            if (prevUsSide > 0.5f && usSide > 0.5f &&
                fabsf(usSide - prevUsSide) > SENSOR_JUMP_REJECT_CM) usValid = false;
            if (prevRearIR > 0.5f && rearIR > 0.5f &&
                fabsf(rearIR - prevRearIR) > SENSOR_JUMP_REJECT_CM) rearValid = false;

            bool usBlocked   = usValid && (usSide > 0.5f && usSide < SIDE_US_BLOCKED_CM);
            bool rearBlocked = rearValid && (rearIR > 0.5f && rearIR < REAR_IR_BLOCKED_CM);
            if (usValid)   prevUsSide = usSide;
            if (rearValid) prevRearIR = rearIR;

            if (usBlocked || rearBlocked) {
                corridorBlockCount++;
                if (corridorBlockCount >= CORRIDOR_BLOCK_TICKS) {
                    motors.stop();
                    Serial.println(F("[EXTRA] Corridor blocked → reassess"));
                    turret_motor.write(TURRET_FWD);
                    motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                    approachSub = AP_DRIVE;
                    break;
                }
            } else {
                corridorBlockCount = 0;
            }

            if (millis() - strafeExtraMs >= STRAFE_EXTRA_MS) {
                motors.stop();
                Serial.println(F("[EXTRA] Done → settle"));
                turret_motor.write(TURRET_FWD);   // US must face forward to confirm clear
                settleStartMs    = millis();
                settleClearCount = 0;
                approachSub = AP_SETTLE;
            }
            break;
        }

        // ── AP_SETTLE: motors stopped. Wait SETTLE_PAUSE_MS for readings to
        //    stabilise, then require all front sensors clear (margin) for
        //    SETTLE_CLEAR_TICKS consecutive ticks before driving forward.
        //    If the front isn't clear, go back to AP_DRIVE to re-evaluate.
        case AP_SETTLE: {
            motors.stop();   // hold still
            if (millis() - settleStartMs < SETTLE_PAUSE_MS) break;

            bool clear = allFrontClearMargin();
            if (clear) settleClearCount++;
            else       settleClearCount = 0;

            static unsigned long lastSet = 0;
            if (millis() - lastSet > 150) {
                lastSet = millis();
                Serial.print(F("[SETTLE] clear=")); Serial.print(clear);
                Serial.print(F(" cnt="));           Serial.println(settleClearCount);
            }

            if (settleClearCount >= SETTLE_CLEAR_TICKS) {
                Serial.println(F("[SETTLE] Front confirmed clear → drive forward"));
                postStrafeMs = millis();
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_POST_STRAFE;
                break;
            }

            // Still blocked after the pause — re-evaluate from AP_DRIVE
            // (but only after giving it a reasonable window to clear)
            if (millis() - settleStartMs > (SETTLE_PAUSE_MS + 600)) {
                Serial.println(F("[SETTLE] Still blocked → re-evaluate (AP_DRIVE)"));
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_DRIVE;
            }
            break;
        }

        // ── AP_POST_STRAFE: drive forward past obstacle, then re-seek flame ──
        case AP_POST_STRAFE: {
            float fl = sensors.readIRFrontLeft();
            float fr = sensors.readIRFrontRight();
            float us = sensors.pingNowCm();
            bool flBlk = (fl > 0.5f && fl < FRONT_IR_TRIGGER_CM);
            bool frBlk = (fr > 0.5f && fr < FRONT_IR_TRIGGER_CM);
            bool usBlk = (us > 0.5f && us < US_FWD_TRIGGER_CM);

            if (flBlk || frBlk || usBlk) {
                motors.stop();
                detectHeading = sensors.getGyroHeading();
                bool preferRight = flBlk ? true : false;
                Serial.print(F("[POST] Obstacle mid-drive → check "));
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
                Serial.println(F("[POST] Forward done → turret seek"));
                approachSub = AP_DRIVE;
                tseekInited = false;
                subState    = FS_TURRET_SEEK;
            }
            break;
        }

        // ── AP_SPIN_GAP: rotate CW up to GAP_SPIN_MAX_DEG looking for a gap ──
        // Gap = FL, FR and forward US all clear (margin) for GAP_CONFIRM_TICKS.
        // CW rotation DECREASES gyro heading; spun = (detectHeading - cur).
        case AP_SPIN_GAP: {
            float cur = sensors.getGyroHeading();
            float spun = detectHeading - cur;
            while (spun < 0.0f)    spun += 360.0f;
            while (spun >= 360.0f) spun -= 360.0f;

            bool clear = allFrontClearMargin();
            if (clear) gapClearCount++;
            else       gapClearCount = 0;

            static unsigned long lastSpin = 0;
            if (millis() - lastSpin > 150) {
                lastSpin = millis();
                Serial.print(F("[SPIN] spun=")); Serial.print(spun, 0);
                Serial.print(F(" clear="));      Serial.print(clear);
                Serial.print(F(" cnt="));        Serial.println(gapClearCount);
            }

            if (gapClearCount >= GAP_CONFIRM_TICKS) {
                motors.stop();
                Serial.println(F("[SPIN] Gap found → drive through"));
                postStrafeMs = millis();
                motors.SetDriveStraightTarget(FORWARD, sensors.getGyroHeading(), 0.0f, 0.0f);
                approachSub = AP_GAP_FWD;
                break;
            }

            if (spun >= GAP_SPIN_MAX_DEG) {
                motors.stop();
                Serial.println(F("[SPIN] No gap in 90° → return heading then reverse"));
                returnTargetHeading = detectHeading;
                afterGapFwd = false;
                approachSub = AP_RETURN_HEADING;
            }
            break;
        }

        // ── AP_GAP_FWD: drive forward through the gap ────────────────────────
        case AP_GAP_FWD: {
            float fl = sensors.readIRFrontLeft();
            float fr = sensors.readIRFrontRight();
            float us = sensors.pingNowCm();
            bool blocked = (fl > 0.5f && fl < FRONT_IR_TRIGGER_CM) ||
                           (fr > 0.5f && fr < FRONT_IR_TRIGGER_CM) ||
                           (us > 0.5f && us < US_FWD_TRIGGER_CM);
            if (blocked) {
                motors.stop();
                Serial.println(F("[GAP_FWD] Re-blocked → spin again"));
                gapClearCount = 0;
                detectHeading = sensors.getGyroHeading();
                motors.rotateClockwise();
                approachSub = AP_SPIN_GAP;
                break;
            }

            motors.driveStraight(sensors.getGyroHeading(), 0.0f, 0.0f);

            if (millis() - postStrafeMs >= POST_STRAFE_FWD_MS) {
                motors.stop();
                Serial.println(F("[GAP_FWD] Done → return to detection heading"));
                returnTargetHeading = detectHeading;
                afterGapFwd = true;
                approachSub = AP_RETURN_HEADING;
            }
            break;
        }

        // ── AP_RETURN_HEADING: turn body back to returnTargetHeading ─────────
        case AP_RETURN_HEADING: {
            static bool turnArmed = false;
            if (!turnArmed) {
                motors.SetTurnTarget(returnTargetHeading);
                turnArmed = true;
                Serial.print(F("[RETURN] target=")); Serial.println(returnTargetHeading);
            }
            if (motors.isTurnComplete(sensors.getGyroHeading())) {
                turnArmed = false;
                motors.stop();
                if (afterGapFwd) {
                    // Spun CW to find the gap → flame is now to our LEFT → seek left
                    Serial.println(F("[RETURN] At heading → turret-seek (flame LEFT)"));
                    strafeRight = true;        // doTurretSeek sweeps toward LEFT
                    tseekInited = false;
                    approachSub = AP_DRIVE;
                    subState    = FS_TURRET_SEEK;
                } else {
                    Serial.println(F("[RETURN] At heading → reverse"));
                    nogapReverseStartCm = sensors.pingNowCm();
                    nogapReverseMs      = millis();
                    motors.driveReverse();
                    approachSub = AP_NOGAP_REVERSE;
                }
            }
            break;
        }

        // ── AP_NOGAP_REVERSE: back up NOGAP_REVERSE_CM ───────────────────────
        case AP_NOGAP_REVERSE: {
            float usFwd = sensors.pingNowCm();
            bool distReached = (usFwd > 0.5f && (usFwd - nogapReverseStartCm) >= NOGAP_REVERSE_CM);
            bool timedOut    = (millis() - nogapReverseMs >= REVERSE_TIMEOUT_MS);

            static unsigned long lastNR = 0;
            if (millis() - lastNR > 200) {
                lastNR = millis();
                Serial.print(F("[NOGAP_REV] US=")); Serial.print(usFwd, 1);
                Serial.print(F(" delta="));         Serial.println(usFwd - nogapReverseStartCm, 1);
            }

            if (distReached || timedOut) {
                motors.stop();
                Serial.println(F("[NOGAP_REV] Done → long strafe LEFT"));
                strafeRight   = false;
                nogapStrafeMs = millis();
                motors.strafeLeft();
                approachSub = AP_NOGAP_STRAFE;
            }
            break;
        }

        // ── AP_NOGAP_STRAFE: long strafe, then re-find flame ─────────────────
        case AP_NOGAP_STRAFE: {
            if (millis() - nogapStrafeMs >= NOGAP_STRAFE_MS) {
                motors.stop();
                Serial.println(F("[NOGAP_STRAFE] Done → turret-seek"));
                strafeRight = false;       // strafed left → seek right
                tseekInited = false;
                approachSub = AP_DRIVE;
                subState    = FS_TURRET_SEEK;
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
// Non-blocking clearance scanner (centre arc 50–130°) — kept for compatibility
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