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

// ─── Fire approach stop distance ─────────────────────────────────────────────
#define APPROACH_IR_STOP_CM   10.0f
#define PT_FIRE_THRESH        100     // PT raw ADC — below this = fire reached (tune on hardware)

// ─── Sweep tuning ─────────────────────────────────────────────────────────────
#define SWEEP_BINS          360
#define SWEEP_UNSAMPLED     0xFFFFu
#define PT_FILTER_SAMPLES   10
#define CENTROID_THRESH      0.7f
#define TURRET_LOCK_DEG     90
#define HEADING_OFFSET      -2.0f

// Full sweep covers 0–360°. Partial sweep covers a 90° arc on the side
// opposite to the strafe direction (where the fire is likely to still be).
#define PARTIAL_SWEEP_ARC_DEG  90.0f

// ─── Turret angles ────────────────────────────────────────────────────────────
// 90° = forward (default / reverse / full sweep)
// 0°  = left  side check
// 180° = right side check
#define TURRET_FWD   90
#define TURRET_LEFT  180
#define TURRET_RIGHT   0

// ─── Ultrasonic scan geometry (reduced arc since IRs cover flanks) ────────────
static constexpr float    CLEARANCE_WIDTH_MM      = 250.0f;
static constexpr float    CLEARANCE_DEPTH_MM      = 400.0f;
static constexpr float    SENSOR_OFFSET_MM        = 50.0f;
static constexpr float    CLEARANCE_HALF_WIDTH_MM = CLEARANCE_WIDTH_MM * 0.5f;
static constexpr int      SCAN_START_DEG          = 50;
static constexpr int      SCAN_END_DEG            = 130;
static constexpr int      SCAN_STEP_DEG           = 6;
static constexpr uint16_t SERVO_SETTLE_MS         = 13;
static constexpr float    DEG_TO_RAD_SCALE        = 3.14159265f / 180.0f;

// ─── Obstacle detection thresholds ───────────────────────────────────────────
static constexpr float IR_FRONT_TRIGGER_CM   =  5.5f;  // front IR triggers avoidance
static constexpr float IR_SIDE_OBSTACLE_CM   = 20.0f;  // rear IR: stop strafing
static constexpr float IR_SIDE_CLEAR_CM      = 25.0f;  // rear IR: corridor confirmed clear
static constexpr float US_SIDE_OBSTACLE_CM   = 15.0f;  // turret side-check: blocked
static constexpr float US_DETECT_THRESH_MM   = 150.0f; // US forward scan: trigger
static constexpr float US_AVOID_THRESH_MM    = 125.0f; // US forward scan: centreline limit

// ─── Reverse odometry ─────────────────────────────────────────────────────────
static constexpr float REVERSE_DIST_CM       = 30.0f;  // reverse until US increases by this
static constexpr float REVERSE_TIMEOUT_MS    = 3000;   // safety fallback

// ─── Strafe timeout fallback ──────────────────────────────────────────────────
static constexpr unsigned long STRAFE_TIMEOUT_MS = 4000;

// ─── Hotspot & sweep state ────────────────────────────────────────────────────
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
static uint8_t  ptBufCount;

// ─── Top-level FSM ────────────────────────────────────────────────────────────
enum FireSubState {
    FS_SWEEP, FS_PARTIAL_SWEEP, FS_TURN, FS_APPROACH,
    FS_AVOID, FS_ALIGN, FS_EXTINGUISH, FS_DONE
};
static FireSubState subState    = FS_SWEEP;
static int          fireIndex   = 0;
static bool         sweepInited = false;
static bool         sweepStarted = false;
static bool         targetSet   = false;

// ─── Partial sweep state ──────────────────────────────────────────────────────
// Set before entering FS_PARTIAL_SWEEP to define the arc to scan.
static float partialSweepStartDeg = 0.0f;
static float partialSweepEndDeg   = 90.0f;

// ─── Approach sub-states ──────────────────────────────────────────────────────
enum ApproachSubState {
    AP_DRIVE,           // driving forward, US scan running
    AP_SCANNING,        // US scan step in progress
    AP_SIDE_CHECK,      // turret rotated to strafe side, checking corridor
    AP_REVERSING,       // both sides blocked — reverse 30cm by US odometry
    AP_STRAFING         // strafing; rear IR + turret watching strafe direction
};
static ApproachSubState approachSub = AP_DRIVE;

// Strafe direction: true = right, false = left
static bool strafeRight = false;

// Reverse odometry: record US distance at reverse start
static float reverseStartCm   = 0.0f;
static unsigned long reverseStartMs = 0;

// Strafe start time for timeout fallback
static unsigned long strafeStartMs = 0;

static void doSweep();
static void doPartialSweep();
static void doTurn();
static void doApproach();
static void doAvoid();
static void doAlign();
static void doExtinguish();
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid);
static void  beginStrafe(bool goRight);
static void  beginSideCheck(bool checkRight);

// ─────────────────────────────────────────────────────────────────────────────
void resetFireRoutine() {
    subState     = FS_SWEEP;
    fireIndex    = 0;
    sweepInited  = false;
    sweepStarted = false;
    targetSet    = false;
    approachSub  = AP_DRIVE;
    turret_motor.write(TURRET_FWD);
}

// ─────────────────────────────────────────────────────────────────────────────
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
            case FS_AVOID:         Serial.println(F("AVOID"));         break;
            case FS_ALIGN:         Serial.println(F("ALIGN"));         break;
            case FS_EXTINGUISH:    Serial.println(F("EXTINGUISH"));    break;
            case FS_DONE:          Serial.println(F("DONE"));          break;
        }
    }
    switch (subState) {
        case FS_SWEEP:         doSweep();        break;
        case FS_PARTIAL_SWEEP: doPartialSweep(); break;
        case FS_TURN:          doTurn();          break;
        case FS_APPROACH:      doApproach();      break;
        case FS_AVOID:         doAvoid();         break;
        case FS_ALIGN:         doAlign();         break;
        case FS_EXTINGUISH:    doExtinguish();    break;
        case FS_DONE:
            Serial.println(F("[FSM] DONE"));
            return NEXT_STEP;
    }
    return CURRENT_STEP;
}

// ─────────────────────────────────────────────────────────────────────────────
// doSweep — full 360° rotation, centroid fire detection
// ─────────────────────────────────────────────────────────────────────────────
static void doSweep() {
    if (!sweepInited) {
        Serial.println(F("[SWEEP] Init — full 360"));
        turret_motor.write(TURRET_LOCK_DEG);
        hotspot = Hotspot(0.0f, 1023, false);
        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;
        ptBufIdx = 0; ptBufSum = 0.0f; ptBufCount = 0;
        for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
            float v = (float)analogRead(PT_CENTRE_PIN);
            ptBuf[i] = v; ptBufSum += v; delay(5);
        }
        ptBufCount = PT_FILTER_SAMPLES;
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
        Serial.print(F(" avg="));       Serial.println(avg, 1);
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
        Serial.print(F(" valid="));         Serial.println(valid);
        subState = FS_TURN;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// doPartialSweep — targeted sweep over a pre-set arc after obstacle avoidance.
//
// Called after strafing — we know which side the obstacle was on, so the fire
// must be somewhere in the opposite 90°. We only need to sweep that arc.
//
// partialSweepStartDeg and partialSweepEndDeg must be set before entering this
// state (set by the avoidance logic in doApproach when transitioning out).
//
// Uses the same per-bin rolling-average + centroid as the full sweep, but only
// fills the bins within the target arc. Existing bin data outside the arc is
// left as SWEEP_UNSAMPLED so computeHotspotAngle ignores it.
// ─────────────────────────────────────────────────────────────────────────────
static void doPartialSweep() {
    if (!sweepInited) {
        Serial.print(F("[PSWEEP] Init — arc "));
        Serial.print(partialSweepStartDeg);
        Serial.print(F("→")); Serial.println(partialSweepEndDeg);

        turret_motor.write(TURRET_LOCK_DEG);

        // Clear only the bins inside our arc so we get a fresh read.
        // Bins outside the arc stay SWEEP_UNSAMPLED — computeHotspotAngle
        // will ignore them correctly.
        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;

        ptBufIdx = 0; ptBufSum = 0.0f; ptBufCount = 0;
        for (uint8_t i = 0; i < PT_FILTER_SAMPLES; i++) {
            float v = (float)analogRead(PT_CENTRE_PIN);
            ptBuf[i] = v; ptBufSum += v; delay(5);
        }
        ptBufCount = PT_FILTER_SAMPLES;

        hotspot = Hotspot(0.0f, 1023, false);
        sweepStarted = false;
        sweepInited  = true;
        sensors.ZeroGyroHeading();
        motors.rotateCounterClockwise();
        return;
    }

    float heading = sensors.getGyroHeading();

    // Wait until we've actually entered the arc.
    // For arcs starting at 0, wait for heading > 5 first to avoid triggering
    // before the robot has actually started rotating.
    if (!sweepStarted) {
        bool entered = (partialSweepStartDeg < 5.0f)
                       ? (heading > 5.0f)                      // 0° arc: wait for motion
                       : (heading >= partialSweepStartDeg);    // 270° arc: wait for arc
        if (entered) {
            sweepStarted = true;
            Serial.println(F("[PSWEEP] Arc entered"));
        }
    }

    // Update rolling-average PT reading and store in bin
    float newVal = (float)analogRead(PT_CENTRE_PIN);
    ptBufSum -= ptBuf[ptBufIdx];
    ptBuf[ptBufIdx] = newVal;
    ptBufSum += newVal;
    ptBufIdx = (ptBufIdx + 1) % PT_FILTER_SAMPLES;
    float avg = ptBufSum / (float)PT_FILTER_SAMPLES;

    // Only write to bins inside the target arc
    if (heading >= partialSweepStartDeg && heading <= partialSweepEndDeg) {
        int bin = (int)heading;
        if (bin >= 0 && bin < SWEEP_BINS) sweepSamples[bin] = (uint16_t)avg;
    }

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print(F("[PSWEEP] hdg=")); Serial.print(heading);
        Serial.print(F(" avg="));        Serial.println(avg, 1);
    }

    // Done once we've passed the end of the arc
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
        Serial.print(F(" valid="));          Serial.println(valid);

        // Fall back to full sweep if partial found nothing
        if (!valid) {
            Serial.println(F("[PSWEEP] No hotspot in arc — falling back to full sweep"));
            subState = FS_SWEEP;
        } else {
            subState = FS_TURN;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
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
    Serial.print(F(" peak=")); Serial.print(minBin);
    Serial.print(F(" above=")); Serial.print(binsAbove);
    Serial.print(F(" norm=")); Serial.println(bestNorm, 3);

    if (sumCos != 0.0f || sumSin != 0.0f) {
        float deg = atan2f(sumSin, sumCos) * 180.0f / (float)PI;
        if (deg < 0.0f) deg += 360.0f;
        outValid = true; return deg;
    }
    if (bestBin >= 0) { outValid = true; return (float)bestBin; }
    outValid = false; return 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// doTurn — PID turn to hotspot heading
// ─────────────────────────────────────────────────────────────────────────────
static void doTurn() {
    if (!hotspot.valid) {
        Serial.println(F("[TURN] No hotspot"));
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
        Serial.println(F("[TURN] Done → APPROACH"));
        targetSet = false;
        approachSub = AP_DRIVE;
        turret_motor.write(TURRET_FWD);
        motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
        subState = FS_APPROACH;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: check both front short-range IRs.
// Returns:  0 = clear
//           1 = left blocked  → should strafe right
//          -1 = right blocked → should strafe left
//           2 = both blocked  → need to reverse first, then strafe right
//          -2 = both blocked, right closer → reverse first, then strafe left
// ─────────────────────────────────────────────────────────────────────────────
static int evalFrontIR() {
    float fl = sensors.readIRFrontLeft();
    float fr = sensors.readIRFrontRight();
    bool leftBlocked  = (fl > 0.5f && fl < IR_FRONT_TRIGGER_CM);
    bool rightBlocked = (fr > 0.5f && fr < IR_FRONT_TRIGGER_CM);

    if (leftBlocked && rightBlocked)
        return (fl <= fr) ? 2 : -2;   // strafe away from the closer side
    if (leftBlocked)  return  1;
    if (rightBlocked) return -1;
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: begin the turret side-check phase.
// Rotates the turret to face the intended strafe direction and records
// which direction is being checked so AP_SIDE_CHECK knows what to do.
// ─────────────────────────────────────────────────────────────────────────────
static void beginSideCheck(bool checkRight) {
    strafeRight = checkRight;
    turret_motor.write(checkRight ? TURRET_RIGHT : TURRET_LEFT);
    Serial.print(F("[SIDE_CHECK] Turret → "));
    Serial.println(checkRight ? F("RIGHT (180°)") : F("LEFT (0°)"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Helper: start strafing. Turret stays pointing to strafe side for continuous
// corridor monitoring. Rear IR on strafe side is also watched.
// ─────────────────────────────────────────────────────────────────────────────
static void beginStrafe(bool goRight) {
    strafeRight   = goRight;
    strafeStartMs = millis();
    // Turret already pointing to strafe side from AP_SIDE_CHECK — leave it there
    if (goRight) motors.strafeRight();
    else         motors.strafeLeft();
    Serial.print(F("[STRAFE] → "));
    Serial.println(goRight ? F("RIGHT") : F("LEFT"));
}

// ─────────────────────────────────────────────────────────────────────────────
// doApproach — gyro-locked drive with layered obstacle avoidance
//
// Detection priority (checked every tick unless in AP_SIDE_CHECK / AP_REVERSING):
//  1. Fire reached via front-left IR ≤ APPROACH_IR_STOP_CM
//  2. Front IR corner sensors — immediate response, no scan needed
//  3. US forward scan (AP_SCANNING) — catches wider obstacles beyond IR range
//
// Avoidance sequence:
//  AP_DRIVE → obstacle detected → AP_SIDE_CHECK (turret rotates, one shot check)
//      → corridor clear    → AP_STRAFING
//      → corridor blocked  → check other side
//          → other side clear  → AP_STRAFING (other direction)
//          → both blocked      → AP_REVERSING (30cm by US odometry)
//                                  → AP_SIDE_CHECK on less-blocked side
//
// During AP_STRAFING:
//  - Rear long IR on strafe side watched continuously
//  - Turret US on strafe side watched continuously
//  - Strafe stops when both clear, OR timeout fires
//  - After strafe: FS_PARTIAL_SWEEP on the side opposite to the strafe direction
//
// During AP_REVERSING:
//  - Turret snapped to TURRET_FWD
//  - US reads distance; stop when distance increases by REVERSE_DIST_CM
// ─────────────────────────────────────────────────────────────────────────────

// Internal state for the two-phase side check (check preferred, then fallback)
static bool sideCheckFallback       = false;  // true when checking second side
static unsigned long sideCheckSettleMs = 0;
static constexpr uint16_t SIDE_CHECK_SETTLE_MS = 30; // ms for turret to settle

static void doApproach() {
    float heading = sensors.getGyroHeading();
    float irFL    = sensors.readIRFrontLeft();

    // ── 1. Fire reached — PT sensor drops below threshold when close to flame ─
    // Corner IR is NOT used for fire-reached; it is obstacle-avoidance only.
    int ptReading = analogRead(PT_CENTRE_PIN);
    if (ptReading < PT_FIRE_THRESH) {
        motors.stop();
        turret_motor.write(TURRET_FWD);
        Serial.print(F("[AP] Fire reached (PT="));
        Serial.print(ptReading);
        Serial.println(F(") → DONE"));
        approachSub = AP_DRIVE;
        subState    = FS_DONE;
        return;
    }

    static unsigned long lastLog = 0;
    if (millis() - lastLog > 200) {
        lastLog = millis();
        Serial.print(F("[AP] sub=")); Serial.print((int)approachSub);
        Serial.print(F(" FL="));     Serial.print(irFL, 1);
        Serial.print(F(" FR="));     Serial.print(sensors.readIRFrontRight(), 1);
        Serial.print(F(" RL="));     Serial.print(sensors.readIRRearLeft(),   1);
        Serial.print(F(" RR="));     Serial.println(sensors.readIRRearRight(), 1);
    }

    switch (approachSub) {

        // ── AP_DRIVE ──────────────────────────────────────────────────────────
        case AP_DRIVE: {
            if (!motors.driveStraight(heading, 0.0f, 0.0f)) {
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
            }
            int action = evalFrontIR();
            if (action != 0) {
                motors.stop();
                // Determine preferred strafe side and initiate side-check
                bool preferRight = (action == 1 || action == 2);
                sideCheckFallback = false;
                sideCheckSettleMs = millis();
                beginSideCheck(preferRight);
                approachSub = AP_SIDE_CHECK;
                break;
            }
            // No IR trigger — start US scan
            clearanceScanStart();
            approachSub = AP_SCANNING;
            break;
        }

        // ── AP_SCANNING ───────────────────────────────────────────────────────
        case AP_SCANNING: {
            motors.driveStraight(heading, 0.0f, 0.0f);

            // Front IR still takes priority mid-scan
            int action = evalFrontIR();
            if (action != 0) {
                motors.stop();
                bool preferRight = (action == 1 || action == 2);
                sideCheckFallback = false;
                sideCheckSettleMs = millis();
                beginSideCheck(preferRight);
                approachSub = AP_SIDE_CHECK;
                break;
            }

            ObstacleClearanceResult scanResult;
            if (clearanceScanStep(scanResult)) {
                if (!scanResult.isClear &&
                    fabsf(scanResult.closestObstacleOffsetMm) < US_AVOID_THRESH_MM &&
                    scanResult.closestObstacleDistanceMm      < US_DETECT_THRESH_MM)
                {
                    motors.stop();
                    // Obstacle on the right of centreline → strafe left, and vice versa
                    bool preferRight = (scanResult.closestObstacleOffsetMm < 0.0f);
                    sideCheckFallback = false;
                    sideCheckSettleMs = millis();
                    beginSideCheck(preferRight);
                    approachSub = AP_SIDE_CHECK;
                } else {
                    approachSub = AP_DRIVE;
                }
            }
            break;
        }

        // ── AP_SIDE_CHECK ─────────────────────────────────────────────────────
        // Turret has been pointed to the intended strafe side.
        // Wait for it to settle, then take a single US reading.
        // If clear → begin strafe.
        // If blocked → check the other side (sideCheckFallback).
        // If both blocked → begin reverse.
        case AP_SIDE_CHECK: {
            if (millis() - sideCheckSettleMs < SIDE_CHECK_SETTLE_MS) break;

            float usSide = sensors.pingNowCm();
            bool sideBlocked = (usSide > 0.5f && usSide < US_SIDE_OBSTACLE_CM);

            Serial.print(F("[SIDE_CHECK] "));
            Serial.print(strafeRight ? F("RIGHT") : F("LEFT"));
            Serial.print(F(" US=")); Serial.print(usSide, 1);
            Serial.println(sideBlocked ? F(" BLOCKED") : F(" CLEAR"));

            if (!sideBlocked) {
                // Corridor clear — go
                beginStrafe(strafeRight);
                approachSub = AP_STRAFING;
            } else if (!sideCheckFallback) {
                // First side blocked — try the other side
                sideCheckFallback = true;
                sideCheckSettleMs = millis();
                beginSideCheck(!strafeRight);  // flip direction
                Serial.println(F("[SIDE_CHECK] Trying other side"));
            } else {
                // Both sides blocked — reverse
                Serial.println(F("[SIDE_CHECK] Both sides blocked → REVERSING"));
                turret_motor.write(TURRET_FWD);
                reverseStartCm  = sensors.pingNowCm();
                reverseStartMs  = millis();
                motors.driveReverse();
                approachSub = AP_REVERSING;
            }
            break;
        }

        // ── AP_REVERSING ──────────────────────────────────────────────────────
        // Turret is forward (TURRET_FWD). Reverse until US distance increases
        // by REVERSE_DIST_CM, indicating we've moved back 30cm.
        // Timeout as fallback if US reads erratically.
        case AP_REVERSING: {
            float usFwd = sensors.pingNowCm();

            // US odometry: stop when we've increased distance by 30cm
            bool distReached = (usFwd > 0.5f &&
                                (usFwd - reverseStartCm) >= REVERSE_DIST_CM);
            bool timedOut    = (millis() - reverseStartMs >= REVERSE_TIMEOUT_MS);

            static unsigned long lastRevLog = 0;
            if (millis() - lastRevLog > 200) {
                lastRevLog = millis();
                Serial.print(F("[REV] US=")); Serial.print(usFwd, 1);
                Serial.print(F(" start=")); Serial.print(reverseStartCm, 1);
                Serial.print(F(" delta=")); Serial.println(usFwd - reverseStartCm, 1);
            }

            if (distReached || timedOut) {
                motors.stop();
                if (timedOut) Serial.println(F("[REV] Timeout"));
                else          Serial.println(F("[REV] 30cm reached"));

                // Re-check both sides to choose best strafe direction
                sideCheckFallback = false;
                sideCheckSettleMs = millis();
                // Default to right after reverse; side-check will flip if needed
                beginSideCheck(true);
                approachSub = AP_SIDE_CHECK;
            }
            break;
        }

        // ── AP_STRAFING ───────────────────────────────────────────────────────
        // Turret points to strafe side — reads US corridor continuously.
        // Rear IR on strafe side also monitored.
        // Stop when front IRs read clear AND rear IR / turret US are clear.
        // Timeout as safety fallback.
        case AP_STRAFING: {
            float usSide  = sensors.pingNowCm();  // turret still pointed to strafe side
            float rearIR  = strafeRight
                            ? sensors.readIRRearRight()   // rear-right faces right
                            : sensors.readIRRearLeft();   // rear-left  faces left

            bool usCorridorBlocked  = (usSide  > 0.5f && usSide  < US_SIDE_OBSTACLE_CM);
            bool rearIRBlocked      = (rearIR  > 0.5f && rearIR  < IR_SIDE_OBSTACLE_CM);
            bool frontClear         = (evalFrontIR() == 0);
            bool sidesClear         = (!usCorridorBlocked && !rearIRBlocked);
            bool timedOut           = (millis() - strafeStartMs >= STRAFE_TIMEOUT_MS);

            static unsigned long lastStrLog = 0;
            if (millis() - lastStrLog > 200) {
                lastStrLog = millis();
                Serial.print(F("[STRAFE] usSide=")); Serial.print(usSide, 1);
                Serial.print(F(" rearIR="));         Serial.print(rearIR,  1);
                Serial.print(F(" frontClear="));     Serial.println(frontClear);
            }

            // New obstacle in strafe direction — stop and reassess
            if (usCorridorBlocked || rearIRBlocked) {
                motors.stop();
                Serial.println(F("[STRAFE] Obstacle in corridor — reassessing"));
                turret_motor.write(TURRET_FWD);
                motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, 0.0f);
                approachSub = AP_DRIVE;
                break;
            }

            if ((frontClear && sidesClear) || timedOut) {
                motors.stop();
                turret_motor.write(TURRET_FWD);
                if (timedOut) Serial.println(F("[STRAFE] Timeout"));
                else          Serial.println(F("[STRAFE] Clear — triggering partial sweep"));

                // Partial sweep arc is relative to a freshly-zeroed gyro (zeroed at
                // doPartialSweep init), so heading starts at 0 and increases CCW.
                // Strafed right → obstacle was left → fire is clockwise (right) of us
                //   CCW rotation reaches clockwise targets at the END of the sweep (270-360°)
                // Strafed left → obstacle was right → fire is CCW (left) of us
                //   CCW rotation reaches CCW targets at the START of the sweep (0-90°)
                if (strafeRight) {
                    partialSweepStartDeg = 270.0f;
                    partialSweepEndDeg   = 358.0f;
                } else {
                    partialSweepStartDeg = 0.0f;
                    partialSweepEndDeg   = 90.0f;
                }
                Serial.print(F("[STRAFE] Partial sweep arc: "));
                Serial.print(partialSweepStartDeg); Serial.print(F("→"));
                Serial.println(partialSweepEndDeg);

                approachSub = AP_DRIVE;
                subState    = FS_PARTIAL_SWEEP;
            }
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Stub states
// ─────────────────────────────────────────────────────────────────────────────
static void doAvoid()      { Serial.println(F("[AVOID] stub")); }
static void doAlign()      { Serial.println(F("[ALIGN] stub")); }
static void doExtinguish() { Serial.println(F("[EXTINGUISH] stub")); }

// ─────────────────────────────────────────────────────────────────────────────
// Blocking clearance scan (utility — not called by doApproach)
// ─────────────────────────────────────────────────────────────────────────────
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
        float brg = ((float)deg - 90.0f) * DEG_TO_RAD_SCALE;
        float fwd = SENSOR_OFFSET_MM + mm * cosf(brg);
        float lat = mm * sinf(brg);
        if (fwd >= 0.0f && fwd <= CLEARANCE_DEPTH_MM &&
            fabsf(lat) <= CLEARANCE_HALF_WIDTH_MM && mm < closestMm) {
            closestMm = mm;
            result.isClear = false;
            result.closestObstacleOffsetMm   = lat;
            result.closestObstacleDistanceMm = mm;
        }
    }
    turret_motor.write(TURRET_FWD); delay(SERVO_SETTLE_MS);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Non-blocking clearance scanner (reduced arc 50–130°)
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
        float brg = ((float)nb_scanAngleDeg - 90.0f) * DEG_TO_RAD_SCALE;
        float fwd = SENSOR_OFFSET_MM + mm * cosf(brg);
        float lat = mm * sinf(brg);
        if (fwd >= 0.0f && fwd <= CLEARANCE_DEPTH_MM &&
            fabsf(lat) <= CLEARANCE_HALF_WIDTH_MM && mm < nb_closestDistMm) {
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