#include "main.h"
#include "fire.h"
#include <Servo.h>
#include "Motor.h"
#include "Sensors.h"

extern Motor   motors;
extern Sensors sensors;
extern Servo    turret_motor;

#define PT_LEFT_PIN    A2
#define PT_CENTRE_PIN  A3
#define PT_RIGHT_PIN   A4

#define SWEEP_SPEED     60
#define TURN_SPEED      60
#define TURN_TOLERANCE   3.0f
#define APPROACH_SPEED  70
#define CLOSE_THRESH   10

static constexpr float CLEARANCE_WIDTH_MM  = 250.0f;
static constexpr float CLEARANCE_DEPTH_MM  = 400.0f;
static constexpr float SENSOR_OFFSET_MM    = 50.0f;
static constexpr float CLEARANCE_HALF_WIDTH_MM = CLEARANCE_WIDTH_MM * 0.5f;
static constexpr int    SCAN_START_DEG      = 30;
static constexpr int    SCAN_END_DEG        = 150;
static constexpr int    SCAN_STEP_DEG       = 6;
static constexpr uint16_t SERVO_SETTLE_MS   = 13;
static constexpr float   DEG_TO_RAD_SCALE   = 3.14159265f / 180.0f;

struct Hotspot { float angle; int intensity; bool valid = false; };
static Hotspot hotspot;

enum FireSubState {
    FS_SWEEP, FS_TURN, FS_APPROACH,
    FS_AVOID, FS_ALIGN, FS_EXTINGUISH, FS_DONE
};
static FireSubState subState    = FS_SWEEP;
static int          fireIndex   = 0;
static bool         sweepInited = false;
static bool         sweepStarted = false;
static bool         targetSet = false;

static void doSweep();
static void doTurn();
static void doApproach();
static void doAvoid();
static void doAlign();
static void doExtinguish();

void resetFireRoutine() {
    subState    = FS_SWEEP;
    fireIndex   = 0;
    sweepInited = false;
}

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
            Serial.println(F("[FSM] FS_DONE reached — returning NEXT_STEP to main"));
            return NEXT_STEP;
    }
    return CURRENT_STEP;
}

static void doSweep() {
    if (!sweepInited) {
        Serial.println(F("[SWEEP] Init — zeroing gyro, starting rotation"));
        hotspot.angle     = 0.0f;
        hotspot.intensity = 1023;
        hotspot.valid     = false;
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

    int ptL = analogRead(PT_LEFT_PIN);
    int ptC = analogRead(PT_CENTRE_PIN);
    int ptR = analogRead(PT_RIGHT_PIN);
    int fused = (int)(0.25f * ptL + 0.50f * ptC + 0.25f * ptR);

    static unsigned long lastSweepPrint = 0;
    if (millis() - lastSweepPrint > 200) {
        lastSweepPrint = millis();
        Serial.print(F("[SWEEP] heading="));
        Serial.print(heading);
        Serial.print(F("  ptL="));  Serial.print(ptL);
        Serial.print(F("  ptC="));  Serial.print(ptC);
        Serial.print(F("  ptR="));  Serial.print(ptR);
        Serial.print(F("  fused=")); Serial.print(fused);
        Serial.print(F("  bestSoFar="));
        Serial.print(hotspot.intensity);
        Serial.print(F(" @ "));
        Serial.println(hotspot.angle);
    }

    if (fused < hotspot.intensity) {
        Serial.print(F("[SWEEP] New best! fused="));
        Serial.print(fused);
        Serial.print(F(" at heading="));
        Serial.println(heading);
        hotspot.angle     = heading;
        hotspot.intensity = fused;
        hotspot.valid     = true;
    }

    if (sweepStarted && heading >= 358.0f) {
        motors.stop();
        sweepInited = false;
        Serial.println(F("[SWEEP] Complete."));
        Serial.print(F("[SWEEP] Hotspot found: valid="));
        Serial.print(hotspot.valid);
        Serial.print(F("  angle="));
        Serial.print(hotspot.angle);
        Serial.print(F("  intensity="));
        Serial.println(hotspot.intensity);
        subState = FS_TURN;
    }
}

static void doTurn() {
    if (!hotspot.valid) {
        Serial.println(F("[TURN] No valid hotspot — skipping to APPROACH"));
        subState = FS_APPROACH;
        return;
    }

    if (!targetSet) {
        motors.SetTurnTarget(hotspot.angle);
        targetSet = true;
        Serial.print(F("[TURN] Target set to "));
        Serial.println(hotspot.angle);
    }

    if (motors.isTurnComplete(sensors.getGyroHeading())) {
        Serial.println(F("[TURN] Complete — moving to APPROACH"));
        targetSet = false;
        subState  = FS_APPROACH;
    }
}

static void doApproach() {
    int ptC = analogRead(PT_CENTRE_PIN);

    static unsigned long lastApproachPrint = 0;
    if (millis() - lastApproachPrint > 200) {
        lastApproachPrint = millis();
        Serial.print(F("[APPROACH] ptC="));
        Serial.print(ptC);
        Serial.print(F("  threshold="));
        Serial.println(CLOSE_THRESH);
    }

    if (ptC <= CLOSE_THRESH) {
        motors.stop();
        Serial.println(F("[APPROACH] Close enough — moving to DONE"));
        subState = FS_DONE;
        return;
    }

    motors.driveForward();
}

static void doAvoid()      { Serial.println(F("[AVOID] stub")); }
static void doAlign()      { Serial.println(F("[ALIGN] stub")); }
static void doExtinguish() { Serial.println(F("[EXTINGUISH] stub")); }

// ─────────────────────────────────────────────────────────────────────────────
// Blocking clearance scan — geometry fixed
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

        float distanceMm = distanceCm * 10.0f;
        float bearingDeg = (float)servoAngleDeg - 90.0f;  // 0° = straight ahead
        float bearingRad = bearingDeg * DEG_TO_RAD_SCALE;

        // Robot-frame coords: forward = cos(bearing), lateral = sin(bearing)
        float fwdMm     = SENSOR_OFFSET_MM + distanceMm * cosf(bearingRad);
        float lateralMm = distanceMm * sinf(bearingRad);

        bool withinForwardBox =
            fwdMm >= 0.0f &&
            fwdMm <= CLEARANCE_DEPTH_MM &&
            fabsf(lateralMm) <= CLEARANCE_HALF_WIDTH_MM;

        if (withinForwardBox && distanceMm < closestDistanceMm) {
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
// Non-blocking clearance scanner — geometry fixed
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
        float distanceMm = distanceCm * 10.0f;
        float bearingDeg = (float)nb_scanAngleDeg - 90.0f;  // 0° = straight ahead
        float bearingRad = bearingDeg * DEG_TO_RAD_SCALE;

        // Robot-frame coords: forward = cos(bearing), lateral = sin(bearing)
        float fwdMm     = SENSOR_OFFSET_MM + distanceMm * cosf(bearingRad);
        float lateralMm = distanceMm * sinf(bearingRad);

        bool withinForwardBox =
            fwdMm >= 0.0f &&
            fwdMm <= CLEARANCE_DEPTH_MM &&
            fabsf(lateralMm) <= CLEARANCE_HALF_WIDTH_MM;

        if (withinForwardBox && distanceMm < nb_closestDistMm) {
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