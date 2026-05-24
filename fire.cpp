#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"

extern Motor   motors;
extern Sensors sensors;

#define PT_LEFT_PIN    A2
#define PT_CENTRE_PIN  A3
#define PT_RIGHT_PIN   A4

#define SWEEP_SPEED     60
#define TURN_SPEED      60
#define TURN_TOLERANCE   5.0f
#define APPROACH_SPEED  70
#define CLOSE_THRESH   50

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
    // Print current substate once per 500ms so we can see if it's stuck
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

// ─────────────────────────────────────────────────────────────────────────────
// doSweep
// ─────────────────────────────────────────────────────────────────────────────
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

    // Don't check for completion until we've actually rotated past 10°
    // This prevents the gyro wrap-at-zero from triggering a false finish
    if (!sweepStarted && heading > 10.0f && heading < 350.0f) {
        sweepStarted = true;
        Serial.println(F("[SWEEP] Rotation confirmed, now watching for 358°"));
    }

    int ptL = analogRead(PT_LEFT_PIN);
    int ptC = analogRead(PT_CENTRE_PIN);
    int ptR = analogRead(PT_RIGHT_PIN);
    int fused = (int)(0.25f * ptL + 0.50f * ptC + 0.25f * ptR);

    // Print sensor readings every 200ms so we can see them without flooding
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

// ─────────────────────────────────────────────────────────────────────────────
// doTurn
// ─────────────────────────────────────────────────────────────────────────────
static void doTurn() {
    if (!hotspot.valid) {
        Serial.println(F("[TURN] No valid hotspot — skipping to APPROACH"));
        subState = FS_APPROACH;
        return;
    }

    motors.SetTurnTarget(targetAngle);
    targetSet = true;

    if (motors.isTurnComplete(sensors.getGyroHeading())) {
        // Serial.print("Gyro: "); 
        // Serial.println(sensors.getGyroHeading());
        targetSet = false;
    } else {
        motors.PrintDetails();
    }
    break;
}

    // float current = sensors.getGyroHeading();
    // float error   = hotspot.angle - current;

    // while (error >  180.0f) error -= 360.0f;
    // while (error < -180.0f) error += 360.0f;

    // static unsigned long lastTurnPrint = 0;
    // if (millis() - lastTurnPrint > 200) {
    //     lastTurnPrint = millis();
    //     Serial.print(F("[TURN] target="));
    //     Serial.print(hotspot.angle);
    //     Serial.print(F("  current="));
    //     Serial.print(current);
    //     Serial.print(F("  error="));
    //     Serial.println(error);
    // }

    // if (fabsf(error) <= TURN_TOLERANCE) {
    //     motors.stop();
    //     Serial.println(F("[TURN] Within tolerance — moving to APPROACH"));
    //     subState = FS_APPROACH;
    //     return;
    // }

    // if (error > 0) {
    //     Serial.println(F("[TURN] Rotating counter-clockwise"));
    //     motors.rotateCounterClockwise();
    // } else {
    //     Serial.println(F("[TURN] Rotating clockwise"));
    //     motors.rotateClockwise();
    // }
}

// ─────────────────────────────────────────────────────────────────────────────
// doApproach
// ─────────────────────────────────────────────────────────────────────────────
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