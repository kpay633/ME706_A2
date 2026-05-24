#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"
#include <Servo.h>

extern Motor   motors;
extern Sensors sensors;
extern Servo   turret_motor;

#define PT_LEFT_PIN    A2
#define PT_CENTRE_PIN  A3
#define PT_RIGHT_PIN   A4

#define SWEEP_SPEED     60
#define TURN_SPEED      60
#define TURN_TOLERANCE   5.0f
#define APPROACH_SPEED  70
#define CLOSE_THRESH   300
#define APPROACH_IR_STOP_CM 10.0f  // stop when front IR reads ≤ this many cm

#define SWEEP_BINS        360
#define SWEEP_UNSAMPLED   0xFFFFu
#define PT_FILTER_SAMPLES 10
#define CENTROID_THRESH   0.7f
#define TURRET_LOCK_DEG   90

// PT mounting offset (deg, CCW positive).
//   With the servo locked at TURRET_LOCK_DEG, this is the angle the PT
//   physically faces relative to the robot's FRONT.
//     PT points forward  → 0    (normal case)
//     PT points right    → 90
//     PT points rear     → 180
//     PT points left     → 270
//   Use the diagnostic log: compare the printed "brightest-bin heading" to
//   the gyro heading the robot's front had when PT was facing the source.
//   The difference (mod 360) is HEADING_OFFSET.
#define HEADING_OFFSET    5.0f

struct Hotspot { float angle; int intensity; bool valid = false; };
static Hotspot hotspot;

static uint16_t sweepSamples[SWEEP_BINS];
static float    ptBuf[PT_FILTER_SAMPLES];
static uint8_t  ptBufIdx;
static float    ptBufSum;
static uint8_t  ptBufCount;

enum FireSubState {
    FS_SWEEP, FS_TURN, FS_APPROACH,
    FS_AVOID, FS_ALIGN, FS_EXTINGUISH, FS_DONE
};
static FireSubState subState    = FS_SWEEP;
static int          fireIndex   = 0;
static bool         sweepInited = false;
static bool         sweepStarted = false;

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
// doSweep — lock the turret straight ahead, rotate the whole robot 360°,
// then locate the brightest direction by normalised-centroid (circular mean)
// on the centre PT signal.
// ─────────────────────────────────────────────────────────────────────────────
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid);

static void doSweep() {
    if (!sweepInited) {
        Serial.println(F("[SWEEP] Init — locking turret @ 90°, zeroing gyro, starting rotation"));

        turret_motor.write(TURRET_LOCK_DEG);

        hotspot.angle     = 0.0f;
        hotspot.intensity = 1023;
        hotspot.valid     = false;

        for (int i = 0; i < SWEEP_BINS; i++) sweepSamples[i] = SWEEP_UNSAMPLED;

        ptBufIdx = 0;
        ptBufSum = 0.0f;
        ptBufCount = 0;
        // Prime rolling average so the very first stored bin isn't noisy.
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

    // Don't check for completion until we've actually rotated past 10°.
    // Prevents the gyro wrap-at-zero from triggering a false finish.
    if (!sweepStarted && heading > 10.0f && heading < 350.0f) {
        sweepStarted = true;
        Serial.println(F("[SWEEP] Rotation confirmed, now watching for 358°"));
    }

    // Update rolling average with newest PT reading.
    float newVal = (float)analogRead(PT_CENTRE_PIN);
    ptBufSum -= ptBuf[ptBufIdx];
    ptBuf[ptBufIdx] = newVal;
    ptBufSum += newVal;
    ptBufIdx = (ptBufIdx + 1) % PT_FILTER_SAMPLES;
    float avg = ptBufSum / (float)PT_FILTER_SAMPLES;

    // Store filtered reading at the integer-degree bin for the current heading.
    int bin = (int)heading;
    if (bin < 0) bin = 0;
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

        // PT mounting offset → convert "PT-pointed-at-source" heading to
        // "robot-front-pointed-at-source" heading.
        float targetAngle = rawAngle + HEADING_OFFSET;
        while (targetAngle >= 360.0f) targetAngle -= 360.0f;
        while (targetAngle < 0.0f)    targetAngle += 360.0f;

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

// Normalised circular-mean centroid over bins above CENTROID_THRESH.
// Falls back to the single brightest bin if no region passes the threshold.
// Wrap-safe: uses atan2(Σnorm·sin, Σnorm·cos) so a peak straddling 0/360° is handled.
static float computeHotspotAngle(uint16_t &outMin, uint16_t &outMax, bool &outValid) {
    uint16_t globalMin = 1023;
    uint16_t globalMax = 0;
    int minBin = -1, maxBin = -1;
    int sampledCount = 0;

    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        sampledCount++;
        if (sweepSamples[i] < globalMin) { globalMin = sweepSamples[i]; minBin = i; }
        if (sweepSamples[i] > globalMax) { globalMax = sweepSamples[i]; maxBin = i; }
    }

    outMin = globalMin;
    outMax = globalMax;

    if (sampledCount == 0) { outValid = false; return 0.0f; }

    float range = (float)(globalMax - globalMin);

    float sumCos  = 0.0f;
    float sumSin  = 0.0f;
    float bestNorm = 0.0f;
    int   bestBin  = -1;
    int   binsAboveThresh = 0;

    for (int i = 0; i < SWEEP_BINS; i++) {
        if (sweepSamples[i] == SWEEP_UNSAMPLED) continue;
        // Lower raw = brighter PT signal, so invert against globalMax.
        float norm = (range > 0.01f)
                     ? (float)(globalMax - sweepSamples[i]) / range
                     : 0.0f;

        if (norm > CENTROID_THRESH) {
            binsAboveThresh++;
            float rad = (float)i * (float)PI / 180.0f;
            sumCos += norm * cosf(rad);
            sumSin += norm * sinf(rad);
        }
        if (norm > bestNorm) {
            bestNorm = norm;
            bestBin  = i;
        }
    }

    Serial.print(F("[CENTROID] samples=")); Serial.print(sampledCount);
    Serial.print(F("/360  brightestBin=")); Serial.print(minBin);
    Serial.print(F(" (raw=")); Serial.print(globalMin); Serial.print(F(")"));
    Serial.print(F("  dimmestBin="));      Serial.print(maxBin);
    Serial.print(F(" (raw=")); Serial.print(globalMax); Serial.println(F(")"));
    Serial.print(F("[CENTROID] binsAboveThresh="));
    Serial.print(binsAboveThresh);
    Serial.print(F("  peakNormBin="));
    Serial.print(bestBin);
    Serial.print(F(" (norm="));
    Serial.print(bestNorm, 3);
    Serial.println(F(")"));

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

// ─────────────────────────────────────────────────────────────────────────────
// doTurn — use the Motor class's PID turn primitive so we don't overshoot
// and oscillate around the target.
// ─────────────────────────────────────────────────────────────────────────────
static void doTurn() {
    static bool turnTargetSet = false;

    if (!hotspot.valid) {
        Serial.println(F("[TURN] No valid hotspot — skipping to DONE"));
        turnTargetSet = false;
        subState = FS_DONE;
        return;
    }

    if (!turnTargetSet) {
        Serial.print(F("[TURN] Setting PID target = "));
        Serial.println(hotspot.angle);
        motors.SetTurnTarget(hotspot.angle);
        turnTargetSet = true;
    }

    float current = sensors.getGyroHeading();

    static unsigned long lastTurnPrint = 0;
    if (millis() - lastTurnPrint > 200) {
        lastTurnPrint = millis();
        Serial.print(F("[TURN] target="));
        Serial.print(hotspot.angle);
        Serial.print(F("  current="));
        Serial.println(current);
    }

    if (motors.isTurnComplete(current)) {
        Serial.println(F("[TURN] Complete — arming drive-straight & moving to APPROACH"));
        turnTargetSet = false;
        // Gyro-only heading hold (usTarget=0 disables the US side-distance loop).
        // IR-front triggers stop at APPROACH_IR_STOP_CM.
        motors.SetDriveStraightTarget(FORWARD, hotspot.angle, 0.0f, APPROACH_IR_STOP_CM);
        subState = FS_APPROACH;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// doApproach — PID-controlled straight drive on locked-in hotspot heading.
// Pure gyro control (usTarget was set to 0); front IR triggers the stop.
// ─────────────────────────────────────────────────────────────────────────────
static void doApproach() {
    float heading = sensors.getGyroHeading();
    float irFront = (float)sensors.readLongRangeIR1();

    bool stillDriving = motors.driveStraight(heading, /*usDist=*/0.0f, irFront);

    static unsigned long lastApproachPrint = 0;
    if (millis() - lastApproachPrint > 200) {
        lastApproachPrint = millis();
        Serial.print(F("[APPROACH] heading="));
        Serial.print(heading);
        Serial.print(F("  irFront="));
        Serial.print(irFront);
        Serial.print(F(" cm  ptC="));
        Serial.println(analogRead(PT_CENTRE_PIN));
    }

    if (!stillDriving) {
        motors.stop();
        Serial.println(F("[APPROACH] Stop condition met — moving to DONE"));
        subState = FS_DONE;
    }
}

static void doAvoid()      { Serial.println(F("[AVOID] stub")); }
static void doAlign()      { Serial.println(F("[ALIGN] stub")); }
static void doExtinguish() { Serial.println(F("[EXTINGUISH] stub")); }



// kala changes here

static void sweepHotspotCW() {
    Serial.println(F("[SWEEP-CW] Sweeping clockwise 90° to relocate hotspot"));

    float startHeading = sensors.getGyroHeading();
    int   bestIntensity = 1023;
    float bestAngle     = startHeading;

    motors.rotateClockwise();

    while (true) {
        float heading = sensors.getGyroHeading();
        int   ptC     = analogRead(PT_CENTRE_PIN);

        if (ptC < bestIntensity) {
            bestIntensity = ptC;
            bestAngle     = heading;
        }

        // How far have we swept CW from start
        float swept = heading - startHeading;
        if (swept < 0.0f) swept += 360.0f;
        if (swept >= 90.0f) break;

        delay(5);
    }

    motors.stop();

    // Only update hotspot if we found something brighter
    if (bestIntensity < hotspot.intensity) {
        hotspot.angle     = bestAngle;
        hotspot.intensity = bestIntensity;
        hotspot.valid     = true;
        Serial.print(F("[SWEEP-CW] Updated hotspot: angle="));
        Serial.print(bestAngle);
        Serial.print(F("  intensity="));
        Serial.println(bestIntensity);
    } else {
        Serial.println(F("[SWEEP-CW] No improvement — keeping existing hotspot"));
    }
}

static void sweepHotspotCCW() {
    Serial.println(F("[SWEEP-CCW] Sweeping counter-clockwise 90° to relocate hotspot"));

    float startHeading  = sensors.getGyroHeading();
    int   bestIntensity = 1023;
    float bestAngle     = startHeading;

    motors.rotateCounterClockwise();

    while (true) {
        float heading = sensors.getGyroHeading();
        int   ptC     = analogRead(PT_CENTRE_PIN);

        if (ptC < bestIntensity) {
            bestIntensity = ptC;
            bestAngle     = heading;
        }

        // How far have we swept CCW from start
        float swept = startHeading - heading;
        if (swept < 0.0f) swept += 360.0f;
        if (swept >= 90.0f) break;

        delay(5);
    }

    motors.stop();

    if (bestIntensity < hotspot.intensity) {
        hotspot.angle     = bestAngle;
        hotspot.intensity = bestIntensity;
        hotspot.valid     = true;
        Serial.print(F("[SWEEP-CCW] Updated hotspot: angle="));
        Serial.print(bestAngle);
        Serial.print(F("  intensity="));
        Serial.println(bestIntensity);
    } else {
        Serial.println(F("[SWEEP-CCW] No improvement — keeping existing hotspot"));
    }
}

static void sweepBack(bool direction) {
    // After vehicle strafes to avoid object, sweep again to find the hotspot if we lost it during the strafe.
    // 0 = was strafe left, 1 = was strafe right 
    Serial.println(F("[SWEEP-BACK] Starting sweep to relocate hotspot after avoidance"));
    if (!direction) {
        sweepHotspotCW();
    } else {
        sweepHotspotCCW();
    }
}