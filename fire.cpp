#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"

extern Motor   motors;
extern Sensors sensors;

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PT_LEFT_PIN    A2
#define PT_CENTRE_PIN  A3
#define PT_RIGHT_PIN   A4

// ── Tuning ────────────────────────────────────────────────────────────────────
#define SWEEP_SPEED     60
#define TURN_SPEED      60   // these are unused rn


#define TURN_TOLERANCE            5.0f
#define APPROACH_SPEED            180
#define STRAFE_SPEED              180

#define CLOSE_THRESH              300     // PT centre raw threshold: lower means close/bright enough
#define FRONT_OBSTACLE_CM         25.0f   // obstacle detected in front if ultrasonic < this
#define MAX_STRAFE_TIME_MS        900     // tune so this is about 20 cm sideways on your robot

#define PARTIAL_SCAN_LIMIT_DEG    75.0f   // one-sided relock scan
#define RELIGHT_TURN_TOLERANCE    5.0f

// ── Hotspot ───────────────────────────────────────────────────────────────────
// intensity here is a RAW analogRead value — LOWER means brighter/hotter
struct Hotspot { float angle; int intensity; bool valid = false; };
static Hotspot hotspot;

// ── Sub-states ────────────────────────────────────────────────────────────────
enum FireSubState {
    FS_SWEEP, FS_TURN, FS_APPROACH,
    FS_AVOID, FS_ALIGN, FS_EXTINGUISH, FS_DONE
};
static FireSubState subState    = FS_SWEEP;
static int          fireIndex   = 0;
static bool         sweepInited = false;

enum AvoidPhase {
    AV_CHOOSE_SIDE,
    AV_STRAFE,
    AV_RELOCK_SCAN,
    AV_TURN_TO_LIGHT
  };

static AvoidPhase avoidPhase = AV_CHOOSE_SIDE;
static bool avoidInited = false;
static bool avoidLeft = true;
static unsigned long avoidStartMs = 0;
static Hotspot relockHotspot = {0.0f, 1023, false};

// ── Forward declarations ──────────────────────────────────────────────────────
static void doSweep();
static void doTurn();
static void doApproach();
static void doAvoid();
static void doAlign();
static void doExtinguish();

// ─────────────────────────────────────────────────────────────────────────────
// Public
// ─────────────────────────────────────────────────────────────────────────────

void resetFireRoutine() {
    subState    = FS_SWEEP;
    fireIndex   = 0;
    sweepInited = false;
}

STEP runFireRoutine() {
    switch (subState) {
        case FS_SWEEP:      doSweep();      break;
        case FS_TURN:       doTurn();        break;
        case FS_APPROACH:   doApproach();    break;
        case FS_AVOID:      doAvoid();       break;
        case FS_ALIGN:      doAlign();       break;
        case FS_EXTINGUISH: doExtinguish();  break;
        case FS_DONE:       return NEXT_STEP;
    }
    return CURRENT_STEP;
}

// ─────────────────────────────────────────────────────────────────────────────
// doSweep — spin 360°, record the heading where raw PT read was lowest
// ─────────────────────────────────────────────────────────────────────────────

static void doSweep() {
    
    if (!sweepInited) {
        Serial.println(F("doSweep initing"));
        hotspot.angle     = 0.0f;
        hotspot.intensity = 1023;
        hotspot.valid     = false;        
        sweepInited  = true;
        sensors.ZeroGyroHeading();
        motors.rotateClockwise();
        return;
    }

    float heading = sensors.getGyroHeading();
    Serial.print(F("Heading: "));
    Serial.println(heading);


    if (heading == 180.0f) {
        motors.stop();
        sweepInited = false;
        subState    = FS_TURN;
        return;
    }

    // Raw reads — lower value means brighter fire, no inversion needed
    int ptL = analogRead(PT_LEFT_PIN);
    Serial.print(F("PT_LEFT: "));
    Serial.println(ptL);
    int ptC = analogRead(PT_CENTRE_PIN);
    Serial.print(F("PT_CENTRE: "));
    Serial.println(ptC);
    int ptR = analogRead(PT_RIGHT_PIN);
    Serial.print(F("PT_RIGHT: "));
    Serial.println(ptR);

    // Fuse: weight centre most heavily, keep in raw domain (lower = brighter)
    int fused = (int)(0.25f * ptL + 0.50f * ptC + 0.25f * ptR);

    // Track the minimum — lowest raw value = brightest spot seen so far
    if (fused < hotspot.intensity) {
        hotspot.angle     = heading;
        hotspot.intensity = fused;
        hotspot.valid     = true;
    }
}


// helper function for turning

static float wrapAngleError(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}


// ─────────────────────────────────────────────────────────────────────────────
// doTurn — spin to face the stored hotspot heading
// ─────────────────────────────────────────────────────────────────────────────

static void doTurn() {
    if (!hotspot.valid) { subState = FS_APPROACH; return; }
    Serial.println(F("doTurn starting"));

    float error = hotspot.angle - sensors.getGyroHeading();

    while (error >  180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    if (fabsf(error) <= TURN_TOLERANCE) {
        motors.stop();
        subState = FS_APPROACH;
        return;
    }

    if (error > 0) motors.rotateClockwise();
    else           motors.rotateCounterClockwise();
}

// ─────────────────────────────────────────────────────────────────────────────
// doApproach — drive forward until PTC raw read drops low enough
// ─────────────────────────────────────────────────────────────────────────────

static void doApproach() {
    int ptC = analogRead(PT_CENTRE_PIN);
    float us = sensors.readUltrasonicCm();

    if (ptC <= CLOSE_THRESH) {
        motors.stop();
        subState = FS_DONE;
        return;
    }

    if (us > 0.0f && us < FRONT_OBSTACLE_CM) {
        motors.stop();
        avoidInited = false;
        avoidPhase = AV_CHOOSE_SIDE;
        subState = FS_AVOID;
        return;
    }

    motors.setSpeed(APPROACH_SPEED);
    motors.driveForward();
}

static void doAvoid() {
  if (!avoidInited) {
    avoidPhase = AV_CHOOSE_SIDE;
    relockHotspot.angle = 0.0f;
    relockHotspot.intensity = 1023;
    relockHotspot.valid = false;
    avoidInited = true;
  }

  switch (avoidPhase) {
    case AV_CHOOSE_SIDE: {
      uint16_t leftSpace  = sensors.readLongRangeIR1();
      uint16_t rightSpace = sensors.readLongRangeIR2();

      // IR sensors are distance sensors: larger value = more space
      avoidLeft = (leftSpace >= rightSpace);

      motors.setSpeed(STRAFE_SPEED);
      avoidStartMs = millis();
      avoidPhase = AV_STRAFE;
      return;
    }

    case AV_STRAFE: {
      if (avoidLeft) motors.strafeLeft();
      else           motors.strafeRight();

      if (millis() - avoidStartMs >= MAX_STRAFE_TIME_MS) {
        motors.stop();

        sensors.ZeroGyroHeading();
        relockHotspot.angle = 0.0f;
        relockHotspot.intensity = 1023;
        relockHotspot.valid = false;

        motors.setSpeed(TURN_SPEED);

        // strafed left => light is now on the right
        if (avoidLeft) motors.rotateClockwise();
        else           motors.rotateCounterClockwise();

        avoidPhase = AV_RELOCK_SCAN;
      }
      return;
    }

    case AV_RELOCK_SCAN: {
      float heading = sensors.getGyroHeading();

      int ptL = analogRead(PT_LEFT_PIN);
      int ptC = analogRead(PT_CENTRE_PIN);
      int ptR = analogRead(PT_RIGHT_PIN);
      int fused = (int)(0.25f * ptL + 0.50f * ptC + 0.25f * ptR);

      // PT sensors: lower value = brighter
      if (fused < relockHotspot.intensity) {
        relockHotspot.angle = heading;
        relockHotspot.intensity = fused;
        relockHotspot.valid = true;
      }

      if (heading >= PARTIAL_SCAN_LIMIT_DEG) {
        motors.stop();

        if (relockHotspot.valid) {
          hotspot = relockHotspot;
          avoidPhase = AV_TURN_TO_LIGHT;
        } else {
          avoidInited = false;
          subState = FS_SWEEP;   // fallback if partial scan fails
        }
      }
      return;
    }

    case AV_TURN_TO_LIGHT: {
      float error = wrapAngleError(hotspot.angle, sensors.getGyroHeading());

      if (fabsf(error) <= TURN_TOLERANCE) {
        motors.stop();
        avoidInited = false;
        subState = FS_APPROACH;
        return;
      }

      motors.setSpeed(TURN_SPEED);

      if (error > 0) motors.rotateClockwise();
      else           motors.rotateCounterClockwise();
      return;
    }
  }
}

static void doAlign()      { }
static void doExtinguish() { }

// ─────────────────────────────────────────────────────────────────────────────
// Implement later
// ─────────────────────────────────────────────────────────────────────────────


// // ── Avoid state variables ─────────────────────────────────────────────────────
// enum AvoidPhase { AV_STRAFE_CLEAR, AV_FORWARD_CLEAR, AV_STRAFE_BACK };
// static AvoidPhase avoidPhase;
// static uint32_t   avoidPhaseStart;
// static bool       avoidLeft;



// static STEP doAvoid() {
//     // ── Initialise on first entry ──────────────────────────────────────────
//     static bool inited = false;
//     if (!inited) {
//         // choose strafe direction based on which IR sensor sees more open space
//         // higher reading = more reflected light = wall is closer, so pick the
//         // side with the *lower* reading (more space)
//         avoidLeft  = (sensors.readLeftIR() > sensors.readRightIR());
//         avoidPhase = AV_STRAFE_CLEAR;
//         avoidPhaseStart = millis();
//         inited = true;
//     }

//     uint32_t elapsed = millis() - avoidPhaseStart;

//     switch (avoidPhase) {

//         // ── Phase 1: strafe sideways to get off the obstacle's line ──────
//         case AV_STRAFE_CLEAR:
//             if (avoidLeft)
//                 motors.setSpeed(-STRAFE_SPEED, STRAFE_SPEED);   // strafe left
//             else
//                 motors.setSpeed(STRAFE_SPEED, -STRAFE_SPEED);   // strafe right

//             if (elapsed >= STRAFE_DURATION_MS) {
//                 motors.stop();
//                 avoidPhase      = AV_FORWARD_CLEAR;
//                 avoidPhaseStart = millis();
//             }
//             break;

//         // ── Phase 2: drive forward until we're past the obstacle ────────
//         case AV_FORWARD_CLEAR:
//             motors.setSpeed(APPROACH_SPEED, APPROACH_SPEED);

//             if (elapsed >= CLEAR_DURATION_MS) {
//                 motors.stop();
//                 avoidPhase      = AV_STRAFE_BACK;
//                 avoidPhaseStart = millis();
//             }
//             break;

//         // ── Phase 3: strafe back to re-centre on the flame ──────────────
//         case AV_STRAFE_BACK:
//             if (avoidLeft)
//                 motors.setSpeed(STRAFE_SPEED, -STRAFE_SPEED);   // strafe right (undo)
//             else
//                 motors.setSpeed(-STRAFE_SPEED, STRAFE_SPEED);   // strafe left  (undo)

//             if (elapsed >= REALIGN_DURATION_MS) {
//                 motors.stop();
//                 inited = false;   // reset for next obstacle encounter
//                 return NEXT_STEP; // back to FS_APPROACH
//             }
//             break;
//     }

//     return CURRENT_STEP;
// }

// static STEP doAlign() {
//     // alignToFire() logic here
//     return CURRENT_STEP;
// }

// static STEP doExtinguish() {
//     // extinguish() logic here
//     return CURRENT_STEP;
// }
