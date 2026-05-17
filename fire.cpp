#include "main.h"
#include "fire.h"
#include "Motor.h"
#include "Sensors.h"

extern Motor   motors;
extern Sensors sensors;

// ── Pins ──────────────────────────────────────────────────────────────────────
#define PT_LEFT_PIN    A1
#define PT_CENTRE_PIN  A2
#define PT_RIGHT_PIN   A3

// ── Tuning ────────────────────────────────────────────────────────────────────
#define SWEEP_SPEED     60
#define TURN_SPEED      60
#define TURN_TOLERANCE   5.0f
#define APPROACH_SPEED  70
#define CLOSE_THRESH   300     // raw analogRead — below this means close enough

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
        hotspot.angle     = 0.0f;
        hotspot.intensity = 1023;
        hotspot.valid     = false;        
        sweepInited  = true;
        sensors.ZeroGyroHeading();
        motors.rotateClockwise();
        return;
    }

    float heading = sensors.getGyroHeading();

    if (heading >= 358.0f) {
        motors.stop();
        sweepInited = false;
        subState    = FS_TURN;
        return;
    }

    // Raw reads — lower value means brighter fire, no inversion needed
    int ptL = analogRead(PT_LEFT_PIN);
    int ptC = analogRead(PT_CENTRE_PIN);
    int ptR = analogRead(PT_RIGHT_PIN);

    // Fuse: weight centre most heavily, keep in raw domain (lower = brighter)
    int fused = (int)(0.25f * ptL + 0.50f * ptC + 0.25f * ptR);

    // Track the minimum — lowest raw value = brightest spot seen so far
    if (fused < hotspot.intensity) {
        hotspot.angle     = heading;
        hotspot.intensity = fused;
        hotspot.valid     = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// doTurn — spin to face the stored hotspot heading
// ─────────────────────────────────────────────────────────────────────────────

static void doTurn() {
    if (!hotspot.valid) { subState = FS_APPROACH; return; }

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

    if (ptC <= CLOSE_THRESH) {
        motors.stop();
        subState = FS_DONE;
        return;
    }

    motors.driveForward();   // simple, no PID fighting itself
}

static void doAvoid()      { }
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