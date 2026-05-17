#include "fire.h"
#include "Motor.h"
#include "Sensors.h"
#include "sweep.h"

extern Motor   motors;
extern Sensors sensors;


// ── Sweep config ──────────────────────────────────────────────────────────────
#define SWEEP_BUF_SIZE   300
#define SWEEP_SPEED       70
#define HOTSPOT_THRESH   150.0f
#define MIN_PEAK_GAP      25.0f

#define PT_LEFT_PIN    A0       // replace with your actual pins
#define PT_CENTRE_PIN  A1
#define PT_RIGHT_PIN   A2

struct Hotspot     { float angle, intensity; bool valid = false; };
struct SweepSample { float angle, ptL, ptC, ptR; };

static Hotspot     hotspots[2];
static SweepSample sweepBuf[SWEEP_BUF_SIZE];
static int         sweepCount    = 0;
static bool        sweepInited   = false;

static float fusePT(float L, float C, float R);
static void detectHotspots();


// ── Approach / Avoid config ──────────────────────────────────────────────────
#define APPROACH_SPEED        60
#define STRAFE_SPEED          60
#define FIRE_CLOSE_DIST       12.0f   // cm — stop and align when this close
#define OBSTACLE_DIST         20.0f   // cm — ultrasonic trigger threshold
#define STRAFE_DURATION_MS   400UL    // ms to strafe sideways past the obstacle
#define CLEAR_DURATION_MS    500UL    // ms to drive forward to clear the obstacle
#define REALIGN_DURATION_MS  400UL    // ms to strafe back and re-centre on flame



// ── Internal sub-states ──────────────────────
enum FireSubState {
    FS_SWEEP,
    FS_TURN,
    FS_APPROACH,
    FS_AVOID,
    FS_ALIGN,
    FS_EXTINGUISH,
    FS_DONE
};

static FireSubState subState = FS_SWEEP;
static int          fireIndex = 0;      // 0 = first fire, 1 = second

// ── Avoid sub-states ─────────────────────────────────────────────────────────
enum AvoidPhase {
    AV_STRAFE_CLEAR,   // step 1: strafe away from obstacle
    AV_FORWARD_CLEAR,  // step 2: drive forward until obstacle is cleared
    AV_STRAFE_BACK     // step 3: strafe back to re-align with flame
};

static AvoidPhase avoidPhase;
static bool       avoidLeft;          // true = strafe left, false = strafe right
static uint32_t   avoidPhaseStart;    // millis() timestamp for timed phases


// ── Forward declarations of sub-state functions ───────────────────────────────
static STEP doSweep();
static STEP doTurn();
static STEP doApproach();
static STEP doAvoid();
static STEP doAlign();
static STEP doExtinguish();

// ── Public entry point ────────────────────────────────────────────────────────
void resetFireRoutine() {
    subState    = FS_SWEEP;
    fireIndex   = 0;
    sweepInited = false;    // replaces resetSweep()
    sweepCount  = 0;
}

STEP runFireRoutine() {
    STEP result = CURRENT_STEP;

    switch (subState) {
        case FS_SWEEP:      result = doSweep();      break;
        case FS_TURN:       result = doTurn();        break;
        case FS_APPROACH:   result = doApproach();    break;
        case FS_AVOID:      result = doAvoid();       break;
        case FS_ALIGN:      result = doAlign();       break;
        case FS_EXTINGUISH: result = doExtinguish();  break;
        case FS_DONE:       return NEXT_STEP;          // signal main we're finished
    }

    // ── Sub-state transitions ─────────────────────────────────────────────────
    if (result == NEXT_STEP) {
        switch (subState) {
            case FS_SWEEP:      subState = FS_TURN;        break;
            case FS_TURN:       subState = FS_APPROACH;    break;
            case FS_APPROACH:   subState = FS_ALIGN;       break;
            case FS_AVOID:      subState = FS_APPROACH;    break;   // resume approach
            case FS_ALIGN:      subState = FS_EXTINGUISH;  break;
            case FS_EXTINGUISH:
                if (fireIndex < 1) {
                    fireIndex++;               // move to second fire
                    subState = FS_TURN;
                } else {
                    subState = FS_DONE;        // both fires out — next tick returns NEXT_STEP
                }
                break;
            default: break;
        }
    } else if (result == AVOID_STEP) {
        subState = FS_AVOID;
    }

    return CURRENT_STEP;    // tell main we're still working
}


// ── IMPLEMENTATION ────────────────

static STEP doSweep() {
    if (!sweepInited) {
        sensors.ZeroGyroHeading();
        sweepCount  = 0;
        sweepInited = true;
        motors.setSpeed(-SWEEP_SPEED, SWEEP_SPEED);
    }

    float heading = sensors.getGyroHeading();

    if (heading >= 358.0f || sweepCount >= SWEEP_BUF_SIZE) {
        motors.stop();
        detectHotspots();
        sweepInited = false;
        return NEXT_STEP;       // → FS_TURN
    }

    sweepBuf[sweepCount++] = {
        heading,
        (float)(1023 - analogRead(PT_LEFT_PIN)),
        (float)(1023 - analogRead(PT_CENTRE_PIN)),
        (float)(1023 - analogRead(PT_RIGHT_PIN))
    };

    return CURRENT_STEP;
}

static STEP doTurn() {
    // turnToFire() logic here
    return CURRENT_STEP;
}

static STEP doApproach() {
    float dist = sensors.getUltrasonicDistance();   // cm

    if (dist > 0.0f && dist < OBSTACLE_DIST) {
        // obstacle detected — hand off to doAvoid
        motors.stop();
        return AVOID_STEP;
    }

    if (dist > 0.0f && dist <= FIRE_CLOSE_DIST) {
        // close enough — proceed to alignment
        motors.stop();
        return NEXT_STEP;
    }

    // nothing in the way — drive straight toward the flame
    motors.setSpeed(APPROACH_SPEED, APPROACH_SPEED);
    return CURRENT_STEP;
}

static STEP doAvoid() {
    // ── Initialise on first entry ──────────────────────────────────────────
    static bool inited = false;
    if (!inited) {
        // choose strafe direction based on which IR sensor sees more open space
        // higher reading = more reflected light = wall is closer, so pick the
        // side with the *lower* reading (more space)
        avoidLeft  = (sensors.readLeftIR() > sensors.readRightIR());
        avoidPhase = AV_STRAFE_CLEAR;
        avoidPhaseStart = millis();
        inited = true;
    }

    uint32_t elapsed = millis() - avoidPhaseStart;

    switch (avoidPhase) {

        // ── Phase 1: strafe sideways to get off the obstacle's line ──────
        case AV_STRAFE_CLEAR:
            if (avoidLeft)
                motors.setSpeed(-STRAFE_SPEED, STRAFE_SPEED);   // strafe left
            else
                motors.setSpeed(STRAFE_SPEED, -STRAFE_SPEED);   // strafe right

            if (elapsed >= STRAFE_DURATION_MS) {
                motors.stop();
                avoidPhase      = AV_FORWARD_CLEAR;
                avoidPhaseStart = millis();
            }
            break;

        // ── Phase 2: drive forward until we're past the obstacle ────────
        case AV_FORWARD_CLEAR:
            motors.setSpeed(APPROACH_SPEED, APPROACH_SPEED);

            if (elapsed >= CLEAR_DURATION_MS) {
                motors.stop();
                avoidPhase      = AV_STRAFE_BACK;
                avoidPhaseStart = millis();
            }
            break;

        // ── Phase 3: strafe back to re-centre on the flame ──────────────
        case AV_STRAFE_BACK:
            if (avoidLeft)
                motors.setSpeed(STRAFE_SPEED, -STRAFE_SPEED);   // strafe right (undo)
            else
                motors.setSpeed(-STRAFE_SPEED, STRAFE_SPEED);   // strafe left  (undo)

            if (elapsed >= REALIGN_DURATION_MS) {
                motors.stop();
                inited = false;   // reset for next obstacle encounter
                return NEXT_STEP; // back to FS_APPROACH
            }
            break;
    }

    return CURRENT_STEP;
}

static STEP doAlign() {
    // alignToFire() logic here
    return CURRENT_STEP;
}

static STEP doExtinguish() {
    // extinguish() logic here
    return CURRENT_STEP;
}






















static float fusePT(float L, float C, float R) {
    return 0.25f * L + 0.50f * C + 0.25f * R;
}

static void detectHotspots() {
    hotspots[0] = hotspots[1] = {0, 0, false};

    for (int i = 0; i < sweepCount; i++) {
        float val = fusePT(sweepBuf[i].ptL, sweepBuf[i].ptC, sweepBuf[i].ptR);
        if (val < HOTSPOT_THRESH) continue;

        for (int h = 0; h < 2; h++) {
            if (!hotspots[h].valid) {
                hotspots[h] = { sweepBuf[i].angle, val, true };
                break;
            }
            float diff = fabsf(sweepBuf[i].angle - hotspots[h].angle);
            if (diff < MIN_PEAK_GAP) {
                if (val > hotspots[h].intensity)
                    hotspots[h] = { sweepBuf[i].angle, val, true };
                break;
            }
            if (h == 1 && val > hotspots[1].intensity)
                hotspots[1] = { sweepBuf[i].angle, val, true };
        }
    }
}
