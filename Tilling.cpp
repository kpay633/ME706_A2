#include "Tilling.h"
#include "Motor.h"
#include "Sensors.h"

extern Motor motors;
extern Sensors sensors;

// --- Exact Constants from tilling.ino ---
#define STOP_DIST_IR_MM       105.0f
#define IR_NOISE_FLOOR_MM      50.0f
#define ARENA_DONE_US_CM       11.0f
#define TARGET_US_START_CM     101.0f
#define TARGET_US_END_CM        5.5f
#define LANE_WIDTH_CM         ((TARGET_US_START_CM - TARGET_US_END_CM) / 9)

#define DRIVE_SPEED             400
#define STRAFE_SPEED            150
#define SETTLE_TIME_MS          150
#define STRAFE_TIMEOUT_MS      3000
#define YAW_ALIGN_TIMEOUT_MS   3000
#define PRE_ALIGN_TIMEOUT_MS   5000
#define PRE_ALIGN_TOLERANCE_CM  2.0f
#define YAW_ALIGN_TOL_DEG       3.0f
#define YAW_ALIGN_KP            4.0f
#define YAW_ALIGN_MIN_SPD        60
#define YAW_ALIGN_MAX_SPD       220

// --- State Machine ---
enum TillingPhase {
    INIT, PRE_ALIGN, FORWARD_PASS, STRAFE_AFTER_FWD, SETTLE,
    SETTLE_AFTER_FWD, YAW_REALIGN_FWD, REVERSE_PASS,
    STRAFE_AFTER_REV, SETTLE_AFTER_REV, YAW_REALIGN_REV
};

static TillingPhase phase = INIT;
static TillingPhase nextPhaseAfterSettle = FORWARD_PASS;
static float startYaw = 0.0f;
static float targetUSDist = TARGET_US_START_CM;
static int laneCount = 0;
static unsigned long timer = 0;

// Helper from ino
static float wrapAngle(float e) {
    while (e > 180.0f)  e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

STEP tilling() {
    float currentYaw = sensors.getGyroHeading();
    float frontDist_mm = (float)sensors.readLongRangeIR2(); // FRONT IR
    float rearDist_mm = (float)sensors.readLongRangeIR1();  // REAR IR
    float usDist_cm = sensors.readUltrasonicCm();

    // Fallback if US fails (matches ino logic)
    float rightDist_cm = (usDist_cm <= 0) ? targetUSDist : usDist_cm;
    if (rightDist_cm > 125.0f) rightDist_cm = targetUSDist; // reject HC-SR04 max-range ghost



    switch (phase) {
        case INIT:
            DUAL_PRINTLN(F("TILLING: Starting (Original Logic)"));
            sensors.setUSStrictFilter(true);
            sensors.WarmUSFilter(20);
            laneCount = 0;
            targetUSDist = TARGET_US_START_CM;
            startYaw = currentYaw;
            timer = millis();
            phase = PRE_ALIGN;
            break;

        case PRE_ALIGN: {
            static bool strafeTargetSet = false;

            if (!strafeTargetSet) {
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, 0, 0);
                motors.setSpeed(STRAFE_SPEED);
                strafeTargetSet = true;
            }

            bool strafing = motors.strafeToUSDist(TARGET_US_START_CM, rightDist_cm, currentYaw, 0);

            if (!strafing || (millis() - timer >= PRE_ALIGN_TIMEOUT_MS)) {
                strafeTargetSet = false;
                motors.stop();
                delay(100);
                sensors.WarmUSFilter(10);
                startYaw = sensors.getGyroHeading();
                motors.setSpeed(DRIVE_SPEED);
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, targetUSDist, STOP_DIST_IR_MM);
                phase = FORWARD_PASS;
            }
            break;
        }

        case SETTLE:
            if (millis() - timer >= 100) {
                phase = nextPhaseAfterSettle;
            }
            break;

        case FORWARD_PASS: {
            static bool targetSet = false;
            if (!targetSet) {
                motors.setSpeed(DRIVE_SPEED);
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, targetUSDist, STOP_DIST_IR_MM);
                targetSet = true;
            }
            motors.driveStraight(currentYaw, rightDist_cm, frontDist_mm);
            if (frontDist_mm > IR_NOISE_FLOOR_MM && frontDist_mm <= STOP_DIST_IR_MM) {
                motors.stop();
                motors.ResetDriveStraightTarget();
                targetSet = false;
                if (usDist_cm > 0 && usDist_cm <= ARENA_DONE_US_CM) { return STEP::DEBUG; }
                timer = millis();
                nextPhaseAfterSettle = STRAFE_AFTER_FWD;
                phase = SETTLE;
            }
            break;
        }

        case STRAFE_AFTER_FWD: {
            static bool strafeTargetSet = false;
            float nextTarget = targetUSDist - LANE_WIDTH_CM;

            if (!strafeTargetSet) {
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, 0, 0);
                motors.setSpeed(STRAFE_SPEED);
                strafeTargetSet = true;
            }

            bool strafing = motors.strafeToUSDist(nextTarget, rightDist_cm, currentYaw, 0);
            if (!strafing) {
                strafeTargetSet = false;
                laneCount++;
                targetUSDist -= LANE_WIDTH_CM;
                timer = millis();
                nextPhaseAfterSettle = REVERSE_PASS;
                phase = SETTLE;
            }
            break;
        }

        case SETTLE_AFTER_FWD:
            if (millis() - timer >= SETTLE_TIME_MS) {
                timer = millis();
                phase = YAW_REALIGN_FWD;
            }
            break;

        case YAW_REALIGN_FWD: {
            float yawErr = wrapAngle(startYaw - currentYaw);
            if (fabsf(yawErr) <= YAW_ALIGN_TOL_DEG || (millis() - timer >= YAW_ALIGN_TIMEOUT_MS)) {
                motors.stop();
                laneCount++;
                targetUSDist -= LANE_WIDTH_CM;
                motors.setSpeed(DRIVE_SPEED);
                motors.SetDriveStraightTarget(DIRECTION::REVERSE, startYaw, targetUSDist, STOP_DIST_IR_MM);
                phase = REVERSE_PASS;
            } else {
                int rotSpd = constrain((int)(fabsf(yawErr) * YAW_ALIGN_KP), YAW_ALIGN_MIN_SPD, YAW_ALIGN_MAX_SPD);
                motors.setSpeed(rotSpd);
                if (yawErr > 0) motors.rotateClockwise();
                else            motors.rotateCounterClockwise();
            }
            break;
        }

        case REVERSE_PASS: {
            static bool targetSet = false;
            if (!targetSet) {
                motors.setSpeed(DRIVE_SPEED);
                motors.SetDriveStraightTarget(DIRECTION::REVERSE, startYaw, targetUSDist, STOP_DIST_IR_MM);
                targetSet = true;
            }
            motors.driveStraight(currentYaw, rightDist_cm, rearDist_mm);
            if (rearDist_mm > IR_NOISE_FLOOR_MM && rearDist_mm <= STOP_DIST_IR_MM) {
                motors.stop();
                motors.ResetDriveStraightTarget();
                targetSet = false;
                if (usDist_cm > 0 && usDist_cm <= ARENA_DONE_US_CM) { return STEP::DEBUG; }
                timer = millis();
                nextPhaseAfterSettle = STRAFE_AFTER_REV;
                phase = SETTLE;
            }
            break;
        }

        case STRAFE_AFTER_REV: {
            static bool strafeTargetSet = false;
            float nextTarget = targetUSDist - LANE_WIDTH_CM;

            if (!strafeTargetSet) {
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, 0, 0);
                motors.setSpeed(STRAFE_SPEED);
                strafeTargetSet = true;
            }

            bool strafing = motors.strafeToUSDist(nextTarget, rightDist_cm, currentYaw, 0);
            if (!strafing) {
                strafeTargetSet = false;
                laneCount++;
                targetUSDist -= LANE_WIDTH_CM;
                timer = millis();
                nextPhaseAfterSettle = FORWARD_PASS;
                phase = SETTLE;
            }
            break;
        }

        case SETTLE_AFTER_REV:
            if (millis() - timer >= SETTLE_TIME_MS) {
                timer = millis();
                //phase = YAW_REALIGN_REV;

                //Rhys's test - to go back, remove these next 5 rows and uncomment above line
                laneCount++;
                targetUSDist -= LANE_WIDTH_CM;
                motors.setSpeed(DRIVE_SPEED);
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, targetUSDist, STOP_DIST_IR_MM);
                phase = FORWARD_PASS;
            }
            break;

        case YAW_REALIGN_REV: {
            float yawErr = wrapAngle(startYaw - currentYaw);
            if (fabsf(yawErr) <= YAW_ALIGN_TOL_DEG || (millis() - timer >= YAW_ALIGN_TIMEOUT_MS)) {
                motors.stop();
                laneCount++;
                targetUSDist -= LANE_WIDTH_CM;
                motors.setSpeed(DRIVE_SPEED);
                motors.SetDriveStraightTarget(DIRECTION::FORWARD, startYaw, targetUSDist, STOP_DIST_IR_MM);
                phase = FORWARD_PASS;
            } else {
                int rotSpd = constrain((int)(fabsf(yawErr) * YAW_ALIGN_KP), YAW_ALIGN_MIN_SPD, YAW_ALIGN_MAX_SPD);
                motors.setSpeed(rotSpd);
                if (yawErr > 0) motors.rotateClockwise();
                else            motors.rotateCounterClockwise();
            }
            break;
        }
    }
    return STEP::TILLING;
}
