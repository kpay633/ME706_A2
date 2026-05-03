#include "main.h"
#include "Homing.h"
#include "Motor.h"
#include "Sensors.h"

extern Motor motors;
extern Sensors sensors;

// ─── Sweep config ───────────────────────────────────────────────
#define SWEEP_SAMPLE_INTERVAL  60      // ms between samples
#define SWEEP_BUF_SIZE         256     // max samples for a full 360
#define MIN_CLUSTER_DIST       1.0f    // cm — readings within this are "same wall"
#define ANGLE_CLUSTER_GAP_DEG  6.5f    // max gap between consecutive wall samples
#define INVALID_READING        -1.0f   // sentinel from readUltrasonicCm()

// ─── Platform dimensions (mm) ───────────────────────────────────
#define LONG_WALL_THRESHOLD    140.0f
#define STRAFE_LIMIT_RIGHT     118.0f
#define STRAFE_LIMIT_LEFT      2.0f

// ─── Buffers ────────────────────────────────────────────────────
static float    sweep_dist[SWEEP_BUF_SIZE];
static float    sweep_angle[SWEEP_BUF_SIZE];
static uint16_t sweep_count = 0;

struct WallResult {
  float dist;
  float angle;
};

static WallResult walls[4];
static uint8_t    walls_found = 0;

//Strafe
bool timerStarted = false;

// ─── Forward declarations ────────────────────────────────────────
static bool       doSweep();
static bool       findWallsFromBuffer();
static float      angleDiff(float a, float b);

// ════════════════════════════════════════════════════════════════
//  HOMING STATE MACHINE
// ════════════════════════════════════════════════════════════════

STEP homing() {
  static uint8_t phase     = 0;
  static uint8_t lastPhase = 255;

  if (phase != lastPhase) {
    Serial.print("--- HOMING PHASE: ");
    Serial.print(phase);
    Serial.println(" ---");
    lastPhase = phase;
  }

  switch (phase) {

    // ── Phase 0: sweep 360° and find all 4 walls ──────────────
    case 0: {
      if (!doSweep()) return STEP::HOMING;
      if (!findWallsFromBuffer()) {
        // Serial.println("Wall detection failed, retrying...");
        sweep_count = 0;
        return STEP::HOMING;
      }
      // Serial.println("Walls found and calculated:");
      // for (uint8_t i = 0; i < 4; i++) {
      //   Serial.print("  Wall "); Serial.print(i);
      //   Serial.print(" | angle: "); Serial.print(walls[i].angle, 1);
      //   Serial.print("° | dist: ");  Serial.print(walls[i].dist, 1);
      //   Serial.println(" cm");
      // }
      phase++;
      break;
    }

    // ── Phase 1: decide which way to turn ─────────────────────
    case 1: {
      static bool targetSet = false;
      float axis02 = walls[0].dist + walls[2].dist;
      // Serial.print("Axis 0/2 total dist: "); Serial.print(axis02, 1); Serial.print(" cm | ");
      float axis13 = walls[1].dist + walls[3].dist;
      // Serial.print("Axis 1/3 total dist: "); Serial.print(axis13, 1); Serial.println(" cm");

      
      if (!targetSet) {
        float targetAngle = 0.0f;
        if (axis02 >= axis13) {
          if (walls[2].dist < walls[0].dist) {
            targetAngle = walls[2].angle + 90.0f;
            // Serial.println("DEBUG: Turning towards wall 2 (Axis 0/2)");
          } else {
            targetAngle = walls[0].angle + 90.0f;
            // Serial.println("DEBUG: Turning towards wall 0 (Axis 0/2)");
          }
        }
        else if (axis13 > axis02) {
          if (walls[1].dist < walls[3].dist) {
            targetAngle = walls[1].angle + 90.0f;
            // Serial.println("DEBUG: Turning towards wall 1 (Axis 1/3)");
          } else {
            targetAngle = walls[3].angle + 90.0f;
            // Serial.println("DEBUG: Turning towards wall 3 (Axis 1/3)");
          }
        }
        
        // Serial.print("DEBUG: Setting Turn Target: "); Serial.println(targetAngle);
        motors.SetTurnTarget(targetAngle);
        targetSet = true;
      }

      if (motors.isTurnComplete(sensors.getGyroHeading())) {
        // Serial.print("DEBUG: Phase 1 Turn Complete. Final Gyro: "); 
        // Serial.println(sensors.getGyroHeading());
        targetSet = false;
        phase++;
      } else {
        motors.PrintDetails();
      }
      break;
    }



    /*
    // ── Phase 2 v1: strafe closest L/R ────────────────────────────────
    
    // strafe must be based on USS reading, as no IRs on sides.
    // back up based on short range IR on rear of vehicle.
    // large margins for the strafe left/right
    case 2: {
      static bool isBackingUp  = true;
      static bool targetSet    = false;
      if (isBackingUp) {
        if (!targetSet) {
          Serial.println("Backing into wall...");
          motors.SetDriveStraightTarget(DIRECTION::REVERSE,
                                        sensors.getGyroHeading(), 0, 50);
          targetSet = true;
          return STEP::HOMING;
        }
        if (!motors.driveStraight(sensors.getGyroHeading(),
                                  sensors.readUltrasonicCm(),
                                  sensors.readLongRangeIR1())) { // rear is IR 1 (switch is rear)
          motors.stop();
          isBackingUp = false;
          targetSet   = false;
        }
        Serial.println("Done backing...");
      }
      else {
        // Serial.println("Moving sideways to closest...");
        float rightDist = sensors.readUltrasonicCm() * 10.0f; // convert to mm
        if (rightDist > STRAFE_LIMIT_RIGHT)      { motors.stop(); phase++; Serial.println("SAFETY LIMIT REACHED - STOPPING"); return STEP::HOMING;}
        if (rightDist < STRAFE_LIMIT_LEFT)       { motors.stop(); phase++; Serial.println("SAFETY LIMIT REACHED - STOPPING"); return STEP::HOMING;}
        else if (rightDist >= (608-10))          { motors.strafeLeft(); }
        else if (rightDist < (608-10))           { motors.strafeRight(); }
      }
      break;
    }
    */




    // ── Phase 2 v2: slam into rear wall, then strafe left to left wall ──
    case 2: {
      enum class SubPhase : uint8_t {
        REVERSE_TO_WALL,
        STRAFE_LEFT_UNTIL_CLEAR,
        STRAFE_LEFT_EXTRA,
        DONE_SUB
      };
      static SubPhase sub           = SubPhase::REVERSE_TO_WALL;
      static bool     targetSet     = false;
      static unsigned long extraStartMs = 0;

      switch (sub) {

        // ── Step 1: reverse until rear IR stops us ──────────────
        case SubPhase::REVERSE_TO_WALL: {
          if (!targetSet) {
            // Serial.println("Phase 2a: Reversing into rear wall...");
            motors.SetDriveStraightTarget(DIRECTION::REVERSE,
                                          sensors.getGyroHeading(), 0, 105);
            targetSet = true;
          }
          bool done = !motors.driveStraight(sensors.getGyroHeading(),
                                            sensors.readUltrasonicCm(),
                                            sensors.readLongRangeIR1());
          if (done) {
            motors.stop();
            targetSet = false;
            // Serial.println("Phase 2a: Rear wall reached. Starting left strafe...");
            sub = SubPhase::STRAFE_LEFT_UNTIL_CLEAR;
          }
          break;
        }

        // ── Step 2: strafe left until US > 90 cm ───────────────
        case SubPhase::STRAFE_LEFT_UNTIL_CLEAR: {
          float usDist = sensors.readUltrasonicCm();
          // Serial.print("Phase 2b: Strafing left | US_R: ");
          // Serial.print(usDist, 1); Serial.println(" cm");
          if (usDist > 95.0f) {
            motors.stop();
            extraStartMs = millis();
            // Serial.println("Phase 2b: US > 90cm. Strafing extra 2s...");
            sub = SubPhase::STRAFE_LEFT_EXTRA;
          } else {
            motors.strafeLeft();
          }
          break;
        }

        // ── Step 3: continue strafing left for 2 more seconds ──
        case SubPhase::STRAFE_LEFT_EXTRA: {
          motors.setSpeed(100);
          if (millis() - extraStartMs >= 1500) {
            motors.stop();
            // Serial.println("Phase 2c: Extra strafe done.");
            sub = SubPhase::DONE_SUB;
          } else {
            motors.strafeLeft();
          }
          break;
        }

        case SubPhase::DONE_SUB: {
          sub = SubPhase::REVERSE_TO_WALL; // reset for next run
          phase++;
          break;
        }
      }
      break; // case 2
    }

    // ── Phase 3: done ──────────────────────────────────────────
    case 3: {
      motors.stop();
      sensors.ZeroGyroHeading();
      // Serial.println("Homing complete.");
      // return STEP::IDLE;
      return STEP::TILLING;
    }
  }
  return STEP::HOMING;
}


// ════════════════════════════════════════════════════════════════
//  SWEEP — call repeatedly from homing(), returns true when done
// ════════════════════════════════════════════════════════════════

static bool doSweep() {
  static unsigned long lastSampleMs = 0;
  static bool          sweepStarted = false;
  static float         startAngle   = 0.0f;
  static bool          fullRotation = false;
  static bool          hasLeftStart = false; // ADDED: Step 1
  static unsigned long sweepStartTime = 0;

  if (!sweepStarted) {
    sweep_count  = 0;
    fullRotation = false;
    hasLeftStart = false; // ADDED: Step 2
    startAngle   = sensors.getGyroHeading();
    sensors.setUSStrictFilter(false);
    sweepStarted = true;
    sweepStartTime = millis();
    // Serial.print("DEBUG: Sweep Started. Start Angle: "); Serial.println(startAngle);
    motors.rotateClockwise();
  }
  if (sweepStarted && ((millis() - sweepStartTime) >= 200UL)) {
    motors.rotateClockwise();
    if (millis() - lastSampleMs < SWEEP_SAMPLE_INTERVAL) return false;
    lastSampleMs = millis();

    float currentAngle = sensors.getGyroHeading();
    float dist         = sensors.readUltrasonicCm();

    if (dist > 0.0f && sweep_count < SWEEP_BUF_SIZE) {
      sweep_dist[sweep_count]  = dist;
      sweep_angle[sweep_count] = currentAngle;
      sweep_count++;
      // Serial.print("Sweep | angle: "); Serial.print(currentAngle, 1);
      // Serial.print("° | dist: ");      Serial.print(dist, 1);
      // Serial.println(" cm");
    }

    if (sweep_count > 25) {
      float travelled = angleDiff(startAngle, currentAngle);
  
      // This forces the robot to move at least 30 degrees away from the 0 point
      // before it is allowed to "finish" the sweep.
      if (!hasLeftStart && travelled > 30.0f && travelled < 100.0f) {
          hasLeftStart = true;
          // Serial.println("DEBUG: Logic gate passed: Robot has left the start zone.");
      }

      if (hasLeftStart && travelled >= 355.0f) {
        fullRotation = true;
      }
    }

    if (!fullRotation) return false;
    
    motors.stop();
    sweepStarted = false;
    // Serial.print("DEBUG: Sweep Stop. Total Samples: "); Serial.println(sweep_count);
    return true;
  }

  return false;
}



// ════════════════════════════════════════════════════════════════
//  FIND WALLS FROM BUFFER
//  1. find global minimum
//  2. cluster readings near that minimum → wall 0
//  3. extrapolate walls 1–3 at +90° increments
//  4. for each extrapolated angle, average nearby buffer readings
// ════════════════════════════════════════════════════════════════
static bool findWallsFromBuffer() {
  if (sweep_count < 10) return false;
  // ── Step 1: find global minimum ────────────────────────────
  float    minDist  = 9999.0f;
  uint16_t minIndex = 0;

  for (uint16_t i = 0; i < sweep_count; i++) {
    if (sweep_dist[i] < minDist) {
      minDist  = sweep_dist[i];
      minIndex = i;
    }
  }

  // ── Step 2: take contiguous low-distance cluster around global min ─
  auto angleGapDeg = [](float a, float b) {
    float d = fabsf(a - b);
    if (d > 180.0f) d = 360.0f - d;
    return d;
  };

  const float degToRad = 0.01745329252f;
  uint16_t count = 0;
  float distSum = 0.0f;
  float sinSum = 0.0f;
  float cosSum = 0.0f;

  auto addSample = [&](uint16_t idx) {
    const float ang = sweep_angle[idx];
    const float angRad = ang * degToRad;
    count++;
    distSum += sweep_dist[idx];
    sinSum += sinf(angRad);
    cosSum += cosf(angRad);
  };

  addSample(minIndex);

  // Expand forward while samples stay both near min distance and angularly contiguous.
  uint16_t prev = minIndex;
  uint16_t i = (minIndex + 1) % sweep_count;
  while (i != minIndex) {
    if (sweep_dist[i] > minDist + MIN_CLUSTER_DIST) break;
    if (angleGapDeg(sweep_angle[prev], sweep_angle[i]) > ANGLE_CLUSTER_GAP_DEG) break;
    addSample(i);
    prev = i;
    i = (i + 1) % sweep_count;
  }

  // Expand backward with the same continuity constraints.
  prev = minIndex;
  i = (minIndex == 0) ? (sweep_count - 1) : (minIndex - 1);
  while (i != minIndex) {
    if (sweep_dist[i] > minDist + MIN_CLUSTER_DIST) break;
    if (angleGapDeg(sweep_angle[i], sweep_angle[prev]) > ANGLE_CLUSTER_GAP_DEG) break;
    addSample(i);
    prev = i;
    i = (i == 0) ? (sweep_count - 1) : (i - 1);
  }

  if (count == 0) return false;

  float averagedAngle = atan2f(sinSum, cosSum) * 57.2957795f;
  if (averagedAngle < 0.0f) averagedAngle += 360.0f;

  walls[0].angle = averagedAngle;
  walls[0].dist  = distSum / count;
  // Serial.print("Wall 0 cluster: "); Serial.print(count);
  // Serial.print(" samples | angle: "); Serial.print(walls[0].angle, 1);
  // Serial.print("° | dist: "); Serial.print(walls[0].dist, 1);
  // Serial.println(" cm");

  // ── Step 3 & 4: extrapolate walls 1–3 at 90° increments ────
  for (uint8_t w = 1; w < 4; w++) {
    float targetAngle = walls[0].angle + (w * 90.0f);
    while (targetAngle >= 360.0f) targetAngle -= 360.0f;

    // find readings within ±20° of target angle
    float aSum  = 0.0f;
    float dSum  = 0.0f;
    uint16_t n  = 0;
    float window = 20.0f; // ADJUST THIS.

    for (uint16_t i = 0; i < sweep_count; i++) {
      float diff = fabs(sweep_angle[i] - targetAngle);
      if (diff > 180.0f) diff = 360.0f - diff;
      if (diff <= window) {
        aSum += sweep_angle[i];
        dSum += sweep_dist[i];
        n++;
      }
    }

    if (n == 0) {
      // no readings near this angle — use geometric estimate
      // Serial.print("Wall "); Serial.print(w);
      // Serial.println(" no readings, using extrapolated angle only");
      walls[w].angle = targetAngle;
      walls[w].dist  = -1.0f;   // unknown
    } else {
      walls[w].angle = aSum / n;
      walls[w].dist  = dSum / n;
    }
  }

  walls_found = 4;
  return true;
}

// ════════════════════════════════════════════════════════════════
//  HELPERS
// ════════════════════════════════════════════════════════════════
// returns absolute angular travel from start to current (0–360)

static float angleDiff(float start, float current) {
  float diff = current - start;
  while (diff < 0.0f)    diff += 360.0f;
  while (diff >= 360.0f) diff -= 360.0f;
  return diff;
}
