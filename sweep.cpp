#include "sweep.h"
#include "main.h"
#include "Motor.h"
#include "Sensors.h"

extern Motor motors;
extern Sensors sensors;

#define SWEEP_BUF_SIZE      256     // max samples for a full 360
#define PT_OFFSET           5


static float PTL[SWEEP_BUF_SIZE];
static float PTC[SWEEP_BUF_SIZE];
static float PTR[SWEEP_BUF_SIZE];

static struct PT_result {
    float PTL;
    float PTC;
    float PTR;
}

static struct result {
    float PT;
    float angle;
}

STEP sweep() {
    sensors.ZeroGyroHeading();
    static int sweep_index = 0;
    if (sweep_index < SWEEP_BUF_SIZE) {
        result
        PTL[sweep_index] = result.PT = sensors.readPTL(), result.angle = gyro_angle = sensors.getGyroHeading();
        PTC[sweep_index] = sensors.readPTC();
        PTR[sweep_index] = sensors.readPTR();
        sweep_index++;
    } else {
        motors.stop();
    }
}
