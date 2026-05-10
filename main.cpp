/*
  MechEng 706 Base Code
  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project
  Hardware:
    Arduino Mega2560 [https://www.arduino.cc/en/Guide/ArduinoMega2560](https://www.arduino.cc/en/Guide/ArduinoMega2560)
    BNO085 [https://www.adafruit.com/product/4754](https://www.adafruit.com/product/4754)
    Ultrasonic Sensor - HC-SR04 [https://www.sparkfun.com/products/13959](https://www.sparkfun.com/products/13959)
    Infrared Proximity Sensor - Sharp [https://www.sparkfun.com/products/242](https://www.sparkfun.com/products/242)
    Infrared Proximity Sensor Short Range - Sharp [https://www.sparkfun.com/products/12728](https://www.sparkfun.com/products/12728)
    Servo - Generic (Sub-Micro Size) [https://www.sparkfun.com/products/9065](https://www.sparkfun.com/products/9065)
    Vex Motor Controller 29 [https://www.vexrobotics.com/276-2193.html](https://www.vexrobotics.com/276-2193.html)
    Vex Motors [https://www.vexrobotics.com/motors.html](https://www.vexrobotics.com/motors.html)
    Turnigy nano-tech 2200mah 2S [https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html](https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html)
    HC 12 Module [https://www.hc01.com/downloads/HC-12%20english%20datasheets.pdf](https://www.hc01.com/downloads/HC-12%20english%20datasheets.pdf)
  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 18/02/2026
  Author: Trishit Ghatak
*/
/*
        DONT CHANGE HEADERS
   
-----------------------------
*/

#include <Servo.h>
#include "Motor.h"
#include "Sensors.h"
#include "sweep.h"


//#define NO_READ_GYRO  //Uncomment if GYRO is not attached.
//#define NO_HC_SR04  //Uncomment if HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment if BATTERY_V_OK if you do not care about battery damage.

char read_serial_command();
void fast_flash_double_LED_builtin();
void slow_flash_LED_builtin();
Servo turret_motor;
Motor motors;
Sensors sensors;

//Serial Pointer
// SerialCom points to Serial1 (HC-12 on hardware UART1: TX1=pin18, RX1=pin19) at 9600 baud (HC-12 factory default).
// USB Serial (Serial, 115200 baud) runs in parallel for PC-side monitoring — no code changes needed elsewhere.
HardwareSerial *SerialCom;

unsigned long main_previous_millis = 0;

int pos = 0;
float gyro_vel;
float gyro_angle = 0.0f;
static STEP machine_step = STEP::INITIALISING;
bool started = false;


void setup(void) {
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);
  main_previous_millis = 0;

  // Start USB Serial for PC-side monitoring (does not affect SerialCom logic)
  Serial.begin(115200);
  Serial.println(F("USB Serial active - monitoring only"));

  // Setup HC-12 on Hardware Serial1 (TX1=pin18, RX1=pin19) at HC-12 factory default baud rate
  Serial2.begin(115200);

  // SerialCom pointer now targets Serial1 (HC-12 wireless); all downstream calls unchanged
  SerialCom = &Serial2;
  SerialCom->println(F("MECHENG706_Base_Code"));
  delay(1000);
  SerialCom->println(F("Setup...."));
  delay(1000);  //settling time but no really needed
}
void loop(void)  //main loop
{
    //Finite-state machine Code
  switch (machine_step) {
    case DEBUG:
      // Serial.println("LR1");
      // Serial.println(sensors.readLongRangeIR1());
      // Serial.println("LR2");
      // Serial.println(sensors.readLongRangeIR2());
      // Serial.println("SR1");
      // Serial.println(sensors.readShortRangeIR1());
      // Serial.println("SR2");
      // Serial.println(sensors.readShortRangeIR2());
      // delay(500);
      machine_step = DEBUG;
    break;
    case INITIALISING:
      machine_step = initialising();
    break;
    case IDLE:
      machine_step = idle();
      break;

    case FIRE:
      machine_step = fire();
      break;

    case STOPPED:
      machine_step = stopped();
      break;
    case ERROR:
      machine_step = error();
      break;
  };
  if((machine_step == STEP::TILLING)){
    if (millis() - main_previous_millis > 200) {  //Arduino style 500ms timed execution statement
      main_previous_millis = millis();
      SerialCom->print(sensors.readUltrasonicCm());
      SerialCom->print(", ");
      SerialCom->print(millis());
      SerialCom->print(", ");
      SerialCom->print(sensors.readLongRangeIR1());
      SerialCom->print(", ");
      SerialCom->print(sensors.readLongRangeIR2());
      SerialCom->println(", ");
    }
  }
}

STEP initialising() {
  static unsigned long next_retry_ms = 50;

  if (!started) {
    SerialCom->println(F("INITIALISING...."));
    SerialCom->println(F("Enabling Motors..."));
    motors.initialise(SerialCom);
    sensors.setSerial(SerialCom);
    started = true;
    Serial.println(F("A"));
  }

  if (millis() < next_retry_ms) {
    Serial.println(F("B"));
    return STEP::INITIALISING;

  }

  SerialCom->println(F("Enabling Sensors..."));
  if (!sensors.initialise()) {
    SerialCom->println(F("Sensor init failed - retrying..."));
    next_retry_ms = millis() + 1000;
    Serial.println(F("C"));
    return STEP::INITIALISING;
  }

  SerialCom->println(F("RUNNING STATE..."));
  Serial.println(F("D"));
  return STEP::IDLE; //CHANGE THIS BACK TO IDLE() IN FUTURE
}

STEP idle() {
  static unsigned long previous_millis;
  char key;
  key = read_serial_command();
  if(key == 'h'){
    return STEP::HOMING;
  } else if (key == 't'){
    return STEP::TILLING;
  }
  motors.driveStraight(sensors.getGyroHeading(), sensors.readUltrasonicCm(), sensors.readLongRangeIR1());
  
  fast_flash_double_LED_builtin();
  //motors.isTurnComplete(sensors.getGyroHeading());

#ifndef NO_BATTERY_V_OK
  if (!sensors.isBatteryVoltageOK()) return STEP::ERROR;
#endif

  if (millis() - previous_millis > 200) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();
    // SerialCom->println(F("RUNNING---------"));
    // SerialCom->print("Current Angle = ");
    // SerialCom->println(sensors.getGyroHeading());
    // SerialCom->print(sensors.readUltrasonicCm());
    // SerialCom->print(", ");

    // motors.PrintDetails();

    turret_motor.write(pos);
    if (pos == 0) {
      pos = 45;
    } else {
      pos = 0;
    }
  }
  return STEP::IDLE;
}


//Stop of Lipo Battery voltage is too low, to protect Battery
STEP stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  motors.disable();
  slow_flash_LED_builtin();
  if (millis() - previous_millis > 500) {  //print massage every 500ms
    previous_millis = millis();
    SerialCom->println(F("STOPPED---------"));
#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
  if (sensors.isBatteryVoltageOK()) {
      SerialCom->print(F("Lipo OK waiting of voltage Counter 10 < "));
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        motors.enable();
        SerialCom->println(F("Lipo OK returning to RUN STATE"));
        return STEP::IDLE;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STEP::STOPPED;
}

STEP error() {
  // Route ERROR handling through the existing safe stopped behavior.
  return stopped();
}

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}
void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

//Serial command pasing
char read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print(F("Speed:"));
    SerialCom->print(motors.getSpeed());
    SerialCom->print(F(" ms "));
    //Perform an action depending on the command
    switch (val) {
      case 'w':  //Move Forward
      case 'W':
        //motors.driveForward();
        SerialCom->println(F("Forward"));
        motors.SetDriveStraightTarget(DIRECTION::FORWARD, sensors.getGyroHeading(), 15 ,0);
        return 'w';
      case 's':  //Move Backwards
      case 'S':
        motors.ResetDriveStraightTarget();
        return 's';
      case 'q':  //Turn Left
      case 'Q':
        motors.SetDriveStraightTarget(DIRECTION::FORWARD, sensors.getGyroHeading() + 90, 0, 15);
        return 'q';
      case 'e':  //Turn Right
      case 'E':
        motors.SetDriveStraightTarget(DIRECTION::FORWARD, sensors.getGyroHeading() - 90, 0, 15);
        return 'e';
      case 'a':  //Turn Left
      case 'A':
        motors.SetTurnTarget(sensors.getGyroHeading() + 45);
        return 'a';
      case 'd':  //Turn Right
      case 'D':
        motors.SetTurnTarget(sensors.getGyroHeading() - 45);
        return 'd';
      case 'h':  //Home
      case 'H':
        return 'h';
      case 't':  //Tillings
      case 'T':
        return 't';
      case '-':  //Turn Right
      case '_':
        motors.SetTurnTarget(-90);
        return '-';
      case '=':
      case '+':
        motors.SetTurnTarget(90);
        return '=';
      case ' ':
        motors.stop();
        return ' ';
    }
  }

  return '\0';
}