/*
  V7RCServoDriver - Differential Drive Example (ESP32-C3)
  -------------------------------------------------------
  This example shows a SIMPLE 2WD differential drive robot:
    - ch0: Throttle  (-1..+1)
    - ch1: Steering  (-1..+1)

  Hardware (recommended):
    - ESP32-C3 + TB6612FNG (or DRV8833)
    - 2 DC motors (Left / Right)

  Notes:
    - Make sure all grounds are connected (common GND).
    - If motor direction is reversed, set dirInvert=true or swap IN1/IN2.

  Repo: https://github.com/v7rc/V7RCServoDriver
*/

#include "V7RCServoDriver.h"

// -------------------------
// Pin assignment (TB6612FNG)
// -------------------------
// Left motor (A channel)
static const uint8_t PIN_L_PWM = 4;   // PWMA
static const uint8_t PIN_L_IN1 = 5;   // AIN1
static const uint8_t PIN_L_IN2 = 6;   // AIN2

// Right motor (B channel)
static const uint8_t PIN_R_PWM = 7;   // PWMB
static const uint8_t PIN_R_IN1 = 8;   // BIN1
static const uint8_t PIN_R_IN2 = 9;   // BIN2

// Enable / Standby
static const uint8_t PIN_STBY = 10;  // STBY (or tie to 3.3V)

// -------------------------
// V7RC config
// -------------------------
V7RCServoDriver v7rc;

// 2 motors for diff drive
V7RC_DCMotorConfig motors[] = {
  // Left motor
  {
    /*pinPwm*/ PIN_L_PWM,
    /*pinDir1*/ PIN_L_IN1,
    /*pinDir2*/ PIN_L_IN2,
    /*pwmFreq*/ 20000,
    /*pwmResolution*/ 8,
    /*dirInvert*/ false,
    /*pwmInvert*/ false
  },
  // Right motor
  {
    /*pinPwm*/ PIN_R_PWM,
    /*pinDir1*/ PIN_R_IN1,
    /*pinDir2*/ PIN_R_IN2,
    /*pwmFreq*/ 20000,
    /*pwmResolution*/ 8,
    /*dirInvert*/ false,
    /*pwmInvert*/ false
  }
};

// Drive config (differential)
V7RC_DriveConfig driveCfg = {
  /*driveType*/ DRIVE_DIFF,
  /*diffLeftMotorIndex*/ 0,
  /*diffRightMotorIndex*/ 1,
  // optional scaling (if your struct has these fields, keep; otherwise remove)
  /*maxThrottle*/ 1.0f,
  /*maxSteer*/ 1.0f
};

// Channel mapping:
// ch0 -> Throttle, ch1 -> Steer
V7RC_ChannelConfig channelMap[] = {
  { CH_DRIVE_THROTTLE, -1 }, // ch0
  { CH_DRIVE_STEER,    -1 }, // ch1
  { CH_NONE,           -1 }, // ch2
  { CH_NONE,           -1 }, // ch3
  { CH_NONE,           -1 }, // ch4
  { CH_NONE,           -1 }, // ch5
  { CH_NONE,           -1 }, // ch6
  { CH_NONE,           -1 }, // ch7
  { CH_NONE,           -1 }, // ch8
  { CH_NONE,           -1 }, // ch9
  { CH_NONE,           -1 }, // ch10
  { CH_NONE,           -1 }, // ch11
  { CH_NONE,           -1 }, // ch12
  { CH_NONE,           -1 }, // ch13
  { CH_NONE,           -1 }, // ch14
  { CH_NONE,           -1 }  // ch15
};

// Driver config (bundle)
V7RC_DriverConfig driverConfig = {
  /*bleBaseName*/ "V7RC-ROBOT",
  /*robotId*/ 1,
  /*channelMap*/ channelMap,
  /*channelCount*/ (uint8_t)(sizeof(channelMap)/sizeof(channelMap[0])),
  /*servos*/ nullptr,
  /*servoCount*/ 0,
  /*dcMotors*/ motors,
  /*dcMotorCount*/ (uint8_t)(sizeof(motors)/sizeof(motors[0])),
  /*drive*/ &driveCfg,
  /*failsafeMs*/ 500
};

void setup() {
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);

  Serial.begin(115200);
  delay(200);

  // Start BLE + driver
  v7rc.begin(driverConfig.robotId, driverConfig);
  Serial.println("V7RC Differential Drive ready. Connect with V7RC App.");
}

void loop() {
  v7rc.loop();
}
