#include <Arduino.h>
#include "V7RCServoDriver.h"

V7RCServoDriver v7rc;

// ───── Servo 設定（目前沒有伺服，可之後擴充） ─────
V7RC_ServoConfig myServos[] = {
  // { pin, minUs, maxUs, minDeg, maxDeg, initDeg }
  { 7, 1000, 2000, 0.0f, 90.0f, 45.0f },
  { 8, 1000, 2000, 0.0f, 90.0f, 45.0f }
};
const uint8_t NUM_SERVOS = sizeof(myServos) / sizeof(myServos[0]);

// 依照你實際接線修改 pinDir / pinPwm
V7RC_DCMotorConfig myMotors[] = {
  { 21, 20, false }, // 0: Front-Left
  { 10, 0, false }, // 1: Front-Rightx
};
const uint8_t NUM_MOTORS = sizeof(myMotors) / sizeof(myMotors[0]);

// ───── Smooth 參數（如果將來有 Servo） ─────
V7RC_SmoothConfig mySmooth = {
  .maxStepDeg  = 45.0f,
  .deadbandDeg = 2.0f,
  .updateMs    = 10
};

// ───── Drive 設定：麥克納姆 ─────
V7RC_DriveConfig myDriveCfg = {
  .type          = DRIVE_DIFF,
  .diffLeftMotor = 0,
  .diffRightMotor= 1,
  .mecFrontLeft  = -1,  // dcMotors[0]
  .mecFrontRight = -1,  // dcMotors[1]
  .mecRearLeft   = -1,  // dcMotors[2]
  .mecRearRight  = -1   // dcMotors[3]
};

// ───── Channel Map：適用麥克納姆車 ─────
// Channel 說明：
// ch0 → CH_DRIVE_MEC_VX    （前後速度）
// ch1 → CH_DRIVE_MEC_VY    （左右平移）
// ch2 → CH_DRIVE_MEC_OMEGA （旋轉）
// 其他 Channel 目前不使用，可之後擴充成 Servo / DC_MOTOR 等
// ───── Channel Map：適用麥克納姆車 ─────
// 建議先依照「你在 V7RC APP 的搖桿定義」對應：
//   CH_DRIVE_MEC_VX    ：前後 (Forward/Backward)   範圍 -1..+1
//   CH_DRIVE_MEC_VY    ：左右 (Strafe Left/Right)  範圍 -1..+1
//   CH_DRIVE_MEC_OMEGA ：旋轉 (Yaw)                範圍 -1..+1
//
// 若你遇到「Y 變 X / X 變 Y」：請交換 ch0/ch1 的角色即可。
// CH_DRIVE_THROTTLE / CH_DRIVE_STEER 是差速專用，麥克納姆車請用 CH_DRIVE_MEC_VX/VY/OMEGA。
// 此範例預設：
//   ch0 → VX, ch1 → VY, ch2 → OMEGA
// 其餘 Channel 先不使用（CH_NONE），之後可擴充成 Servo / 直接 DC motor 控制。
V7RC_ChannelConfig myChannelMap[] = {
  { CH_DRIVE_THROTTLE,    -1 }, // ch0
  { CH_DRIVE_STEER,    -1 }, // ch1
  { CH_SERVO, 0 }, // ch2
  { CH_SERVO,            1 }, // ch3
  { CH_NONE,            -1 }, // ch4
  { CH_NONE,            -1 }, // ch5
  { CH_NONE,            -1 }, // ch6
  { CH_NONE,            -1 }, // ch7
  { CH_NONE,            -1 }, // ch8
  { CH_NONE,            -1 }, // ch9
  { CH_NONE,            -1 }, // ch10
  { CH_NONE,            -1 }, // ch11
  { CH_NONE,            -1 }, // ch12
  { CH_NONE,            -1 }, // ch13
  { CH_NONE,            -1 }, // ch14
  { CH_NONE,            -1 }  // ch15
};


const uint8_t NUM_CHANNEL_MAP = sizeof(myChannelMap) / sizeof(myChannelMap[0]);

// ───── Driver 總設定 ─────
V7RC_DriverConfig driverCfg = {
  .bleBaseName        = "V7RC-ZERO",
  .servos             = myServos,
  .numServos          = NUM_SERVOS,
  .smooth             = mySmooth,
  .waveDemoServoIndex = -1,
  .dcMotors           = myMotors,
  .numDCMotors        = NUM_MOTORS,
  .drive              = myDriveCfg,
  .channelMap         = myChannelMap,
  .numChannelMap      = NUM_CHANNEL_MAP,

  // WS2812 strip (default data pin 8, 8 LEDs)
  .ws2812Brightness   = 50, // dimmed by default; change 0-255
  .ws2812Enable       = true,
  .ws2812Pin          = 8,
  .ws2812Count        = 8
};


const uint8_t ROBOT_ID = 0;  // 多台機器可用 1..99

void setup() {
  Serial.begin(115200);
  delay(1000);

  v7rc.begin(ROBOT_ID, driverCfg);

  // you can change strip colour at any time:
  // v7rc.setAllLeds(255, 0, 0); // red
  //
  // To control individual LEDs from the V7RC app use the LE* protocol:
  // e.g. send "LED0F0F0A...#" to set first four pixels.
}

void loop() {
  v7rc.loop();
}
