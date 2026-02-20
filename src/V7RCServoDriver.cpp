#include "V7RCServoDriver.h"

#include <ESP32Servo.h>
#include <math.h>

// WS2812 (NeoPixel) support
#include <Adafruit_NeoPixel.h>

// ===== h2zero NimBLE-Arduino =====
#include <NimBLEDevice.h>


// ===== Protocol / Frame 定義 =====
enum V7RC_ProtocolType : uint8_t {
  V7RC_NONE = 0,
  V7RC_HEX,
  V7RC_DEG,
  V7RC_SRV,   // SRV / SRT 同用
  V7RC_SS8,
  V7RC_LED    // text-based LED control protocol (any group)
};

struct V7RC_Frame {
  V7RC_ProtocolType type;
  int16_t values[16];  // 16 個 Channel 的數值
  bool valid;
};

static const int V7RC_NUM_CHANNELS   = 16;
static const int V7RC_FRAME_SIZE     = 20;  // 3 bytes header + 16 payload + '#' = 20

// ===== 全域 Driver 狀態 =====
static V7RC_ServoConfig*   g_servos      = nullptr;
static uint8_t             g_numServos   = 0;
static V7RC_DCMotorConfig* g_dcMotors    = nullptr;
static uint8_t             g_numDCMotors = 0;

static V7RC_SmoothConfig   g_smooth      = {3.0f, 0.8f, 10};
static int                 g_waveDemoServoIndex = -1;

static V7RC_ChannelConfig* g_channelMap   = nullptr;
static uint8_t             g_numChannelMap= 0;

static V7RC_DriveConfig    g_driveCfg;
static bool                g_driveEnabled = false;
static float               g_diffThrottle = 0.0f;
static float               g_diffSteer    = 0.0f;
static float               g_mecVx        = 0.0f;
static float               g_mecVy        = 0.0f;
static float               g_mecOmega     = 0.0f;
static int                 g_chX          = 1500;
static int                 g_chY          = 1500;
static int                 g_chR          = 1500;

// WS2812 globals
static Adafruit_NeoPixel*  g_ws2812       = nullptr;
static uint8_t             g_ws2812Count  = 0;

// per-LED custom state (set by new LE* protocol)
struct LedState {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t blinkLevel; // 0..10 (100ms units)
};
static LedState*          g_ledStates       = nullptr;
static bool               g_ledCustomActive = false;

// LED status for connection animation (used when no custom active)
static bool               g_ledBlinkState    = false;
static unsigned long      g_ledLastToggleMs  = 0;

// Servo 物件與狀態
static Servo* g_servoObjs  = nullptr;
static float* g_currentDeg = nullptr;
static float* g_targetDeg  = nullptr;

// ===== BLE (NimBLE) =====
static char                  g_bleDeviceName[24] = "V7RC-ROBOT-01";
static NimBLEServer*         pServer             = nullptr;
static NimBLECharacteristic* pRxCharacteristic   = nullptr;
static NimBLECharacteristic* pTxCharacteristic   = nullptr;
static bool                  deviceConnected     = false;

// Binary Frame 接收 buffer
static char rxBuf[V7RC_FRAME_SIZE];
static int  rxIndex = 0;

// 心跳 / 逾時
static unsigned long g_lastFrameMs   = 0;
static const unsigned long COMMAND_TIMEOUT_MS = 1000;

// ===== 工具函式 =====
static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (fabsf(in_max - in_min) < 1e-6f) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return 0;
}

// 角度 ↔ us
static uint16_t degToUs(const V7RC_ServoConfig &cfg, float deg) {
  float d  = clampFloat(deg, cfg.minDeg, cfg.maxDeg);
  float us = mapFloat(d, cfg.minDeg, cfg.maxDeg, cfg.minUs, cfg.maxUs);
  if (us < cfg.minUs) us = cfg.minUs;
  if (us > cfg.maxUs) us = cfg.maxUs;
  return (uint16_t)us;
}

static float usToDeg(const V7RC_ServoConfig &cfg, float us) {
  float u = clampFloat(us, cfg.minUs, cfg.maxUs);
  float d = mapFloat(u, cfg.minUs, cfg.maxUs, cfg.minDeg, cfg.maxDeg);
  return clampFloat(d, cfg.minDeg, cfg.maxDeg);
}

// DC 馬達：PWM(us) → DIR/PWM 輸出
static void processDCMotorPwm(uint16_t pwmUs, const V7RC_DCMotorConfig& m) {
  if (pwmUs < 1000) pwmUs = 1000;
  if (pwmUs > 2000) pwmUs = 2000;

  if (pwmUs == 1500) {
    digitalWrite(m.pinDir, LOW);
    analogWrite(m.pinPwm, 0);
  } else if (pwmUs > 1500) {
    int power = map(pwmUs, 1500, 2000, 0 , 255);
    digitalWrite(m.pinDir, LOW);
    analogWrite(m.pinPwm, power);
  } else {
    int power = map(pwmUs, 1500, 1000, 255 , 0);
    digitalWrite(m.pinDir, HIGH);
    analogWrite(m.pinPwm, power);
  }
}

// Drive speed (-1..1) → PWM us
static uint16_t speedToPwmUs(float s) {
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  float us = 1500.0f + s * 500.0f;
  if (us < 1000.0f) us = 1000.0f;
  if (us > 2000.0f) us = 2000.0f;
  return (uint16_t)us;
}

// ===== BLE TX =====
static void sendTxLine(const String &line) {
  if (!deviceConnected || !pTxCharacteristic) return;
  pTxCharacteristic->setValue((uint8_t*)line.c_str(), (size_t)line.length());
  pTxCharacteristic->notify();
}

// ===== Frame 解碼 =====
static V7RC_Frame decodeV7RC(const char *buf) {
  V7RC_Frame frame;
  frame.valid = false;
  frame.type  = V7RC_NONE;

  if (!buf) return frame;
  if (buf[V7RC_FRAME_SIZE - 1] != '#') return frame;

  // 預設：每個 Channel 先放中立 PWM（若是 Servo 則取 initDeg 對應的 us）
  for (int ch = 0; ch < V7RC_NUM_CHANNELS; ch++) {
    uint16_t us = 1500;

    if (g_channelMap && ch < g_numChannelMap) {
      V7RC_ChannelConfig &cc = g_channelMap[ch];
      if (cc.role == CH_SERVO &&
          cc.targetIndex >= 0 &&
          cc.targetIndex < (int)g_numServos &&
          g_servos) {
        const V7RC_ServoConfig &cfg = g_servos[cc.targetIndex];
        us = degToUs(cfg, cfg.initDeg);
      }
    }

    frame.values[ch] = (int16_t)us;
  }

  // 判斷 header
  if (buf[0] == 'H' && buf[1] == 'E' && buf[2] == 'X') {
    frame.type = V7RC_HEX;
  } else if (buf[0] == 'D' && buf[1] == 'E' && buf[2] == 'G') {
    frame.type = V7RC_DEG;
  } else if (buf[0] == 'S' && buf[1] == 'R' && buf[2] == 'V') {
    frame.type = V7RC_SRV;
  } else if (buf[0] == 'S' && buf[1] == 'S' && buf[2] == '8') {
    frame.type = V7RC_SS8;
  } else if (buf[0] == 'S' && buf[1] == 'R' && buf[2] == 'T') {
    frame.type = V7RC_SRV; // SRT 當作 SRV 處理
  } else if (buf[0] == 'L' && buf[1] == 'E') {
    // any LED/LE? command maps to LED protocol
    frame.type = V7RC_LED;
  } else {
    return frame;
  }

  // 解析 payload
  if (frame.type == V7RC_HEX) {
    // 16 bytes，每個 *10 → us
    for (int i = 0; i < V7RC_NUM_CHANNELS; i++) {
      int8_t raw = buf[3 + i];
      uint16_t us = (uint8_t)raw * 10;
      frame.values[i] = (int16_t)us;
    }
  } else if (frame.type == V7RC_DEG) {
    // 16 bytes，每個 -127 → deg
    for (int i = 0; i < V7RC_NUM_CHANNELS; i++) {
      int8_t raw = buf[3 + i];
      int16_t deg = (int16_t)raw - 127;
      frame.values[i] = deg;
    }
  } else if (frame.type == V7RC_SRV) {
    // "SRV"/"SRT" + 4 組 "dddd" → ch0~3
    for (int i = 0; i < 4; i++) {
      char tmp[5];
      int offset = 3 + i * 4;
      tmp[0] = buf[offset + 0];
      tmp[1] = buf[offset + 1];
      tmp[2] = buf[offset + 2];
      tmp[3] = buf[offset + 3];
      tmp[4] = '\0';
      int pwm = atoi(tmp);
      if (pwm < 0)    pwm = 0;
      if (pwm > 3000) pwm = 3000;
      frame.values[i] = (int16_t)pwm;
    }
  } else if (frame.type == V7RC_SS8) {
    // "SS8" + 8 組兩位 HEX → ch0~7，各 *10 → us
    for (int i = 0; i < 8; i++) {
      int offset = 3 + i * 2;
      char h0 = buf[offset + 0];
      char h1 = buf[offset + 1];
      int value = (hexNibble(h0) << 4) | hexNibble(h1);
      int pwm   = value * 10;   // 0..2550
      frame.values[i] = (int16_t)pwm;
    }
  }

  frame.valid = true;
  return frame;
}

inline float pwmToNorm(int pwm) {
  return constrain((pwm - 1500) / 500.0f, -1.0f, 1.0f);
}

static void driveDCMotorNorm(float v, const V7RC_DCMotorConfig& m) {
  v = constrain(v, -1.0f, 1.0f);
  // 可依你的遙控抖動調大到 0.03~0.06
  const float dead = 0.03f;

  if (fabs(v) < dead) {
    analogWrite(m.pinPwm, 0);
    // 建議：停車時 DIR 固定到一個狀態，避免某些驅動板在另一狀態是 brake
    digitalWrite(m.pinDir, m.dirInvert ? HIGH : LOW);
    return;
  }

  int power = (int)(fabs(v) * 255.0f);

  if(v < 0) {
    
     power = (int)(fabs(1.0f + v) * 255.0f); 

  }

  power = constrain(power, 0, 255);

    bool dirLevel = (v > 0) ? LOW : HIGH;
    if (m.dirInvert) dirLevel = !dirLevel;

    digitalWrite(m.pinDir, dirLevel);
    analogWrite(m.pinPwm, power);
}

void driveDCMotorNorm_IN1IN2(float v, const V7RC_DCMotorConfig& m) {
  v = constrain(v, -1.0f, 1.0f);
  const float dead = 0.03f;

  int power = (int)(fabs(v) * 255.0f);
  power = constrain(power, 0, 255);

  if (fabs(v) < dead) {
    // Coast：兩腳都 LOW（不煞車）
    digitalWrite(m.pinDir, LOW);
    digitalWrite(m.pinPwm, LOW);
    return;
  }

  // 可選：最低起轉力，避免低速卡住
  const int minPwm = 35;
  if (power > 0 && power < minPwm) power = minPwm;

  bool forward = (v > 0);
  if (m.dirInvert) forward = !forward;

  if (forward) {
    // IN1 = PWM, IN2 = LOW
    analogWrite(m.pinDir, power);
    digitalWrite(m.pinPwm, LOW);
  } else {
    // IN2 = PWM, IN1 = LOW
    analogWrite(m.pinPwm, power);
    digitalWrite(m.pinDir, LOW);
  }
}

static void driveMecanum(
  int chX, int chY, int chR,
  const V7RC_DCMotorConfig& FL,
  const V7RC_DCMotorConfig& FR,
  const V7RC_DCMotorConfig& RL,
  const V7RC_DCMotorConfig& RR
) {
  float vx = pwmToNorm(chX);
  float vy = pwmToNorm(chY);
  float w  = pwmToNorm(chR);

  float fl = vy - vx + w;
  float fr = vy + vx - w;
  float rl = vy + vx + w;
  float rr = vy - vx - w;

  float maxVal = max(max(abs(fl), abs(fr)), max(abs(rl), abs(rr)));

  if (maxVal > 1.0f) {
    fl /= maxVal;
    fr /= maxVal;
    rl /= maxVal;
    rr /= maxVal;
  }

  driveDCMotorNorm(fl, FL);
  driveDCMotorNorm(fr, FR);
  driveDCMotorNorm(rl, RL);
  driveDCMotorNorm(rr, RR);
}

// ===== 把 Frame 套用到 Servo / Motor / Drive =====
static void applyFrameToTargets(const V7RC_Frame &frame) {
  if (!frame.valid) return;

  bool anyDriveChannelUsed = false;

  int numCh = g_numChannelMap;
  if (numCh > V7RC_NUM_CHANNELS) numCh = V7RC_NUM_CHANNELS;

  for (int ch = 0; ch < numCh; ch++) {
    V7RC_ChannelConfig &cc = g_channelMap[ch];
    int16_t v = frame.values[ch];

    switch (cc.role) {
      case CH_NONE:
        break;

      case CH_SERVO: {
        int idx = cc.targetIndex;
        if (idx < 0 || idx >= (int)g_numServos || !g_servos) break;
        const V7RC_ServoConfig &cfg = g_servos[idx];

        if (frame.type == V7RC_HEX ||
            frame.type == V7RC_SRV ||
            frame.type == V7RC_SS8) {
          float deg = usToDeg(cfg, (float)v);
          g_targetDeg[idx] = deg;
        } else if (frame.type == V7RC_DEG) {
          float deg = (float)v;
          deg = clampFloat(deg, cfg.minDeg, cfg.maxDeg);
          g_targetDeg[idx] = deg;
        }
        break;
      }

      case CH_DC_MOTOR: {
        int midx = cc.targetIndex;
        if (midx < 0 || midx >= (int)g_numDCMotors || !g_dcMotors) break;

        if (frame.type == V7RC_HEX ||
            frame.type == V7RC_SRV ||
            frame.type == V7RC_SS8) {
          uint16_t pwmUs = (uint16_t)v;
          processDCMotorPwm(pwmUs, g_dcMotors[midx]);
        }
        break;
      }

      case CH_DRIVE_THROTTLE:
      case CH_DRIVE_STEER:
      case CH_DRIVE_MEC_VX:
      case CH_DRIVE_MEC_VY:
      case CH_DRIVE_MEC_OMEGA: {
        float s = 0.0f;

        if (frame.type == V7RC_HEX ||
            frame.type == V7RC_SRV ||
            frame.type == V7RC_SS8) {
          float us = (float)v;
          s = (us - 1500.0f) / 500.0f;   // 1000→-1, 1500→0, 2000→+1
        } else if (frame.type == V7RC_DEG) {
          s = (float)v / 127.0f;         // -127..127 → 約 -1..1
        }

        s = constrain(s, -1.0f, 1.0f);

        if (cc.role == CH_DRIVE_THROTTLE) {
          g_diffThrottle = s;
        } else if (cc.role == CH_DRIVE_STEER) {
          g_diffSteer = s;
        } else if (cc.role == CH_DRIVE_MEC_VX) {
          g_mecVx = s;
          g_chX = (int)v;
        } else if (cc.role == CH_DRIVE_MEC_VY) {
          g_mecVy = s;
          g_chY = (int)v;
        } else if (cc.role == CH_DRIVE_MEC_OMEGA) {
          g_mecOmega = s;
          g_chR = (int)v;
        }

        anyDriveChannelUsed = true;
        break;
      }
    }
  }

  if (anyDriveChannelUsed) {
    g_driveEnabled = true;
    if (g_driveCfg.type == DRIVE_NONE) {
      g_driveCfg.type = DRIVE_MECANUM;
    }
  }

  g_lastFrameMs = millis();
}

// ===== Drive 更新（差速 / 麥克納姆） =====
static void updateDrive() {
  if (!g_driveEnabled) return;
  if (!g_dcMotors || g_numDCMotors == 0) return;

  if (g_driveCfg.type == DRIVE_DIFF) {
    int li = g_driveCfg.diffLeftMotor;
    int ri = g_driveCfg.diffRightMotor;
    if (li < 0 || ri < 0 || li >= (int)g_numDCMotors || ri >= (int)g_numDCMotors) return;

    float left  = g_diffThrottle + g_diffSteer;
    float right = g_diffThrottle - g_diffSteer;

    float maxMag = max(fabsf(left), fabsf(right));
    if (maxMag > 1.0f) {
      left  /= maxMag;
      right /= maxMag;
    }

    uint16_t pwmL = speedToPwmUs(left);
    uint16_t pwmR = speedToPwmUs(right);

    processDCMotorPwm(pwmL, g_dcMotors[li]);
    processDCMotorPwm(pwmR, g_dcMotors[ri]);
  }
  else if (g_driveCfg.type == DRIVE_MECANUM) {
    int fl = g_driveCfg.mecFrontLeft;
    int fr = g_driveCfg.mecFrontRight;
    int rl = g_driveCfg.mecRearLeft;
    int rr = g_driveCfg.mecRearRight;


    if (fl < 0 || fr < 0 || rl < 0 || rr < 0) return;
    if (fl >= (int)g_numDCMotors || fr >= (int)g_numDCMotors ||
        rl >= (int)g_numDCMotors || rr >= (int)g_numDCMotors) return;

    driveMecanum(g_chX, g_chY, g_chR, g_dcMotors[fl], g_dcMotors[fr], g_dcMotors[rl], g_dcMotors[rr]);
  }
}

// ===== Servo Smooth 更新 =====
static void updateServosSmooth() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  lastUpdate = now;

  if (!g_servos || !g_servoObjs) return;

  for (int i = 0; i < (int)g_numServos; i++) {
    const V7RC_ServoConfig &cfg = g_servos[i];

    float delta = g_targetDeg[i] - g_currentDeg[i];
    if (fabsf(delta) < g_smooth.deadbandDeg) continue;

    float step = g_smooth.maxStepDeg;
    if (fabsf(delta) <= step) {
      g_currentDeg[i] = g_targetDeg[i];
    } else {
      g_currentDeg[i] += (delta > 0 ? step : -step);
    }

    uint16_t us = degToUs(cfg, g_currentDeg[i]);
    g_servoObjs[i].writeMicroseconds(us);
  }
}

// ===== 安全保護（逾時停車 + Servo 回中立） =====
// ---------- WS2812 helpers ----------

static void setWs2812Color(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (!g_ws2812 || index >= g_ws2812Count) return;
  g_ws2812->setPixelColor(index, g_ws2812->Color(r, g, b));
  g_ws2812->show();
}

static void setWs2812All(uint8_t r, uint8_t g, uint8_t b) {
  if (!g_ws2812) return;
  for (uint8_t i = 0; i < g_ws2812Count; i++) {
    g_ws2812->setPixelColor(i, g_ws2812->Color(r, g, b));
  }
  g_ws2812->show();
}

// public wrappers exposed via V7RCServoDriver class
void V7RCServoDriver::setLedColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  setWs2812Color(index, r, g, b);
}

void V7RCServoDriver::setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  setWs2812All(r, g, b);
}

static void safetyStop() {
  // Servo 回 initDeg
  if (g_servos && g_servoObjs && g_currentDeg && g_targetDeg) {
    for (int i = 0; i < (int)g_numServos; i++) {
      g_targetDeg[i]  = g_servos[i].initDeg;
      g_currentDeg[i] = g_servos[i].initDeg;
      uint16_t us = degToUs(g_servos[i], g_servos[i].initDeg);
      g_servoObjs[i].writeMicroseconds(us);
    }
  }

  // DC Motor 停車
  if (g_dcMotors) {
    for (int i = 0; i < (int)g_numDCMotors; i++) {
      processDCMotorPwm(1500, g_dcMotors[i]);
    }
  }

  // turn off WS2812 leds if any
  if (g_ws2812) {
    setWs2812All(0, 0, 0);
  }
  // clear custom state so connection animation resumes next time
  g_ledCustomActive = false;

  g_driveEnabled = false;
}

static void handleTimeoutSafety() {
  if (g_lastFrameMs == 0) return;
  unsigned long now = millis();
  if (now - g_lastFrameMs > COMMAND_TIMEOUT_MS) {
    safetyStop();
    g_lastFrameMs = 0;
    Serial.println("Timeout → safety stop");
  }
}

// update LED animation (connection state or custom per-led settings)
static void updateLedAnimation() {
  if (!g_ws2812) return;

  if (g_ledCustomActive) {
    uint32_t now = millis();
    for (uint8_t i = 0; i < g_ws2812Count; i++) {
      LedState &s = g_ledStates[i];
      uint8_t r = 0, g = 0, b = 0;
      if (s.blinkLevel > 0) {
        uint32_t period = 1000;
        uint32_t onTime = s.blinkLevel * 100;
        if (onTime >= period) {
          r = s.r; g = s.g; b = s.b;
        } else {
          uint32_t phase = now % period;
          if (phase < onTime) {
            r = s.r; g = s.g; b = s.b;
          }
        }
      }
      g_ws2812->setPixelColor(i, g_ws2812->Color(r, g, b));
    }
    g_ws2812->show();
  } else {
    if (!deviceConnected) {
      unsigned long now = millis();
      if (now - g_ledLastToggleMs >= 1000) {
        g_ledBlinkState = !g_ledBlinkState;
        g_ledLastToggleMs = now;
        if (g_ledBlinkState) {
          setWs2812All(255, 0, 0);
        } else {
          setWs2812All(0, 0, 0);
        }
      }
    } else {
      setWs2812All(0, 255, 0);
    }
  }
}

// ===== Frame Byte 流處理（共用 Serial / BLE） =====
static void handleLedCommand(const char *cmd) {
  // cmd does not include trailing '#'
  int len = strlen(cmd);
  if (len < 3) return;
  if (cmd[0] != 'L' || cmd[1] != 'E') return;
  char grpChar = cmd[2];
  int group;
  // LED or LE1 = group0 (first four leds)
  if (grpChar == 'D' || grpChar == '1') {
    group = 0;
  } else if (grpChar >= '2' && grpChar <= '9') {
    // LE2->group1, LE3->group2, ...
    group = grpChar - '1';
  } else {
    return;
  }
  if (!g_ws2812 || g_ws2812Count == 0) return;
  int start = group * 4;
  if (start >= g_ws2812Count) return;

  const char *p = cmd + 3;
  for (int led = 0; led < 4; led++) {
    int idx = start + led;
    if (idx >= g_ws2812Count) break;
    if ((int)(p - cmd) + 4 > len) break;
    uint8_t r = hexNibble(p[0]) * 17;
    uint8_t g = hexNibble(p[1]) * 17;
    uint8_t b = hexNibble(p[2]) * 17;
    uint8_t blink = hexNibble(p[3]);
    if (blink > 10) blink = 10;
    g_ledStates[idx].r = r;
    g_ledStates[idx].g = g;
    g_ledStates[idx].b = b;
    g_ledStates[idx].blinkLevel = blink;
    p += 4;
  }

  g_ledCustomActive = true;
}

static char ledBuf[64];
static int  ledIndex = 0;

static void processIncomingByte(char c) {
  if (c == '#') {
    // finalize text buffer
    if (ledIndex < (int)sizeof(ledBuf) - 1) {
      ledBuf[ledIndex] = '#';
      ledBuf[ledIndex + 1] = '\0';
    } else {
      ledBuf[0] = '\0';
    }

    // try to decode as frame (including LED/LE1..LE4)
    V7RC_Frame frame = decodeV7RC(ledBuf);
    if (frame.valid) {
      if (frame.type == V7RC_LED) {
      handleLedCommand(ledBuf);
    } else {
      applyFrameToTargets(frame);
    }
    }

    // reset all indices
    ledIndex = 0;
    rxIndex  = 0;
  } else {
    // accumulate both raw bytes and text buffer
    if (rxIndex < V7RC_FRAME_SIZE - 1) {
      rxBuf[rxIndex++] = c;
    } else {
      rxIndex = 0;  // corrupted frame, restart
    }
    if (ledIndex < (int)(sizeof(ledBuf) - 1)) {
      ledBuf[ledIndex++] = c;
    } else {
      ledIndex = 0;
    }
  }
}

static void processSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    processIncomingByte(c);
  }
}

// ===== NimBLE Callbacks（h2zero 版正確簽名）=====
class V7RC_NimBLEServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    (void)pServer;
    (void)connInfo;
    deviceConnected = true;
    g_lastFrameMs   = 0;
    Serial.println("BLE connected");
    // change LEDs to steady green when connection is established
    if (g_ws2812) {
      setWs2812All(0, 255, 0);
    }
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer;
    (void)connInfo;
    (void)reason;
    deviceConnected = false;
    Serial.println("BLE disconnected");
    safetyStop();
    NimBLEDevice::startAdvertising();
    // reset blink animation timer so red blink starts immediately
    if (g_ws2812) {
      g_ledBlinkState = false;
      g_ledLastToggleMs = millis();
      setWs2812All(0, 0, 0);
    }
  }
};

class V7RC_NimBLECharacteristicCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;

    std::string value = pCharacteristic->getValue();
    if (value.empty()) return;

    // Debug：確認真的有收到 V7RC APP 寫入
    // Serial.printf("RX len=%u\n", (unsigned)value.size());

    for (size_t i = 0; i < value.size(); i++) {
      processIncomingByte((char)value[i]);
    }
  }
};

// ===== BLE 初始化（h2zero NimBLE-Arduino 相容寫法）=====
static void setupBLE() {
  NimBLEDevice::init(g_bleDeviceName);
  NimBLEDevice::setMTU(247);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new V7RC_NimBLEServerCallbacks());

  NimBLEService* pService = pServer->createService(
    NimBLEUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
  );

  // RX: App -> Device（WRITE + WRITE_NR）
  pRxCharacteristic = pService->createCharacteristic(
    NimBLEUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new V7RC_NimBLECharacteristicCallbacks());

  // TX: Device -> App（NOTIFY）
  pTxCharacteristic = pService->createCharacteristic(
    NimBLEUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    NIMBLE_PROPERTY::NOTIFY
  );
  // pTxCharacteristic->addDescriptor(new NimBLE2902());

  pService->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->reset();
  adv->addServiceUUID(pService->getUUID());
  adv->setName(g_bleDeviceName);

  NimBLEDevice::startAdvertising();

  Serial.print("BLE advertising as: ");
  Serial.println(g_bleDeviceName);
}

// ===== Servo / DC 初始化 =====
static void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  if (!g_servos || g_numServos == 0) return;

  g_servoObjs  = new Servo[g_numServos];
  g_currentDeg = new float[g_numServos];
  g_targetDeg  = new float[g_numServos];

  for (int i = 0; i < (int)g_numServos; i++) {
    const V7RC_ServoConfig &cfg = g_servos[i];

    g_servoObjs[i].setPeriodHertz(50);
    g_servoObjs[i].attach(cfg.pin, cfg.minUs, cfg.maxUs);

    g_currentDeg[i] = cfg.initDeg;
    g_targetDeg[i]  = cfg.initDeg;

    uint16_t us = degToUs(cfg, cfg.initDeg);
    g_servoObjs[i].writeMicroseconds(us);

    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(" on pin ");
    Serial.print(cfg.pin);
    Serial.print(", initDeg=");
    Serial.print(cfg.initDeg);
    Serial.print(" (");
    Serial.print(us);
    Serial.println(" us)");
  }
}

static void initDCMotors() {
  if (!g_dcMotors || g_numDCMotors == 0) return;

  for (int i = 0; i < (int)g_numDCMotors; i++) {
    pinMode(g_dcMotors[i].pinDir, OUTPUT);
    pinMode(g_dcMotors[i].pinPwm, OUTPUT);
    digitalWrite(g_dcMotors[i].pinDir, LOW);
    analogWrite(g_dcMotors[i].pinPwm, 0);
  }
}

// ===== V7RCServoDriver 實作 =====
void V7RCServoDriver::begin(uint8_t robotId, const V7RC_DriverConfig& cfg) {
  if (robotId < 1)  robotId = 1;
  if (robotId > 99) robotId = 99;

  g_servos      = cfg.servos;
  g_numServos   = cfg.numServos;
  g_smooth      = cfg.smooth;
  g_waveDemoServoIndex = cfg.waveDemoServoIndex;

  g_dcMotors    = cfg.dcMotors;
  g_numDCMotors = cfg.numDCMotors;

  g_channelMap   = cfg.channelMap;
  g_numChannelMap= cfg.numChannelMap;

  g_driveCfg     = cfg.drive;
  g_driveEnabled = false;

  g_diffThrottle = 0.0f;
  g_diffSteer    = 0.0f;
  g_mecVx        = 0.0f;
  g_mecVy        = 0.0f;
  g_mecOmega     = 0.0f;
  g_chX          = 1500;
  g_chY          = 1500;
  g_chR          = 1500;

  snprintf(g_bleDeviceName, sizeof(g_bleDeviceName),
           "%s-%02u", cfg.bleBaseName ? cfg.bleBaseName : "V7RC", robotId);

  Serial.println();
  Serial.print("V7RC Servo Driver Begin, BLE Name = ");
  Serial.println(g_bleDeviceName);

  initServos();
  initDCMotors();
  setupBLE();

  g_lastFrameMs = 0;

  // optional WS2812 initialization
  if (cfg.ws2812Enable) {
    uint8_t count = cfg.ws2812Count;
    if (count == 0) {
      count = 8; // default count
    }
    uint8_t pin = cfg.ws2812Pin ? cfg.ws2812Pin : 8;
    g_ws2812Count = count;
    g_ws2812 = new Adafruit_NeoPixel(g_ws2812Count, pin, NEO_GRB + NEO_KHZ800);
    g_ws2812->begin();

    // apply brightness (user-specified, or fallback default)
    uint8_t br = cfg.ws2812Brightness;
    if (br == 0) {
      br = 50; // sensible default to avoid full power
    }
    g_ws2812->setBrightness(br);

    // allocate state array
    g_ledStates = new LedState[g_ws2812Count];
    for (uint8_t i = 0; i < g_ws2812Count; i++) {
      g_ledStates[i].r = 255;
      g_ledStates[i].g = 0;
      g_ledStates[i].b = 0;
      g_ledStates[i].blinkLevel = 5; // default blink 500ms on
      g_ws2812->setPixelColor(i, g_ws2812->Color(255, 0, 0));
    }
    g_ws2812->show();
    // initialize blink timing
    g_ledBlinkState   = true;
    g_ledLastToggleMs = millis();
  }
}

void V7RCServoDriver::loop() {
  unsigned long startTime = millis();

  processSerial();
  updateDrive();
  updateServosSmooth();
  handleTimeoutSafety();
  updateLedAnimation();

  unsigned long endTime = millis();
  int waitTime = (int)g_smooth.updateMs - (int)(endTime - startTime);
  if (waitTime > 2) {
    delay(waitTime - 1);
  }
}