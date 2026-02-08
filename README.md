# V7RCServoDriver

**Low-latency BLE control driver for ESP32**, compatible with the **V7RC App (V7RCDOM V2.0)**.
It maps incoming control channels to **servos**, **DC motors**, and common robot kinematics such as
**Differential Drive** and **Mecanum Drive**.

## Languages
- English (this file)
- 繁體中文：`README.zh-TW.md`
- 日本語：`README.ja.md`
- ภาษาไทย：`README.th.md`

## Quick Start (Education / 10 minutes)
See: `QUICK_START_EDU.md`

## Examples
- Mecanum (ESP32-C3): `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
- Differential Drive (ESP32-C3): `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`

## Features
- BLE (NUS) low-latency control via **NimBLE-Arduino**
- Supported protocols: `HEX`, `DEG`, `SRV / SRT`, `SS8`
- Channel mapping to:
  - Servo (angle control)
  - DC Motor (DIR + PWM)
  - Differential drive (Throttle / Steer)
  - Mecanum drive (Vx / Vy / Omega)
- Servo smoothing and deadband
- Failsafe timeout (auto stop on signal loss)

## Requirements
- ESP32 Arduino Core
- **ESP32Servo**
- **NimBLE-Arduino** (by h2zero)

Install dependencies via:
Arduino IDE → Tools → Manage Libraries

## Troubleshooting (most common)
### NimBLE header not found
Install **NimBLE-Arduino** and include:
```cpp
#include <NimBLEDevice.h>
```
Avoid legacy headers such as `NimBLE2902.h`.

## License
MIT License. See `LICENSE`.

## Maintainer
V7RC / V7 Idea Technology Ltd.  
Maintainer: louis
