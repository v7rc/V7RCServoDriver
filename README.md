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

> **LED behaviour**
> When WS2812 is enabled the strip starts red and blinks once per second while not
> connected. After a BLE connection is established the LEDs switch to steady green.
> LED support must still be enabled via `ws2812Enable` in `V7RC_DriverConfig`.
>
> ### LED protocol
> A custom V7RC protocol allows the app to set individual LEDs. It looks like:
> `LE?yyyyyyyyyyyyyyyy#` where `?` can be `D` or a digit (`1`–`9`). `D` and `1` both refer to the first four LEDs; `2` means LEDs 5‑8, `3` means 9‑12, etc. (using a digit `n` selects group `n‑1` of four LEDs). You can extend further by interpreting higher digits simply as additional groups.
> Each LED consumes 4 characters (R,G,B,blink). R/G/B are hex digits `0`–`F` (0..15)
> scaled to 0..255 (`*17`). `blink` is `0`–`A` (0..10) representing 0–1000 ms of on-time per
> second (`value*100`). Example:
> `LED0123A4567B89C#` sets the first four LEDs.
>
> Received commands are parsed by the library and applied immediately; custom
> settings override the default connection animation until cleared (no clear
> command currently).

## Features
- BLE (NUS) low-latency control via **NimBLE-Arduino**
- Supported protocols: `HEX`, `DEG`, `SRV / SRT`, `SS8`, `LED` (text LED control)
- Channel mapping to:
  - Servo (PWM or angle control)
  - DC Motor (DIR + PWM)
  - Differential drive (Throttle / Steer)
  - Mecanum drive (Vx / Vy / Omega)
- Built-in WS2812 RGB LED strip support (up to 8 LEDs by default; must enable via config)
- Servo smoothing and deadband
- Failsafe timeout (auto stop on signal loss)

## Requirements
- ESP32 Arduino Core
- **ESP32Servo**
- **NimBLE-Arduino** (by h2zero)
- **Adafruit_NeoPixel** (for WS2812 LEDs)

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
Maintainer: Louis Chuang <louis@v7idea.com.tw>
