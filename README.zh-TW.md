# V7RCServoDriver（繁體中文）

這是一套給 ESP32 用的 **V7RC BLE 低延遲控制函式庫**，可與 **V7RC APP（V7RCDOM V2.0）** 搭配使用。
它會把 V7RC App 送來的控制 Channel 映射成 **Servo**、**DC 馬達**輸出，並支援 **差速車** 與 **麥克納姆輪**。

## 文件導覽
- Quick Start（教育版 10 分鐘上手）：`QUICK_START_EDU.md`
- 範例：
  - 麥克納姆輪：`examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 差速車：`examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`

## 常見問題（最常見）
- NimBLE 找不到 header：請安裝 **NimBLE-Arduino**，並使用 `#include <NimBLEDevice.h>`。
