# V7RCServoDriver（日本語）

ESP32 向けの **V7RC BLE 低遅延制御ライブラリ**です。**V7RC App（V7RCDOM V2.0）** からの入力を
**サーボ / DC モーター**にマッピングし、**差動駆動** と **メカナム駆動**をサポートします。

## ドキュメント
- Quick Start（教育向け 10分）：`QUICK_START_EDU.md`
- Examples：
  - メカナム：`examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 差動駆動：`examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`

## よくある問題
- NimBLE のヘッダが見つからない：**NimBLE-Arduino** をインストールし、`#include <NimBLEDevice.h>` を使用してください。
