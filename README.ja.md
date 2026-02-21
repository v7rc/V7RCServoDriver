# V7RCServoDriver（日本語）

ESP32 向けの **V7RC BLE 低遅延制御ライブラリ**です。**V7RC App（V7RCDOM V2.0）** からの入力を
**サーボ / DC モーター**にマッピングし、**差動駆動** と **メカナム駆動**をサポートします。

## ドキュメント
- Quick Start（教育向け 10分）：`QUICK_START_EDU.md`
- Examples：
  - メカナム：`examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 差動駆動：`examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`


> **LED 動作**
> WS2812 を有効にすると、未接続時にストリップが赤で点滅（一秒ごと）し、
> BLE 接続後は緑色の常灯になります。
> LED 機能を使用するには `V7RC_DriverConfig` の `ws2812Enable` を true にしてください。
>
> ### LED プロトコル
> カスタム V7RC プロトコルでは各 LED の色や点滅を設定できます。形式は
> `LE?yyyyyyyyyyyyyyyy#` です。`?` は `D` または数字 (`1`–`9`) で、
> `D` と `1` は最初の 4 灯を指し、`2` は 5–8 灯、`3` は 9–12 灯を意味します。
> 数字 n は 4 灯ごとの n‑1 番目のグループを選び、さらに大きな数字で
> グループを拡張できます。
> 各 LED には 4 文字を使い、順に R/G/B/点滅値です。
> R/G/B は 16 進数 `0`–`F`（0..15）で記述し、0..255 に *17 倍します。
> `blink` は `0`–`A`（0..10）で、1 秒あたりの点灯時間を 0–1000 ms（値×100）で示します。
> 例：`LEDF00A0F0A00FAFFF5#` は最初の 4 灯を設定し、1 番目が赤、2 番目が緑、
> 3 番目が青、4 番目が 500 ms 点灯/500 ms 消灯の白になります。
>
> 受信した命令はライブラリで解析され即時に適用され、カスタム設定は
> デフォルトの接続アニメーションを上書きします（クリア命令は未実装）。

## Requirements
- ESP32 Arduino Core
- **ESP32Servo**
- **NimBLE-Arduino** (by h2zero)
- **Adafruit_NeoPixel** (for WS2812 LEDs)

## よくある問題
- NimBLE のヘッダが見つからない：**NimBLE-Arduino** をインストールし、`#include <NimBLEDevice.h>` を使用してください。

## Maintainer
V7RC / V7 Idea Technology Ltd.  
Maintainer: Louis Chuang <louis@v7idea.com.tw>