# V7RCServoDriver（繁體中文）

這是一套給 ESP32 用的 **V7RC BLE 低延遲控制函式庫**，可與 **V7RC APP（V7RCDOM V2.0）** 搭配使用。
它會把 V7RC App 送來的控制 Channel 映射成 **Servo**、**DC 馬達**輸出，並支援 **差速車** 與 **麥克納姆輪**。

## 文件導覽
- Quick Start（教育版 10 分鐘上手）：`QUICK_START_EDU.md`
- 範例：
  - 麥克納姆輪：`examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 差速車：`examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`

> **LED 行為**
> 當 WS2812 啟用時，燈條在未連線時會先顯示紅色並每秒閃爍一次；當 BLE 連線建立後，LED 會改為恆亮綠色。
> 若要使用 LED 功能，需在 `V7RC_DriverConfig` 中把 `ws2812Enable` 設為 true。
>
> ### LED 協議
> 使用自訂的 V7RC 協議可讓 App 設定每顆 LED。格式為 `LE?yyyyyyyyyyyyyyyy#`，其中 `?` 可為 `D` 或數字 (`1`–`9`)。`D` 與 `1` 均指第一組四顆 LED；`2` 表示第 5–8 顆，`3` 表示第 9–12 顆，以此類推（數字 n 會選擇第 n‑1 組 4 顆）。可用更大的數字擴充到更多組。
> 每顆 LED 使用四個字元（R、G、B、blink）。R/G/B 為 16 進位 `0`–`F`（0..15），乘 17 後轉成 0..255。`blink` 為 `0`–`A`（0..10），表示每秒開啟時間 0–1000 ms（值×100）。例如：
> `LEDF00A0F0A00FAFFF5#` 設定前四顆 LED，第一顆為紅色，第二顆為綠色，第三顆為藍色，第四顆為白色並以 500ms 亮/500ms 滅閃。

## 常見問題（最常見）
- NimBLE 找不到 header：請安裝 **NimBLE-Arduino**，並使用 `#include <NimBLEDevice.h>`。

## Maintainer
V7RC / V7 Idea Technology Ltd.  
Maintainer: Louis Chuang <louis@v7idea.com.tw>