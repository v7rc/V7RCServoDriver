# V7RCServoDriver（繁體中文）

這是一套給 ESP32 用的 **V7RC BLE 低延遲控制函式庫**，可與 **V7RC APP（V7RCDOM V2.0）** 搭配使用。
它會把 V7RC App 送來的控制 Channel 映射成 **Servo**、**DC 馬達**輸出，並支援 **差速車** 與 **麥克納姆輪**。

## 文件導覽
- Quick Start（教育版 10 分鐘上手）：`QUICK_START_EDU.md`
- 範例：
  - 麥克納姆輪：`examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - 差速車：`examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`

> **WS2812 LED**
> 範例已經啟用燈條（`.ws2812Enable = true`），使用 GPIO8、8 顆 LED；
> 開機時紅色閃爍（每秒亮/暗），BLE 連線後會自動改為恆亮綠色。
> 若不需要可把 `.ws2812Enable` 設為 `false`，那麼 `ws2812Pin`/`ws2812Count` 會被忽略；啟用後可
> 使用這兩個欄位修改腳位與數量，且設為零亦會得到預設值。
>
> ### LED 控制協議
> 從 App 傳送字串可個別設定燈色與閃爍。格式為 `LE?rrrrggggbbbbx#`。
> `?` 為 `D` 表示第1~4顆, `2` 表示第5~8顆, 以此類推。每顆 LED 用 4 個字元：
> 紅/綠/藍（`0`–`F`，乘 17 轉成 0–255），和閃爍程度 (`0`–`A` 表示 0~10 十分位
> 的 100ms 片段)。例如 `LED0123A4567B89C#` 將設定前四顆。送達後會立即生效，並
> 取代預設連線動畫。

## 常見問題（最常見）
- NimBLE 找不到 header：請安裝 **NimBLE-Arduino**，並使用 `#include <NimBLEDevice.h>`。

## Maintainer
V7RC / V7 Idea Technology Ltd.  
Maintainer: Louis Chuang <louis@v7idea.com.tw>