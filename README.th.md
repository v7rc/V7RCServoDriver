# V7RCServoDriver (ภาษาไทย)

ไลบรารีควบคุม BLE แบบหน่วงต่ำสำหรับ ESP32 ใช้งานร่วมกับ **V7RC App (V7RCDOM V2.0)**
เพื่อควบคุม **Servo / DC Motor** และรองรับการขับเคลื่อนแบบ **Differential** และ **Mecanum**.

## เอกสาร
- Quick Start (การศึกษา 10 นาที): `QUICK_START_EDU.md`
- Examples:
  - Mecanum: `examples/ESP32_C3_Mini_Mecanum_V3/ESP32_C3_Mini_Mecanum_V3.ino`
  - Differential: `examples/ESP32_C3_Differential_Drive/ESP32_C3_Differential_Drive.ino`

## ปัญหาที่พบบ่อย
- หา header ของ NimBLE ไม่เจอ: ติดตั้ง **NimBLE-Arduino** และใช้ `#include <NimBLEDevice.h>`

## Maintainer
V7RC / V7 Idea Technology Ltd.  
Maintainer: Louis Chuang <louis@v7idea.com.tw>