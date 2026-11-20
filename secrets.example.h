// Mau file thong tin bi mat.
// Copy file nay thanh "secrets.h" (cung thu muc) roi dien thong tin that.
// secrets.h da duoc dua vao .gitignore nen se KHONG bi day len GitHub.
#pragma once

// Lay tu Blynk Console -> Devices -> (thiet bi) -> Device Info
#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxxx"
#define BLYNK_TEMPLATE_NAME "Smart Fall-Detection Belt"
#define BLYNK_AUTH_TOKEN    "DIEN_AUTH_TOKEN_CUA_THIET_BI"

// Wi-Fi 2.4 GHz (ESP32 khong bat duoc 5 GHz)
const char WIFI_SSID[] = "TEN_WIFI";
const char WIFI_PASS[] = "MAT_KHAU_WIFI";
