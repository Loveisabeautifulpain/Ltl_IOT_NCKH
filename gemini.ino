/*
 * THAT LUNG THONG MINH CANH BAO TE NGA - PHIEN BAN BLYNK IoT
 * Phat trien tu gemini.ino (giu nguyen thuat toan 3 nhanh + may trang thai)
 * Bo sung: Wi-Fi + Blynk IoT (dashboard, thong bao khan cap, tat bao dong tu xa)
 *
 * NGUYEN TAC: Coi canh bao tai thiet bi LUON hoat dong ke ca khi mat Wi-Fi.
 *             Mang chi dung de gui thong bao ra ngoai.
 *
 * Thu vien can cai: "Blynk" (by Volodymyr Shymanskyy) trong Library Manager
 *
 * ===== BANG CHAN AO (khop template "Smart Fall-Detection Belt") =====
 *  V0  System Status    String          - Labeled Value
 *  V1  Tilt Angle       Double 0..180   - Gauge
 *  V2  Acceleration (G) Double 0..5     - SuperChart
 *  V3  Remote Silence   Integer 0..1    - Button (Push)
 *
 * ===== EVENT =====
 *  Code: fall_alert  (Critical, Device Owner)
 */

// ================= 0. THONG TIN BLYNK & WI-FI =================
// Token va mat khau Wi-Fi nam trong secrets.h (KHONG dua len GitHub).
// Lan dau: copy secrets.example.h -> secrets.h roi dien thong tin that.
#include "secrets.h"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <math.h>

// ================= 1. KHAI BAO PHAN CUNG (STICK 'EM ESP32) =================
#define I2C_SDA           21
#define I2C_SCL           22
#define MPU_ADDR          0x68
#define BUZZER_PIN        25    // Coi Buzzer tich hop
#define TOUCH_CANCEL_PIN  4     // Cong 2 - chan Vang (S) cam ung cham

// ================= 2. CAC NGUONG THUAT TOAN (giu nguyen ban gemini.ino) =================
#define TOUCH_RATIO       0.80
#define FREEFALL_G        0.55
#define IMPACT_G          2.05
#define COLLAPSE_GYRO     110.0
#define COLLAPSE_TILT     55.0
#define SLOW_TILT_DEG     65.0
#define SLOW_TILT_MS      1200
#define CHECK_TIME_MS     2500
#define SETTLE_TIME_MS    500
#define STILL_GYRO_MAX    25.0
#define STANDUP_TILT      40.0
#define COUNTDOWN_MS      10000

// Chu ky gui du lieu len Blynk
#define TELEMETRY_MS      1000   // Gui G/Gyro/Goc moi 1 giay (tranh vuot gioi han goi mien phi)
#define RECONNECT_MS      15000  // Thu ket noi lai Blynk moi 15 giay (chi khi dang IDLE)

// ================= 3. BIEN HE THONG =================
enum SystemState {
    STATE_IDLE,
    STATE_POST_FALL_CHECK,
    STATE_COUNTDOWN,
    STATE_ALARM
};

const char* STATE_TEXT[] = { "Normal", "Checking...", "Countdown", "FALL ALERT!" };

SystemState currentState = STATE_IDLE;
SystemState lastReportedState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long freeFallTime = 0;
unsigned long slowTiltStartTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastBeepTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastReconnectTry = 0;
bool alarmSent = false;

int totalCheckSamples = 0;
int stillCheckSamples = 0;

float gxOffset = 0, gyOffset = 0, gzOffset = 0;
float refAx = 0, refAy = 0, refAz = 1.0;
float smoothAx = 0, smoothAy = 0, smoothAz = 1.0;
int touchBaseline = 70;

// Co lenh tu app Blynk
volatile bool remoteCancel = false;   // V3
bool pendingAlertEvent = false;       // Bao dong xay ra luc mat mang -> gui bu khi co mang
String lastFallType = "";             // Nhanh nao kich hoat (ghi vao thong bao)

// Gia tri cam bien moi nhat (de gui len Blynk)
float lastG = 1.0, lastGyro = 0, lastTilt = 0;

// ================= 4. HAM HO TRO =================
void readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax = (int16_t)(Wire.read() << 8 | Wire.read());
    ay = (int16_t)(Wire.read() << 8 | Wire.read());
    az = (int16_t)(Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read();
    gx = (int16_t)(Wire.read() << 8 | Wire.read());
    gy = (int16_t)(Wire.read() << 8 | Wire.read());
    gz = (int16_t)(Wire.read() << 8 | Wire.read());
}

void playCancelSound() {
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, 1200, 100); delay(130);
    tone(BUZZER_PIN, 900, 150);  delay(180);
    noTone(BUZZER_PIN);
}

bool isCancelTriggered() {
    // Lenh tat tu xa tu app Blynk (nguoi than)
    if (remoteCancel) {
        remoteCancel = false;
        Serial.println(">>> [BLYNK] Nguoi than tat bao dong tu xa.");
        return true;
    }

    int val = touchRead(TOUCH_CANCEL_PIN);
    int threshold = (int)(touchBaseline * TOUCH_RATIO);
    if (val > 0 && val < threshold) {
        delay(60);
        int val2 = touchRead(TOUCH_CANCEL_PIN);
        if (val2 > 0 && val2 < threshold) {
            Serial.print(">>> [CHAM GPIO 4] Gia tri: ");
            Serial.print(val2);
            Serial.print(" / Nen: ");
            Serial.println(touchBaseline);
            return true;
        }
    }

    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            Serial.println(">>> [LENH SERIAL] Nhan lenh 'c' huy canh bao.");
            return true;
        }
    }
    return false;
}

void resetToIdle() {
    noTone(BUZZER_PIN);
    currentState = STATE_IDLE;
    slowTiltStartTime = 0;
    totalCheckSamples = 0;
    stillCheckSamples = 0;
}

void startCountdown(unsigned long now) {
    currentState = STATE_COUNTDOWN;
    stateStartTime = now;
    lastBeepTime = 0;
}

// ================= 5. BLYNK =================
// V3: Nut "Tat bao dong" tren app (che do Push)
BLYNK_WRITE(V3) {
    if (param.asInt() == 1) remoteCancel = true;
}

// Khi vua ket noi (hoac ket noi lai): dong bo trang thai, gui bu canh bao bi lo
BLYNK_CONNECTED() {
    Serial.println(">>> [BLYNK] Da ket noi server.");
    Blynk.virtualWrite(V0, STATE_TEXT[currentState]);
    if (pendingAlertEvent) {
        Blynk.logEvent("fall_alert", "Phat hien te nga (" + lastFallType + ") - gui lai sau khi mat mang");
        pendingAlertEvent = false;
    }
}

// Ket noi Wi-Fi/Blynk KHONG chan chuong trinh: mat mang thi thuat toan van chay
void handleNetwork(unsigned long now) {
    if (WiFi.status() != WL_CONNECTED) return;       // ESP32 tu ket noi lai Wi-Fi
    if (Blynk.connected()) {
        Blynk.run();
        return;
    }
    // Chi thu ket noi lai khi dang IDLE, de khong lam tre coi/dem nguoc
    if (currentState == STATE_IDLE && now - lastReconnectTry >= RECONNECT_MS) {
        lastReconnectTry = now;
        Serial.println(">>> [BLYNK] Dang thu ket noi...");
        Blynk.connect(3000);
    }
}

void sendTelemetry(unsigned long now) {
    if (now - lastTelemetryTime < TELEMETRY_MS) return;
    lastTelemetryTime = now;
    if (Blynk.connected()) {
        Blynk.virtualWrite(V1, lastTilt);
        Blynk.virtualWrite(V2, lastG);
        // Trong luc dem nguoc: hien so giay con lai ngay tren o trang thai
        if (currentState == STATE_COUNTDOWN) {
            long left = (COUNTDOWN_MS - (long)(now - stateStartTime)) / 1000 + 1;
            if (left < 0) left = 0;
            Blynk.virtualWrite(V0, String("Countdown ") + left + "s");
        }
    }
    lastG = 0; lastGyro = 0;   // Bat dau chu ky moi: lay gia tri lon nhat trong 1 giay
}

// Goi moi vong lap: phat hien chuyen trang thai de bao len Blynk ngay lap tuc
void reportStateChange() {
    if (currentState == lastReportedState) return;
    SystemState prev = lastReportedState;
    lastReportedState = currentState;

    bool online = Blynk.connected();
    if (online) Blynk.virtualWrite(V0, STATE_TEXT[currentState]);

    if (currentState == STATE_ALARM) {
        if (online) {
            Blynk.logEvent("fall_alert", "Nguoi deo co the da te nga (" + lastFallType + "). Hay kiem tra ngay!");
        } else {
            pendingAlertEvent = true;   // Gui bu khi co mang lai
            Serial.println(">>> [BLYNK] Mat mang - canh bao se gui khi ket noi lai.");
        }
    }
    if (currentState == STATE_IDLE && (prev == STATE_COUNTDOWN || prev == STATE_ALARM)) {
        pendingAlertEvent = false;      // Da huy thi khong gui bu nua
    }
}

// ================= 6. KHOI TAO & HIEU CHUAN =================
void setup() {
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Bat dau ket noi Wi-Fi o che do nen (khong cho)
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Blynk.config(BLYNK_AUTH_TOKEN);

    delay(500);
    Serial.println("\n=======================================================");
    Serial.println("DANG HIEU CHUAN... GIU MACH O TU THE DUNG & KHONG CHAM CONG 2");
    tone(BUZZER_PIN, 1500, 150);

    long sumAx = 0, sumAy = 0, sumAz = 0;
    long sumGx = 0, sumGy = 0, sumGz = 0;
    int16_t ax, ay, az, gx, gy, gz;
    for (int i = 0; i < 200; i++) {
        readRawMPU(ax, ay, az, gx, gy, gz);
        sumAx += ax; sumAy += ay; sumAz += az;
        sumGx += gx; sumGy += gy; sumGz += gz;
        delay(5);
    }
    gxOffset = sumGx / 200.0;
    gyOffset = sumGy / 200.0;
    gzOffset = sumGz / 200.0;

    float avgAx = (sumAx / 200.0) / 16384.0;
    float avgAy = (sumAy / 200.0) / 16384.0;
    float avgAz = (sumAz / 200.0) / 16384.0;
    float refMag = sqrt(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);
    if (refMag > 0.1) {
        refAx = avgAx / refMag;
        refAy = avgAy / refMag;
        refAz = avgAz / refMag;
        smoothAx = avgAx; smoothAy = avgAy; smoothAz = avgAz;
    }

    long sumTouch = 0;
    for (int i = 0; i < 30; i++) {
        sumTouch += touchRead(TOUCH_CANCEL_PIN);
        delay(15);
    }
    touchBaseline = sumTouch / 30;

    tone(BUZZER_PIN, 2000, 120); delay(160);
    tone(BUZZER_PIN, 2500, 120);

    Serial.print(">>> Muc nen cam ung GPIO 4: ");
    Serial.print(touchBaseline);
    Serial.print(" (Nguong tat < ");
    Serial.print((int)(touchBaseline * TOUCH_RATIO));
    Serial.println(")");

    // Cho Wi-Fi toi da 8 giay luc khoi dong; khong duoc thi van chay che do offline
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print(">>> Wi-Fi OK, IP: "); Serial.println(WiFi.localIP());
        Blynk.connect(5000);
    } else {
        Serial.println(">>> Chua co Wi-Fi - chay che do OFFLINE (coi van hoat dong).");
    }
    lastReconnectTry = millis();

    Serial.println(">>> SAN SANG GIAM SAT! (Go 't' de test nhanh dem nguoc, 'c' de huy)");
    Serial.println("=======================================================\n");
}

// ================= 7. VONG LAP MAY TRANG THAI (50Hz) =================
void loop() {
    unsigned long now = millis();
    handleNetwork(now);

    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    float ax = rawAx / 16384.0;
    float ay = rawAy / 16384.0;
    float az = rawAz / 16384.0;
    float gx = (rawGx - gxOffset) / 131.0;
    float gy = (rawGy - gyOffset) / 131.0;
    float gz = (rawGz - gzOffset) / 131.0;

    float totalG = sqrt(ax * ax + ay * ay + az * az);
    float totalGyro = sqrt(gx * gx + gy * gy + gz * gz);

    smoothAx = 0.75 * smoothAx + 0.25 * ax;
    smoothAy = 0.75 * smoothAy + 0.25 * ay;
    smoothAz = 0.75 * smoothAz + 0.25 * az;
    float smoothMag = sqrt(smoothAx * smoothAx + smoothAy * smoothAy + smoothAz * smoothAz);

    float tiltAngle = 0;
    if (smoothMag > 0.1) {
        float dot = (smoothAx * refAx + smoothAy * refAy + smoothAz * refAz) / smoothMag;
        dot = constrain(dot, -1.0f, 1.0f);
        tiltAngle = acos(dot) * 180.0 / PI;
    }

    // Luu gia tri lon nhat trong chu ky gui de khong bo lo dinh va cham tren bieu do
    lastG = max(lastG, totalG);
    lastGyro = max(lastGyro, totalGyro);
    lastTilt = tiltAngle;

    now = millis();

    switch (currentState) {
        case STATE_IDLE:
            if (Serial.available() > 0) {
                char cmd = Serial.read();
                if (cmd == 't' || cmd == 'T') {
                    lastFallType = "Thu nghiem Serial";
                    startCountdown(now);
                    Serial.println("\n>>> [TEST NHANH] Chuyen sang DEM NGUOC 10s! Hay cham tay vao GPIO 4...");
                    break;
                }
            }

            if (now - lastPrintTime >= 300) {
                lastPrintTime = now;
                Serial.print("[IDLE] G: "); Serial.print(totalG, 2);
                Serial.print("g | Gyro: "); Serial.print(totalGyro, 1);
                Serial.print(" do/s | Goc lech: "); Serial.print(tiltAngle, 1);
                Serial.print(" do | Blynk: "); Serial.println(Blynk.connected() ? "ON" : "OFF");
            }

            if (totalG < FREEFALL_G) freeFallTime = now;

            if (tiltAngle > SLOW_TILT_DEG) {
                if (slowTiltStartTime == 0) slowTiltStartTime = now;
            } else {
                slowTiltStartTime = 0;
            }

            if (totalG > IMPACT_G) {
                currentState = STATE_POST_FALL_CHECK;
                stateStartTime = now;
                totalCheckSamples = 0; stillCheckSamples = 0;
                lastFallType = (now - freeFallTime < 600) ? "Roi tu do + va cham" : "Va cham manh";
                Serial.println("\n>>> [NGHI NGO - NHANH 1] Phat hien VA CHAM MANH! Dang kiem tra 2.5s...");
            }
            else if (totalGyro > COLLAPSE_GYRO && tiltAngle > COLLAPSE_TILT) {
                currentState = STATE_POST_FALL_CHECK;
                stateStartTime = now;
                totalCheckSamples = 0; stillCheckSamples = 0;
                lastFallType = "Do nguoi nhanh";
                Serial.println("\n>>> [NGHI NGO - NHANH 2] Phat hien DO NGUOI NHANH! Dang kiem tra 2.5s...");
            }
            else if (slowTiltStartTime > 0 && (now - slowTiltStartTime >= SLOW_TILT_MS)) {
                currentState = STATE_POST_FALL_CHECK;
                stateStartTime = now;
                slowTiltStartTime = 0;
                totalCheckSamples = 0; stillCheckSamples = 0;
                lastFallType = "Guc nguoi tu tu";
                Serial.println("\n>>> [NGHI NGO - NHANH 3] Phat hien GUC NGUOI TU TU! Dang kiem tra 2.5s...");
            }
            break;

        case STATE_POST_FALL_CHECK:
            if (now - stateStartTime > SETTLE_TIME_MS) {
                if (tiltAngle < STANDUP_TILT) {
                    Serial.println(">>> [HUY NGHI NGO] Da dung day / tu the binh thuong.\n");
                    resetToIdle();
                    break;
                }
                totalCheckSamples++;
                if (totalGyro < STILL_GYRO_MAX) stillCheckSamples++;
            }
            if (now - stateStartTime >= CHECK_TIME_MS) {
                float stillRatio = (totalCheckSamples > 0) ? ((float)stillCheckSamples / totalCheckSamples) : 0;
                if (tiltAngle >= COLLAPSE_TILT && stillRatio >= 0.75) {
                    startCountdown(now);
                    Serial.println("\n!!! [XAC NHAN NGA QUY] Bat dau dem nguoc 10s! CHAM TAY CONG 2 (GPIO 4) DE HUY !!!");
                } else {
                    Serial.println(">>> [HUY NGHI NGO] Van con cu dong, khong phai bat tinh.\n");
                    resetToIdle();
                }
            }
            break;

        case STATE_COUNTDOWN:
            if (isCancelTriggered()) {
                playCancelSound();
                Serial.println(">>> [DA HUY THANH CONG] Nguoi dung xac nhan an toan. Quay ve giam sat.\n");
                resetToIdle();
                delay(600);
                break;
            }
            if (now - lastBeepTime >= 1000) {
                lastBeepTime = now;
                tone(BUZZER_PIN, 2000, 180);
                int secLeft = (COUNTDOWN_MS - (now - stateStartTime)) / 1000 + 1;
                if (secLeft < 0) secLeft = 0;
                Serial.print("... Dem nguoc: "); Serial.print(secLeft);
                Serial.println(" giay (Cham GPIO 4 de tat)");
            }
            if (now - stateStartTime >= COUNTDOWN_MS) {
                currentState = STATE_ALARM;
                alarmSent = false;
                lastBeepTime = 0;
            }
            break;

        case STATE_ALARM:
            if (!alarmSent) {
                Serial.println("\n=======================================================");
                Serial.println("!!! CANH BAO KHAN CAP: PHAT HIEN NGA QUY !!!");
                Serial.println(Blynk.connected() ? "-> Da gui thong bao len Blynk." : "-> Mat mang: se gui khi ket noi lai.");
                Serial.println("-> CHAM TAY CONG 2 (GPIO 4) hoac bam nut tren app de tat coi.");
                Serial.println("=======================================================\n");
                alarmSent = true;
            }
            if (now - lastBeepTime >= 350) {
                lastBeepTime = now;
                tone(BUZZER_PIN, 2800, 200);
            }
            if (isCancelTriggered()) {
                playCancelSound();
                Serial.println(">>> [DA TAT BAO DONG] Quay ve trang thai giam sat.\n");
                resetToIdle();
                delay(600);
            }
            break;
    }

    reportStateChange();
    sendTelemetry(now);

    delay(20); // ~50Hz
}
