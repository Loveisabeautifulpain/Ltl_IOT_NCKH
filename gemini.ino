#include <Wire.h>
#include <math.h>

// ================= 1. KHAI BAO PHAN CUNG (STICK 'EM ESP32) =================
#define I2C_SDA           21
#define I2C_SCL           22
#define MPU_ADDR          0x68
#define BUZZER_PIN        25    // Coi Buzzer tich hop
#define TOUCH_CANCEL_PIN  4     // Cong 2 - chan Vang (S) cam ung cham

// ================= 2. CAC NGUONG THUAT TOAN =================
// Nguong cam ung cham tay (khop voi ban test GPIO 4)
#define TOUCH_RATIO       0.80  // Tut xuong duoi 80% muc nen -> Phat hien cham

// Nhanh 1: Te nga manh (Roi tu do / Va cham)
#define FREEFALL_G        0.55  // Gia toc roi tu do (< 0.55g)
#define IMPACT_G          2.05  // Gia toc va cham manh (> 2.05g)

// Nhanh 2: Nga quy nhanh (Xoay nhanh + Guc nguoi)
#define COLLAPSE_GYRO     110.0 // Van toc goc xoay nhanh (> 110 do/s)
#define COLLAPSE_TILT     55.0  // Goc nghieng so voi tu the dung (> 55 do)

// Nhanh 3: Nga quy tu tu (Khuyu goi/ngat xiu cham, khong va cham manh)
#define SLOW_TILT_DEG     65.0  // Guc han > 65 do
#define SLOW_TILT_MS      1200  // Duy tri guc > 65 do lien tuc 1.2 giay

// Giai doan xac nhan sau nga (Post-Fall Check)
#define CHECK_TIME_MS     2500  // Theo doi 2.5 giay sau khi nghi ngo
#define SETTLE_TIME_MS    500   // Bo qua 0.5s dau de het rung chan
#define STILL_GYRO_MAX    25.0  // Nguong bat dong (< 25 do/s)
#define STANDUP_TILT      40.0  // Neu goc < 40 do -> Da dung day

// Giai doan dem nguoc canh bao
#define COUNTDOWN_MS      10000 // Dem nguoc 10 giay de cham tay huy

// ================= 3. BIEN HE THONG =================
enum SystemState {
    STATE_IDLE,            // 1. Giam sat binh thuong
    STATE_POST_FALL_CHECK, // 2. Nghi ngo -> Theo doi 2.5s sau nga
    STATE_COUNTDOWN,       // 3. Dem nguoc 10s + Coi canh bao
    STATE_ALARM            // 4. Bao dong khan cap
};

SystemState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long freeFallTime = 0;
unsigned long slowTiltStartTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastBeepTime = 0;
bool alarmSent = false;

// Dem ty le bat dong trong luc Post-Fall Check
int totalCheckSamples = 0;
int stillCheckSamples = 0;

// Bien hieu chuan MPU6050 & Cam ung cham
float gxOffset = 0, gyOffset = 0, gzOffset = 0;
float refAx = 0, refAy = 0, refAz = 1.0;
float smoothAx = 0, smoothAy = 0, smoothAz = 1.0; // Loc muot goc nghieng
int touchBaseline = 70;

// ================= 4. HAM HO TRO =================
void readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax = (int16_t)(Wire.read() << 8 | Wire.read());
    ay = (int16_t)(Wire.read() << 8 | Wire.read());
    az = (int16_t)(Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read(); // Bo qua nhiet do
    gx = (int16_t)(Wire.read() << 8 | Wire.read());
    gy = (int16_t)(Wire.read() << 8 | Wire.read());
    gz = (int16_t)(Wire.read() << 8 | Wire.read());
}

// Phat am thanh xac nhan da huy (2 tieng bip tram nhe)
void playCancelSound() {
    noTone(BUZZER_PIN);
    tone(BUZZER_PIN, 1200, 100); delay(130);
    tone(BUZZER_PIN, 900, 150);  delay(180);
    noTone(BUZZER_PIN);
}

// Kiem tra cham tay vao GPIO 4 (Cong 2) hoac go 'c' tren Serial
bool isCancelTriggered() {
    int val = touchRead(TOUCH_CANCEL_PIN);
    int threshold = (int)(touchBaseline * TOUCH_RATIO);

    if (val > 0 && val < threshold) {
        delay(60); // Loc nhieu thoang qua 60ms
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

// ================= 5. KHOI TAO & HIEU CHUAN =================
void setup() {
    Serial.begin(115200);
    pinMode(BUZZER_PIN, OUTPUT);
    // KHONG goi pinMode cho TOUCH_CANCEL_PIN (GPIO 4) khi dung touchRead()

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    delay(500);
    Serial.println("\n=======================================================");
    Serial.println("DANG HIEU CHUAN... GIU MACH O TU THE DUNG & KHONG CHAM CONG 2");
    tone(BUZZER_PIN, 1500, 150);

    // 1. Hieu chuan MPU6050 (200 mau)
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

    // 2. Do muc nen cam ung cham GPIO 4 (30 mau)
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
    Serial.println(">>> SAN SANG GIAM SAT! (Go 't' de test nhanh dem nguoc, 'c' de huy)");
    Serial.println("=======================================================\n");
}

// ================= 6. VONG LAP MAY TRANG THAI (50Hz) =================
void loop() {
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    readRawMPU(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);

    float ax = rawAx / 16384.0;
    float ay = rawAy / 16384.0;
    float az = rawAz / 16384.0;

    float gx = (rawGx - gxOffset) / 131.0;
    float gy = (rawGy - gyOffset) / 131.0;
    float gz = (rawGz - gzOffset) / 131.0;

    // Tinh gia toc & van toc goc tuc thoi
    float totalG = sqrt(ax * ax + ay * ay + az * az);
    float totalGyro = sqrt(gx * gx + gy * gy + gz * gz);

    // Loc muot EMA cho gia toc de tinh goc nghieng on dinh (it bi nhieu rung)
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

    unsigned long now = millis();

    switch (currentState) {
        // ---------------------------------------------------------
        case STATE_IDLE:
            // Cho phep go 't' tren Serial de test nhanh che do dem nguoc + cham tat
            if (Serial.available() > 0) {
                char cmd = Serial.read();
                if (cmd == 't' || cmd == 'T') {
                    currentState = STATE_COUNTDOWN;
                    stateStartTime = now;
                    lastBeepTime = 0;
                    Serial.println("\n>>> [TEST NHANH] Chuyen sang DEM NGUOC 10s! Hay cham tay vao GPIO 4...");
                    break;
                }
            }

            if (now - lastPrintTime >= 300) {
                lastPrintTime = now;
                Serial.print("[IDLE] G: "); Serial.print(totalG, 2);
                Serial.print("g | Gyro: "); Serial.print(totalGyro, 1);
                Serial.print(" do/s | Goc lech: "); Serial.print(tiltAngle, 1);
                Serial.println(" do");
            }

            // Ghi nhan roi tu do
            if (totalG < FREEFALL_G) {
                freeFallTime = now;
            }

            // Theo doi nhanh 3: Nga quy tu tu (nghieng > 65 do lien tuc)
            if (tiltAngle > SLOW_TILT_DEG) {
                if (slowTiltStartTime == 0) slowTiltStartTime = now;
            } else {
                slowTiltStartTime = 0;
            }

            // Kiem tra 3 nhanh kich hoat
            if (totalG > IMPACT_G) {
                currentState = STATE_POST_FALL_CHECK;
                stateStartTime = now;
                totalCheckSamples = 0; stillCheckSamples = 0;
                Serial.println("\n>>> [NGHI NGO - NHANH 1] Phat hien VA CHAM MANH! Dang kiem tra 2.5s...");
                if (now - freeFallTime < 600) Serial.println("    (Co dau hieu roi tu do truoc va cham)");
            }
            else if (totalGyro > COLLAPSE_GYRO && tiltAngle > COLLAPSE_TILT) {
                currentState = STATE_POST_FALL_CHECK;
                stateStartTime = now;
                totalCheckSamples = 0; stillCheckSamples = 0;
                Serial.println("\n>>> [NGHI NGO - NHANH 2] Phat hien DO NGUOI NHANH! Dang kiem tra 2.5s...");
            }
            else if (slowTiltStartTime > 0 && (now - slowTiltStartTime >= SLOW_TILT_MS)) {
                currentState = STATE_POST_FALL_CHECK;
                stateStartTime = now;
                slowTiltStartTime = 0;
                totalCheckSamples = 0; stillCheckSamples = 0;
                Serial.println("\n>>> [NGHI NGO - NHANH 3] Phat hien GUC NGUOI TU TU! Dang kiem tra 2.5s...");
            }
            break;

            // ---------------------------------------------------------
        case STATE_POST_FALL_CHECK:
            if (now - stateStartTime > SETTLE_TIME_MS) {
                // Neu nguoi dung da dung day -> Huy nghi ngo ngay
                if (tiltAngle < STANDUP_TILT) {
                    Serial.println(">>> [HUY NGHI NGO] Da dung day / tu the binh thuong.\n");
                    resetToIdle();
                    break;
                }

                // Dem ty le mau bat dong
                totalCheckSamples++;
                if (totalGyro < STILL_GYRO_MAX) {
                    stillCheckSamples++;
                }
            }

            // Het 2.5s: Xac nhan neu van guc VA bat dong >= 75% thoi gian
            if (now - stateStartTime >= CHECK_TIME_MS) {
                float stillRatio = (totalCheckSamples > 0) ? ((float)stillCheckSamples / totalCheckSamples) : 0;
                if (tiltAngle >= COLLAPSE_TILT && stillRatio >= 0.75) {
                    currentState = STATE_COUNTDOWN;
                    stateStartTime = now;
                    lastBeepTime = 0;
                    Serial.println("\n!!! [XAC NHAN NGA QUY] Bat dau dem nguoc 10s! CHAM TAY CONG 2 (GPIO 4) DE HUY !!!");
                } else {
                    Serial.println(">>> [HUY NGHI NGO] Van con cu dong, khong phai bat tinh.\n");
                    resetToIdle();
                }
            }
            break;

            // ---------------------------------------------------------
        case STATE_COUNTDOWN:
            // 1. Kiem tra cham tay GPIO 4 lien tuc
            if (isCancelTriggered()) {
                playCancelSound();
                Serial.println(">>> [DA HUY THANH CONG] Nguoi dung xac nhan an toan. Quay ve giam sat.\n");
                resetToIdle();
                delay(600); // Cho nguoi dung nha tay ra
                break;
            }

            // 2. Keu bip moi 1 giay + in dem nguoc
            if (now - lastBeepTime >= 1000) {
                lastBeepTime = now;
                tone(BUZZER_PIN, 2000, 180);
                int secLeft = (COUNTDOWN_MS - (now - stateStartTime)) / 1000 + 1;
                if (secLeft < 0) secLeft = 0;
                Serial.print("... Dem nguoc: "); Serial.print(secLeft);
                Serial.println(" giay (Cham GPIO 4 de tat)");
            }

            // 3. Het 10 giay khong cham -> Chuyen sang BAO DONG
            if (now - stateStartTime >= COUNTDOWN_MS) {
                currentState = STATE_ALARM;
                alarmSent = false;
                lastBeepTime = 0;
            }
            break;

            // ---------------------------------------------------------
        case STATE_ALARM:
            if (!alarmSent) {
                Serial.println("\n=======================================================");
                Serial.println("!!! CANH BAO KHAN CAP: PHAT HIEN NGA QUY !!!");
                Serial.println("-> (Giai doan sau se gui canh bao Wi-Fi / Telegram tai day)");
                Serial.println("-> CHAM TAY CONG 2 (GPIO 4) de tat coi bao dong.");
                Serial.println("=======================================================\n");
                alarmSent = true;
            }

            // Coi hu lien tuc
            if (now - lastBeepTime >= 350) {
                lastBeepTime = now;
                tone(BUZZER_PIN, 2800, 200);
            }

            // Cham tay GPIO 4 de tat bao dong
            if (isCancelTriggered()) {
                playCancelSound();
                Serial.println(">>> [DA TAT BAO DONG] Quay ve trang thai giam sat.\n");
                resetToIdle();
                delay(600);
            }
            break;
    }

    delay(20); // Tan so lay mau ~50Hz
}
