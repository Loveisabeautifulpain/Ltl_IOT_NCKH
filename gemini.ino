#include <Wire.h>
#include <math.h>

// ================= 1. KHAI BAO PHAN CUNG =================
#define I2C_SDA         21
#define I2C_SCL         22
#define MPU_ADDR        0x68
#define BUZZER_PIN      25    // Sua lai neu chan Buzzer cua ban khac 25
#define BTN_CANCEL_PIN  4     // Cong 2 tren mach Stick 'Em

// ================= 2. CAC NGUONG THUAT TOAN (DE TINH CHINH) =================
// Nhanh 1: Te nga manh (Roi tu do / Va cham)
#define FREEFALL_G      0.55  // Gia toc roi tu do (< 0.55g)
#define IMPACT_G        2.10  // Gia toc va cham manh (> 2.1g)

// Nhanh 2: Nga quy / Khuyu goi (Xoay nhanh + Guc nguoi)
#define COLLAPSE_GYRO   130.0 // Van toc goc xoay nhanh (> 130 do/s)
#define COLLAPSE_TILT   55.0  // Goc nghieng so voi luc dung thang (> 55 do)

// Giai doan xac nhan sau nga (Post-Fall Check)
#define CHECK_TIME_MS   2500  // Theo doi 2.5 giay sau khi nghi ngo
#define STILL_GYRO_MAX  25.0  // Nguong bat dong: toc do xoay < 25 do/s
#define STANDUP_TILT    40.0  // Neu goc nghieng < 40 do -> Da dung day (Huy nghi ngo)

// Giai doan dem nguoc canh bao
#define COUNTDOWN_MS    10000 // Dem nguoc 10 giay de bam nut huy

// ================= 3. BIEN HE THONG =================
enum SystemState {
  STATE_IDLE,            // 1. Giam sat binh thuong
  STATE_POST_FALL_CHECK, // 2. Nghi ngo -> Theo doi 2.5s sau nga
  STATE_COUNTDOWN,       // 3. Dem nguoc 10s + Keu coi canh bao
  STATE_ALARM            // 4. Bao dong khan cap
};

SystemState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
unsigned long freeFallTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastBeepTime = 0;
bool alarmSent = false;

// Bien hieu chuan & tu the dung chuan
float gxOffset = 0, gyOffset = 0, gzOffset = 0;
float refAx = 0, refAy = 0, refAz = 1.0; // Vector trong luc luc dung thang

// ================= 4. HAM DOC MPU6050 =================
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

// Kiem tra nut huy (Cong 2 - GPIO 4) hoac lenh 'c' tu Serial Monitor
bool isCancelPressed() {
  if (digitalRead(BTN_CANCEL_PIN) == LOW) return true;
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') return true;
  }
  return false;
}

// ================= 5. KHOI TAO & HIEU CHUAN =================
void setup() {
  Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BTN_CANCEL_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  // Danh thuc MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(500);
  Serial.println("\n==================================================");
  Serial.println("DANG HIEU CHUAN... HAY GIU YEN O TU THE DUNG THANG");
  tone(BUZZER_PIN, 1500, 150); // Bip 1 tieng bao bat dau hieu chuan

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

  // Chuan hoa vector trong luc tham chieu (tu the dung thang = 0 do)
  float avgAx = (sumAx / 200.0) / 16384.0;
  float avgAy = (sumAy / 200.0) / 16384.0;
  float avgAz = (sumAz / 200.0) / 16384.0;
  float refMag = sqrt(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);
  if (refMag > 0.1) {
    refAx = avgAx / refMag;
    refAy = avgAy / refMag;
    refAz = avgAz / refMag;
  }

  // Bip 2 tieng bao san sang giam sat
  tone(BUZZER_PIN, 2000, 120); delay(180);
  tone(BUZZER_PIN, 2500, 120);
  Serial.println("SAN SANG GIAM SAT! (Go 'c' hoac bam nut Cong 2 de huy canh bao)");
  Serial.println("==================================================");
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

  // Tinh 3 chi so cot loi
  float totalG = sqrt(ax * ax + ay * ay + az * az);
  float totalGyro = sqrt(gx * gx + gy * gy + gz * gz);

  // Tinh goc nghieng so voi tu the dung ban dau (0 -> 180 do)
  float tiltAngle = 0;
  if (totalG > 0.1) {
    float dot = (ax * refAx + ay * refAy + az * refAz) / totalG;
    dot = constrain(dot, -1.0f, 1.0f);
    tiltAngle = acos(dot) * 180.0 / PI;
  }

  unsigned long now = millis();

  switch (currentState) {
    // ---------------------------------------------------------
    case STATE_IDLE:
      // In du lieu moi 250ms de theo doi / lay so lieu thuc nghiem
      if (now - lastPrintTime >= 250) {
        lastPrintTime = now;
        Serial.print("[IDLE] G: "); Serial.print(totalG, 2);
        Serial.print(" g | Gyro: "); Serial.print(totalGyro, 1);
        Serial.print(" do/s | Goc lech: "); Serial.print(tiltAngle, 1);
        Serial.println(" do");
      }

      // Ghi nhan roi tu do
      if (totalG < FREEFALL_G) {
        freeFallTime = now;
      }

      // Kiem tra 2 nhanh kich hoat: (1) Va cham manh hoac (2) Nga quy xoay nhanh + guc
      if (totalG > IMPACT_G || (totalGyro > COLLAPSE_GYRO && tiltAngle > COLLAPSE_TILT)) {
        currentState = STATE_POST_FALL_CHECK;
        stateStartTime = now;
        Serial.println("\n>>> [NGHI NGO] Phat hien chan dong / guc nguoi! Dang kiem tra 2.5 giay...");
        if (now - freeFallTime < 600) {
          Serial.println("    (Co dau hieu roi tu do truoc va cham)");
        }
      }
      break;

    // ---------------------------------------------------------
    case STATE_POST_FALL_CHECK:
      // Cho 0.5 giay dau de het rung chan, sau do kiem tra tu the & do bat dong
      if (now - stateStartTime > 500) {
        // Neu nguoi dung da dung day lai -> Bao dong gia, quay ve IDLE
        if (tiltAngle < STANDUP_TILT) {
          Serial.println(">>> [HUY NGHI NGO] Nguoi dung da dung day / cu dong binh thuong.\n");
          currentState = STATE_IDLE;
          break;
        }
      }

      // Het 2.5 giay: Kiem tra xem van dang nam/guc VA bat dong khong
      if (now - stateStartTime >= CHECK_TIME_MS) {
        if (tiltAngle >= COLLAPSE_TILT && totalGyro < STILL_GYRO_MAX) {
          currentState = STATE_COUNTDOWN;
          stateStartTime = now;
          lastBeepTime = 0;
          Serial.println("\n!!! [XAC NHAN NGA QUY] Bat dau dem nguoc 10 giay! Bam nut Cong 2 de huy !!!");
        } else {
          Serial.println(">>> [HUY NGHI NGO] Khong thoa man dieu kien bat dong sau nga.\n");
          currentState = STATE_IDLE;
        }
      }
      break;

    // ---------------------------------------------------------
    case STATE_COUNTDOWN:
      // Kiem tra nut huy lien tuc
      if (isCancelPressed()) {
        noTone(BUZZER_PIN);
        Serial.println(">>> [DA HUY] Nguoi dung da xac nhan an toan. Quay ve giam sat.\n");
        currentState = STATE_IDLE;
        delay(500); // Chong doi nut
        break;
      }

      // Keu bip moi 1 giay + in dem nguoc
      if (now - lastBeepTime >= 1000) {
        lastBeepTime = now;
        tone(BUZZER_PIN, 2000, 200);
        int secLeft = (COUNTDOWN_MS - (now - stateStartTime)) / 1000 + 1;
        if (secLeft < 0) secLeft = 0;
        Serial.print("... Dem nguoc: "); Serial.print(secLeft); Serial.println(" giay");
      }

      // Het 10 giay khong huy -> Chuyen sang BAO DONG
      if (now - stateStartTime >= COUNTDOWN_MS) {
        currentState = STATE_ALARM;
        alarmSent = false;
        lastBeepTime = 0;
      }
      break;

    // ---------------------------------------------------------
    case STATE_ALARM:
      if (!alarmSent) {
        Serial.println("\n==================================================");
        Serial.println("!!! CANH BAO KHAN CAP: PHAT HIEN NGA QUY !!!");
        Serial.println("-> (Buoc tiep theo se gui tin nhan Wi-Fi / Telegram tai day)");
        Serial.println("-> Bam nut Cong 2 (hoac go 'c') de tat coi.");
        Serial.println("==================================================\n");
        alarmSent = true;
      }

      // Coi hu lien tuc (bip nhanh 250ms)
      if (now - lastBeepTime >= 350) {
        lastBeepTime = now;
        tone(BUZZER_PIN, 2800, 200);
      }

      // Cho phep tat bao dong khi co nguoi ho tro
      if (isCancelPressed()) {
        noTone(BUZZER_PIN);
        Serial.println(">>> [DA TAT BAO DONG] Quay ve trang thai giam sat.\n");
        currentState = STATE_IDLE;
        delay(500);
      }
      break;
  }

  delay(20); // Tan so lay mau ~50Hz
}
