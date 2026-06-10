/*
 * ================================================================
 *  ROCKET PARACHUTE EJECTION SYSTEM  v6.0  — ROCKET (TX) NODE
 *  Hardware confirmed working:
 *    BMP280  at I2C 0x76  (chip_id 0x58)
 *    MPU-6500 at I2C 0x68 (WHO_AM_I 0x70)
 *    LoRa SX1278, 2x Servo, ESP32 DevKit v1 38-pin
 *
 *  POWER WIRING (parallel — all 3.3V devices share one rail):
 *
 *    ESP32 3V3 pin ──┬──► BMP280  VCC
 *                   ├──► MPU-6500 VCC
 *                   └──► LoRa     VCC
 *
 *    ESP32 GND    ──┬──► BMP280  GND
 *                   ├──► MPU-6500 GND
 *                   ├──► LoRa     GND
 *                   └──► Servo    GND  (signal ground shared)
 *
 *    External 5V  ──┬──► Servo1  VCC  (never power servos from 3V3)
 *                   └──► Servo2  VCC
 *
 *  I2C (both sensors in parallel on same two wires):
 *    GPIO21 SDA ──┬──► BMP280  SDA
 *                 └──► MPU-6500 SDA
 *    GPIO22 SCL ──┬──► BMP280  SCL
 *                 └──► MPU-6500 SCL
 *
 *    BMP280  SDO → GND   (locks address to 0x76)
 *    BMP280  CSB → 3V3   (enables I2C mode, not SPI)
 *    MPU-6500 AD0 → GND  (locks address to 0x68)
 *
 *  PULLUP RESISTORS:
 *    Keep pullups on ONE module only. If both modules have 4.7k
 *    pullups, desolder or lift the resistors on one of them.
 *    Two 4.7k in parallel = 2.35k, causes bus instability.
 *
 *  LoRa SPI (separate bus, no conflict with I2C):
 *    NSS=GPIO5  SCK=GPIO18  MISO=GPIO19  MOSI=GPIO23
 *    RST=GPIO14  DIO0=GPIO2
 *
 *  Servo signals:
 *    Servo1 → GPIO25
 *    Servo2 → GPIO26
 *
 *  Libraries required:
 *    LoRa (Sandeep Mistry)
 *    Adafruit BMP280
 *    ESP32Servo
 *    ArduinoJson
 * ================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <math.h>
#include <algorithm>

// ── Pin definitions ───────────────────────────────────────────
#define PIN_SDA       21
#define PIN_SCL       22
#define PIN_SERVO1    25
#define PIN_SERVO2    26
#define PIN_LORA_SS    5
#define PIN_LORA_RST  14
#define PIN_LORA_DIO0  2

// ── I2C ──────────────────────────────────────────────────────
// 80 kHz: reliable for BMP280 clones + MPU-6500 on the same bus.
// Wire.setTimeout() is the correct ESP32 Arduino core method.
// Wire.setWireTimeout() does not exist and will not compile.
#define I2C_HZ        80000
#define I2C_TIMEOUT   30     // ms — prevents infinite hang on stuck bus

// ── Confirmed I2C addresses (from hardware scan) ─────────────
#define BMP_ADDR      0x76
#define MPU_ADDR      0x68

// ── LoRa ─────────────────────────────────────────────────────
#define LORA_FREQ     433E6
#define LORA_SF       7
#define LORA_BW       125E3
#define LORA_CR       5
#define LORA_PWR      17
#define LORA_SYNC     0xB4
#define LORA_RX_WIN   80    // ms to listen for commands after TX

// ── MPU-6500 register map ─────────────────────────────────────
#define REG_SMPLRT    0x19
#define REG_CONFIG    0x1A
#define REG_GYRO_CFG  0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_ACCEL_CFG2 0x1D
#define REG_ACCEL_OUT 0x3B  // first of 14 bytes: AX AY AZ TEMP GX GY GZ
#define REG_PWR1      0x6B
#define REG_PWR2      0x6C
#define REG_WHOAMI    0x75

// ── Scaling ──────────────────────────────────────────────────
#define ACCEL_LSB     16384.0f  // ±2g range: 16384 LSB/g
#define GYRO_LSB      65.5f     // ±500°/s range

// ── Timing ───────────────────────────────────────────────────
#define T_SENSOR_MS   40    // sensor read interval — 25 Hz
#define T_LORA_MS     150   // telemetry TX interval — ~6.5 Hz
#define T_WDG_MS      5000  // watchdog check interval

// ── Apogee detection tuning ───────────────────────────────────
// Require 20 consecutive falling altitude samples to confirm apogee.
// At 25 Hz that is 800ms of sustained descent — filters out turbulence.
#define FALL_WINDOW   20
#define FALL_RESET    5     // non-falling samples to break a partial streak
#define FF_WINDOW     5     // MPU free-fall confirm samples
#define MIN_ALT       8.0f  // metres AGL — ignore apogee below this (pad safety)
#define FALL_HYSTERESIS 0.15f  // metres — must drop this much to count as falling

// ── Flight log ───────────────────────────────────────────────
#define LOG_MAX 400
struct LogEntry {
  uint32_t ms;
  float alt, net, vert, ax, ay, az;
};
LogEntry log_[LOG_MAX];
int logN = 0;

// ── EEPROM layout ────────────────────────────────────────────
#define EE_SIZE       96
#define EE_MAGIC_ADDR  0
#define EE_MAGIC_VAL  0xBE   // changed from 0xAB so old corrupt data is ignored
#define EE_SV1         1     // int16
#define EE_SV2         3     // int16
#define EE_AUTO        5     // byte
#define EE_APOGEE      6     // int16
#define EE_FUSION      8     // byte
#define EE_FF          9     // int16
#define EE_ACTIVE_SV  11     // byte
#define EE_SEALVL     12     // float (4 bytes)

// ── Defaults ─────────────────────────────────────────────────
#define DEF_SV1       90
#define DEF_SV2       90
#define DEF_AUTO      true
#define DEF_APOGEE    200
#define DEF_FUSION    1
#define DEF_FF        300
#define DEF_ACTIVE    1
#define DEF_SEALVL    1013.25f

// ═══════════════════════════════════════════════════════════════
//  GLOBAL STATE
// ═══════════════════════════════════════════════════════════════

Adafruit_BMP280 bmp;
Servo sv1, sv2;

// sensor health
bool bmpOK  = false;
bool mpuOK  = false;

// MPU calibration offsets (applied to raw reads)
float aOX=0,aOY=0,aOZ=0;
float gOX=0,gOY=0,gOZ=0;

// calibration lock — blocks mpuRead() during cal collection
volatile bool calLock = false;

// MPU last values
float accelX=0,accelY=0,accelZ=1;
float gyroX=0,gyroY=0,gyroZ=0;
float mpuTempC=0,netG=1,vertG=0,tiltDeg=0;
bool  fflag=false;

// BMP last values
float bmpAltRaw=0,bmpPres=1013.25f,bmpTempC=25;

// BMP watchdog counters
float bmpLastAlt = -9999;
int   bmpFrozen  = 0;
int   bmpBad     = 0;
#define BMP_BAD_MAX    5
#define BMP_FROZEN_MAX 8

// Altitude tracking
float seaLvl   = DEF_SEALVL;
float baseAlt  = 0;
float curAlt   = 0;
float maxAlt   = 0;
float prevAlt  = 0;

// Apogee state
int   fallCnt    = 0;
int   nofallCnt  = 0;
bool  apoDetect  = false;
bool  deployed   = false;

// MPU apogee assist
int   ffCnt      = 0;
bool  mpuApoConf = false;

// System state
bool  armed      = false;
bool  recOn      = false;
unsigned long recStart = 0;

// Settings (loaded from EEPROM)
int   sv1Ang    = DEF_SV1;
int   sv2Ang    = DEF_SV2;
bool  autoMode  = DEF_AUTO;
int   apoMargin = DEF_APOGEE;
int   activeSv  = DEF_ACTIVE;
int   fusionMode= DEF_FUSION;
int   ffThresh  = DEF_FF;

// Servo test
bool  svTestOn   = false;
int   svTestNum  = 1;
unsigned long svTestEnd = 0;

// Non-blocking tare state machine
enum TarePhase { TP_IDLE, TP_RESET, TP_COLLECT, TP_SETTLE };
TarePhase tphase = TP_IDLE;
unsigned long tSetTimer = 0;
long  tAX=0,tAY=0,tAZ=0,tGX=0,tGY=0,tGZ=0;
int   tN = 0;
#define TARE_N 150

// Loop timers
unsigned long tSensor=0, tLora=0, tWdg=0;

// ═══════════════════════════════════════════════════════════════
//  I2C HELPERS
// ═══════════════════════════════════════════════════════════════

// Restart Wire with correct settings. Called after bus recovery.
void wireStart() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeout(I2C_TIMEOUT);
}

// Release a device holding SDA low by bit-banging 9 clock pulses.
// This is the correct recovery for a stuck I2C bus.
void busRecover() {
  Wire.end();
  delay(10);

  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(5);

  // Clock until SDA is released or 9 cycles done
  for (int i = 0; i < 9; i++) {
    if (digitalRead(PIN_SDA)) break;  // released — done
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(60);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(60);
  }

  // Issue STOP condition
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(10);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(10);

  delay(20);
  wireStart();
}

// ── MPU raw register write ────────────────────────────────────
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// ── MPU single-byte read ──────────────────────────────────────
uint8_t mpuRead1(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// ── MPU burst read (n bytes starting at reg) ─────────────────
int mpuReadBurst(uint8_t reg, uint8_t* buf, uint8_t n) {
  memset(buf, 0, n);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((uint8_t)MPU_ADDR, n);
  int got = 0;
  while (Wire.available() && got < n) buf[got++] = Wire.read();
  return got;
}

// ═══════════════════════════════════════════════════════════════
//  MPU-6500 INIT
// ═══════════════════════════════════════════════════════════════
bool mpuInit() {
  // Power-reset the MPU
  if (!mpuWrite(REG_PWR1, 0x80)) {
    Serial.println("[MPU] Reset write failed — check wiring");
    return false;
  }
  delay(250);  // datasheet: wait 100ms after reset, 250ms is safe

  // Check WHO_AM_I (MPU-6500 returns 0x70; some clones return 0x68/0x71)
  uint8_t who = 0xFF;
  for (int i = 0; i < 5; i++) {
    who = mpuRead1(REG_WHOAMI);
    Serial.printf("[MPU] WHO_AM_I attempt %d = 0x%02X\n", i+1, who);
    if (who == 0x70 || who == 0x68 || who == 0x71) break;
    delay(60);
  }
  if (who != 0x70 && who != 0x68 && who != 0x71) {
    Serial.println("[MPU] Not found — wrong address or wiring");
    return false;
  }

  // Wake from sleep, use PLL clock
  mpuWrite(REG_PWR1, 0x01);  delay(50);
  mpuWrite(REG_PWR2, 0x00);  // enable all axes

  // Sample rate divider: ODR = 1000/(1+div) Hz
  // div=24 → 40Hz, which is fine at our 25Hz read rate
  mpuWrite(REG_SMPLRT, 24);

  // DLPF at 20Hz — attenuates motor vibration frequencies
  mpuWrite(REG_CONFIG, 0x04);

  // Accel: ±2g range, DLPF 20Hz
  mpuWrite(REG_ACCEL_CFG,  0x00);
  mpuWrite(REG_ACCEL_CFG2, 0x04);

  // Gyro: ±500°/s
  mpuWrite(REG_GYRO_CFG, 0x08);

  delay(80);

  // Verify DLPF register wrote correctly
  uint8_t cfg = mpuRead1(REG_CONFIG);
  if (cfg != 0x04) {
    Serial.printf("[MPU] CONFIG verify failed (read 0x%02X, want 0x04)\n", cfg);
    return false;
  }

  Serial.println("[MPU] Init OK");
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  MPU CALIBRATION — blocking, called once at startup
//  Collects N samples, applies trimmed mean (drops outer 10%),
//  stores offsets. Rocket must be stationary and upright.
// ═══════════════════════════════════════════════════════════════
void mpuCalibrate(int N = 200) {
  if (!mpuOK) return;

  // Reduce N if heap is tight
  size_t needed = (size_t)N * 6 * sizeof(int16_t);
  if (ESP.getFreeHeap() < needed + 8192) {
    N = 100;
    Serial.printf("[MPU] Low heap — reducing calibration to %d samples\n", N);
  }

  calLock = true;
  Serial.printf("[MPU] Calibrating (%d samples)...\n", N);

  int16_t *rAX = new int16_t[N](),
          *rAY = new int16_t[N](),
          *rAZ = new int16_t[N](),
          *rGX = new int16_t[N](),
          *rGY = new int16_t[N](),
          *rGZ = new int16_t[N]();

  uint8_t buf[14];
  int got = 0;
  for (int i = 0; i < N; i++) {
    if (mpuReadBurst(REG_ACCEL_OUT, buf, 14) == 14) {
      rAX[got] = (int16_t)((buf[0]<<8)|buf[1]);
      rAY[got] = (int16_t)((buf[2]<<8)|buf[3]);
      rAZ[got] = (int16_t)((buf[4]<<8)|buf[5]);
      rGX[got] = (int16_t)((buf[8]<<8)|buf[9]);
      rGY[got] = (int16_t)((buf[10]<<8)|buf[11]);
      rGZ[got] = (int16_t)((buf[12]<<8)|buf[13]);
      got++;
    }
    delay(7);
    if (i % 50 == 49) Serial.print(".");
  }
  Serial.println();

  // Trimmed mean: sort, drop outer 10%, average middle 80%
  auto tmean = [](int16_t* a, int n) -> float {
    std::sort(a, a + n);
    int lo = n / 10, hi = n - n / 10;
    long s = 0;
    for (int i = lo; i < hi; i++) s += a[i];
    return (hi > lo) ? (float)s / (hi - lo) : 0.0f;
  };

  aOX = tmean(rAX, got);
  aOY = tmean(rAY, got);
  aOZ = tmean(rAZ, got) - ACCEL_LSB;  // subtract 1g gravity from Z
  gOX = tmean(rGX, got);
  gOY = tmean(rGY, got);
  gOZ = tmean(rGZ, got);

  delete[] rAX; delete[] rAY; delete[] rAZ;
  delete[] rGX; delete[] rGY; delete[] rGZ;

  calLock = false;
  Serial.printf("[MPU] Cal done: AX=%.0f AY=%.0f AZ=%.0f | GX=%.1f GY=%.1f GZ=%.1f\n",
    aOX, aOY, aOZ, gOX, gOY, gOZ);
}

// ═══════════════════════════════════════════════════════════════
//  MPU READ — called every T_SENSOR_MS
// ═══════════════════════════════════════════════════════════════
void mpuRead() {
  if (!mpuOK || calLock) return;

  uint8_t buf[14];
  if (mpuReadBurst(REG_ACCEL_OUT, buf, 14) < 14) return;

  int16_t rAX = (int16_t)((buf[0]<<8)|buf[1]);
  int16_t rAY = (int16_t)((buf[2]<<8)|buf[3]);
  int16_t rAZ = (int16_t)((buf[4]<<8)|buf[5]);
  int16_t rT  = (int16_t)((buf[6]<<8)|buf[7]);
  int16_t rGX = (int16_t)((buf[8]<<8)|buf[9]);
  int16_t rGY = (int16_t)((buf[10]<<8)|buf[11]);
  int16_t rGZ = (int16_t)((buf[12]<<8)|buf[13]);

  accelX = (rAX - aOX) / ACCEL_LSB;
  accelY = (rAY - aOY) / ACCEL_LSB;
  accelZ = (rAZ - aOZ) / ACCEL_LSB;
  gyroX  = (rGX - gOX) / GYRO_LSB;
  gyroY  = (rGY - gOY) / GYRO_LSB;
  gyroZ  = (rGZ - gOZ) / GYRO_LSB;
  mpuTempC = rT / 340.0f + 36.53f;

  netG    = sqrtf(accelX*accelX + accelY*accelY + accelZ*accelZ);
  tiltDeg = degrees(acosf(constrain(accelZ / fmaxf(netG, 0.01f), -1.0f, 1.0f)));
  vertG   = accelZ - 1.0f;  // subtract 1g so 0 = level, negative = falling

  // Free-fall: net acceleration well below 1g
  fflag = (netG < (ffThresh / 1000.0f));

  // Accumulate free-fall counter only when armed and above pad
  if (armed && curAlt > MIN_ALT) {
    ffCnt = fflag ? ffCnt + 1 : 0;
    if (ffCnt >= FF_WINDOW) mpuApoConf = true;
  } else {
    ffCnt = 0;
  }
}

// ═══════════════════════════════════════════════════════════════
//  BMP280 INIT
// ═══════════════════════════════════════════════════════════════
bool bmpInit(bool softReset = true) {
  if (softReset) {
    // Reset register 0xE0, value 0xB6 — forces a full power-on reset
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(0xE0); Wire.write(0xB6);
    Wire.endTransmission();
    delay(150);
  }

  // Try confirmed address first
  bool found = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.printf("[BMP] Init attempt %d/5 at 0x%02X\n", attempt, BMP_ADDR);
    if (bmp.begin(BMP_ADDR)) { found = true; break; }
    delay(200);
  }

  if (!found) {
    // Fallback: try 0x77 in case SDO is not properly grounded
    Serial.println("[BMP] 0x76 failed — trying 0x77 (check SDO pin)");
    if (bmp.begin(0x77)) {
      found = true;
      Serial.println("[BMP] Found at 0x77 — SDO pin is not connected to GND");
    }
  }

  if (!found) {
    Serial.println("[BMP] Not found on bus — check VCC=3.3V, CSB=3.3V, SDA/SCL wiring");
    return false;
  }

  // Configure for best altitude accuracy
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X4,   // temperature oversampling
    Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling
    Adafruit_BMP280::FILTER_X4,     // IIR filter coefficient
    Adafruit_BMP280::STANDBY_MS_125 // standby between measurements
  );

  // FILTER_X4 needs ~5 measurement cycles to converge from reset.
  // Each cycle ≈ 125ms standby + ~44ms conversion = ~170ms.
  // We wait 1200ms and flush 6 readings to guarantee clean data.
  Serial.print("[BMP] Waiting for filter convergence");
  for (int i = 0; i < 6; i++) {
    delay(200);
    bmp.readPressure();  // flush
    Serial.print(".");
  }
  Serial.println(" done");

  // Sanity check: pressure must be realistic at any Earth altitude
  float p = bmp.readPressure() / 100.0f;
  if (p < 800.0f || p > 1100.0f) {
    Serial.printf("[BMP] Post-init sanity fail: P=%.1f hPa (expected 800-1100)\n", p);
    return false;
  }

  // Reset watchdog counters
  bmpFrozen = 0; bmpBad = 0; bmpLastAlt = -9999;

  Serial.printf("[BMP] Init OK — P=%.1f hPa\n", p);
  return true;
}

// ═══════════════════════════════════════════════════════════════
//  BMP280 READ — called every T_SENSOR_MS
// ═══════════════════════════════════════════════════════════════
void bmpRead() {
  if (!bmpOK) return;

  float alt = bmp.readAltitude(seaLvl);
  float p   = bmp.readPressure() / 100.0f;
  float t   = bmp.readTemperature();

  // Reject physically impossible readings
  if (p < 800.0f || p > 1100.0f || t < -50.0f || t > 100.0f) {
    bmpBad++;
    Serial.printf("[BMP] Bad reading #%d: p=%.1f t=%.1f\n", bmpBad, p, t);
    if (bmpBad >= BMP_BAD_MAX) {
      Serial.println("[BMP] Too many bad readings — reinitialising");
      busRecover();
      bmpOK = bmpInit(true);
      bmpBad = 0;
    }
    return;
  }
  bmpBad = 0;

  // Detect frozen output (output stuck at exact same value)
  if (fabsf(alt - bmpLastAlt) < 0.004f) {
    bmpFrozen++;
    if (bmpFrozen >= BMP_FROZEN_MAX) {
      Serial.println("[BMP] Output frozen — reinitialising");
      busRecover();
      bmpOK = bmpInit(true);
      bmpFrozen = 0;
    }
    return;
  }
  bmpFrozen = 0;
  bmpLastAlt = alt;

  bmpAltRaw = alt;
  bmpPres   = p;
  bmpTempC  = t;
  curAlt    = bmpAltRaw - baseAlt;
  if (curAlt > maxAlt) maxAlt = curAlt;
}

// ═══════════════════════════════════════════════════════════════
//  APOGEE DETECTION
// ═══════════════════════════════════════════════════════════════
void checkApogee() {
  if (!armed || !autoMode || deployed) return;
  if (curAlt < MIN_ALT) return;

  // Hysteresis: must drop by at least FALL_HYSTERESIS metres to count
  if (curAlt < prevAlt - FALL_HYSTERESIS) {
    fallCnt++;
    nofallCnt = 0;
  } else {
    nofallCnt++;
    // Only reset falling counter if we haven't nearly confirmed yet
    if (nofallCnt >= FALL_RESET && fallCnt < FALL_WINDOW) {
      fallCnt = 0;
      nofallCnt = 0;
    }
  }

  bool bmpApo  = (fallCnt  >= FALL_WINDOW);
  bool mpuApo  = mpuApoConf;
  bool shouldDeploy = false;
  const char* why = "";

  switch (fusionMode) {
    case 0:  // BMP only — simple, no IMU
      if (bmpApo) { shouldDeploy = true; why = "BMP_ONLY"; }
      break;

    case 1:  // Fusion — prefer both agree, fall back to BMP if MPU absent
      if (bmpApo && mpuApo)                         { shouldDeploy=true; why="FUSION"; }
      else if (bmpApo && ffCnt >= FF_WINDOW/2)      { shouldDeploy=true; why="FUSION_PARTIAL"; }
      else if (bmpApo && !mpuOK)                    { shouldDeploy=true; why="BMP_FALLBACK"; }
      break;

    case 2:  // MPU primary
      if (mpuApo)                                   { shouldDeploy=true; why="MPU_PRIMARY"; }
      else if (bmpApo && !mpuOK)                    { shouldDeploy=true; why="BMP_FALLBACK"; }
      break;
  }

  if (shouldDeploy && !apoDetect) {
    apoDetect = true;
    deployNow(why);
  }
}

// ═══════════════════════════════════════════════════════════════
//  DEPLOY
// ═══════════════════════════════════════════════════════════════
void deployNow(const char* reason) {
  if (deployed) return;
  deployed = true;
  int ang = (activeSv == 1) ? sv1Ang : sv2Ang;
  if (activeSv == 1) sv1.write(ang); else sv2.write(ang);
  Serial.printf("[DEPLOY] %s | servo=%d angle=%d\n", reason, activeSv, ang);
}

// ═══════════════════════════════════════════════════════════════
//  SENSOR SAMPLE — main 25 Hz tick
// ═══════════════════════════════════════════════════════════════
void sensorTick() {
  bmpRead();
  mpuRead();
  if (recOn && logN < LOG_MAX) {
    log_[logN++] = {
      (uint32_t)(millis() - recStart),
      curAlt, netG, vertG, accelX, accelY, accelZ
    };
  }
  checkApogee();
  prevAlt = curAlt;
}

// ═══════════════════════════════════════════════════════════════
//  WATCHDOG — runs every T_WDG_MS, recovers dead sensors
// ═══════════════════════════════════════════════════════════════
void watchdog() {
  // BMP watchdog
  if (!bmpOK) {
    Serial.println("[WDG] BMP offline — recovery");
    busRecover();
    bmpOK = bmpInit(true);
    if (bmpOK && baseAlt == 0) {
      float s = 0;
      for (int i = 0; i < 5; i++) { s += bmp.readAltitude(seaLvl); delay(150); }
      baseAlt = s / 5.0f;
    }
  }

  // MPU watchdog
  if (!mpuOK) {
    Serial.println("[WDG] MPU offline — recovery");
    busRecover();
    mpuOK = mpuInit();
    if (mpuOK) { delay(100); mpuCalibrate(100); }
    return;
  }

  // Spot-check MPU WHO_AM_I to catch silent failures
  uint8_t who = mpuRead1(REG_WHOAMI);
  if (who != 0x70 && who != 0x68 && who != 0x71) {
    Serial.printf("[WDG] MPU WHO_AM_I=0x%02X unexpected — reinit\n", who);
    mpuOK = false;
    busRecover();
    mpuOK = mpuInit();
    if (mpuOK) { delay(100); mpuCalibrate(100); }
  }
}

// ═══════════════════════════════════════════════════════════════
//  NON-BLOCKING TARE STATE MACHINE
//  Runs one step per loop() call. Full sequence takes ~6 seconds
//  at 25 Hz (150 samples × 40ms). Does not block the LoRa TX loop.
// ═══════════════════════════════════════════════════════════════
void tareStep() {
  uint8_t buf[14];

  switch (tphase) {
    case TP_IDLE:
      return;

    case TP_RESET:
      // Reset all flight state immediately
      curAlt=0; maxAlt=0; prevAlt=0;
      fallCnt=0; nofallCnt=0;
      apoDetect=false; deployed=false;
      mpuApoConf=false; ffCnt=0;
      tAX=0; tAY=0; tAZ=0; tGX=0; tGY=0; tGZ=0; tN=0;
      calLock = true;
      tphase = TP_COLLECT;
      Serial.println("[TARE] Collecting samples...");
      return;

    case TP_COLLECT:
      if (tN < TARE_N && mpuOK) {
        if (mpuReadBurst(REG_ACCEL_OUT, buf, 14) == 14) {
          tAX += (int16_t)((buf[0]<<8)|buf[1]);
          tAY += (int16_t)((buf[2]<<8)|buf[3]);
          tAZ += (int16_t)((buf[4]<<8)|buf[5]);
          tGX += (int16_t)((buf[8]<<8)|buf[9]);
          tGY += (int16_t)((buf[10]<<8)|buf[11]);
          tGZ += (int16_t)((buf[12]<<8)|buf[13]);
          tN++;
        }
        return;  // one sample per tick
      }
      // Apply averaged offsets
      if (tN > 0 && mpuOK) {
        aOX = tAX / (float)tN;  aOY = tAY / (float)tN;
        aOZ = tAZ / (float)tN - ACCEL_LSB;
        gOX = tGX / (float)tN;  gOY = tGY / (float)tN;
        gOZ = tGZ / (float)tN;
        Serial.printf("[TARE] MPU cal done (%d samples)\n", tN);
      }
      calLock = false;
      tSetTimer = millis();
      tphase = TP_SETTLE;
      return;

    case TP_SETTLE:
      if (millis() - tSetTimer < 700) return;
      // Average 5 BMP readings for stable base altitude
      if (bmpOK) {
        float s = 0;
        for (int i = 0; i < 5; i++) { s += bmp.readAltitude(seaLvl); delay(150); }
        baseAlt = s / 5.0f;
        curAlt = 0; prevAlt = 0;
        Serial.printf("[TARE] Base alt = %.2f m\n", baseAlt);
      }
      tphase = TP_IDLE;
      Serial.println("[TARE] Done");
      return;
  }
}

// ═══════════════════════════════════════════════════════════════
//  EEPROM
// ═══════════════════════════════════════════════════════════════
void saveSettings() {
  EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VAL);
  EEPROM.put(EE_SV1,        (int16_t)sv1Ang);
  EEPROM.put(EE_SV2,        (int16_t)sv2Ang);
  EEPROM.write(EE_AUTO,     autoMode ? 1 : 0);
  EEPROM.put(EE_APOGEE,     (int16_t)apoMargin);
  EEPROM.write(EE_FUSION,   (uint8_t)fusionMode);
  EEPROM.put(EE_FF,         (int16_t)ffThresh);
  EEPROM.write(EE_ACTIVE_SV,(uint8_t)activeSv);
  EEPROM.put(EE_SEALVL,     seaLvl);
  EEPROM.commit();
}

void loadSettings() {
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VAL) {
    Serial.println("[EEPROM] First boot or magic changed — using defaults");
    sv1Ang=DEF_SV1; sv2Ang=DEF_SV2; autoMode=DEF_AUTO;
    apoMargin=DEF_APOGEE; activeSv=DEF_ACTIVE; seaLvl=DEF_SEALVL;
    fusionMode=DEF_FUSION; ffThresh=DEF_FF;
    saveSettings();
    return;
  }
  int16_t t;
  EEPROM.get(EE_SV1, t);     sv1Ang    = constrain((int)t, 0, 180);
  EEPROM.get(EE_SV2, t);     sv2Ang    = constrain((int)t, 0, 180);
  autoMode   = EEPROM.read(EE_AUTO) == 1;
  EEPROM.get(EE_APOGEE, t);  apoMargin = constrain((int)t, 0, 5000);
  fusionMode = constrain((int)EEPROM.read(EE_FUSION), 0, 2);
  EEPROM.get(EE_FF, t);      ffThresh  = constrain((int)t, 50, 900);
  activeSv   = constrain((int)EEPROM.read(EE_ACTIVE_SV), 1, 2);
  EEPROM.get(EE_SEALVL,      seaLvl);
  if (seaLvl < 900.0f || seaLvl > 1100.0f) seaLvl = DEF_SEALVL;
  Serial.printf("[EEPROM] Loaded: auto=%d fusion=%d sv1=%d sv2=%d ff=%d sea=%.2f\n",
    autoMode, fusionMode, sv1Ang, sv2Ang, ffThresh, seaLvl);
}

// ═══════════════════════════════════════════════════════════════
//  LoRa TELEMETRY TX
//  24 comma-separated fields, prefix "T,"
//  Matches ground station parseTelemetry() exactly.
// ═══════════════════════════════════════════════════════════════
void sendTelemetry() {
  char p[280];
  snprintf(p, sizeof(p),
    "T,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,"
    "%.3f,%.3f,%.1f,%d,%d,%d,"
    "%.1f,%d,%d,%d,%lu,%d,%d,"
    "%.3f,%.3f,%.3f",
    curAlt, maxAlt, bmpPres, bmpTempC,
    armed?1:0, deployed?1:0, apoDetect?1:0, fallCnt,
    netG, vertG, tiltDeg,
    fflag?1:0, mpuApoConf?1:0, ffCnt,
    mpuTempC, mpuOK?1:0, bmpOK?1:0, fusionMode,
    (unsigned long)ESP.getFreeHeap(), recOn?1:0, logN,
    accelX, accelY, accelZ
  );
  LoRa.beginPacket(); LoRa.print(p); LoRa.endPacket();
}

void sendAck(const char* cmd, bool ok) {
  char b[50];
  snprintf(b, sizeof(b), "A,%s,%d", cmd, ok ? 1 : 0);
  LoRa.beginPacket(); LoRa.print(b); LoRa.endPacket();
}

// ═══════════════════════════════════════════════════════════════
//  COMMAND PROCESSOR
// ═══════════════════════════════════════════════════════════════
void processCmd(String cmd) {
  cmd.trim();
  Serial.printf("[CMD] %s\n", cmd.c_str());

  if (cmd == "ARM") {
    armed = true;
    mpuApoConf = false; ffCnt = 0;
    fallCnt = 0; nofallCnt = 0;
    if (!recOn) { recOn = true; recStart = millis(); logN = 0; }
    sendAck("ARM", true);
  }
  else if (cmd == "DISARM") {
    armed = false;
    sendAck("DISARM", true);
  }
  else if (cmd == "FIRE") {
    if (armed) { deployNow("MANUAL"); sendAck("FIRE", true); }
    else       sendAck("FIRE", false);
  }
  else if (cmd == "RESET") {
    deployed=false; apoDetect=false; fallCnt=0; nofallCnt=0;
    maxAlt=0; armed=false; recOn=false; logN=0;
    mpuApoConf=false; ffCnt=0;
    sv1.write(0); sv2.write(0);
    sendAck("RESET", true);
  }
  else if (cmd == "TARE") {
    if (tphase == TP_IDLE) { tphase = TP_RESET; sendAck("TARE", true); }
    else sendAck("TARE", false);
  }
  else if (cmd == "CALMPU") {
    if (tphase == TP_IDLE && mpuOK) {
      tAX=0; tAY=0; tAZ=0; tGX=0; tGY=0; tGZ=0; tN=0;
      calLock = true;
      tphase = TP_COLLECT;
      sendAck("CALMPU", true);
    } else sendAck("CALMPU", false);
  }
  else if (cmd == "RECSTART") {
    recOn = true; recStart = millis(); logN = 0;
    sendAck("RECSTART", true);
  }
  else if (cmd == "RECSTOP") {
    recOn = false;
    sendAck("RECSTOP", true);
  }
  else if (cmd == "PING") {
    sendAck("PING", true);
  }
  else if (cmd == "GETLOG") {
    char buf[70];
    snprintf(buf, sizeof(buf), "LC,%d", logN);
    LoRa.beginPacket(); LoRa.print(buf); LoRa.endPacket();
    delay(20);
    for (int i = 0; i < logN; i++) {
      snprintf(buf, sizeof(buf), "L,%lu,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f",
        (unsigned long)log_[i].ms, log_[i].alt,
        log_[i].net, log_[i].vert,
        log_[i].ax, log_[i].ay, log_[i].az);
      LoRa.beginPacket(); LoRa.print(buf); LoRa.endPacket();
      delay(15);
    }
    LoRa.beginPacket(); LoRa.print("LE"); LoRa.endPacket();
  }
  else if (cmd.startsWith("SET:")) {
    StaticJsonDocument<300> doc;
    if (!deserializeJson(doc, cmd.substring(4))) {
      if (doc.containsKey("servo1Angle"))    sv1Ang    = constrain((int)doc["servo1Angle"],    0, 180);
      if (doc.containsKey("servo2Angle"))    sv2Ang    = constrain((int)doc["servo2Angle"],    0, 180);
      if (doc.containsKey("autoEnabled"))    autoMode  = (bool)doc["autoEnabled"];
      if (doc.containsKey("apogeeMargin"))   apoMargin = constrain((int)doc["apogeeMargin"],   0, 5000);
      if (doc.containsKey("activeServo"))    activeSv  = constrain((int)doc["activeServo"],    1, 2);
      if (doc.containsKey("seaLevel"))       seaLvl    = constrain((float)doc["seaLevel"],   900.0f, 1100.0f);
      if (doc.containsKey("fusionMode"))     fusionMode= constrain((int)doc["fusionMode"],     0, 2);
      if (doc.containsKey("freeFallThresh")) ffThresh  = constrain((int)doc["freeFallThresh"], 50, 900);
      saveSettings();
      char ack[64];
      snprintf(ack, sizeof(ack), "A,SET,1,f%d,a%d,sv%d", fusionMode, autoMode?1:0, sv1Ang);
      LoRa.beginPacket(); LoRa.print(ack); LoRa.endPacket();
    } else {
      sendAck("SET", false);
    }
  }
  else if (cmd.startsWith("TEST:")) {
    int c = cmd.indexOf(',', 5);
    if (c > 0) {
      svTestNum = constrain(cmd.substring(5, c).toInt(), 1, 2);
      int ang   = constrain(cmd.substring(c+1).toInt(), 0, 180);
      if (svTestNum == 1) sv1.write(ang); else sv2.write(ang);
      svTestEnd = millis() + 1500;
      svTestOn  = true;
      sendAck("TEST", true);
    }
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // 2-second delay: allows 3.3V rail to stabilise and sensors to
  // complete their internal power-on reset sequence.
  // With 100µF + 100nF caps on each sensor's VCC-GND this is
  // conservative but guarantees clean startup every time.
  delay(2000);
  Serial.println("\n=== ROCKET TX v6.0 ===");

  EEPROM.begin(EE_SIZE);
  loadSettings();

  // Start I2C bus
  // Wire.setTimeout(ms) is the correct ESP32 Arduino core method.
  // Wire.setWireTimeout() does not exist and will not compile.
  wireStart();
  delay(100);

  // Bus recovery first: clears any device stuck from a previous crash
  busRecover();
  delay(100);

  // ── BMP280 ──────────────────────────────────────────────────
  // Init BMP first. It is more sensitive to power-on timing.
  bmpOK = bmpInit(true);
  if (bmpOK) {
    float s = 0;
    for (int i = 0; i < 5; i++) { s += bmp.readAltitude(seaLvl); delay(150); }
    baseAlt = s / 5.0f;
    Serial.printf("[BMP] Base altitude = %.2f m\n", baseAlt);
  }

  // 200ms gap before MPU init — prevents bus contention between
  // two sensors initialising simultaneously on shared SDA/SCL.
  delay(200);

  // ── MPU-6500 ─────────────────────────────────────────────────
  mpuOK = mpuInit();
  if (mpuOK) {
    delay(100);
    mpuCalibrate(200);
  }

  // ── Servos ───────────────────────────────────────────────────
  // Servo power comes from external 5V, NOT the ESP32 3V3 rail.
  // Signal pins are GPIO25 and GPIO26 (3.3V logic, compatible with most servos).
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  sv1.setPeriodHertz(50); sv2.setPeriodHertz(50);
  sv1.attach(PIN_SERVO1, 500, 2400);
  sv2.attach(PIN_SERVO2, 500, 2400);
  sv1.write(0); sv2.write(0);

  // ── LoRa ─────────────────────────────────────────────────────
  // SPI bus is completely separate from I2C — no interference.
  SPI.begin(18, 19, 23, PIN_LORA_SS);
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[LORA] INIT FAILED");
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setTxPower(LORA_PWR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();

  Serial.printf("[SYS] BMP=%s  MPU=%s  LoRa=OK  auto=%s  fusion=%d  heap=%luB\n",
    bmpOK?"OK":"FAIL", mpuOK?"OK":"FAIL",
    autoMode?"ON":"OFF", fusionMode, ESP.getFreeHeap());
  Serial.println("[SYS] Ready.\n");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // Return servo to 0 after test pulse
  if (svTestOn && now >= svTestEnd) {
    svTestOn = false;
    if (svTestNum == 1) sv1.write(0); else sv2.write(0);
  }

  // Tare state machine (one step per loop)
  if (tphase != TP_IDLE) tareStep();

  // Sensor sample at 25 Hz
  if (now - tSensor >= T_SENSOR_MS) {
    tSensor = now;
    sensorTick();
  }

  // Watchdog at 5 Hz / 5s interval
  if (now - tWdg >= T_WDG_MS) {
    tWdg = now;
    watchdog();
  }

  // LoRa TX + command RX window
  if (now - tLora >= T_LORA_MS) {
    tLora = now;
    sendTelemetry();
    // Listen for incoming commands for LORA_RX_WIN ms
    unsigned long rxDeadline = millis() + LORA_RX_WIN;
    while (millis() < rxDeadline) {
      if (LoRa.parsePacket() > 0) {
        String cmd = "";
        while (LoRa.available()) cmd += (char)LoRa.read();
        processCmd(cmd);
        break;
      }
      delayMicroseconds(200);
    }
  }
}
