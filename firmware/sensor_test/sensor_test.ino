/*
 * DASN Sensor Test — Legacy Interactive Hardware Verification
 * Board: ESP32 WROOM-32
 *
 * NOTE:
 *   This is the older all-in-one test sketch (kept for compatibility).
 *   Current mobile-unit validation files are:
 *   - mobile_unit_esp32_sensor_test.ino
 *   - mobile_unit_esp32_motor_encoder_test.ino
 *   - mobile_unit_aiduino_motor_test.ino
 *
 * Flash this sketch, open Serial Monitor at 115200 baud.
 * Type the number of the test you want to run.
 *
 * Menu:
 *   1 — I2C scan (find all devices on the bus)
 *   2 — IMU (ISM330DHCX) — accel + gyro, hold still to verify ~0,0,1g
 *   3 — Magnetometer (MMC5983MA) — heading in degrees, rotate to verify
 *   4 — INA219 — battery voltage (should match multimeter)
 *   5 — VL53L5CX ToF — 8x8 depth grid in mm, put hand in front
 *   6 — Encoders — turn wheels by hand, verify tick counts change
 *   7 — Ultrasonic (HC-SR04) — rear distance in mm
 *   8 — Buzzer — plays 3 tones
 *   9 — Aiduino serial — sends enable + motor test + stop
 *   0 — OLED — displays test pattern
 *   a — ALL sensors continuous (1 Hz summary)
 *
 * Press any key to stop a running test and return to menu.
 */

#ifndef ARDUINO_ARCH_ESP32
#error "sensor_test.ino requires an ESP32 board selection."
#endif

#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_VL53L5CX_Library.h>

#include "../shared/config.h"

// ─── Pin definitions (same as mobile_unit_esp32) ──────────────────────────
#define ENC1_A 34  // no internal pullup
#define ENC1_B 32
#define ENC1_SIGN -1
#define ENC2_A 35  // no internal pullup
#define ENC2_B 13
#define ENC2_SIGN 1
#define ENC3_A 25
#define ENC3_B 26
#define ENC3_SIGN -1
#define ENC4_A 27
#define ENC4_B 33
#define ENC4_SIGN -1

#define PIN_US_TRIG 18
#define PIN_US_ECHO 19
#define PIN_BUZZER_BOT 4
#define AIDUINO_TX 16
#define AIDUINO_RX 17
#define PIN_SDA 21
#define PIN_SCL 22
#define SCREEN_W 128
#define SCREEN_H 64

// ─── Sensor objects ────────────────────────────────────────────────────────
SparkFun_ISM330DHCX imu;
SFE_MMC5983MA mag;
Adafruit_INA219 ina219(I2C_ADDR_INA219);
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);
SparkFun_VL53L5CX tofSensor;
VL53L5CX_ResultsData tofResults;

// ─── Encoder counters ──────────────────────────────────────────────────────
volatile long enc1Count = 0, enc2Count = 0, enc3Count = 0, enc4Count = 0;

void IRAM_ATTR enc1ISR() {
  enc1Count += (digitalRead(ENC1_B) ? 1 : -1) * ENC1_SIGN;
}
void IRAM_ATTR enc2ISR() {
  enc2Count += (digitalRead(ENC2_B) ? 1 : -1) * ENC2_SIGN;
}
void IRAM_ATTR enc3ISR() {
  enc3Count += (digitalRead(ENC3_B) ? 1 : -1) * ENC3_SIGN;
}
void IRAM_ATTR enc4ISR() {
  enc4Count += (digitalRead(ENC4_B) ? 1 : -1) * ENC4_SIGN;
}

// ─── Helpers ───────────────────────────────────────────────────────────────
bool userAbort() {
  return Serial.available() > 0;
}
void flushSerial() {
  while (Serial.available()) Serial.read();
}

void printMenu() {
  Serial.println();
  Serial.println("=== DASN SENSOR TEST ===");
  Serial.println("1 — I2C scan");
  Serial.println("2 — IMU (accel + gyro)");
  Serial.println("3 — Magnetometer (heading)");
  Serial.println("4 — INA219 (battery voltage)");
  Serial.println("5 — VL53L5CX ToF (8x8 depth)");
  Serial.println("6 — Encoders (turn wheels)");
  Serial.println("7 — Ultrasonic (rear distance)");
  Serial.println("8 — Aiduino buzzer test");
  Serial.println("9 — Aiduino serial (motor test)");
  Serial.println("0 — Aiduino OLED activity test");
  Serial.println("a — ALL sensors continuous");
  Serial.println("========================");
  Serial.println("Enter test number:");
}

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, AIDUINO_RX, AIDUINO_TX);

  // Buzzer (use generic tone/noTone for ESP32 core compatibility)
  pinMode(PIN_BUZZER_BOT, OUTPUT);
  noTone(PIN_BUZZER_BOT);

  // Ultrasonic
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // Encoders
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), enc4ISR, RISING);

  delay(500);
  Serial.println("\n[DASN] Sensor Test Ready");
  printMenu();
}

// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    flushSerial();
    Serial.println();

    switch (c) {
      case '1': testI2CScan(); break;
      case '2': testIMU(); break;
      case '3': testMag(); break;
      case '4': testINA219(); break;
      case '5': testToF(); break;
      case '6': testEncoders(); break;
      case '7': testUltrasonic(); break;
      case '8': testBuzzer(); break;
      case '9': testAiduino(); break;
      case '0': testOLED(); break;
      case 'a':
      case 'A': testAll(); break;
      default:
        Serial.println("Unknown option");
        break;
    }
    printMenu();
  }
}

// ─── 1: I2C Scan ───────────────────────────────────────────────────────────
void testI2CScan() {
  Serial.println("--- I2C SCAN ---");
  Serial.println("Expected devices (mobile-unit ESP32):");
  Serial.println("  0x29 = VL53L5CX (ToF)");
  Serial.println("  0x30 = MMC5983MA (Mag)");
  Serial.println("  0x40 = INA219 (Battery)");
  Serial.println("  0x6A or 0x6B = ISM330DHCX (IMU)");
  Serial.println("Optional (legacy):");
  Serial.println("  0x3C = SSD1306 (OLED on ESP32)");
  Serial.println();

  int found = 0;
  bool hasToF = false;
  bool hasMag = false;
  bool hasINA = false;
  bool hasIMU = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  FOUND: 0x%02X", addr);
      if (addr == 0x29) {
        Serial.print(" — VL53L5CX");
        hasToF = true;
      } else if (addr == 0x30) {
        Serial.print(" — MMC5983MA");
        hasMag = true;
      } else if (addr == 0x3C) {
        Serial.print(" — SSD1306 (optional/legacy)");
      } else if (addr == 0x40) {
        Serial.print(" — INA219");
        hasINA = true;
      } else if (addr == 0x6A || addr == 0x6B) {
        Serial.print(" — ISM330DHCX");
        hasIMU = true;
      } else if (addr == 0x7E) {
        Serial.print(" — RESERVED/ARTIFACT");
      } else {
        Serial.print(" — UNKNOWN");
      }
      Serial.println();
      found++;
    }
  }
  Serial.printf("\nTotal devices found: %d\n", found);
  Serial.printf("ToF: %s | Mag: %s | INA219: %s | IMU: %s\n",
                hasToF ? "OK" : "MISSING",
                hasMag ? "OK" : "MISSING",
                hasINA ? "OK" : "MISSING",
                hasIMU ? "OK" : "MISSING");
  if (!(hasToF && hasMag && hasINA && hasIMU)) {
    Serial.println("WARNING: One or more required ESP32 devices missing.");
  }
}

// ─── 2: IMU Test ───────────────────────────────────────────────────────────
void testIMU() {
  Serial.println("--- IMU TEST (ISM330DHCX) ---");
  Serial.println("Hold the bot still and level.");
  Serial.println("Expected: ax~0, ay~0, az~1000 mg | gx~0, gy~0, gz~0 mdps");
  Serial.println("Press any key to stop.\n");

  if (!imu.begin()) {
    Serial.println("FAIL: ISM330DHCX not found!");
    return;
  }
  imu.setDeviceConfig();
  imu.setBlockDataUpdate();
  imu.setAccelDataRate(ISM_XL_ODR_104Hz);
  imu.setAccelFullScale(ISM_4g);
  imu.setGyroDataRate(ISM_GY_ODR_104Hz);
  imu.setGyroFullScale(ISM_500dps);
  Serial.println("ISM330DHCX initialized OK\n");

  while (!userAbort()) {
    if (imu.checkStatus()) {
      sfe_ism_data_t accel, gyro;
      imu.getAccel(&accel);
      imu.getGyro(&gyro);

      Serial.printf("Accel (mg):  X=%8.1f  Y=%8.1f  Z=%8.1f  |  ",
                    accel.xData, accel.yData, accel.zData);
      Serial.printf("Gyro (mdps): X=%8.1f  Y=%8.1f  Z=%8.1f\n",
                    gyro.xData, gyro.yData, gyro.zData);

      // Sanity check
      float az_g = accel.zData / 1000.0f;
      if (az_g < 0.8f || az_g > 1.2f) {
        Serial.println("  ^ WARNING: Z accel not ~1g. Is the board level?");
      }
    }
    delay(200);
  }
  flushSerial();
  Serial.println("\nIMU test stopped.");
}

// ─── 3: Magnetometer Test ──────────────────────────────────────────────────
void testMag() {
  Serial.println("--- MAGNETOMETER TEST (MMC5983MA) ---");
  Serial.println("Rotate the bot slowly. Heading should change 0-360.");
  Serial.println("Press any key to stop.\n");

  if (!mag.begin()) {
    Serial.println("FAIL: MMC5983MA not found!");
    return;
  }
  mag.softReset();
  delay(10);
  mag.setFilterBandwidth(800);
  mag.setContinuousModeFrequency(100);
  mag.enableContinuousMode();
  Serial.println("MMC5983MA initialized OK\n");

  while (!userAbort()) {
    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

    float mx = ((float)rawX - 131072.0f) / 131072.0f * 8.0f;
    float my = ((float)rawY - 131072.0f) / 131072.0f * 8.0f;
    float mz = ((float)rawZ - 131072.0f) / 131072.0f * 8.0f;

    float heading = atan2f(my, mx) * 180.0f / PI;
    if (heading < 0) heading += 360.0f;

    Serial.printf("Mag (Gauss): X=%6.3f  Y=%6.3f  Z=%6.3f  |  Heading: %6.1f deg\n",
                  mx, my, mz, heading);
    delay(200);
  }
  flushSerial();
  Serial.println("\nMagnetometer test stopped.");
}

// ─── 4: INA219 Test ────────────────────────────────────────────────────────
void testINA219() {
  Serial.println("--- INA219 TEST (Battery Voltage) ---");
  Serial.println("Compare with multimeter on battery terminals.");
  Serial.println("Press any key to stop.\n");

  if (!ina219.begin()) {
    Serial.println("FAIL: INA219 not found!");
    return;
  }
  ina219.setCalibration_16V_400mA();
  Serial.println("INA219 initialized OK (voltage-only mode)\n");

  while (!userAbort()) {
    float busV = ina219.getBusVoltage_V();
    float shuntmV = ina219.getShuntVoltage_mV();

    Serial.printf("Bus Voltage: %6.3f V  |  Shunt: %6.3f mV", busV, shuntmV);

    // Sanity checks
    if (busV < 0.1f) {
      Serial.print("  <- WARNING: ~0V. Is battery connected?");
    } else if (busV > 13.0f) {
      Serial.print("  <- WARNING: Unusually high. Check wiring.");
    }
    if (abs(shuntmV) > 5.0f) {
      Serial.print("  <- WARNING: Shunt should be ~0 (VIN+ shorted to VIN-)");
    }
    Serial.println();
    delay(500);
  }
  flushSerial();
  Serial.println("\nINA219 test stopped.");
}

// ─── 5: VL53L5CX ToF Test ──────────────────────────────────────────────────
void testToF() {
  Serial.println("--- VL53L5CX ToF TEST (8x8 Depth Grid) ---");
  Serial.println("Place hand in front of sensor at various distances.");
  Serial.println("Press any key to stop.\n");

  if (!tofSensor.begin()) {
    Serial.println("FAIL: VL53L5CX not found!");
    return;
  }
  tofSensor.setResolution(64);
  tofSensor.setRangingFrequency(15);
  tofSensor.startRanging();
  Serial.println("VL53L5CX initialized OK (8x8 @ 15Hz)\n");

  while (!userAbort()) {
    if (tofSensor.isDataReady()) {
      if (tofSensor.getRangingData(&tofResults)) {
        // Print as 8x8 grid
        Serial.println("Distance (mm):");
        for (int row = 0; row < 8; row++) {
          Serial.print("  ");
          for (int col = 0; col < 8; col++) {
            int idx = row * 8 + col;
            Serial.printf("%5d", tofResults.distance_mm[idx]);
          }
          Serial.println();
        }

        // Print min/max
        int minD = 9999, maxD = 0;
        for (int i = 0; i < 64; i++) {
          int d = tofResults.distance_mm[i];
          if (d > 0 && d < minD) minD = d;
          if (d > maxD) maxD = d;
        }
        Serial.printf("Min: %d mm  Max: %d mm\n\n", minD, maxD);
      }
    }
    delay(200);
  }
  tofSensor.stopRanging();
  flushSerial();
  Serial.println("\nToF test stopped.");
}

// ─── 6: Encoder Test ────────────────────────────────────────────────────────
void testEncoders() {
  Serial.println("--- ENCODER TEST (4 encoders) ---");
  Serial.println("Turn each wheel by hand. Ticks should change.");
  Serial.println("1 full wheel revolution = 300 ticks");
  Serial.println("Enc1=FL(25,26) Enc2=FR(27,33) Enc3=RL(34,32) Enc4=RR(35,13)");
  Serial.println("Press any key to stop.\n");

  // Reset counts
  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  enc3Count = 0;
  enc4Count = 0;
  interrupts();

  while (!userAbort()) {
    noInterrupts();
    long e1 = enc1Count, e2 = enc2Count, e3 = enc3Count, e4 = enc4Count;
    interrupts();

    float rev1 = e1 / 300.0f, rev2 = e2 / 300.0f;
    float rev3 = e3 / 300.0f, rev4 = e4 / 300.0f;
    float dist1 = e1 * 0.000357f, dist2 = e2 * 0.000357f;
    float dist3 = e3 * 0.000357f, dist4 = e4 * 0.000357f;

    Serial.printf("FL: %6ld ticks (%5.2f rev, %6.3f m)  |  ", e1, rev1, dist1);
    Serial.printf("FR: %6ld ticks (%5.2f rev, %6.3f m)\n", e2, rev2, dist2);
    Serial.printf("RL: %6ld ticks (%5.2f rev, %6.3f m)  |  ", e3, rev3, dist3);
    Serial.printf("RR: %6ld ticks (%5.2f rev, %6.3f m)\n\n", e4, rev4, dist4);

    delay(250);
  }
  flushSerial();
  Serial.println("\nEncoder test stopped.");
}

// ─── 7: Ultrasonic Test ─────────────────────────────────────────────────────
void testUltrasonic() {
  Serial.println("--- ULTRASONIC TEST (HC-SR04) ---");
  Serial.println("Place object behind bot. Verify distance with ruler.");
  Serial.println("Press any key to stop.\n");

  while (!userAbort()) {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    unsigned long dur = pulseIn(PIN_US_ECHO, HIGH, 30000);
    float distMM = (dur == 0) ? -1.0f : (float)dur * 0.343f / 2.0f;

    if (distMM < 0) {
      Serial.println("Distance: OUT OF RANGE (no echo)");
    } else {
      Serial.printf("Distance: %6.0f mm  (%5.1f cm)\n", distMM, distMM / 10.0f);
    }
    delay(200);
  }
  flushSerial();
  Serial.println("\nUltrasonic test stopped.");
}

// ─── 8: Buzzer Test ─────────────────────────────────────────────────────────
void testBuzzer() {
  Serial.println("--- AIDUINO BUZZER TEST ---");
  Serial.println("Sending tones to Aiduino buzzer over UART...\n");

  Serial2.println("B,1200,250");
  delay(350);
  Serial2.println("B,1800,250");
  delay(350);
  Serial2.println("B,2400,250");
  delay(350);
  Serial2.println("B,0");
  readAiduinoResponse();

  Serial.println("If buzzer is on Aiduino D11, you should hear 3 ascending tones.");
}

// ─── 9: Aiduino Serial Test ────────────────────────────────────────────────
void testAiduino() {
  Serial.println("--- AIDUINO SERIAL TEST ---");
  Serial.println("Sending commands to Aiduino via Serial2 (GPIO 16/17)...\n");

  // Flush any pending data
  while (Serial2.available()) Serial2.read();

  // Enable motors
  Serial.println("1) Sending 'E' (enable motors)...");
  Serial2.println("E");
  delay(500);
  readAiduinoResponse();

  // Brief motor test — low PWM
  Serial.println("2) Sending 'M,50,1,50,1' (slow forward)...");
  Serial.println("   WARNING: Wheels will spin briefly!");
  Serial2.println("M,50,1,50,1");
  delay(1000);
  readAiduinoResponse();

  // Stop
  Serial.println("3) Sending 'S' (emergency stop)...");
  Serial2.println("S");
  delay(500);
  readAiduinoResponse();

  // Disable
  Serial.println("4) Sending 'D' (disable motors)...");
  Serial2.println("D");
  delay(500);
  readAiduinoResponse();

  Serial.println("\nAiduino test complete.");
  Serial.println("If you saw [AID] responses above, serial link is working.");
  Serial.println("If no responses, check UART wiring (ESP32 TX2=16 -> Aiduino RX, ESP32 RX2=17 -> Aiduino TX).");
}

void readAiduinoResponse() {
  delay(100);
  bool gotData = false;
  while (Serial2.available()) {
    char c = Serial2.read();
    if (!gotData) {
      Serial.print("   Response: ");
      gotData = true;
    }
    Serial.print(c);
  }
  if (!gotData) {
    Serial.println("   Response: (none — no data from Aiduino)");
  } else {
    Serial.println();
  }
}

// ─── 0: OLED Test ──────────────────────────────────────────────────────────
void testOLED() {
  Serial.println("--- AIDUINO OLED ACTIVITY TEST ---");
  Serial.println("Sending movement commands so Aiduino OLED values change.");
  Serial.println("Keep wheels lifted.\n");

  Serial2.println("E");
  delay(100);
  Serial2.println("M,90,1,90,1");
  delay(700);
  Serial2.println("S");
  delay(120);
  Serial2.println("D");
  readAiduinoResponse();

  Serial.println("Check Aiduino OLED: motor/status fields should update.");
}

// ─── a: All Sensors Continuous ──────────────────────────────────────────────
void testAll() {
  Serial.println("--- ALL SENSORS CONTINUOUS (1 Hz) ---");
  Serial.println("Press any key to stop.\n");

  // Init all sensors
  bool hasIMU = imu.begin();
  if (hasIMU) {
    imu.setDeviceConfig();
    imu.setBlockDataUpdate();
    imu.setAccelDataRate(ISM_XL_ODR_104Hz);
    imu.setAccelFullScale(ISM_4g);
    imu.setGyroDataRate(ISM_GY_ODR_104Hz);
    imu.setGyroFullScale(ISM_500dps);
  }
  bool hasMag = mag.begin();
  if (hasMag) {
    mag.softReset();
    delay(10);
    mag.setFilterBandwidth(800);
    mag.setContinuousModeFrequency(100);
    mag.enableContinuousMode();
  }
  bool hasINA = ina219.begin();
  if (hasINA) ina219.setCalibration_16V_400mA();

  bool hasToF = tofSensor.begin();
  if (hasToF) {
    tofSensor.setResolution(64);
    tofSensor.setRangingFrequency(15);
    tofSensor.startRanging();
  }

  Serial.printf("Sensors: IMU=%s MAG=%s INA=%s ToF=%s\n\n",
                hasIMU ? "OK" : "FAIL", hasMag ? "OK" : "FAIL",
                hasINA ? "OK" : "FAIL", hasToF ? "OK" : "FAIL");

  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  enc3Count = 0;
  enc4Count = 0;
  interrupts();

  while (!userAbort()) {
    Serial.println("────────────────────────────────────────");

    // IMU
    if (hasIMU && imu.checkStatus()) {
      sfe_ism_data_t accel, gyro;
      imu.getAccel(&accel);
      imu.getGyro(&gyro);
      Serial.printf("IMU  | Accel(mg) X=%7.1f Y=%7.1f Z=%7.1f | Gyro(mdps) X=%7.1f Y=%7.1f Z=%7.1f\n",
                    accel.xData, accel.yData, accel.zData,
                    gyro.xData, gyro.yData, gyro.zData);
    }

    // Mag
    if (hasMag) {
      uint32_t rx = 0, ry = 0, rz = 0;
      mag.getMeasurementXYZ(&rx, &ry, &rz);
      float mx = ((float)rx - 131072.0f) / 131072.0f * 8.0f;
      float my = ((float)ry - 131072.0f) / 131072.0f * 8.0f;
      float hdg = atan2f(my, mx) * 180.0f / PI;
      if (hdg < 0) hdg += 360.0f;
      Serial.printf("MAG  | Heading: %6.1f deg\n", hdg);
    }

    // INA219
    if (hasINA) {
      Serial.printf("BAT  | Voltage: %6.3f V\n", ina219.getBusVoltage_V());
    }

    // ToF (just min distance)
    if (hasToF && tofSensor.isDataReady()) {
      if (tofSensor.getRangingData(&tofResults)) {
        int minD = 9999;
        for (int i = 0; i < 64; i++) {
          int d = tofResults.distance_mm[i];
          if (d > 0 && d < minD) minD = d;
        }
        Serial.printf("ToF  | Closest object: %d mm\n", minD);
      }
    }

    // Encoders
    noInterrupts();
    long e1 = enc1Count, e2 = enc2Count, e3 = enc3Count, e4 = enc4Count;
    interrupts();
    Serial.printf("ENC  | FL=%ld  FR=%ld  RL=%ld  RR=%ld\n", e1, e2, e3, e4);

    // Ultrasonic
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);
    unsigned long dur = pulseIn(PIN_US_ECHO, HIGH, 30000);
    float usDist = (dur == 0) ? -1.0f : (float)dur * 0.343f / 2.0f;
    Serial.printf("US   | Rear: %.0f mm\n", usDist);

    Serial.println();
    delay(1000);
  }

  if (hasToF) tofSensor.stopRanging();
  flushSerial();
  Serial.println("\nAll-sensor test stopped.");
}
