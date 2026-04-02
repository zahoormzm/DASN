/*
 * DASN Mobile Unit — ESP32 Sensor Test
 * Board: ESP32 WROOM-32
 *
 * Purpose:
 *   Validate ESP32-side sensors and encoder inputs using the final pin layout.
 *   (Buzzer + OLED are now on Aiduino, not on ESP32.)
 *
 * Serial monitor: 115200
 */

#ifndef ARDUINO_ARCH_ESP32
#error "This sketch requires an ESP32 board selection."
#endif

#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Adafruit_INA219.h>
#include <SparkFun_VL53L5CX_Library.h>

#include "../../shared/config.h"

// ESP32 mobile-unit pins
#define ENC1_A  34  // no internal pull-up
#define ENC1_B  32
#define ENC1_SIGN -1
#define ENC2_A  35  // no internal pull-up
#define ENC2_B  13
#define ENC2_SIGN 1
#define ENC3_A  25
#define ENC3_B  26
#define ENC3_SIGN -1
#define ENC4_A  27
#define ENC4_B  33
#define ENC4_SIGN -1

#define PIN_US_TRIG 18
#define PIN_US_ECHO 19

#define AIDUINO_TX 16
#define AIDUINO_RX 17

#define PIN_SDA 21
#define PIN_SCL 22

SparkFun_ISM330DHCX imu;
SFE_MMC5983MA       mag;
Adafruit_INA219     ina219(I2C_ADDR_INA219);
SparkFun_VL53L5CX   tofSensor;
VL53L5CX_ResultsData tofResults;

volatile long enc1Count = 0, enc2Count = 0, enc3Count = 0, enc4Count = 0;

void IRAM_ATTR enc1ISR() { enc1Count += (digitalRead(ENC1_B) ? 1 : -1) * ENC1_SIGN; }
void IRAM_ATTR enc2ISR() { enc2Count += (digitalRead(ENC2_B) ? 1 : -1) * ENC2_SIGN; }
void IRAM_ATTR enc3ISR() { enc3Count += (digitalRead(ENC3_B) ? 1 : -1) * ENC3_SIGN; }
void IRAM_ATTR enc4ISR() { enc4Count += (digitalRead(ENC4_B) ? 1 : -1) * ENC4_SIGN; }

bool userAbort() { return Serial.available() > 0; }
void flushSerial() { while (Serial.available()) Serial.read(); }

float readUltrasonicMM() {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, 30000);
    if (duration == 0) return -1.0f;
    return duration * 0.343f / 2.0f;
}

void printMenu() {
    Serial.println();
    Serial.println("=== DASN ESP32 SENSOR TEST ===");
    Serial.println("1 - I2C scan");
    Serial.println("2 - IMU (ISM330DHCX)");
    Serial.println("3 - Magnetometer (MMC5983MA)");
    Serial.println("4 - INA219 battery voltage");
    Serial.println("5 - VL53L5CX ToF 8x8");
    Serial.println("6 - Encoders (spin wheels by hand)");
    Serial.println("7 - Ultrasonic (HC-SR04)");
    Serial.println("8 - Aiduino link + buzzer/OLED test");
    Serial.println("a - One-shot all summary");
    Serial.println("Press key during a running test to stop.");
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, AIDUINO_RX, AIDUINO_TX);

    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    digitalWrite(PIN_US_TRIG, LOW);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

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

    Serial.println("\n[DASN] ESP32 sensor test ready");
    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    char c = Serial.read();
    flushSerial();

    if (c == '1') {
        Serial.println("--- I2C SCAN ---");
        Serial.println("Expected:");
        Serial.println("  0x29 VL53L5CX");
        Serial.println("  0x30 MMC5983MA");
        Serial.println("  0x40 INA219");
        Serial.println("  0x6A or 0x6B ISM330DHCX");
        Serial.println("Optional legacy: 0x3C SSD1306 on ESP32");

        int found = 0;
        bool hasToF = false, hasMag = false, hasINA = false, hasIMU = false;
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.printf("  FOUND 0x%02X", addr);
                if (addr == 0x29) {
                    Serial.print(" - VL53L5CX");
                    hasToF = true;
                } else if (addr == 0x30) {
                    Serial.print(" - MMC5983MA");
                    hasMag = true;
                } else if (addr == 0x40) {
                    Serial.print(" - INA219");
                    hasINA = true;
                } else if (addr == 0x6A || addr == 0x6B) {
                    Serial.print(" - ISM330DHCX");
                    hasIMU = true;
                } else if (addr == 0x3C) {
                    Serial.print(" - SSD1306 (legacy optional)");
                } else if (addr == 0x7E) {
                    Serial.print(" - RESERVED/ARTIFACT");
                }
                Serial.println();
                found++;
            }
        }
        Serial.printf("Total found: %d\n", found);
        Serial.printf("ToF:%s Mag:%s INA:%s IMU:%s\n",
                      hasToF ? "OK" : "MISS",
                      hasMag ? "OK" : "MISS",
                      hasINA ? "OK" : "MISS",
                      hasIMU ? "OK" : "MISS");
    } else if (c == '2') {
        Serial.println("--- IMU TEST ---");
        if (!imu.begin()) {
            Serial.println("FAIL: IMU not found");
        } else {
            imu.setDeviceConfig();
            imu.setBlockDataUpdate();
            imu.setAccelDataRate(ISM_XL_ODR_104Hz);
            imu.setAccelFullScale(ISM_4g);
            imu.setGyroDataRate(ISM_GY_ODR_104Hz);
            imu.setGyroFullScale(ISM_500dps);
            Serial.println("Press any key to stop.");
            while (!userAbort()) {
                if (imu.checkStatus()) {
                    sfe_ism_data_t accel, gyro;
                    imu.getAccel(&accel);
                    imu.getGyro(&gyro);
                    Serial.printf("ACC mg x=%7.1f y=%7.1f z=%7.1f | GYR mdps x=%7.1f y=%7.1f z=%7.1f\n",
                                  accel.xData, accel.yData, accel.zData,
                                  gyro.xData, gyro.yData, gyro.zData);
                }
                delay(200);
            }
            flushSerial();
        }
    } else if (c == '3') {
        Serial.println("--- MAG TEST ---");
        if (!mag.begin()) {
            Serial.println("FAIL: magnetometer not found");
        } else {
            mag.softReset();
            delay(10);
            mag.setFilterBandwidth(800);
            mag.setContinuousModeFrequency(100);
            mag.enableContinuousMode();
            Serial.println("Press any key to stop.");
            while (!userAbort()) {
                uint32_t rawX = 0, rawY = 0, rawZ = 0;
                mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);
                float mx = ((float)rawX - 131072.0f) / 131072.0f * 8.0f;
                float my = ((float)rawY - 131072.0f) / 131072.0f * 8.0f;
                float heading = atan2f(my, mx) * 180.0f / PI;
                if (heading < 0) heading += 360.0f;
                Serial.printf("Heading: %6.1f deg\n", heading);
                delay(200);
            }
            flushSerial();
        }
    } else if (c == '4') {
        Serial.println("--- INA219 TEST ---");
        if (!ina219.begin()) {
            Serial.println("FAIL: INA219 not found");
        } else {
            ina219.setCalibration_16V_400mA();
            Serial.println("Press any key to stop.");
            while (!userAbort()) {
                Serial.printf("Battery V: %.2f\n", ina219.getBusVoltage_V());
                delay(300);
            }
            flushSerial();
        }
    } else if (c == '5') {
        Serial.println("--- TOF TEST ---");
        if (!tofSensor.begin()) {
            Serial.println("FAIL: VL53L5CX not found");
        } else {
            tofSensor.setResolution(64);
            tofSensor.setRangingFrequency(15);
            tofSensor.startRanging();
            Serial.println("Press any key to stop.");
            while (!userAbort()) {
                if (tofSensor.isDataReady() && tofSensor.getRangingData(&tofResults)) {
                    Serial.print("ToF[0..7] mm: ");
                    for (int i = 0; i < 8; i++) {
                        Serial.print(tofResults.distance_mm[i]);
                        if (i < 7) Serial.print(",");
                    }
                    Serial.println();
                }
                delay(120);
            }
            tofSensor.stopRanging();
            flushSerial();
        }
    } else if (c == '6') {
        Serial.println("--- ENCODER TEST ---");
        Serial.println("Spin each wheel by hand. Press any key to stop.");
        long p1 = enc1Count, p2 = enc2Count, p3 = enc3Count, p4 = enc4Count;
        while (!userAbort()) {
            noInterrupts();
            long c1 = enc1Count, c2 = enc2Count, c3 = enc3Count, c4 = enc4Count;
            interrupts();
            Serial.printf("FL:%8ld (%4ld)  FR:%8ld (%4ld)  RL:%8ld (%4ld)  RR:%8ld (%4ld)\n",
                          c1, c1 - p1, c2, c2 - p2, c3, c3 - p3, c4, c4 - p4);
            p1 = c1; p2 = c2; p3 = c3; p4 = c4;
            delay(200);
        }
        flushSerial();
    } else if (c == '7') {
        Serial.println("--- ULTRASONIC TEST ---");
        Serial.println("Press any key to stop.");
        while (!userAbort()) {
            Serial.printf("Rear distance mm: %.0f\n", readUltrasonicMM());
            delay(200);
        }
        flushSerial();
    } else if (c == '8') {
        Serial.println("--- AIDUINO LINK + BUZZER/OLED TEST ---");
        Serial.println("Keep wheels lifted before running this test.");
        Serial2.println("P");
        unsigned long t0 = millis();
        bool gotAny = false;
        while (millis() - t0 < 2000) {
            while (Serial2.available()) {
                gotAny = true;
                Serial.write(Serial2.read());
            }
        }
        if (!gotAny) {
            Serial.println("No response. Check UART wiring TX2/RX2 and Aiduino sketch.");
        } else {
            Serial.println("Triggering Aiduino buzzer and OLED activity...");
            Serial2.println("B,2000,250");
            delay(300);
            Serial2.println("E");
            delay(100);
            Serial2.println("M,80,1,80,1");
            delay(700);
            Serial2.println("S");
            delay(100);
            Serial2.println("D");
            Serial.println("Expected: short buzzer beep and OLED status update on Aiduino.");
        }
    } else if (c == 'a' || c == 'A') {
        Serial.println("--- ALL SUMMARY ---");
        // I2C quick
        int found = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) found++;
        }
        Serial.printf("I2C devices found: %d\n", found);
        Serial.printf("Ultrasonic mm: %.0f\n", readUltrasonicMM());
        noInterrupts();
        long c1 = enc1Count, c2 = enc2Count, c3 = enc3Count, c4 = enc4Count;
        interrupts();
        Serial.printf("Enc FL:%ld FR:%ld RL:%ld RR:%ld\n", c1, c2, c3, c4);
    } else {
        Serial.println("Unknown option");
    }

    printMenu();
}
