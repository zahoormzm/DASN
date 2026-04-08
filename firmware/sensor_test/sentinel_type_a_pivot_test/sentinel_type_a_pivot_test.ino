/*
 * DASN Sentinel Type A Pivot Test
 * Tests SmartElex sound sensor, VL53L5CX approach features, BME688, CAP1203, LCD, LEDs.
 */

#ifndef ARDUINO_ARCH_ESP32
#error "This sketch requires an ESP32 board selection."
#endif

#include <Wire.h>

#include <Adafruit_BME680.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_VL53L5CX_Library.h>

#include "config.h"

Adafruit_BME680 bme;
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofResults;

LiquidCrystal_I2C lcdPrimary(LCD_I2C_ADDR_PRIMARY, 16, 2);
LiquidCrystal_I2C lcdFallback(LCD_I2C_ADDR_FALLBACK, 16, 2);
LiquidCrystal_I2C *lcd = nullptr;

static bool lcdReady = false;
static bool bmeReady = false;
static bool cap1203Ready = false;
static bool tofReady = false;
static bool soundDoutLast = false;
static unsigned long soundEventCount = 0;

bool userAbort() { return Serial.available() > 0; }
void flushSerial() { while (Serial.available()) Serial.read(); }

void lcdShow(const char *line1, const char *line2) {
    if (!lcdReady || lcd == nullptr) return;
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print(line1);
    lcd->setCursor(0, 1);
    lcd->print(line2);
}

void printMenu() {
    Serial.println();
    Serial.println("=== TYPE A PIVOT TEST ===");
    Serial.println("1 - I2C scan");
    Serial.println("2 - LCD + LEDs");
    Serial.println("3 - Sound sensor live");
    Serial.println("4 - VL53L5CX approach live");
    Serial.println("5 - BME688 live");
    Serial.println("6 - CAP1203 raw touch");
    Serial.println("7 - One-shot summary");
    Serial.println("h - Help/menu");
    Serial.println("Press any key during a live test to stop.");
}

void initDisplay() {
    Wire.beginTransmission(LCD_I2C_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) lcd = &lcdPrimary;
    else {
        Wire.beginTransmission(LCD_I2C_ADDR_FALLBACK);
        if (Wire.endTransmission() == 0) lcd = &lcdFallback;
    }

    if (lcd == nullptr) {
        Serial.println("[TEST] LCD not detected");
        lcdReady = false;
        return;
    }
    lcd->init();
    lcd->backlight();
    lcdReady = true;
    lcdShow("Type A Pivot", "Ready");
    Serial.println("[TEST] LCD ready");
}

void initLEDs() {
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
}

void initSoundSensor() {
    pinMode(PIN_SOUND_DOUT, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_SOUND_AOUT, ADC_11db);
    soundDoutLast = digitalRead(PIN_SOUND_DOUT);
    Serial.println("[TEST] Sound sensor pins ready");
}

void initBME() {
    if (!bme.begin(I2C_ADDR_BME688, &Wire)) {
        bmeReady = false;
        Serial.println("[TEST] BME688 not found");
        return;
    }
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    bmeReady = true;
    Serial.println("[TEST] BME688 ready");
}

void initCAP1203() {
    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_MAIN_CTRL);
    Wire.write(0x00);
    cap1203Ready = (Wire.endTransmission() == 0);
    Serial.println(cap1203Ready ? "[TEST] CAP1203 ready" : "[TEST] CAP1203 not found");
}

void initTof() {
    if (!tof.begin()) {
        tofReady = false;
        Serial.println("[TEST] VL53L5CX not found");
        return;
    }
    tof.setResolution(64);
    tof.setRangingFrequency(15);
    tof.startRanging();
    tofReady = true;
    Serial.println("[TEST] VL53L5CX ready (8x8 @ 15Hz)");
}

bool readBME(float &tempC, float &humPct, float &presHpa, float &gasOhm) {
    tempC = humPct = presHpa = gasOhm = 0.0f;
    if (!bmeReady || !bme.performReading()) return false;
    tempC = bme.temperature;
    humPct = bme.humidity;
    presHpa = bme.pressure / 100.0f;
    gasOhm = bme.gas_resistance;
    return true;
}

bool readCAPMask(uint8_t &mask) {
    mask = 0;
    if (!cap1203Ready) return false;
    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_SENSOR_INPUT);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)I2C_ADDR_CAP1203, (uint8_t)1);
    if (!Wire.available()) return false;
    mask = Wire.read();
    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();
    return true;
}

static bool tofCellOccupied(uint16_t distanceMm, uint8_t status) {
    if (distanceMm == 0 || distanceMm > TOF_OCCUPIED_MM) return false;
    return status == 5 || status == 6 || status == 9 || status == 12 || status == 13;
}

bool readTofFeatures(
    uint16_t &nearestMm,
    uint16_t &centerMinMm,
    uint8_t &occupiedCells,
    bool &centerOccupied,
    bool &leftOccupied,
    bool &rightOccupied,
    uint8_t &spanCells
) {
    nearestMm = centerMinMm = 0;
    occupiedCells = spanCells = 0;
    centerOccupied = leftOccupied = rightOccupied = false;

    if (!tofReady || !tof.isDataReady() || !tof.getRangingData(&tofResults)) {
        return false;
    }

    uint16_t nearest = 65535;
    uint16_t centerMin = 65535;
    int minCol = 8;
    int maxCol = -1;

    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            const int idx = row * 8 + col;
            const uint16_t distance = tofResults.distance_mm[idx];
            const uint8_t status = tofResults.target_status[idx];
            if (!tofCellOccupied(distance, status)) continue;
            occupiedCells++;
            if (distance < nearest) nearest = distance;
            if (col >= 2 && col <= 5 && distance < centerMin) centerMin = distance;
            if (col <= 1) leftOccupied = true;
            if (col >= 6) rightOccupied = true;
            if (col >= 2 && col <= 5) centerOccupied = true;
            if (col < minCol) minCol = col;
            if (col > maxCol) maxCol = col;
        }
    }

    if (occupiedCells == 0) return true;
    nearestMm = nearest == 65535 ? 0 : nearest;
    centerMinMm = centerMin == 65535 ? nearestMm : centerMin;
    spanCells = maxCol >= minCol ? (uint8_t)(maxCol - minCol + 1) : 0;
    return true;
}

void testI2CScan() {
    Serial.println("--- I2C SCAN ---");
    Serial.println("Expected: LCD=0x27/0x3F BME688=0x76 CAP1203=0x28 VL53L5CX=0x29");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) Serial.printf("FOUND 0x%02X\n", addr);
    }
}

void testLCDAndLEDs() {
    Serial.println("--- LCD + LED TEST ---");
    lcdShow("LED TEST", "RED -> GREEN");
    digitalWrite(PIN_LED_RED, HIGH); digitalWrite(PIN_LED_GREEN, LOW); delay(1000);
    digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_GREEN, HIGH); delay(1000);
    digitalWrite(PIN_LED_RED, HIGH); digitalWrite(PIN_LED_GREEN, HIGH); delay(1000);
    digitalWrite(PIN_LED_RED, LOW); digitalWrite(PIN_LED_GREEN, LOW);
    lcdShow("Type A Pivot", "Ready");
}

void testSoundSensor() {
    Serial.println("--- SOUND SENSOR LIVE ---");
    while (!userAbort()) {
        const int aout = analogRead(PIN_SOUND_AOUT);
        const bool dout = digitalRead(PIN_SOUND_DOUT);
        if (dout != soundDoutLast) {
            soundDoutLast = dout;
            soundEventCount++;
        }
        Serial.printf("SOUND: AOUT=%4d DOUT=%d events=%lu\n", aout, dout ? 1 : 0, soundEventCount);
        char line2[17];
        snprintf(line2, sizeof(line2), "A:%4d D:%d", aout, dout ? 1 : 0);
        lcdShow("Sound Sensor", line2);
        delay(150);
    }
    flushSerial();
    lcdShow("Type A Pivot", "Ready");
}

void testTof() {
    Serial.println("--- VL53L5CX APPROACH LIVE ---");
    Serial.println("Move in front of the sensor. Press any key to stop.");
    while (!userAbort()) {
        uint16_t nearestMm = 0, centerMinMm = 0;
        uint8_t occupiedCells = 0, spanCells = 0;
        bool centerOccupied = false, leftOccupied = false, rightOccupied = false;

        if (!readTofFeatures(nearestMm, centerMinMm, occupiedCells, centerOccupied, leftOccupied, rightOccupied, spanCells)) {
            Serial.println("TOF: waiting...");
        } else {
            const bool warning = centerOccupied && centerMinMm > 0 && centerMinMm <= TOF_WARNING_MM;
            Serial.printf(
                "TOF: near=%umm center=%umm occ=%u span=%u L=%d C=%d R=%d warning=%d\n",
                nearestMm, centerMinMm, occupiedCells, spanCells,
                leftOccupied ? 1 : 0, centerOccupied ? 1 : 0, rightOccupied ? 1 : 0,
                warning ? 1 : 0
            );
            char line1[17];
            char line2[17];
            snprintf(line1, sizeof(line1), "C:%4u O:%2u", centerMinMm, occupiedCells);
            snprintf(line2, sizeof(line2), "%s %s%s%s",
                     warning ? "STEPBACK" : "NORMAL",
                     leftOccupied ? "L" : "-",
                     centerOccupied ? "C" : "-",
                     rightOccupied ? "R" : "-");
            lcdShow(line1, line2);
        }
        delay(120);
    }
    flushSerial();
    lcdShow("Type A Pivot", "Ready");
}

void testBME() {
    Serial.println("--- BME688 LIVE ---");
    while (!userAbort()) {
        float tempC = 0.0f, humPct = 0.0f, presHpa = 0.0f, gasOhm = 0.0f;
        if (!readBME(tempC, humPct, presHpa, gasOhm)) {
            Serial.println("BME688: read failed");
        } else {
            Serial.printf("BME688: temp=%.1fC hum=%.1f%% pres=%.1fhPa gas=%.1fohm\n", tempC, humPct, presHpa, gasOhm);
            char line1[17], line2[17];
            snprintf(line1, sizeof(line1), "T:%.1f H:%2.0f%%", tempC, humPct);
            snprintf(line2, sizeof(line2), "G:%5.0f", gasOhm);
            lcdShow(line1, line2);
        }
        delay(800);
    }
    flushSerial();
    lcdShow("Type A Pivot", "Ready");
}

void testCAP1203() {
    Serial.println("--- CAP1203 RAW TOUCH ---");
    while (!userAbort()) {
        uint8_t mask = 0;
        if (!readCAPMask(mask)) {
            Serial.println("CAP1203: read failed");
        } else {
            Serial.printf("CAP1203: raw=0x%02X ch1=%d ch2=%d ch3=%d\n",
                          mask, (mask & 0x01) ? 1 : 0, (mask & 0x02) ? 1 : 0, (mask & 0x04) ? 1 : 0);
            char line2[17];
            snprintf(line2, sizeof(line2), "1:%d 2:%d 3:%d",
                     (mask & 0x01) ? 1 : 0, (mask & 0x02) ? 1 : 0, (mask & 0x04) ? 1 : 0);
            lcdShow("CAP1203", line2);
        }
        delay(150);
    }
    flushSerial();
    lcdShow("Type A Pivot", "Ready");
}

void testSummary() {
    Serial.println("--- ONE-SHOT SUMMARY ---");
    const int aout = analogRead(PIN_SOUND_AOUT);
    const bool dout = digitalRead(PIN_SOUND_DOUT);
    Serial.printf("SOUND: AOUT=%d DOUT=%d\n", aout, dout ? 1 : 0);

    uint16_t nearestMm = 0, centerMinMm = 0;
    uint8_t occupiedCells = 0, spanCells = 0;
    bool centerOccupied = false, leftOccupied = false, rightOccupied = false;
    if (readTofFeatures(nearestMm, centerMinMm, occupiedCells, centerOccupied, leftOccupied, rightOccupied, spanCells)) {
        Serial.printf("TOF: near=%umm center=%umm occ=%u span=%u L=%d C=%d R=%d\n",
                      nearestMm, centerMinMm, occupiedCells, spanCells,
                      leftOccupied ? 1 : 0, centerOccupied ? 1 : 0, rightOccupied ? 1 : 0);
    } else {
        Serial.println("TOF: no fresh reading");
    }

    float tempC = 0.0f, humPct = 0.0f, presHpa = 0.0f, gasOhm = 0.0f;
    if (readBME(tempC, humPct, presHpa, gasOhm)) {
        Serial.printf("BME688: temp=%.1fC hum=%.1f%% pres=%.1fhPa gas=%.1fohm\n", tempC, humPct, presHpa, gasOhm);
    } else {
        Serial.println("BME688: not ready");
    }

    uint8_t mask = 0;
    if (readCAPMask(mask)) {
        Serial.printf("CAP1203: raw=0x%02X ch1=%d ch2=%d ch3=%d\n",
                      mask, (mask & 0x01) ? 1 : 0, (mask & 0x02) ? 1 : 0, (mask & 0x04) ? 1 : 0);
    } else {
        Serial.println("CAP1203: not ready");
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    initLEDs();
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    initDisplay();
    initSoundSensor();
    initBME();
    initCAP1203();
    initTof();

    Serial.println();
    Serial.println("[DASN] Type A pivot hardware test ready");
    Serial.printf("[DASN] SDA/SCL=%d/%d SOUND DOUT/AOUT=%d/%d LED R/G=%d/%d\n",
                  PIN_I2C_SDA, PIN_I2C_SCL, PIN_SOUND_DOUT, PIN_SOUND_AOUT, PIN_LED_RED, PIN_LED_GREEN);
    printMenu();
}

void loop() {
    if (!Serial.available()) return;
    const char cmd = Serial.read();
    flushSerial();
    Serial.println();

    switch (cmd) {
        case '1': testI2CScan(); break;
        case '2': testLCDAndLEDs(); break;
        case '3': testSoundSensor(); break;
        case '4': testTof(); break;
        case '5': testBME(); break;
        case '6': testCAP1203(); break;
        case '7': testSummary(); break;
        default: break;
    }
    printMenu();
}
