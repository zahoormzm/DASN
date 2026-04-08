/*
 * DASN Sentinel Type A — Pivot Runtime Node with VL53L5CX approach sensing.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include <Adafruit_BME680.h>
#include <LiquidCrystal_I2C.h>
#include <SparkFun_VL53L5CX_Library.h>

#include "espnow_protocol.h"
#include "config.h"

static const char ZONE_ID[] = "FRONT_DOOR";

LiquidCrystal_I2C lcdPrimary(LCD_I2C_ADDR_PRIMARY, 16, 2);
LiquidCrystal_I2C lcdFallback(LCD_I2C_ADDR_FALLBACK, 16, 2);
LiquidCrystal_I2C *lcd = nullptr;

Adafruit_BME680 bme;
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofResults;

static bool lcdReady = false;
static bool bmeReady = false;
static bool cap1203Ready = false;
static bool tofReady = false;

static WiFiUDP udpClient;
static WiFiUDP udpControlServer;
static const IPAddress WIFI_BROADCAST_IP(255, 255, 255, 255);

static SentinelData_t sensorData;
static bool systemArmed = true;
static uint8_t lastCapTouchMask = 0;
static bool lastComboPressed = false;
static unsigned long lastSensorReadMs = 0;
static unsigned long lastDisplayUpdateMs = 0;
static unsigned long lastWiFiRetryMs = 0;
static unsigned long lastRemoteControlMs = 0;
static uint8_t lcdPage = 0;

static int soundRaw = 0;
static bool soundTriggered = false;

static uint16_t tofNearestMm = 0;
static uint16_t tofCenterMinMm = 0;
static uint8_t tofOccupiedCells = 0;
static uint8_t tofSpanCells = 0;
static bool tofCenterOccupied = false;
static bool tofLeftOccupied = false;
static bool tofRightOccupied = false;
static int tofApproachSpeedMmS = 0;
static float tofApproachScore = 0.0f;
static bool tofStepBackWarning = false;
static uint16_t prevTofCenterMinMm = 0;
static unsigned long prevTofMillis = 0;

static char authState[8] = "ARMED";
static char remotePhase[16] = "ARMED";
static char remoteLedMode[16] = "armed";
static char remoteLcdLine1[17] = "";
static char remoteLcdLine2[17] = "";

void initDisplay();
void initSensors();
void initWiFiTransport();
bool ensureWiFiConnected();
void checkWiFiControl();

void readSoundSensor();
void readBME688();
void readCAP1203();
void readTofFeatures();
void updateDisplay();
void updateStatusLEDs();
float computeLocalThreatScore();
bool sendWiFiJSON();

bool remoteControlActive();
void copyStringToBuffer(const String &src, char *dst, size_t dstSize);
String jsonStringValue(const String &json, const char *key, const String &fallback);
bool jsonBoolValue(const String &json, const char *key, bool fallback);

void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    Serial.println("[TypeA] Pivot runtime starting...");

    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);

    pinMode(PIN_SOUND_DOUT, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_SOUND_AOUT, ADC_11db);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    initDisplay();
    initSensors();
    initWiFiTransport();

    memset(&sensorData, 0, sizeof(sensorData));
    strncpy(sensorData.zone_id, ZONE_ID, sizeof(sensorData.zone_id) - 1);
    sensorData.comm_mode = COMM_MODE_WIFI;

    Serial.println("[TypeA] Setup complete");
}

void loop() {
    const unsigned long now = millis();

    checkWiFiControl();

    if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
        lastSensorReadMs = now;

        readSoundSensor();
        readTofFeatures();
        readBME688();
        readCAP1203();

        sensorData.timestamp = now;
        sensorData.radar_presence = tofCenterOccupied;
        sensorData.radar_confidence = tofApproachScore;
        sensorData.pir_triggered = false;
        sensorData.sound_transient = soundTriggered;
        sensorData.local_threat_score = computeLocalThreatScore();
        sensorData.comm_mode = COMM_MODE_WIFI;
        sensorData.battery_v = 0.0f;

        sendWiFiJSON();
    }

    if (now - lastDisplayUpdateMs >= DISPLAY_UPDATE_INTERVAL_MS) {
        lastDisplayUpdateMs = now;
        updateDisplay();
    }

    updateStatusLEDs();
}

void initDisplay() {
    Wire.beginTransmission(LCD_I2C_ADDR_PRIMARY);
    if (Wire.endTransmission() == 0) {
        lcd = &lcdPrimary;
    } else {
        Wire.beginTransmission(LCD_I2C_ADDR_FALLBACK);
        if (Wire.endTransmission() == 0) {
            lcd = &lcdFallback;
        }
    }

    if (lcd == nullptr) {
        lcdReady = false;
        Serial.println("[TypeA] LCD not detected");
        return;
    }

    lcd->init();
    lcd->backlight();
    lcdReady = true;
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("DASN Type A");
    lcd->setCursor(0, 1);
    lcd->print("Booting...");
    Serial.println("[TypeA] LCD ready");
}

void initSensors() {
    if (!bme.begin(I2C_ADDR_BME688, &Wire)) {
        bmeReady = false;
        Serial.println("[TypeA] BME688 not found");
    } else {
        bmeReady = true;
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150);
        Serial.println("[TypeA] BME688 ready");
    }

    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_MAIN_CTRL);
    Wire.write(0x00);
    cap1203Ready = (Wire.endTransmission() == 0);
    Serial.println(cap1203Ready ? "[TypeA] CAP1203 ready" : "[TypeA] CAP1203 not found");

    if (tof.begin()) {
        tof.setResolution(64);
        tof.setRangingFrequency(15);
        tof.startRanging();
        tofReady = true;
        Serial.println("[TypeA] VL53L5CX ready (8x8 @ 15Hz)");
    } else {
        tofReady = false;
        Serial.println("[TypeA] VL53L5CX not found");
    }
}

void readSoundSensor() {
    soundRaw = analogRead(PIN_SOUND_AOUT);
    soundTriggered = digitalRead(PIN_SOUND_DOUT) == HIGH;
    sensorData.sound_level_db =
        (constrain(soundRaw, 0, (int)SOUND_ANALOG_FULL_SCALE) / SOUND_ANALOG_FULL_SCALE) *
        SOUND_DB_FULL_SCALE;
}

void readBME688() {
    if (!bmeReady) {
        sensorData.bme688_temperature = 0.0f;
        sensorData.bme688_humidity = 0.0f;
        sensorData.bme688_pressure = 0.0f;
        sensorData.bme688_gas_resistance = 0.0f;
        return;
    }

    if (bme.performReading()) {
        sensorData.bme688_temperature = bme.temperature;
        sensorData.bme688_humidity = bme.humidity;
        sensorData.bme688_pressure = bme.pressure / 100.0f;
        sensorData.bme688_gas_resistance = bme.gas_resistance;
    }
}

void readCAP1203() {
    if (!cap1203Ready) {
        return;
    }

    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_SENSOR_INPUT);
    if (Wire.endTransmission(false) != 0) {
        return;
    }

    Wire.requestFrom((uint8_t)I2C_ADDR_CAP1203, (uint8_t)1);
    if (!Wire.available()) {
        return;
    }

    const uint8_t touched = Wire.read();
    lastCapTouchMask = touched;
    const bool comboPressed = (touched & 0x01) && (touched & 0x04);

    if (comboPressed && !lastComboPressed && !remoteControlActive()) {
        systemArmed = !systemArmed;
        strncpy(authState, systemArmed ? "ARMED" : "SAFE", sizeof(authState) - 1);
        authState[sizeof(authState) - 1] = '\0';
        Serial.printf("[TypeA] Local arm toggle -> %s\n", systemArmed ? "ARMED" : "DISARMED");
    }
    lastComboPressed = comboPressed;

    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_MAIN_CTRL);
    Wire.write(0x00);
    Wire.endTransmission();
}

static bool tofCellOccupied(uint16_t distanceMm, uint8_t status) {
    if (distanceMm == 0 || distanceMm > TOF_OCCUPIED_MM) {
        return false;
    }
    return status == 5 || status == 6 || status == 9 || status == 12 || status == 13;
}

void readTofFeatures() {
    tofNearestMm = 0;
    tofCenterMinMm = 0;
    tofOccupiedCells = 0;
    tofSpanCells = 0;
    tofCenterOccupied = false;
    tofLeftOccupied = false;
    tofRightOccupied = false;
    tofApproachSpeedMmS = 0;
    tofApproachScore = 0.0f;
    tofStepBackWarning = false;

    if (!tofReady || !tof.isDataReady()) {
        return;
    }
    if (!tof.getRangingData(&tofResults)) {
        return;
    }

    int minCol = 8;
    int maxCol = -1;
    uint16_t nearest = 65535;
    uint16_t centerMin = 65535;

    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            const int idx = row * 8 + col;
            const uint16_t distance = tofResults.distance_mm[idx];
            const uint8_t status = tofResults.target_status[idx];
            if (!tofCellOccupied(distance, status)) {
                continue;
            }

            tofOccupiedCells++;
            if (distance < nearest) {
                nearest = distance;
            }
            if (col >= 2 && col <= 5 && distance < centerMin) {
                centerMin = distance;
            }
            if (col <= 1) {
                tofLeftOccupied = true;
            }
            if (col >= 6) {
                tofRightOccupied = true;
            }
            if (col >= 2 && col <= 5) {
                tofCenterOccupied = true;
            }
            if (col < minCol) minCol = col;
            if (col > maxCol) maxCol = col;
        }
    }

    if (tofOccupiedCells == 0) {
        prevTofCenterMinMm = 0;
        prevTofMillis = millis();
        return;
    }

    tofNearestMm = (nearest == 65535) ? 0 : nearest;
    tofCenterMinMm = (centerMin == 65535) ? tofNearestMm : centerMin;
    tofSpanCells = (maxCol >= minCol) ? (uint8_t)(maxCol - minCol + 1) : 0;

    const unsigned long now = millis();
    if (prevTofMillis > 0 && prevTofCenterMinMm > 0 && tofCenterMinMm > 0 && now > prevTofMillis) {
        const float dt = (now - prevTofMillis) / 1000.0f;
        tofApproachSpeedMmS = (int)((prevTofCenterMinMm - tofCenterMinMm) / dt);
    }
    prevTofCenterMinMm = tofCenterMinMm;
    prevTofMillis = now;

    float score = 0.0f;
    if (tofCenterOccupied) score += 0.35f;
    score += min(0.35f, tofOccupiedCells / 16.0f);
    if (tofCenterMinMm > 0) {
        const float nearFactor = 1.0f - min((float)tofCenterMinMm / (float)TOF_OCCUPIED_MM, 1.0f);
        score += 0.20f * nearFactor;
    }
    if (tofApproachSpeedMmS > 150) {
        score += min(0.20f, tofApproachSpeedMmS / 2000.0f);
    }
    if (score > 1.0f) score = 1.0f;
    tofApproachScore = score;

    if (tofCenterOccupied && tofCenterMinMm > 0 && tofCenterMinMm <= TOF_WARNING_MM) {
        tofStepBackWarning = true;
    }
}

float computeLocalThreatScore() {
    float score = 0.0f;
    score += 0.55f * tofApproachScore;

    if (sensorData.sound_level_db > SOUND_BASELINE_DB) {
        float soundNorm =
            (sensorData.sound_level_db - SOUND_BASELINE_DB) /
            (SOUND_TRANSIENT_DB - SOUND_BASELINE_DB);
        if (soundNorm > 1.0f) soundNorm = 1.0f;
        if (soundNorm < 0.0f) soundNorm = 0.0f;
        score += 0.25f * soundNorm;
    }

    if (soundTriggered) {
        score += 0.10f;
    }

    if (!systemArmed) {
        score *= 0.25f;
    }

    if (score > 1.0f) score = 1.0f;
    return score;
}

void updateDisplay() {
    if (!lcdReady || lcd == nullptr) {
        return;
    }

    lcd->clear();

    if (remoteControlActive()) {
        lcd->setCursor(0, 0);
        lcd->print(remoteLcdLine1);
        lcd->setCursor(0, 1);
        lcd->print(remoteLcdLine2);
        return;
    }

    if (!systemArmed) {
        lcd->setCursor(0, 0);
        lcd->print("ALARM DISABLED");
        lcd->setCursor(0, 1);
        lcd->print("T:");
        lcd->print(sensorData.bme688_temperature, 1);
        lcd->print(" H:");
        lcd->print((int)sensorData.bme688_humidity);
        return;
    }

    if (tofStepBackWarning) {
        lcd->setCursor(0, 0);
        lcd->print("STEP BACK");
        lcd->setCursor(0, 1);
        lcd->print("TOO CLOSE");
        return;
    }

    if (lcdPage == 0) {
        lcd->setCursor(0, 0);
        lcd->print("APP:");
        lcd->print((int)(tofApproachScore * 100.0f));
        lcd->print("% S:");
        lcd->print((int)sensorData.sound_level_db);
        lcd->setCursor(0, 1);
        lcd->print("C:");
        lcd->print(tofCenterOccupied ? "Y " : "N ");
        lcd->print("D:");
        lcd->print((int)tofCenterMinMm);
        lcd->print("mm");
    } else {
        lcd->setCursor(0, 0);
        lcd->print("O:");
        lcd->print((int)tofOccupiedCells);
        lcd->print(" W:");
        lcd->print((int)tofSpanCells);
        lcd->print(" V:");
        lcd->print(tofApproachSpeedMmS);
        lcd->setCursor(0, 1);
        lcd->print("L:");
        lcd->print(tofLeftOccupied ? "1" : "0");
        lcd->print(" C:");
        lcd->print(tofCenterOccupied ? "1" : "0");
        lcd->print(" R:");
        lcd->print(tofRightOccupied ? "1" : "0");
    }

    lcdPage ^= 1;
}

void updateStatusLEDs() {
    const unsigned long now = millis();
    bool redOn = false;
    bool greenOn = false;

    if (remoteControlActive()) {
        if (strcmp(remoteLedMode, "alarm") == 0) {
            redOn = ((now / 150UL) % 2UL) == 0;
        } else if (strcmp(remoteLedMode, "countdown") == 0) {
            redOn = ((now / 250UL) % 2UL) == 0;
            greenOn = !redOn;
        } else if (strcmp(remoteLedMode, "warning") == 0) {
            redOn = ((now / 180UL) % 2UL) == 0;
            greenOn = ((now / 180UL) % 2UL) != 0;
        } else if (strcmp(remoteLedMode, "disarmed") == 0) {
            greenOn = ((now / 800UL) % 2UL) == 0;
        } else {
            greenOn = true;
        }
    } else if (systemArmed) {
        if (tofStepBackWarning || sensorData.local_threat_score >= 0.75f) {
            redOn = ((now / 150UL) % 2UL) == 0;
        } else if (sensorData.local_threat_score >= 0.40f) {
            redOn = ((now / 300UL) % 2UL) == 0;
        } else {
            greenOn = true;
        }
    } else {
        greenOn = ((now / 800UL) % 2UL) == 0;
    }

    digitalWrite(PIN_LED_RED, redOn ? HIGH : LOW);
    digitalWrite(PIN_LED_GREEN, greenOn ? HIGH : LOW);
}

void initWiFiTransport() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

    Serial.printf("[TypeA] Connecting Wi-Fi SSID: %s", WIFI_STA_SSID);
    const unsigned long started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        udpClient.begin(WIFI_UDP_PORT);
        udpControlServer.begin(WIFI_CONTROL_PORT);
        Serial.printf("[TypeA] Wi-Fi connected. IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[TypeA] Wi-Fi connect failed");
    }

    lastWiFiRetryMs = millis();
}

bool ensureWiFiConnected() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    const unsigned long now = millis();
    if (now - lastWiFiRetryMs < WIFI_RETRY_INTERVAL_MS) {
        return false;
    }
    lastWiFiRetryMs = now;

    WiFi.disconnect();
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

    const unsigned long started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
        udpClient.begin(WIFI_UDP_PORT);
        udpControlServer.begin(WIFI_CONTROL_PORT);
        Serial.printf("[TypeA] Wi-Fi reconnected. IP: %s\n", WiFi.localIP().toString().c_str());
        return true;
    }

    Serial.println("[TypeA] Wi-Fi reconnect failed");
    return false;
}

bool remoteControlActive() {
    return (millis() - lastRemoteControlMs) < REMOTE_CONTROL_HOLD_MS;
}

void copyStringToBuffer(const String &src, char *dst, size_t dstSize) {
    if (dstSize == 0) return;
    size_t copyLen = src.length();
    if (copyLen >= dstSize) copyLen = dstSize - 1;
    memcpy(dst, src.c_str(), copyLen);
    dst[copyLen] = '\0';
}

String jsonStringValue(const String &json, const char *key, const String &fallback) {
    const String pattern = String("\"") + key + "\":\"";
    const int start = json.indexOf(pattern);
    if (start < 0) return fallback;
    const int valueStart = start + pattern.length();
    const int valueEnd = json.indexOf('"', valueStart);
    if (valueEnd < 0) return fallback;
    return json.substring(valueStart, valueEnd);
}

bool jsonBoolValue(const String &json, const char *key, bool fallback) {
    const String pattern = String("\"") + key + "\":";
    const int start = json.indexOf(pattern);
    if (start < 0) return fallback;
    const int valueStart = start + pattern.length();
    if (json.startsWith("true", valueStart)) return true;
    if (json.startsWith("false", valueStart)) return false;
    return fallback;
}

void checkWiFiControl() {
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    const int packetSize = udpControlServer.parsePacket();
    if (packetSize <= 0) return;

    char packet[384];
    const int len = udpControlServer.read(packet, sizeof(packet) - 1);
    if (len <= 0) return;
    packet[len] = '\0';

    const String payload(packet);
    lastRemoteControlMs = millis();
    systemArmed = jsonBoolValue(payload, "armed", systemArmed);
    copyStringToBuffer(jsonStringValue(payload, "phase", String(remotePhase)), remotePhase, sizeof(remotePhase));
    copyStringToBuffer(jsonStringValue(payload, "led_mode", String(remoteLedMode)), remoteLedMode, sizeof(remoteLedMode));
    copyStringToBuffer(jsonStringValue(payload, "lcd1", String(remoteLcdLine1)), remoteLcdLine1, sizeof(remoteLcdLine1));
    copyStringToBuffer(jsonStringValue(payload, "lcd2", String(remoteLcdLine2)), remoteLcdLine2, sizeof(remoteLcdLine2));

    strncpy(authState, systemArmed ? "ARMED" : "SAFE", sizeof(authState) - 1);
    authState[sizeof(authState) - 1] = '\0';
}

bool sendWiFiJSON() {
    if (!ensureWiFiConnected()) {
        return false;
    }

    char payload[768];
    const int written = snprintf(
        payload,
        sizeof(payload),
        "{\"zone_id\":\"%s\",\"radar\":%s,\"radar_conf\":%.2f,"
        "\"pir\":false,\"sound_db\":%.1f,\"sound_trans\":%s,\"sound_raw\":%d,"
        "\"tof_mm\":%u,\"tof_nearest_mm\":%u,\"tof_center_min_mm\":%u,"
        "\"tof_occupied_cells\":%u,\"tof_span_cells\":%u,"
        "\"tof_center_occupied\":%s,\"tof_left_occupied\":%s,\"tof_right_occupied\":%s,"
        "\"tof_approach_speed_mm_s\":%d,\"tof_approach_score\":%.2f,"
        "\"tof_warning\":%s,"
        "\"gas_r\":%.1f,\"temp\":%.1f,\"hum\":%.1f,\"pres\":%.1f,"
        "\"bat\":%.1f,\"threat\":%.2f,\"comm_mode\":\"wifi\","
        "\"armed\":%s,\"cap_mask\":%u,\"auth_state\":\"%s\"}",
        sensorData.zone_id,
        tofCenterOccupied ? "true" : "false",
        tofApproachScore,
        sensorData.sound_level_db,
        sensorData.sound_transient ? "true" : "false",
        soundRaw,
        (unsigned int)tofCenterMinMm,
        (unsigned int)tofNearestMm,
        (unsigned int)tofCenterMinMm,
        (unsigned int)tofOccupiedCells,
        (unsigned int)tofSpanCells,
        tofCenterOccupied ? "true" : "false",
        tofLeftOccupied ? "true" : "false",
        tofRightOccupied ? "true" : "false",
        tofApproachSpeedMmS,
        tofApproachScore,
        tofStepBackWarning ? "true" : "false",
        sensorData.bme688_gas_resistance,
        sensorData.bme688_temperature,
        sensorData.bme688_humidity,
        sensorData.bme688_pressure,
        sensorData.battery_v,
        sensorData.local_threat_score,
        systemArmed ? "true" : "false",
        (unsigned int)lastCapTouchMask,
        authState
    );

    if (written <= 0 || written >= (int)sizeof(payload)) {
        Serial.println("[TypeA] Wi-Fi payload formatting failed");
        return false;
    }

    if (!udpClient.beginPacket(WIFI_BROADCAST_IP, WIFI_UDP_PORT)) {
        return false;
    }
    udpClient.write((const uint8_t *)payload, written);
    return udpClient.endPacket();
}
