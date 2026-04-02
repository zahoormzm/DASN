/*
 * DASN Sentinel Type A — Full Sensor Node
 * Board: ESP32 WROOM-32
 *
 * Components:
 *   RD-01 radar (UART), HC-SR501 PIR, BME688 (I2C 0x76),
 *   INMP441 mic (I2S), DS18B20 temp probe (OneWire),
 *   CAP1203 touch (I2C 0x28), NFC tag reader,
 *   Buzzer (PWM), OLED SSD1306 128x64 (I2C 0x3C)
 *
 * Sends SentinelData_t via ESP-NOW to the gateway.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <driver/i2s.h>

#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "../shared/espnow_protocol.h"
#include "../shared/config.h"

// ─── Zone identity ──────────────────────────────────────────────────────────
static const char ZONE_ID[] = "FRONT_DOOR";   // change per deployment

// ─── OLED ───────────────────────────────────────────────────────────────────
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

// ─── BME688 ─────────────────────────────────────────────────────────────────
Adafruit_BME680 bme;

// ─── DS18B20 ────────────────────────────────────────────────────────────────
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

// ─── Radar (UART) ───────────────────────────────────────────────────────────
HardwareSerial radarSerial(2);
static bool     radarPresence    = false;
static float    radarConfidence  = 0.0f;

// ─── PIR ────────────────────────────────────────────────────────────────────
static volatile bool pirTriggered = false;
static unsigned long pirLastTrigger = 0;
static void IRAM_ATTR pirISR() {
    unsigned long now = millis();
    if (now - pirLastTrigger > PIR_DEBOUNCE_MS) {
        pirTriggered = true;
        pirLastTrigger = now;
    }
}

// ─── I2S mic (INMP441) ─────────────────────────────────────────────────────
#define I2S_PORT        I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_BUF_LEN     512
static int16_t i2sBuf[I2S_BUF_LEN];

// ─── CAP1203 touch (I2C 0x28) ──────────────────────────────────────────────
#define CAP1203_MAIN_CTRL       0x00
#define CAP1203_SENSOR_INPUT    0x03
static bool systemArmed = true;

// ─── NFC ────────────────────────────────────────────────────────────────────
// Using Wire-based PN532 or similar; simplified polling approach
static bool nfcTapDetected = false;

// ─── Timing ─────────────────────────────────────────────────────────────────
static unsigned long lastSensorRead = 0;
static unsigned long lastOLEDUpdate = 0;

// ─── Sensor data ────────────────────────────────────────────────────────────
static SentinelData_t sensorData;
static esp_now_peer_info_t gatewayPeer;

// ─── Forward declarations ───────────────────────────────────────────────────
void initESPNOW();
void initI2S();
void initRadar();
void initSensors();
void readRadar();
float readMicLevel();
void readBME688();
void readCAP1203();
float readBattery();
float computeThreatScore();
void updateOLED();
void playBuzzerPattern(float threat);
void sendESPNOW();
void checkDeepSleep();

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    Serial.println("[TypeA] Sentinel Type A starting...");

    // Buzzer
    ledcSetup(BUZZER_CHANNEL, 2000, 8);
    ledcAttachPin(PIN_BUZZER, BUZZER_CHANNEL);
    ledcWriteTone(BUZZER_CHANNEL, 0);

    // PIR
    pinMode(PIN_PIR, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_PIR), pirISR, RISING);

    // I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    // OLED
    if (!oled.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_SSD1306)) {
        Serial.println("[TypeA] OLED init failed");
    } else {
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(0, 0);
        oled.println("DASN Type A");
        oled.println("Initialising...");
        oled.display();
    }

    initSensors();
    initI2S();
    initRadar();
    initESPNOW();

    // Prepare static fields
    memset(&sensorData, 0, sizeof(sensorData));
    strncpy(sensorData.zone_id, ZONE_ID, sizeof(sensorData.zone_id) - 1);
    sensorData.comm_mode = COMM_MODE_ESPNOW;

    Serial.println("[TypeA] Setup complete");
}

// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();

    if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
        lastSensorRead = now;

        // Radar
        readRadar();
        sensorData.radar_presence   = radarPresence;
        sensorData.radar_confidence = radarConfidence;

        // PIR
        sensorData.pir_triggered = pirTriggered;
        pirTriggered = false;

        // Microphone
        float dbLevel = readMicLevel();
        sensorData.sound_level_db  = dbLevel;
        sensorData.sound_transient = (dbLevel > SOUND_TRANSIENT_DB);

        // BME688
        readBME688();

        // DS18B20
        ds18b20.requestTemperatures();
        float extTemp = ds18b20.getTempCByIndex(0);
        if (extTemp != DEVICE_DISCONNECTED_C) {
            // Could be used for external temp; BME688 has onboard temp
        }

        // Battery
        sensorData.battery_v = readBattery();

        // Threat score
        sensorData.local_threat_score = computeThreatScore();
        sensorData.timestamp = now;

        // Touch / arm-disarm
        readCAP1203();

        // Send via ESP-NOW
        if (systemArmed) {
            sendESPNOW();
        }

        // Buzzer
        if (systemArmed) {
            playBuzzerPattern(sensorData.local_threat_score);
        } else {
            ledcWriteTone(BUZZER_CHANNEL, 0);
        }
    }

    // OLED refresh at 5 Hz
    if (now - lastOLEDUpdate >= 200) {
        lastOLEDUpdate = now;
        updateOLED();
    }

    // Deep sleep check
    checkDeepSleep();
}

// ─── ESP-NOW ────────────────────────────────────────────────────────────────
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[TypeA] ESP-NOW send failed");
    }
}

void initESPNOW() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TypeA] ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(onDataSent);

    memset(&gatewayPeer, 0, sizeof(gatewayPeer));
    memcpy(gatewayPeer.peer_addr, GATEWAY_MAC, 6);
    gatewayPeer.channel = ESPNOW_CHANNEL;
    gatewayPeer.encrypt = ESPNOW_ENCRYPT;

    if (esp_now_add_peer(&gatewayPeer) != ESP_OK) {
        Serial.println("[TypeA] Failed to add gateway peer");
    }
    Serial.println("[TypeA] ESP-NOW ready");
}

void sendESPNOW() {
    esp_err_t result = esp_now_send(GATEWAY_MAC,
                                    (uint8_t *)&sensorData,
                                    sizeof(SentinelData_t));
    if (result != ESP_OK) {
        Serial.printf("[TypeA] ESP-NOW send error: %d\n", result);
    }
}

// ─── I2S microphone ────────────────────────────────────────────────────────
void initI2S() {
    i2s_config_t i2sCfg = {};
    i2sCfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2sCfg.sample_rate          = I2S_SAMPLE_RATE;
    i2sCfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    i2sCfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2sCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2sCfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    i2sCfg.dma_buf_count        = 4;
    i2sCfg.dma_buf_len          = I2S_BUF_LEN;
    i2sCfg.use_apll             = false;

    i2s_driver_install(I2S_PORT, &i2sCfg, 0, NULL);

    i2s_pin_config_t pinCfg = {};
    pinCfg.bck_io_num   = PIN_I2S_SCK;
    pinCfg.ws_io_num    = PIN_I2S_WS;
    pinCfg.data_in_num  = PIN_I2S_SD;
    pinCfg.data_out_num = I2S_PIN_NO_CHANGE;

    i2s_set_pin(I2S_PORT, &pinCfg);
    Serial.println("[TypeA] I2S mic ready");
}

float readMicLevel() {
    size_t bytesRead = 0;
    i2s_read(I2S_PORT, i2sBuf, sizeof(i2sBuf), &bytesRead, pdMS_TO_TICKS(50));
    int samples = bytesRead / sizeof(int16_t);
    if (samples == 0) return 0.0f;

    // RMS calculation
    float sumSq = 0.0f;
    for (int i = 0; i < samples; i++) {
        float s = (float)i2sBuf[i];
        sumSq += s * s;
    }
    float rms = sqrtf(sumSq / (float)samples);

    // Convert to approximate dB SPL (reference: 1 count)
    float db = 20.0f * log10f(rms + 1.0f);
    return db;
}

// ─── Radar (RD-01 on UART2) ────────────────────────────────────────────────
void initRadar() {
    radarSerial.begin(256000, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);
    Serial.println("[TypeA] Radar UART ready");
}

void readRadar() {
    // RD-01 sends binary frames. Simplified: look for presence byte.
    // Frame: [header 0xAA] [type] [data...] [checksum]
    radarPresence   = false;
    radarConfidence = 0.0f;

    while (radarSerial.available() >= 4) {
        uint8_t header = radarSerial.read();
        if (header != 0xAA) continue;

        uint8_t frameType = radarSerial.read();
        uint8_t dataLen   = radarSerial.read();

        if (dataLen > 32) {
            // invalid, flush
            while (radarSerial.available()) radarSerial.read();
            break;
        }

        uint8_t frameBuf[32];
        size_t got = 0;
        unsigned long t0 = millis();
        while (got < dataLen && millis() - t0 < 50) {
            if (radarSerial.available()) {
                frameBuf[got++] = radarSerial.read();
            }
        }
        if (got < dataLen) break;

        // Read checksum byte
        if (radarSerial.available()) radarSerial.read();

        // Parse: frameType 0x01 = presence report
        if (frameType == 0x01 && dataLen >= 2) {
            radarPresence   = (frameBuf[0] != 0);
            radarConfidence = frameBuf[1] / 255.0f;
        }
    }
}

// ─── BME688 ─────────────────────────────────────────────────────────────────
void initSensors() {
    // BME688
    if (!bme.begin(I2C_ADDR_BME688, &Wire)) {
        Serial.println("[TypeA] BME688 not found");
    } else {
        bme.setTemperatureOversampling(BME680_OS_8X);
        bme.setHumidityOversampling(BME680_OS_2X);
        bme.setPressureOversampling(BME680_OS_4X);
        bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme.setGasHeater(320, 150); // 320C for 150ms
        Serial.println("[TypeA] BME688 ready");
    }

    // DS18B20
    ds18b20.begin();
    Serial.println("[TypeA] DS18B20 ready");

    // CAP1203 — enable all 3 channels
    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_MAIN_CTRL);
    Wire.write(0x00); // normal operation
    Wire.endTransmission();
    Serial.println("[TypeA] CAP1203 ready");

    // Battery ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.println("[TypeA] Sensors initialised");
}

void readBME688() {
    if (bme.performReading()) {
        sensorData.bme688_temperature    = bme.temperature;
        sensorData.bme688_humidity       = bme.humidity;
        sensorData.bme688_pressure       = bme.pressure / 100.0f; // Pa -> hPa
        sensorData.bme688_gas_resistance = bme.gas_resistance;
    }
}

// ─── CAP1203 touch ─────────────────────────────────────────────────────────
void readCAP1203() {
    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(CAP1203_SENSOR_INPUT);
    if (Wire.endTransmission(false) != 0) return;

    Wire.requestFrom((uint8_t)I2C_ADDR_CAP1203, (uint8_t)1);
    if (!Wire.available()) return;

    uint8_t touched = Wire.read();

    // Arm/disarm pattern: channels 1+3 simultaneously
    bool ch1 = touched & 0x01;
    bool ch3 = touched & 0x04;
    if (ch1 && ch3) {
        systemArmed = !systemArmed;
        Serial.printf("[TypeA] System %s\n", systemArmed ? "ARMED" : "DISARMED");
        // Short beep to confirm
        ledcWriteTone(BUZZER_CHANNEL, 1500);
        delay(100);
        ledcWriteTone(BUZZER_CHANNEL, 0);
    }

    // Clear interrupt
    Wire.beginTransmission(I2C_ADDR_CAP1203);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();
}

// ─── Battery ────────────────────────────────────────────────────────────────
float readBattery() {
    int raw = analogRead(BATTERY_ADC_PIN);
    float v = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER_RATIO;
    return v;
}

// ─── Threat score ───────────────────────────────────────────────────────────
float computeThreatScore() {
    float score = 0.0f;

    // Radar contribution (0 - 0.40)
    if (sensorData.radar_presence && sensorData.radar_confidence > RADAR_CONFIDENCE_MIN) {
        score += 0.40f * sensorData.radar_confidence;
    }

    // PIR contribution (0 or 0.25)
    if (sensorData.pir_triggered) {
        score += 0.25f;
    }

    // Sound contribution (0 - 0.25)
    if (sensorData.sound_level_db > SOUND_BASELINE_DB) {
        float soundNorm = (sensorData.sound_level_db - SOUND_BASELINE_DB) /
                          (SOUND_TRANSIENT_DB - SOUND_BASELINE_DB);
        if (soundNorm > 1.0f) soundNorm = 1.0f;
        score += 0.25f * soundNorm;
    }

    // Gas resistance anomaly (0 - 0.10)
    if (sensorData.bme688_gas_resistance > 0 &&
        sensorData.bme688_gas_resistance < GAS_RESISTANCE_ALERT) {
        score += 0.10f;
    }

    if (score > 1.0f) score = 1.0f;
    return score;
}

// ─── Buzzer ─────────────────────────────────────────────────────────────────
void playBuzzerPattern(float threat) {
    if (threat >= THREAT_CRITICAL) {
        // Continuous high tone
        ledcWriteTone(BUZZER_CHANNEL, BUZZER_FREQ_CRIT);
    } else if (threat >= THREAT_HIGH) {
        // Fast pulse
        static bool toggle = false;
        toggle = !toggle;
        ledcWriteTone(BUZZER_CHANNEL, toggle ? BUZZER_FREQ_HIGH : 0);
    } else if (threat >= THREAT_MEDIUM) {
        // Slow pulse
        static uint8_t cnt = 0;
        cnt++;
        ledcWriteTone(BUZZER_CHANNEL, (cnt % 4 == 0) ? BUZZER_FREQ_MED : 0);
    } else {
        ledcWriteTone(BUZZER_CHANNEL, 0);
    }
}

// ─── OLED ───────────────────────────────────────────────────────────────────
void updateOLED() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);

    oled.printf("Zone: %s\n", sensorData.zone_id);
    oled.printf("Armed: %s\n", systemArmed ? "YES" : "NO");
    oled.printf("Radar: %s (%.0f%%)\n",
                sensorData.radar_presence ? "DET" : "---",
                sensorData.radar_confidence * 100.0f);
    oled.printf("PIR: %s  Snd:%.0fdB\n",
                sensorData.pir_triggered ? "Y" : "N",
                sensorData.sound_level_db);
    oled.printf("T:%.1fC H:%.0f%%\n",
                sensorData.bme688_temperature,
                sensorData.bme688_humidity);
    oled.printf("Gas:%.0f  Bat:%.1fV\n",
                sensorData.bme688_gas_resistance,
                sensorData.battery_v);
    oled.printf("Threat: %.2f\n", sensorData.local_threat_score);

    // Threat bar
    int barW = (int)(sensorData.local_threat_score * SCREEN_W);
    oled.fillRect(0, 57, barW, 7, SSD1306_WHITE);

    oled.display();
}

// ─── Deep sleep ─────────────────────────────────────────────────────────────
void checkDeepSleep() {
    if (sensorData.battery_v > 0.5f && sensorData.battery_v < BATTERY_CRITICAL_V) {
        Serial.println("[TypeA] Battery critical — entering deep sleep");
        oled.clearDisplay();
        oled.setCursor(0, 20);
        oled.println("LOW BATTERY");
        oled.println("Sleeping...");
        oled.display();
        delay(1000);

        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_PIR, 1);
        esp_deep_sleep_start();
    }
}
