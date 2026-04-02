/*
 * DASN Sentinel Type C — Secondary Sensor Node
 * Board: ESP32 WROOM-32
 *
 * Components:
 *   RD-01 radar (UART), STTS22H temperature sensor (I2C),
 *   Analog sound sensor, Passive buzzer (PWM)
 *
 * Simpler variant of Type A. Reads radar presence, analog sound
 * threshold, and temperature. Computes local_threat_score and
 * sends SentinelData_t via ESP-NOW to the gateway.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

#include "../shared/espnow_protocol.h"
#include "../shared/config.h"

// ─── Zone identity ──────────────────────────────────────────────────────────
#define ZONE_ID_STR  "SIDE_GATE"   // change per deployment

// ─── STTS22H I2C (temperature sensor) ──────────────────────────────────────
// Default address 0x3C conflicts with OLED; Type C has no OLED so this is fine.
// If address differs on your board, update in config.h (I2C_ADDR_STTS22H).
#define STTS22H_ADDR        I2C_ADDR_STTS22H
#define STTS22H_REG_STATUS  0x05
#define STTS22H_REG_TEMP_L  0x06
#define STTS22H_REG_TEMP_H  0x07
#define STTS22H_REG_CTRL    0x04

// ─── Analog sound sensor ───────────────────────────────────────────────────
#define PIN_SOUND_ANALOG    34      // ADC1_CH6

// ─── Radar (RD-01 on UART2) ────────────────────────────────────────────────
HardwareSerial radarSerial(2);
static bool    radarPresence   = false;
static float   radarConfidence = 0.0f;

// ─── Timing ─────────────────────────────────────────────────────────────────
static unsigned long lastSensorRead = 0;

// ─── Sensor data ────────────────────────────────────────────────────────────
static SentinelData_t sensorData;
static esp_now_peer_info_t gatewayPeer;

// ─── Forward declarations ───────────────────────────────────────────────────
void initESPNOW();
void initRadar();
void initSTTS22H();
void readRadar();
float readSTTS22H();
float readSoundLevel();
float readBattery();
float computeThreatScore();
void playBuzzerPattern(float threat);
void sendESPNOW();
void checkDeepSleep();

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    Serial.println("[TypeC] Sentinel Type C starting...");

    // Buzzer
    ledcSetup(BUZZER_CHANNEL, 2000, 8);
    ledcAttachPin(PIN_BUZZER, BUZZER_CHANNEL);
    ledcWriteTone(BUZZER_CHANNEL, 0);

    // I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    // Sensors
    initSTTS22H();
    initRadar();

    // Battery ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Sound sensor ADC pin
    pinMode(PIN_SOUND_ANALOG, INPUT);

    // ESP-NOW
    initESPNOW();

    // Prepare static fields
    memset(&sensorData, 0, sizeof(sensorData));
    strncpy(sensorData.zone_id, ZONE_ID_STR, sizeof(sensorData.zone_id) - 1);
    sensorData.comm_mode = COMM_MODE_ESPNOW;

    Serial.println("[TypeC] Setup complete");
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

        // No PIR on Type C
        sensorData.pir_triggered = false;

        // Analog sound sensor
        float soundDb = readSoundLevel();
        sensorData.sound_level_db  = soundDb;
        sensorData.sound_transient = (soundDb > SOUND_TRANSIENT_DB);

        // Temperature (STTS22H)
        float temp = readSTTS22H();
        sensorData.bme688_temperature = temp;
        // Type C has no BME688; zero out gas/humidity/pressure
        sensorData.bme688_humidity       = 0.0f;
        sensorData.bme688_pressure       = 0.0f;
        sensorData.bme688_gas_resistance = 0.0f;

        // Battery
        sensorData.battery_v = readBattery();

        // Threat score
        sensorData.local_threat_score = computeThreatScore();
        sensorData.timestamp = now;

        // Send via ESP-NOW
        sendESPNOW();

        // Buzzer feedback
        playBuzzerPattern(sensorData.local_threat_score);

        // Debug output
        Serial.printf("[TypeC] radar=%d conf=%.2f snd=%.1fdB temp=%.1fC bat=%.2fV threat=%.2f\n",
                      sensorData.radar_presence,
                      sensorData.radar_confidence,
                      sensorData.sound_level_db,
                      sensorData.bme688_temperature,
                      sensorData.battery_v,
                      sensorData.local_threat_score);
    }

    // Deep sleep check
    checkDeepSleep();
}

// ─── ESP-NOW ────────────────────────────────────────────────────────────────
static void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("[TypeC] ESP-NOW send failed");
    }
}

void initESPNOW() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TypeC] ESP-NOW init failed");
        return;
    }
    esp_now_register_send_cb(onDataSent);

    memset(&gatewayPeer, 0, sizeof(gatewayPeer));
    memcpy(gatewayPeer.peer_addr, GATEWAY_MAC, 6);
    gatewayPeer.channel = ESPNOW_CHANNEL;
    gatewayPeer.encrypt = ESPNOW_ENCRYPT;

    if (esp_now_add_peer(&gatewayPeer) != ESP_OK) {
        Serial.println("[TypeC] Failed to add gateway peer");
    }
    Serial.println("[TypeC] ESP-NOW ready");
}

void sendESPNOW() {
    esp_err_t result = esp_now_send(GATEWAY_MAC,
                                    (uint8_t *)&sensorData,
                                    sizeof(SentinelData_t));
    if (result != ESP_OK) {
        Serial.printf("[TypeC] ESP-NOW send error: %d\n", result);
    }
}

// ─── Radar (RD-01 on UART2) ────────────────────────────────────────────────
void initRadar() {
    radarSerial.begin(256000, SERIAL_8N1, PIN_RADAR_RX, PIN_RADAR_TX);
    Serial.println("[TypeC] Radar UART ready");
}

void readRadar() {
    // RD-01 sends binary frames: [0xAA] [type] [len] [data...] [checksum]
    radarPresence   = false;
    radarConfidence = 0.0f;

    while (radarSerial.available() >= 4) {
        uint8_t header = radarSerial.read();
        if (header != 0xAA) continue;

        uint8_t frameType = radarSerial.read();
        uint8_t dataLen   = radarSerial.read();

        if (dataLen > 32) {
            // Invalid frame, flush buffer
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

        // frameType 0x01 = presence report
        if (frameType == 0x01 && dataLen >= 2) {
            radarPresence   = (frameBuf[0] != 0);
            radarConfidence = frameBuf[1] / 255.0f;
        }
    }
}

// ─── STTS22H temperature sensor ─────────────────────────────────────────────
void initSTTS22H() {
    // Enable continuous mode: write 0x04 to CTRL register
    // Bit 2 = IF_ADD_INC (auto-increment), Bit 0 = ONE_SHOT in freerun
    // 0x0C = freerun mode at 1 Hz, BDU enabled
    Wire.beginTransmission(STTS22H_ADDR);
    Wire.write(STTS22H_REG_CTRL);
    Wire.write(0x0C);  // freerun mode, BDU enabled
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("[TypeC] STTS22H init failed (err=%d)\n", err);
    } else {
        Serial.println("[TypeC] STTS22H ready");
    }
}

float readSTTS22H() {
    // Read status to check data ready
    Wire.beginTransmission(STTS22H_ADDR);
    Wire.write(STTS22H_REG_STATUS);
    if (Wire.endTransmission(false) != 0) return -999.0f;

    Wire.requestFrom((uint8_t)STTS22H_ADDR, (uint8_t)1);
    if (!Wire.available()) return -999.0f;
    uint8_t status = Wire.read();

    // Bit 0 = BUSY (data not ready)
    if (status & 0x01) return -999.0f;

    // Read temperature registers (16-bit, little-endian)
    Wire.beginTransmission(STTS22H_ADDR);
    Wire.write(STTS22H_REG_TEMP_L);
    if (Wire.endTransmission(false) != 0) return -999.0f;

    Wire.requestFrom((uint8_t)STTS22H_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return -999.0f;

    uint8_t tempL = Wire.read();
    uint8_t tempH = Wire.read();

    int16_t rawTemp = (int16_t)((tempH << 8) | tempL);
    // STTS22H resolution: 0.01 C per LSB
    float temperature = rawTemp * 0.01f;

    return temperature;
}

// ─── Analog sound sensor ───────────────────────────────────────────────────
float readSoundLevel() {
    // Take multiple samples for a rough RMS estimate
    const int numSamples = 64;
    float sumSq = 0.0f;

    for (int i = 0; i < numSamples; i++) {
        int raw = analogRead(PIN_SOUND_ANALOG);
        // Centre around midpoint (2048 for 12-bit ADC with biased output)
        float sample = (float)(raw - 2048);
        sumSq += sample * sample;
    }

    float rms = sqrtf(sumSq / (float)numSamples);

    // Convert to approximate dB (sensor-dependent scaling)
    // Map ADC RMS range [0..2048] to roughly [30..100] dB
    float db = 20.0f * log10f(rms + 1.0f) + 30.0f;
    if (db < 30.0f) db = 30.0f;
    if (db > 120.0f) db = 120.0f;

    return db;
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

    // Radar contribution (0 - 0.50)
    if (sensorData.radar_presence && sensorData.radar_confidence > RADAR_CONFIDENCE_MIN) {
        score += 0.50f * sensorData.radar_confidence;
    }

    // Sound contribution (0 - 0.40)
    if (sensorData.sound_level_db > SOUND_BASELINE_DB) {
        float soundNorm = (sensorData.sound_level_db - SOUND_BASELINE_DB) /
                          (SOUND_TRANSIENT_DB - SOUND_BASELINE_DB);
        if (soundNorm > 1.0f) soundNorm = 1.0f;
        score += 0.40f * soundNorm;
    }

    // Sound transient bonus (0.10)
    if (sensorData.sound_transient) {
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
    } else if (threat >= THREAT_LOW) {
        // Single short beep every few cycles
        static uint8_t cnt2 = 0;
        cnt2++;
        ledcWriteTone(BUZZER_CHANNEL, (cnt2 % 8 == 0) ? BUZZER_FREQ_LOW : 0);
    } else {
        ledcWriteTone(BUZZER_CHANNEL, 0);
    }
}

// ─── Deep sleep ─────────────────────────────────────────────────────────────
void checkDeepSleep() {
    if (sensorData.battery_v > 0.5f && sensorData.battery_v < BATTERY_CRITICAL_V) {
        Serial.println("[TypeC] Battery critical - entering deep sleep");
        ledcWriteTone(BUZZER_CHANNEL, 500);
        delay(500);
        ledcWriteTone(BUZZER_CHANNEL, 0);
        delay(200);

        esp_sleep_enable_timer_wakeup(DEEP_SLEEP_US);
        esp_deep_sleep_start();
    }
}
