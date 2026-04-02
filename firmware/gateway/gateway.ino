/*
 * DASN Gateway Node
 * Board: ESP32 WROOM-32
 *
 * Receives ESP-NOW packets from all sentinel nodes, converts
 * SentinelData_t to JSON, and forwards over USB Serial at 115200 baud.
 *
 * Libraries: ArduinoJson
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

#include "../shared/espnow_protocol.h"
#include "../shared/config.h"

// ─── Status LED ─────────────────────────────────────────────────────────────
#define PIN_STATUS_LED  2   // onboard LED on most ESP32 dev boards

// ─── Ring buffer for received packets ───────────────────────────────────────
#define RX_BUFFER_SIZE  16

static SentinelData_t rxBuffer[RX_BUFFER_SIZE];
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;

// ─── Per-zone tracking ──────────────────────────────────────────────────────
#define MAX_ZONES  16

struct ZoneTracker {
    char     zone_id[16];
    uint32_t lastHeardMs;    // millis() when last packet arrived
    bool     active;
};

static ZoneTracker zones[MAX_ZONES];
static uint8_t zoneCount = 0;

// ─── LED blink state ────────────────────────────────────────────────────────
static unsigned long ledOnUntil = 0;

// ─── Forward declarations ───────────────────────────────────────────────────
void initESPNOW();
void processPacket(const SentinelData_t &pkt);
void updateZoneTracker(const char *zoneId);
void sendJSON(const SentinelData_t &pkt);
void blinkLED();
void printZoneStatus();

// ─── ESP-NOW receive callback ───────────────────────────────────────────────
static void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(SentinelData_t)) {
        Serial.printf("[GW] Bad packet size: %d (expected %d)\n",
                      len, (int)sizeof(SentinelData_t));
        return;
    }

    // Push into ring buffer (drop oldest if full)
    uint8_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
    memcpy(&rxBuffer[rxHead], data, sizeof(SentinelData_t));
    rxHead = nextHead;

    // If buffer was full, advance tail (drop oldest)
    if (rxHead == rxTail) {
        rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    Serial.println("[GW] DASN Gateway starting...");

    // Status LED
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    // Initialise zone tracker
    memset(zones, 0, sizeof(zones));

    // ESP-NOW
    initESPNOW();

    Serial.println("[GW] Gateway ready — waiting for sentinel packets");
}

// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    // Process all pending packets from ring buffer
    while (rxTail != rxHead) {
        SentinelData_t pkt;
        memcpy(&pkt, &rxBuffer[rxTail], sizeof(SentinelData_t));
        rxTail = (rxTail + 1) % RX_BUFFER_SIZE;

        processPacket(pkt);
    }

    // LED auto-off
    if (ledOnUntil > 0 && millis() > ledOnUntil) {
        digitalWrite(PIN_STATUS_LED, LOW);
        ledOnUntil = 0;
    }

    // Periodic zone status report (every 10 seconds)
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint >= 10000) {
        lastStatusPrint = millis();
        printZoneStatus();
    }
}

// ─── ESP-NOW init ───────────────────────────────────────────────────────────
void initESPNOW() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Print own MAC so sentinels can target this gateway
    Serial.printf("[GW] MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[GW] ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    Serial.println("[GW] ESP-NOW ready (receive mode)");
}

// ─── Process a single received packet ───────────────────────────────────────
void processPacket(const SentinelData_t &pkt) {
    // Update zone tracker
    updateZoneTracker(pkt.zone_id);

    // Convert to JSON and print on Serial
    sendJSON(pkt);

    // Blink LED
    blinkLED();
}

// ─── Zone tracker ───────────────────────────────────────────────────────────
void updateZoneTracker(const char *zoneId) {
    unsigned long now = millis();

    // Look for existing zone
    for (uint8_t i = 0; i < zoneCount; i++) {
        if (strncmp(zones[i].zone_id, zoneId, 15) == 0) {
            zones[i].lastHeardMs = now;
            zones[i].active = true;
            return;
        }
    }

    // New zone — add if space available
    if (zoneCount < MAX_ZONES) {
        strncpy(zones[zoneCount].zone_id, zoneId, 15);
        zones[zoneCount].zone_id[15] = '\0';
        zones[zoneCount].lastHeardMs = now;
        zones[zoneCount].active = true;
        zoneCount++;
        Serial.printf("[GW] New zone registered: %s (total: %d)\n", zoneId, zoneCount);
    } else {
        Serial.println("[GW] Zone table full — cannot register new zone");
    }
}

// ─── JSON serialisation and output ──────────────────────────────────────────
void sendJSON(const SentinelData_t &pkt) {
    StaticJsonDocument<384> doc;

    // Null-terminate zone_id safely
    char zoneStr[17];
    memcpy(zoneStr, pkt.zone_id, 16);
    zoneStr[16] = '\0';

    doc["zone_id"]    = zoneStr;
    doc["radar"]      = pkt.radar_presence;
    doc["radar_conf"] = serialized(String(pkt.radar_confidence, 2));
    doc["pir"]        = pkt.pir_triggered;
    doc["sound_db"]   = serialized(String(pkt.sound_level_db, 1));
    doc["sound_trans"] = pkt.sound_transient;
    doc["gas_r"]      = serialized(String(pkt.bme688_gas_resistance, 1));
    doc["temp"]       = serialized(String(pkt.bme688_temperature, 1));
    doc["hum"]        = serialized(String(pkt.bme688_humidity, 1));
    doc["pres"]       = serialized(String(pkt.bme688_pressure, 1));
    doc["bat"]        = serialized(String(pkt.battery_v, 1));
    doc["threat"]     = serialized(String(pkt.local_threat_score, 2));

    serializeJson(doc, Serial);
    Serial.println();  // newline-delimited JSON
}

// ─── Status LED ─────────────────────────────────────────────────────────────
void blinkLED() {
    digitalWrite(PIN_STATUS_LED, HIGH);
    ledOnUntil = millis() + 50;  // 50ms flash per packet
}

// ─── Periodic zone status (debug) ───────────────────────────────────────────
void printZoneStatus() {
    if (zoneCount == 0) return;

    unsigned long now = millis();
    Serial.println("[GW] --- Zone Status ---");
    for (uint8_t i = 0; i < zoneCount; i++) {
        unsigned long age = now - zones[i].lastHeardMs;
        // Mark stale if no packet for 30 seconds
        if (age > 30000) {
            zones[i].active = false;
        }
        Serial.printf("[GW]  %s : %s (last %lums ago)\n",
                      zones[i].zone_id,
                      zones[i].active ? "ACTIVE" : "STALE",
                      age);
    }
    Serial.println("[GW] --------------------");
}
