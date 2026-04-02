#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <stdint.h>

/*
 * DASN ESP-NOW Protocol Header
 * Shared struct for all sentinel-to-gateway communication.
 * comm_mode is always 0 (ESP-NOW only). LoRa is not implemented.
 */

#define COMM_MODE_ESPNOW 0

typedef struct __attribute__((packed)) {
    char     zone_id[16];           // e.g. "FRONT_DOOR"
    uint32_t timestamp;             // millis()
    bool     radar_presence;
    float    radar_confidence;
    bool     pir_triggered;
    float    sound_level_db;
    bool     sound_transient;
    float    bme688_gas_resistance;
    float    bme688_temperature;
    float    bme688_humidity;
    float    bme688_pressure;
    float    battery_v;
    float    local_threat_score;
    uint8_t  comm_mode;             // 0 = espnow only
} SentinelData_t;

// ESP-NOW maximum payload is 250 bytes; SentinelData_t must fit.
static_assert(sizeof(SentinelData_t) <= 250, "SentinelData_t exceeds ESP-NOW payload limit");

#endif // ESPNOW_PROTOCOL_H
