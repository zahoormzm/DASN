#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <stdint.h>

/*
 * Local copy for Arduino IDE builds.
 * Keep this in sync with firmware/shared/espnow_protocol.h.
 */

#define COMM_MODE_ESPNOW 0
#define COMM_MODE_WIFI   1

typedef struct __attribute__((packed)) {
    char     zone_id[16];
    uint32_t timestamp;
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
    uint8_t  comm_mode;
} SentinelData_t;

static_assert(sizeof(SentinelData_t) <= 250, "SentinelData_t exceeds ESP-NOW payload limit");

#endif // ESPNOW_PROTOCOL_H
