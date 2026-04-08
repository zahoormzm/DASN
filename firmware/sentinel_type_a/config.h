#ifndef DASN_TYPE_A_CONFIG_H
#define DASN_TYPE_A_CONFIG_H

/*
 * Local Arduino IDE config for the pivoted Sentinel Type A node.
 * This build is the ESP32 sensor/display endpoint for the laptop master server.
 */

#define WIFI_STA_SSID              "Airtel_Geto"
#define WIFI_STA_PASS              "Ami@2004"
#define WIFI_UDP_PORT              37020
#define WIFI_CONTROL_PORT          37021
#define WIFI_CONNECT_TIMEOUT_MS    15000UL
#define WIFI_RETRY_INTERVAL_MS     5000UL

#define SERIAL_BAUD_USB            115200
#define SENSOR_READ_INTERVAL_MS    250UL
#define DISPLAY_UPDATE_INTERVAL_MS 250UL
#define REMOTE_CONTROL_HOLD_MS     5000UL

#define PIN_I2C_SDA                21
#define PIN_I2C_SCL                22

#define PIN_LED_GREEN              13
#define PIN_LED_RED                23

#define PIN_SOUND_DOUT             14
#define PIN_SOUND_AOUT             34

#define LCD_I2C_ADDR_PRIMARY       0x27
#define LCD_I2C_ADDR_FALLBACK      0x3F

#define I2C_ADDR_BME688            0x76
#define I2C_ADDR_CAP1203           0x28
#define I2C_ADDR_VL53L5CX          0x29

#define CAP1203_MAIN_CTRL          0x00
#define CAP1203_SENSOR_INPUT       0x03

#define SOUND_ANALOG_FULL_SCALE    4095.0f
#define SOUND_DB_FULL_SCALE        100.0f
#define SOUND_TRANSIENT_DB         70.0f
#define SOUND_BASELINE_DB          35.0f

#define TOF_OCCUPIED_MM            1800
#define TOF_WARNING_MM             900

#endif
