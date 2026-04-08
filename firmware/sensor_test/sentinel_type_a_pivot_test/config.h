#ifndef DASN_TYPE_A_PIVOT_TEST_CONFIG_H
#define DASN_TYPE_A_PIVOT_TEST_CONFIG_H

#define SERIAL_BAUD_USB         115200

#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22

#define PIN_LED_GREEN           13
#define PIN_LED_RED             23

// SmartElex sound sensor
// DOUT can be any digital input pin.
// AOUT should stay on an ADC1 pin because ESP32 Wi-Fi and ADC2 conflict.
#define PIN_SOUND_DOUT          14
#define PIN_SOUND_AOUT          34

#define LCD_I2C_ADDR_PRIMARY    0x27
#define LCD_I2C_ADDR_FALLBACK   0x3F

#define I2C_ADDR_BME688         0x76
#define I2C_ADDR_CAP1203        0x28
#define I2C_ADDR_VL53L5CX       0x29

#define CAP1203_MAIN_CTRL       0x00
#define CAP1203_SENSOR_INPUT    0x03

#define TOF_OCCUPIED_MM         1800
#define TOF_WARNING_MM          900

#endif
