#ifndef DASN_CONFIG_H
#define DASN_CONFIG_H

/*
 * DASN Shared Configuration
 * Common settings for all sentinel nodes and the gateway.
 */

// ─── Gateway MAC address (replace with your actual gateway ESP32 MAC) ────────
static const uint8_t GATEWAY_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ─── ESP-NOW ─────────────────────────────────────────────────────────────────
#define ESPNOW_CHANNEL          1
#define ESPNOW_ENCRYPT          false

// ─── Serial baud rates ──────────────────────────────────────────────────────
#define SERIAL_BAUD_USB         115200
#define SERIAL_BAUD_UART        115200

// ─── Sensor reading interval (ms) ───────────────────────────────────────────
#define SENSOR_READ_INTERVAL_MS 500

// ─── Threat score thresholds ────────────────────────────────────────────────
#define THREAT_LOW              0.25f
#define THREAT_MEDIUM           0.50f
#define THREAT_HIGH             0.75f
#define THREAT_CRITICAL         0.90f

// ─── Sensor thresholds ─────────────────────────────────────────────────────
#define SOUND_TRANSIENT_DB      70.0f   // dB above which counts as transient
#define SOUND_BASELINE_DB       40.0f   // expected ambient noise floor
#define RADAR_CONFIDENCE_MIN    0.30f   // minimum confidence to consider valid
#define PIR_DEBOUNCE_MS         2000    // PIR signal debounce time
#define GAS_RESISTANCE_ALERT    10000.0f // ohms, below this is suspicious

// ─── Battery thresholds ────────────────────────────────────────────────────
#define BATTERY_LOW_V           3.3f
#define BATTERY_CRITICAL_V      3.0f
#define BATTERY_ADC_PIN         36      // VP pin on ESP32
#define BATTERY_DIVIDER_RATIO   2.0f    // voltage divider ratio

// ─── I2C addresses ─────────────────────────────────────────────────────────
#define I2C_ADDR_BME688         0x76
#define I2C_ADDR_CAP1203        0x28
#define I2C_ADDR_SSD1306        0x3C
#define I2C_ADDR_STTS22H        0x3C    // Type C uses different OLED addr or no OLED
#define I2C_ADDR_ISM330DHCX     0x6A
#define I2C_ADDR_MMC5983MA      0x30
#define I2C_ADDR_INA219         0x40
#define I2C_ADDR_VL53L5CX       0x29

// ─── Common pin definitions (Type A / Type C) ──────────────────────────────
#define PIN_PIR                 13
#define PIN_BUZZER              4
#define PIN_RADAR_RX            16
#define PIN_RADAR_TX            17
#define PIN_DS18B20             15
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22

// ─── I2S pins (INMP441 microphone) ─────────────────────────────────────────
#define PIN_I2S_SCK             26
#define PIN_I2S_WS              25
#define PIN_I2S_SD              33

// ─── Buzzer tones ──────────────────────────────────────────────────────────
#define BUZZER_FREQ_LOW         1000
#define BUZZER_FREQ_MED         2000
#define BUZZER_FREQ_HIGH        3000
#define BUZZER_FREQ_CRIT        4000
#define BUZZER_CHANNEL          0       // LEDC channel

// ─── Deep sleep ────────────────────────────────────────────────────────────
#define DEEP_SLEEP_US           5000000 // 5 seconds

// ─── Watchdog / safety ─────────────────────────────────────────────────────
#define MOTOR_WATCHDOG_MS       1000    // auto-stop if no command in this window
#define STALL_ENCODER_TIMEOUT_MS 500    // if encoders not moving while commanded, stall
#define STALL_MIN_PWM            30     // minimum PWM to consider "commanding movement"

#endif // DASN_CONFIG_H
