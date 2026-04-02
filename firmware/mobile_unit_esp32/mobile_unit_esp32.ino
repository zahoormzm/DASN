/*
 * DASN Mobile Unit — ESP32 Upper Board
 * Board: ESP32 WROOM-32
 *
 * Reads onboard sensors and sends JSON lines over USB Serial.
 * Receives motor/buzzer commands from laptop and forwards them
 * to Aiduino over UART2.
 *
 * I2C devices:
 *   ISM330DHCX (0x6A), MMC5983MA (0x30), INA219 (0x40), VL53L5CX (0x29)
 *
 * INA219 wiring (voltage-only):
 *   VIN+ and VIN- shorted together, connected to battery+ via Aiduino VIN.
 *   Reads bus voltage only. No current measurement.
 */

#include <Wire.h>
#include <ArduinoJson.h>

#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Adafruit_INA219.h>
#include <SparkFun_VL53L5CX_Library.h>

#include "../shared/config.h"

// ─── Pin definitions ────────────────────────────────────────────────────────
// Encoders (quadrature, 4 wheels)
// Front-left
#define ENC1_A  34  // interrupt (input-only)
#define ENC1_B  32
#define ENC1_SIGN -1
// Front-right
#define ENC2_A  35  // interrupt (input-only)
#define ENC2_B  13
#define ENC2_SIGN 1
// Rear-left
#define ENC3_A  25  // interrupt
#define ENC3_B  26
#define ENC3_SIGN -1
// Rear-right
#define ENC4_A  27  // interrupt
#define ENC4_B  33
#define ENC4_SIGN -1

// HC-SR04 ultrasonic
#define PIN_US_TRIG    18
#define PIN_US_ECHO    19

// Serial2 to Aiduino
#define AIDUINO_TX     16
#define AIDUINO_RX     17

// I2C
#define PIN_SDA        21
#define PIN_SCL        22

// ─── Sensor objects ─────────────────────────────────────────────────────────
SparkFun_ISM330DHCX imu;
SFE_MMC5983MA       mag;
Adafruit_INA219     ina219(I2C_ADDR_INA219);
SparkFun_VL53L5CX   tofSensor;

VL53L5CX_ResultsData tofResults;
static bool tofReady = false;

// ─── Encoder state ──────────────────────────────────────────────────────────
static volatile long enc1Count = 0;
static volatile long enc2Count = 0;
static volatile long enc3Count = 0;
static volatile long enc4Count = 0;

static void IRAM_ATTR enc1ISR() { enc1Count += (digitalRead(ENC1_B) ? 1 : -1) * ENC1_SIGN; }
static void IRAM_ATTR enc2ISR() { enc2Count += (digitalRead(ENC2_B) ? 1 : -1) * ENC2_SIGN; }
static void IRAM_ATTR enc3ISR() { enc3Count += (digitalRead(ENC3_B) ? 1 : -1) * ENC3_SIGN; }
static void IRAM_ATTR enc4ISR() { enc4Count += (digitalRead(ENC4_B) ? 1 : -1) * ENC4_SIGN; }

// ─── Stall detection state ──────────────────────────────────────────────────
static long prevEnc1 = 0;
static long prevEnc2 = 0;
static long prevEnc3 = 0;
static long prevEnc4 = 0;
static unsigned long stallStartM1 = 0;
static unsigned long stallStartM2 = 0;
static bool stallFlagM1 = false;
static bool stallFlagM2 = false;
static int lastPWM1 = 0;
static int lastPWM2 = 0;

// ─── Timing ─────────────────────────────────────────────────────────────────
static unsigned long lastEncSend    = 0;   // 50 Hz = 20 ms
static unsigned long lastIMUSend    = 0;   // 50 Hz = 20 ms
static unsigned long lastToFSend    = 0;   // 15 Hz ~ 67 ms
static unsigned long lastBatSend    = 0;   // 1 Hz  = 1000 ms
static unsigned long lastUSSend     = 0;   // 10 Hz = 100 ms
static unsigned long lastStallCheck = 0;   // 10 Hz = 100 ms

// ─── Serial command buffer ──────────────────────────────────────────────────
#define CMD_BUF_SIZE 128
static char cmdBuf[CMD_BUF_SIZE];
static uint8_t cmdIdx = 0;

// ─── Forward declarations ───────────────────────────────────────────────────
void initSensors();
void initEncoders();
void processCommand(const char *cmd);
void sendEncoderJSON();
void sendIMUJSON();
void sendToFJSON();
void sendBatteryJSON();
void sendUltrasonicJSON();
void sendStallJSON(uint8_t channel);
float readUltrasonicMM();
void checkStall();

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(SERIAL_BAUD_USB);
    Serial2.begin(SERIAL_BAUD_UART, SERIAL_8N1, AIDUINO_RX, AIDUINO_TX);

    Serial.println("[BOT] Mobile Unit ESP32 starting...");

    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);
    digitalWrite(PIN_US_TRIG, LOW);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    initSensors();
    initEncoders();

    Serial.println("[BOT] Setup complete");
}

// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    const unsigned long now = millis();

    // Read commands from USB (laptop -> ESP32 -> Aiduino)
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                processCommand(cmdBuf);
                cmdIdx = 0;
            }
        } else if (cmdIdx < CMD_BUF_SIZE - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }

    if (now - lastEncSend >= 20) {
        lastEncSend = now;
        sendEncoderJSON();
    }
    if (now - lastIMUSend >= 20) {
        lastIMUSend = now;
        sendIMUJSON();
    }
    if (now - lastToFSend >= 67) {
        lastToFSend = now;
        sendToFJSON();
    }
    if (now - lastBatSend >= 1000) {
        lastBatSend = now;
        sendBatteryJSON();
    }
    if (now - lastUSSend >= 100) {
        lastUSSend = now;
        sendUltrasonicJSON();
    }
    if (now - lastStallCheck >= 100) {
        lastStallCheck = now;
        checkStall();
    }
}

// ─── Sensor initialisation ──────────────────────────────────────────────────
void initSensors() {
    if (imu.begin()) {
        imu.setDeviceConfig();
        imu.setBlockDataUpdate();
        imu.setAccelDataRate(ISM_XL_ODR_104Hz);
        imu.setAccelFullScale(ISM_4g);
        imu.setGyroDataRate(ISM_GY_ODR_104Hz);
        imu.setGyroFullScale(ISM_500dps);
        Serial.println("[BOT] ISM330DHCX ready");
    } else {
        Serial.println("[BOT] ISM330DHCX not found");
    }

    if (mag.begin()) {
        mag.softReset();
        delay(10);
        mag.setFilterBandwidth(800);
        mag.setContinuousModeFrequency(100);
        mag.enableContinuousMode();
        Serial.println("[BOT] MMC5983MA ready");
    } else {
        Serial.println("[BOT] MMC5983MA not found");
    }

    if (ina219.begin()) {
        ina219.setCalibration_16V_400mA();
        Serial.println("[BOT] INA219 ready (voltage-only mode)");
    } else {
        Serial.println("[BOT] INA219 not found");
    }

    if (tofSensor.begin()) {
        tofSensor.setResolution(64);
        tofSensor.setRangingFrequency(15);
        tofSensor.startRanging();
        tofReady = true;
        Serial.println("[BOT] VL53L5CX ready (8x8 @ 15Hz)");
    } else {
        tofReady = false;
        Serial.println("[BOT] VL53L5CX not found");
    }
}

// ─── Encoder init ───────────────────────────────────────────────────────────
void initEncoders() {
    // GPIO34/35 are input-only and have no internal pull-up.
    pinMode(ENC1_A, INPUT);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT);
    pinMode(ENC2_B, INPUT_PULLUP);
    pinMode(ENC3_A, INPUT_PULLUP);
    pinMode(ENC3_B, INPUT_PULLUP);
    pinMode(ENC4_A, INPUT_PULLUP);
    pinMode(ENC4_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC4_A), enc4ISR, RISING);

    Serial.println("[BOT] Encoders ready (4 wheels)");
}

// ─── Command processing ────────────────────────────────────────────────────
void processCommand(const char *cmd) {
    if (cmd[0] == 'M' && cmd[1] == ',') {
        int p1 = 0, d1 = 0, p2 = 0, d2 = 0;
        if (sscanf(cmd, "M,%d,%d,%d,%d", &p1, &d1, &p2, &d2) == 4) {
            lastPWM1 = p1;
            lastPWM2 = p2;
        }
        Serial2.println(cmd);
        return;
    }

    if (cmd[0] == 'S' && (cmd[1] == '\0' || cmd[1] == ',')) {
        Serial2.println("S");
        lastPWM1 = 0;
        lastPWM2 = 0;
        return;
    }

    if (cmd[0] == 'E' || cmd[0] == 'D' || (cmd[0] == 'B' && cmd[1] == ',')) {
        Serial2.println(cmd);
        if (cmd[0] == 'D') {
            lastPWM1 = 0;
            lastPWM2 = 0;
        }
        return;
    }
}

// ─── Encoder JSON (50 Hz) ───────────────────────────────────────────────────
void sendEncoderJSON() {
    noInterrupts();
    const long fl = enc1Count;
    const long fr = enc2Count;
    const long rl = enc3Count;
    const long rr = enc4Count;
    interrupts();

    const long l = (fl + rl) / 2;
    const long r = (fr + rr) / 2;

    StaticJsonDocument<192> doc;
    doc["type"] = "enc";
    doc["l"] = l;
    doc["r"] = r;
    doc["fl"] = fl;
    doc["fr"] = fr;
    doc["rl"] = rl;
    doc["rr"] = rr;
    doc["t"] = millis();
    serializeJson(doc, Serial);
    Serial.println();
}

// ─── IMU + Magnetometer JSON (50 Hz) ───────────────────────────────────────
void sendIMUJSON() {
    sfe_ism_data_t accelData, gyroData;
    if (!imu.checkStatus()) return;

    imu.getAccel(&accelData);
    imu.getGyro(&gyroData);

    uint32_t rawX = 0, rawY = 0, rawZ = 0;
    mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);
    const float mx = ((float)rawX - 131072.0f) / 131072.0f * 8.0f;
    const float my = ((float)rawY - 131072.0f) / 131072.0f * 8.0f;
    const float mz = ((float)rawZ - 131072.0f) / 131072.0f * 8.0f;

    StaticJsonDocument<256> doc;
    doc["type"] = "imu";
    doc["ax"] = serialized(String(accelData.xData, 3));
    doc["ay"] = serialized(String(accelData.yData, 3));
    doc["az"] = serialized(String(accelData.zData, 3));
    doc["gx"] = serialized(String(gyroData.xData, 3));
    doc["gy"] = serialized(String(gyroData.yData, 3));
    doc["gz"] = serialized(String(gyroData.zData, 3));
    doc["mx"] = serialized(String(mx, 3));
    doc["my"] = serialized(String(my, 3));
    doc["mz"] = serialized(String(mz, 3));
    doc["t"] = millis();
    serializeJson(doc, Serial);
    Serial.println();
}

// ─── ToF 8x8 JSON (15 Hz) ──────────────────────────────────────────────────
void sendToFJSON() {
    if (!tofReady) return;
    if (!tofSensor.isDataReady()) return;
    if (!tofSensor.getRangingData(&tofResults)) return;

    StaticJsonDocument<768> doc;
    doc["type"] = "tof";
    JsonArray dArr = doc.createNestedArray("d");
    for (int i = 0; i < 64; i++) {
        dArr.add(tofResults.distance_mm[i]);
    }
    doc["t"] = millis();
    serializeJson(doc, Serial);
    Serial.println();
}

// ─── Battery JSON (1 Hz) ───────────────────────────────────────────────────
void sendBatteryJSON() {
    const float busV = ina219.getBusVoltage_V();

    StaticJsonDocument<64> doc;
    doc["type"] = "bat";
    doc["v"] = serialized(String(busV, 2));
    doc["t"] = millis();
    serializeJson(doc, Serial);
    Serial.println();
}

// ─── Ultrasonic JSON (10 Hz) ───────────────────────────────────────────────
void sendUltrasonicJSON() {
    const float distMM = readUltrasonicMM();

    StaticJsonDocument<64> doc;
    doc["type"] = "us";
    doc["d"] = serialized(String(distMM, 0));
    doc["t"] = millis();
    serializeJson(doc, Serial);
    Serial.println();
}

// ─── HC-SR04 ultrasonic reading (mm) ───────────────────────────────────────
float readUltrasonicMM() {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    const unsigned long duration = pulseIn(PIN_US_ECHO, HIGH, 30000);
    if (duration == 0) return -1.0f;

    return (float)duration * 0.343f / 2.0f;
}

// ─── Encoder-based stall detection ─────────────────────────────────────────
void checkStall() {
    const unsigned long now = millis();

    noInterrupts();
    const long curEnc1 = enc1Count;
    const long curEnc2 = enc2Count;
    const long curEnc3 = enc3Count;
    const long curEnc4 = enc4Count;
    interrupts();

    const bool leftMoving = (curEnc1 != prevEnc1) || (curEnc3 != prevEnc3);
    const bool rightMoving = (curEnc2 != prevEnc2) || (curEnc4 != prevEnc4);

    if (lastPWM1 > STALL_MIN_PWM && !leftMoving) {
        if (stallStartM1 == 0) {
            stallStartM1 = now;
        } else if (!stallFlagM1 && (now - stallStartM1 >= STALL_ENCODER_TIMEOUT_MS)) {
            stallFlagM1 = true;
            sendStallJSON(1);
        }
    } else {
        stallStartM1 = 0;
        stallFlagM1 = false;
    }

    if (lastPWM2 > STALL_MIN_PWM && !rightMoving) {
        if (stallStartM2 == 0) {
            stallStartM2 = now;
        } else if (!stallFlagM2 && (now - stallStartM2 >= STALL_ENCODER_TIMEOUT_MS)) {
            stallFlagM2 = true;
            sendStallJSON(2);
        }
    } else {
        stallStartM2 = 0;
        stallFlagM2 = false;
    }

    prevEnc1 = curEnc1;
    prevEnc2 = curEnc2;
    prevEnc3 = curEnc3;
    prevEnc4 = curEnc4;
}

// ─── Stall event JSON ───────────────────────────────────────────────────────
void sendStallJSON(uint8_t channel) {
    StaticJsonDocument<96> doc;
    doc["type"] = "stall";
    doc["ch"] = channel;
    doc["t"] = millis();
    serializeJson(doc, Serial);
    Serial.println();

    Serial2.println("S");
    lastPWM1 = 0;
    lastPWM2 = 0;
    Serial.println("[BOT] STALL detected — emergency stop sent to Aiduino");
}
