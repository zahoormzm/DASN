/*
 * DASN Mobile Unit — Aiduino Motor Test
 * Board: ATmega328P (Uno/Nano)
 *
 * Purpose:
 *   1) Validate motor driver pins and directions
 *   2) Validate buzzer control on D11
 *   3) Validate OLED on SPI (CLK=4, MOSI=3, DC=2, CS=6, RST=12)
 *   4) Respond to ESP32 motor/command tests over UART
 *
 * Serial monitor: 115200
 */

#if !defined(ARDUINO_ARCH_AVR)
#error "This sketch requires an AVR board (Arduino Uno/Nano ATmega328P)."
#endif

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define M1_PWM 9
#define M1_DIR 7
#define M2_PWM 10
#define M2_DIR 8
#define M1_DIR_INVERT 1
#define M2_DIR_INVERT 0
#define MOSFET 5

#define BUZZER_PIN 11
#define BUZZER_IS_ACTIVE 1
#define OLED_RST_PIN 12
#define OLED_DC_PIN 2
#define OLED_CS_PIN 6
#define OLED_CLK_PIN 4
#define OLED_MOSI_PIN 3
#define LED_PIN 13

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define WATCHDOG_MS 1200

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, OLED_MOSI_PIN, OLED_CLK_PIN, OLED_DC_PIN, OLED_RST_PIN, OLED_CS_PIN);

static bool motorsEnabled = false;
static bool watchdogTrip = false;
static unsigned long lastCmdMs = 0;
static unsigned long lastOLEDMs = 0;

static int pwm1 = 0, pwm2 = 0, dir1 = 0, dir2 = 0;
static bool buzzerOn = false;
static int buzzerFreq = 0;
static unsigned long buzzerOffAt = 0;

static bool autoMode = false;
static int autoStep = 0;
static unsigned long autoStepStart = 0;
static bool oledReady = false;

#define CMD_BUF_SIZE 64
static char cmdBuf[CMD_BUF_SIZE];
static uint8_t cmdIdx = 0;

void setMotors(int p1, int d1, int p2, int d2) {
    int outDir1 = (d1 ? 1 : 0) ^ M1_DIR_INVERT;
    int outDir2 = (d2 ? 1 : 0) ^ M2_DIR_INVERT;

    pwm1 = constrain(p1, 0, 255);
    pwm2 = constrain(p2, 0, 255);
    dir1 = outDir1;
    dir2 = outDir2;

    digitalWrite(M1_DIR, dir1 ? HIGH : LOW);
    digitalWrite(M2_DIR, dir2 ? HIGH : LOW);
    analogWrite(M1_PWM, pwm1);
    analogWrite(M2_PWM, pwm2);
}

void stopMotors() {
    pwm1 = 0;
    pwm2 = 0;
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
}

void enableMotors() {
    motorsEnabled = true;
    watchdogTrip = false;
    digitalWrite(MOSFET, LOW);  // keep as current board's safe polarity
}

void disableMotors() {
    stopMotors();
    motorsEnabled = false;
    digitalWrite(MOSFET, LOW);
}

void applyBuzzerOff() {
#if BUZZER_IS_ACTIVE
    digitalWrite(BUZZER_PIN, LOW);
#else
    noTone(BUZZER_PIN);
#endif
    buzzerOn = false;
    buzzerFreq = 0;
    buzzerOffAt = 0;
}

void applyBuzzerOn(int freq, int durationMs) {
    int f = constrain(freq, 50, 10000);
#if BUZZER_IS_ACTIVE
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerFreq = 1;
#else
    tone(BUZZER_PIN, f);
    buzzerFreq = f;
#endif
    buzzerOn = true;
    if (durationMs > 0) {
        buzzerOffAt = millis() + (unsigned long)constrain(durationMs, 1, 10000);
    } else {
        buzzerOffAt = 0;
    }
}

void startAutoTest() {
    autoMode = true;
    autoStep = 0;
    autoStepStart = millis();
    enableMotors();
    Serial.println(F("[AID_TEST] Auto test started"));
}

void runAutoTest() {
    if (!autoMode) return;

    unsigned long now = millis();
    unsigned long elapsed = now - autoStepStart;

    switch (autoStep) {
        case 0: // settle
            stopMotors();
            if (elapsed >= 700) {
                autoStep = 1;
                autoStepStart = now;
                applyBuzzerOn(1800, 80);
            }
            break;
        case 1: // forward
            setMotors(120, 1, 120, 1);
            if (elapsed >= 2500) {
                autoStep = 2;
                autoStepStart = now;
            }
            break;
        case 2: // stop
            stopMotors();
            if (elapsed >= 700) {
                autoStep = 3;
                autoStepStart = now;
                applyBuzzerOn(2000, 80);
            }
            break;
        case 3: // reverse
            setMotors(120, 0, 120, 0);
            if (elapsed >= 2500) {
                autoStep = 4;
                autoStepStart = now;
            }
            break;
        case 4: // spin
            setMotors(130, 1, 130, 0);
            if (elapsed >= 1800) {
                autoStep = 5;
                autoStepStart = now;
            }
            break;
        default:
            stopMotors();
            autoMode = false;
            applyBuzzerOn(2400, 120);
            Serial.println(F("[AID_TEST] Auto test done"));
            break;
    }
}

void initOLED() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC)) {
        oledReady = false;
        Serial.println(F("[AID_TEST] OLED init failed"));
        return;
    }
    oledReady = true;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println(F("Aiduino Test"));
    oled.println(F("Ready"));
    oled.display();
}

void updateOLED() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println(F("Aiduino Motor Test"));

    oled.print(F("M:"));
    oled.print(motorsEnabled ? F("EN ") : F("DIS"));
    oled.print(F(" WD:"));
    oled.println(watchdogTrip ? F("TRIP") : F("OK"));

    oled.print(F("M1 "));
    oled.print(dir1 ? F("F") : F("R"));
    oled.print(F(" "));
    oled.println(pwm1);

    oled.print(F("M2 "));
    oled.print(dir2 ? F("F") : F("R"));
    oled.print(F(" "));
    oled.println(pwm2);

    oled.print(F("Buz:"));
    if (buzzerOn) {
        oled.println(F("ON"));
    } else {
        oled.println(F("OFF"));
    }

    oled.print(F("Auto:"));
    oled.println(autoMode ? F("RUN") : F("IDLE"));
    oled.display();
}

void printHelp() {
    Serial.println(F("=== AIDUINO MOTOR TEST ==="));
    Serial.println(F("Commands:"));
    Serial.println(F("  E  -> enable motors"));
    Serial.println(F("  D  -> disable motors"));
    Serial.println(F("  S  -> stop motors"));
    Serial.println(F("  M,<p1>,<d1>,<p2>,<d2>"));
    Serial.println(F("  B,0 | B,1 | B,<freq> | B,<freq>,<ms>"));
    Serial.println(F("  T  -> run local auto test"));
    Serial.println(F("  P  -> ping response"));
    Serial.println(F("  H  -> help"));
}

void processCommand(const char *cmd) {
    lastCmdMs = millis();
    watchdogTrip = false;

    if (strcmp(cmd, "H") == 0) {
        printHelp();
        return;
    }
    if (strcmp(cmd, "P") == 0) {
        Serial.println(F("[AID_TEST] PONG"));
        return;
    }
    if (strcmp(cmd, "E") == 0) {
        autoMode = false;
        enableMotors();
        Serial.println(F("[AID_TEST] motors enabled"));
        return;
    }
    if (strcmp(cmd, "D") == 0) {
        autoMode = false;
        disableMotors();
        Serial.println(F("[AID_TEST] motors disabled"));
        return;
    }
    if (strcmp(cmd, "S") == 0) {
        autoMode = false;
        stopMotors();
        Serial.println(F("[AID_TEST] motors stopped"));
        return;
    }
    if (strcmp(cmd, "T") == 0) {
        startAutoTest();
        return;
    }

    if (cmd[0] == 'M' && cmd[1] == ',') {
        int p1 = 0, d1 = 0, p2 = 0, d2 = 0;
        if (sscanf(cmd, "M,%d,%d,%d,%d", &p1, &d1, &p2, &d2) == 4) {
            autoMode = false;
            if (!motorsEnabled) {
                Serial.println(F("[AID_TEST] motors disabled; use E first"));
                return;
            }
            setMotors(p1, d1, p2, d2);
            Serial.println(F("[AID_TEST] M applied"));
        } else {
            Serial.println(F("[AID_TEST] bad M command"));
        }
        return;
    }

    if (cmd[0] == 'B' && cmd[1] == ',') {
        int freq = 0;
        int dur = 0;
        if (strcmp(cmd, "B,0") == 0) {
            applyBuzzerOff();
            Serial.println(F("[AID_TEST] buzzer off"));
        } else if (strcmp(cmd, "B,1") == 0) {
            applyBuzzerOn(2000, 0);
            Serial.println(F("[AID_TEST] buzzer on 2kHz"));
        } else if (sscanf(cmd, "B,%d,%d", &freq, &dur) == 2) {
            applyBuzzerOn(freq, dur);
            Serial.println(F("[AID_TEST] buzzer timed"));
        } else if (sscanf(cmd, "B,%d", &freq) == 1) {
            applyBuzzerOn(freq, 0);
            Serial.println(F("[AID_TEST] buzzer freq"));
        } else {
            Serial.println(F("[AID_TEST] bad B command"));
        }
        return;
    }

    Serial.println(F("[AID_TEST] unknown command"));
}

void setup() {
    Serial.begin(115200);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(MOSFET, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(MOSFET, LOW);
    digitalWrite(LED_PIN, LOW);
    stopMotors();
    applyBuzzerOff();

    initOLED();
    applyBuzzerOn(1800, 120);
    printHelp();

    lastCmdMs = millis();
    Serial.println(F("[AID_TEST] ready"));
}

void loop() {
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

    unsigned long now = millis();

    if (motorsEnabled && (now - lastCmdMs > WATCHDOG_MS) && !autoMode) {
        stopMotors();
        watchdogTrip = true;
    }

    if (buzzerOffAt > 0 && now >= buzzerOffAt) {
        applyBuzzerOff();
    }

    runAutoTest();

    digitalWrite(LED_PIN, motorsEnabled ? ((now / 250) % 2) : ((now / 1200) % 2));

    if (oledReady && (now - lastOLEDMs >= 200)) {
        lastOLEDMs = now;
        updateOLED();
    }
}
