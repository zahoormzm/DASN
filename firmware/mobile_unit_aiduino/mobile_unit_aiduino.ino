/*
 * DASN Mobile Unit — Aiduino Motor Controller
 * Board: ATmega328P (Arduino Uno / Nano compatible)
 *
 * Simple serial listener. Receives motor commands from the ESP32
 * via hardware Serial (UART). Controls two DC motors via H-bridge
 * and provides a watchdog auto-stop. Also drives an SSD1306 OLED
 * over SPI for local status.
 *
 * Pins:
 *   Motor 1: PWM=9, DIR=7
 *   Motor 2: PWM=10, DIR=8
 *   MOSFET gate: 5 (keep LOW for safety unless motors enabled)
 *   Passive buzzer: 11 (PWM capable)
 *   OLED SPI: CLK=4, MOSI=3, DC=2, CS=6, RST=12
 *   Status LED: 13 (onboard)
 *
 * Libraries: SPI, Adafruit_GFX, Adafruit_SSD1306
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── Pin definitions ────────────────────────────────────────────────────────
#define M1_PWM    9
#define M1_DIR    7
#define M2_PWM    10
#define M2_DIR    8
#define M1_DIR_INVERT 1
#define M2_DIR_INVERT 0
#define MOSFET    5
#define BUZZER_PIN 11
#define BUZZER_IS_ACTIVE 1
#define OLED_RST_PIN 12
#define OLED_DC_PIN  2
#define OLED_CS_PIN  6
#define OLED_CLK_PIN 4
#define OLED_MOSI_PIN 3
#define LED_PIN   13

// ─── OLED config (SPI SSD1306) ─────────────────────────────────────────────
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, OLED_MOSI_PIN, OLED_CLK_PIN, OLED_DC_PIN, OLED_RST_PIN, OLED_CS_PIN);

// ─── Watchdog timeout ───────────────────────────────────────────────────────
#define WATCHDOG_MS  1000   // auto-stop if no command within this window

// ─── State ──────────────────────────────────────────────────────────────────
static unsigned long lastCommandMs  = 0;
static bool          motorsEnabled  = false;
static bool          watchdogTripped = false;
static unsigned long buzzerOffAtMs = 0;
static bool          buzzerActive = false;
static int           buzzerFreq = 0;
static int           currentPWM1 = 0;
static int           currentPWM2 = 0;
static int           currentDir1 = 0;
static int           currentDir2 = 0;
static bool          oledAvailable = false;
static unsigned long lastOLEDUpdateMs = 0;

// ─── Serial command buffer ──────────────────────────────────────────────────
#define CMD_BUF_SIZE  64
static char    cmdBuf[CMD_BUF_SIZE];
static uint8_t cmdIdx = 0;

// ─── LED blink state ────────────────────────────────────────────────────────
static unsigned long lastBlink  = 0;
static bool          ledState   = false;

// ─── Forward declarations ───────────────────────────────────────────────────
void processCommand(const char *cmd);
void setMotors(int pwm1, int dir1, int pwm2, int dir2);
void emergencyStop();
void enableMotors();
void disableMotors();
void updateStatusLED();
void buzzerOn(int freqIgnored = 0);
void buzzerOff();
void initOLED();
void updateOLED();

// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // Motor pins
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // MOSFET gate — keep LOW (motors disconnected) until explicitly enabled
    pinMode(MOSFET, OUTPUT);
    digitalWrite(MOSFET, LOW);

    // Status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // OLED
    initOLED();
    buzzerOn();
    delay(120);
    buzzerOff();

    // Start with motors stopped
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M2_DIR, LOW);

    motorsEnabled = false;
    lastCommandMs = millis();

    // Startup blink pattern
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }

    Serial.println("[AID] Aiduino ready");
}

// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    // ── Read serial commands ──
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

    // ── Watchdog: auto-stop if no command received within timeout ──
    unsigned long now = millis();
    if (motorsEnabled && (now - lastCommandMs >= WATCHDOG_MS)) {
        if (!watchdogTripped) {
            watchdogTripped = true;
            emergencyStop();
            Serial.println("[AID] Watchdog: no command — motors stopped");
        }
    }

    // ── Timed buzzer tone auto-off ──
    if (buzzerOffAtMs > 0 && now >= buzzerOffAtMs) {
        buzzerOff();
        buzzerOffAtMs = 0;
    }

    // ── Status LED ──
    updateStatusLED();

    // ── OLED status refresh ──
    if (oledAvailable && (now - lastOLEDUpdateMs >= 200)) {
        lastOLEDUpdateMs = now;
        updateOLED();
    }
}

// ─── Command processing ────────────────────────────────────────────────────
void processCommand(const char *cmd) {
    lastCommandMs = millis();
    watchdogTripped = false;

    switch (cmd[0]) {
        case 'M': {
            // M,<pwm1>,<dir1>,<pwm2>,<dir2>
            int pwm1 = 0, dir1 = 0, pwm2 = 0, dir2 = 0;
            if (sscanf(cmd, "M,%d,%d,%d,%d", &pwm1, &dir1, &pwm2, &dir2) == 4) {
                // Clamp PWM to [0, 255]
                pwm1 = constrain(pwm1, 0, 255);
                pwm2 = constrain(pwm2, 0, 255);
                dir1 = (dir1 != 0) ? 1 : 0;
                dir2 = (dir2 != 0) ? 1 : 0;

                if (motorsEnabled) {
                    setMotors(pwm1, dir1, pwm2, dir2);
                } else {
                    Serial.println("[AID] Motors disabled — ignoring M command");
                }
            } else {
                Serial.println("[AID] Bad M command");
            }
            break;
        }

        case 'S':
            // Emergency stop
            emergencyStop();
            Serial.println("[AID] Emergency stop");
            break;

        case 'E':
            // Enable motors
            enableMotors();
            Serial.println("[AID] Motors enabled");
            break;

        case 'D':
            // Disable motors
            disableMotors();
            Serial.println("[AID] Motors disabled");
            break;

        case 'B': {
            // B,0           -> buzzer off
            // B,1           -> default tone on
            // B,<freq>      -> tone on
            // B,<freq>,<ms> -> timed tone
            int freq = 0, dur = 0;
            if (strcmp(cmd, "B,0") == 0) {
                buzzerOff();
                buzzerOffAtMs = 0;
                Serial.println("[AID] Buzzer off");
            } else if (strcmp(cmd, "B,1") == 0) {
                buzzerOn(2000);
                buzzerOffAtMs = 0;
                Serial.println("[AID] Buzzer on");
            } else if (sscanf(cmd, "B,%d,%d", &freq, &dur) == 2) {
                freq = constrain(freq, 50, 10000);
                dur = constrain(dur, 1, 5000);
                buzzerOn(freq);
                buzzerOffAtMs = millis() + dur;
                Serial.print("[AID] Buzzer on for ");
                Serial.print(dur);
                Serial.println(" ms");
            } else if (sscanf(cmd, "B,%d", &freq) == 1) {
                freq = constrain(freq, 50, 10000);
                buzzerOn(freq);
                buzzerOffAtMs = 0;
                Serial.println("[AID] Buzzer on");
            } else {
                Serial.println("[AID] Bad B command");
            }
            break;
        }

        default:
            Serial.print("[AID] Unknown command: ");
            Serial.println(cmd);
            break;
    }
}

// ─── Motor control ──────────────────────────────────────────────────────────
void setMotors(int pwm1, int dir1, int pwm2, int dir2) {
    int outDir1 = (dir1 ? 1 : 0) ^ M1_DIR_INVERT;
    int outDir2 = (dir2 ? 1 : 0) ^ M2_DIR_INVERT;

    currentPWM1 = pwm1;
    currentPWM2 = pwm2;
    currentDir1 = outDir1;
    currentDir2 = outDir2;

    digitalWrite(M1_DIR, outDir1 ? HIGH : LOW);
    analogWrite(M1_PWM, pwm1);

    digitalWrite(M2_DIR, outDir2 ? HIGH : LOW);
    analogWrite(M2_PWM, pwm2);
}

void emergencyStop() {
    currentPWM1 = 0;
    currentPWM2 = 0;

    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    digitalWrite(M1_DIR, LOW);
    digitalWrite(M2_DIR, LOW);
    // Do NOT disable MOSFET here — just stop PWM so we can resume quickly
}

void enableMotors() {
    motorsEnabled = true;
    digitalWrite(MOSFET, LOW);   // MOSFET gate LOW = connected (P-channel or active-low)
    // Note: adjust polarity based on your actual MOSFET circuit.
    // If N-channel high-side with driver, set HIGH to enable.
}

void disableMotors() {
    emergencyStop();
    motorsEnabled = false;
    digitalWrite(MOSFET, LOW);   // Keep MOSFET off / safe state
    buzzerOff();
    buzzerOffAtMs = 0;
}

void buzzerOn(int freqIgnored) {
    (void)freqIgnored;
#if BUZZER_IS_ACTIVE
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerFreq = 1;
#else
    tone(BUZZER_PIN, (freqIgnored > 0) ? freqIgnored : 2000);
    buzzerFreq = (freqIgnored > 0) ? freqIgnored : 2000;
#endif
    buzzerActive = true;
}

void buzzerOff() {
#if BUZZER_IS_ACTIVE
    digitalWrite(BUZZER_PIN, LOW);
#else
    noTone(BUZZER_PIN);
#endif
    buzzerActive = false;
    buzzerFreq = 0;
}

// ─── Status LED patterns ────────────────────────────────────────────────────
void updateStatusLED() {
    unsigned long now = millis();
    unsigned long blinkInterval;

    if (watchdogTripped) {
        // Fast blink: watchdog tripped
        blinkInterval = 100;
    } else if (motorsEnabled) {
        // Slow blink: motors enabled, normal operation
        blinkInterval = 500;
    } else {
        // Very slow blink: motors disabled / idle
        blinkInterval = 1500;
    }

    if (now - lastBlink >= blinkInterval) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    }
}

// ─── OLED helpers ───────────────────────────────────────────────────────────
void initOLED() {
    if (!oled.begin(SSD1306_SWITCHCAPVCC)) {
        oledAvailable = false;
        Serial.println("[AID] OLED init failed");
        return;
    }
    oledAvailable = true;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println(F("DASN Aiduino"));
    oled.println(F("OLED READY"));
    oled.display();
}

void updateOLED() {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0, 0);

    oled.println(F("DASN Aiduino"));
    oled.print(F("M:"));
    oled.print(motorsEnabled ? F("EN ") : F("DIS"));
    oled.print(F(" WD:"));
    oled.println(watchdogTripped ? F("TRIP") : F("OK"));

    oled.print(F("M1 "));
    oled.print(currentDir1 ? F("F") : F("R"));
    oled.print(F(" "));
    oled.println(currentPWM1);

    oled.print(F("M2 "));
    oled.print(currentDir2 ? F("F") : F("R"));
    oled.print(F(" "));
    oled.println(currentPWM2);

    oled.print(F("Buzz:"));
    if (buzzerActive) {
        oled.print(F("ON "));
        oled.print(buzzerFreq);
        oled.println(F("Hz"));
    } else {
        oled.println(F("OFF"));
    }

    unsigned long age = millis() - lastCommandMs;
    oled.print(F("Cmd age:"));
    oled.print(age);
    oled.println(F("ms"));

    oled.display();
}
