/*
 * DASN Mobile Unit — ESP32 Motor + Encoder Test
 * Board: ESP32 WROOM-32
 *
 * Purpose:
 *   Command motors through Aiduino (UART2) and verify all 4 encoder counts
 *   increase/decrease during motion.
 *
 * Requires:
 *   Aiduino flashed with mobile_unit_aiduino_motor_test.ino
 *
 * Serial monitor: 115200
 * Commands:
 *   r = run test sequence
 *   s = stop motors now
 *   z = zero encoder counts
 *   h = help
 */

#ifndef ARDUINO_ARCH_ESP32
#error "This sketch requires an ESP32 board selection."
#endif

#define ENC1_A  34
#define ENC1_B  32
#define ENC1_SIGN -1
#define ENC2_A  35
#define ENC2_B  13
#define ENC2_SIGN 1
#define ENC3_A  25
#define ENC3_B  26
#define ENC3_SIGN -1
#define ENC4_A  27
#define ENC4_B  33
#define ENC4_SIGN -1

#define AIDUINO_TX 16
#define AIDUINO_RX 17

volatile long enc1Count = 0, enc2Count = 0, enc3Count = 0, enc4Count = 0;
void IRAM_ATTR enc1ISR() { enc1Count += (digitalRead(ENC1_B) ? 1 : -1) * ENC1_SIGN; }
void IRAM_ATTR enc2ISR() { enc2Count += (digitalRead(ENC2_B) ? 1 : -1) * ENC2_SIGN; }
void IRAM_ATTR enc3ISR() { enc3Count += (digitalRead(ENC3_B) ? 1 : -1) * ENC3_SIGN; }
void IRAM_ATTR enc4ISR() { enc4Count += (digitalRead(ENC4_B) ? 1 : -1) * ENC4_SIGN; }

enum TestPhase {
    PH_IDLE = 0,
    PH_FWD,
    PH_STOP1,
    PH_REV,
    PH_STOP2,
    PH_SPIN,
    PH_STOP3,
    PH_DONE
};

TestPhase phase = PH_IDLE;
unsigned long phaseStartMs = 0;
unsigned long lastPrintMs = 0;
long prev1 = 0, prev2 = 0, prev3 = 0, prev4 = 0;
bool seenMoveFwd = false;
bool seenMoveRev = false;
bool seenMoveSpin = false;

void printHelp() {
    Serial.println("=== ESP32 MOTOR+ENCODER TEST ===");
    Serial.println("r - run test sequence");
    Serial.println("s - stop motors");
    Serial.println("z - zero encoder counters");
    Serial.println("h - help");
}

void snapshot(long &c1, long &c2, long &c3, long &c4) {
    noInterrupts();
    c1 = enc1Count;
    c2 = enc2Count;
    c3 = enc3Count;
    c4 = enc4Count;
    interrupts();
}

void zeroCounters() {
    noInterrupts();
    enc1Count = 0;
    enc2Count = 0;
    enc3Count = 0;
    enc4Count = 0;
    interrupts();
    prev1 = prev2 = prev3 = prev4 = 0;
    Serial.println("[TEST] Counters zeroed");
}

void sendMotor(int p1, int d1, int p2, int d2) {
    char buf[40];
    snprintf(buf, sizeof(buf), "M,%d,%d,%d,%d", p1, d1, p2, d2);
    Serial2.println(buf);
}

void stopMotors() {
    Serial2.println("S");
}

void setPhase(int next) {
    phase = (TestPhase)next;
    phaseStartMs = millis();

    if (phase == PH_FWD) {
        Serial.println("[TEST] Phase FWD");
        sendMotor(120, 1, 120, 1);
        Serial2.println("B,1700,80");
    } else if (phase == PH_STOP1) {
        Serial.println("[TEST] Phase STOP1");
        stopMotors();
    } else if (phase == PH_REV) {
        Serial.println("[TEST] Phase REV");
        sendMotor(120, 0, 120, 0);
        Serial2.println("B,1700,80");
    } else if (phase == PH_STOP2) {
        Serial.println("[TEST] Phase STOP2");
        stopMotors();
    } else if (phase == PH_SPIN) {
        Serial.println("[TEST] Phase SPIN");
        sendMotor(130, 1, 130, 0);
        Serial2.println("B,2100,100");
    } else if (phase == PH_STOP3) {
        Serial.println("[TEST] Phase STOP3");
        stopMotors();
    } else if (phase == PH_DONE) {
        stopMotors();
        long c1, c2, c3, c4;
        snapshot(c1, c2, c3, c4);
        Serial.printf("[TEST] FINAL FL:%ld FR:%ld RL:%ld RR:%ld\n", c1, c2, c3, c4);

        bool pass = seenMoveFwd && seenMoveRev && seenMoveSpin;
        Serial.printf("[TEST] RESULT: %s\n", pass ? "PASS" : "FAIL");
        Serial.printf("[TEST] seenMoveFwd=%d seenMoveRev=%d seenMoveSpin=%d\n",
                      seenMoveFwd ? 1 : 0, seenMoveRev ? 1 : 0, seenMoveSpin ? 1 : 0);
        Serial.println("[TEST] Press 'r' to run again.");
    }
}

void startSequence() {
    seenMoveFwd = false;
    seenMoveRev = false;
    seenMoveSpin = false;
    zeroCounters();
    Serial2.println("E");
    delay(40);
    setPhase(PH_FWD);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, AIDUINO_RX, AIDUINO_TX);

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

    Serial.println("\n[DASN] ESP32 motor+encoder test ready");
    printHelp();
}

void loop() {
    while (Serial2.available()) {
        char c = Serial2.read();
        Serial.write(c);
    }

    if (Serial.available()) {
        char c = Serial.read();
        while (Serial.available()) Serial.read();

        if (c == 'r' || c == 'R') {
            startSequence();
        } else if (c == 's' || c == 'S') {
            setPhase(PH_IDLE);
            stopMotors();
            Serial.println("[TEST] Stopped");
        } else if (c == 'z' || c == 'Z') {
            zeroCounters();
        } else if (c == 'h' || c == 'H') {
            printHelp();
        }
    }

    if (phase == PH_IDLE || phase == PH_DONE) return;

    unsigned long now = millis();
    unsigned long elapsed = now - phaseStartMs;

    if (now - lastPrintMs >= 200) {
        lastPrintMs = now;
        long c1, c2, c3, c4;
        snapshot(c1, c2, c3, c4);

        long d1 = c1 - prev1;
        long d2 = c2 - prev2;
        long d3 = c3 - prev3;
        long d4 = c4 - prev4;

        prev1 = c1; prev2 = c2; prev3 = c3; prev4 = c4;

        long leftDelta = (d1 + d3) / 2;
        long rightDelta = (d2 + d4) / 2;

        Serial.printf("FL:%7ld (%4ld) FR:%7ld (%4ld) RL:%7ld (%4ld) RR:%7ld (%4ld) | Ld:%4ld Rd:%4ld\n",
                      c1, d1, c2, d2, c3, d3, c4, d4, leftDelta, rightDelta);

        bool moving = (labs(leftDelta) >= 2) && (labs(rightDelta) >= 2);
        if (moving) {
            if (phase == PH_FWD) seenMoveFwd = true;
            if (phase == PH_REV) seenMoveRev = true;
            if (phase == PH_SPIN) seenMoveSpin = true;
        }
    }

    if (phase == PH_FWD && elapsed >= 3000) setPhase(PH_STOP1);
    else if (phase == PH_STOP1 && elapsed >= 800) setPhase(PH_REV);
    else if (phase == PH_REV && elapsed >= 3000) setPhase(PH_STOP2);
    else if (phase == PH_STOP2 && elapsed >= 800) setPhase(PH_SPIN);
    else if (phase == PH_SPIN && elapsed >= 2500) setPhase(PH_STOP3);
    else if (phase == PH_STOP3 && elapsed >= 800) setPhase(PH_DONE);
}
