/*
 * DASN Mobile Unit - IMU + Encoder Calibration and Coordinate Drive
 * Board: ESP32 WROOM-32
 *
 * Hardware verified against the DASN mobile-unit firmware:
 *   IMU        : ISM330DHCX
 *   Mag        : MMC5983MA
 *   I2C pins   : SDA=21, SCL=22
 *   Encoders   : FL 34/32, FR 35/13, RL 25/26, RR 27/33
 *   Aiduino TX : GPIO16
 *   Aiduino RX : GPIO17
 *
 * Serial monitor: 115200
 *
 * Commands:
 *   HELP
 *   IMU
 *     Stationary accel + gyro calibration. Keep the robot still.
 *
 *   MAG
 *     Flat magnetometer calibration. Rotate the robot slowly through full
 *     headings, then press any key to finish.
 *
 *   TICKS
 *     Live stream of all four encoder tick counts until a key is pressed.
 *
 *   ENC,<meters>
 *     Manual straight-line encoder calibration.
 *     Example: ENC,1.0
 *     After sending the command, roll the robot straight by hand for the exact
 *     distance and press any key to finish.
 *
 *   GO,<x>,<y>
 *     Drives to the absolute target in meters from the current estimated pose.
 *     Use ZERO when you want to redefine the current pose as (0,0,90 deg).
 *     Example: GO,1.0,1.0
 *
 *   ZERO
 *     Reset pose and encoder baselines to zero.
 *
 *   POSE
 *     Print estimated pose.
 *
 *   STOP
 *     Emergency stop.
 */

#ifndef ARDUINO_ARCH_ESP32
#error "This sketch requires an ESP32 board selection."
#endif

#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <math.h>

// ─── Pins ──────────────────────────────────────────────────────────────────
#define ENC1_A 34
#define ENC1_B 32
#define ENC1_SIGN -1

#define ENC2_A 35
#define ENC2_B 13
#define ENC2_SIGN 1

#define ENC3_A 25
#define ENC3_B 26
#define ENC3_SIGN -1

#define ENC4_A 27
#define ENC4_B 33
#define ENC4_SIGN -1

#define PIN_SDA 21
#define PIN_SCL 22

#define AIDUINO_TX 16
#define AIDUINO_RX 17

// ─── Serial ────────────────────────────────────────────────────────────────
static const uint32_t SERIAL_BAUD_USB = 115200;
static const uint32_t SERIAL_BAUD_UART = 115200;

// ─── Robot constants ───────────────────────────────────────────────────────
static const float DEFAULT_WHEEL_DIAMETER_M = 0.034f;
static const float DEFAULT_WHEEL_CIRCUMFERENCE_M = DEFAULT_WHEEL_DIAMETER_M * PI;
static const float DEFAULT_TICKS_PER_REV = 300.0f;
static const float DEFAULT_TICKS_PER_METER_LEFT = 5610.0f;
static const float DEFAULT_TICKS_PER_METER_RIGHT = 5742.0f;
static const float TRACK_WIDTH_M = 0.16f;
static const float INITIAL_HEADING_RAD = PI * 0.5f; // facing +Y axis
static const float MAG_IDLE_FUSION_GAIN = 0.08f;

// ─── Motion control constants ──────────────────────────────────────────────
static const int TURN_PWM = 95;
static const int DRIVE_PWM = 105;
static const int MIN_DRIVE_PWM = 70;
static const float TURN_TOLERANCE_RAD = 0.10f;     // ~5.7 deg
static const float DRIVE_TOLERANCE_M = 0.03f;      // 3 cm
static const float TURN_KP = 85.0f;                // rad -> pwm
static const float HEADING_HOLD_KP = 110.0f;       // rad -> pwm split
static const unsigned long TURN_SETTLE_MS = 200;
static const unsigned long CONTROL_DT_MS = 20;
static const unsigned long TURN_TIMEOUT_MS = 8000;
static const unsigned long DRIVE_TIMEOUT_MS = 15000;
static const unsigned long TURN_STUCK_TIMEOUT_MS = 1200;
static const float TURN_PROGRESS_EPS_RAD = 0.03f;
static const int MIN_TURN_PWM = 65;
static const int MAX_TURN_PWM = 120;
static const unsigned long NAV_LOOP_MS = 20;
static const float POSITION_TOLERANCE_M = 0.05f;
static const float HEADING_TOLERANCE_RAD = 0.087f;
static const float KP_ROTATION_NAV = 150.0f;
static const float KP_DISTANCE_NAV = 200.0f;
static const float KP_STRAIGHT_LINE_NAV = 900.0f;
static const int NAV_MAX_PWM = 180;

// ─── IMU calibration constants ─────────────────────────────────────────────
static const uint16_t IMU_CAL_SAMPLES = 1000;
static const uint16_t IMU_SETTLE_MS = 2000;
static const uint8_t IMU_SAMPLE_DELAY_MS = 5;
static const float EXPECTED_GRAVITY_MG = 1000.0f;
static const uint16_t GYRO_ZERO_SAMPLES = 500;
static const uint16_t GYRO_ZERO_SETTLE_MS = 1200;
static const uint8_t GYRO_ZERO_SAMPLE_DELAY_MS = 4;

// ─── Global state ──────────────────────────────────────────────────────────
SparkFun_ISM330DHCX imu;
SFE_MMC5983MA mag;
bool imuReady = false;
bool magReady = false;

volatile long enc1Count = 0;
volatile long enc2Count = 0;
volatile long enc3Count = 0;
volatile long enc4Count = 0;

float accelOffsetXMg = 0.0f;
float accelOffsetYMg = 0.0f;
float accelOffsetZMg = 0.0f;
float gyroBiasXMdps = 0.0f;
float gyroBiasYMdps = 0.0f;
float gyroBiasZMdps = 0.0f;
float magOffsetXG = -0.250f;
float magOffsetYG = -0.016f;
float magOffsetZG = -0.179f;
float magScaleX = 0.677f;
float magScaleY = 0.677f;
float magScaleZ = 22.322f;

float ticksPerMeterLeft = DEFAULT_TICKS_PER_METER_LEFT;
float ticksPerMeterRight = DEFAULT_TICKS_PER_METER_RIGHT;

float poseX = 0.0f;
float poseY = 0.0f;
float poseHeading = 0.0f;

long prevLeftTicks = 0;
long prevRightTicks = 0;
unsigned long lastOdomMicros = 0;
float magHeadingOffsetRad = 0.0f;
bool magHeadingOffsetValid = false;

#define CMD_BUF_SIZE 96
char cmdBuf[CMD_BUF_SIZE];
uint8_t cmdIdx = 0;

enum NavState {
  NAV_IDLE = 0,
  NAV_ROTATING,
  NAV_MOVING,
  NAV_ARRIVED,
  NAV_ERROR
};

NavState navState = NAV_IDLE;
float navTargetX = 0.0f;
float navTargetY = 0.0f;
float navTargetHeading = 0.0f;
float navTargetDistance = 0.0f;
long navDriveStartLeftTicks = 0;
long navDriveStartRightTicks = 0;
float navDistanceToTravel = 0.0f;
unsigned long navArrivalAtMs = 0;
unsigned long lastNavUpdateMs = 0;
static const uint8_t NAV_MAX_WAYPOINTS = 4;
float navWaypointX[NAV_MAX_WAYPOINTS];
float navWaypointY[NAV_MAX_WAYPOINTS];
uint8_t navWaypointCount = 0;
uint8_t navWaypointIndex = 0;

// ─── ISR ───────────────────────────────────────────────────────────────────
void IRAM_ATTR enc1ISR() { enc1Count += (digitalRead(ENC1_B) ? 1 : -1) * ENC1_SIGN; }
void IRAM_ATTR enc2ISR() { enc2Count += (digitalRead(ENC2_B) ? 1 : -1) * ENC2_SIGN; }
void IRAM_ATTR enc3ISR() { enc3Count += (digitalRead(ENC3_B) ? 1 : -1) * ENC3_SIGN; }
void IRAM_ATTR enc4ISR() { enc4Count += (digitalRead(ENC4_B) ? 1 : -1) * ENC4_SIGN; }

// ─── Helpers ───────────────────────────────────────────────────────────────
void flushSerialInput() {
  while (Serial.available()) {
    Serial.read();
  }
}

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

float normalizeAngle(float angleRad) {
  while (angleRad > PI) angleRad -= 2.0f * PI;
  while (angleRad < -PI) angleRad += 2.0f * PI;
  return angleRad;
}

void snapshotEncoders(long &fl, long &fr, long &rl, long &rr) {
  noInterrupts();
  fl = enc1Count;
  fr = enc2Count;
  rl = enc3Count;
  rr = enc4Count;
  interrupts();
}

long leftTicksNow() {
  long fl, fr, rl, rr;
  snapshotEncoders(fl, fr, rl, rr);
  return (fl + rl) / 2;
}

long rightTicksNow() {
  long fl, fr, rl, rr;
  snapshotEncoders(fl, fr, rl, rr);
  return (fr + rr) / 2;
}

void zeroEncoders() {
  noInterrupts();
  enc1Count = 0;
  enc2Count = 0;
  enc3Count = 0;
  enc4Count = 0;
  interrupts();
}

bool initIMU() {
  if (imuReady) {
    return true;
  }

  if (!imu.begin()) {
    imuReady = false;
    return false;
  }

  imu.setDeviceConfig();
  imu.setBlockDataUpdate();
  imu.setAccelDataRate(ISM_XL_ODR_104Hz);
  imu.setAccelFullScale(ISM_4g);
  imu.setGyroDataRate(ISM_GY_ODR_104Hz);
  imu.setGyroFullScale(ISM_500dps);

  imuReady = true;
  return true;
}

float rawMagToGauss(uint32_t rawValue) {
  return ((float)rawValue - 131072.0f) / 131072.0f * 8.0f;
}

bool initMag() {
  if (magReady) {
    return true;
  }

  if (!mag.begin()) {
    magReady = false;
    return false;
  }

  mag.softReset();
  delay(10);
  mag.setFilterBandwidth(800);
  mag.setContinuousModeFrequency(100);
  mag.enableContinuousMode();

  magReady = true;
  return true;
}

bool readCalibratedMag(float &mx, float &my, float &mz) {
  if (!initMag()) {
    return false;
  }

  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);

  const float mxRaw = rawMagToGauss(rawX);
  const float myRaw = rawMagToGauss(rawY);
  const float mzRaw = rawMagToGauss(rawZ);

  mx = (mxRaw - magOffsetXG) * magScaleX;
  my = (myRaw - magOffsetYG) * magScaleY;
  mz = (mzRaw - magOffsetZG) * magScaleZ;
  return true;
}

void captureMagHeadingReference() {
  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;
  if (!readCalibratedMag(mx, my, mz)) {
    magHeadingOffsetValid = false;
    return;
  }

  const float horiz = sqrtf(mx * mx + my * my);
  if (horiz < 0.05f) {
    magHeadingOffsetValid = false;
    return;
  }

  const float absoluteHeading = normalizeAngle(atan2f(my, mx));
  magHeadingOffsetRad = normalizeAngle(INITIAL_HEADING_RAD - absoluteHeading);
  magHeadingOffsetValid = true;
}

bool readLocalMagHeading(float &headingRad) {
  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;
  if (!readCalibratedMag(mx, my, mz) || !magHeadingOffsetValid) {
    return false;
  }

  const float horiz = sqrtf(mx * mx + my * my);
  if (horiz < 0.05f) {
    return false;
  }

  const float absoluteHeading = normalizeAngle(atan2f(my, mx));
  headingRad = normalizeAngle(absoluteHeading + magHeadingOffsetRad);
  return true;
}

bool calibrateGyroBiasQuick(bool verbose) {
  if (!initIMU()) {
    if (verbose) {
      Serial.println("Gyro calibration failed: IMU not available.");
    }
    return false;
  }

  if (verbose) {
    Serial.println("Gyro zero calibration starting. Keep the robot still.");
  }

  delay(GYRO_ZERO_SETTLE_MS);

  float gxSum = 0.0f;
  float gySum = 0.0f;
  float gzSum = 0.0f;
  uint16_t samples = 0;

  while (samples < GYRO_ZERO_SAMPLES) {
    if (imu.checkStatus()) {
      sfe_ism_data_t gyro;
      imu.getGyro(&gyro);
      gxSum += gyro.xData;
      gySum += gyro.yData;
      gzSum += gyro.zData;
      samples++;
    }
    delay(GYRO_ZERO_SAMPLE_DELAY_MS);
  }

  if (samples == 0) {
    if (verbose) {
      Serial.println("Gyro zero calibration failed: no samples.");
    }
    return false;
  }

  gyroBiasXMdps = gxSum / (float)samples;
  gyroBiasYMdps = gySum / (float)samples;
  gyroBiasZMdps = gzSum / (float)samples;

  if (verbose) {
    Serial.printf("Gyro zero complete: x=%.3f y=%.3f z=%.3f mdps\n",
                  gyroBiasXMdps, gyroBiasYMdps, gyroBiasZMdps);
  }

  return true;
}

void initEncoders() {
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
}

void enableMotors() {
  Serial2.println("E");
  delay(40);
}

void stopMotors() {
  Serial2.println("S");
}

void disableMotors() {
  Serial2.println("D");
}

void sendMotorCommand(int leftPwm, int leftDir, int rightPwm, int rightDir) {
  char buf[40];
  snprintf(buf, sizeof(buf), "M,%d,%d,%d,%d", leftPwm, leftDir, rightPwm, rightDir);
  Serial2.println(buf);
}

void setForwardPWM(int leftPwm, int rightPwm) {
  leftPwm = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);
  sendMotorCommand(leftPwm, 1, rightPwm, 1);
}

void setDrivePWM(int leftPwm, int rightPwm, bool forward) {
  leftPwm = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);
  sendMotorCommand(leftPwm, forward ? 1 : 0, rightPwm, forward ? 1 : 0);
}

void setTurnPWM(float signedPwm) {
  int pwm = constrain((int)fabsf(signedPwm), 0, MAX_TURN_PWM);
  if (pwm == 0) {
    stopMotors();
    return;
  }

  // Positive turn command means positive yaw / CCW.
  if (signedPwm > 0.0f) {
    sendMotorCommand(pwm, 0, pwm, 1);
  } else {
    sendMotorCommand(pwm, 1, pwm, 0);
  }
}

void resetPoseAndOdometry() {
  zeroEncoders();
  poseX = 0.0f;
  poseY = 0.0f;
  poseHeading = INITIAL_HEADING_RAD;
  prevLeftTicks = 0;
  prevRightTicks = 0;
  lastOdomMicros = micros();
  captureMagHeadingReference();
}

void printPose() {
  Serial.printf("POSE x=%.3f m y=%.3f m heading=%.2f deg state=%s\n",
                poseX, poseY, poseHeading * 180.0f / PI, navStateName());
}

const char *navStateName() {
  switch (navState) {
    case NAV_IDLE: return "IDLE";
    case NAV_ROTATING: return "ROTATING";
    case NAV_MOVING: return "MOVING";
    case NAV_ARRIVED: return "ARRIVED";
    case NAV_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void updateOdometry() {
  const unsigned long nowMicros = micros();
  float dt = 0.0f;

  if (lastOdomMicros != 0) {
    dt = (nowMicros - lastOdomMicros) * 1e-6f;
  }
  lastOdomMicros = nowMicros;

  if (imuReady && imu.checkStatus() && dt > 0.0f && dt < 0.1f) {
    sfe_ism_data_t gyro;
    imu.getGyro(&gyro);
    const float gzDps = (gyro.zData - gyroBiasZMdps) / 1000.0f;
    poseHeading = normalizeAngle(poseHeading + gzDps * (PI / 180.0f) * dt);
  }

  // The magnetometer gets disturbed by motors, wiring, and battery current
  // while the robot is moving. Use it only while idle/arrived to gently
  // re-anchor heading, and rely on the gyro during active navigation.
  float magHeading = 0.0f;
  if ((navState == NAV_IDLE || navState == NAV_ARRIVED) && readLocalMagHeading(magHeading)) {
    const float magError = normalizeAngle(magHeading - poseHeading);
    poseHeading = normalizeAngle(poseHeading + MAG_IDLE_FUSION_GAIN * magError);
  }

  const long left = leftTicksNow();
  const long right = rightTicksNow();
  const long deltaLeftTicks = left - prevLeftTicks;
  const long deltaRightTicks = right - prevRightTicks;

  prevLeftTicks = left;
  prevRightTicks = right;

  const float dLeft = (ticksPerMeterLeft > 0.0f) ? ((float)deltaLeftTicks / ticksPerMeterLeft) : 0.0f;
  const float dRight = (ticksPerMeterRight > 0.0f) ? ((float)deltaRightTicks / ticksPerMeterRight) : 0.0f;
  const float dCenter = 0.5f * (dLeft + dRight);

  poseX += dCenter * cosf(poseHeading);
  poseY += dCenter * sinf(poseHeading);
}

float navCalculateTargetHeading(float tx, float ty) {
  const float dx = tx - poseX;
  const float dy = ty - poseY;
  return atan2f(dy, dx);
}

float navCalculateTargetDistance(float tx, float ty) {
  const float dx = tx - poseX;
  const float dy = ty - poseY;
  return sqrtf(dx * dx + dy * dy);
}

void navStopAndIdle() {
  stopMotors();
  disableMotors();
  navState = NAV_IDLE;
  navArrivalAtMs = 0;
  navWaypointCount = 0;
  navWaypointIndex = 0;
}

void navStartWaypoint(uint8_t index) {
  if (index >= navWaypointCount) {
    navStopAndIdle();
    return;
  }

  updateOdometry();

  navWaypointIndex = index;
  navTargetX = navWaypointX[index];
  navTargetY = navWaypointY[index];
  navTargetHeading = navCalculateTargetHeading(navTargetX, navTargetY);
  navTargetDistance = navCalculateTargetDistance(navTargetX, navTargetY);

  if (navTargetDistance <= POSITION_TOLERANCE_M) {
    if (index + 1 < navWaypointCount) {
      navStartWaypoint(index + 1);
    } else {
      Serial.println("[nav] target already within tolerance.");
      navStopAndIdle();
    }
    return;
  }

  enableMotors();
  navState = NAV_ROTATING;
  navArrivalAtMs = 0;

  Serial.printf("[nav] wp %u/%u target=(%.3f, %.3f) heading=%.1f deg dist=%.3f m\n",
                (unsigned)(index + 1), (unsigned)navWaypointCount,
                navTargetX, navTargetY, navTargetHeading * 180.0f / PI, navTargetDistance);
}

void navPlanAndStart(float tx, float ty) {
  updateOdometry();

  navWaypointCount = 0;
  navWaypointIndex = 0;

  const float currentX = poseX;
  const float currentY = poseY;

  if (fabsf(ty - currentY) > POSITION_TOLERANCE_M && navWaypointCount < NAV_MAX_WAYPOINTS) {
    navWaypointX[navWaypointCount] = currentX;
    navWaypointY[navWaypointCount] = ty;
    navWaypointCount++;
  }

  if (fabsf(tx - currentX) > POSITION_TOLERANCE_M && navWaypointCount < NAV_MAX_WAYPOINTS) {
    navWaypointX[navWaypointCount] = tx;
    navWaypointY[navWaypointCount] = ty;
    navWaypointCount++;
  }

  if (navWaypointCount == 0) {
    Serial.println("Target already within tolerance.");
    navStopAndIdle();
    return;
  }

  Serial.printf("[nav] planned %u leg(s)\n", (unsigned)navWaypointCount);
  navStartWaypoint(0);
}

void navUpdate() {
  updateOdometry();

  switch (navState) {
    case NAV_IDLE:
      break;

    case NAV_ROTATING: {
      const float headingError = normalizeAngle(navTargetHeading - poseHeading);

      if (fabsf(headingError) < HEADING_TOLERANCE_RAD) {
        stopMotors();
        navDriveStartLeftTicks = leftTicksNow();
        navDriveStartRightTicks = rightTicksNow();
        navDistanceToTravel = navCalculateTargetDistance(navTargetX, navTargetY);
        navState = NAV_MOVING;
        Serial.printf("[nav] rotate done, driving %.3f m\n", navDistanceToTravel);
        break;
      }

      float turnPWM = headingError * KP_ROTATION_NAV;
      turnPWM = clampFloat(turnPWM, -NAV_MAX_PWM, NAV_MAX_PWM);

      if (fabsf(turnPWM) > 0.0f && fabsf(turnPWM) < MIN_TURN_PWM) {
        turnPWM = (turnPWM > 0.0f) ? MIN_TURN_PWM : -MIN_TURN_PWM;
      }

      setTurnPWM(turnPWM);
      break;
    }

    case NAV_MOVING: {
      const long leftCount = leftTicksNow() - navDriveStartLeftTicks;
      const long rightCount = rightTicksNow() - navDriveStartRightTicks;

      const float distLeft = (ticksPerMeterLeft > 0.0f) ? ((float)leftCount / ticksPerMeterLeft) : 0.0f;
      const float distRight = (ticksPerMeterRight > 0.0f) ? ((float)rightCount / ticksPerMeterRight) : 0.0f;
      const float distanceTraveled = 0.5f * (distLeft + distRight);
      const float remaining = navDistanceToTravel - distanceTraveled;

      if (remaining <= POSITION_TOLERANCE_M) {
        stopMotors();
        poseX = navTargetX;
        poseY = navTargetY;
        navState = NAV_ARRIVED;
        navArrivalAtMs = millis();
        Serial.printf("[nav] arrived at target (%.3f, %.3f)\n", navTargetX, navTargetY);
        break;
      }

      float basePWM = remaining * KP_DISTANCE_NAV;
      basePWM = clampFloat(basePWM, MIN_DRIVE_PWM, NAV_MAX_PWM);

      const float distanceDiff = distLeft - distRight;
      float correction = distanceDiff * KP_STRAIGHT_LINE_NAV;
      correction = clampFloat(correction, -40.0f, 40.0f);

      int leftPWM = constrain((int)(basePWM - correction), 0, NAV_MAX_PWM);
      int rightPWM = constrain((int)(basePWM + correction), 0, NAV_MAX_PWM);
      setForwardPWM(leftPWM, rightPWM);
      break;
    }

    case NAV_ARRIVED:
      if (millis() - navArrivalAtMs > 200) {
        if (navWaypointIndex + 1 < navWaypointCount) {
          navStartWaypoint(navWaypointIndex + 1);
        } else {
          navStopAndIdle();
          Serial.printf("[nav] complete; pose x=%.3f y=%.3f heading=%.2f deg\n",
                        poseX, poseY, poseHeading * 180.0f / PI);
        }
      }
      break;

    case NAV_ERROR:
      stopMotors();
      break;
  }
}

void printHelp() {
  Serial.println();
  Serial.println("=== DASN IMU + ENCODER TOOL ===");
  Serial.println("IMU");
  Serial.println("MAG");
  Serial.println("TICKS");
  Serial.println("ENC,<meters>");
  Serial.println("GO,<x>,<y>");
  Serial.println("ZERO");
  Serial.println("POSE");
  Serial.println("STOP");
  Serial.println("HELP");
  Serial.println();
  Serial.println("Global frame assumption: start pose is x=0, y=0, heading=90 deg (+Y axis).");
  Serial.println("ZERO reruns gyro zeroing and defines the current robot pose as that frame origin.");
  Serial.println("GO uses the current estimated pose and moves to the absolute target.");
  Serial.println("GO plans axis-aligned legs (Y first, then X) to reduce diagonal heading error.");
  Serial.printf("Current ticks/m: left=%.3f right=%.3f\n", ticksPerMeterLeft, ticksPerMeterRight);
  Serial.printf("Current mag offsets/scales: off=(%.3f, %.3f, %.3f) scale=(%.3f, %.3f, %.3f)\n",
                magOffsetXG, magOffsetYG, magOffsetZG, magScaleX, magScaleY, magScaleZ);
}

void streamEncoderTicks() {
  Serial.println("Live encoder ticks. Press any key to stop.");
  flushSerialInput();

  long prevFl = 0;
  long prevFr = 0;
  long prevRl = 0;
  long prevRr = 0;
  snapshotEncoders(prevFl, prevFr, prevRl, prevRr);

  while (!Serial.available()) {
    long fl = 0;
    long fr = 0;
    long rl = 0;
    long rr = 0;
    snapshotEncoders(fl, fr, rl, rr);

    Serial.printf(
      "FL:%8ld (%4ld)  FR:%8ld (%4ld)  RL:%8ld (%4ld)  RR:%8ld (%4ld)  | L:%8ld R:%8ld\n",
      fl, fl - prevFl,
      fr, fr - prevFr,
      rl, rl - prevRl,
      rr, rr - prevRr,
      (fl + rl) / 2,
      (fr + rr) / 2
    );

    prevFl = fl;
    prevFr = fr;
    prevRl = rl;
    prevRr = rr;
    delay(200);
  }

  flushSerialInput();
}

void calibrateIMU() {
  Serial.println("IMU calibration starting.");
  Serial.println("Place the robot flat and keep it completely still.");

  if (!initIMU()) {
    Serial.println("IMU init failed. Check ISM330DHCX on SDA=21 SCL=22.");
    return;
  }

  flushSerialInput();
  delay(IMU_SETTLE_MS);

  float axSum = 0.0f;
  float aySum = 0.0f;
  float azSum = 0.0f;
  uint16_t samples = 0;

  while (samples < IMU_CAL_SAMPLES) {
    if (imu.checkStatus()) {
      sfe_ism_data_t accel;
      imu.getAccel(&accel);

      axSum += accel.xData;
      aySum += accel.yData;
      azSum += accel.zData;
      samples++;
    }
    delay(IMU_SAMPLE_DELAY_MS);
  }

  const float axAvg = axSum / (float)samples;
  const float ayAvg = aySum / (float)samples;
  const float azAvg = azSum / (float)samples;

  accelOffsetXMg = axAvg;
  accelOffsetYMg = ayAvg;
  accelOffsetZMg = azAvg - EXPECTED_GRAVITY_MG;
  calibrateGyroBiasQuick(false);

  Serial.println("IMU calibration complete.");
  Serial.printf("const float IMU_ACCEL_OFFSET_X_MG = %.3ff;\n", accelOffsetXMg);
  Serial.printf("const float IMU_ACCEL_OFFSET_Y_MG = %.3ff;\n", accelOffsetYMg);
  Serial.printf("const float IMU_ACCEL_OFFSET_Z_MG = %.3ff;\n", accelOffsetZMg);
  Serial.printf("const float IMU_GYRO_OFFSET_X_MDPS = %.3ff;\n", gyroBiasXMdps);
  Serial.printf("const float IMU_GYRO_OFFSET_Y_MDPS = %.3ff;\n", gyroBiasYMdps);
  Serial.printf("const float IMU_GYRO_OFFSET_Z_MDPS = %.3ff;\n", gyroBiasZMdps);
}

void calibrateMagnetometer() {
  Serial.println("Magnetometer calibration starting.");
  Serial.println("Keep the robot level and rotate it slowly through full 360 deg headings.");
  Serial.println("Press any key when finished.");

  if (!initMag()) {
    Serial.println("Mag init failed. Check MMC5983MA on SDA=21 SCL=22.");
    return;
  }

  flushSerialInput();

  uint32_t rawX = 0;
  uint32_t rawY = 0;
  uint32_t rawZ = 0;
  mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);
  float mx = rawMagToGauss(rawX);
  float my = rawMagToGauss(rawY);
  float mz = rawMagToGauss(rawZ);

  float minX = mx;
  float maxX = mx;
  float minY = my;
  float maxY = my;
  float minZ = mz;
  float maxZ = mz;
  unsigned long lastPrintMs = 0;

  while (!Serial.available()) {
    mag.getMeasurementXYZ(&rawX, &rawY, &rawZ);
    mx = rawMagToGauss(rawX);
    my = rawMagToGauss(rawY);
    mz = rawMagToGauss(rawZ);

    if (mx < minX) minX = mx;
    if (mx > maxX) maxX = mx;
    if (my < minY) minY = my;
    if (my > maxY) maxY = my;
    if (mz < minZ) minZ = mz;
    if (mz > maxZ) maxZ = mz;

    const unsigned long now = millis();
    if (now - lastPrintMs >= 250) {
      lastPrintMs = now;
      Serial.printf("MAG live X[%6.3f,%6.3f] Y[%6.3f,%6.3f] Z[%6.3f,%6.3f]\n",
                    minX, maxX, minY, maxY, minZ, maxZ);
    }
    delay(20);
  }
  flushSerialInput();

  magOffsetXG = 0.5f * (minX + maxX);
  magOffsetYG = 0.5f * (minY + maxY);
  magOffsetZG = 0.5f * (minZ + maxZ);

  const float radiusX = 0.5f * (maxX - minX);
  const float radiusY = 0.5f * (maxY - minY);
  const float radiusZ = 0.5f * (maxZ - minZ);
  const float avgRadius = (radiusX + radiusY + radiusZ) / 3.0f;

  magScaleX = (radiusX > 0.001f) ? (avgRadius / radiusX) : 1.0f;
  magScaleY = (radiusY > 0.001f) ? (avgRadius / radiusY) : 1.0f;
  magScaleZ = (radiusZ > 0.001f) ? (avgRadius / radiusZ) : 1.0f;

  Serial.println("Magnetometer calibration complete.");
  Serial.printf("const float MAG_OFFSET_X_G = %.3ff;\n", magOffsetXG);
  Serial.printf("const float MAG_OFFSET_Y_G = %.3ff;\n", magOffsetYG);
  Serial.printf("const float MAG_OFFSET_Z_G = %.3ff;\n", magOffsetZG);
  Serial.printf("const float MAG_SCALE_X = %.3ff;\n", magScaleX);
  Serial.printf("const float MAG_SCALE_Y = %.3ff;\n", magScaleY);
  Serial.printf("const float MAG_SCALE_Z = %.3ff;\n", magScaleZ);

  captureMagHeadingReference();
  if (magHeadingOffsetValid) {
    Serial.printf("Mag local heading offset set to %.2f deg\n", magHeadingOffsetRad * 180.0f / PI);
  } else {
    Serial.println("Warning: mag heading reference could not be set.");
  }
}

void calibrateEncodersForDistance(float knownDistanceMeters) {
  if (knownDistanceMeters <= 0.0f) {
    Serial.println("Known distance must be > 0.");
    return;
  }

  Serial.printf("Encoder calibration starting for %.3f m.\n", knownDistanceMeters);
  Serial.println("Align the robot straight.");
  Serial.println("Roll it by hand for the exact distance, then press any key.");

  stopMotors();
  disableMotors();
  resetPoseAndOdometry();
  flushSerialInput();

  unsigned long lastPrintMs = 0;
  while (!Serial.available()) {
    const unsigned long now = millis();
    if (now - lastPrintMs >= 250) {
      lastPrintMs = now;
      const long left = leftTicksNow();
      const long right = rightTicksNow();
      Serial.printf("Live ticks left=%ld right=%ld\n", left, right);
    }
    delay(10);
  }
  flushSerialInput();

  const long left = leftTicksNow();
  const long right = rightTicksNow();
  const float leftAbs = fabsf((float)left);
  const float rightAbs = fabsf((float)right);

  if (leftAbs < 1.0f || rightAbs < 1.0f) {
    Serial.println("Not enough encoder ticks captured. Check wiring and try again.");
    return;
  }

  ticksPerMeterLeft = leftAbs / knownDistanceMeters;
  ticksPerMeterRight = rightAbs / knownDistanceMeters;

  Serial.println("Encoder distance calibration complete.");
  Serial.printf("Measured ticks: left=%ld right=%ld\n", left, right);
  Serial.printf("ticksPerMeterLeft  = %.3f\n", ticksPerMeterLeft);
  Serial.printf("ticksPerMeterRight = %.3f\n", ticksPerMeterRight);
  Serial.printf("distancePerTickLeft  = %.6f m\n", 1.0f / ticksPerMeterLeft);
  Serial.printf("distancePerTickRight = %.6f m\n", 1.0f / ticksPerMeterRight);

  const float mismatch = fabsf(ticksPerMeterLeft - ticksPerMeterRight) /
                         ((ticksPerMeterLeft + ticksPerMeterRight) * 0.5f);
  if (mismatch > 0.10f) {
    Serial.println("Warning: left/right encoder distance calibration differs by more than 10%.");
  }
}

bool turnToHeading(float targetHeadingRad) {
  const unsigned long startedAt = millis();
  unsigned long settledAt = 0;
  unsigned long lastProgressAt = millis();
  float bestAbsError = 1000.0f;

  while (millis() - startedAt < TURN_TIMEOUT_MS) {
    updateOdometry();

    const float error = normalizeAngle(targetHeadingRad - poseHeading);
    const float absError = fabsf(error);

    if (absError + TURN_PROGRESS_EPS_RAD < bestAbsError) {
      bestAbsError = absError;
      lastProgressAt = millis();
    }

    if (absError <= TURN_TOLERANCE_RAD) {
      if (settledAt == 0) {
        settledAt = millis();
      }
      stopMotors();
      if (millis() - settledAt >= TURN_SETTLE_MS) {
        return true;
      }
    } else {
      settledAt = 0;
      float turnCmd = TURN_KP * error;
      if (fabsf(turnCmd) < MIN_TURN_PWM) {
        turnCmd = (turnCmd >= 0.0f) ? MIN_TURN_PWM : -MIN_TURN_PWM;
      }
      turnCmd = clampFloat(turnCmd, -MAX_TURN_PWM, MAX_TURN_PWM);
      setTurnPWM(turnCmd);
    }

    if (millis() - lastProgressAt > TURN_STUCK_TIMEOUT_MS) {
      stopMotors();
      Serial.printf("Turn failed: heading error stuck at %.2f deg\n", absError * 180.0f / PI);
      return false;
    }

    delay(CONTROL_DT_MS);
  }

  stopMotors();
  Serial.println("Turn timeout.");
  return false;
}

bool driveStraightDistance(float distanceMeters, float headingHoldRad, bool forward) {
  const unsigned long startedAt = millis();
  const float startX = poseX;
  const float startY = poseY;

  while (millis() - startedAt < DRIVE_TIMEOUT_MS) {
    updateOdometry();

    const float dx = poseX - startX;
    const float dy = poseY - startY;
    const float traveled = sqrtf(dx * dx + dy * dy);
    const float remaining = distanceMeters - traveled;

    if (remaining <= DRIVE_TOLERANCE_M) {
      stopMotors();
      return true;
    }

    const float headingError = normalizeAngle(headingHoldRad - poseHeading);
    int basePwm = DRIVE_PWM;
    if (remaining < 0.25f) {
      basePwm = MIN_DRIVE_PWM;
    }

    const float correction = HEADING_HOLD_KP * headingError;
    const int leftPwm = constrain((int)(basePwm - correction), MIN_DRIVE_PWM, 200);
    const int rightPwm = constrain((int)(basePwm + correction), MIN_DRIVE_PWM, 200);
    setDrivePWM(leftPwm, rightPwm, forward);

    delay(CONTROL_DT_MS);
  }

  stopMotors();
  Serial.println("Drive timeout.");
  return false;
}

void goToXY(float targetX, float targetY) {
  if (!initIMU()) {
    Serial.println("IMU init failed. Cannot run GO command.");
    return;
  }
  Serial.printf("GO command queued for target=(%.3f, %.3f)\n", targetX, targetY);
  navPlanAndStart(targetX, targetY);
}

void processCommand(const char *cmd) {
  float a = 0.0f;
  float b = 0.0f;

  if (strcmp(cmd, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcmp(cmd, "IMU") == 0) {
    calibrateIMU();
    return;
  }

  if (strcmp(cmd, "MAG") == 0) {
    calibrateMagnetometer();
    return;
  }

  if (strcmp(cmd, "TICKS") == 0) {
    streamEncoderTicks();
    return;
  }

  if (strcmp(cmd, "ZERO") == 0) {
    navStopAndIdle();
    calibrateGyroBiasQuick(true);
    resetPoseAndOdometry();
    Serial.println("Pose reset to x=0, y=0, heading=90 deg. Encoders zeroed.");
    return;
  }

  if (strcmp(cmd, "POSE") == 0) {
    updateOdometry();
    printPose();
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    navStopAndIdle();
    Serial.println("Navigation stopped.");
    return;
  }

  if (sscanf(cmd, "ENC,%f", &a) == 1) {
    calibrateEncodersForDistance(a);
    return;
  }

  if (sscanf(cmd, "GO,%f,%f", &a, &b) == 2) {
    goToXY(a, b);
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(cmd);
  printHelp();
}

void setup() {
  Serial.begin(SERIAL_BAUD_USB);
  Serial2.begin(SERIAL_BAUD_UART, SERIAL_8N1, AIDUINO_RX, AIDUINO_TX);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  initEncoders();

  Serial.println();
  Serial.println("[DASN] ESP32 IMU + encoder calibration tool");
  if (initIMU()) {
    Serial.println("[DASN] ISM330DHCX ready");
  } else {
    Serial.println("[DASN] ISM330DHCX not found");
  }
  if (initMag()) {
    Serial.println("[DASN] MMC5983MA ready");
  } else {
    Serial.println("[DASN] MMC5983MA not found");
  }

  resetPoseAndOdometry();
  printHelp();
}

void loop() {
  const unsigned long now = millis();
  if (now - lastNavUpdateMs >= NAV_LOOP_MS) {
    lastNavUpdateMs = now;
    navUpdate();
  }

  while (Serial.available()) {
    const char c = Serial.read();
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
}
