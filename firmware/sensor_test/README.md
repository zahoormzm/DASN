# DASN Mobile Unit Test Sketches

## Files
- `mobile_unit_esp32_sensor_test/mobile_unit_esp32_sensor_test.ino`  
  Tests ESP32-side sensors: IMU, magnetometer, INA219, ToF, ultrasonic, and manual encoder spin.
- `mobile_unit_aiduino_motor_test/mobile_unit_aiduino_motor_test.ino`  
  Tests Aiduino motor outputs, buzzer, OLED, watchdog, and serial command parser.
- `mobile_unit_esp32_motor_encoder_test/mobile_unit_esp32_motor_encoder_test.ino`  
  Runs motors through Aiduino and verifies encoder deltas while moving.
- `sensor_test.ino`  
  Legacy all-in-one sketch (kept for backward compatibility).

Important: Arduino IDE compiles all `.ino` files in the same sketch folder together.  
Each new test is in its own subfolder so they compile independently.

## Motor + Encoder Validation Flow
1. Flash Aiduino with `mobile_unit_aiduino_motor_test/mobile_unit_aiduino_motor_test.ino`.
2. Flash ESP32 with `mobile_unit_esp32_motor_encoder_test/mobile_unit_esp32_motor_encoder_test.ino`.
3. Open ESP32 serial monitor at `115200`.
4. Send `r` to run the full sequence.
5. Confirm per-wheel counts (`FL/FR/RL/RR`) and deltas change during FWD/REV/SPIN phases.
6. Check final line prints `RESULT: PASS`.

## Sensor Validation Flow
1. Flash ESP32 with `mobile_unit_esp32_sensor_test/mobile_unit_esp32_sensor_test.ino`.
2. Open serial monitor at `115200`.
3. Run menu items in order: `1,2,3,4,5,6,7,8`.

## Required Wiring For Combined Motor/Encoder Test
- ESP32 `TX2 GPIO16` -> Aiduino `RX`
- ESP32 `RX2 GPIO17` <- Aiduino `TX`
- Common `GND` between ESP32 and Aiduino
