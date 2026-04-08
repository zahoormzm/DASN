# DASN Demo Firmware Test Sketches

This folder now keeps only the sketches that matter for the pivoted presentation stack.

## Kept Files
- `sentinel_type_a_pivot_test/sentinel_type_a_pivot_test.ino`
  Hardware test sketch for the fixed Sentinel Type A ESP32 node.
- `sentinel_type_a_pivot_test/config.h`
  Local pin map for the pivot test sketch.
- `mobile_unit_esp32_imu_encoder_calibration/mobile_unit_esp32_imu_encoder_calibration.ino`
  Mobile bot motion / pose calibration sketch used by the new demo bot bridge.

## Sentinel Type A Pivot Validation Flow
1. Flash `sentinel_type_a_pivot_test/sentinel_type_a_pivot_test.ino`.
2. Install Arduino libraries `Adafruit BME680`, `LiquidCrystal_I2C`, and `SparkFun VL53L5CX`.
3. Open Serial Monitor at `115200`.
4. Run menu items in order: `1,2,3,4,5,6,7`.
5. If the sound sensor pins differ from your build, edit `sentinel_type_a_pivot_test/config.h` first.

## Mobile Bot Calibration Flow
1. Flash `mobile_unit_esp32_imu_encoder_calibration/mobile_unit_esp32_imu_encoder_calibration.ino`.
2. Open Serial Monitor at `115200`.
3. Run `IMU`, `MAG`, and `ZERO` before using the bot in the demo.
4. Use `GO,x,y`, `POSE`, and `STOP` to verify the bot responds before launching ROS.
