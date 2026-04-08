# DASN

Distributed Autonomous Security Network is now trimmed to the active presentation stack:

- **Laptop** as the main server and web dashboard
- **Raspberry Pi 5** as the fixed Door A camera node
- **ESP32 Sentinel Type A** as the fixed sensor/display node
- **Mobile bot** with phone video/audio and a direct serial motion bridge

## Active Repository Layout

```text
DASN/
├── dasn_ws/
│   └── src/
│       ├── dasn_msgs/
│       ├── dasn_perception/
│       ├── dasn_sentinel/
│       ├── dasn_navigation/
│       ├── dasn_dashboard/
│       └── dasn_bringup/
├── docs/
│   └── PRESENTATION_SETUP.md
└── firmware/
    ├── sentinel_type_a/
    └── sensor_test/
```

## Active ROS Packages

- `dasn_msgs`: shared message definitions
- `dasn_perception`: RP5 RTSP ingest, bot phone ingest, object detection, face detection
- `dasn_sentinel`: Wi-Fi receiver and central security controller
- `dasn_navigation`: direct bot bridge for the IMU/encoder calibration sketch
- `dasn_dashboard`: phone speaker / local speech alert node
- `dasn_bringup`: launch files and active config

## Active Firmware

- [sentinel_type_a.ino](/Users/zahoormashahir/Documents/Projects/DASN/firmware/sentinel_type_a/sentinel_type_a.ino)
  Fixed node runtime with:
  - SmartElex sound sensor
  - BME688
  - CAP1203
  - LCD + LEDs
  - `VL53L5CX` doorway approach sensing
- [sentinel_type_a_pivot_test.ino](/Users/zahoormashahir/Documents/Projects/DASN/firmware/sensor_test/sentinel_type_a_pivot_test/sentinel_type_a_pivot_test.ino)
  Arduino IDE hardware test sketch for the fixed node
- [mobile_unit_esp32_imu_encoder_calibration.ino](/Users/zahoormashahir/Documents/Projects/DASN/firmware/sensor_test/mobile_unit_esp32_imu_encoder_calibration/mobile_unit_esp32_imu_encoder_calibration.ino)
  Bot motion / pose command sketch used by the demo bot bridge

## Demo Flow

1. RP5 streams Door A video to the laptop.
2. Bot phone streams video/audio to the laptop.
3. Laptop runs object detection and face detection.
4. Person detection starts a countdown.
5. Node A shortens the countdown using sound and `VL53L5CX` approach features.
6. If the person gets too close, Node A shows `STEP BACK` and the laptop issues a spoken warning.
7. The password page disarms the alarm if entered in time.
8. If not, the laptop escalates, dispatches the bot, and exposes manual override controls in the dashboard.

## Build

```bash
cd /Users/zahoormashahir/Documents/Projects/DASN/dasn_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

Full demo:

```bash
ros2 launch dasn_bringup dasn_full.launch.py
```

Fixed-node test stack:

```bash
ros2 launch dasn_bringup sentinel_test.launch.py
```

## Web UI

- Dashboard: `http://<laptop-ip>:8765/`
- Password page: `http://<laptop-ip>:8765/auth/FRONT_DOOR`

## Setup Notes

Use [PRESENTATION_SETUP.md](/Users/zahoormashahir/Documents/Projects/DASN/docs/PRESENTATION_SETUP.md) for the RP5, ESP32, bot, and presentation-day bring-up order.
