# DASN

Distributed Autonomous Security Network is a hybrid indoor security robotics system that combines fixed sensor sentinels, a ROS 2 control and perception stack, and a mobile patrol robot.

## Overview

DASN is organized into three cooperating layers:

- Sentinel nodes placed at doors and windows for local event sensing
- A ROS 2 laptop that performs sensor fusion, threat assessment, navigation, and operator-facing monitoring
- A dual-controller mobile robot that can be dispatched to inspect zones with onboard sensors and a phone-based camera feed

The system is designed for modularity. Fixed nodes detect activity, the central ROS 2 stack reasons about threats, and the mobile unit provides close-range inspection and patrol capability.

## System Architecture

### 1. Sentinel Layer

Fixed ESP32-based sentinel nodes monitor entry points and environmental conditions.

- Type A: radar, PIR, BME688, INMP441, DS18B20, CAP1203, NFC, OLED, buzzer
- Type B: ESP32-CAM, PIR, INMP441, buzzer, microSD
- Type C: radar, STTS22H, analog sound, buzzer

Sentinels transmit compact sensor summaries to the gateway using ESP-NOW.

### 2. Compute Layer

A laptop running ROS 2 Humble acts as the main orchestration point.

- Receives sentinel data from the gateway
- Runs perception nodes on video and audio streams
- Fuses signals into zone-level threat scores
- Dispatches the mobile unit when a threshold is crossed
- Serves telemetry to the dashboard and alerting components

### 3. Mobile Unit

The mobile unit uses a split-controller design.

- ESP32 upper board:
  - Reads IMU, magnetometer, ToF, ultrasonic, battery voltage, and wheel encoders
  - Bridges sensor telemetry to the laptop over USB serial
  - Forwards motion and buzzer commands to the motor controller over UART2
- ATmega328P motor controller:
  - Drives the two motor channels
  - Handles motor enable/disable and watchdog stop behavior
  - Drives the local buzzer and SPI OLED
- Android phone:
  - Streams RTSP video and HTTP audio into the ROS 2 perception pipeline
  - Can be used for remote audio and flashlight alerts through the IP Webcam interface

## Repository Layout

```text
DASN/
├── dasn_ws/
│   └── src/
│       ├── dasn_msgs/
│       ├── dasn_perception/
│       ├── dasn_sentinel/
│       ├── dasn_threat/
│       ├── dasn_navigation/
│       ├── dasn_dashboard/
│       ├── dasn_bringup/
│       └── dasn_description/
└── firmware/
    ├── shared/
    ├── sentinel_type_a/
    ├── sentinel_type_b/
    ├── sentinel_type_c/
    ├── gateway/
    ├── mobile_unit_esp32/
    ├── mobile_unit_aiduino/
    └── sensor_test/
```

## ROS 2 Packages

- `dasn_msgs`: custom message definitions for zone sensors, threats, bot status, depth grids, encoder ticks, alarm events, and related data types
- `dasn_perception`: phone camera ingestion, ESP32-CAM ingestion, object detection, face analysis, pose analysis, sound classification, gas analysis
- `dasn_sentinel`: gateway receiver and authorization manager
- `dasn_threat`: threat fusion and escalation logic
- `dasn_navigation`: serial bridge, odometry, navigator, obstacle avoidance, commander, stall detector
- `dasn_dashboard`: Telegram alerts, phone speaker alerts, browser dashboard
- `dasn_bringup`: launch files and robot configuration
- `dasn_description`: URDF/Xacro robot model

## Mobile Robot Data Flow

1. The laptop publishes velocity commands through ROS 2.
2. The navigation stack filters them with obstacle avoidance.
3. `serial_bridge` converts motion commands into plain-text serial commands.
4. The ESP32 forwards those commands to the ATmega328P motor controller.
5. The motor controller drives the motors.
6. The ESP32 streams encoder, IMU, depth, ultrasonic, battery, and stall data back as JSON lines.
7. ROS 2 nodes convert those streams into typed topics for odometry, safety, and mission control.

## Key Topics

- `/zone/<zone_id>/sensors`
- `/zone/<zone_id>/threat_level`
- `/alerts/alarm`
- `/camera/phone/image_raw`
- `/camera/phone/audio_raw`
- `/odom/encoders`
- `/imu/data`
- `/tof/depth_grid`
- `/battery/voltage`
- `/bot/status`
- `/cmd_vel`
- `/cmd_vel_safe`

## Dependencies

Target software platform:

- Ubuntu 22.04
- ROS 2 Humble

Required ROS packages:

```bash
sudo apt install ros-humble-desktop \
  ros-humble-nav2-bringup \
  ros-humble-tf2-ros \
  ros-humble-rosbridge-suite \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher
```

Required Python packages:

```bash
pip install opencv-python mediapipe tflite-runtime numpy python-telegram-bot pyserial
```

## Build

```bash
cd dasn_ws
colcon build --symlink-install
source install/setup.bash
```

## Launch

Full system:

```bash
ros2 launch dasn_bringup dasn_full.launch.py
```

Mobile robot only:

```bash
ros2 launch dasn_bringup bot_only.launch.py
```

Sentinel and threat pipeline only:

```bash
ros2 launch dasn_bringup sentinel_test.launch.py
```

## Firmware

The `firmware/` directory contains:

- Sentinel firmware for all three node types
- Gateway firmware for ESP-NOW to USB serial bridging
- Mobile robot ESP32 firmware
- Mobile robot ATmega328P motor controller firmware
- Standalone hardware validation sketches under `firmware/sensor_test/`

## Current Status

This repository contains the architecture, firmware, ROS 2 packages, launch configuration, and hardware test sketches for DASN. Individual subsystems and hardware interfaces have been implemented and exercised during development. Full integrated validation should be treated as ongoing until the complete end-to-end run sequence is finished on target hardware.

## License

No license has been added yet. Add one before sharing publicly if you want to define reuse terms.
