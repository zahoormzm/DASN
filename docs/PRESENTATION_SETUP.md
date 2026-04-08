# DASN Presentation Setup

## Final Roles
- **Laptop**: master server, dashboard, password page, vision inference, bot command source.
- **Raspberry Pi 5**: camera node for Door A using Camera Module v2.
- **ESP32 Sentinel Type A**: fixed sensor/display node at Door A.
- **Mobile Bot**: phone camera + audio witness, driven by the ESP32 motion controller.

## Models Used In The Demo
- `YOLOv8n TFLite` object detector on the laptop.
- `MediaPipe Face Detection` on the laptop.

The other old perception models are not part of the new demo path.

## RP5 Presentation Placement
1. Mount the RP5 so the Camera v2 faces Door A directly.
2. Keep the RP5 on stable power for the full demo.
3. Put the RP5 on the same Wi-Fi network as the laptop.
4. Start the camera stream before the ROS stack on the laptop.
5. Verify the laptop can open the RP5 stream URL before the audience walkthrough.

## ESP32 Presentation Placement
1. Mount the ESP32 Sentinel Type A next to Door A.
2. Keep the LCD facing the audience.
3. Place the CAP touch pads where they are easy to reach.
4. Aim the VL53L5CX toward the approach path to Door A so the center cells cover the doorway.
5. Put the sound sensor where it can hear the scene without the LCD or enclosure blocking it.

## Mobile Bot Presentation Placement
1. Start the bot on the Door B table / dock position.
2. Zero the bot pose with the calibration sketch before the demo.
3. Connect the phone stream before starting patrol.
4. Verify `GO,x,y`, `POSE`, and `STOP` manually once before the live run.

## Demo Bring-Up Order
1. Start the RP5 camera stream.
2. Flash and power the ESP32 Sentinel Type A runtime sketch.
3. Flash and verify the mobile bot calibration sketch.
4. On the laptop:
   - `cd /Users/zahoormashahir/Documents/Projects/DASN/dasn_ws`
   - `colcon build --symlink-install`
   - `source install/setup.bash`
   - `ros2 launch dasn_bringup dasn_full.launch.py`
5. Open `http://<laptop-ip>:8765/` for the dashboard.
6. Confirm the auth page opens from the dashboard for `FRONT_DOOR`.

## What To Test Before Presentation
1. RP5 camera stream reaches the laptop and object detection works on that feed.
2. Phone camera stream reaches the laptop and face detection works on that feed.
3. ESP32 sends Wi-Fi telemetry and the dashboard shows live sound / ToF / BME values.
4. Dashboard buttons can arm, disarm, force countdown, force alarm, and dispatch the bot.
5. Password page disarms the alarm correctly.
6. Bot accepts `GO`, `STOP`, and patrol / return commands from the laptop.
7. Alarm escalation reaches the phone alert path and the call webhook placeholder or real webhook.
