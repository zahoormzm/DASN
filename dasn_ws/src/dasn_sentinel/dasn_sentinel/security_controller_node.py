"""Central security controller with dashboard, auth page, bot orchestration, and overrides."""

from __future__ import annotations

import html
import json
import socket
import threading
import time
import urllib.request
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from dasn_msgs.msg import AlarmEvent, AuthEvent, BotStatus, DispatchCommand, FaceDetection, ZoneSensorData
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray


@dataclass
class ZoneRuntime:
    zone_id: str
    armed: bool = True
    phase: str = 'ARMED'
    countdown_deadline: float | None = None
    source_ip: str | None = None
    sensor_payload: dict[str, Any] = field(default_factory=dict)
    sensor_msg: ZoneSensorData | None = None
    last_state_hash: str = ''
    last_sound_penalty_at: float = 0.0
    last_tof_penalty_at: float = 0.0
    last_cap_mask: int = 0
    alarm_active: bool = False
    alarm_started_at: float | None = None
    escalation_called: bool = False
    bot_dispatched: bool = False
    warning_active: bool = False
    last_warning_at: float = 0.0


class SecurityControllerNode(Node):
    def __init__(self):
        super().__init__('security_controller', namespace='security')

        self.declare_parameter('zone_ids', ['FRONT_DOOR'])
        self.declare_parameter('detection_zone', 'FRONT_DOOR')
        self.declare_parameter('countdown_seconds', 60)
        self.declare_parameter('sound_threshold_db', 70.0)
        self.declare_parameter('sound_penalty_seconds', 10)
        self.declare_parameter('sound_penalty_cooldown_seconds', 5)
        self.declare_parameter('tof_near_threshold_mm', 900)
        self.declare_parameter('tof_penalty_seconds', 15)
        self.declare_parameter('tof_penalty_cooldown_seconds', 5)
        self.declare_parameter('tof_approach_score_threshold', 0.55)
        self.declare_parameter('tof_warning_threshold_mm', 900)
        self.declare_parameter('tof_warning_penalty_seconds', 20)
        self.declare_parameter('warning_cooldown_seconds', 8)
        self.declare_parameter('default_password', '1234')
        self.declare_parameter('zone_passwords', [])
        self.declare_parameter('auth_host', '0.0.0.0')
        self.declare_parameter('auth_port', 8765)
        self.declare_parameter('public_base_url', 'http://localhost:8765')
        self.declare_parameter('control_udp_port', 37021)
        self.declare_parameter('state_publish_topic', '/security/state')
        self.declare_parameter('auto_dispatch_bot', True)
        self.declare_parameter('auto_patrol_on_start', True)
        self.declare_parameter('escalation_delay_seconds', 15)
        self.declare_parameter('call_webhook_url', '')
        self.declare_parameter('call_cooldown_seconds', 120)
        self.declare_parameter('door_a_x', 1.2)
        self.declare_parameter('door_a_y', 0.0)
        self.declare_parameter('door_b_x', 0.0)
        self.declare_parameter('door_b_y', 0.0)
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)

        zone_ids = list(self.get_parameter('zone_ids').get_parameter_value().string_array_value)
        self._detection_zone = self.get_parameter('detection_zone').get_parameter_value().string_value
        self._countdown_seconds = float(
            self.get_parameter('countdown_seconds').get_parameter_value().integer_value
        )
        self._sound_threshold_db = float(
            self.get_parameter('sound_threshold_db').get_parameter_value().double_value
        )
        self._sound_penalty_seconds = float(
            self.get_parameter('sound_penalty_seconds').get_parameter_value().integer_value
        )
        self._sound_penalty_cooldown_seconds = float(
            self.get_parameter('sound_penalty_cooldown_seconds').get_parameter_value().integer_value
        )
        self._tof_near_threshold_mm = int(
            self.get_parameter('tof_near_threshold_mm').get_parameter_value().integer_value
        )
        self._tof_penalty_seconds = float(
            self.get_parameter('tof_penalty_seconds').get_parameter_value().integer_value
        )
        self._tof_penalty_cooldown_seconds = float(
            self.get_parameter('tof_penalty_cooldown_seconds').get_parameter_value().integer_value
        )
        self._tof_approach_score_threshold = float(self.get_parameter('tof_approach_score_threshold').value)
        self._tof_warning_threshold_mm = int(self.get_parameter('tof_warning_threshold_mm').value)
        self._tof_warning_penalty_seconds = float(self.get_parameter('tof_warning_penalty_seconds').value)
        self._warning_cooldown_seconds = float(self.get_parameter('warning_cooldown_seconds').value)
        self._default_password = self.get_parameter('default_password').get_parameter_value().string_value
        self._zone_password_entries = list(
            self.get_parameter('zone_passwords').get_parameter_value().string_array_value
        )
        self._auth_host = self.get_parameter('auth_host').get_parameter_value().string_value
        self._auth_port = self.get_parameter('auth_port').get_parameter_value().integer_value
        self._public_base_url = (
            self.get_parameter('public_base_url').get_parameter_value().string_value.rstrip('/')
        )
        self._control_udp_port = self.get_parameter('control_udp_port').get_parameter_value().integer_value
        self._state_publish_topic = self.get_parameter('state_publish_topic').get_parameter_value().string_value
        self._auto_dispatch_bot = bool(self.get_parameter('auto_dispatch_bot').value)
        self._auto_patrol_on_start = bool(self.get_parameter('auto_patrol_on_start').value)
        self._escalation_delay_seconds = float(self.get_parameter('escalation_delay_seconds').value)
        self._call_webhook_url = str(self.get_parameter('call_webhook_url').value).strip()
        self._call_cooldown_seconds = float(self.get_parameter('call_cooldown_seconds').value)

        self._manual_points = {
            'door_a': self._point(
                float(self.get_parameter('door_a_x').value),
                float(self.get_parameter('door_a_y').value),
            ),
            'door_b': self._point(
                float(self.get_parameter('door_b_x').value),
                float(self.get_parameter('door_b_y').value),
            ),
            'dock': self._point(
                float(self.get_parameter('dock_x').value),
                float(self.get_parameter('dock_y').value),
            ),
        }

        self._zones = {zone_id: ZoneRuntime(zone_id=zone_id) for zone_id in zone_ids}
        self._zone_passwords = self._parse_zone_passwords(self._zone_password_entries)
        self._lock = threading.Lock()
        self._control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._bot_status = {
            'status': 'UNKNOWN',
            'target_zone': '',
            'position': {'x': 0.0, 'y': 0.0},
            'heading_deg': 0.0,
            'distance_remaining': 0.0,
        }
        self._last_call_at = 0.0
        self._startup_patrol_sent = False
        self._started_at = time.time()

        self._state_pub = self.create_publisher(String, self._state_publish_topic, 10)
        self._alarm_pub = self.create_publisher(AlarmEvent, '/alerts/alarm', 10)
        self._auth_pub = self.create_publisher(AuthEvent, '/security/auth_event', 10)
        self._bot_dispatch_pub = self.create_publisher(DispatchCommand, '/bot/dispatch', 10)
        self._voice_prompt_pub = self.create_publisher(String, '/alerts/voice_prompt', 10)

        self.create_subscription(Detection2DArray, '/ml/objects', self._objects_cb, 10)
        self.create_subscription(FaceDetection, '/ml/faces', self._faces_cb, 10)
        self.create_subscription(BotStatus, '/bot/status', self._bot_status_cb, 10)

        for zone_id in zone_ids:
            self.create_subscription(
                ZoneSensorData,
                f'/zone/{zone_id}/sensors',
                lambda msg, z=zone_id: self._sensor_cb(msg, z),
                10,
            )
            self.create_subscription(
                String,
                f'/zone/{zone_id}/sensor_meta',
                lambda msg, z=zone_id: self._sensor_meta_cb(msg, z),
                10,
            )

        self.create_timer(0.25, self._tick)
        self._start_http_server()

        self.get_logger().info(
            f'Security controller ready — zones={zone_ids}, dashboard={self._public_base_url}/'
        )

    def _point(self, x: float, y: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        return point

    def _parse_zone_passwords(self, entries: list[str]) -> dict[str, str]:
        passwords: dict[str, str] = {}
        for entry in entries:
            if ':' not in entry:
                continue
            zone_id, password = entry.split(':', 1)
            zone_id = zone_id.strip()
            password = password.strip()
            if zone_id and password:
                passwords[zone_id] = password
        return passwords

    def _sensor_cb(self, msg: ZoneSensorData, zone_id: str) -> None:
        with self._lock:
            zone = self._zones[zone_id]
            zone.sensor_msg = msg
            if zone.phase == 'COUNTDOWN':
                if msg.sound_transient or msg.sound_level_db >= self._sound_threshold_db:
                    self._apply_penalty_locked(zone, 'sound', self._sound_penalty_seconds)

    def _sensor_meta_cb(self, msg: String, zone_id: str) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid sensor_meta JSON for {zone_id}: {exc}')
            return

        with self._lock:
            zone = self._zones[zone_id]
            zone.source_ip = str(payload.get('source_ip') or zone.source_ip or '')
            sensor_payload = payload.get('payload') or {}
            if isinstance(sensor_payload, dict):
                zone.sensor_payload = sensor_payload

            tof_mm = self._extract_tof_mm(sensor_payload)
            tof_approach_score = self._extract_tof_approach_score(sensor_payload)
            tof_warning = bool(sensor_payload.get('tof_warning', False))

            if zone.phase == 'COUNTDOWN':
                if tof_warning or (tof_mm is not None and tof_mm <= self._tof_warning_threshold_mm):
                    zone.warning_active = True
                    self._emit_step_back_warning_locked(zone, 'tof_warning')
                    self._apply_penalty_locked(zone, 'tof_warning', self._tof_warning_penalty_seconds)
                elif (
                    tof_mm is not None
                    and tof_mm <= self._tof_near_threshold_mm
                    and tof_approach_score >= self._tof_approach_score_threshold
                ):
                    zone.warning_active = False
                    self._apply_penalty_locked(zone, 'tof', self._tof_penalty_seconds)
                else:
                    zone.warning_active = False

            cap_mask = int(sensor_payload.get('cap_mask', 0) or 0)
            if cap_mask != zone.last_cap_mask:
                zone.last_cap_mask = cap_mask
                if zone.phase == 'DISARMED' and (cap_mask & 0x01) and (cap_mask & 0x04):
                    self._arm_zone_locked(zone, reason='touch_rearm')

    def _extract_tof_mm(self, sensor_payload: dict[str, Any]) -> int | None:
        for key in ('tof_center_min_mm', 'tof_mm', 'tof_distance_mm', 'distance_mm'):
            value = sensor_payload.get(key)
            if value is None:
                continue
            try:
                return int(value)
            except (TypeError, ValueError):
                continue
        return None

    def _extract_tof_approach_score(self, sensor_payload: dict[str, Any]) -> float:
        value = sensor_payload.get('tof_approach_score', 0.0)
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def _objects_cb(self, msg: Detection2DArray) -> None:
        saw_person = False
        for detection in msg.detections:
            for result in detection.results:
                if str(result.hypothesis.class_id).lower() == 'person':
                    saw_person = True
                    break
            if saw_person:
                break

        if not saw_person:
            return

        with self._lock:
            zone = self._zones.get(self._detection_zone)
            if zone is None:
                return
            if zone.armed and zone.phase == 'ARMED':
                self._start_countdown_locked(zone, reason='person_detected')

    def _faces_cb(self, msg: FaceDetection) -> None:
        if msg.source != 'phone' or not msg.confidences:
            return

        with self._lock:
            zone = self._zones.get(self._detection_zone)
            if zone is None:
                return
            if zone.phase == 'ALARM':
                self._trigger_escalation_locked(zone, reason='bot_face_detected')

    def _bot_status_cb(self, msg: BotStatus) -> None:
        with self._lock:
            self._bot_status = {
                'status': msg.status,
                'target_zone': msg.target_zone,
                'position': {'x': float(msg.position.x), 'y': float(msg.position.y)},
                'heading_deg': float(msg.heading_deg),
                'distance_remaining': float(msg.distance_remaining),
            }

    def _start_countdown_locked(self, zone: ZoneRuntime, reason: str) -> None:
        zone.phase = 'COUNTDOWN'
        zone.countdown_deadline = time.time() + self._countdown_seconds
        zone.alarm_active = False
        zone.warning_active = False
        zone.alarm_started_at = None
        zone.escalation_called = False
        zone.bot_dispatched = False
        self._publish_alarm_locked(zone, level=3, reason=reason)
        self._publish_state_locked(zone)
        self._send_control_locked(zone)
        self.get_logger().info(f'Countdown started for {zone.zone_id} ({reason})')

    def _apply_penalty_locked(self, zone: ZoneRuntime, source: str, seconds: float) -> None:
        now = time.time()
        if source == 'sound':
            if now - zone.last_sound_penalty_at < self._sound_penalty_cooldown_seconds:
                return
            zone.last_sound_penalty_at = now
        elif source == 'tof':
            if now - zone.last_tof_penalty_at < self._tof_penalty_cooldown_seconds:
                return
            zone.last_tof_penalty_at = now

        if zone.countdown_deadline is None:
            return

        remaining = zone.countdown_deadline - now
        new_remaining = max(5.0, remaining - seconds)
        zone.countdown_deadline = now + new_remaining
        self._publish_state_locked(zone)
        self._send_control_locked(zone)
        self.get_logger().info(
            f'Countdown reduced for {zone.zone_id} by {seconds:.0f}s due to {source}; remaining={new_remaining:.1f}s'
        )

    def _trigger_alarm_locked(self, zone: ZoneRuntime, reason: str) -> None:
        zone.phase = 'ALARM'
        zone.countdown_deadline = None
        zone.alarm_active = True
        zone.warning_active = False
        zone.alarm_started_at = time.time()
        self._publish_alarm_locked(zone, level=4, reason=reason)
        if self._auto_dispatch_bot and not zone.bot_dispatched:
            self._dispatch_bot_locked('go_door_a', zone.zone_id, self._manual_points['door_a'])
            zone.bot_dispatched = True
        self._publish_state_locked(zone)
        self._send_control_locked(zone)
        self.get_logger().warning(f'Alarm active for {zone.zone_id} ({reason})')

    def _trigger_escalation_locked(self, zone: ZoneRuntime, reason: str) -> None:
        if zone.phase == 'ESCALATED':
            return

        zone.phase = 'ESCALATED'
        zone.countdown_deadline = None
        zone.alarm_active = True
        zone.warning_active = False
        self._publish_alarm_locked(zone, level=5, reason=reason)
        if self._auto_dispatch_bot:
            self._dispatch_bot_locked('investigate_door_a', zone.zone_id, self._manual_points['door_a'])
        self._trigger_call_async(zone.zone_id, reason)
        self._publish_state_locked(zone)
        self._send_control_locked(zone)
        self.get_logger().error(f'Escalation active for {zone.zone_id} ({reason})')

    def _disarm_zone_locked(self, zone: ZoneRuntime, method: str, tag_id: str = 'web') -> None:
        zone.armed = False
        zone.phase = 'DISARMED'
        zone.countdown_deadline = None
        zone.alarm_active = False
        zone.warning_active = False
        zone.alarm_started_at = None
        zone.bot_dispatched = False
        auth = AuthEvent()
        auth.zone_id = zone.zone_id
        auth.method = method
        auth.authorized = True
        auth.tag_id = tag_id
        auth.timestamp = self.get_clock().now().to_msg()
        self._auth_pub.publish(auth)
        self._dispatch_bot_locked('return_dock', 'dock', self._manual_points['dock'])
        self._publish_state_locked(zone)
        self._send_control_locked(zone)
        self.get_logger().info(f'Zone {zone.zone_id} disarmed by {method}')

    def _auth_failure_locked(self, zone: ZoneRuntime, method: str, tag_id: str = 'web') -> None:
        auth = AuthEvent()
        auth.zone_id = zone.zone_id
        auth.method = method
        auth.authorized = False
        auth.tag_id = tag_id
        auth.timestamp = self.get_clock().now().to_msg()
        self._auth_pub.publish(auth)

    def _arm_zone_locked(self, zone: ZoneRuntime, reason: str) -> None:
        zone.armed = True
        zone.phase = 'ARMED'
        zone.countdown_deadline = None
        zone.alarm_active = False
        zone.warning_active = False
        zone.alarm_started_at = None
        zone.escalation_called = False
        self._publish_state_locked(zone)
        self._send_control_locked(zone)
        if self._auto_patrol_on_start:
            self._dispatch_bot_locked('patrol_door_b', 'DOOR_B', self._manual_points['door_b'])
            self._startup_patrol_sent = True
        self.get_logger().info(f'Zone {zone.zone_id} armed ({reason})')

    def _publish_alarm_locked(self, zone: ZoneRuntime, level: int, reason: str) -> None:
        alarm = AlarmEvent()
        alarm.level = int(level)
        alarm.zone_id = zone.zone_id
        alarm.reason = reason
        alarm.timestamp = self.get_clock().now().to_msg()
        self._alarm_pub.publish(alarm)

    def _dispatch_bot_locked(
        self,
        mission_type: str,
        target_zone: str,
        point: Point | None = None,
        points: list[Point] | None = None,
    ) -> None:
        command = DispatchCommand()
        command.target_zone = target_zone
        command.threat_level = 0
        command.mission_type = mission_type
        if points:
            command.waypoints = points
        elif point is not None:
            command.waypoints = [point]
        self._bot_dispatch_pub.publish(command)

    def _emit_step_back_warning_locked(self, zone: ZoneRuntime, reason: str) -> None:
        now = time.time()
        if now - zone.last_warning_at < self._warning_cooldown_seconds:
            return
        zone.last_warning_at = now
        prompt = String()
        prompt.data = json.dumps(
            {'zone_id': zone.zone_id, 'prompt': 'step back', 'reason': reason},
            separators=(',', ':'),
        )
        self._voice_prompt_pub.publish(prompt)

    def _zone_state_payload_locked(self, zone: ZoneRuntime) -> dict[str, Any]:
        remaining = 0
        if zone.phase == 'COUNTDOWN' and zone.countdown_deadline is not None:
            remaining = max(0, int(zone.countdown_deadline - time.time()))

        sensor_payload = dict(zone.sensor_payload)
        bme = {
            'temp_c': float(sensor_payload.get('temp', 0.0) or 0.0),
            'humidity_pct': float(sensor_payload.get('hum', 0.0) or 0.0),
            'gas_ohm': float(sensor_payload.get('gas_r', 0.0) or 0.0),
        }

        return {
            'zone_id': zone.zone_id,
            'phase': zone.phase,
            'armed': zone.armed,
            'warning_active': zone.warning_active,
            'remaining_seconds': remaining,
            'source_ip': zone.source_ip,
            'sensor': sensor_payload,
            'bme': bme,
            'auth_url': f'{self._public_base_url}/auth/{zone.zone_id}',
        }

    def _publish_state_locked(self, zone: ZoneRuntime) -> None:
        payload = {
            'zones': {
                zone.zone_id: self._zone_state_payload_locked(zone),
            },
            'bot': self._bot_status,
        }
        encoded = json.dumps(payload, separators=(',', ':'))
        if encoded == zone.last_state_hash:
            return
        zone.last_state_hash = encoded
        msg = String()
        msg.data = encoded
        self._state_pub.publish(msg)

    def _send_control_locked(self, zone: ZoneRuntime) -> None:
        if not zone.source_ip:
            return

        payload = self._zone_state_payload_locked(zone)
        remaining = payload['remaining_seconds']
        bme = payload['bme']

        if zone.phase == 'COUNTDOWN':
            if zone.warning_active:
                lcd1 = 'STEP BACK'
                lcd2 = 'TOO CLOSE'
                led_mode = 'warning'
            else:
                lcd1 = 'PERSON DETECTED'
                lcd2 = f'CODE IN {remaining:02d}s'
                led_mode = 'countdown'
        elif zone.phase == 'ALARM':
            lcd1 = 'ALARM ACTIVE'
            lcd2 = 'BOT DISPATCHED'
            led_mode = 'alarm'
        elif zone.phase == 'ESCALATED':
            lcd1 = 'ESCALATED'
            lcd2 = 'CALLING OWNER'
            led_mode = 'alarm'
        elif zone.phase == 'DISARMED':
            lcd1 = 'ALARM DISABLED'
            lcd2 = f"T:{bme['temp_c']:.1f} H:{int(bme['humidity_pct'])}"
            led_mode = 'disarmed'
        else:
            lcd1 = 'SYSTEM ARMED'
            lcd2 = 'MONITORING...'
            led_mode = 'armed'

        command = {
            'zone_id': zone.zone_id,
            'phase': zone.phase,
            'armed': zone.armed,
            'remaining_s': remaining,
            'led_mode': led_mode,
            'lcd1': lcd1[:16],
            'lcd2': lcd2[:16],
            'show_bme': zone.phase == 'DISARMED',
            'bme_temp': bme['temp_c'],
            'bme_hum': bme['humidity_pct'],
            'bme_gas': bme['gas_ohm'],
        }

        try:
            self._control_sock.sendto(
                json.dumps(command, separators=(',', ':')).encode('utf-8'),
                (zone.source_ip, int(self._control_udp_port)),
            )
        except OSError as exc:
            self.get_logger().warn(
                f'Failed to send control to {zone.zone_id} at {zone.source_ip}:{self._control_udp_port}: {exc}'
            )

    def _tick(self) -> None:
        with self._lock:
            now = time.time()
            if (
                self._auto_patrol_on_start
                and not self._startup_patrol_sent
                and now - self._started_at >= 3.0
            ):
                self._dispatch_bot_locked('patrol_door_b', 'DOOR_B', self._manual_points['door_b'])
                self._startup_patrol_sent = True

            for zone in self._zones.values():
                if zone.phase == 'COUNTDOWN' and zone.countdown_deadline is not None:
                    if now >= zone.countdown_deadline:
                        self._trigger_alarm_locked(zone, reason='countdown_expired')
                    else:
                        self._publish_state_locked(zone)
                        self._send_control_locked(zone)
                elif zone.phase == 'ALARM':
                    if (
                        zone.alarm_started_at is not None
                        and now - zone.alarm_started_at >= self._escalation_delay_seconds
                    ):
                        self._trigger_escalation_locked(zone, reason='alarm_persisted')
                    else:
                        self._publish_state_locked(zone)
                        self._send_control_locked(zone)
                else:
                    self._publish_state_locked(zone)
                    self._send_control_locked(zone)

    def _password_for_zone(self, zone_id: str) -> str:
        return self._zone_passwords.get(zone_id, self._default_password)

    def _start_http_server(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == '/api/security-state':
                    node._handle_http_state(self)
                    return
                if parsed.path == '/api/bot-state':
                    node._handle_http_bot_state(self)
                    return
                if parsed.path == '/':
                    node._handle_http_dashboard(self)
                    return
                if parsed.path.startswith('/auth/'):
                    zone_id = parsed.path.split('/auth/', 1)[1].strip('/')
                    node._handle_http_auth_page(self, zone_id, message='')
                    return
                self.send_error(404, 'Not found')

            def do_POST(self):
                parsed = urlparse(self.path)
                length = int(self.headers.get('Content-Length', '0'))
                body = self.rfile.read(length).decode('utf-8', errors='replace')

                if parsed.path.startswith('/auth/'):
                    zone_id = parsed.path.split('/auth/', 1)[1].strip('/')
                    form = parse_qs(body, keep_blank_values=True)
                    password = form.get('password', [''])[0]
                    node._handle_http_auth_submit(self, zone_id, password)
                    return

                if parsed.path.startswith('/api/zone/'):
                    parts = [part for part in parsed.path.split('/') if part]
                    if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'zone':
                        zone_id = parts[2]
                        action = parts[3]
                        node._handle_zone_action(self, zone_id, action)
                        return

                if parsed.path == '/api/bot/action':
                    node._handle_bot_action(self, body)
                    return

                if parsed.path == '/api/call/test':
                    node._handle_call_test(self)
                    return

                self.send_error(404, 'Not found')

            def log_message(self, fmt, *args):
                node.get_logger().debug(fmt % args)

        self._http_server = ThreadingHTTPServer((self._auth_host, int(self._auth_port)), Handler)
        self._http_thread = threading.Thread(target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()

    def _handle_http_dashboard(self, handler: BaseHTTPRequestHandler) -> None:
        page = """
        <html>
        <head>
          <title>DASN Control</title>
          <style>
            body{font-family:system-ui,sans-serif;margin:0;background:#eef2e9;color:#1f2a1c}
            .wrap{max-width:1100px;margin:0 auto;padding:24px}
            .grid{display:grid;grid-template-columns:1.3fr 1fr;gap:20px}
            .card{background:#fff;border:1px solid #d5dfcf;border-radius:16px;padding:18px;box-shadow:0 10px 24px rgba(31,42,28,.06)}
            h1,h2{margin:0 0 14px}
            button{margin:4px 6px 4px 0;padding:10px 14px;border:none;border-radius:10px;background:#214d35;color:#fff;cursor:pointer}
            button.alt{background:#6c7f65}
            button.warn{background:#a85f1a}
            button.danger{background:#912c2c}
            .mono{font-family:ui-monospace,monospace}
            .small{font-size:13px;color:#5d6958}
            input,select{padding:10px;border:1px solid #c7d1c2;border-radius:10px;width:100%;margin:6px 0 10px}
          </style>
        </head>
        <body>
          <div class="wrap">
            <h1>DASN Demo Control</h1>
            <div class="grid">
              <div class="card">
                <h2>Security</h2>
                <div id="zones"></div>
              </div>
              <div class="card">
                <h2>Bot</h2>
                <div id="bot" class="small">Loading...</div>
                <div style="margin-top:12px">
                  <button onclick="botAction({mission_type:'patrol_door_b'})">Start Patrol</button>
                  <button onclick="botAction({mission_type:'go_door_a'})">Go Door A</button>
                  <button onclick="botAction({mission_type:'go_door_b'})">Go Door B</button>
                  <button onclick="botAction({mission_type:'investigate_door_a'})">Investigate</button>
                  <button class="alt" onclick="botAction({mission_type:'return_dock'})">Return Dock</button>
                  <button class="danger" onclick="botAction({mission_type:'stop'})">Stop</button>
                </div>
                <h2 style="margin-top:18px">Manual Coordinates</h2>
                <label>Mission</label>
                <select id="mission">
                  <option value="manual_go">Manual Go</option>
                  <option value="go_door_a">Go Door A</option>
                  <option value="go_door_b">Go Door B</option>
                  <option value="return_dock">Return Dock</option>
                </select>
                <label>X (m)</label>
                <input id="botx" value="1.20"/>
                <label>Y (m)</label>
                <input id="boty" value="0.00"/>
                <button onclick="sendManualBot()">Send Manual Command</button>
                <button class="warn" onclick="fetch('/api/call/test',{method:'POST'})">Trigger Call</button>
              </div>
            </div>
          </div>
          <script>
            async function loadState(){
              const res = await fetch('/api/security-state');
              const data = await res.json();
              const zones = data.zones || {};
              const bot = data.bot || {};
              document.getElementById('zones').innerHTML = Object.values(zones).map(z => `
                <div style="border-top:1px solid #e5ece2;padding-top:14px;margin-top:14px">
                  <div><strong>${z.zone_id}</strong> <span class="small">phase=${z.phase} armed=${z.armed}</span></div>
                  <div class="small mono">auth: <a href="${z.auth_url}" target="_blank">${z.auth_url}</a></div>
                  <div class="small">remaining: ${z.remaining_seconds}s | sound: ${Number(z.sensor.sound_db || 0).toFixed(1)} | tof: ${z.sensor.tof_center_min_mm || z.sensor.tof_mm || 0} | app: ${Number(z.sensor.tof_approach_score || 0).toFixed(2)}</div>
                  <div class="small">BME: ${Number(z.bme.temp_c).toFixed(1)}C / ${Number(z.bme.humidity_pct).toFixed(0)}%</div>
                  <div style="margin-top:8px">
                    <button onclick="zoneAction('${z.zone_id}','arm')">Arm</button>
                    <button class="alt" onclick="zoneAction('${z.zone_id}','disarm')">Disarm</button>
                    <button class="warn" onclick="zoneAction('${z.zone_id}','countdown')">Start Countdown</button>
                    <button class="danger" onclick="zoneAction('${z.zone_id}','alarm')">Force Alarm</button>
                    <button class="danger" onclick="zoneAction('${z.zone_id}','escalate')">Escalate</button>
                    <button class="alt" onclick="zoneAction('${z.zone_id}','reset')">Reset</button>
                  </div>
                </div>
              `).join('');
              document.getElementById('bot').innerHTML =
                `<div class="mono">status=${bot.status || 'UNKNOWN'}</div>
                 <div class="small">target=${bot.target_zone || '-'}</div>
                 <div class="small">pos=(${Number(bot.position?.x || 0).toFixed(2)}, ${Number(bot.position?.y || 0).toFixed(2)})</div>
                 <div class="small">heading=${Number(bot.heading_deg || 0).toFixed(1)} deg</div>
                 <div class="small">remaining=${Number(bot.distance_remaining || 0).toFixed(2)} m</div>`;
            }
            async function zoneAction(zone, action){
              await fetch(`/api/zone/${zone}/${action}`, {method:'POST'});
              loadState();
            }
            async function botAction(payload){
              await fetch('/api/bot/action', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
              loadState();
            }
            async function sendManualBot(){
              const payload = {
                mission_type: document.getElementById('mission').value,
                x: parseFloat(document.getElementById('botx').value || '0'),
                y: parseFloat(document.getElementById('boty').value || '0'),
              };
              await botAction(payload);
            }
            loadState();
            setInterval(loadState, 1000);
          </script>
        </body>
        </html>
        """
        self._send_http_html(handler, page)

    def _handle_http_state(self, handler: BaseHTTPRequestHandler) -> None:
        with self._lock:
            payload = {
                'zones': {
                    zone_id: self._zone_state_payload_locked(zone)
                    for zone_id, zone in self._zones.items()
                },
                'bot': self._bot_status,
            }
        self._send_http_json(handler, payload)

    def _handle_http_bot_state(self, handler: BaseHTTPRequestHandler) -> None:
        with self._lock:
            payload = dict(self._bot_status)
        self._send_http_json(handler, payload)

    def _handle_http_auth_page(self, handler: BaseHTTPRequestHandler, zone_id: str, message: str) -> None:
        with self._lock:
            zone = self._zones.get(zone_id)
            if zone is None:
                handler.send_error(404, 'Unknown zone')
                return
            payload = self._zone_state_payload_locked(zone)

        state_line = f"State: {html.escape(payload['phase'])}"
        if payload['remaining_seconds']:
            state_line += f" | Time left: {payload['remaining_seconds']}s"

        page = f"""
        <html>
        <head><title>DASN Auth</title></head>
        <body style="font-family:sans-serif;max-width:420px;margin:40px auto;">
            <h1>{html.escape(zone_id)}</h1>
            <p>{state_line}</p>
            <p>{html.escape(message)}</p>
            <form method="post">
                <label>Password</label><br/>
                <input type="password" name="password" autofocus style="width:100%;padding:8px;margin:8px 0;"/><br/>
                <button type="submit" style="padding:10px 16px;">Disable Alarm</button>
            </form>
        </body>
        </html>
        """
        self._send_http_html(handler, page)

    def _handle_http_auth_submit(self, handler: BaseHTTPRequestHandler, zone_id: str, password: str) -> None:
        with self._lock:
            zone = self._zones.get(zone_id)
            if zone is None:
                handler.send_error(404, 'Unknown zone')
                return

            if password == self._password_for_zone(zone_id):
                self._disarm_zone_locked(zone, method='password', tag_id='web')
                message = 'Password accepted. Alarm disabled.'
            else:
                self._auth_failure_locked(zone, method='password', tag_id='web')
                message = 'Incorrect password.'

        self._handle_http_auth_page(handler, zone_id, message=message)

    def _handle_zone_action(self, handler: BaseHTTPRequestHandler, zone_id: str, action: str) -> None:
        with self._lock:
            zone = self._zones.get(zone_id)
            if zone is None:
                handler.send_error(404, 'Unknown zone')
                return

            if action == 'arm':
                self._arm_zone_locked(zone, reason='manual')
            elif action == 'disarm':
                self._disarm_zone_locked(zone, method='manual', tag_id='dashboard')
            elif action == 'countdown':
                self._start_countdown_locked(zone, reason='manual')
            elif action == 'alarm':
                self._trigger_alarm_locked(zone, reason='manual')
            elif action == 'escalate':
                self._trigger_escalation_locked(zone, reason='manual')
            elif action == 'reset':
                self._arm_zone_locked(zone, reason='manual_reset')
            else:
                handler.send_error(400, f'Unknown action {action}')
                return

        self._send_http_json(handler, {'ok': True, 'zone': zone_id, 'action': action})

    def _handle_bot_action(self, handler: BaseHTTPRequestHandler, body: str) -> None:
        try:
            payload = json.loads(body or '{}')
        except json.JSONDecodeError:
            payload = {}

        mission = str(payload.get('mission_type') or '').strip().lower()
        if not mission:
            handler.send_error(400, 'Missing mission_type')
            return

        points: list[Point] | None = None
        if mission == 'manual_go':
            try:
                x = float(payload.get('x', 0.0))
                y = float(payload.get('y', 0.0))
            except (TypeError, ValueError):
                handler.send_error(400, 'Invalid x/y')
                return
            mission = 'manual_go'
            points = [self._point(x, y)]

        with self._lock:
            if points is not None:
                self._dispatch_bot_locked(mission, 'manual', points=points)
            elif mission == 'go_door_a':
                self._dispatch_bot_locked(mission, 'FRONT_DOOR', self._manual_points['door_a'])
            elif mission == 'go_door_b':
                self._dispatch_bot_locked(mission, 'DOOR_B', self._manual_points['door_b'])
            elif mission == 'return_dock':
                self._dispatch_bot_locked(mission, 'dock', self._manual_points['dock'])
            elif mission in {'stop', 'patrol_door_b', 'investigate_door_a'}:
                point = None
                target = 'DOOR_B'
                if mission == 'investigate_door_a':
                    point = self._manual_points['door_a']
                    target = 'FRONT_DOOR'
                if mission == 'patrol_door_b':
                    point = self._manual_points['door_b']
                self._dispatch_bot_locked(mission, target, point)
            else:
                handler.send_error(400, f'Unsupported mission {mission}')
                return

        self._send_http_json(handler, {'ok': True, 'mission_type': mission})

    def _handle_call_test(self, handler: BaseHTTPRequestHandler) -> None:
        self._trigger_call_async('manual', 'dashboard_test')
        self._send_http_json(handler, {'ok': True})

    def _trigger_call_async(self, zone_id: str, reason: str) -> None:
        now = time.time()
        if now - self._last_call_at < self._call_cooldown_seconds:
            return
        self._last_call_at = now

        def _worker():
            payload = {
                'zone_id': zone_id,
                'reason': reason,
                'timestamp': int(time.time()),
            }
            if not self._call_webhook_url:
                self.get_logger().warning(f'CALL ESCALATION placeholder -> {json.dumps(payload)}')
                return
            try:
                request = urllib.request.Request(
                    self._call_webhook_url,
                    data=json.dumps(payload).encode('utf-8'),
                    headers={'Content-Type': 'application/json'},
                    method='POST',
                )
                with urllib.request.urlopen(request, timeout=5) as response:
                    self.get_logger().warning(
                        f'Call webhook sent for {zone_id} ({reason}) status={response.status}'
                    )
            except Exception as exc:
                self.get_logger().error(f'Call webhook failed for {zone_id}: {exc}')

        threading.Thread(target=_worker, daemon=True).start()

    def _send_http_json(self, handler: BaseHTTPRequestHandler, payload: Any, status: int = 200) -> None:
        data = json.dumps(payload).encode('utf-8')
        handler.send_response(status)
        handler.send_header('Content-Type', 'application/json; charset=utf-8')
        handler.send_header('Content-Length', str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)

    def _send_http_html(self, handler: BaseHTTPRequestHandler, html_body: str, status: int = 200) -> None:
        data = html_body.encode('utf-8')
        handler.send_response(status)
        handler.send_header('Content-Type', 'text/html; charset=utf-8')
        handler.send_header('Content-Length', str(len(data)))
        handler.end_headers()
        handler.wfile.write(data)


def main(args=None):
    rclpy.init(args=args)
    node = SecurityControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
