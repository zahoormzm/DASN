"""Threat fusion node – aggregates sensor and ML data per zone into a
unified threat score and level, published at 2 Hz."""

import math
from datetime import datetime, timezone
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from dasn_msgs.msg import (
    AuthEvent,
    BotStatus,
    FaceIdentity,
    GasEvent,
    PoseAnalysis,
    SoundEvent,
    ThreatLevel,
    ZoneSensorData,
)
from vision_msgs.msg import Detection2DArray


# ---------------------------------------------------------------------------
# Per-zone state container
# ---------------------------------------------------------------------------
class _ZoneState:
    """Holds the latest readings from every input topic for one zone."""

    def __init__(self) -> None:
        # Sensor board
        self.radar_presence: bool = False
        self.radar_confidence: float = 0.0
        self.pir_triggered: bool = False
        self.sound_transient: bool = False
        self.sound_level_db: float = 0.0
        self.gas_resistance: float = 0.0

        # ML pipeline
        self.face_unknown: bool = False
        self.face_confidence: float = 0.0
        self.sound_anomaly: bool = False
        self.sound_anomaly_confidence: float = 0.0
        self.pose_suspicious: bool = False
        self.pose_confidence: float = 0.0
        self.object_detections: int = 0
        self.gas_event_severity: float = 0.0

        # Auth
        self.auth_failure: bool = False

        # History: list of (timestamp_seconds, score) tuples
        self.history: List[tuple] = []


# ---------------------------------------------------------------------------
# Fusion node
# ---------------------------------------------------------------------------
class FusionNode(Node):
    """Subscribes to sensor / ML / auth topics, fuses per-zone threat."""

    def __init__(self) -> None:
        super().__init__('fusion', namespace='threat')

        # ---- Zone IDs parameter ----
        self.declare_parameter(
            'zone_ids', ['FRONT_DOOR', 'WINDOW_1', 'BACK_DOOR']
        )
        self.zone_ids: List[str] = (
            self.get_parameter('zone_ids')
            .get_parameter_value()
            .string_array_value
        )

        # ---- Weight parameters ----
        self.declare_parameter('weight.radar_presence', 0.25)
        self.declare_parameter('weight.pir_motion', 0.10)
        self.declare_parameter('weight.sound_anomaly', 0.20)
        self.declare_parameter('weight.unknown_face', 0.25)
        self.declare_parameter('weight.gas_smoke_anomaly', 0.10)
        self.declare_parameter('weight.auth_failure', 0.10)

        # ---- Time-of-day parameters ----
        self.declare_parameter('nighttime_start', 22)
        self.declare_parameter('nighttime_end', 6)
        self.declare_parameter('nighttime_factor', 2.0)

        # ---- Zone-history parameters ----
        self.declare_parameter('zone_history_max', 1.5)
        self.declare_parameter('zone_history_decay_hours', 24)
        self.declare_parameter('espcam_zone_id', '')

        # ---- Per-zone state & publishers ----
        self._zones: Dict[str, _ZoneState] = {}
        self._threat_pubs: Dict[str, object] = {}
        self._active_bot_zone = ''

        for zid in self.zone_ids:
            self._zones[zid] = _ZoneState()
            self._threat_pubs[zid] = self.create_publisher(
                ThreatLevel, f'/zone/{zid}/threat_level', 10
            )

        # ---- Subscriptions ----
        self._create_subscriptions()

        # ---- 2 Hz fusion timer ----
        self.create_timer(0.5, self._fusion_tick)

        self.get_logger().info(
            f'Fusion node started – zones: {self.zone_ids}'
        )

    # ------------------------------------------------------------------
    # Subscription setup
    # ------------------------------------------------------------------
    def _create_subscriptions(self) -> None:
        # Per-zone sensor board
        for zid in self.zone_ids:
            self.create_subscription(
                ZoneSensorData,
                f'/zone/{zid}/sensors',
                lambda msg, z=zid: self._cb_zone_sensor(msg, z),
                10,
            )

        # Global ML topics (zone inferred from message or applied to all)
        self.create_subscription(
            FaceIdentity, '/ml/face_identity', self._cb_face, 10
        )
        self.create_subscription(
            SoundEvent, '/ml/sound_events', self._cb_sound, 10
        )
        self.create_subscription(
            PoseAnalysis, '/ml/poses', self._cb_pose, 10
        )
        self.create_subscription(
            Detection2DArray, '/ml/objects', self._cb_objects, 10
        )
        self.create_subscription(
            GasEvent, '/ml/gas_events', self._cb_gas, 10
        )
        self.create_subscription(
            AuthEvent, '/security/auth_event', self._cb_auth, 10
        )
        self.create_subscription(
            BotStatus, '/bot/status', self._cb_bot_status, 10
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cb_zone_sensor(self, msg: ZoneSensorData, zone_id: str) -> None:
        zs = self._zones.get(zone_id)
        if zs is None:
            return
        zs.radar_presence = msg.radar_presence
        zs.radar_confidence = msg.radar_confidence
        zs.pir_triggered = msg.pir_triggered
        zs.sound_transient = msg.sound_transient
        zs.sound_level_db = msg.sound_level_db
        zs.gas_resistance = msg.bme688_gas_resistance

    def _cb_face(self, msg: FaceIdentity) -> None:
        zone_id = self._resolve_face_zone(msg.source)
        if zone_id is None:
            return
        self._zones[zone_id].face_unknown = not msg.is_known
        self._zones[zone_id].face_confidence = msg.confidence

    def _cb_sound(self, msg: SoundEvent) -> None:
        anomaly_classes = {
            'glass_breaking', 'scream', 'gunshot', 'explosion', 'siren',
        }
        is_anomaly = msg.classification.lower() in anomaly_classes
        zid = msg.zone_id
        if zid in self._zones:
            self._zones[zid].sound_anomaly = is_anomaly
            self._zones[zid].sound_anomaly_confidence = msg.confidence
        else:
            for zs in self._zones.values():
                zs.sound_anomaly = is_anomaly
                zs.sound_anomaly_confidence = msg.confidence

    def _cb_pose(self, msg: PoseAnalysis) -> None:
        suspicious_poses = {
            'climbing', 'crouching', 'running', 'fighting', 'lurking',
        }
        is_suspicious = any(
            p.lower() in suspicious_poses for p in msg.detected_poses
        )
        zone_id = self._active_bot_zone if self._active_bot_zone in self._zones else None
        if zone_id is None:
            return
        self._zones[zone_id].pose_suspicious = is_suspicious
        self._zones[zone_id].pose_confidence = msg.confidence

    def _cb_objects(self, msg: Detection2DArray) -> None:
        zone_id = self._active_bot_zone if self._active_bot_zone in self._zones else None
        if zone_id is None:
            return
        self._zones[zone_id].object_detections = len(msg.detections)

    def _cb_gas(self, msg: GasEvent) -> None:
        zid = msg.zone_id
        if zid in self._zones:
            self._zones[zid].gas_event_severity = msg.severity
        else:
            for zs in self._zones.values():
                zs.gas_event_severity = msg.severity

    def _cb_auth(self, msg: AuthEvent) -> None:
        zid = msg.zone_id
        if zid in self._zones:
            self._zones[zid].auth_failure = not msg.authorized
        else:
            for zs in self._zones.values():
                zs.auth_failure = not msg.authorized

    def _cb_bot_status(self, msg: BotStatus) -> None:
        status = (msg.status or '').lower()
        if msg.target_zone in self._zones and status in {
            'navigating', 'sweeping', 'returning'
        }:
            self._active_bot_zone = msg.target_zone
        elif status == 'docked':
            self._active_bot_zone = ''

    def _resolve_face_zone(self, source: str) -> str | None:
        if source in self._zones:
            return source
        if source == 'phone':
            return self._active_bot_zone if self._active_bot_zone in self._zones else None
        if source == 'espcam':
            espcam_zone_id = self.get_parameter('espcam_zone_id').value
            if espcam_zone_id in self._zones:
                return espcam_zone_id
        self.get_logger().debug(f'Ignoring face event from unmapped source: {source}')
        return None

    # ------------------------------------------------------------------
    # Fusion logic (runs at 2 Hz)
    # ------------------------------------------------------------------
    def _fusion_tick(self) -> None:
        w_radar = self.get_parameter('weight.radar_presence').value
        w_pir = self.get_parameter('weight.pir_motion').value
        w_sound = self.get_parameter('weight.sound_anomaly').value
        w_face = self.get_parameter('weight.unknown_face').value
        w_gas = self.get_parameter('weight.gas_smoke_anomaly').value
        w_auth = self.get_parameter('weight.auth_failure').value

        night_start = self.get_parameter('nighttime_start').value
        night_end = self.get_parameter('nighttime_end').value
        night_factor = self.get_parameter('nighttime_factor').value

        hist_max = self.get_parameter('zone_history_max').value
        hist_decay_h = self.get_parameter('zone_history_decay_hours').value

        now_local = datetime.now().astimezone()
        hour_now = now_local.hour
        now_stamp = now_local.timestamp()

        # Time-of-day multiplier
        if night_start > night_end:
            is_night = hour_now >= night_start or hour_now < night_end
        else:
            is_night = night_start <= hour_now < night_end
        time_mult = night_factor if is_night else 1.0

        for zid, zs in self._zones.items():
            factors: List[str] = []
            raw_score = 0.0

            # --- weighted components ---
            if zs.radar_presence:
                raw_score += w_radar * zs.radar_confidence
                factors.append(
                    f'radar_presence(conf={zs.radar_confidence:.2f})'
                )

            if zs.pir_triggered:
                raw_score += w_pir
                factors.append('pir_motion')

            if zs.sound_anomaly:
                raw_score += w_sound * zs.sound_anomaly_confidence
                factors.append(
                    f'sound_anomaly(conf={zs.sound_anomaly_confidence:.2f})'
                )

            if zs.sound_transient:
                raw_score += w_sound * 0.5  # partial weight for raw transient
                if 'sound_anomaly' not in ''.join(factors):
                    factors.append('sound_transient')

            if zs.face_unknown:
                raw_score += w_face * zs.face_confidence
                factors.append(
                    f'unknown_face(conf={zs.face_confidence:.2f})'
                )

            if zs.gas_event_severity > 0.0:
                raw_score += w_gas * zs.gas_event_severity
                factors.append(
                    f'gas_smoke(sev={zs.gas_event_severity:.2f})'
                )

            if zs.auth_failure:
                raw_score += w_auth
                factors.append('auth_failure')

            if zs.pose_suspicious:
                raw_score += 0.10 * zs.pose_confidence
                factors.append(
                    f'suspicious_pose(conf={zs.pose_confidence:.2f})'
                )

            # --- time-of-day multiplier ---
            score = raw_score * time_mult
            if is_night and raw_score > 0.0:
                factors.append('nighttime_multiplier')

            # --- zone history multiplier ---
            # Decay old events
            cutoff = now_stamp - hist_decay_h * 3600.0
            zs.history = [
                (ts, s) for ts, s in zs.history if ts > cutoff
            ]
            if zs.history:
                # More recent events contribute more; weighted average of
                # recency gives a boost up to hist_max.
                total_weight = 0.0
                for ts, s in zs.history:
                    age_h = (now_stamp - ts) / 3600.0
                    w = math.exp(-age_h / hist_decay_h)
                    total_weight += w * s
                hist_mult = min(1.0 + total_weight, hist_max)
                score *= hist_mult
                if hist_mult > 1.0:
                    factors.append(
                        f'zone_history(mult={hist_mult:.2f})'
                    )

            # Clamp to [0, 1]
            score = max(0.0, min(1.0, score))

            # Record in history if non-trivial
            if score >= 0.1:
                zs.history.append((now_stamp, score))

            # --- map score to level ---
            level = self._score_to_level(score)

            # --- publish ---
            msg = ThreatLevel()
            msg.zone_id = zid
            msg.score = score
            msg.level = level
            msg.contributing_factors = factors
            msg.timestamp = self.get_clock().now().to_msg()
            self._threat_pubs[zid].publish(msg)

    # ------------------------------------------------------------------
    @staticmethod
    def _score_to_level(score: float) -> int:
        if score < 0.1:
            return 0   # Idle
        elif score < 0.25:
            return 1   # Watch
        elif score < 0.45:
            return 2   # Alert
        elif score < 0.65:
            return 3   # Dispatch
        elif score < 0.85:
            return 4   # Alarm
        else:
            return 5   # Tamper


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
