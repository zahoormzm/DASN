"""Escalation node – watches per-zone threat levels and triggers dispatch
commands and alarm events when thresholds are crossed."""

from typing import Dict, List

import rclpy
from rclpy.node import Node

from dasn_msgs.msg import AlarmEvent, DispatchCommand, ThreatLevel
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image


# ---------------------------------------------------------------------------
# Escalation node
# ---------------------------------------------------------------------------
class EscalationNode(Node):
    """Subscribes to per-zone threat levels, escalates when appropriate."""

    _LEVEL_NAMES = {
        0: 'Idle',
        1: 'Watch',
        2: 'Alert',
        3: 'Dispatch',
        4: 'Alarm',
        5: 'Tamper',
    }

    def __init__(self) -> None:
        super().__init__('escalation', namespace='threat')

        # ---- Parameters ----
        self.declare_parameter(
            'zone_ids', ['FRONT_DOOR', 'WINDOW_1', 'BACK_DOOR']
        )
        self.zone_ids: List[str] = (
            self.get_parameter('zone_ids')
            .get_parameter_value()
            .string_array_value
        )

        self.declare_parameter('dispatch_cooldown_sec', 60.0)

        # Zone waypoints – flat list: [x1,y1,z1, x2,y2,z2, ...]
        # One waypoint per zone in zone_ids order.
        default_waypoints = [0.0] * (len(self.zone_ids) * 3)
        self.declare_parameter('zone_waypoints', default_waypoints)

        # ---- Publishers ----
        self._dispatch_pub = self.create_publisher(
            DispatchCommand, '/bot/dispatch', 10
        )
        self._alarm_pub = self.create_publisher(
            AlarmEvent, '/alerts/alarm', 10
        )

        # ---- Per-zone tracking ----
        self._prev_levels: Dict[str, int] = {z: 0 for z in self.zone_ids}
        self._last_dispatch_time: Dict[str, float] = {
            z: 0.0 for z in self.zone_ids
        }

        # ---- Subscriptions ----
        for zid in self.zone_ids:
            self.create_subscription(
                ThreatLevel,
                f'/zone/{zid}/threat_level',
                lambda msg, z=zid: self._cb_threat(msg, z),
                10,
            )

        self.get_logger().info(
            f'Escalation node started – zones: {self.zone_ids}'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _get_zone_waypoint(self, zone_id: str) -> List[Point]:
        """Return the waypoint(s) for the given zone as a list of Points."""
        raw = (
            self.get_parameter('zone_waypoints')
            .get_parameter_value()
            .double_array_value
        )
        try:
            idx = self.zone_ids.index(zone_id)
        except ValueError:
            return []

        base = idx * 3
        if base + 2 < len(raw):
            pt = Point()
            pt.x = raw[base]
            pt.y = raw[base + 1]
            pt.z = raw[base + 2]
            return [pt]
        return []

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cooldown_ok(self, zone_id: str) -> bool:
        cooldown = self.get_parameter('dispatch_cooldown_sec').value
        elapsed = self._now_sec() - self._last_dispatch_time.get(zone_id, 0.0)
        return elapsed >= cooldown

    # ------------------------------------------------------------------
    # Threat callback
    # ------------------------------------------------------------------
    def _cb_threat(self, msg: ThreatLevel, zone_id: str) -> None:
        level: int = msg.level
        prev = self._prev_levels.get(zone_id, 0)
        level_name = self._LEVEL_NAMES.get(level, 'Unknown')

        # ---- De-escalation detection ----
        if prev >= 3 and level < 3:
            self.get_logger().info(
                f'Zone {zone_id} de-escalated from '
                f'{self._LEVEL_NAMES.get(prev, "?")} to {level_name}'
            )

        # ---- Level 0 – Idle ----
        if level == 0:
            pass  # no action

        # ---- Level 1 – Watch ----
        elif level == 1:
            self.get_logger().info(
                f'[Watch] Zone {zone_id} – score={msg.score:.3f}, '
                f'factors={msg.contributing_factors}'
            )

        # ---- Level 2 – Alert ----
        elif level == 2:
            self.get_logger().warn(
                f'[Alert] Zone {zone_id} – score={msg.score:.3f}, '
                f'factors={msg.contributing_factors}'
            )

        # ---- Level 3 – Dispatch ----
        elif level == 3:
            self.get_logger().warn(
                f'[Dispatch] Zone {zone_id} – score={msg.score:.3f}'
            )
            self._try_dispatch(zone_id, msg, mission_type='investigate')

        # ---- Level 4 – Alarm ----
        elif level == 4:
            self.get_logger().error(
                f'[Alarm] Zone {zone_id} – score={msg.score:.3f}'
            )
            self._publish_alarm(zone_id, msg, level=4)

        # ---- Level 5 – Tamper ----
        elif level == 5:
            self.get_logger().fatal(
                f'[Tamper] Zone {zone_id} – score={msg.score:.3f}'
            )
            self._publish_alarm(zone_id, msg, level=5)
            self._try_dispatch(zone_id, msg, mission_type='investigate')

        self._prev_levels[zone_id] = level

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------
    def _try_dispatch(
        self,
        zone_id: str,
        threat_msg: ThreatLevel,
        mission_type: str,
    ) -> None:
        if not self._cooldown_ok(zone_id):
            self.get_logger().info(
                f'Dispatch to {zone_id} suppressed (cooldown active)'
            )
            return

        cmd = DispatchCommand()
        cmd.target_zone = zone_id
        cmd.waypoints = self._get_zone_waypoint(zone_id)
        cmd.threat_level = threat_msg.level
        cmd.mission_type = mission_type
        self._dispatch_pub.publish(cmd)

        self._last_dispatch_time[zone_id] = self._now_sec()
        self.get_logger().info(
            f'Published DispatchCommand → zone={zone_id}, '
            f'mission={mission_type}'
        )

    # ------------------------------------------------------------------
    # Alarm
    # ------------------------------------------------------------------
    def _publish_alarm(
        self,
        zone_id: str,
        threat_msg: ThreatLevel,
        level: int,
    ) -> None:
        alarm = AlarmEvent()
        alarm.level = level
        alarm.zone_id = zone_id
        alarm.reason = ', '.join(threat_msg.contributing_factors)
        alarm.evidence_frame = Image()  # empty – no frame available here
        alarm.timestamp = self.get_clock().now().to_msg()
        self._alarm_pub.publish(alarm)
        self.get_logger().info(
            f'Published AlarmEvent → zone={zone_id}, level={level}, '
            f'reason={alarm.reason}'
        )


# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = EscalationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
