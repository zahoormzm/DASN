import math
import re
import threading
import time
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from dasn_msgs.msg import BotStatus, DispatchCommand

try:
    import serial
except ImportError:
    serial = None


POSE_RE = re.compile(
    r'POSE x=(?P<x>-?\d+(?:\.\d+)?) y=(?P<y>-?\d+(?:\.\d+)?) '
    r'heading=(?P<heading>-?\d+(?:\.\d+)?) state=(?P<state>\d+)'
)


class DemoBotBridgeNode(Node):
    def __init__(self):
        super().__init__('demo_bot_bridge', namespace='bot')

        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('pose_poll_interval_s', 1.0)
        self.declare_parameter('status_publish_interval_s', 0.5)
        self.declare_parameter('door_a_x', 1.2)
        self.declare_parameter('door_a_y', 0.0)
        self.declare_parameter('door_b_x', 0.0)
        self.declare_parameter('door_b_y', 0.0)
        self.declare_parameter('dock_x', 0.0)
        self.declare_parameter('dock_y', 0.0)
        self.declare_parameter(
            'door_b_patrol_points',
            [0.0, 0.0, 0.25, 0.0, 0.25, 0.25, 0.0, 0.25],
        )

        self._serial_port = str(self.get_parameter('serial_port').value)
        self._baud_rate = int(self.get_parameter('baud_rate').value)
        self._pose_poll_interval_s = float(self.get_parameter('pose_poll_interval_s').value)
        self._status_publish_interval_s = float(
            self.get_parameter('status_publish_interval_s').value
        )

        self._named_points = {
            'door_a': self._make_point(
                float(self.get_parameter('door_a_x').value),
                float(self.get_parameter('door_a_y').value),
            ),
            'door_b': self._make_point(
                float(self.get_parameter('door_b_x').value),
                float(self.get_parameter('door_b_y').value),
            ),
            'dock': self._make_point(
                float(self.get_parameter('dock_x').value),
                float(self.get_parameter('dock_y').value),
            ),
        }
        self._patrol_points = self._flat_to_points(
            list(self.get_parameter('door_b_patrol_points').value)
        )
        if not self._patrol_points:
            self._patrol_points = [self._named_points['door_b']]

        self._serial = None
        self._serial_lock = threading.Lock()
        self._running = True

        self._status_pub = self.create_publisher(BotStatus, '/bot/status', 10)
        self.create_subscription(DispatchCommand, '/bot/dispatch', self._dispatch_cb, 10)

        self._pose_x = 0.0
        self._pose_y = 0.0
        self._heading_deg = 90.0
        self._raw_nav_state = 0
        self._mode = 'DOCKED'
        self._target_zone = 'dock'
        self._distance_remaining = 0.0
        self._goal_point = self._named_points['dock']
        self._pending_points: list[Point] = []
        self._active_waypoint_idx = -1
        self._cycle_patrol = False
        self._awaiting_arrival = False

        self._connect_serial()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        self.create_timer(self._pose_poll_interval_s, self._poll_pose)
        self.create_timer(self._status_publish_interval_s, self._publish_status)

        self.get_logger().info(
            f'Demo bot bridge ready on {self._serial_port} @ {self._baud_rate}'
        )

    def _make_point(self, x: float, y: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        return point

    def _flat_to_points(self, values: Iterable[float]) -> list[Point]:
        raw = [float(v) for v in values]
        points: list[Point] = []
        for idx in range(0, len(raw) - 1, 2):
            points.append(self._make_point(raw[idx], raw[idx + 1]))
        return points

    def _connect_serial(self) -> bool:
        if serial is None:
            self.get_logger().error('pyserial is not installed')
            return False
        try:
            self._serial = serial.Serial(self._serial_port, self._baud_rate, timeout=1.0)
            self.get_logger().info(f'Connected to bot serial {self._serial_port}')
            return True
        except Exception as exc:
            self.get_logger().warn(f'Bot serial connect failed: {exc}')
            self._serial = None
            return False

    def _read_loop(self) -> None:
        while self._running:
            if self._serial is None or not self._serial.is_open:
                time.sleep(1.0)
                self._connect_serial()
                continue

            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
                if line:
                    self._handle_serial_line(line)
            except Exception as exc:
                self.get_logger().warn(f'Bot serial read failed: {exc}')
                with self._serial_lock:
                    try:
                        if self._serial is not None:
                            self._serial.close()
                    except Exception:
                        pass
                    self._serial = None

    def _handle_serial_line(self, line: str) -> None:
        match = POSE_RE.search(line)
        if match:
            self._pose_x = float(match.group('x'))
            self._pose_y = float(match.group('y'))
            self._heading_deg = float(match.group('heading'))
            self._raw_nav_state = int(match.group('state'))
            self._update_distance_remaining()
            return

        if '[nav] complete' in line.lower():
            self._awaiting_arrival = False
            self._advance_after_arrival()
            return

        if '[nav] target=' in line.lower():
            self._awaiting_arrival = True
            return

        if 'Emergency stop'.lower() in line.lower():
            self._mode = 'STOPPED'
            self._awaiting_arrival = False

    def _dispatch_cb(self, msg: DispatchCommand) -> None:
        mission = (msg.mission_type or '').strip().lower()
        if not mission:
            return

        if mission == 'stop':
            self._pending_points = []
            self._cycle_patrol = False
            self._mode = 'STOPPED'
            self._awaiting_arrival = False
            self._send_line('STOP')
            return

        if mission == 'zero':
            self._send_line('ZERO')
            return

        if msg.waypoints:
            points = [self._copy_point(point) for point in msg.waypoints]
        else:
            points = self._points_for_mission(mission)

        if not points:
            self.get_logger().warn(f'No waypoint mapping for bot mission "{mission}"')
            return

        self._pending_points = points
        self._active_waypoint_idx = 0
        self._cycle_patrol = mission == 'patrol_door_b'
        self._target_zone = msg.target_zone or self._target_zone_for_mission(mission)
        self._mode = self._mode_for_mission(mission)
        self._send_go(points[0])

    def _copy_point(self, src: Point) -> Point:
        point = Point()
        point.x = src.x
        point.y = src.y
        point.z = src.z
        return point

    def _points_for_mission(self, mission: str) -> list[Point]:
        if mission == 'patrol_door_b':
            return [self._copy_point(point) for point in self._patrol_points]
        if mission in {'go_door_a', 'investigate_door_a'}:
            return [self._copy_point(self._named_points['door_a'])]
        if mission == 'go_door_b':
            return [self._copy_point(self._named_points['door_b'])]
        if mission in {'return_dock', 'dock'}:
            return [self._copy_point(self._named_points['dock'])]
        return []

    def _mode_for_mission(self, mission: str) -> str:
        if mission == 'patrol_door_b':
            return 'PATROLLING_DOOR_B'
        if mission in {'go_door_a', 'investigate_door_a'}:
            return 'DISPATCH_TO_DOOR_A'
        if mission == 'go_door_b':
            return 'GUARDING_DOOR_B'
        if mission in {'return_dock', 'dock'}:
            return 'RETURNING'
        return 'MANUAL'

    def _target_zone_for_mission(self, mission: str) -> str:
        if mission in {'go_door_a', 'investigate_door_a'}:
            return 'FRONT_DOOR'
        if mission == 'go_door_b':
            return 'DOOR_B'
        if mission in {'return_dock', 'dock'}:
            return 'dock'
        return 'door_b'

    def _advance_after_arrival(self) -> None:
        if not self._pending_points:
            return

        if self._cycle_patrol:
            self._active_waypoint_idx = (self._active_waypoint_idx + 1) % len(self._pending_points)
            self._mode = 'PATROLLING_DOOR_B'
            self._send_go(self._pending_points[self._active_waypoint_idx])
            return

        if self._mode == 'DISPATCH_TO_DOOR_A':
            self._mode = 'INVESTIGATING_DOOR_A'
        elif self._mode == 'RETURNING':
            self._mode = 'DOCKED'
            self._target_zone = 'dock'
        elif self._mode == 'GUARDING_DOOR_B':
            self._mode = 'GUARDING_DOOR_B'

        self._update_distance_remaining()

    def _send_go(self, point: Point) -> None:
        self._goal_point = self._copy_point(point)
        self._update_distance_remaining()
        self._send_line(f'GO,{point.x:.3f},{point.y:.3f}')
        self._awaiting_arrival = True

    def _send_line(self, command: str) -> None:
        with self._serial_lock:
            if self._serial is None or not self._serial.is_open:
                self.get_logger().warn(f'Bot serial unavailable, dropped command: {command}')
                return
            try:
                self._serial.write((command.strip() + '\n').encode('utf-8'))
                self.get_logger().info(f'Bot <= {command}')
            except Exception as exc:
                self.get_logger().warn(f'Bot serial write failed: {exc}')

    def _poll_pose(self) -> None:
        self._send_line('POSE')

    def _update_distance_remaining(self) -> None:
        dx = self._goal_point.x - self._pose_x
        dy = self._goal_point.y - self._pose_y
        self._distance_remaining = math.sqrt(dx * dx + dy * dy)

    def _publish_status(self) -> None:
        self._update_distance_remaining()
        msg = BotStatus()
        msg.status = self._mode
        msg.position = self._make_point(self._pose_x, self._pose_y)
        msg.heading_deg = float(self._heading_deg)
        msg.battery_v = 0.0
        msg.target_zone = self._target_zone
        msg.distance_remaining = float(self._distance_remaining)
        self._status_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        with self._serial_lock:
            try:
                if self._serial is not None:
                    self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DemoBotBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
