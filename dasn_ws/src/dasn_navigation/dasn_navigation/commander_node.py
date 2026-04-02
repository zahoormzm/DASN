import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from dasn_msgs.msg import BotStatus, DispatchCommand, StallEvent


class MissionState(Enum):
    DOCKED = 'DOCKED'
    NAVIGATING = 'NAVIGATING'
    SWEEPING = 'SWEEPING'
    RETURNING = 'RETURNING'
    STALLED = 'STALLED'


class CommanderNode(Node):

    def __init__(self):
        super().__init__('commander', namespace='bot')

        # Parameters
        self.declare_parameter('dock_position_x', 0.5)
        self.declare_parameter('dock_position_y', 0.5)
        self.declare_parameter('sweep_duration', 5.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('low_battery_voltage', 10.0)
        self.declare_parameter('critical_battery_voltage', 9.5)

        self.dock_x = self.get_parameter('dock_position_x').value
        self.dock_y = self.get_parameter('dock_position_y').value
        self.sweep_duration = self.get_parameter('sweep_duration').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.low_battery_voltage = self.get_parameter('low_battery_voltage').value
        self.critical_battery_voltage = self.get_parameter('critical_battery_voltage').value

        # State
        self.state = MissionState.DOCKED
        self.target_zone = ''
        self.battery_v = 12.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.distance_remaining = 0.0
        self.sweep_start_time = None
        self.stall_recovery_attempts = 0
        self.max_stall_recovery = 3
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Publishers
        self.pub_status = self.create_publisher(BotStatus, '/bot/status', 10)
        self.pub_dispatch = self.create_publisher(
            DispatchCommand, '/bot/dispatch', 10
        )

        # Subscribers
        self.sub_dispatch = self.create_subscription(
            DispatchCommand, '/bot/dispatch', self.dispatch_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.sub_battery = self.create_subscription(
            Float32, '/battery/voltage', self.battery_callback, 10
        )
        self.sub_stall = self.create_subscription(
            StallEvent, '/bot/stall_event', self.stall_callback, 10
        )

        # Status publish timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Mission state machine timer (10 Hz)
        self.mission_timer = self.create_timer(0.1, self.mission_tick)

        self.get_logger().info('Commander node started (state=DOCKED)')

    def dispatch_callback(self, msg: DispatchCommand):
        """Receive dispatch command and transition to NAVIGATING."""
        mission_type = (msg.mission_type or '').lower()
        is_return = mission_type in {'dock', 'return'}

        if is_return:
            self.target_zone = 'dock'
            self._set_goal_from_dispatch(msg)
            self.state = MissionState.RETURNING
            self.get_logger().info('Return-to-dock command received')
            return

        if self.state == MissionState.DOCKED:
            self.target_zone = msg.target_zone
            self._set_goal_from_dispatch(msg)
            self.state = MissionState.NAVIGATING
            self.stall_recovery_attempts = 0
            self.get_logger().info(
                f'Dispatch received: navigating to {self.target_zone}'
            )
        elif self.state in (MissionState.NAVIGATING, MissionState.SWEEPING):
            self.get_logger().warn(
                f'Ignoring dispatch to {msg.target_zone}: '
                f'currently {self.state.value}'
            )
        else:
            self.target_zone = msg.target_zone
            self._set_goal_from_dispatch(msg)
            self.state = MissionState.NAVIGATING
            self.get_logger().info(
                f'Dispatch override: navigating to {self.target_zone}'
            )

    def odom_callback(self, msg: Odometry):
        """Update current position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)
        self._update_distance_remaining()

    def battery_callback(self, msg: Float32):
        """Monitor battery voltage and auto-return if low."""
        self.battery_v = msg.data
        if self.battery_v < self.critical_battery_voltage and self.state not in (
            MissionState.DOCKED, MissionState.RETURNING
        ):
            self.get_logger().error(
                f'Critical battery ({self.battery_v:.2f}V) - returning to dock'
            )
            self._return_to_dock()
        elif self.battery_v < self.low_battery_voltage and self.state not in (
            MissionState.DOCKED, MissionState.RETURNING
        ):
            self.get_logger().warn(
                f'Low battery ({self.battery_v:.2f}V) - returning to dock'
            )
            self._return_to_dock()

    def stall_callback(self, msg: StallEvent):
        """Handle motor stall events."""
        self.get_logger().warn(
            f'Stall detected on motor {msg.motor_channel}'
        )
        prev_state = self.state
        self.state = MissionState.STALLED
        self.stall_recovery_attempts += 1

        if self.stall_recovery_attempts > self.max_stall_recovery:
            self.get_logger().error(
                'Max stall recovery attempts reached - returning to dock'
            )
            self._return_to_dock()
        else:
            self.get_logger().info(
                f'Stall recovery attempt {self.stall_recovery_attempts}'
                f'/{self.max_stall_recovery}'
            )
            # Resume previous navigation state after brief pause
            # The mission_tick will handle the STALLED state
            self._stall_previous_state = prev_state

    def mission_tick(self):
        """State machine update at 10 Hz."""
        if self.state == MissionState.NAVIGATING:
            self._tick_navigating()
        elif self.state == MissionState.SWEEPING:
            self._tick_sweeping()
        elif self.state == MissionState.RETURNING:
            self._tick_returning()
        elif self.state == MissionState.STALLED:
            self._tick_stalled()

    def _tick_navigating(self):
        """Check if we have arrived at the target zone."""
        if self.distance_remaining < self.goal_tol and self.target_zone:
            self.state = MissionState.SWEEPING
            self.sweep_start_time = self.get_clock().now()
            self.get_logger().info(
                f'Arrived at {self.target_zone} - starting sweep'
            )

    def _tick_sweeping(self):
        """Wait for sweep duration to complete."""
        if self.sweep_start_time is None:
            self.sweep_start_time = self.get_clock().now()

        elapsed = (
            self.get_clock().now() - self.sweep_start_time
        ).nanoseconds * 1e-9

        if elapsed >= self.sweep_duration:
            self.get_logger().info('Sweep complete - returning to dock')
            self._return_to_dock()

    def _tick_returning(self):
        """Check if we have arrived back at the dock."""
        dist_to_dock = math.sqrt(
            (self.current_x - self.dock_x) ** 2
            + (self.current_y - self.dock_y) ** 2
        )
        self.distance_remaining = dist_to_dock
        if dist_to_dock < self.goal_tol:
            self.state = MissionState.DOCKED
            self.target_zone = ''
            self.stall_recovery_attempts = 0
            self.get_logger().info('Docked successfully')

    def _tick_stalled(self):
        """Auto-recovery: resume previous state after a brief pause."""
        # Simple recovery: transition back to previous state
        prev = getattr(self, '_stall_previous_state', MissionState.DOCKED)
        if prev == MissionState.NAVIGATING:
            self.state = MissionState.NAVIGATING
            self.get_logger().info('Stall recovery: resuming navigation')
        elif prev == MissionState.RETURNING:
            self.state = MissionState.RETURNING
            self.get_logger().info('Stall recovery: resuming return')
        else:
            self._return_to_dock()

    def _return_to_dock(self):
        """Issue dispatch to dock and set state to RETURNING."""
        self.state = MissionState.RETURNING
        self.target_zone = 'dock'
        self.goal_x = self.dock_x
        self.goal_y = self.dock_y
        self._update_distance_remaining()

        dispatch = DispatchCommand()
        dispatch.target_zone = 'dock'
        dispatch.threat_level = 0
        dispatch.mission_type = 'return'
        dock_point = Point()
        dock_point.x = self.dock_x
        dock_point.y = self.dock_y
        dock_point.z = 0.0
        dispatch.waypoints = [dock_point]
        self.pub_dispatch.publish(dispatch)

    def _set_goal_from_dispatch(self, msg: DispatchCommand):
        """Update the active mission goal from a dispatch command."""
        if msg.waypoints:
            self.goal_x = msg.waypoints[-1].x
            self.goal_y = msg.waypoints[-1].y
        elif (msg.target_zone or '').lower() == 'dock':
            self.goal_x = self.dock_x
            self.goal_y = self.dock_y
        self._update_distance_remaining()

    def _update_distance_remaining(self):
        """Recompute distance to the current goal when one is active."""
        if not self.target_zone and self.state != MissionState.RETURNING:
            self.distance_remaining = 0.0
            return
        self.distance_remaining = math.sqrt(
            (self.current_x - self.goal_x) ** 2
            + (self.current_y - self.goal_y) ** 2
        )

    def publish_status(self):
        """Publish BotStatus at 1 Hz."""
        status = BotStatus()
        status.status = self.state.value
        status.position = Point()
        status.position.x = self.current_x
        status.position.y = self.current_y
        status.position.z = 0.0
        status.heading_deg = math.degrees(self.current_heading)
        status.battery_v = self.battery_v
        status.target_zone = self.target_zone

        # Compute distance remaining based on state
        if self.state == MissionState.RETURNING:
            status.distance_remaining = math.sqrt(
                (self.current_x - self.dock_x) ** 2
                + (self.current_y - self.dock_y) ** 2
            )
        else:
            status.distance_remaining = self.distance_remaining

        self.distance_remaining = status.distance_remaining
        self.pub_status.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
