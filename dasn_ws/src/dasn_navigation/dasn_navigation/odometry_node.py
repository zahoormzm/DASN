import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (
    TransformStamped,
    Quaternion,
)

from tf2_ros import TransformBroadcaster

from dasn_msgs.msg import EncoderTicks


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry', namespace='bot')

        # Parameters
        self.declare_parameter('alpha', 0.98)
        self.alpha = self.get_parameter('alpha').value

        # Encoder constants
        self.wheel_circumference = 0.107   # meters (pi * 0.034)
        self.pulses_per_rev = 300
        self.ticks_to_meters = self.wheel_circumference / self.pulses_per_rev  # ~0.000357
        self.track_width = 0.16            # meters

        # State
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0  # radians

        self.prev_left_ticks = None
        self.prev_right_ticks = None

        # IMU state
        self.gyro_delta = 0.0  # accumulated gyro delta since last encoder tick
        self.last_imu_time = None

        # Velocity tracking
        self.last_encoder_time = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.sub_encoder = self.create_subscription(
            EncoderTicks, '/odom/encoders', self.encoder_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        self.get_logger().info('Odometry node started')

    def imu_callback(self, msg: Imu):
        """Accumulate gyroscope Z delta between encoder updates."""
        now = self.get_clock().now()
        if self.last_imu_time is not None:
            dt = (now - self.last_imu_time).nanoseconds * 1e-9
            if dt > 0.0 and dt < 1.0:
                gz = msg.angular_velocity.z
                self.gyro_delta += gz * dt
        self.last_imu_time = now

    def encoder_callback(self, msg: EncoderTicks):
        """Process encoder ticks and update odometry."""
        left = msg.left_ticks
        right = msg.right_ticks
        now = self.get_clock().now()

        if self.prev_left_ticks is None:
            # First reading: store and return
            self.prev_left_ticks = left
            self.prev_right_ticks = right
            self.last_encoder_time = now
            return

        # Compute deltas
        delta_left_ticks = left - self.prev_left_ticks
        delta_right_ticks = right - self.prev_right_ticks
        self.prev_left_ticks = left
        self.prev_right_ticks = right

        d_left = delta_left_ticks * self.ticks_to_meters
        d_right = delta_right_ticks * self.ticks_to_meters

        distance_center = (d_left + d_right) / 2.0
        delta_heading_enc = (d_right - d_left) / self.track_width

        # Complementary filter: fuse encoder delta with gyro delta
        delta_heading_fused = self.alpha * delta_heading_enc + (1.0 - self.alpha) * self.gyro_delta
        self.heading += delta_heading_fused
        self.gyro_delta = 0.0  # reset for next encoder cycle

        # Normalize heading to [-pi, pi]
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))

        # Update position
        self.x += distance_center * math.cos(self.heading)
        self.y += distance_center * math.sin(self.heading)

        # Compute velocity
        dt = (now - self.last_encoder_time).nanoseconds * 1e-9 if self.last_encoder_time else 0.0
        self.last_encoder_time = now
        if dt > 0.0 and dt < 2.0:
            self.linear_velocity = distance_center / dt
            self.angular_velocity = delta_heading_enc / dt
        else:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        # Publish odometry
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.heading)
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = self.angular_velocity
        self.pub_odom.publish(odom)

        # Broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = yaw_to_quaternion(self.heading)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
