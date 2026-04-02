import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from dasn_msgs.msg import EncoderTicks, StallEvent


class StallDetectorNode(Node):

    def __init__(self):
        super().__init__('stall_detector', namespace='bot')

        # Parameters
        self.declare_parameter('stall_encoder_timeout_s', 0.5)
        self.declare_parameter('stall_min_pwm', 30)

        self.timeout = self.get_parameter('stall_encoder_timeout_s').value
        self.min_pwm = self.get_parameter('stall_min_pwm').value

        # State tracking
        self.last_enc_left = None
        self.last_enc_right = None
        self.last_enc_time = None
        self.commanding_motion = False
        self.stall_start_left = None
        self.stall_start_right = None
        self.stall_published_left = False
        self.stall_published_right = False

        # Publishers
        self.pub_stall = self.create_publisher(StallEvent, '/bot/stall_event', 10)

        # Subscribers
        self.sub_encoder = self.create_subscription(
            EncoderTicks, '/odom/encoders', self.encoder_callback, 10
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.get_logger().info(
            f'Stall detector started (encoder-based, '
            f'timeout={self.timeout}s, min_pwm={self.min_pwm})'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Track whether we are commanding the motors to move."""
        self.commanding_motion = (
            abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        )
        if not self.commanding_motion:
            # Reset stall tracking when not commanding motion
            self.stall_start_left = None
            self.stall_start_right = None
            self.stall_published_left = False
            self.stall_published_right = False

    def encoder_callback(self, msg: EncoderTicks):
        """Check if encoders are moving when motion is commanded."""
        now = self.get_clock().now()

        if not self.commanding_motion:
            self.last_enc_left = msg.left_ticks
            self.last_enc_right = msg.right_ticks
            self.last_enc_time = now
            return

        if self.last_enc_left is None:
            self.last_enc_left = msg.left_ticks
            self.last_enc_right = msg.right_ticks
            self.last_enc_time = now
            return

        left_moving = msg.left_ticks != self.last_enc_left
        right_moving = msg.right_ticks != self.last_enc_right

        # Left motor stall check
        if not left_moving:
            if self.stall_start_left is None:
                self.stall_start_left = now
            elif not self.stall_published_left:
                elapsed = (now - self.stall_start_left).nanoseconds * 1e-9
                if elapsed >= self.timeout:
                    self._publish_stall(1, now)
                    self.stall_published_left = True
        else:
            self.stall_start_left = None
            self.stall_published_left = False

        # Right motor stall check
        if not right_moving:
            if self.stall_start_right is None:
                self.stall_start_right = now
            elif not self.stall_published_right:
                elapsed = (now - self.stall_start_right).nanoseconds * 1e-9
                if elapsed >= self.timeout:
                    self._publish_stall(2, now)
                    self.stall_published_right = True
        else:
            self.stall_start_right = None
            self.stall_published_right = False

        self.last_enc_left = msg.left_ticks
        self.last_enc_right = msg.right_ticks
        self.last_enc_time = now

    def _publish_stall(self, channel: int, now):
        """Publish a stall event."""
        event = StallEvent()
        event.motor_channel = channel
        event.timestamp = now.to_msg()
        self.pub_stall.publish(event)
        self.get_logger().warn(f'Stall detected: motor {channel} (encoder not moving)')


def main(args=None):
    rclpy.init(args=args)
    node = StallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
