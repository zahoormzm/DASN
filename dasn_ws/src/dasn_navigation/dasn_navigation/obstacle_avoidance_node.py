import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from dasn_msgs.msg import DepthGrid


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance', namespace='bot')

        # Parameters
        self.declare_parameter('min_distance_mm', 200)
        self.declare_parameter('slow_distance_mm', 300)

        self.min_dist = self.get_parameter('min_distance_mm').value
        self.slow_dist = self.get_parameter('slow_distance_mm').value

        # Latest depth grid (8x8 = 64 values, row-major)
        self.depth_grid = None

        # Latest rear ultrasonic distance (mm)
        self.rear_distance_mm = 9999

        # Latest commanded velocity
        self.latest_cmd = Twist()

        # Publishers
        self.pub_safe = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # Subscribers
        self.sub_depth = self.create_subscription(
            DepthGrid, '/tof/depth_grid', self.depth_callback, 10
        )
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.sub_rear = self.create_subscription(
            Float32, '/sonar/rear_mm', self.rear_distance_callback, 10
        )

        self.get_logger().info(
            f'Obstacle avoidance started '
            f'(min={self.min_dist}mm, slow={self.slow_dist}mm)'
        )

    def depth_callback(self, msg: DepthGrid):
        """Store the latest 8x8 depth grid."""
        self.depth_grid = msg.distances_mm

    def cmd_vel_callback(self, msg: Twist):
        """Apply safety filtering to incoming velocity commands."""
        self.latest_cmd = msg
        safe = self.compute_safe_velocity(msg)
        self.pub_safe.publish(safe)

    def rear_distance_callback(self, msg: Float32):
        """Store the latest rear ultrasonic distance for reverse safety checks."""
        self.rear_distance_mm = msg.data

    def compute_safe_velocity(self, cmd: Twist) -> Twist:
        """Filter velocity based on ToF depth grid readings."""
        safe = Twist()
        safe.linear.x = cmd.linear.x
        safe.linear.y = cmd.linear.y
        safe.linear.z = cmd.linear.z
        safe.angular.x = cmd.angular.x
        safe.angular.y = cmd.angular.y
        safe.angular.z = cmd.angular.z

        if self.depth_grid is None or len(self.depth_grid) < 64:
            # No depth data available; pass through unchanged
            return safe

        grid = self.depth_grid  # flat list of 64 values, row-major 8x8

        # Check central 4x4 zone (rows 2-5, cols 2-5) for emergency stop
        central_stop = False
        for row in range(2, 6):
            for col in range(2, 6):
                idx = row * 8 + col
                if grid[idx] < self.min_dist and grid[idx] > 0:
                    central_stop = True
                    break
            if central_stop:
                break

        if central_stop:
            # Stop: obstacle directly ahead
            safe.linear.x = 0.0
            safe.linear.y = 0.0
            safe.angular.z = 0.0
            self.get_logger().debug('Central obstacle detected - stopping')
            return safe

        # Check left side (cols 0-3) for close obstacles
        left_close = False
        for row in range(8):
            for col in range(0, 4):
                idx = row * 8 + col
                if grid[idx] < self.slow_dist and grid[idx] > 0:
                    left_close = True
                    break
            if left_close:
                break

        # Check right side (cols 4-7) for close obstacles
        right_close = False
        for row in range(8):
            for col in range(4, 8):
                idx = row * 8 + col
                if grid[idx] < self.slow_dist and grid[idx] > 0:
                    right_close = True
                    break
            if right_close:
                break

        if left_close and right_close:
            # Both sides blocked: slow down significantly
            safe.linear.x = cmd.linear.x * 0.3
        elif left_close:
            # Obstacle on left: veer right
            safe.angular.z = cmd.angular.z - 0.5
            safe.linear.x = cmd.linear.x * 0.7
            self.get_logger().debug('Left obstacle - veering right')
        elif right_close:
            # Obstacle on right: veer left
            safe.angular.z = cmd.angular.z + 0.5
            safe.linear.x = cmd.linear.x * 0.7
            self.get_logger().debug('Right obstacle - veering left')

        # Reverse checking with rear ultrasonic
        if cmd.linear.x < 0 and self.rear_distance_mm < self.min_dist:
            safe.linear.x = 0.0
            self.get_logger().debug('Rear obstacle - blocking reverse')

        return safe


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
