import math
import heapq
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from dasn_msgs.msg import DispatchCommand, BotStatus


class AStarPlanner:
    """Grid-based A* path planner."""

    def __init__(self, grid, resolution, origin_x, origin_y):
        self.grid = grid           # 2D list: 0=free, 100=occupied
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x + self.resolution / 2.0
        wy = gy * self.resolution + self.origin_y + self.resolution / 2.0
        return wx, wy

    def is_valid(self, gx, gy):
        return 0 <= gx < self.cols and 0 <= gy < self.rows

    def is_free(self, gx, gy):
        if not self.is_valid(gx, gy):
            return False
        return self.grid[gy][gx] < 50

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def plan(self, start_world, goal_world):
        """Return a list of (world_x, world_y) waypoints from start to goal."""
        sx, sy = self.world_to_grid(start_world[0], start_world[1])
        gx, gy = self.world_to_grid(goal_world[0], goal_world[1])

        if not self.is_free(sx, sy):
            # Snap start to nearest free cell
            sx, sy = self._nearest_free(sx, sy)
        if not self.is_free(gx, gy):
            gx, gy = self._nearest_free(gx, gy)

        start = (sx, sy)
        goal = (gx, gy)

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        # 8-connected neighbors
        directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ]
        diag_cost = math.sqrt(2.0)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                node = current
                while node in came_from:
                    path.append(self.grid_to_world(node[0], node[1]))
                    node = came_from[node]
                path.reverse()
                return path

            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if not self.is_free(nx, ny):
                    continue
                move_cost = diag_cost if (dx != 0 and dy != 0) else 1.0
                tentative_g = g_score[current] + move_cost
                neighbor = (nx, ny)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        # No path found
        return []

    def _nearest_free(self, gx, gy):
        """BFS to find the nearest free cell."""
        from collections import deque
        visited = set()
        queue = deque()
        queue.append((gx, gy))
        visited.add((gx, gy))
        while queue:
            cx, cy = queue.popleft()
            if self.is_valid(cx, cy) and self.grid[cy][cx] < 50:
                return cx, cy
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if (nx, ny) not in visited and self.is_valid(nx, ny):
                        visited.add((nx, ny))
                        queue.append((nx, ny))
        return gx, gy


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator', namespace='bot')

        # Parameters
        self.declare_parameter('planner_type', 'astar')
        self.declare_parameter('map_file', '')
        self.declare_parameter('lookahead_distance', 0.15)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('goal_tolerance', 0.1)

        # Zone coordinate mapping (zone_name -> (x, y))
        self.declare_parameter('zone_config', '')

        self.planner_type = self.get_parameter('planner_type').value
        self.map_file = self.get_parameter('map_file').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_lin = self.get_parameter('max_linear_speed').value
        self.max_ang = self.get_parameter('max_angular_speed').value
        self.goal_tol = self.get_parameter('goal_tolerance').value

        # Publisher
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.sub_dispatch = self.create_subscription(
            DispatchCommand, '/bot/dispatch', self.dispatch_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.path = []
        self.path_index = 0
        self.navigating = False
        self.target_zone = ''

        # A* planner
        self.planner = None
        if self.planner_type == 'astar' and self.map_file:
            self._load_map()

        # Zone coordinates (default zones)
        self.zone_coords = {
            'zone_a': (1.0, 0.0),
            'zone_b': (2.0, 0.0),
            'zone_c': (2.0, 1.0),
            'zone_d': (1.0, 1.0),
            'dock': (0.5, 0.5),
        }
        self._load_zone_config()

        # Nav2 mode
        self.nav2_navigator = None
        if self.planner_type == 'nav2':
            self._init_nav2()

        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Navigator started with planner_type={self.planner_type}'
        )

    def _load_map(self):
        """Load occupancy grid from a YAML map file."""
        try:
            with open(self.map_file, 'r') as f:
                map_data = yaml.safe_load(f)
            if 'occupancy_grid' in map_data:
                occ = map_data.get('occupancy_grid', {})
                resolution = occ.get('resolution', 0.05)
                origin_cfg = occ.get('origin', {})
                origin = [
                    float(origin_cfg.get('x', 0.0)),
                    float(origin_cfg.get('y', 0.0)),
                    float(origin_cfg.get('theta', 0.0)),
                ]
                width = int(occ.get('width', 0))
                height = int(occ.get('height', 0))
                grid_raw = [[0 for _ in range(width)] for _ in range(height)]
                for wall in occ.get('wall_cells', []):
                    if not isinstance(wall, dict):
                        continue
                    if 'row' in wall:
                        row = int(wall['row'])
                        col_start = int(wall.get('col_start', 0))
                        col_end = int(wall.get('col_end', col_start))
                        if 0 <= row < height:
                            for col in range(max(0, col_start), min(width - 1, col_end) + 1):
                                grid_raw[row][col] = 100
                    elif 'col' in wall:
                        col = int(wall['col'])
                        row_start = int(wall.get('row_start', 0))
                        row_end = int(wall.get('row_end', row_start))
                        if 0 <= col < width:
                            for row in range(max(0, row_start), min(height - 1, row_end) + 1):
                                grid_raw[row][col] = 100
            else:
                resolution = map_data.get('resolution', 0.05)
                origin = map_data.get('origin', [0.0, 0.0, 0.0])
                grid_raw = map_data.get('grid', [])
            if grid_raw:
                self.planner = AStarPlanner(
                    grid_raw, resolution, origin[0], origin[1]
                )
                self.get_logger().info(
                    f'Loaded map: {len(grid_raw)}x{len(grid_raw[0])} '
                    f'res={resolution}m'
                )
            else:
                self.get_logger().warn('Map file has no grid data')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')

    def _load_zone_config(self):
        """Load zone coordinates from parameter if provided."""
        config_path = self.get_parameter('zone_config').value
        if not config_path:
            return
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            loaded = 0
            zones = data.get('zones', data) if isinstance(data, dict) else {}
            if isinstance(zones, dict):
                for zone_name, coords in zones.items():
                    if isinstance(coords, dict):
                        coords = coords.get('position', [])
                    if isinstance(coords, (list, tuple)) and len(coords) >= 2:
                        self.zone_coords[zone_name] = (
                            float(coords[0]),
                            float(coords[1]),
                        )
                        loaded += 1
                if 'dock' in data and isinstance(data['dock'], dict):
                    dock_pos = data['dock'].get('position', [])
                    if isinstance(dock_pos, (list, tuple)) and len(dock_pos) >= 2:
                        self.zone_coords['dock'] = (
                            float(dock_pos[0]),
                            float(dock_pos[1]),
                        )
                self.get_logger().info(
                    f'Loaded {loaded} zone coordinates'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to load zone config: {e}')

    def _init_nav2(self):
        """Initialize Nav2 BasicNavigator for nav2 planner mode."""
        try:
            from nav2_simple_commander.robot_navigator import BasicNavigator
            self.nav2_navigator = BasicNavigator()
            self.get_logger().info('Nav2 BasicNavigator initialized')
        except ImportError:
            self.get_logger().error(
                'nav2_simple_commander not available; '
                'falling back to astar planner'
            )
            self.planner_type = 'astar'

    def dispatch_callback(self, msg: DispatchCommand):
        """Receive a dispatch command and begin navigation."""
        zone = msg.target_zone
        self.target_zone = zone

        if msg.waypoints:
            # Use provided waypoints directly
            self.path = [
                (wp.x, wp.y) for wp in msg.waypoints
            ]
            self.path_index = 0
            self.navigating = True
            self.get_logger().info(
                f'Dispatch: navigating to {zone} via '
                f'{len(self.path)} waypoints'
            )
            return

        # Look up zone coordinates
        goal = self.zone_coords.get(zone)
        if goal is None:
            self.get_logger().error(f'Unknown zone: {zone}')
            return

        if self.planner_type == 'nav2' and self.nav2_navigator is not None:
            self._navigate_nav2(goal)
        else:
            self._navigate_astar(goal)

    def _navigate_astar(self, goal):
        """Plan a path using A* and start waypoint following."""
        start = (self.current_x, self.current_y)
        if self.planner is not None:
            self.path = self.planner.plan(start, goal)
        else:
            # No map loaded: direct path
            self.path = [goal]

        if not self.path:
            self.get_logger().warn('A* found no path; attempting direct')
            self.path = [goal]

        self.path_index = 0
        self.navigating = True
        self.get_logger().info(
            f'A* planned {len(self.path)} waypoints to '
            f'{self.target_zone} ({goal[0]:.2f}, {goal[1]:.2f})'
        )

    def _navigate_nav2(self, goal):
        """Send goal to Nav2 BasicNavigator."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.nav2_navigator.goToPose(pose)
        self.navigating = True
        self.get_logger().info(
            f'Nav2 goal sent: {self.target_zone} '
            f'({goal[0]:.2f}, {goal[1]:.2f})'
        )

    def odom_callback(self, msg: Odometry):
        """Update current pose from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Pure pursuit controller running at 10 Hz."""
        if not self.navigating:
            return

        # Nav2 mode: check status
        if self.planner_type == 'nav2' and self.nav2_navigator is not None:
            if self.nav2_navigator.isTaskComplete():
                self.navigating = False
                self.get_logger().info(
                    f'Nav2 navigation to {self.target_zone} complete'
                )
                self._stop()
            return

        # A* / waypoint following mode
        if self.path_index >= len(self.path):
            self.navigating = False
            self.get_logger().info(
                f'Reached {self.target_zone}'
            )
            self._stop()
            return

        # Find lookahead point
        target = None
        for i in range(self.path_index, len(self.path)):
            wx, wy = self.path[i]
            dist = math.sqrt(
                (wx - self.current_x) ** 2 + (wy - self.current_y) ** 2
            )
            if dist >= self.lookahead:
                target = (wx, wy)
                self.path_index = i
                break
        else:
            # All remaining waypoints are within lookahead; go to last
            target = self.path[-1]
            self.path_index = len(self.path) - 1

        tx, ty = target
        dist_to_goal = math.sqrt(
            (self.path[-1][0] - self.current_x) ** 2
            + (self.path[-1][1] - self.current_y) ** 2
        )

        if dist_to_goal < self.goal_tol:
            self.navigating = False
            self.get_logger().info(
                f'Goal reached: {self.target_zone} '
                f'(dist={dist_to_goal:.3f}m)'
            )
            self._stop()
            return

        # Pure pursuit steering
        dx = tx - self.current_x
        dy = ty - self.current_y
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_heading
        # Normalize to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()
        # Proportional angular control
        cmd.angular.z = max(
            -self.max_ang, min(self.max_ang, 2.0 * angle_error)
        )
        # Reduce linear speed when turning sharply
        speed_factor = max(0.0, 1.0 - abs(angle_error) / (math.pi / 2.0))
        cmd.linear.x = self.max_lin * speed_factor
        self.pub_cmd_vel.publish(cmd)

    def _stop(self):
        """Publish zero velocity."""
        cmd = Twist()
        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
