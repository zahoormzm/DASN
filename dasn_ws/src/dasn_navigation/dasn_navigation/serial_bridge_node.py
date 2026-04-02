import json
import threading
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from dasn_msgs.msg import (
    DepthGrid,
    EncoderTicks,
    StallEvent,
)

try:
    import serial
except ImportError:
    serial = None


class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge', namespace='bot')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        # Publishers
        self.pub_encoder = self.create_publisher(EncoderTicks, '/odom/encoders', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_depth = self.create_publisher(DepthGrid, '/tof/depth_grid', 10)
        self.pub_battery = self.create_publisher(Float32, '/battery/voltage', 10)
        self.pub_stall = self.create_publisher(StallEvent, '/bot/stall_event', 10)
        self.pub_rear_sonar = self.create_publisher(Float32, '/sonar/rear_mm', 10)

        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.sub_cmd_vel_safe = self.create_subscription(
            Twist, '/cmd_vel_safe', self.cmd_vel_safe_callback, 10
        )

        # Differential drive constants
        self.track_width = 0.16       # meters
        self.wheel_radius = 0.017     # 34mm diameter / 2
        self.max_speed = 0.535        # m/s at 300RPM

        # Emergency stop tracking
        self.last_twist_nonzero = False
        self.latest_raw_cmd = Twist()
        self.latest_safe_cmd = Twist()
        self.last_raw_received = 0.0
        self.last_safe_received = 0.0

        # Serial connection
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True

        self.connect_serial()

        # Reader thread
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()
        self.cmd_timer = self.create_timer(0.05, self._command_tick)

        self.get_logger().info(
            f'Serial bridge started on {self.serial_port} @ {self.baud_rate}'
        )

    def connect_serial(self):
        """Attempt to open the serial port. Returns True on success."""
        if serial is None:
            self.get_logger().error('pyserial is not installed')
            return False
        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=1.0,
            )
            self.get_logger().info(f'Connected to {self.serial_port}')
            return True
        except Exception as e:
            self.get_logger().warn(f'Serial connect failed: {e}')
            self.ser = None
            return False

    def serial_read_loop(self):
        """Background thread: read JSON lines from ESP32 and publish."""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(2.0)
                self.get_logger().info('Attempting serial reconnect...')
                self.connect_serial()
                continue

            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                data = json.loads(line)
                self.dispatch_message(data)
            except json.JSONDecodeError:
                self.get_logger().debug(f'Non-JSON line: {line}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                with self.ser_lock:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
            except Exception as e:
                self.get_logger().error(f'Read loop error: {e}')

    def dispatch_message(self, data: dict):
        """Route a parsed JSON dict to the appropriate publisher."""
        msg_type = data.get('type', '')
        now = self.get_clock().now().to_msg()

        if msg_type == 'enc':
            msg = EncoderTicks()
            msg.left_ticks = int(data.get('l', 0))
            msg.right_ticks = int(data.get('r', 0))
            msg.timestamp = now
            self.pub_encoder.publish(msg)

        elif msg_type == 'imu':
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = 'imu_link'
            # ISM330DHCX library returns milli-g and milli-dps
            # Convert: milli-g -> m/s² (×9.81/1000), milli-dps -> rad/s (×π/180/1000)
            MG_TO_MS2 = 9.81 / 1000.0
            MDPS_TO_RADS = 3.14159265 / 180.0 / 1000.0
            msg.linear_acceleration.x = float(data.get('ax', 0.0)) * MG_TO_MS2
            msg.linear_acceleration.y = float(data.get('ay', 0.0)) * MG_TO_MS2
            msg.linear_acceleration.z = float(data.get('az', 0.0)) * MG_TO_MS2
            msg.angular_velocity.x = float(data.get('gx', 0.0)) * MDPS_TO_RADS
            msg.angular_velocity.y = float(data.get('gy', 0.0)) * MDPS_TO_RADS
            msg.angular_velocity.z = float(data.get('gz', 0.0)) * MDPS_TO_RADS
            # Magnetometer data is not part of sensor_msgs/Imu; log for reference
            self.pub_imu.publish(msg)

        elif msg_type == 'tof':
            msg = DepthGrid()
            distances = data.get('d', [])
            msg.distances_mm = [float(d) for d in distances]
            # Pad or truncate to 64 values
            while len(msg.distances_mm) < 64:
                msg.distances_mm.append(0.0)
            msg.distances_mm = msg.distances_mm[:64]
            msg.rows = 8
            msg.cols = 8
            msg.fov_deg = 63.0
            msg.timestamp = now
            self.pub_depth.publish(msg)

        elif msg_type == 'bat':
            msg = Float32()
            msg.data = float(data.get('v', 0.0))
            self.pub_battery.publish(msg)

        elif msg_type == 'us':
            dist = data.get('d', 0)
            msg = Float32()
            msg.data = float(dist)
            self.pub_rear_sonar.publish(msg)

        elif msg_type == 'stall':
            msg = StallEvent()
            msg.motor_channel = int(data.get('ch', 0))
            msg.timestamp = now
            self.pub_stall.publish(msg)

        else:
            self.get_logger().debug(f'Unknown message type: {msg_type}')

    def cmd_vel_callback(self, msg: Twist):
        """Store raw navigation commands for the safety arbiter."""
        self.latest_raw_cmd = deepcopy(msg)
        self.last_raw_received = time.monotonic()

    def cmd_vel_safe_callback(self, msg: Twist):
        """Store safety-filtered commands so they take precedence over raw cmd_vel."""
        self.latest_safe_cmd = deepcopy(msg)
        self.last_safe_received = time.monotonic()

    def _command_tick(self):
        """Send the most recent safe command, falling back to raw cmd_vel if needed."""
        now = time.monotonic()
        cmd = None

        if now - self.last_safe_received <= 0.25:
            cmd = self.latest_safe_cmd
        elif now - self.last_raw_received <= 0.25:
            cmd = self.latest_raw_cmd
        else:
            cmd = Twist()

        self._send_twist(cmd)

    def _send_twist(self, msg: Twist):
        """Convert Twist to motor PWM command and send over serial."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        is_zero = abs(linear_x) < 1e-6 and abs(angular_z) < 1e-6

        if is_zero and self.last_twist_nonzero:
            self.serial_write('S\n')
            self.last_twist_nonzero = False
            return

        self.last_twist_nonzero = not is_zero

        if is_zero:
            return

        # Differential drive kinematics
        v_left = linear_x - (angular_z * self.track_width / 2.0)
        v_right = linear_x + (angular_z * self.track_width / 2.0)

        m1_pwm = min(int(abs(v_left) / self.max_speed * 255), 255)
        m2_pwm = min(int(abs(v_right) / self.max_speed * 255), 255)
        m1_dir = 1 if v_left >= 0 else 0
        m2_dir = 1 if v_right >= 0 else 0

        cmd = f'M,{m1_pwm},{m1_dir},{m2_pwm},{m2_dir}\n'
        self.serial_write(cmd)

    def serial_write(self, data: str):
        """Thread-safe write to serial port."""
        with self.ser_lock:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.write(data.encode('utf-8'))
                except Exception as e:
                    self.get_logger().error(f'Serial write error: {e}')
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None

    def send_buzzer(self, freq: int, duration: int):
        """Send a buzzer command to the ESP32."""
        cmd = f'B,{freq},{duration}\n'
        self.serial_write(cmd)

    def destroy_node(self):
        self.running = False
        with self.ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
