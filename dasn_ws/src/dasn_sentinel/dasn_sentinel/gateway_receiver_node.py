"""Gateway Receiver Node — reads USB serial from gateway ESP32 and publishes ZoneSensorData."""

import json
import threading

import rclpy
from rclpy.node import Node

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from dasn_msgs.msg import ZoneSensorData
from builtin_interfaces.msg import Time


class GatewayReceiverNode(Node):
    """Reads JSON lines from gateway ESP32 over USB serial and publishes per-zone sensor data."""

    def __init__(self):
        super().__init__('gateway_receiver', namespace='sentinel')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self._serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self._baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Dynamic publishers: zone_id -> publisher
        self._zone_publishers: dict[str, rclpy.publisher.Publisher] = {}

        # Serial connection state
        self._ser = None
        self._running = True

        if not SERIAL_AVAILABLE:
            self.get_logger().error(
                'pyserial is not installed. Install with: pip install pyserial'
            )
            return

        # Start the serial reading thread
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()
        self.get_logger().info(
            f'Gateway receiver started — port={self._serial_port}, baud={self._baud_rate}'
        )

    def _connect_serial(self):
        """Attempt to open the serial port. Returns True on success."""
        try:
            self._ser = serial.Serial(
                port=self._serial_port,
                baudrate=self._baud_rate,
                timeout=1.0,
            )
            self.get_logger().info(f'Serial connected on {self._serial_port}')
            return True
        except serial.SerialException as exc:
            self.get_logger().warn(f'Serial connection failed: {exc}')
            self._ser = None
            return False

    def _serial_loop(self):
        """Background thread: read lines from serial, parse, and publish."""
        import time

        while self._running and rclpy.ok():
            # Ensure we have a connection
            if self._ser is None or not self._ser.is_open:
                if not self._connect_serial():
                    time.sleep(2.0)  # wait before retry
                    continue

            try:
                raw_line = self._ser.readline()
                if not raw_line:
                    continue  # timeout, no data

                line = raw_line.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                self._process_line(line)

            except serial.SerialException as exc:
                self.get_logger().warn(f'Serial read error: {exc} — reconnecting...')
                self._close_serial()
                time.sleep(2.0)
            except Exception as exc:
                self.get_logger().error(f'Unexpected error in serial loop: {exc}')
                time.sleep(0.1)

    def _process_line(self, line: str):
        """Parse a JSON line and publish ZoneSensorData."""
        try:
            data = json.loads(line)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON: {exc} — line: {line!r}')
            return

        zone_id = data.get('zone_id')
        if not zone_id:
            self.get_logger().warn(f'Missing zone_id in JSON: {line!r}')
            return

        # Get or create publisher for this zone
        pub = self._get_zone_publisher(zone_id)

        # Build message
        msg = ZoneSensorData()
        msg.zone_id = str(zone_id)

        now = self.get_clock().now().to_msg()
        msg.timestamp = now

        msg.radar_presence = bool(data.get('radar', False))
        msg.radar_confidence = float(data.get('radar_conf', 0.0))
        msg.pir_triggered = bool(data.get('pir', False))
        msg.sound_level_db = float(data.get('sound_db', 0.0))
        msg.sound_transient = bool(data.get('sound_trans', False))
        msg.bme688_gas_resistance = float(data.get('gas_r', 0.0))
        msg.bme688_temperature = float(data.get('temp', 0.0))
        msg.bme688_humidity = float(data.get('hum', 0.0))
        msg.bme688_pressure = float(data.get('pres', 0.0))
        msg.battery_v = float(data.get('bat', 0.0))
        msg.local_threat_score = float(data.get('threat', 0.0))
        msg.comm_mode = 'espnow'

        pub.publish(msg)
        self.get_logger().debug(
            f'Published ZoneSensorData for zone={zone_id} '
            f'radar={msg.radar_presence} threat={msg.local_threat_score:.2f}'
        )

    def _get_zone_publisher(self, zone_id: str):
        """Return existing publisher for zone_id, or create one."""
        if zone_id not in self._zone_publishers:
            topic = f'/zone/{zone_id}/sensors'
            pub = self.create_publisher(ZoneSensorData, topic, 10)
            self._zone_publishers[zone_id] = pub
            self.get_logger().info(f'Created publisher for topic: {topic}')
        return self._zone_publishers[zone_id]

    def _close_serial(self):
        """Safely close the serial port."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def destroy_node(self):
        self._running = False
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GatewayReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
