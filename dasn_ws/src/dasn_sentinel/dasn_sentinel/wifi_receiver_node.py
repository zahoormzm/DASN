"""Wi-Fi receiver node — listens for sentinel JSON over UDP and publishes ZoneSensorData."""

import json
import socket
import threading

import rclpy
from rclpy.node import Node

from dasn_msgs.msg import ZoneSensorData
from std_msgs.msg import String


class WifiReceiverNode(Node):
    """Reads JSON datagrams from sentinel nodes over Wi-Fi and republishes them as ROS topics."""

    def __init__(self):
        super().__init__('wifi_receiver', namespace='sentinel')

        self.declare_parameter('bind_host', '0.0.0.0')
        self.declare_parameter('udp_port', 37020)
        self.declare_parameter('buffer_size', 2048)
        self.declare_parameter('default_comm_mode', 'wifi')

        self._bind_host = self.get_parameter('bind_host').get_parameter_value().string_value
        self._udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self._buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self._default_comm_mode = (
            self.get_parameter('default_comm_mode').get_parameter_value().string_value
        )

        self._zone_publishers: dict[str, rclpy.publisher.Publisher] = {}
        self._zone_meta_publishers: dict[str, rclpy.publisher.Publisher] = {}
        self._running = True
        self._sock: socket.socket | None = None

        self._listen_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._listen_thread.start()
        self.get_logger().info(
            f'Wi-Fi receiver started — bind={self._bind_host}:{self._udp_port}'
        )

    def _open_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._bind_host, self._udp_port))
        sock.settimeout(1.0)
        self._sock = sock
        self.get_logger().info(
            f'UDP socket listening on {self._bind_host}:{self._udp_port}'
        )

    def _udp_loop(self):
        while self._running and rclpy.ok():
            try:
                if self._sock is None:
                    self._open_socket()

                raw_payload, source = self._sock.recvfrom(self._buffer_size)
                if not raw_payload:
                    continue

                payload = raw_payload.decode('utf-8', errors='replace').strip()
                if not payload:
                    continue

                for line in payload.splitlines():
                    line = line.strip()
                    if line:
                        self._process_line(line, source)

            except socket.timeout:
                continue
            except OSError as exc:
                self.get_logger().warn(f'UDP socket error: {exc}')
                self._close_socket()
            except Exception as exc:
                self.get_logger().error(f'Unexpected Wi-Fi receiver error: {exc}')

    def _process_line(self, line: str, source):
        try:
            data = json.loads(line)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON from {source[0]}:{source[1]}: {exc}')
            return

        zone_id = str(data.get('zone_id', '')).strip()
        if not zone_id:
            self.get_logger().warn(
                f'Missing zone_id in Wi-Fi payload from {source[0]}:{source[1]}'
            )
            return

        pub = self._get_zone_publisher(zone_id)
        meta_pub = self._get_zone_meta_publisher(zone_id)

        msg = ZoneSensorData()
        msg.zone_id = zone_id
        msg.timestamp = self.get_clock().now().to_msg()
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
        msg.comm_mode = str(data.get('comm_mode') or self._default_comm_mode)

        pub.publish(msg)

        meta_msg = String()
        meta_msg.data = json.dumps({
            'zone_id': zone_id,
            'source_ip': source[0],
            'source_port': source[1],
            'payload': data,
        }, separators=(',', ':'))
        meta_pub.publish(meta_msg)

    def _get_zone_publisher(self, zone_id: str):
        if zone_id not in self._zone_publishers:
            topic = f'/zone/{zone_id}/sensors'
            pub = self.create_publisher(ZoneSensorData, topic, 10)
            self._zone_publishers[zone_id] = pub
            self.get_logger().info(f'Created publisher for topic: {topic}')
        return self._zone_publishers[zone_id]

    def _get_zone_meta_publisher(self, zone_id: str):
        if zone_id not in self._zone_meta_publishers:
            topic = f'/zone/{zone_id}/sensor_meta'
            pub = self.create_publisher(String, topic, 10)
            self._zone_meta_publishers[zone_id] = pub
            self.get_logger().info(f'Created meta publisher for topic: {topic}')
        return self._zone_meta_publishers[zone_id]

    def _close_socket(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def destroy_node(self):
        self._running = False
        self._close_socket()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WifiReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
