"""Auth Manager Node — validates NFC tags and touch patterns against authorized config."""

import json
from collections import defaultdict

import rclpy
from rclpy.node import Node

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

from dasn_msgs.msg import AuthEvent, ZoneSensorData
from std_msgs.msg import String


class AuthManagerNode(Node):
    """Subscribes to zone sensor topics and NFC scan events, validates credentials,
    publishes auth events, and tracks failed attempts."""

    def __init__(self):
        super().__init__('auth_manager', namespace='sentinel')

        # Declare parameters
        self.declare_parameter('zone_ids', ['FRONT_DOOR', 'WINDOW_1', 'BACK_DOOR'])
        self.declare_parameter('auth_config_path', 'config/authorized_tags.yaml')
        self.declare_parameter('max_failures', 3)

        zone_ids = (
            self.get_parameter('zone_ids')
            .get_parameter_value()
            .string_array_value
        )
        self._auth_config_path = (
            self.get_parameter('auth_config_path')
            .get_parameter_value()
            .string_value
        )
        self._max_failures = (
            self.get_parameter('max_failures')
            .get_parameter_value()
            .integer_value
        )

        # Auth event publisher
        self._auth_pub = self.create_publisher(AuthEvent, '/security/auth_event', 10)

        # Failed attempt counters per zone
        self._failure_counts: dict[str, int] = defaultdict(int)

        # Load authorized tags config
        self._authorized_tags: dict = {}
        self._load_auth_config()

        # Subscribe to each zone sensor topic
        self._zone_subs = []
        for zone_id in zone_ids:
            topic = f'/zone/{zone_id}/sensors'
            sub = self.create_subscription(
                ZoneSensorData,
                topic,
                lambda msg, zid=zone_id: self._zone_sensor_callback(msg, zid),
                10,
            )
            self._zone_subs.append(sub)
            self.get_logger().info(f'Subscribed to zone sensor topic: {topic}')

        # Subscribe to NFC scan topic
        self._nfc_sub = self.create_subscription(
            String,
            '/sentinel/nfc_scan',
            self._nfc_scan_callback,
            10,
        )
        self.get_logger().info('Subscribed to /sentinel/nfc_scan')

        self.get_logger().info(
            f'Auth manager started — zones={zone_ids}, max_failures={self._max_failures}'
        )

    def _load_auth_config(self):
        """Load authorized tags from YAML configuration file."""
        if not YAML_AVAILABLE:
            self.get_logger().error(
                'PyYAML is not installed. Install with: pip install pyyaml'
            )
            return

        try:
            with open(self._auth_config_path, 'r') as f:
                self._authorized_tags = yaml.safe_load(f) or {}
            tag_count = len(self._get_tag_entries())
            self.get_logger().info(
                f'Loaded auth config from {self._auth_config_path} '
                f'({tag_count} tags)'
            )
        except FileNotFoundError:
            self.get_logger().warn(
                f'Auth config not found at {self._auth_config_path} — '
                'no tags will be authorized'
            )
            self._authorized_tags = {}
        except Exception as exc:
            self.get_logger().error(f'Failed to load auth config: {exc}')
            self._authorized_tags = {}

    def _zone_sensor_callback(self, msg: ZoneSensorData, zone_id: str):
        """Handle incoming zone sensor data. Logs at debug level for monitoring."""
        self.get_logger().debug(
            f'Zone {zone_id} sensor update — '
            f'radar={msg.radar_presence} pir={msg.pir_triggered} '
            f'threat={msg.local_threat_score:.2f}'
        )

    def _nfc_scan_callback(self, msg: String):
        """Handle NFC scan event from /sentinel/nfc_scan topic."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Invalid JSON on /sentinel/nfc_scan: {exc}')
            return

        zone_id = data.get('zone_id', '')
        tag_id = data.get('tag_id', '')
        method = data.get('method', 'nfc')
        pattern = data.get('pattern', '')

        if not zone_id or (not tag_id and not pattern):
            self.get_logger().warn(
                f'NFC scan missing zone_id and credential data: {data}'
            )
            return

        authorized = self._validate_credential(
            zone_id=zone_id,
            method=method,
            tag_id=tag_id,
            pattern=pattern,
        )

        # Publish auth event
        auth_msg = AuthEvent()
        auth_msg.zone_id = str(zone_id)
        auth_msg.method = str(method)
        auth_msg.authorized = authorized
        auth_msg.tag_id = str(tag_id)
        auth_msg.timestamp = self.get_clock().now().to_msg()
        self._auth_pub.publish(auth_msg)

        if authorized:
            # Reset failure counter on success
            self._failure_counts[zone_id] = 0
            self.get_logger().info(
                f'AUTH SUCCESS — zone={zone_id} method={method} tag={tag_id}'
            )
        else:
            self._failure_counts[zone_id] += 1
            count = self._failure_counts[zone_id]
            self.get_logger().info(
                f'AUTH FAILURE — zone={zone_id} method={method} tag={tag_id} '
                f'(failures: {count}/{self._max_failures})'
            )
            if count >= self._max_failures:
                self.get_logger().warning(
                    f'ALERT: Max failed auth attempts reached for zone={zone_id} '
                    f'({count} failures) — possible intrusion attempt'
                )

    def _get_tag_entries(self):
        tags = self._authorized_tags.get('tags')
        if tags is None:
            tags = self._authorized_tags.get('authorized_tags', [])
        return tags if isinstance(tags, list) else []

    def _validate_credential(
        self,
        zone_id: str,
        method: str,
        tag_id: str = '',
        pattern: str = '',
    ) -> bool:
        """Validate NFC tags or touch patterns against the configured auth data."""
        if method == 'touch':
            touch_patterns = self._authorized_tags.get('touch_patterns', {})
            if not isinstance(touch_patterns, dict):
                return False
            return pattern in touch_patterns.values()

        tags = self._get_tag_entries()
        if not isinstance(tags, list):
            return False

        for entry in tags:
            if not isinstance(entry, dict):
                continue
            entry_id = entry.get('id', entry.get('tag_id'))
            if entry_id == tag_id:
                allowed_methods = entry.get('methods', ['nfc'])
                allowed_zones = entry.get('zones', [])
                if allowed_zones and zone_id not in allowed_zones:
                    self.get_logger().debug(
                        f'Tag {tag_id} is not authorized for zone {zone_id}'
                    )
                    return False
                if method in allowed_methods:
                    return True
                self.get_logger().debug(
                    f'Tag {tag_id} found but method {method} not in '
                    f'allowed methods {allowed_methods}'
                )
                return False

        return False

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuthManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
