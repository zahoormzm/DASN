"""
Gas Analyzer Node — /ml/gas_analyzer

Subscribes to zone sensor topics for each configured zone, analyzes BME688
gas resistance, temperature, and humidity data using threshold-based
classification, and publishes dasn_msgs/GasEvent.
"""

try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    raise ImportError(f'rclpy is required: {e}.')

try:
    from dasn_msgs.msg import ZoneSensorData, GasEvent
except ImportError as e:
    raise ImportError(f'dasn_msgs is required: {e}. Build the dasn_msgs package first.')


# Threshold-based gas classification parameters
# Gas resistance values in Ohms — BME688 typical ranges
# Normal air: 50k-500k Ohms
# VOCs (solvents, paints): 10k-50k Ohms
# Smoke: 5k-20k Ohms
# Butane/combustible gas: 1k-10k Ohms
GAS_THRESHOLDS = {
    'butane': {
        'gas_resistance_max': 10000.0,  # Below 10kOhm
        'severity_base': 0.9,
    },
    'smoke': {
        'gas_resistance_max': 20000.0,  # Below 20kOhm
        'severity_base': 0.7,
    },
    'voc': {
        'gas_resistance_max': 50000.0,  # Below 50kOhm
        'severity_base': 0.5,
    },
}

# Temperature and humidity thresholds that may indicate fire/hazard
TEMP_FIRE_THRESHOLD = 50.0  # Celsius
HUMIDITY_CONDENSATION_THRESHOLD = 95.0  # Percent


class GasAnalyzerNode(Node):

    def __init__(self):
        super().__init__('gas_analyzer', namespace='ml')

        self.declare_parameter(
            'zone_ids',
            ['FRONT_DOOR', 'WINDOW_1', 'BACK_DOOR']
        )
        self.declare_parameter('publish_normal', False)

        self._zone_ids = self.get_parameter('zone_ids').value
        self._publish_normal = self.get_parameter('publish_normal').value

        self._gas_pub = self.create_publisher(GasEvent, 'gas_events', 10)

        # Subscribe to each zone's sensor topic
        self._subscriptions = []
        for zone_id in self._zone_ids:
            topic = f'/zone/{zone_id}/sensors'
            sub = self.create_subscription(
                ZoneSensorData, topic, self._on_sensor_data, 10
            )
            self._subscriptions.append(sub)
            self.get_logger().info(f'Subscribed to {topic}')

        self.get_logger().info(
            f'Gas analyzer node started — monitoring {len(self._zone_ids)} zone(s).'
        )

    def _classify_gas(self, gas_resistance: float, temperature: float,
                      humidity: float) -> tuple:
        """
        Classify gas based on BME688 resistance value with temperature/humidity
        context. Returns (gas_type, severity) tuple.
        """
        # Check from most dangerous to least dangerous
        for gas_type in ['butane', 'smoke', 'voc']:
            thresh = GAS_THRESHOLDS[gas_type]
            if gas_resistance < thresh['gas_resistance_max']:
                # Compute severity: lower resistance = higher severity
                severity = thresh['severity_base']

                # Boost severity if temperature is also elevated (possible fire)
                if temperature > TEMP_FIRE_THRESHOLD and gas_type in ('smoke', 'butane'):
                    temp_factor = min(1.0, (temperature - TEMP_FIRE_THRESHOLD) / 50.0)
                    severity = min(1.0, severity + temp_factor * 0.2)

                # Scale severity by how far below the threshold the reading is
                resistance_ratio = gas_resistance / thresh['gas_resistance_max']
                severity = min(1.0, severity * (1.0 - resistance_ratio * 0.5))

                return gas_type, float(severity)

        return 'normal', 0.0

    def _on_sensor_data(self, msg: ZoneSensorData):
        gas_resistance = msg.bme688_gas_resistance
        temperature = msg.bme688_temperature
        humidity = msg.bme688_humidity
        zone_id = msg.zone_id

        # Skip if sensor data looks invalid (resistance of 0 typically means sensor error)
        if gas_resistance <= 0.0:
            self.get_logger().debug(
                f'Zone {zone_id}: invalid gas resistance ({gas_resistance}), skipping.'
            )
            return

        gas_type, severity = self._classify_gas(gas_resistance, temperature, humidity)

        # Only publish non-normal events (unless configured otherwise)
        if gas_type == 'normal' and not self._publish_normal:
            return

        event_msg = GasEvent()
        event_msg.gas_type = gas_type
        event_msg.severity = severity
        event_msg.temperature = temperature
        event_msg.humidity = humidity
        event_msg.zone_id = zone_id

        self._gas_pub.publish(event_msg)

        if gas_type != 'normal':
            self.get_logger().warn(
                f'Gas alert in zone {zone_id}: {gas_type} '
                f'(severity={severity:.2f}, resistance={gas_resistance:.0f} Ohm, '
                f'temp={temperature:.1f}C, humidity={humidity:.1f}%)'
            )

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GasAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
