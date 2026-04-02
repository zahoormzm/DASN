import time
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from dasn_msgs.msg import AlarmEvent


class PhoneSpeakerNode(Node):
    """Triggers audio/visual alerts on a phone running IP Webcam."""

    # IP Webcam endpoints
    _SOUND_URL = 'http://{ip}:8080/audio.wav'
    _TORCH_ON_URL = 'http://{ip}:8080/enabletorch'
    _TORCH_OFF_URL = 'http://{ip}:8080/disabletorch'

    def __init__(self):
        super().__init__('phone_speaker', namespace='/alerts')

        self.declare_parameter('phone_ip', '192.168.1.100')
        self._phone_ip = (
            self.get_parameter('phone_ip')
            .get_parameter_value()
            .string_value
        )

        self._sub = self.create_subscription(
            AlarmEvent,
            '/alerts/alarm',
            self._alarm_cb,
            10,
        )

        # Timer handle for strobe effect (level 5)
        self._strobe_timer = None
        self._strobe_state = False
        self._strobe_cycles_remaining = 0

        self.get_logger().info(
            f'Phone speaker node ready (phone_ip={self._phone_ip})'
        )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def _alarm_cb(self, msg: AlarmEvent):
        self._phone_ip = (
            self.get_parameter('phone_ip')
            .get_parameter_value()
            .string_value
        )

        level = msg.level
        zone = msg.zone_id or 'unknown'
        self.get_logger().info(
            f'Alarm received: zone={zone} level={level} reason={msg.reason}'
        )

        if level <= 2:
            # Below threshold for phone alerts
            self.get_logger().info(
                f'Level {level} below phone alert threshold; ignoring'
            )
            return

        if level == 3:
            self._single_beep()
        elif level == 4:
            self._alarm_siren_loop()
        elif level >= 5:
            self._alarm_siren_loop()
            self._start_strobe()

    # ------------------------------------------------------------------
    # Alert actions
    # ------------------------------------------------------------------
    def _single_beep(self):
        """Play a single beep via the phone speaker."""
        self.get_logger().info('Triggering single beep on phone')
        self._http_get(self._SOUND_URL.format(ip=self._phone_ip))

    def _alarm_siren_loop(self):
        """Trigger alarm siren loop via repeated audio plays."""
        self.get_logger().info('Triggering alarm siren loop on phone')
        # IP Webcam plays the audio once per request; we fire multiple
        # requests to produce a sustained siren effect.
        for i in range(3):
            self._http_get(self._SOUND_URL.format(ip=self._phone_ip))

    def _start_strobe(self):
        """Start a flashlight strobe effect (10 cycles, 0.3s interval)."""
        self.get_logger().info('Starting flashlight strobe on phone')
        self._strobe_cycles_remaining = 10
        self._strobe_state = False
        # Cancel any previous strobe timer
        if self._strobe_timer is not None:
            self._strobe_timer.cancel()
            self.destroy_timer(self._strobe_timer)
        self._strobe_timer = self.create_timer(0.3, self._strobe_tick)

    def _strobe_tick(self):
        """Toggle the flashlight on/off for strobe effect."""
        if self._strobe_cycles_remaining <= 0:
            # Done strobing; make sure torch is off
            self._http_get(self._TORCH_OFF_URL.format(ip=self._phone_ip))
            if self._strobe_timer is not None:
                self._strobe_timer.cancel()
                self.destroy_timer(self._strobe_timer)
                self._strobe_timer = None
            self.get_logger().info('Flashlight strobe finished')
            return

        self._strobe_state = not self._strobe_state
        if self._strobe_state:
            self._http_get(self._TORCH_ON_URL.format(ip=self._phone_ip))
        else:
            self._http_get(self._TORCH_OFF_URL.format(ip=self._phone_ip))

        self._strobe_cycles_remaining -= 1

    # ------------------------------------------------------------------
    # HTTP helper
    # ------------------------------------------------------------------
    def _http_get(self, url: str) -> bool:
        """Fire an HTTP GET request. Returns True on success."""
        try:
            req = urllib.request.Request(url, method='GET')
            with urllib.request.urlopen(req, timeout=3) as resp:
                self.get_logger().info(
                    f'Phone HTTP OK: {url} (status={resp.status})'
                )
                return True
        except urllib.error.URLError as e:
            self.get_logger().warn(
                f'Phone offline or unreachable ({url}): {e.reason}'
            )
            return False
        except urllib.error.HTTPError as e:
            self.get_logger().error(
                f'Phone HTTP error {e.code} for {url}'
            )
            return False
        except Exception as e:
            self.get_logger().error(
                f'Phone HTTP request failed ({url}): {e}'
            )
            return False


def main(args=None):
    rclpy.init(args=args)
    node = PhoneSpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
