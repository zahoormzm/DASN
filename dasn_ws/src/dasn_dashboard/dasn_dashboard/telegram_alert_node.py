import io
import json
import time
import urllib.request
import urllib.error
from datetime import datetime

import rclpy
from rclpy.node import Node
from dasn_msgs.msg import AlarmEvent

try:
    import cv2
    import numpy as np
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


class TelegramAlertNode(Node):
    """Sends Telegram notifications when alarm events are received."""

    def __init__(self):
        super().__init__('telegram_alert', namespace='/alerts')

        # Parameters
        self.declare_parameter('bot_token', '')
        self.declare_parameter('chat_id', '')

        self._bot_token = self.get_parameter('bot_token').get_parameter_value().string_value
        self._chat_id = self.get_parameter('chat_id').get_parameter_value().string_value

        # Rate limiting: track last send time per zone
        self._last_send: dict[str, float] = {}
        self._rate_limit_secs = 5.0

        # Subscriber
        self._sub = self.create_subscription(
            AlarmEvent,
            '/alerts/alarm',
            self._alarm_cb,
            10,
        )

        if not self._bot_token or not self._chat_id:
            self.get_logger().warn('Telegram not configured')
        else:
            self.get_logger().info(
                f'Telegram alert node ready (chat_id={self._chat_id})'
            )

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------
    def _alarm_cb(self, msg: AlarmEvent):
        # Re-read parameters in case they were updated at runtime
        self._bot_token = self.get_parameter('bot_token').get_parameter_value().string_value
        self._chat_id = self.get_parameter('chat_id').get_parameter_value().string_value

        if not self._bot_token or not self._chat_id:
            self.get_logger().warn('Telegram not configured')
            return

        # Rate limiting per zone
        now = time.monotonic()
        zone = msg.zone_id or 'unknown'
        last = self._last_send.get(zone, 0.0)
        if now - last < self._rate_limit_secs:
            self.get_logger().info(
                f'Rate-limited: skipping alarm for zone "{zone}" '
                f'({now - last:.1f}s since last send)'
            )
            return

        # Format timestamp from builtin_interfaces/Time
        try:
            dt = datetime.utcfromtimestamp(
                msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
            )
            ts_str = dt.strftime('%Y-%m-%d %H:%M:%S UTC')
        except Exception:
            ts_str = 'N/A'

        level_labels = {
            1: 'LOW',
            2: 'MODERATE',
            3: 'HIGH',
            4: 'CRITICAL',
            5: 'EMERGENCY',
        }
        level_str = level_labels.get(msg.level, f'LEVEL-{msg.level}')

        text = (
            f'DASN ALARM [{level_str}]\n'
            f'Zone: {zone}\n'
            f'Reason: {msg.reason}\n'
            f'Time: {ts_str}'
        )

        # Check if evidence_frame is populated
        has_image = (
            msg.evidence_frame.height > 0
            and msg.evidence_frame.width > 0
            and len(msg.evidence_frame.data) > 0
        )

        if has_image:
            jpeg_bytes = self._image_to_jpeg(msg.evidence_frame)
            if jpeg_bytes is not None:
                ok = self._send_photo(text, jpeg_bytes)
            else:
                ok = self._send_text(text)
        else:
            ok = self._send_text(text)

        if ok:
            self._last_send[zone] = now

    # ------------------------------------------------------------------
    # Image conversion
    # ------------------------------------------------------------------
    def _image_to_jpeg(self, img_msg) -> bytes | None:
        """Convert sensor_msgs/Image to JPEG bytes."""
        if not _HAS_CV2:
            self.get_logger().warn(
                'cv2 not available; cannot convert evidence frame to JPEG'
            )
            return None

        try:
            height = img_msg.height
            width = img_msg.width
            encoding = img_msg.encoding.lower()
            data = bytes(img_msg.data)

            if encoding in ('rgb8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape(
                    (height, width, 3)
                )
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif encoding in ('bgr8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape(
                    (height, width, 3)
                )
            elif encoding in ('mono8', '8uc1'):
                arr = np.frombuffer(data, dtype=np.uint8).reshape(
                    (height, width)
                )
            elif encoding in ('rgba8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape(
                    (height, width, 4)
                )
                arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
            elif encoding in ('bgra8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape(
                    (height, width, 4)
                )
                arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            else:
                self.get_logger().warn(
                    f'Unsupported image encoding "{encoding}"; skipping photo'
                )
                return None

            success, jpeg_buf = cv2.imencode('.jpg', arr)
            if not success:
                self.get_logger().error('Failed to encode image to JPEG')
                return None
            return jpeg_buf.tobytes()

        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return None

    # ------------------------------------------------------------------
    # Telegram API helpers
    # ------------------------------------------------------------------
    def _send_text(self, text: str) -> bool:
        """Send a text message via Telegram Bot API."""
        url = (
            f'https://api.telegram.org/bot{self._bot_token}/sendMessage'
        )
        payload = json.dumps({
            'chat_id': self._chat_id,
            'text': text,
            'parse_mode': 'HTML',
        }).encode('utf-8')

        req = urllib.request.Request(
            url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )

        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                body = resp.read().decode('utf-8')
                self.get_logger().info(
                    f'Telegram message sent (status={resp.status})'
                )
                return True
        except urllib.error.HTTPError as e:
            self.get_logger().error(
                f'Telegram HTTP error {e.code}: {e.read().decode("utf-8", errors="replace")}'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Telegram send failed: {e}')
            return False

    def _send_photo(self, caption: str, jpeg_bytes: bytes) -> bool:
        """Send a photo with caption via Telegram Bot API (multipart form)."""
        url = (
            f'https://api.telegram.org/bot{self._bot_token}/sendPhoto'
        )

        boundary = '----DASNBoundary'
        body = io.BytesIO()

        # chat_id field
        body.write(f'--{boundary}\r\n'.encode())
        body.write(
            b'Content-Disposition: form-data; name="chat_id"\r\n\r\n'
        )
        body.write(f'{self._chat_id}\r\n'.encode())

        # caption field
        body.write(f'--{boundary}\r\n'.encode())
        body.write(
            b'Content-Disposition: form-data; name="caption"\r\n\r\n'
        )
        body.write(f'{caption}\r\n'.encode())

        # photo file field
        body.write(f'--{boundary}\r\n'.encode())
        body.write(
            b'Content-Disposition: form-data; name="photo"; '
            b'filename="evidence.jpg"\r\n'
        )
        body.write(b'Content-Type: image/jpeg\r\n\r\n')
        body.write(jpeg_bytes)
        body.write(b'\r\n')

        # closing boundary
        body.write(f'--{boundary}--\r\n'.encode())

        payload = body.getvalue()

        req = urllib.request.Request(
            url,
            data=payload,
            headers={
                'Content-Type': f'multipart/form-data; boundary={boundary}',
            },
            method='POST',
        )

        try:
            with urllib.request.urlopen(req, timeout=30) as resp:
                self.get_logger().info(
                    f'Telegram photo sent (status={resp.status})'
                )
                return True
        except urllib.error.HTTPError as e:
            self.get_logger().error(
                f'Telegram photo HTTP error {e.code}: '
                f'{e.read().decode("utf-8", errors="replace")}'
            )
            return False
        except Exception as e:
            self.get_logger().error(f'Telegram photo send failed: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = TelegramAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
