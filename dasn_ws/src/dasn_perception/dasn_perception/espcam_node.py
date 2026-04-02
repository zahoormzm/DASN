"""
ESP32-CAM Node — /camera/espcam

Pulls MJPEG stream from ESP32-CAM over HTTP and publishes sensor_msgs/Image.
"""

import threading
import time

try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    raise ImportError(f'rclpy is required: {e}. Install ROS 2 or source your workspace.')

try:
    import cv2
except ImportError as e:
    raise ImportError(f'OpenCV is required: {e}. Install with: pip install opencv-python')

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(f'cv_bridge is required: {e}. Install the ros-<distro>-cv-bridge package.')

try:
    from sensor_msgs.msg import CompressedImage, Image
except ImportError as e:
    raise ImportError(f'sensor_msgs is required: {e}.')

try:
    import numpy as np
except ImportError as e:
    raise ImportError(f'numpy is required: {e}. Install with: pip install numpy')

import urllib.request


class EspCamNode(Node):

    def __init__(self):
        super().__init__('espcam', namespace='camera/espcam')

        self.declare_parameter('espcam_ip', '192.168.1.101')
        self.declare_parameter('reconnect_delay', 3.0)

        self._espcam_ip = self.get_parameter('espcam_ip').value
        self._reconnect_delay = self.get_parameter('reconnect_delay').value

        self._image_pub = self.create_publisher(Image, 'image_raw', 10)
        self._compressed_pub = self.create_publisher(
            CompressedImage, 'image_raw/compressed', 10
        )
        self._bridge = CvBridge()
        self._running = True

        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

        self.get_logger().info(
            f'ESP32-CAM node started — stream: http://{self._espcam_ip}/stream'
        )

    def _stream_loop(self):
        """Continuously read MJPEG frames from ESP32-CAM HTTP stream."""
        stream_url = f'http://{self._espcam_ip}/stream'

        while self._running and rclpy.ok():
            try:
                self.get_logger().info(f'Connecting to MJPEG stream: {stream_url}')
                req = urllib.request.Request(stream_url)
                with urllib.request.urlopen(req, timeout=10) as response:
                    buf = b''
                    while self._running and rclpy.ok():
                        chunk = response.read(4096)
                        if not chunk:
                            break
                        buf += chunk

                        # Look for JPEG frame boundaries
                        start = buf.find(b'\xff\xd8')
                        end = buf.find(b'\xff\xd9')

                        if start != -1 and end != -1 and end > start:
                            jpg_data = buf[start:end + 2]
                            buf = buf[end + 2:]

                            # Decode JPEG to OpenCV frame
                            img_array = np.frombuffer(jpg_data, dtype=np.uint8)
                            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                            if frame is not None:
                                try:
                                    img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                                    img_msg.header.stamp = self.get_clock().now().to_msg()
                                    img_msg.header.frame_id = 'espcam'
                                    self._image_pub.publish(img_msg)
                                    compressed = CompressedImage()
                                    compressed.header = img_msg.header
                                    compressed.format = 'jpeg'
                                    compressed.data = jpg_data
                                    self._compressed_pub.publish(compressed)
                                except Exception as e:
                                    self.get_logger().error(f'Failed to publish image: {e}')

            except Exception as e:
                self.get_logger().warn(
                    f'MJPEG stream error: {e}. Retrying in {self._reconnect_delay}s...'
                )
                time.sleep(self._reconnect_delay)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EspCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
