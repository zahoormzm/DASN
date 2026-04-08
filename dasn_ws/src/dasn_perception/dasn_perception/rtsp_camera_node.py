"""Generic RTSP camera node for the Raspberry Pi 5 camera feed."""

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class RtspCameraNode(Node):
    def __init__(self):
        super().__init__('rtsp_camera', namespace='camera/rp5')

        self.declare_parameter('rtsp_url', 'rtsp://192.168.1.100:8554/stream')
        self.declare_parameter('frame_rate', 20.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('frame_id', 'rp5_camera')

        self._rtsp_url = self.get_parameter('rtsp_url').value
        self._frame_rate = float(self.get_parameter('frame_rate').value)
        self._reconnect_delay = float(self.get_parameter('reconnect_delay').value)
        self._frame_id = self.get_parameter('frame_id').value

        self._image_pub = self.create_publisher(Image, 'image_raw', 10)
        self._compressed_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self._bridge = CvBridge()
        self._cap = None
        self._running = True

        self._thread = threading.Thread(target=self._video_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'RTSP camera node started — {self._rtsp_url}')

    def _connect(self):
        self.get_logger().info(f'Connecting to RTSP stream: {self._rtsp_url}')
        cap = cv2.VideoCapture(self._rtsp_url)
        if cap.isOpened():
            self._cap = cap
            self.get_logger().info('RTSP stream connected.')
            return True
        cap.release()
        self.get_logger().warn(f'Failed to open RTSP stream at {self._rtsp_url}')
        return False

    def _video_loop(self):
        period = 1.0 / self._frame_rate if self._frame_rate > 0 else 0.05
        while self._running and rclpy.ok():
            if self._cap is None or not self._cap.isOpened():
                if not self._connect():
                    time.sleep(self._reconnect_delay)
                    continue

            loop_start = time.monotonic()
            ok, frame = self._cap.read()
            if not ok:
                self.get_logger().warn('Lost RTSP stream. Reconnecting...')
                self._cap.release()
                self._cap = None
                time.sleep(self._reconnect_delay)
                continue

            try:
                img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = self._frame_id
                self._image_pub.publish(img_msg)

                enc_ok, encoded = cv2.imencode('.jpg', frame)
                if enc_ok:
                    compressed = CompressedImage()
                    compressed.header = img_msg.header
                    compressed.format = 'jpeg'
                    compressed.data = encoded.tobytes()
                    self._compressed_pub.publish(compressed)
            except Exception as exc:
                self.get_logger().error(f'Failed to publish RTSP frame: {exc}')

            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RtspCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
