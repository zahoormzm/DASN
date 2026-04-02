"""
Phone Camera Node — /camera/phone

Pulls RTSP video stream and HTTP audio from an IP Webcam app on a phone.
Publishes video frames as sensor_msgs/Image and audio as dasn_msgs/AudioChunk.
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
    from dasn_msgs.msg import AudioChunk
except ImportError as e:
    raise ImportError(f'dasn_msgs is required: {e}. Build the dasn_msgs package first.')

try:
    import numpy as np
except ImportError as e:
    raise ImportError(f'numpy is required: {e}. Install with: pip install numpy')

try:
    import urllib.request
except ImportError:
    pass  # stdlib, always available


class PhoneCameraNode(Node):

    def __init__(self):
        super().__init__('phone_camera', namespace='camera/phone')

        self.declare_parameter('phone_ip', '192.168.1.100')
        self.declare_parameter('rtsp_port', 8080)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('audio_sample_rate', 16000)

        self._phone_ip = self.get_parameter('phone_ip').value
        self._rtsp_port = self.get_parameter('rtsp_port').value
        self._frame_rate = self.get_parameter('frame_rate').value
        self._reconnect_delay = self.get_parameter('reconnect_delay').value
        self._audio_sample_rate = self.get_parameter('audio_sample_rate').value

        self._image_pub = self.create_publisher(Image, 'image_raw', 10)
        self._compressed_pub = self.create_publisher(
            CompressedImage, 'image_raw/compressed', 10
        )
        self._audio_pub = self.create_publisher(AudioChunk, 'audio_raw', 10)

        self._bridge = CvBridge()
        self._cap = None
        self._running = True

        # Start video capture in a background thread
        self._video_thread = threading.Thread(target=self._video_loop, daemon=True)
        self._video_thread.start()

        # Start audio capture in a background thread
        self._audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self._audio_thread.start()

        self.get_logger().info(
            f'Phone camera node started — RTSP: rtsp://{self._phone_ip}:{self._rtsp_port}/h264'
        )

    def _connect_video(self):
        """Attempt to connect to the RTSP stream. Returns True on success."""
        rtsp_url = f'rtsp://{self._phone_ip}:{self._rtsp_port}/h264'
        self.get_logger().info(f'Connecting to RTSP stream: {rtsp_url}')
        cap = cv2.VideoCapture(rtsp_url)
        if cap.isOpened():
            self._cap = cap
            self.get_logger().info('RTSP stream connected.')
            return True
        else:
            cap.release()
            self.get_logger().warn(f'Failed to open RTSP stream at {rtsp_url}')
            return False

    def _video_loop(self):
        """Continuously capture frames and publish them."""
        period = 1.0 / self._frame_rate

        while self._running and rclpy.ok():
            # Connect if needed
            if self._cap is None or not self._cap.isOpened():
                if not self._connect_video():
                    self.get_logger().warn(
                        f'Retrying RTSP in {self._reconnect_delay}s...'
                    )
                    time.sleep(self._reconnect_delay)
                    continue

            loop_start = time.monotonic()
            ret, frame = self._cap.read()

            if not ret:
                self.get_logger().warn('Lost RTSP stream. Reconnecting...')
                self._cap.release()
                self._cap = None
                time.sleep(self._reconnect_delay)
                continue

            try:
                img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'phone_camera'
                self._image_pub.publish(img_msg)
                ok, encoded = cv2.imencode('.jpg', frame)
                if ok:
                    compressed = CompressedImage()
                    compressed.header = img_msg.header
                    compressed.format = 'jpeg'
                    compressed.data = encoded.tobytes()
                    self._compressed_pub.publish(compressed)
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {e}')

            # Throttle to target frame rate
            elapsed = time.monotonic() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _audio_loop(self):
        """Pull audio chunks from IP Webcam HTTP API and publish them."""
        audio_url = f'http://{self._phone_ip}:{self._rtsp_port}/audio.wav'

        while self._running and rclpy.ok():
            try:
                self.get_logger().info(f'Connecting to audio stream: {audio_url}')
                req = urllib.request.Request(audio_url)
                with urllib.request.urlopen(req, timeout=10) as response:
                    # Skip WAV header (44 bytes)
                    header = response.read(44)
                    if len(header) < 44:
                        raise ValueError('Incomplete WAV header')

                    chunk_size = self._audio_sample_rate  # 1 second of 16-bit mono
                    while self._running and rclpy.ok():
                        raw = response.read(chunk_size * 2)  # 2 bytes per int16 sample
                        if not raw:
                            break

                        samples = np.frombuffer(raw, dtype=np.int16)
                        msg = AudioChunk()
                        msg.data = samples.tolist()
                        msg.sample_rate = self._audio_sample_rate
                        msg.channels = 1
                        msg.timestamp = self.get_clock().now().to_msg()
                        self._audio_pub.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Audio stream error: {e}. Retrying in {self._reconnect_delay}s...')
                time.sleep(self._reconnect_delay)

    def destroy_node(self):
        self._running = False
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhoneCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
