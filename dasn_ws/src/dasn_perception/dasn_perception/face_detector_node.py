"""
Face Detector Node — /ml/face_detector

Subscribes to camera image topics, runs MediaPipe Face Detection,
and publishes dasn_msgs/FaceDetection with bounding box centers and confidences.
"""

try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    raise ImportError(f'rclpy is required: {e}.')

try:
    import cv2
except ImportError as e:
    raise ImportError(f'OpenCV is required: {e}. Install with: pip install opencv-python')

try:
    from cv_bridge import CvBridge
except ImportError as e:
    raise ImportError(f'cv_bridge is required: {e}.')

try:
    import mediapipe as mp
except ImportError as e:
    raise ImportError(f'mediapipe is required: {e}. Install with: pip install mediapipe')

try:
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point
except ImportError as e:
    raise ImportError(f'sensor_msgs/geometry_msgs required: {e}.')

try:
    from dasn_msgs.msg import FaceDetection
except ImportError as e:
    raise ImportError(f'dasn_msgs is required: {e}. Build the dasn_msgs package first.')

import numpy as np


class FaceDetectorNode(Node):

    def __init__(self):
        super().__init__('face_detector', namespace='ml')

        self.declare_parameter('min_detection_confidence', 0.5)

        min_conf = self.get_parameter('min_detection_confidence').value

        self._bridge = CvBridge()
        self._face_pub = self.create_publisher(FaceDetection, 'faces', 10)

        # Initialize MediaPipe Face Detection
        self._mp_face = mp.solutions.face_detection
        self._detector = self._mp_face.FaceDetection(
            model_selection=0,  # 0 = short-range (< 2m), 1 = full-range (< 5m)
            min_detection_confidence=min_conf,
        )

        # Subscribe to both camera topics
        self.create_subscription(Image, '/camera/phone/image_raw', self._on_phone_image, 10)
        self.create_subscription(Image, '/camera/espcam/image_raw', self._on_espcam_image, 10)

        self.get_logger().info('Face detector node started (MediaPipe).')

    def _on_phone_image(self, msg: Image):
        self._process_image(msg, source='phone')

    def _on_espcam_image(self, msg: Image):
        self._process_image(msg, source='espcam')

    def _process_image(self, msg: Image, source: str):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        # MediaPipe requires RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self._detector.process(rgb)

        det_msg = FaceDetection()
        det_msg.source = source

        if results.detections:
            h, w, _ = frame.shape
            centers = []
            confidences = []

            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                # Compute center in pixel coordinates
                cx = (bbox.xmin + bbox.width / 2.0) * w
                cy = (bbox.ymin + bbox.height / 2.0) * h

                point = Point()
                point.x = float(cx)
                point.y = float(cy)
                point.z = 0.0
                centers.append(point)
                confidences.append(float(detection.score[0]))

            det_msg.bounding_box_centers = centers
            det_msg.confidences = confidences

            # Attach the source frame for downstream face recognition
            det_msg.source_frame = msg

        self._face_pub.publish(det_msg)

    def destroy_node(self):
        self._detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
