"""
Pose Analyzer Node — /ml/pose_analyzer

Subscribes to /camera/phone/image_raw, runs MediaPipe Pose estimation,
classifies postures (standing, walking, crouching, climbing, lying_down),
and publishes dasn_msgs/PoseAnalysis.
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
    import numpy as np
except ImportError as e:
    raise ImportError(f'numpy is required: {e}. Install with: pip install numpy')

try:
    from sensor_msgs.msg import Image
except ImportError as e:
    raise ImportError(f'sensor_msgs is required: {e}.')

try:
    from dasn_msgs.msg import PoseAnalysis
except ImportError as e:
    raise ImportError(f'dasn_msgs is required: {e}. Build the dasn_msgs package first.')

import math


class PoseAnalyzerNode(Node):

    POSTURE_CLASSES = ['standing', 'walking', 'crouching', 'climbing', 'lying_down']

    def __init__(self):
        super().__init__('pose_analyzer', namespace='ml')

        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence', 0.5)

        min_det = self.get_parameter('min_detection_confidence').value
        min_trk = self.get_parameter('min_tracking_confidence').value

        self._bridge = CvBridge()
        self._pose_pub = self.create_publisher(PoseAnalysis, 'poses', 10)

        # MediaPipe Pose
        self._mp_pose = mp.solutions.pose
        self._pose = self._mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            min_detection_confidence=min_det,
            min_tracking_confidence=min_trk,
        )

        # Track previous hip position for walking detection
        self._prev_hip_x = None
        self._motion_history = []

        self.create_subscription(Image, '/camera/phone/image_raw', self._on_image, 10)
        self.get_logger().info('Pose analyzer node started (MediaPipe Pose).')

    def _angle_between(self, a, b, c) -> float:
        """Compute angle at point b formed by points a-b-c, in degrees."""
        ba = np.array([a.x - b.x, a.y - b.y])
        bc = np.array([c.x - b.x, c.y - b.y])
        cos_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc) + 1e-8)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        return float(np.degrees(np.arccos(cos_angle)))

    def _classify_posture(self, landmarks) -> tuple:
        """
        Classify the detected pose into a posture category.
        Returns (primary_classification, confidence, list_of_detected_postures).
        """
        lm = landmarks.landmark
        PoseLM = self._mp_pose.PoseLandmark

        # Key landmarks
        nose = lm[PoseLM.NOSE]
        l_shoulder = lm[PoseLM.LEFT_SHOULDER]
        r_shoulder = lm[PoseLM.RIGHT_SHOULDER]
        l_hip = lm[PoseLM.LEFT_HIP]
        r_hip = lm[PoseLM.RIGHT_HIP]
        l_knee = lm[PoseLM.LEFT_KNEE]
        r_knee = lm[PoseLM.RIGHT_KNEE]
        l_ankle = lm[PoseLM.LEFT_ANKLE]
        r_ankle = lm[PoseLM.RIGHT_ANKLE]
        l_wrist = lm[PoseLM.LEFT_WRIST]
        r_wrist = lm[PoseLM.RIGHT_WRIST]

        # Midpoints
        mid_shoulder_y = (l_shoulder.y + r_shoulder.y) / 2.0
        mid_hip_y = (l_hip.y + r_hip.y) / 2.0
        mid_hip_x = (l_hip.x + r_hip.x) / 2.0
        mid_knee_y = (l_knee.y + r_knee.y) / 2.0
        mid_ankle_y = (l_ankle.y + r_ankle.y) / 2.0

        # Torso angle (vertical = 0 degrees)
        torso_dy = mid_hip_y - mid_shoulder_y
        torso_dx = (l_hip.x + r_hip.x) / 2.0 - (l_shoulder.x + r_shoulder.x) / 2.0
        torso_angle = abs(math.degrees(math.atan2(torso_dx, torso_dy)))

        # Knee angles
        l_knee_angle = self._angle_between(l_hip, l_knee, l_ankle)
        r_knee_angle = self._angle_between(r_hip, r_knee, r_ankle)
        avg_knee_angle = (l_knee_angle + r_knee_angle) / 2.0

        # Height ratio: vertical span of the body
        body_height = mid_ankle_y - nose.y  # positive means person is upright
        body_width = abs(l_ankle.x - r_ankle.x) + abs(l_shoulder.x - r_shoulder.x)

        detected = []
        scores = {}

        # --- LYING DOWN ---
        # Torso is nearly horizontal (angle > 60 degrees from vertical)
        if torso_angle > 60:
            conf = min(1.0, (torso_angle - 60) / 30.0)
            detected.append('lying_down')
            scores['lying_down'] = conf

        # --- CROUCHING ---
        # Knees are significantly bent and torso is low
        if avg_knee_angle < 110 and torso_angle < 45:
            conf = min(1.0, (110 - avg_knee_angle) / 50.0)
            detected.append('crouching')
            scores['crouching'] = conf

        # --- CLIMBING ---
        # One hand is significantly above the shoulder and knees bent asymmetrically
        hands_above_shoulder = (
            l_wrist.y < l_shoulder.y - 0.1 or r_wrist.y < r_shoulder.y - 0.1
        )
        knee_asymmetry = abs(l_knee_angle - r_knee_angle) > 30
        if hands_above_shoulder and knee_asymmetry:
            conf = 0.7
            detected.append('climbing')
            scores['climbing'] = conf

        # --- WALKING ---
        # Detect lateral hip motion over time
        if self._prev_hip_x is not None:
            dx = abs(mid_hip_x - self._prev_hip_x)
            self._motion_history.append(dx)
            if len(self._motion_history) > 15:
                self._motion_history.pop(0)

            avg_motion = sum(self._motion_history) / len(self._motion_history)
            if avg_motion > 0.005 and torso_angle < 30 and avg_knee_angle > 120:
                conf = min(1.0, avg_motion / 0.02)
                detected.append('walking')
                scores['walking'] = conf

        self._prev_hip_x = mid_hip_x

        # --- STANDING ---
        # Upright torso with relatively straight knees
        if torso_angle < 25 and avg_knee_angle > 150:
            conf = min(1.0, (avg_knee_angle - 140) / 30.0)
            detected.append('standing')
            scores['standing'] = conf

        # Default if nothing detected
        if not detected:
            detected.append('standing')
            scores['standing'] = 0.5

        # Primary classification is the one with highest confidence
        primary = max(scores, key=scores.get)
        return primary, scores[primary], detected

    def _on_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self._pose.process(rgb)

        pose_msg = PoseAnalysis()
        pose_msg.timestamp = self.get_clock().now().to_msg()

        if results.pose_landmarks:
            classification, confidence, detected = self._classify_posture(results.pose_landmarks)
            pose_msg.classification = classification
            pose_msg.confidence = float(confidence)
            pose_msg.detected_poses = detected
        else:
            pose_msg.classification = 'none'
            pose_msg.confidence = 0.0
            pose_msg.detected_poses = []

        self._pose_pub.publish(pose_msg)

    def destroy_node(self):
        self._pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
