"""
Face Recognizer Node — /ml/face_recognizer

Subscribes to /ml/faces (dasn_msgs/FaceDetection), crops detected faces,
runs MobileFaceNet TFLite to get 128-dim embeddings, compares against a
SQLite database of known faces using cosine similarity, and publishes
dasn_msgs/FaceIdentity.
"""

import os
import sqlite3
import struct

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
    import numpy as np
except ImportError as e:
    raise ImportError(f'numpy is required: {e}. Install with: pip install numpy')

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError as e:
        raise ImportError(
            f'TFLite runtime is required: {e}. '
            'Install with: pip install tflite-runtime or pip install tensorflow'
        )

try:
    from dasn_msgs.msg import FaceDetection, FaceIdentity
except ImportError as e:
    raise ImportError(f'dasn_msgs is required: {e}. Build the dasn_msgs package first.')

from sensor_msgs.msg import Image


# Embedding dimension for MobileFaceNet
EMBEDDING_DIM = 128


def _embedding_to_blob(embedding: np.ndarray) -> bytes:
    """Pack a float32 numpy array into raw bytes for SQLite storage."""
    return struct.pack(f'{len(embedding)}f', *embedding.tolist())


def _blob_to_embedding(blob: bytes) -> np.ndarray:
    """Unpack raw bytes from SQLite into a float32 numpy array."""
    count = len(blob) // 4
    return np.array(struct.unpack(f'{count}f', blob), dtype=np.float32)


def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    if norm_a == 0.0 or norm_b == 0.0:
        return 0.0
    return float(np.dot(a, b) / (norm_a * norm_b))


class FaceRecognizerNode(Node):

    def __init__(self):
        super().__init__('face_recognizer', namespace='ml')

        self.declare_parameter('model_path', 'models/mobilefacenet.tflite')
        self.declare_parameter('similarity_threshold', 0.6)
        self.declare_parameter('db_path', os.path.expanduser('~/.dasn/faces.db'))
        self.declare_parameter('face_input_size', 112)  # MobileFaceNet expects 112x112

        self._model_path = self.get_parameter('model_path').value
        self._threshold = self.get_parameter('similarity_threshold').value
        self._db_path = self.get_parameter('db_path').value
        self._face_size = self.get_parameter('face_input_size').value

        self._bridge = CvBridge()
        self._identity_pub = self.create_publisher(FaceIdentity, 'face_identity', 10)

        # Initialize SQLite database
        self._init_database()

        # Load known face embeddings into memory
        self._known_faces = self._load_known_faces()
        self.get_logger().info(
            f'Loaded {len(self._known_faces)} known face(s) from {self._db_path}'
        )

        # Initialize TFLite interpreter
        self._interpreter = None
        self._init_model()

        self.create_subscription(FaceDetection, 'faces', self._on_faces, 10)
        self.get_logger().info('Face recognizer node started (MobileFaceNet).')

    def _init_database(self):
        """Create the SQLite database and faces table if they do not exist."""
        db_dir = os.path.dirname(self._db_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir, exist_ok=True)
            self.get_logger().info(f'Created database directory: {db_dir}')

        conn = sqlite3.connect(self._db_path)
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS faces (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                embedding BLOB NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        conn.commit()
        conn.close()
        self.get_logger().info(f'Database initialized at {self._db_path}')

    def _load_known_faces(self) -> list:
        """Load all known face embeddings from the database.
        Returns a list of (name, embedding_ndarray) tuples.
        """
        conn = sqlite3.connect(self._db_path)
        cursor = conn.cursor()
        cursor.execute('SELECT name, embedding FROM faces')
        rows = cursor.fetchall()
        conn.close()

        known = []
        for name, blob in rows:
            emb = _blob_to_embedding(blob)
            known.append((name, emb))
        return known

    def _init_model(self):
        """Load the MobileFaceNet TFLite model."""
        if not os.path.isfile(self._model_path):
            self.get_logger().error(
                f'MobileFaceNet model not found at {self._model_path}. '
                'Face recognition will not work until the model is provided. '
                'See models/README.md for download instructions.'
            )
            return

        try:
            self._interpreter = tflite.Interpreter(model_path=self._model_path)
            self._interpreter.allocate_tensors()
            self._input_details = self._interpreter.get_input_details()
            self._output_details = self._interpreter.get_output_details()
            self.get_logger().info(f'Loaded MobileFaceNet model from {self._model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load MobileFaceNet model: {e}')
            self._interpreter = None

    def _get_embedding(self, face_img: np.ndarray) -> np.ndarray:
        """Run MobileFaceNet inference on a face crop and return 128-dim embedding."""
        # Preprocess: resize to expected input, normalize to [-1, 1]
        resized = cv2.resize(face_img, (self._face_size, self._face_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        inp = rgb.astype(np.float32) / 127.5 - 1.0
        inp = np.expand_dims(inp, axis=0)

        self._interpreter.set_tensor(self._input_details[0]['index'], inp)
        self._interpreter.invoke()
        embedding = self._interpreter.get_tensor(self._output_details[0]['index'])[0]
        return embedding.astype(np.float32)

    def _on_faces(self, msg: FaceDetection):
        if self._interpreter is None:
            return

        if not msg.bounding_box_centers:
            return

        # Decode the source frame
        try:
            frame = self._bridge.imgmsg_to_cv2(msg.source_frame, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to decode source frame: {e}')
            return

        h, w, _ = frame.shape

        for i, center in enumerate(msg.bounding_box_centers):
            cx, cy = center.x, center.y

            # Estimate a square crop around the face center
            # Use a generous crop size based on typical face proportions
            crop_half = int(min(w, h) * 0.15)
            x1 = max(0, int(cx) - crop_half)
            y1 = max(0, int(cy) - crop_half)
            x2 = min(w, int(cx) + crop_half)
            y2 = min(h, int(cy) + crop_half)

            if x2 - x1 < 10 or y2 - y1 < 10:
                continue

            face_crop = frame[y1:y2, x1:x2]
            embedding = self._get_embedding(face_crop)

            # Compare against known faces
            best_name = 'unknown'
            best_score = 0.0
            is_known = False

            for name, known_emb in self._known_faces:
                sim = _cosine_similarity(embedding, known_emb)
                if sim > best_score:
                    best_score = sim
                    best_name = name

            if best_score >= self._threshold:
                is_known = True

            # Publish identity
            identity_msg = FaceIdentity()
            identity_msg.name = best_name if is_known else 'unknown'
            identity_msg.confidence = float(best_score)
            identity_msg.is_known = is_known
            identity_msg.source = msg.source

            # Attach the face crop
            try:
                identity_msg.face_crop = self._bridge.cv2_to_imgmsg(face_crop, encoding='bgr8')
            except Exception:
                pass

            self._identity_pub.publish(identity_msg)

            self.get_logger().debug(
                f'Face {i}: {"KNOWN" if is_known else "UNKNOWN"} '
                f'name={identity_msg.name} conf={best_score:.3f}'
            )

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
