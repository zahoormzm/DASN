"""
Object Detector Node — /ml/object_detector

Subscribes to /camera/phone/image_raw, runs YOLOv8n TFLite inference,
and publishes vision_msgs/Detection2DArray.
"""

import os

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
    from sensor_msgs.msg import Image
except ImportError as e:
    raise ImportError(f'sensor_msgs is required: {e}.')

try:
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    from geometry_msgs.msg import Pose2D
except ImportError as e:
    raise ImportError(f'vision_msgs is required: {e}. Install ros-<distro>-vision-msgs.')


# COCO class names (80 classes) used by YOLOv8
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
]


class ObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('object_detector', namespace='ml')

        self.declare_parameter('model_path', 'models/yolov8n.tflite')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('input_size', 640)

        self._model_path = self.get_parameter('model_path').value
        self._conf_threshold = self.get_parameter('confidence_threshold').value
        self._nms_iou = self.get_parameter('nms_iou_threshold').value
        self._input_size = self.get_parameter('input_size').value

        self._bridge = CvBridge()
        self._detection_pub = self.create_publisher(Detection2DArray, 'objects', 10)

        # Initialize TFLite interpreter
        self._interpreter = None
        self._init_model()

        self.create_subscription(Image, '/camera/phone/image_raw', self._on_image, 10)
        self.get_logger().info('Object detector node started (YOLOv8n TFLite).')

    def _init_model(self):
        if not os.path.isfile(self._model_path):
            self.get_logger().error(
                f'YOLOv8n model file not found at: {self._model_path}. '
                'Object detection will not work until the model is provided. '
                'See models/README.md for export instructions: '
                'from ultralytics import YOLO; model = YOLO("yolov8n.pt"); '
                'model.export(format="tflite")'
            )
            return

        try:
            self._interpreter = tflite.Interpreter(model_path=self._model_path)
            self._interpreter.allocate_tensors()
            self._input_details = self._interpreter.get_input_details()
            self._output_details = self._interpreter.get_output_details()
            self.get_logger().info(f'Loaded YOLOv8n model from {self._model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv8n model: {e}')
            self._interpreter = None

    def _preprocess(self, frame: np.ndarray) -> tuple:
        """Resize and normalize frame for YOLOv8 input. Returns (input_tensor, scale, pad)."""
        h, w, _ = frame.shape
        size = self._input_size

        # Compute scale and padding to preserve aspect ratio (letterbox)
        scale = min(size / w, size / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        pad_x = (size - new_w) // 2
        pad_y = (size - new_h) // 2

        resized = cv2.resize(frame, (new_w, new_h))
        canvas = np.full((size, size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

        # Normalize to [0, 1] float32 and add batch dimension
        inp = canvas.astype(np.float32) / 255.0
        inp = np.expand_dims(inp, axis=0)

        return inp, scale, pad_x, pad_y

    def _postprocess(self, output: np.ndarray, orig_h: int, orig_w: int,
                     scale: float, pad_x: int, pad_y: int) -> list:
        """Parse YOLOv8 output into a list of (x_center, y_center, width, height, class_id, conf)."""
        # YOLOv8 TFLite output shape: [1, num_boxes, 84] (4 box coords + 80 class scores)
        # or transposed [1, 84, num_boxes] — handle both
        if output.ndim == 3:
            if output.shape[1] == 84:
                output = np.transpose(output, (0, 2, 1))
            output = output[0]  # Remove batch dim -> [num_boxes, 84]

        detections = []
        size = self._input_size

        for row in output:
            box = row[:4]  # cx, cy, w, h in input-space
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            conf = float(class_scores[class_id])

            if conf < self._conf_threshold:
                continue

            # Convert from input-space to original image space
            cx = (box[0] - pad_x) / scale
            cy = (box[1] - pad_y) / scale
            bw = box[2] / scale
            bh = box[3] / scale

            # Clamp to image bounds
            cx = max(0.0, min(float(orig_w), float(cx)))
            cy = max(0.0, min(float(orig_h), float(cy)))
            bw = max(0.0, float(bw))
            bh = max(0.0, float(bh))

            detections.append((cx, cy, bw, bh, class_id, conf))

        # Apply NMS
        if not detections:
            return []

        boxes_for_nms = []
        confidences_for_nms = []
        for cx, cy, bw, bh, cid, conf in detections:
            x1 = cx - bw / 2
            y1 = cy - bh / 2
            boxes_for_nms.append([x1, y1, bw, bh])
            confidences_for_nms.append(conf)

        indices = cv2.dnn.NMSBoxes(
            boxes_for_nms, confidences_for_nms,
            self._conf_threshold, self._nms_iou
        )

        if len(indices) == 0:
            return []

        # OpenCV NMSBoxes returns different shapes depending on version
        if isinstance(indices, np.ndarray):
            indices = indices.flatten().tolist()
        else:
            indices = [i for i in indices]

        return [detections[i] for i in indices]

    def _on_image(self, msg: Image):
        if self._interpreter is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        orig_h, orig_w, _ = frame.shape
        inp, scale, pad_x, pad_y = self._preprocess(frame)

        self._interpreter.set_tensor(self._input_details[0]['index'], inp)
        self._interpreter.invoke()
        raw_output = self._interpreter.get_tensor(self._output_details[0]['index'])

        detections = self._postprocess(raw_output, orig_h, orig_w, scale, pad_x, pad_y)

        det_array = Detection2DArray()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id

        for cx, cy, bw, bh, class_id, conf in detections:
            det = Detection2D()

            # Set bounding box center and size
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = bw
            det.bbox.size_y = bh

            # Set classification result
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else str(class_id)
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            det_array.detections.append(det)

        self._detection_pub.publish(det_array)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
