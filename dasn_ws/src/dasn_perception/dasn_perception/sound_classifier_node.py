"""
Sound Classifier Node — /ml/sound_classifier

Subscribes to /camera/phone/audio_raw (dasn_msgs/AudioChunk),
runs YAMNet TFLite inference for audio event classification,
and publishes dasn_msgs/SoundEvent for security-relevant sounds.
"""

import os

try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    raise ImportError(f'rclpy is required: {e}.')

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
    from dasn_msgs.msg import AudioChunk, SoundEvent
except ImportError as e:
    raise ImportError(f'dasn_msgs is required: {e}. Build the dasn_msgs package first.')


# YAMNet has 521 audio event classes. These are the security-relevant ones
# with their approximate YAMNet class indices.
SECURITY_CLASSES = {
    'glass_breaking': [441, 442],  # Glass, Shatter
    'scream': [1, 14, 15],         # Scream, Shout, Yell
    'gunshot': [427, 428, 429],    # Gunshot, Machine gun, Fusillade
    'dog_bark': [74],              # Bark
    'siren': [396, 397, 398],      # Siren, Civil defense siren, Ambulance siren
    'explosion': [425, 426],       # Explosion, Burst
    'door_knock': [355, 356],      # Knock, Tap
}

# Full set of security-relevant YAMNet indices mapped to our class names
SECURITY_INDEX_MAP = {}
for class_name, indices in SECURITY_CLASSES.items():
    for idx in indices:
        SECURITY_INDEX_MAP[idx] = class_name


class SoundClassifierNode(Node):

    def __init__(self):
        super().__init__('sound_classifier', namespace='ml')

        self.declare_parameter('model_path', 'models/yamnet.tflite')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('zone_id', 'FRONT_DOOR')
        self.declare_parameter('yamnet_sample_rate', 16000)

        self._model_path = self.get_parameter('model_path').value
        self._conf_threshold = self.get_parameter('confidence_threshold').value
        self._zone_id = self.get_parameter('zone_id').value
        self._yamnet_sr = self.get_parameter('yamnet_sample_rate').value

        self._sound_pub = self.create_publisher(SoundEvent, 'sound_events', 10)

        # Audio buffer for accumulating chunks into YAMNet-sized windows
        # YAMNet expects ~0.975 seconds of audio at 16kHz = 15600 samples
        self._audio_buffer = np.array([], dtype=np.float32)
        self._yamnet_window = 15600  # samples

        # Initialize TFLite interpreter
        self._interpreter = None
        self._init_model()

        self.create_subscription(AudioChunk, '/camera/phone/audio_raw', self._on_audio, 10)
        self.get_logger().info('Sound classifier node started (YAMNet TFLite).')

    def _init_model(self):
        if not os.path.isfile(self._model_path):
            self.get_logger().error(
                f'YAMNet model file not found at: {self._model_path}. '
                'Sound classification will not work until the model is provided. '
                'See models/README.md for download instructions.'
            )
            return

        try:
            self._interpreter = tflite.Interpreter(model_path=self._model_path)
            self._interpreter.allocate_tensors()
            self._input_details = self._interpreter.get_input_details()
            self._output_details = self._interpreter.get_output_details()
            self.get_logger().info(f'Loaded YAMNet model from {self._model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YAMNet model: {e}')
            self._interpreter = None

    def _on_audio(self, msg: AudioChunk):
        if self._interpreter is None:
            return

        # Convert int16 samples to float32 in [-1, 1]
        samples = np.array(msg.data, dtype=np.float32) / 32768.0

        # Resample if necessary (simple nearest-neighbor for speed)
        if msg.sample_rate != self._yamnet_sr and msg.sample_rate > 0:
            ratio = self._yamnet_sr / msg.sample_rate
            new_len = int(len(samples) * ratio)
            indices = np.linspace(0, len(samples) - 1, new_len).astype(int)
            samples = samples[indices]

        # If stereo, average to mono
        if msg.channels == 2 and len(samples) > 1:
            samples = samples.reshape(-1, 2).mean(axis=1)

        # Accumulate in buffer
        self._audio_buffer = np.concatenate([self._audio_buffer, samples])

        # Process when we have enough for a YAMNet window
        while len(self._audio_buffer) >= self._yamnet_window:
            window = self._audio_buffer[:self._yamnet_window]
            self._audio_buffer = self._audio_buffer[self._yamnet_window:]
            self._classify_window(window, msg.timestamp)

    def _classify_window(self, waveform: np.ndarray, timestamp):
        """Run YAMNet inference on one audio window and publish security-relevant events."""
        input_data = waveform.astype(np.float32)

        # Reshape to match expected input
        expected_shape = self._input_details[0]['shape']
        if len(expected_shape) == 1:
            input_data = input_data[:expected_shape[0]]
        elif len(expected_shape) == 2:
            input_data = input_data[:expected_shape[1]].reshape(expected_shape)

        self._interpreter.set_tensor(self._input_details[0]['index'], input_data)
        self._interpreter.invoke()

        # YAMNet output: scores [num_frames, 521]
        scores = self._interpreter.get_tensor(self._output_details[0]['index'])

        # Average scores across all frames
        if scores.ndim == 2:
            avg_scores = scores.mean(axis=0)
        else:
            avg_scores = scores.flatten()

        # Check for security-relevant classes
        for idx, score in enumerate(avg_scores):
            if idx in SECURITY_INDEX_MAP and score >= self._conf_threshold:
                event_msg = SoundEvent()
                event_msg.classification = SECURITY_INDEX_MAP[idx]
                event_msg.confidence = float(score)
                event_msg.zone_id = self._zone_id
                event_msg.timestamp = timestamp
                self._sound_pub.publish(event_msg)

                self.get_logger().info(
                    f'Sound event: {event_msg.classification} '
                    f'(conf={score:.3f}) in zone {self._zone_id}'
                )

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
