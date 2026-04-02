# DASN Perception Models

Place the following TFLite model files in this directory before running inference nodes.

## YAMNet (Sound Classification)

Download the TFLite model from TensorFlow Hub:

    wget https://tfhub.dev/google/lite-model/yamnet/tflite/1?lite-format=tflite -O yamnet.tflite

Or browse: https://tfhub.dev/google/yamnet/1

Expected file: `yamnet.tflite`

## MobileFaceNet (Face Recognition)

Download the TFLite model from the sirius-ai/MobileFaceNet_TF repository:

    git clone https://github.com/sirius-ai/MobileFaceNet_TF.git
    cd MobileFaceNet_TF
    # Follow the repo instructions to convert the checkpoint to TFLite format
    # Place the resulting model here as mobilefacenet.tflite

Expected file: `mobilefacenet.tflite`

## YOLOv8n (Object Detection)

Export via the Ultralytics Python API:

    pip install ultralytics
    python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.export(format='tflite')"
    cp yolov8n_saved_model/yolov8n_float32.tflite models/yolov8n.tflite

Expected file: `yolov8n.tflite`

## MediaPipe Face Detection and Pose

These models are bundled internally with the `mediapipe` Python package.
No separate download is needed. Just install mediapipe:

    pip install mediapipe
