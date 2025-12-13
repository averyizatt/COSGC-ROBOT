import numpy as np
from utils import resize_frame

try:
    import tflite_runtime.interpreter as tflite
except Exception:
    try:
        # fallback to TF lite if available
        from tensorflow.lite import Interpreter as tflite
    except Exception:
        tflite = None


class ObstacleDetector:
    """TFLite SSD-based obstacle detector wrapper.

    Returns detections in the format used by the rest of the project:
      [{ 'box': [ymin, xmin, ymax, xmax], 'class': int, 'score': float }, ...]

    `box` values are normalized (0..1) by default to remain compatible with
    existing decision logic which checks vertical position (ymin).
    """

    def __init__(self, model_path="models/mobilenet_ssd.tflite", input_size=(300, 300), score_threshold=0.5, labels=None):
        if tflite is None:
            raise RuntimeError("TFLite interpreter not available; install tflite-runtime or TensorFlow Lite")

        self.model_path = model_path
        self.input_size = input_size
        self.score_threshold = float(score_threshold)
        self.labels = labels or {}

        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def _preprocess(self, frame):
        # Resize to model input and ensure uint8
        img = resize_frame(frame, self.input_size)

        # Brightness normalization: use HSV V-channel equalization (CLAHE)
        try:
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            v = hsv[:, :, 2]
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            v_eq = clahe.apply(v)
            hsv[:, :, 2] = v_eq
            img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        except Exception:
            # If OpenCV conversion fails for any reason, fall back to original
            pass

        # Some TFLite models expect uint8, others float - for MobileNet SSD uint8 is common
        if img.dtype != np.uint8:
            img = img.astype('uint8')

        return np.expand_dims(img, axis=0)

    def detect(self, frame):
        """Run the detector on an RGB frame and return normalized detections.

        Keeps compatibility: boxes are normalized as [ymin, xmin, ymax, xmax].
        """
        inp = self._preprocess(frame)

        self.interpreter.set_tensor(self.input_details[0]['index'], inp)
        self.interpreter.invoke()

        # output ordering can vary by model; many SSDs use boxes, classes, scores
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]

        detections = []
        for i, score in enumerate(scores):
            if float(score) >= self.score_threshold:
                bbox = boxes[i].tolist()
                cls = int(classes[i]) if classes is not None else 0
                detections.append({
                    "box": bbox,
                    "class": cls,
                    "score": float(score),
                    "label": self.labels.get(cls, str(cls))
                })

        return detections
