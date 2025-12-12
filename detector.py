import numpy as np
import tflite_runtime.interpreter as tflite
from utils import resize_frame

class ObstacleDetector:
    def __init__(self, model_path="models/mobilenet_ssd.tflite"):
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def detect(self, frame):
        inp = resize_frame(frame, (300, 300))
        inp = np.expand_dims(inp, axis=0)

        self.interpreter.set_tensor(self.input_details[0]['index'], inp)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]

        detections = []
        for i, score in enumerate(scores):
            if score > 0.5:
                detections.append({
                    "box": boxes[i],
                    "class": int(classes[i]),
                    "score": float(score)
                })
        return detections
