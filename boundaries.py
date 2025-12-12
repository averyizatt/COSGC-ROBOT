import cv2
import numpy as np

class BoundaryDetector:
    def __init__(self):
        pass

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 60, 120)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=40, maxLineGap=50)

        left_lines = []
        right_lines = []

        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-5)

                if slope < -0.5:  
                    left_lines.append(l[0])
                elif slope > 0.5:
                    right_lines.append(l[0])

        return {
            "left_boundary": left_lines,
            "right_boundary": right_lines
        }
