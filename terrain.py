import cv2
import numpy as np

class TerrainAnalyzer:
    def __init__(self):
        pass

    def analyze(self, frame):
        h, w, _ = frame.shape
        lower = frame[int(h*0.6):, :]  
        gray = cv2.cvtColor(lower, cv2.COLOR_RGB2GRAY)

        edges = cv2.Canny(gray, 50, 120)
        edge_density = np.sum(edges > 0) / edges.size

        dip = edge_density < 0.015
        incline = edge_density > 0.10

        return {
            "dip_detected": dip,
            "incline_detected": incline,
            "edge_density": float(edge_density)
        }
