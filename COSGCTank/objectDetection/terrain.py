import cv2
import numpy as np


class TerrainAnalyzer:
    def __init__(self, lower_band_fraction=0.35, dip_variance_thresh=200.0, incline_grad_thresh=20.0):
        """Analyze near-field terrain for dips and inclines.

        Parameters are tunable; the defaults are conservative guesses and should be
        tuned on a per-vehicle basis.
        """
        self.lower_band_fraction = float(lower_band_fraction)
        self.dip_variance_thresh = float(dip_variance_thresh)
        self.incline_grad_thresh = float(incline_grad_thresh)

    def analyze(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * (1.0 - self.lower_band_fraction))
        lower = frame[y0:, :]

        gray = cv2.cvtColor(lower, cv2.COLOR_RGB2GRAY)

        # measure local variance: a large flat dark area (dip) tends to lower variance
        variance = float(np.var(gray))

        # vertical gradients: large gradients can indicate inclines or drop-offs
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        grad_mean = float(np.mean(np.abs(sobel_y)))

        # edge-based density for additional signal
        edges = cv2.Canny(gray, 50, 120)
        edge_density = float(np.sum(edges > 0) / edges.size)

        # heuristics (tunable)
        dip = variance < self.dip_variance_thresh and edge_density < 0.02
        incline = grad_mean > self.incline_grad_thresh or edge_density > 0.12

        return {
            "dip_detected": bool(dip),
            "incline_detected": bool(incline),
            "edge_density": edge_density,
            "variance": variance,
            "grad_mean": grad_mean
        }
