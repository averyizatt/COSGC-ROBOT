"""Scale estimator for monocular SLAM/VO using external distance measurements.

Collects pairs of (vo_delta, measured_delta) where vo_delta is the
unscaled translation magnitude reported by the SLAM/VO system between two
poses and measured_delta is a real-world distance observed by a reliable
sensor (wheel encoder or ultrasonic delta while moving). The estimator
computes a robust scale factor (median) to convert VO translation units into
meters.

Usage:
  se = ScaleEstimator(window=50)
  se.add_sample(vo_delta, measured_delta)
  scale = se.get_scale()
"""

from collections import deque
import numpy as np

class ScaleEstimator:
    def __init__(self, window=50):
        self.window = int(window)
        self.samples = deque(maxlen=self.window)

    def add_sample(self, vo_delta, measured_delta):
        if vo_delta is None or measured_delta is None:
            return
        if vo_delta <= 1e-6:
            return
        self.samples.append(float(measured_delta) / float(vo_delta))

    def get_scale(self):
        if not self.samples:
            return None
        arr = np.array(self.samples)
        return float(np.median(arr))

    def clear(self):
        self.samples.clear()
