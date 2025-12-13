import cv2
import numpy as np


class BoundaryDetector:
    def __init__(self, roi_fraction=0.6):
        """Boundary detector using Canny + Hough lines.

        roi_fraction: fraction of the lower image to analyze (0..1). Typical
        values are 0.5-0.8 to focus on the near-field.
        """
        self.roi_fraction = float(roi_fraction)

    def _adaptive_canny(self, gray, sigma=0.33):
        # compute median of the pixel intensities
        v = np.median(gray)
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        return cv2.Canny(gray, lower, upper)

    def _average_lines(self, lines):
        # average multiple lines into a representative line (x1,y1,x2,y2)
        if not lines:
            return []
        xs = []
        ys = []
        for l in lines:
            x1, y1, x2, y2 = l
            xs += [x1, x2]
            ys += [y1, y2]
        x1, x2 = int(np.min(xs)), int(np.max(xs))
        y1, y2 = int(np.min(ys)), int(np.max(ys))
        return [[x1, y1, x2, y2]]

    def detect(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * (1.0 - self.roi_fraction))
        lower = frame[y0:, :]

        gray = cv2.cvtColor(lower, cv2.COLOR_RGB2GRAY)
        edges = self._adaptive_canny(gray)

        # Hough on ROI
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=40, minLineLength=30, maxLineGap=40)

        left_lines = []
        right_lines = []

        if lines is not None:
            for l in lines:
                x1, y1, x2, y2 = l[0]
                # remap to full-frame coordinates
                y1_full = y1 + y0
                y2_full = y2 + y0

                # slope: positive means right-leaning
                slope = (y2 - y1) / (x2 - x1 + 1e-5)

                if slope < -0.3:
                    left_lines.append([x1, y1_full, x2, y2_full])
                elif slope > 0.3:
                    right_lines.append([x1, y1_full, x2, y2_full])

        # Reduce multiple segments to representative averaged lines
        left_avg = self._average_lines(left_lines)
        right_avg = self._average_lines(right_lines)

        return {
            "left_boundary": left_avg,
            "right_boundary": right_avg,
            "raw_left": left_lines,
            "raw_right": right_lines
        }
