import cv2
import numpy as np


class BoundaryDetector:
    def __init__(self, roi_fraction=0.6):
        """Boundary detector using Canny + Hough lines with robustness tweaks.

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
        # weighted average by segment length; returns a single representative line
        if not lines:
            return []
        xs1, ys1, xs2, ys2, ws = [], [], [], [], []
        for l in lines:
            x1, y1, x2, y2 = l
            w = max(1.0, np.hypot(x2-x1, y2-y1))
            xs1.append(x1); ys1.append(y1); xs2.append(x2); ys2.append(y2); ws.append(w)
        ws = np.array(ws, dtype=np.float32)
        def wavg(vals):
            vals = np.array(vals, dtype=np.float32)
            return int(np.clip(np.sum(vals*ws)/np.sum(ws), 0, 10000))
        return [[wavg(xs1), wavg(ys1), wavg(xs2), wavg(ys2)]]

    def detect(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * (1.0 - self.roi_fraction))
        lower = frame[y0:, :]

        gray = cv2.cvtColor(lower, cv2.COLOR_RGB2GRAY)
        # contrast equalization for stability under harsh lighting
        try:
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
        except Exception:
            pass
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        edges = self._adaptive_canny(gray)

        # Hough on ROI
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=max(25, w//15), maxLineGap=40)

        left_lines = []
        right_lines = []
        left_conf, right_conf = 0.0, 0.0

        if lines is not None:
            cx = w * 0.5
            for l in lines:
                x1, y1, x2, y2 = l[0]
                # remap to full-frame coordinates
                y1_full = y1 + y0
                y2_full = y2 + y0

                # slope: positive means right-leaning
                slope = (y2 - y1) / (x2 - x1 + 1e-5)
                length = np.hypot(x2-x1, y2-y1)
                # also use position relative to center to classify
                side = ((x1 + x2) * 0.5) - cx

                if slope < -0.3 and side < 0:
                    left_lines.append([x1, y1_full, x2, y2_full])
                    left_conf += max(0.0, min(1.0, length / (w*0.5)))
                elif slope > 0.3 and side > 0:
                    right_lines.append([x1, y1_full, x2, y2_full])
                    right_conf += max(0.0, min(1.0, length / (w*0.5)))

        # Reduce multiple segments to representative averaged lines
        left_avg = self._average_lines(left_lines)
        right_avg = self._average_lines(right_lines)

        return {
            "left_boundary": left_avg,
            "right_boundary": right_avg,
            "raw_left": left_lines,
            "raw_right": right_lines,
            "confidence_left": float(left_conf),
            "confidence_right": float(right_conf)
        }
