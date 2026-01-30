import cv2
import numpy as np

class BarrierDetector:
    """Heuristic barrier detector for low-height rover.

    Detects tall, narrow vertical structures in the lower ROI using edges and
    contour aspect ratio filters. Useful as a complementary signal when the
    primary detector model lacks a 'barrier' class.
    """
    def __init__(self, roi_fraction=0.6, min_rel_height=0.15, max_aspect=0.6, min_area=300):
        self.roi_fraction = float(roi_fraction)
        self.min_rel_height = float(min_rel_height)
        self.max_aspect = float(max_aspect)  # width/height must be <= this
        self.min_area = float(min_area)

    def detect(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * (1.0 - self.roi_fraction))
        roi = frame[y0:, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(gray, 60, 140)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,7))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        dets = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            # Require tall shapes
            if ch <= 0:
                continue
            aspect = float(cw) / float(ch)
            rel_h = float(ch) / max(roi.shape[0], 1)
            if aspect > self.max_aspect:
                continue
            if rel_h < self.min_rel_height:
                continue
            # Map to full-frame normalized box
            ymin = (y + y0) / h
            xmin = x / w
            ymax = (y + ch + y0) / h
            xmax = (x + cw) / w
            # Score by height and thinness
            score = max(0.2, min(1.0, 0.5*rel_h + 0.5*(1.0 - aspect)))
            dets.append({
                'box': [float(ymin), float(xmin), float(ymax), float(xmax)],
                'class': 1,
                'label': 'barrier',
                'score': float(score)
            })
        # Non-maximum suppression among barrier boxes (simple IoU-based)
        dets = sorted(dets, key=lambda d: d.get('score', 0.0), reverse=True)
        keep = []
        def iou(a,b):
            ax0, ay0, ax1, ay1 = a['box'][1], a['box'][0], a['box'][3], a['box'][2]
            bx0, by0, bx1, by1 = b['box'][1], b['box'][0], b['box'][3], b['box'][2]
            ix0, iy0 = max(ax0, bx0), max(ay0, by0)
            ix1, iy1 = min(ax1, bx1), min(ay1, by1)
            iw, ih = max(0.0, ix1-ix0), max(0.0, iy1-iy0)
            inter = iw*ih
            aa = (ax1-ax0)*(ay1-ay0)
            bb = (bx1-bx0)*(by1-by0)
            return inter / max(aa+bb-inter, 1e-6)
        for d in dets:
            if all(iou(d, k) < 0.3 for k in keep):
                keep.append(d)
        return keep
