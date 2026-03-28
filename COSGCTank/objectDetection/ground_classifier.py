import cv2
import numpy as np

class GroundClassifier:
    """Lightweight ground-type classifier for the lower ROI.

    Heuristic model using HSV color ranges + texture metrics to classify
    ground as 'sand', 'dirt', 'grass', or 'unknown'. Designed for rover-height
    viewpoints with a strong ground presence in the lower image band.
    """
    def __init__(self, roi_fraction=0.5):
        self.roi_fraction = float(roi_fraction)

    def analyze(self, frame):
        h, w = frame.shape[:2]
        y0 = int(h * (1.0 - self.roi_fraction))
        roi = frame[y0:, :]

        # RGB->HSV for color-based discrimination
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
        hch, sch, vch = cv2.split(hsv)

        # Texture/entropy features
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        try:
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            gray = clahe.apply(gray)
        except Exception:
            pass
        hist = cv2.calcHist([gray],[0],None,[32],[0,256]).flatten()
        p = hist / max(np.sum(hist), 1e-6)
        entropy = float(-np.sum(p * np.log2(p + 1e-12)))
        variance = float(np.var(gray))
        brightness = float(np.mean(vch))

        # Color masks (loose) — tuned for outdoor natural light
        # Sand: light yellow/beige (H ~15-40), moderate S (20-140), high V
        sand_mask = cv2.inRange(hsv, (10, 20, 120), (40, 140, 255))
        # Dirt: darker brown (H ~10-20), higher S, lower V
        dirt_mask = cv2.inRange(hsv, (5, 40, 40), (25, 200, 160))
        # Grass: green (H ~35-85), moderate-to-high S, moderate V
        grass_mask = cv2.inRange(hsv, (35, 50, 40), (85, 255, 220))

        total = roi.shape[0] * roi.shape[1]
        frac = lambda m: float(np.sum(m > 0)) / max(total, 1)
        f_sand = frac(sand_mask)
        f_dirt = frac(dirt_mask)
        f_grass = frac(grass_mask)

        # Simple rule-based probabilities mixing color fraction and texture cues
        # Sand tends to be brighter with low-to-mid entropy; dirt darker; grass greener with higher entropy.
        ps_sand = 0.55 * f_sand + 0.25 * max(0.0, (brightness - 120.0) / 135.0) + 0.20 * max(0.0, (3.5 - entropy) / 3.5)
        ps_dirt = 0.6 * f_dirt + 0.2 * max(0.0, (160.0 - brightness) / 160.0) + 0.2 * max(0.0, (variance - 120.0) / 300.0)
        ps_grass = 0.75 * f_grass + 0.25 * max(0.0, (entropy - 3.0) / 5.0)

        raw = np.array([ps_sand, ps_dirt, ps_grass, 0.01], dtype=np.float32)  # last is unknown prior
        # softmax-like normalization
        e = np.exp(raw - np.max(raw))
        probs = e / np.sum(e)
        labels = ['sand', 'dirt', 'grass', 'unknown']
        idx = int(np.argmax(probs))
        return {
            'type': labels[idx],
            'confidence': float(probs[idx]),
            'probs': {labels[i]: float(probs[i]) for i in range(len(labels))},
            'entropy': entropy,
            'variance': variance,
            'brightness': brightness,
            'roi': {'y0': y0, 'fraction': self.roi_fraction}
        }
