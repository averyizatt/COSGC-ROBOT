import numpy as np
import cv2
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

    def __init__(self, model_path="models/mobilenet_ssd.tflite", input_size=(300, 300), score_threshold=0.5, labels=None, settings=None):
        if tflite is None:
            raise RuntimeError("TFLite interpreter not available; install tflite-runtime or TensorFlow Lite")

        self.model_path = model_path
        self.input_size = input_size
        self.score_threshold = float(score_threshold)
        self.labels = labels or {}
        self.settings = settings or {}

        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def _preprocess(self, frame):
        # Resize to model input and ensure uint8
        img = resize_frame(frame, self.input_size)

        # Convert to LAB and apply CLAHE on L channel to reduce brightness/contrast issues
        try:
            lab = cv2.cvtColor(img, cv2.COLOR_RGB2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            lab = cv2.merge((l, a, b))
            img = cv2.cvtColor(lab, cv2.COLOR_LAB2RGB)
        except Exception:
            # fallback to HSV equalization if LAB fails
            try:
                hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
                v = hsv[:, :, 2]
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                v_eq = clahe.apply(v)
                hsv[:, :, 2] = v_eq
                img = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
            except Exception:
                pass

        # gamma correction to reduce blown highlights typical on sandy surfaces
        try:
            gamma = float(self.settings.get('det_gamma', 0.9))
            invGamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype('uint8')
            img = cv2.LUT(img.astype('uint8'), table)
        except Exception:
            pass

        # Some TFLite models expect uint8, others float - for MobileNet SSD uint8 is common
        if img.dtype != np.uint8:
            img = img.astype('uint8')

        return np.expand_dims(img, axis=0)

    def _fallback_contours(self, frame):
        """Simple contour-based obstacle/rock detector as a fallback or supplement.

        Returns detections in the same normalized format: [{'box':[ymin,xmin,ymax,xmax],'class':0,'score':...}, ...]
        This helps catch high-contrast rocky obstacles and edges that the model may miss in extreme lighting.
        """
        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            # apply bilateral or small blur to reduce sand grain noise
            blur = cv2.bilateralFilter(gray, d=5, sigmaColor=75, sigmaSpace=75)
            # adaptively threshold to keep features in varying lighting
            th = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)
            # morphology to close small holes
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            th = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            h, w = frame.shape[:2]
            dets = []
            min_area = float(self.settings.get('det_contour_min_area', 400))
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < min_area:  # ignore small noisy regions
                    continue
                x, y, cw, ch = cv2.boundingRect(cnt)
                ymin = y / h
                xmin = x / w
                ymax = (y + ch) / h
                xmax = (x + cw) / w
                # score based on area and aspect ratio
                score = min(1.0, (area / (w*h)) * 10.0)
                dets.append({'box':[ymin, xmin, ymax, xmax], 'class': 0, 'score': float(score), 'label': 'contour'})
            return dets
        except Exception:
            return []

    def _rock_score(self, frame, box):
        """Return a rough rockiness score for the patch in box.

        Uses normalized gradient energy + local binary pattern-like comparison.
        Returns value in [0,1] where higher means more likely rock/texture.
        """
        try:
            h, w = frame.shape[:2]
            ymin, xmin, ymax, xmax = box
            y0 = max(0, min(h-1, int(ymin*h)))
            y1 = max(0, min(h, int(ymax*h)))
            x0 = max(0, min(w-1, int(xmin*w)))
            x1 = max(0, min(w, int(xmax*w)))
            if (y1 - y0) < 10 or (x1 - x0) < 10:
                return 0.0
            patch = frame[y0:y1, x0:x1]
            gray = cv2.cvtColor(patch, cv2.COLOR_RGB2GRAY)
            gray = cv2.GaussianBlur(gray, (3,3), 0)
            gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
            gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
            mag = cv2.magnitude(gx, gy)
            # gradient energy normalized
            ge = float(np.mean(mag)) / 64.0
            ge = max(0.0, min(1.0, ge))

            # simple local binary pattern-like: compare to center pixel on a subsampled grid
            g = gray.astype(np.int16)
            c = g[1:-1:2, 1:-1:2]
            if c.size == 0:
                lbp_ratio = 0.0
            else:
                n1 = (g[0:-2:2, 1:-1:2] > c).astype(np.uint8)
                n2 = (g[2::2, 1:-1:2] > c).astype(np.uint8)
                n3 = (g[1:-1:2, 0:-2:2] > c).astype(np.uint8)
                n4 = (g[1:-1:2, 2::2] > c).astype(np.uint8)
                lbp = (n1 + n2 + n3 + n4)
                lbp_ratio = float(np.mean(lbp)) / 4.0
            score = 0.65*ge + 0.35*lbp_ratio
            return max(0.0, min(1.0, score))
        except Exception:
            return 0.0

    def detect(self, frame):
        """Run the detector on an RGB frame and return normalized detections.

        Keeps compatibility: boxes are normalized as [ymin, xmin, ymax, xmax].
        """
        inp = self._preprocess(frame)

        detections = []
        try:
            self.interpreter.set_tensor(self.input_details[0]['index'], inp)
            self.interpreter.invoke()
            # output ordering can vary by model; many SSDs use boxes, classes, scores
            boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
            classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]
            scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]

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
        except Exception:
            # model may fail on some platforms; we'll rely on fallback below
            detections = []

        # Complement with contour-based detections to improve recall in sandy bright scenes
        try:
            fallback = self._fallback_contours(frame)
            # Merge: if fallback bbox overlaps a model detection, boost score; else append
            for f in fallback:
                fx0, fy0, fx1, fy1 = f['box'][1], f['box'][0], f['box'][3], f['box'][2]
                matched = False
                for md in detections:
                    mx0, my0, mx1, my1 = md['box'][1], md['box'][0], md['box'][3], md['box'][2]
                    # compute IoU
                    ix0 = max(fx0, mx0); iy0 = max(fy0, my0)
                    ix1 = min(fx1, mx1); iy1 = min(fy1, my1)
                    iw = max(0.0, ix1 - ix0); ih = max(0.0, iy1 - iy0)
                    inter = iw * ih
                    a_f = (fx1 - fx0) * (fy1 - fy0)
                    a_m = (mx1 - mx0) * (my1 - my0)
                    union = a_f + a_m - inter if (a_f + a_m - inter) > 0 else 1e-6
                    iou = inter / union
                    if iou > 0.2:
                        # boost model score slightly when contour supports it
                        md['score'] = min(1.0, md.get('score', 0.5) + 0.15)
                        matched = True
                        break
                if not matched:
                    # append fallback detection with moderate score
                    f['score'] = max(0.35, f.get('score', 0.5))
                    detections.append(f)
        except Exception:
            pass

        # final: sort by score desc
        detections.sort(key=lambda x: x.get('score', 0.0), reverse=True)

        # Attach texture-based rockiness score to help distinguish sand glare from rocks
        try:
            thr = float(self.settings.get('det_rock_score_threshold', 0.45))
            for d in detections:
                rs = self._rock_score(frame, d['box'])
                d['rock_score'] = float(rs)
                if rs >= thr:
                    d['rock_like'] = True
                else:
                    d['rock_like'] = False
        except Exception:
            pass
        return detections
