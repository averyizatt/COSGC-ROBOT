import numpy as np
import cv2
import os
# Robust import for resize_frame: support both root utils.py and tools/utils.py
try:
    from utils import resize_frame  # type: ignore
except Exception:
    try:
        from tools.utils import resize_frame  # type: ignore
    except Exception:
        # Last-resort inline implementation to avoid import errors
        def resize_frame(frame, size):
            return cv2.resize(frame, size).astype('uint8')
from typing import List, Dict, Any, Optional

_HAS_TRT = False
try:
    import tensorrt as trt  # type: ignore
    import pycuda.driver as cuda  # type: ignore
    import pycuda.autoinit  # type: ignore
    _HAS_TRT = True
except Exception:
    _HAS_TRT = False

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
        self.model_path = model_path
        self.input_size = input_size
        self.score_threshold = float(score_threshold)
        self.labels = labels or {}
        self.settings = settings or {}
        # region-of-interest (only consider lower portion of frame for obstacles)
        self.lower_roi_fraction = float(self.settings.get('det_lower_roi_fraction', 0.6))
        # temporal smoothing / persistence
        self.temporal_alpha = float(self.settings.get('det_temporal_alpha', 0.6))
        self.temporal_enabled = bool(self.settings.get('det_temporal_smoothing', True))
        self.min_persistence = int(self.settings.get('det_min_persistence', 2))
        self._tracks: List[Dict[str, Any]] = []
        self._next_id = 1

        # Enable OpenCV optimizations on capable platforms (Jetson/Pi)
        try:
            cv2.setUseOptimized(True)
            cv2.setNumThreads(max(1, int(self.settings.get('cv_threads', 2))))
        except Exception:
            pass

        # Optional TensorRT backend
        self._use_trt = bool(self.settings.get('use_trt', False)) and _HAS_TRT
        self._trt = None
        self._trt_context = None
        self._trt_bindings = {}
        self._trt_stream = None
        self._trt_in_name = None
        self._trt_out_names = []

        if self._use_trt:
            engine_path = str(self.settings.get('trt_engine_path', 'models/mobilenet_ssd.engine'))
            if not os.path.exists(engine_path):
                self._use_trt = False
            else:
                try:
                    logger = trt.Logger(trt.Logger.WARNING)
                    with open(engine_path, 'rb') as f:
                        runtime = trt.Runtime(logger)
                        self._trt = runtime.deserialize_cuda_engine(f.read())
                    self._trt_context = self._trt.create_execution_context()
                    self._trt_stream = cuda.Stream()
                    # Prepare bindings
                    for i in range(self._trt.num_bindings):
                        name = self._trt.get_binding_name(i)
                        dtype = trt.nptype(self._trt.get_binding_dtype(i))
                        shape = tuple(self._trt.get_binding_shape(i))
                        is_input = self._trt.binding_is_input(i)
                        size = int(np.prod(shape))
                        d_buf = cuda.mem_alloc(size * np.dtype(dtype).itemsize)
                        self._trt_bindings[name] = {
                            'index': i,
                            'dtype': dtype,
                            'shape': shape,
                            'size': size,
                            'device_buf': d_buf,
                            'is_input': is_input,
                        }
                    # Configure inputs/outputs
                    self._trt_out_names = list(self.settings.get('trt_outputs', ['boxes', 'scores', 'classes']))
                    if not all(n in self._trt_bindings for n in self._trt_out_names):
                        self._trt_out_names = [n for n, b in self._trt_bindings.items() if not b['is_input']]
                    self._trt_in_name = self.settings.get('trt_input', None)
                    if not self._trt_in_name:
                        for n, b in self._trt_bindings.items():
                            if b['is_input']:
                                self._trt_in_name = n
                                break
                except Exception:
                    self._use_trt = False

        if not self._use_trt:
            # Graceful fallback: if TFLite is unavailable, run contour-based detector only.
            if tflite is None:
                self.interpreter = None
                self.input_details = []
                self.output_details = []
            else:
                self.interpreter = tflite.Interpreter(model_path=model_path)
                self.interpreter.allocate_tensors()
                self.input_details = self.interpreter.get_input_details()
                self.output_details = self.interpreter.get_output_details()

        # Optional labels mapping from settings or models/labels.json
        try:
            import json
            labels_path = self.settings.get('labels_path', None)
            if not labels_path:
                # default path
                for p in ('models/labels.json', 'COSGC-ROBOT/COSGCTank/models/labels.json'):
                    if os.path.exists(p):
                        labels_path = p
                        break
            if labels_path and os.path.exists(labels_path):
                with open(labels_path, 'r') as f:
                    data = json.load(f)
                # accept dict of {str/int: name}
                lab = {}
                for k, v in data.items():
                    try:
                        lab[int(k)] = str(v)
                    except Exception:
                        continue
                if lab:
                    self.labels = lab
        except Exception:
            pass

    def _gray_world(self, img: np.ndarray) -> np.ndarray:
        try:
            imgf = img.astype(np.float32) + 1e-6
            r, g, b = cv2.split(imgf)
            mr, mg, mb = np.mean(r), np.mean(g), np.mean(b)
            avg = (mr + mg + mb) / 3.0
            r *= (avg / mr)
            g *= (avg / mg)
            b *= (avg / mb)
            out = cv2.merge((r, g, b))
            return np.clip(out, 0, 255).astype(np.uint8)
        except Exception:
            return img

    def _preprocess(self, frame):
        # Resize to model input and ensure uint8 (prefer CUDA resize when available)
        try:
            use_cuda = bool(self.settings.get('use_cuda_resize', True)) and hasattr(cv2, 'cuda') and cv2.cuda.getCudaEnabledDeviceCount() > 0
        except Exception:
            use_cuda = False
        if use_cuda:
            try:
                g = cv2.cuda_GpuMat()
                g.upload(frame)
                g_resized = cv2.cuda.resize(g, self.input_size)
                img = g_resized.download()
            except Exception:
                img = resize_frame(frame, self.input_size)
        else:
            img = resize_frame(frame, self.input_size)

        # Basic gray-world white balance to stabilize color under varying illumination
        img = self._gray_world(img)

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
                # ignore boxes too high in the image (likely far-away or sky)
                if (y + ch/2.0) / h < (1.0 - self.lower_roi_fraction):
                    continue
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
        if self._use_trt and self._trt and self._trt_context is not None:
            try:
                in_b = self._trt_bindings[self._trt_in_name]
                in_host = np.ascontiguousarray(inp.astype(in_b['dtype']).reshape(in_b['shape']))
                cuda.memcpy_htod_async(in_b['device_buf'], in_host, self._trt_stream)

                bindings = [None] * self._trt.num_bindings
                for name, b in self._trt_bindings.items():
                    bindings[b['index']] = int(b['device_buf'])

                self._trt_context.execute_async_v2(bindings=bindings, stream_handle=self._trt_stream.handle)

                out_arrays = {}
                for name in self._trt_out_names:
                    b = self._trt_bindings.get(name)
                    if not b:
                        continue
                    host = np.empty(b['shape'], dtype=b['dtype'])
                    cuda.memcpy_dtoh_async(host, b['device_buf'], self._trt_stream)
                    out_arrays[name] = host
                self._trt_stream.synchronize()

                keys = list(out_arrays.keys())
                if len(keys) >= 3:
                    boxes, scores, classes = out_arrays[keys[0]], out_arrays[keys[1]], out_arrays[keys[2]]
                else:
                    boxes = out_arrays.get('boxes')
                    scores = out_arrays.get('scores')
                    classes = out_arrays.get('classes')

                if boxes is None or scores is None or classes is None:
                    raise RuntimeError('TRT outputs unavailable')

                boxes = boxes[0] if boxes.ndim == 3 else boxes
                scores = scores[0] if scores.ndim > 1 else scores
                classes = classes[0] if classes.ndim > 1 else classes

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
                detections = []
        else:
            try:
                if self.interpreter is not None:
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
                else:
                    # No interpreter available; keep detections empty and rely on fallback below
                    detections = []
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

        # filter detections to lower ROI (helps ignore sky/horizon false positives)
        try:
            roi_min_y = 1.0 - self.lower_roi_fraction
            detections = [d for d in detections if ((d['box'][0] + d['box'][2]) * 0.5) >= roi_min_y]
        except Exception:
            pass

        # Apply NMS to reduce duplicates
        try:
            detections = self._nms(detections, iou_thresh=float(self.settings.get('det_nms_iou', 0.45)))
        except Exception:
            pass

        # Temporal smoothing to reduce jitter and flicker
        if self.temporal_enabled:
            try:
                detections = self._temporal_update(detections)
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

    # ---------------------- helpers ----------------------
    def _nms(self, dets: List[Dict[str, Any]], iou_thresh: float = 0.45) -> List[Dict[str, Any]]:
        if not dets:
            return dets
        # group by label/class to avoid suppressing cross-class detections
        def iou(a, b):
            ax0, ay0, ax1, ay1 = a['box'][1], a['box'][0], a['box'][3], a['box'][2]
            bx0, by0, bx1, by1 = b['box'][1], b['box'][0], b['box'][3], b['box'][2]
            ix0, iy0 = max(ax0, bx0), max(ay0, by0)
            ix1, iy1 = min(ax1, bx1), min(ay1, by1)
            iw, ih = max(0.0, ix1-ix0), max(0.0, iy1-iy0)
            inter = iw*ih
            aa = (ax1-ax0)*(ay1-ay0)
            bb = (bx1-bx0)*(by1-by0)
            return inter / max(aa+bb-inter, 1e-6)

        out: List[Dict[str, Any]] = []
        # split groups
        groups: Dict[str, List[Dict[str, Any]]] = {}
        for d in dets:
            key = f"{d.get('label', '')}:{d.get('class', 0)}"
            groups.setdefault(key, []).append(d)
        for key, arr in groups.items():
            arr = sorted(arr, key=lambda x: x.get('score', 0.0), reverse=True)
            keep = []
            while arr:
                cur = arr.pop(0)
                keep.append(cur)
                arr = [x for x in arr if iou(cur, x) < iou_thresh]
            out.extend(keep)
        return out

    def _temporal_update(self, dets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        # simple IOU-based tracker with EMA on box coords; increases persistence and adds ids
        def iou(a, b):
            ax0, ay0, ax1, ay1 = a['box'][1], a['box'][0], a['box'][3], a['box'][2]
            bx0, by0, bx1, by1 = b['box'][1], b['box'][0], b['box'][3], b['box'][2]
            ix0, iy0 = max(ax0, bx0), max(ay0, by0)
            ix1, iy1 = min(ax1, bx1), min(ay1, by1)
            iw, ih = max(0.0, ix1-ix0), max(0.0, iy1-iy0)
            inter = iw*ih
            aa = (ax1-ax0)*(ay1-ay0)
            bb = (bx1-bx0)*(by1-by0)
            return inter / max(aa+bb-inter, 1e-6)

        updated_tracks: List[Dict[str, Any]] = []
        used = set()
        for t in self._tracks:
            t['age'] += 1

        for d_idx, d in enumerate(dets):
            # match to best track with same class/label
            best_iou, best_idx = 0.0, -1
            for idx, t in enumerate(self._tracks):
                if idx in used:
                    continue
                if (t.get('class') != d.get('class')) or (t.get('label') != d.get('label')):
                    continue
                i = iou(t, d)
                if i > best_iou:
                    best_iou, best_idx = i, idx
            if best_iou > float(self.settings.get('det_track_iou', 0.3)) and best_idx >= 0:
                t = self._tracks[best_idx]
                used.add(best_idx)
                # EMA on box
                tb = np.array(t['box'], dtype=np.float32)
                db = np.array(d['box'], dtype=np.float32)
                newb = self.temporal_alpha * db + (1.0 - self.temporal_alpha) * tb
                t['box'] = newb.tolist()
                t['score'] = max(t.get('score', 0.0)*0.9, d.get('score', 0.0))
                t['hits'] += 1
                t['age'] = 0
                updated_tracks.append(t)
            else:
                # new track
                t = {
                    'box': d['box'],
                    'class': d.get('class', 0),
                    'label': d.get('label'),
                    'score': d.get('score', 0.0),
                    'hits': 1,
                    'age': 0,
                    'id': self._next_id
                }
                self._next_id += 1
                updated_tracks.append(t)

        # decay unmatched old tracks
        for idx, t in enumerate(self._tracks):
            if idx in used:
                continue
            t['age'] += 1
            if t['age'] <= 2:  # brief occlusion tolerance
                updated_tracks.append(t)

        # keep only recent tracks
        self._tracks = [t for t in updated_tracks if t['age'] <= 4]

        # emit detections for tracks that are sufficiently persistent
        out: List[Dict[str, Any]] = []
        for t in self._tracks:
            if t['hits'] >= self.min_persistence:
                out.append({k: t[k] for k in t.keys() if k in ['box','class','label','score','id']})
        # if nothing meets persistence yet, fall back to current raw detections to avoid emptiness
        if not out:
            return dets
        return out
