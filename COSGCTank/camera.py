"""Camera / Frame provider utilities.

This module exposes `FrameProvider` which wraps Picamera2 (preferred) and
falls back to OpenCV `VideoCapture` when Picamera2 is not available. It
provides helpers for resizing, ROI cropping, color conversion and a
generator-style `frames()` method that yields timestamped frames suitable
for the perception pipeline.

Returned frame dict:
    {
      'frame': numpy array (H x W x 3) in RGB order,
      'timestamp': float (seconds since epoch),
      'width': int,
      'height': int
    }

The class is intentionally small and dependency-light so it can be used in
headless testing (with a video file or webcam) as well as on the Pi with the
PiCamera2 module.
"""

import time
import cv2
import numpy as np
import json
from pathlib import Path

try:
    from picamera2 import Picamera2
    _HAS_PICAM2 = True
except Exception:
    _HAS_PICAM2 = False


class FrameProvider:
    def __init__(self, width=640, height=480, fps=20, camera_index=0, roi=None):
        """Create a frame provider.

        Args:
            width (int): output frame width
            height (int): output frame height
            fps (int): target frames per second (used for throttling)
            camera_index (int): fallback OpenCV camera index or video file path
            roi (tuple|None): optional ROI as (x, y, w, h) in output coordinates
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.roi = roi
        self._last_time = 0.0

        self._use_picam = False
        if _HAS_PICAM2:
            try:
                self.picam = Picamera2()
                self.config = self.picam.create_video_configuration(
                    main={"size": (width, height), "format": "RGB888"}
                )
                self.picam.configure(self.config)
                self.picam.start()
                self._use_picam = True
            except Exception:
                self._use_picam = False

        if not self._use_picam:
            # OpenCV fallback. `camera_index` may be an int or a path to a video file.
            try:
                self.cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
            except Exception:
                self.cap = cv2.VideoCapture(camera_index)

            # Prefer MJPG for USB cams (lower USB bandwidth); harmless if unsupported.
            try:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            except Exception:
                pass
            # try to set preferred size/fps
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, float(fps))

    def _preprocess(self, frame, to_rgb=True, resize=True):
        """Apply basic preprocessing: convert to RGB and resize to target size."""
        if not isinstance(frame, np.ndarray):
            return None
        if to_rgb and frame.shape[2] == 3:
            # OpenCV gives BGR by default; Picamera2 gives RGB.
            # We only convert when using OpenCV capture.
            if not self._use_picam:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        if resize:
            frame = cv2.resize(frame, (self.width, self.height))

        return frame

    def get_frame(self):
        """Return a single preprocessed frame dict or None on failure."""
        if self._use_picam:
            raw = self.picam.capture_array()
        else:
            ret, raw = self.cap.read()
            if not ret:
                return None

        frame = self._preprocess(raw)
        ts = time.time()
        return {"frame": frame, "timestamp": ts, "width": frame.shape[1], "height": frame.shape[0]}

    def frames(self, throttle=True):
        """Generator that yields frames at approximately `self.fps`.

        Use `for data in provider.frames():` in the main loop.
        """
        interval = 1.0 / float(max(1, self.fps))
        while True:
            data = self.get_frame()
            if data is None:
                # upstream failure; yield nothing and sleep briefly
                time.sleep(0.05)
                continue

            now = time.time()
            if throttle and (now - self._last_time) < interval:
                # simple throttle to target FPS
                time.sleep(max(0, interval - (now - self._last_time)))
                now = time.time()

            self._last_time = now
            yield data

    def release(self):
        if self._use_picam:
            try:
                self.picam.stop()
            except Exception:
                pass


def load_stereo_calibration(path):
    """Load stereo calibration JSON produced by tools/calibrate_stereo.py."""
    p = Path(path)
    data = json.loads(p.read_text())

    w = int(data['image_width'])
    h = int(data['image_height'])

    def mat(key, shape):
        arr = np.array(data[key], dtype=np.float64)
        return arr.reshape(shape)

    calib = {
        'image_size': (w, h),
        'K1': mat('K1', (3, 3)),
        'D1': np.array(data['D1'], dtype=np.float64).reshape(-1, 1),
        'K2': mat('K2', (3, 3)),
        'D2': np.array(data['D2'], dtype=np.float64).reshape(-1, 1),
        'R': mat('R', (3, 3)),
        'T': np.array(data['T'], dtype=np.float64).reshape(3, 1),
        'R1': mat('R1', (3, 3)),
        'R2': mat('R2', (3, 3)),
        'P1': mat('P1', (3, 4)),
        'P2': mat('P2', (3, 4)),
        'Q': mat('Q', (4, 4)),
        'baseline_m': float(data.get('baseline_m', 0.0)),
        'rms': float(data.get('rms', 0.0)),
    }
    return calib


class StereoFrameProvider:
    """Two-device stereo provider for synchronous dual-lens modules.

    Returns dict:
      {
        'left': RGB image,
        'right': RGB image,
        'left_rect': RGB image (if calib loaded),
        'right_rect': RGB image (if calib loaded),
        'timestamp': float,
        'width': int,
        'height': int,
      }
    """

    def __init__(self, left_index=0, right_index=1, width=640, height=480, fps=30, calib_path=None):
        self.left_index = left_index
        self.right_index = right_index
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self._last_time = 0.0

        self.capL = cv2.VideoCapture(left_index, cv2.CAP_V4L2)
        self.capR = cv2.VideoCapture(right_index, cv2.CAP_V4L2)
        if not self.capL.isOpened() or not self.capR.isOpened():
            raise RuntimeError(f'Failed to open stereo cameras: {left_index}, {right_index}')

        for cap in (self.capL, self.capR):
            try:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            except Exception:
                pass
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
            cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        self.calib = None
        self._maps = None
        if calib_path:
            try:
                self.calib = load_stereo_calibration(calib_path)
                self._init_rectify_maps()
            except Exception:
                self.calib = None
                self._maps = None

    def _init_rectify_maps(self):
        if not self.calib:
            return
        w, h = self.calib['image_size']
        K1, D1, R1, P1 = self.calib['K1'], self.calib['D1'], self.calib['R1'], self.calib['P1']
        K2, D2, R2, P2 = self.calib['K2'], self.calib['D2'], self.calib['R2'], self.calib['P2']
        m1x, m1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
        m2x, m2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)
        self._maps = (m1x, m1y, m2x, m2y)

    def _read_pair(self):
        okL, rawL = self.capL.read()
        okR, rawR = self.capR.read()
        if not okL or not okR:
            return None
        # Ensure size (drivers can ignore requests)
        if rawL.shape[1] != self.width or rawL.shape[0] != self.height:
            rawL = cv2.resize(rawL, (self.width, self.height))
        if rawR.shape[1] != self.width or rawR.shape[0] != self.height:
            rawR = cv2.resize(rawR, (self.width, self.height))
        # Convert to RGB
        left = cv2.cvtColor(rawL, cv2.COLOR_BGR2RGB)
        right = cv2.cvtColor(rawR, cv2.COLOR_BGR2RGB)
        return left, right

    def frames(self, throttle=True):
        interval = 1.0 / float(max(1.0, self.fps))
        while True:
            pair = self._read_pair()
            if pair is None:
                time.sleep(0.01)
                continue
            left, right = pair
            now = time.time()
            if throttle and (now - self._last_time) < interval:
                time.sleep(max(0, interval - (now - self._last_time)))
                now = time.time()
            self._last_time = now

            out = {
                'left': left,
                'right': right,
                'timestamp': now,
                'width': int(left.shape[1]),
                'height': int(left.shape[0]),
            }
            if self._maps is not None and self.calib is not None:
                m1x, m1y, m2x, m2y = self._maps
                out['left_rect'] = cv2.remap(left, m1x, m1y, cv2.INTER_LINEAR)
                out['right_rect'] = cv2.remap(right, m2x, m2y, cv2.INTER_LINEAR)
            yield out

    def release(self):
        try:
            self.capL.release()
        except Exception:
            pass
        try:
            self.capR.release()
        except Exception:
            pass
        else:
            try:
                self.cap.release()
            except Exception:
                pass


if __name__ == "__main__":
    # Quick test: stream frames and show with overlay FPS
    provider = FrameProvider(width=640, height=480, fps=20)
    import time
    from overlay import OverlayDrawer

    drawer = OverlayDrawer()
    last = time.time()
    cnt = 0
    try:
        for data in provider.frames():
            frame = data["frame"].copy()
            ts = data["timestamp"]
            cnt += 1
            if time.time() - last >= 1.0:
                fps = cnt / (time.time() - last)
                cnt = 0
                last = time.time()
            else:
                fps = 0.0

            cv2.putText(frame, f"FPS: {fps:.1f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.imshow("frame", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        provider.release()
        cv2.destroyAllWindows()
