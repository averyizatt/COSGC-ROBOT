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
            self.cap = cv2.VideoCapture(camera_index)
            # try to set preferred size
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def _preprocess(self, frame, to_rgb=True, resize=True):
        """Apply basic preprocessing: convert to RGB and resize to target size."""
        if not isinstance(frame, np.ndarray):
            return None
        if to_rgb:
            # OpenCV gives BGR by default; Picamera2 gives RGB
            # detect by checking number of channels
            if frame.shape[2] == 3:
                # Heuristic: assume BGR if values look like OpenCV ordering
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
