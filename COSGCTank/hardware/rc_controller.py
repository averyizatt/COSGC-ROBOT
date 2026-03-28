import threading
import time
import logging

try:
    from evdev import InputDevice, categorize, ecodes, list_devices  # type: ignore
    _HAS_EVDEV = True
except Exception:
    _HAS_EVDEV = False


class RCController:
    """Background reader for a Bluetooth/Xbox gamepad using evdev.

    Normalizes:
      - Steering: ABS_X in [-1, 1]
      - Throttle: RT (ABS_RZ) forward minus LT (ABS_Z) reverse → [-1, 1]
      - E‑stop: BTN_A pressed → True

    Runs a background thread to read events so the main loop can poll state.
    """

    def __init__(self, device_name_hints=(
        'Xbox Wireless Controller', 'Microsoft X-Box pad', 'Xbox 360 Controller', 'Gamepad'), reconnect_s=2.5):
        self.available = False
        self._dev = None
        self._stop = False
        self._thread = None
        self._reconnect_s = float(reconnect_s)
        self._device_name_hints = tuple(device_name_hints)
        self.device_name = None

        # Raw axis/button state
        self._axes = {
            'ABS_X': 0,     # 0..255 center ~128
            'ABS_Y': 0,
            'ABS_Z': 0,     # LT
            'ABS_RZ': 0,    # RT
        }
        self._buttons = {
            'BTN_A': 0,
        }

        # Normalized outputs
        self.steer = 0.0
        self.throttle = 0.0
        self.estop = False

        if _HAS_EVDEV:
            try:
                self._open_device()
            except Exception:
                pass
            if self._dev is not None:
                self._thread = threading.Thread(target=self._run, daemon=True)
                self._thread.start()

    def _find_candidate_device(self):
        for path in list_devices():
            try:
                dev = InputDevice(path)
                name = (dev.name or '').strip()
                if any(h.lower() in name.lower() for h in self._device_name_hints):
                    return dev
                # Fallback: accept first device with ABS axes
                caps = dev.capabilities(verbose=True)
                if any(k for k in caps.keys() if isinstance(k, tuple) and k[0] == ecodes.EV_ABS):
                    return dev
            except Exception:
                continue
        return None

    def _open_device(self):
        dev = self._find_candidate_device()
        if dev is None:
            self.available = False
            self._dev = None
            return
        try:
            # set nonblocking
            dev.fd
            try:
                dev.set_nonblocking(True)  # may not exist in some versions; thread uses read_loop anyway
            except Exception:
                pass
            self._dev = dev
            self.available = True
            self.device_name = (dev.name or '').strip()
            logging.info(f"RCController: using input device '{dev.name}' at {dev.path}")
        except Exception:
            self._dev = None
            self.available = False

    def _normalize(self):
        # Axes are typically 0..255 with center ~128
        def norm_axis(val, center=128, span=128):
            try:
                v = float(val)
            except Exception:
                v = 0.0
            return max(-1.0, min(1.0, (v - center) / span))

        # Triggers are 0..255 forward/right (RT) and 0..255 reverse/left (LT)
        rt = max(0.0, min(255.0, float(self._axes.get('ABS_RZ', 0)))) / 255.0
        lt = max(0.0, min(255.0, float(self._axes.get('ABS_Z', 0)))) / 255.0
        throttle = rt - lt  # [-1, 1]

        steer = norm_axis(self._axes.get('ABS_X', 128))
        # Deadzone
        dz = 0.08
        if abs(steer) < dz:
            steer = 0.0

        self.throttle = throttle
        self.steer = steer
        self.estop = bool(self._buttons.get('BTN_A', 0))

    def _run(self):
        while not self._stop:
            try:
                if self._dev is None:
                    time.sleep(self._reconnect_s)
                    self._open_device()
                    continue
                for ev in self._dev.read_loop():
                    if self._stop:
                        break
                    if ev.type == ecodes.EV_ABS:
                        code = ev.code
                        if code == ecodes.ABS_X:
                            self._axes['ABS_X'] = ev.value
                        elif code == ecodes.ABS_Y:
                            self._axes['ABS_Y'] = ev.value
                        elif code == ecodes.ABS_Z:
                            self._axes['ABS_Z'] = ev.value
                        elif code == ecodes.ABS_RZ:
                            self._axes['ABS_RZ'] = ev.value
                        self._normalize()
                    elif ev.type == ecodes.EV_KEY:
                        code = ev.code
                        if code == ecodes.BTN_A:
                            self._buttons['BTN_A'] = ev.value
                        self._normalize()
                # If read_loop exits, device likely disconnected
                self._dev = None
                self.available = False
            except Exception:
                # Device not ready or disconnected; attempt reconnect
                self._dev = None
                self.available = False
                time.sleep(self._reconnect_s)

    def get(self):
        """Return (throttle, steer, estop)."""
        return (float(self.throttle), float(self.steer), bool(self.estop))

    def close(self):
        self._stop = True
        try:
            if self._thread is not None:
                self._thread.join(timeout=0.5)
        except Exception:
            pass
        self._dev = None
        self.available = False
