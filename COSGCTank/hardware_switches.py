# Hardware switch handling (Jetson/RPi-compatible GPIO)
# Supports up to 3 inputs pulled to GND with internal pull-ups.
# - BTN1 (momentary)
# - SW1 (SPDT logical mode)
# - SW2 (SPDT safety)

import time
from typing import Optional

try:
    import RPi.GPIO as _GPIO  # Prefer Raspberry Pi GPIO
    GPIO = _GPIO
    _GPIO_LIB = 'RPi.GPIO'
except Exception:
    try:
        import Jetson.GPIO as _GPIO  # Jetson fallback
        GPIO = _GPIO
        _GPIO_LIB = 'Jetson.GPIO'
    except Exception:
        GPIO = None
        _GPIO_LIB = None


class Switches:
    """Debounced switch/button reader (active-low)."""

    def __init__(self, btn1_pin: int = 23, sw1_pin: int = 22, sw2_pin: int = 27, debounce_s: float = 0.03):
        self.btn1_pin = int(btn1_pin)
        self.sw1_pin = int(sw1_pin)
        self.sw2_pin = int(sw2_pin)
        self.debounce_s = float(debounce_s)
        self._last = {}
        self._state = {'btn1': False, 'sw1': False, 'sw2': False}
        self._last_ts = {}
        self.page_index = 0
        self.mode = 'NAV'  # NAV vs EXPLORE (from sw1)
        self.safe = False   # from sw2
        self._available = False

        self._gpio = GPIO
        if self._gpio is not None:
            try:
                self._gpio.setmode(self._gpio.BCM)
                # Internal pull-up so active-low wiring is easy
                for pin in (self.btn1_pin, self.sw1_pin, self.sw2_pin):
                    self._gpio.setup(pin, self._gpio.IN, pull_up_down=self._gpio.PUD_UP)
                self._available = True
            except Exception:
                self._available = False

    @property
    def available(self) -> bool:
        return bool(self._available)

    def _read_active_low(self, pin: int) -> Optional[bool]:
        if not self.available or self._gpio is None:
            return None
        try:
            val = self._gpio.input(pin)
            return bool(val == self._gpio.LOW)
        except Exception:
            return None

    def poll(self) -> None:
        """Debounce and update state; updates mode/safe/page_index accordingly."""
        now = time.time()
        for key, pin in (('btn1', self.btn1_pin), ('sw1', self.sw1_pin), ('sw2', self.sw2_pin)):
            v = self._read_active_low(pin)
            if v is None:
                continue
            if key not in self._last:
                self._last[key] = v
                self._last_ts[key] = now
            if v != self._last[key]:
                self._last[key] = v
                self._last_ts[key] = now
            # stable?
            if (now - self._last_ts.get(key, now)) >= self.debounce_s:
                if self._state.get(key) != v:
                    self._state[key] = v
                    # handle edges for momentary
                    if key == 'btn1' and v:
                        self.page_index = (self.page_index + 1) % 3

        # Map SPDT positions to logical toggles
        self.mode = 'EXPLORE' if self._state.get('sw1', False) else 'NAV'
        self.safe = bool(self._state.get('sw2', False))

    def cleanup(self):
        try:
            if self._gpio is not None:
                self._gpio.cleanup()
        except Exception:
            pass
