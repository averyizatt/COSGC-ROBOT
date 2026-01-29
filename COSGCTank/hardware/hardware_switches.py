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
    """Debounced switch/button reader (active-low).

    Configuration (BCM):
    - BTN1: single momentary button (toggles pairing)
    - SPDT switch with two throws to GND:
        * RC throw → rc_pin (active selects RC mode)
        * NAV throw → nav_pin (active selects Nav mode)
    """

    def __init__(self, btn1_pin: int = 5, rc_pin: int = 16, nav_pin: int = 27, debounce_s: float = 0.03):
        self.btn1_pin = int(btn1_pin)
        self.rc_pin = int(rc_pin)
        self.nav_pin = int(nav_pin)
        self.debounce_s = float(debounce_s)
        self._last = {}
        self._state = {'btn1': False, 'rc': False, 'nav': False}
        self._last_ts = {}
        self.mode = 'NAV'
        self._available = False
        self._pair_toggle = False  # one-shot flag on BTN1 press

        self._gpio = GPIO
        if self._gpio is not None:
            try:
                self._gpio.setmode(self._gpio.BCM)
                # Internal pull-up so active-low wiring is easy
                for pin in (self.btn1_pin, self.rc_pin, self.nav_pin):
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
        for key, pin in (('btn1', self.btn1_pin), ('rc', self.rc_pin), ('nav', self.nav_pin)):
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
                    # on BTN press (active-low), set one-shot toggle flag
                    if key == 'btn1' and v:
                        self._pair_toggle = True

        # Map SPDT throws to mode
        if self._state.get('rc', False) and not self._state.get('nav', False):
            self.mode = 'RC'
        elif self._state.get('nav', False) and not self._state.get('rc', False):
            self.mode = 'NAV'
        else:
            # invalid/center: keep last mode
            self.mode = self.mode

    def consume_pair_toggle(self) -> bool:
        """Return True once when BTN1 was pressed; resets the flag."""
        if self._pair_toggle:
            self._pair_toggle = False
            return True
        return False

    def cleanup(self):
        try:
            if self._gpio is not None:
                self._gpio.cleanup()
        except Exception:
            pass
