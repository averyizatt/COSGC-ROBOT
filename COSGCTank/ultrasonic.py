"""Ultrasonic sensor support (HC-SR04 style) with RPi.GPIO usage and
simulation fallback.

Default wiring (BCM GPIO numbers) for this rover:
    - TRIG -> GPIO 24
    - ECHO -> GPIO 25

IMPORTANT (Raspberry Pi safety): HC-SR04 ECHO is typically 5V.
You MUST level-shift ECHO to 3.3V before connecting to a Pi GPIO.
Use a voltage divider (e.g., 1k/2k) or a proper level shifter.

Provides `Ultrasonic` class with:
    - `read_distance_cm()` returning cm or None on failure
    - `read_distance_m()` returning meters or None on failure

Usage:
    us = Ultrasonic()  # defaults to TRIG=24, ECHO=25
    d_cm = us.read_distance_cm()
"""

import time
import logging
import statistics

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except Exception:
    _HAS_GPIO = False

logging.basicConfig(level=logging.INFO)


class Ultrasonic:
    DEFAULT_TRIG_BCM = 24
    DEFAULT_ECHO_BCM = 25

    def __init__(self, trig_pin=DEFAULT_TRIG_BCM, echo_pin=DEFAULT_ECHO_BCM, timeout_s=0.03):
        self.trig = int(trig_pin)
        self.echo = int(echo_pin)
        self.timeout_s = float(timeout_s)
        self._using_gpio = _HAS_GPIO

        if self._using_gpio:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.trig, GPIO.OUT)
            GPIO.setup(self.echo, GPIO.IN)
            GPIO.output(self.trig, GPIO.LOW)
            time.sleep(0.05)
        else:
            logging.info("Ultrasonic: RPi.GPIO not available — running in simulation mode")

    def _read_once_cm(self):
        """Perform a single ping and return distance in centimeters, or None."""
        if not self._using_gpio:
            # Simulation: return None to indicate not present, or a safe large distance
            # You may replace this with a deterministic simulation for tests
            return None

        # trigger a 10us pulse
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)

        # Wait for echo to go HIGH (start)
        t0 = time.time()
        deadline = t0 + self.timeout_s
        while GPIO.input(self.echo) == 0:
            if time.time() > deadline:
                return None
        start = time.time()

        # Wait for echo to go LOW (end)
        deadline = start + self.timeout_s
        while GPIO.input(self.echo) == 1:
            if time.time() > deadline:
                return None
        stop = time.time()

        elapsed = max(0.0, stop - start)
        # sound speed ~34300 cm/s -> distance = (elapsed * 34300) / 2
        dist_cm = (elapsed * 34300.0) / 2.0
        if dist_cm <= 0:
            return None
        return dist_cm

    def read_distance_cm(self, samples=3, sample_delay_s=0.01):
        """Return distance in cm using a small median filter.

        `samples` pings are taken; failures are dropped; median is returned.
        Returns None if all samples fail.
        """
        try:
            n = int(samples)
        except Exception:
            n = 1
        n = max(1, min(9, n))
        vals = []
        for i in range(n):
            d = self._read_once_cm()
            if d is not None:
                vals.append(d)
            if i != n - 1:
                time.sleep(float(sample_delay_s))
        if not vals:
            return None
        if len(vals) == 1:
            return vals[0]
        try:
            return float(statistics.median(vals))
        except Exception:
            return float(vals[len(vals) // 2])

    def read_distance_m(self, samples=3, sample_delay_s=0.01):
        d = self.read_distance_cm(samples=samples, sample_delay_s=sample_delay_s)
        return None if d is None else (d / 100.0)

    def cleanup(self):
        if self._using_gpio:
            try:
                GPIO.cleanup([self.trig, self.echo])
            except Exception:
                pass


if __name__ == '__main__':
    u = Ultrasonic()
    try:
        while True:
            d = u.read_distance_cm(samples=5)
            print("Distance (cm):", d)
            time.sleep(0.5)
    finally:
        u.cleanup()
