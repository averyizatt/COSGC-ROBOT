"""Ultrasonic sensor support (HC-SR04 style) with RPi.GPIO usage and
simulation fallback.

Provides `Ultrasonic` class with a `read_distance()` method returning meters
or centimeters (configurable) or `None` if measurement failed.

Usage:
    us = Ultrasonic(trig_pin=20, echo_pin=21)
    d_cm = us.read_distance()
"""

import time
import logging

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except Exception:
    _HAS_GPIO = False

logging.basicConfig(level=logging.INFO)


class Ultrasonic:
    def __init__(self, trig_pin=20, echo_pin=21, unit='cm', timeout=0.02):
        self.trig = trig_pin
        self.echo = echo_pin
        self.unit = unit
        self.timeout = timeout
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

    def read_distance(self):
        """Return the measured distance. If unit=='cm' returns centimeters,
        else if unit=='m' returns meters. Returns None on failure."""
        if not self._using_gpio:
            # Simulation: return None to indicate not present, or a safe large distance
            # You may replace this with a deterministic simulation for tests
            return None

        # trigger a 10us pulse
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)

        start = time.time()
        timeout_time = start + self.timeout

        # wait for echo high
        while GPIO.input(self.echo) == 0 and time.time() < timeout_time:
            start = time.time()
        if time.time() >= timeout_time:
            return None

        # wait for echo low
        stop = time.time()
        timeout_time = stop + self.timeout
        while GPIO.input(self.echo) == 1 and time.time() < timeout_time:
            stop = time.time()
        if time.time() >= timeout_time:
            return None

        # elapsed time
        elapsed = stop - start
        # sound speed ~34300 cm/s -> distance = (elapsed * 34300) / 2
        dist_cm = (elapsed * 34300.0) / 2.0
        if self.unit == 'cm':
            return dist_cm
        else:
            return dist_cm / 100.0

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
            d = u.read_distance()
            print("Distance:", d)
            time.sleep(0.5)
    finally:
        u.cleanup()
