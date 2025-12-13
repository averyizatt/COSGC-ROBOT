"""Motor control abstraction for COSGCTank.

Provides a MotorController with a simple API and safe fallbacks when running on
non-RPi machines. The controller supports forward, reverse, left, right, stop,
adjust_left/right and a small 'stop_and_reverse' emergency maneuver.

Pins and PWM usage can be adapted for specific H-bridge or ESC hardware.
"""

import time
import logging

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except Exception:
    _HAS_GPIO = False

logging.basicConfig(level=logging.INFO)


class MotorController:
    def __init__(self, pins=None, pwm_frequency=1000):
        # default pins follow rover_server.py layout if not provided
        self.pins = pins or {"AIN1": 17, "AIN2": 27, "BIN1": 23, "BIN2": 18, "STBY": 4}
        self.pwm_frequency = pwm_frequency
        self._using_gpio = _HAS_GPIO
        self._pwms = {}

        if self._using_gpio:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for name, pin in self.pins.items():
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)

            # No hardware PWM setup here; simple digital control used.
        else:
            logging.info("RPi.GPIO not available — running in simulation mode")

    def _set_pins(self, ain1, ain2, bin1, bin2, standby=True):
        if self._using_gpio:
            GPIO.output(self.pins['AIN1'], GPIO.HIGH if ain1 else GPIO.LOW)
            GPIO.output(self.pins['AIN2'], GPIO.HIGH if ain2 else GPIO.LOW)
            GPIO.output(self.pins['BIN1'], GPIO.HIGH if bin1 else GPIO.LOW)
            GPIO.output(self.pins['BIN2'], GPIO.HIGH if bin2 else GPIO.LOW)
            GPIO.output(self.pins['STBY'], GPIO.HIGH if standby else GPIO.LOW)
        else:
            logging.info(f"Set pins AIN1={ain1} AIN2={ain2} BIN1={bin1} BIN2={bin2} STBY={standby}")

    def forward(self, speed=0.6):
        # speed currently not used with digital outputs; kept for API compatibility
        self._set_pins(1, 0, 1, 0, standby=True)

    def reverse(self, speed=0.6):
        self._set_pins(0, 1, 0, 1, standby=True)

    def turn_left(self, speed=0.5):
        # pivot left: left wheel backward, right wheel forward
        self._set_pins(0, 1, 1, 0, standby=True)

    def turn_right(self, speed=0.5):
        # pivot right: left wheel forward, right wheel backward
        self._set_pins(1, 0, 0, 1, standby=True)

    def adjust_left(self, speed=0.45):
        # gentle left correction: slow left wheel, forward right wheel
        # emulate by brief pivot then forward
        self.turn_left(speed=0.4)
        time.sleep(0.08)
        self.forward(speed=0.6)

    def adjust_right(self, speed=0.45):
        self.turn_right(speed=0.4)
        time.sleep(0.08)
        self.forward(speed=0.6)

    def stop(self):
        self._set_pins(0, 0, 0, 0, standby=False)

    def safe_stop(self):
        # immediate stop and disable standby
        self.stop()

    def stop_and_reverse(self, duration=0.5):
        # emergency escape: brief reverse then stop
        self.stop()
        time.sleep(0.05)
        self.reverse()
        time.sleep(duration)
        self.stop()

    def cleanup(self):
        if self._using_gpio:
            self.stop()
            GPIO.cleanup()
        else:
            logging.info("Motor controller cleanup (simulated)")


if __name__ == "__main__":
    # quick simulation test
    mc = MotorController()
    mc.forward()
    time.sleep(0.5)
    mc.turn_left()
    time.sleep(0.3)
    mc.adjust_right()
    time.sleep(0.4)
    mc.stop()
    mc.cleanup()
