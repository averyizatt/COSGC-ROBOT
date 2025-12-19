"""Motor control abstraction for COSGCTank.

Provides a MotorController with a simple API and safe fallbacks when running on
non-RPi machines. The controller supports forward, reverse, left, right, stop,
adjust_left/right and a small 'stop_and_reverse' emergency maneuver.

Pins and PWM usage can be adapted for specific H-bridge or ESC hardware.

This project commonly uses a DRV8833 dual H-bridge board.

User pinout (BCM GPIO numbers):
    - BIN1 (BN1)  -> GPIO 23
    - BIN2 (BN2)  -> GPIO 18
    - STBY/SLEEP  -> GPIO 4
    - AIN2 (AN2)  -> GPIO 27
    - AIN1 (AN1)  -> GPIO 17

For DRV8833 boards without dedicated EN/PWM pins, speed control is done by
applying software PWM on the active direction input per motor.
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
    def __init__(self, pins=None, pwm_frequency=1000, invert_left=False, invert_right=False, pwm_mode='auto'):
        # default pins follow rover_server.py layout if not provided
        # H-bridge direction: IN1/IN2 per motor; optional STBY and optional PWM pins.
        self.pins = pins or {"AIN1": 17, "AIN2": 27, "BIN1": 23, "BIN2": 18, "STBY": 4}
        self.pwm_frequency = pwm_frequency
        self._using_gpio = _HAS_GPIO
        self._pwms = {}
        self.invert_left = bool(invert_left)
        self.invert_right = bool(invert_right)
        self.pwm_mode = pwm_mode  # 'auto' | 'enable' | 'in_pins' | 'stby' | 'none'

        # For small DC motors + DRV8833 it is common to have a duty-cycle deadzone.
        # These values treat the incoming `speed` (0..1) as a throttle knob and map
        # it into a usable PWM window.
        self.min_effective_duty = 0.75  # 75% duty minimum to reliably start moving
        self.max_effective_duty = 1.00

        if self._using_gpio:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for name, pin in self.pins.items():
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)

            # PWM strategy:
            # - enable: PWMA/PWMB pins exist (typical TB6612-style)
            # - in_pins: DRV8833-style PWM on direction inputs
            # - stby: PWM on standby/sleep (global throttle)
            mode = self.pwm_mode
            if mode == 'auto':
                if 'PWMA' in self.pins or 'PWMB' in self.pins:
                    mode = 'enable'
                else:
                    # DRV8833 commonly has only IN pins; prefer IN-pin PWM.
                    mode = 'in_pins'

            if mode == 'enable':
                if 'PWMA' in self.pins:
                    self._pwms['PWMA'] = GPIO.PWM(self.pins['PWMA'], self.pwm_frequency)
                    self._pwms['PWMA'].start(0)
                if 'PWMB' in self.pins:
                    self._pwms['PWMB'] = GPIO.PWM(self.pins['PWMB'], self.pwm_frequency)
                    self._pwms['PWMB'].start(0)
            elif mode == 'stby':
                if 'STBY' in self.pins:
                    self._pwms['STBY'] = GPIO.PWM(self.pins['STBY'], self.pwm_frequency)
                    self._pwms['STBY'].start(0)
            elif mode == 'in_pins':
                # PWM on direction pins. We create PWM objects for all 4 direction pins.
                for k in ('AIN1', 'AIN2', 'BIN1', 'BIN2'):
                    if k in self.pins:
                        self._pwms[k] = GPIO.PWM(self.pins[k], self.pwm_frequency)
                        self._pwms[k].start(0)

            self.pwm_mode = mode
        else:
            logging.info("RPi.GPIO not available — running in simulation mode")

    def _clamp_speed(self, speed):
        try:
            s = float(speed)
        except Exception:
            s = 0.0
        if s < 0.0:
            s = 0.0
        if s > 1.0:
            s = 1.0
        return s

    def _map_effective_speed(self, speed):
        """Map a 0..1 'requested speed' into an effective PWM window.

        Many drivetrains won't move below ~70-80% PWM due to static friction and
        battery sag. This maps the UI speed into [min_effective_duty, max_effective_duty].
        """
        s = self._clamp_speed(speed)
        lo = float(self.min_effective_duty)
        hi = float(self.max_effective_duty)
        if hi < lo:
            lo, hi = hi, lo
        return lo + (hi - lo) * s

    def _set_speed(self, left_speed, right_speed, standby=True):
        """Apply speed via PWM if available.

        If PWMA/PWMB are present: sets duty per side.
        If only STBY PWM is present: uses max(left,right) as a global speed.
        """
        ls = self._clamp_speed(left_speed)
        rs = self._clamp_speed(right_speed)
        if not self._using_gpio:
            logging.info(f"Set speed L={ls:.2f} R={rs:.2f} (simulated)")
            return

        if 'PWMA' in self._pwms and 'PWMB' in self._pwms:
            self._pwms['PWMA'].ChangeDutyCycle(ls * 100.0)
            self._pwms['PWMB'].ChangeDutyCycle(rs * 100.0)
        elif 'STBY' in self._pwms:
            self._pwms['STBY'].ChangeDutyCycle(max(ls, rs) * 100.0)
        else:
            # no PWM configured; direction-only
            pass

        if 'STBY' in self.pins and 'STBY' not in self._pwms:
            GPIO.output(self.pins['STBY'], GPIO.HIGH if standby else GPIO.LOW)

    def _set_pins(self, ain1, ain2, bin1, bin2, standby=True):
        if self._using_gpio:
            GPIO.output(self.pins['AIN1'], GPIO.HIGH if ain1 else GPIO.LOW)
            GPIO.output(self.pins['AIN2'], GPIO.HIGH if ain2 else GPIO.LOW)
            GPIO.output(self.pins['BIN1'], GPIO.HIGH if bin1 else GPIO.LOW)
            GPIO.output(self.pins['BIN2'], GPIO.HIGH if bin2 else GPIO.LOW)
            if 'STBY' in self.pins and 'STBY' not in self._pwms:
                GPIO.output(self.pins['STBY'], GPIO.HIGH if standby else GPIO.LOW)
        else:
            logging.info(f"Set pins AIN1={ain1} AIN2={ain2} BIN1={bin1} BIN2={bin2} STBY={standby}")

    def _drive(self, left_forward, right_forward, left_speed=0.6, right_speed=0.6):
        """Drive both motors with direction + (optional) PWM speed."""
        lf = bool(left_forward)
        rf = bool(right_forward)
        if self.invert_left:
            lf = not lf
        if self.invert_right:
            rf = not rf

        # If using IN-pin PWM, keep one pin LOW and PWM the other.
        if self._using_gpio and self.pwm_mode == 'in_pins':
            ls = self._map_effective_speed(left_speed)
            rs = self._map_effective_speed(right_speed)

            # ensure standby enabled (if present)
            if 'STBY' in self.pins:
                GPIO.output(self.pins['STBY'], GPIO.HIGH)

            # Left motor
            if lf:
                # forward: AIN2 low, PWM AIN1
                GPIO.output(self.pins['AIN2'], GPIO.LOW)
                if 'AIN2' in self._pwms:
                    self._pwms['AIN2'].ChangeDutyCycle(0)
                if 'AIN1' in self._pwms:
                    self._pwms['AIN1'].ChangeDutyCycle(ls * 100.0)
                else:
                    GPIO.output(self.pins['AIN1'], GPIO.HIGH if ls > 0 else GPIO.LOW)
            else:
                # reverse: AIN1 low, PWM AIN2
                GPIO.output(self.pins['AIN1'], GPIO.LOW)
                if 'AIN1' in self._pwms:
                    self._pwms['AIN1'].ChangeDutyCycle(0)
                if 'AIN2' in self._pwms:
                    self._pwms['AIN2'].ChangeDutyCycle(ls * 100.0)
                else:
                    GPIO.output(self.pins['AIN2'], GPIO.HIGH if ls > 0 else GPIO.LOW)

            # Right motor
            if rf:
                GPIO.output(self.pins['BIN2'], GPIO.LOW)
                if 'BIN2' in self._pwms:
                    self._pwms['BIN2'].ChangeDutyCycle(0)
                if 'BIN1' in self._pwms:
                    self._pwms['BIN1'].ChangeDutyCycle(rs * 100.0)
                else:
                    GPIO.output(self.pins['BIN1'], GPIO.HIGH if rs > 0 else GPIO.LOW)
            else:
                GPIO.output(self.pins['BIN1'], GPIO.LOW)
                if 'BIN1' in self._pwms:
                    self._pwms['BIN1'].ChangeDutyCycle(0)
                if 'BIN2' in self._pwms:
                    self._pwms['BIN2'].ChangeDutyCycle(rs * 100.0)
                else:
                    GPIO.output(self.pins['BIN2'], GPIO.HIGH if rs > 0 else GPIO.LOW)
            return

        # Default: Direction pins for a typical H-bridge: forward => IN1=1 IN2=0
        self._set_pins(1 if lf else 0, 0 if lf else 1, 1 if rf else 0, 0 if rf else 1, standby=True)
        self._set_speed(left_speed, right_speed, standby=True)

    def forward(self, speed=0.6):
        sp = self._clamp_speed(speed)
        self._drive(True, True, left_speed=sp, right_speed=sp)

    def reverse(self, speed=0.6):
        sp = self._clamp_speed(speed)
        self._drive(False, False, left_speed=sp, right_speed=sp)

    def turn_left(self, speed=0.5):
        # True pivot left: left wheel reverse, right wheel forward.
        sp = self._clamp_speed(speed)
        self._drive(False, True, left_speed=sp, right_speed=sp)

    def turn_right(self, speed=0.5):
        # True pivot right: left wheel forward, right wheel reverse.
        sp = self._clamp_speed(speed)
        self._drive(True, False, left_speed=sp, right_speed=sp)

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
        # Coast stop: all low.
        self._set_pins(0, 0, 0, 0, standby=False)
        self._set_speed(0.0, 0.0, standby=False)

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
            for p in self._pwms.values():
                try:
                    p.stop()
                except Exception:
                    pass
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
