import time


class PID:
    """Simple PID controller with dt handling and anti-windup.

    This is designed for rover steering control (output clamped to [-out_limit, out_limit]).
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, out_limit=1.0, i_limit=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = float(abs(out_limit))
        self.i_limit = float(abs(i_limit))

        self._i = 0.0
        self._prev_err = None
        self._prev_t = None

    def reset(self):
        self._i = 0.0
        self._prev_err = None
        self._prev_t = None

    def update(self, err, now=None):
        """Update PID and return control output.

        Args:
            err (float): current error
            now (float|None): timestamp; if None uses time.time()
        """
        if now is None:
            now = time.time()

        err = float(err)

        if self._prev_t is None:
            dt = 0.0
        else:
            dt = float(now) - float(self._prev_t)
            if dt < 0.0:
                dt = 0.0

        # Derivative term
        if self._prev_err is None or dt <= 0.0:
            d = 0.0
        else:
            d = (err - float(self._prev_err)) / dt

        # Integrator with clamp
        if dt > 0.0 and self.ki != 0.0:
            self._i += err * dt
            if self._i > self.i_limit:
                self._i = self.i_limit
            elif self._i < -self.i_limit:
                self._i = -self.i_limit

        u = (self.kp * err) + (self.ki * self._i) + (self.kd * d)

        # Output clamp
        if self.out_limit > 0.0:
            if u > self.out_limit:
                u = self.out_limit
            elif u < -self.out_limit:
                u = -self.out_limit

        self._prev_err = err
        self._prev_t = now
        return float(u)
