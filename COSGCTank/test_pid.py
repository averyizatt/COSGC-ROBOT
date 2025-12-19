"""Very small offline test for PID behavior.

Run:
  python3 COSGC-ROBOT/COSGCTank/test_pid.py
"""

import math


def test_pid_clamps_and_converges():
    from pid import PID

    pid = PID(kp=2.0, ki=0.5, kd=0.1, out_limit=1.0, i_limit=0.3)

    # Start with a large error; output should clamp.
    u0 = pid.update(err=2.0, now=0.0)
    assert abs(u0) <= 1.0 + 1e-9

    # Simulate error decaying to 0 over time; output magnitude should generally reduce.
    t = 0.0
    last_mag = abs(u0)
    for k in range(1, 50):
        t += 0.1
        err = 2.0 * math.exp(-t)
        u = pid.update(err=err, now=t)
        assert abs(u) <= 1.0 + 1e-9
        # not strictly monotonic, but should trend downward; allow occasional bumps
        if k % 10 == 0:
            assert abs(u) <= last_mag + 0.25
            last_mag = abs(u)

    # Reset should clear integral and history.
    pid.reset()
    u = pid.update(err=0.0, now=10.0)
    assert abs(u) < 1e-6


if __name__ == '__main__':
    test_pid_clamps_and_converges()
    print('OK')
