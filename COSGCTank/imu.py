"""IMU support for GY-521 (MPU-6050) over I2C.

Wiring (Raspberry Pi, BCM / physical):
  - SDA: GPIO 2 (pin 3)
  - SCL: GPIO 3 (pin 5)
  - VCC: 3.3V (pin 1 or 17)
  - GND: any GND (pin 6)
  - AD0: LOW => addr 0x68 (default), HIGH => addr 0x69
  - INT: optional (not required; we poll)

This module is written to be safe on non-Pi/dev machines:
- If `smbus2` is unavailable or I2C cannot be opened, it falls back to a
  simulation mode that returns None.

Outputs:
- accel_mps2: dict with ax, ay, az
- gyro_rps: dict with gx, gy, gz
- temperature_c
- orientation: roll/pitch estimate in radians (simple complementary filter)

Notes:
- MPU-6050 outputs are noisy; for autonomy, treat this as a helpful signal for
  incline/roughness/smoothing, not an absolute heading.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any


try:
    from smbus2 import SMBus

    _HAS_SMBUS = True
except Exception:
    SMBus = None  # type: ignore
    _HAS_SMBUS = False


# MPU-6050 registers
_REG_PWR_MGMT_1 = 0x6B
_REG_SMPLRT_DIV = 0x19
_REG_CONFIG = 0x1A
_REG_GYRO_CONFIG = 0x1B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B


def _twos_complement_16(msb: int, lsb: int) -> int:
    v = (msb << 8) | lsb
    if v & 0x8000:
        v = -((~v & 0xFFFF) + 1)
    return v


@dataclass
class ImuSettings:
    i2c_bus: int = 1
    i2c_addr: int = 0x68
    # Complementary filter alpha: 0..1 (higher => trust gyro more)
    alpha: float = 0.98
    # Low-pass filter config (MPU CONFIG register); 0..6 typical
    dlpf_cfg: int = 3
    # Sample rate divider (SMPLRT_DIV). SampleRate = GyroOutputRate/(1+div)
    # With DLPF enabled, GyroOutputRate=1kHz.
    sample_div: int = 4
    # Calibration window when requested
    gyro_calib_seconds: float = 1.0


class MPU6050:
    def __init__(self, settings: Optional[ImuSettings] = None):
        self.settings = settings or ImuSettings()
        self._bus = None
        self._using_i2c = False

        self._gyro_bias = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0}
        self._roll = 0.0
        self._pitch = 0.0
        self._last_ts = None

        if _HAS_SMBUS:
            try:
                self._bus = SMBus(self.settings.i2c_bus)
                self._using_i2c = True
                self._init_device()
            except Exception:
                self._bus = None
                self._using_i2c = False

    def _write_byte(self, reg: int, val: int) -> None:
        if not self._bus:
            return
        self._bus.write_byte_data(self.settings.i2c_addr, reg, val & 0xFF)

    def _read_block(self, reg: int, length: int) -> Optional[bytes]:
        if not self._bus:
            return None
        try:
            data = self._bus.read_i2c_block_data(self.settings.i2c_addr, reg, length)
            return bytes(data)
        except Exception:
            return None

    def _init_device(self) -> None:
        # Wake up
        self._write_byte(_REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)

        # DLPF config
        dlpf = int(self.settings.dlpf_cfg)
        dlpf = max(0, min(6, dlpf))
        self._write_byte(_REG_CONFIG, dlpf)

        # Sample rate divider
        div = int(self.settings.sample_div)
        div = max(0, min(255, div))
        self._write_byte(_REG_SMPLRT_DIV, div)

        # Gyro full scale: +/- 250 deg/s (00)
        self._write_byte(_REG_GYRO_CONFIG, 0x00)

        # Accel full scale: +/- 2g (00)
        self._write_byte(_REG_ACCEL_CONFIG, 0x00)

    @property
    def available(self) -> bool:
        return bool(self._using_i2c and self._bus is not None)

    def calibrate_gyro_bias(self, seconds: Optional[float] = None) -> bool:
        """Estimate stationary gyro bias over a short window."""
        if not self.available:
            return False
        dur = self.settings.gyro_calib_seconds if seconds is None else float(seconds)
        dur = max(0.2, min(10.0, dur))

        samples = []
        t_end = time.time() + dur
        while time.time() < t_end:
            r = self.read_raw()
            if r is not None:
                samples.append(r['gyro_rps'])
            time.sleep(0.01)

        if not samples:
            return False
        self._gyro_bias = {
            'gx': sum(s['gx'] for s in samples) / len(samples),
            'gy': sum(s['gy'] for s in samples) / len(samples),
            'gz': sum(s['gz'] for s in samples) / len(samples),
        }
        return True

    def read_raw(self) -> Optional[Dict[str, Any]]:
        """Read accel/gyro/temp with basic unit conversion.

        Returns None on failure.
        """
        if not self.available:
            return None
        data = self._read_block(_REG_ACCEL_XOUT_H, 14)
        if not data or len(data) != 14:
            return None

        ax = _twos_complement_16(data[0], data[1])
        ay = _twos_complement_16(data[2], data[3])
        az = _twos_complement_16(data[4], data[5])
        temp = _twos_complement_16(data[6], data[7])
        gx = _twos_complement_16(data[8], data[9])
        gy = _twos_complement_16(data[10], data[11])
        gz = _twos_complement_16(data[12], data[13])

        # Sensitivities for FS=+/-2g and +/-250deg/s
        # accel: 16384 LSB/g, gyro: 131 LSB/(deg/s)
        g = 9.80665
        accel_mps2 = {
            'ax': (ax / 16384.0) * g,
            'ay': (ay / 16384.0) * g,
            'az': (az / 16384.0) * g,
        }
        gyro_dps = {
            'gx': gx / 131.0,
            'gy': gy / 131.0,
            'gz': gz / 131.0,
        }
        gyro_rps = {
            'gx': math.radians(gyro_dps['gx']),
            'gy': math.radians(gyro_dps['gy']),
            'gz': math.radians(gyro_dps['gz']),
        }
        temperature_c = (temp / 340.0) + 36.53

        return {
            'accel_mps2': accel_mps2,
            'gyro_rps': gyro_rps,
            'temperature_c': temperature_c,
            'ts': time.time(),
        }

    def read(self) -> Optional[Dict[str, Any]]:
        """Read IMU and return fused roll/pitch estimate.

        Returns dict with accel/gyro/temp and orientation fields, or None.
        """
        raw = self.read_raw()
        if raw is None:
            return None

        ts = float(raw['ts'])
        if self._last_ts is None:
            dt = 0.0
        else:
            dt = max(0.0, min(0.2, ts - self._last_ts))
        self._last_ts = ts

        ax = raw['accel_mps2']['ax']
        ay = raw['accel_mps2']['ay']
        az = raw['accel_mps2']['az']

        # Roll/pitch from accelerometer (assuming gravity dominates)
        roll_acc = math.atan2(ay, az if abs(az) > 1e-6 else 1e-6)
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az) + 1e-6)

        gx = raw['gyro_rps']['gx'] - float(self._gyro_bias['gx'])
        gy = raw['gyro_rps']['gy'] - float(self._gyro_bias['gy'])
        # gz bias tracked but yaw estimate not provided here

        alpha = float(self.settings.alpha)
        alpha = max(0.0, min(0.999, alpha))

        if dt > 0:
            self._roll = alpha * (self._roll + gx * dt) + (1.0 - alpha) * roll_acc
            self._pitch = alpha * (self._pitch + gy * dt) + (1.0 - alpha) * pitch_acc
        else:
            self._roll = roll_acc
            self._pitch = pitch_acc

        out = {
            'accel_mps2': raw['accel_mps2'],
            'gyro_rps': raw['gyro_rps'],
            'temperature_c': raw['temperature_c'],
            'orientation': {
                'roll_rad': self._roll,
                'pitch_rad': self._pitch,
            },
            'bias': self._gyro_bias,
        }
        return out

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
        self._bus = None
        self._using_i2c = False


if __name__ == '__main__':
    imu = MPU6050()
    print('IMU available:', imu.available)
    if imu.available:
        imu.calibrate_gyro_bias(seconds=1.0)
        while True:
            print(imu.read())
            time.sleep(0.1)
