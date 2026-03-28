"""I2C power/current sensing helpers.

This module is designed to be safe on dev machines:
- If `smbus2` isn't available or the I2C bus can't be opened, reads return None.

Default implementation supports INA219-style devices (common at 0x40/0x41/0x44...).
We intentionally compute current from shunt voltage + shunt resistance so we don't
need device-specific calibration constants.

Addresses requested by the project:
- Motor current sensors: 0x40 and 0x43
- Overall power management channel: 0x41

If your board is *not* INA219 compatible, reads may fail; in that case tell me the
exact chip/board and register map and I'll adapt the driver.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

try:
    from smbus2 import SMBus

    _HAS_SMBUS = True
except Exception:
    SMBus = None  # type: ignore
    _HAS_SMBUS = False


def _twos_complement_16(v: int) -> int:
    v &= 0xFFFF
    if v & 0x8000:
        return -((~v & 0xFFFF) + 1)
    return v


@dataclass
class Ina219Settings:
    i2c_bus: int = 1
    i2c_addr: int = 0x40
    # Ohms, e.g. many INA219 breakouts use 0.1Ω. Set to your actual shunt.
    shunt_ohm: float = 0.1


class INA219:
    """Minimal INA219 reader (bus voltage + shunt voltage).

    Registers (INA219):
      0x01 shunt voltage (signed, LSB=10uV)
      0x02 bus voltage   (unsigned, bits[15:3], LSB=4mV)
    """

    _REG_SHUNT_V = 0x01
    _REG_BUS_V = 0x02

    def __init__(self, settings: Optional[Ina219Settings] = None):
        self.settings = settings or Ina219Settings()
        self._bus = None
        self._using_i2c = False

        if _HAS_SMBUS:
            try:
                self._bus = SMBus(int(self.settings.i2c_bus))
                self._using_i2c = True
            except Exception:
                self._bus = None
                self._using_i2c = False

    @property
    def available(self) -> bool:
        return bool(self._using_i2c and self._bus is not None)

    def _read_u16(self, reg: int) -> Optional[int]:
        if not self._bus:
            return None
        try:
            # INA219 uses big-endian register order.
            v = self._bus.read_word_data(int(self.settings.i2c_addr), int(reg))
            v = ((v & 0xFF) << 8) | ((v >> 8) & 0xFF)
            return int(v)
        except Exception:
            return None

    def read(self) -> Optional[Dict[str, Any]]:
        if not self.available:
            return None

        shunt_raw = self._read_u16(self._REG_SHUNT_V)
        bus_raw = self._read_u16(self._REG_BUS_V)
        if shunt_raw is None or bus_raw is None:
            return None

        shunt_uv = float(_twos_complement_16(shunt_raw)) * 10.0  # 10uV LSB
        shunt_v = shunt_uv * 1e-6
        shunt_mv = shunt_v * 1e3

        # bus voltage: bits [15:3] * 4mV
        bus_mv = float((bus_raw >> 3) & 0x1FFF) * 4.0
        bus_v = bus_mv / 1000.0

        r = float(self.settings.shunt_ohm)
        if r <= 0:
            current_a = None
        else:
            current_a = shunt_v / r

        power_w = None
        if current_a is not None:
            power_w = bus_v * current_a

        return {
            'ts': time.time(),
            'addr': int(self.settings.i2c_addr),
            'bus_v': float(bus_v),
            'shunt_mv': float(shunt_mv),
            'current_a': None if current_a is None else float(current_a),
            'power_w': None if power_w is None else float(power_w),
        }

    def close(self) -> None:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
        self._bus = None
        self._using_i2c = False


@dataclass
class PowerMonitorSettings:
    enabled: bool = False
    i2c_bus: int = 1

    motor_left_addr: int = 0x40
    motor_right_addr: int = 0x43
    system_addr: int = 0x41

    motor_left_shunt_ohm: float = 0.1
    motor_right_shunt_ohm: float = 0.1
    system_shunt_ohm: float = 0.1


class PowerMonitor:
    def __init__(self, settings: Optional[PowerMonitorSettings] = None):
        self.settings = settings or PowerMonitorSettings()

        self._left = None
        self._right = None
        self._system = None

        self._init_devices()

    def _init_devices(self) -> None:
        s = self.settings
        self._left = INA219(Ina219Settings(i2c_bus=s.i2c_bus, i2c_addr=s.motor_left_addr, shunt_ohm=s.motor_left_shunt_ohm))
        self._right = INA219(Ina219Settings(i2c_bus=s.i2c_bus, i2c_addr=s.motor_right_addr, shunt_ohm=s.motor_right_shunt_ohm))
        self._system = INA219(Ina219Settings(i2c_bus=s.i2c_bus, i2c_addr=s.system_addr, shunt_ohm=s.system_shunt_ohm))

    def update_settings(self, new_settings: PowerMonitorSettings) -> None:
        # Re-create devices if bus/addr/shunt change.
        self.settings = new_settings
        try:
            self.close()
        finally:
            self._init_devices()

    def read(self) -> Dict[str, Any]:
        s = self.settings
        out: Dict[str, Any] = {
            'enabled': bool(s.enabled),
            'ts': time.time(),
            'i2c_bus': int(s.i2c_bus),
            'motor_left': {'addr': int(s.motor_left_addr), 'ok': False},
            'motor_right': {'addr': int(s.motor_right_addr), 'ok': False},
            'system': {'addr': int(s.system_addr), 'ok': False},
            'total_motor_current_a': None,
        }

        if not s.enabled:
            return out

        left = self._left.read() if self._left is not None else None
        right = self._right.read() if self._right is not None else None
        system = self._system.read() if self._system is not None else None

        if left is not None:
            out['motor_left'] = {**left, 'ok': True}
        if right is not None:
            out['motor_right'] = {**right, 'ok': True}
        if system is not None:
            out['system'] = {**system, 'ok': True}

        try:
            la = out['motor_left'].get('current_a')
            ra = out['motor_right'].get('current_a')
            if isinstance(la, (int, float)) and isinstance(ra, (int, float)):
                out['total_motor_current_a'] = float(la + ra)
        except Exception:
            pass

        return out

    def close(self) -> None:
        for dev in (self._left, self._right, self._system):
            try:
                if dev is not None:
                    dev.close()
            except Exception:
                pass
        self._left = None
        self._right = None
        self._system = None
