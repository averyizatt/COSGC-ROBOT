# ST7735 TFT display support for COSGC Tank project
# Tries to use luma.lcd (SPI) with Pillow rendering. Falls back gracefully if
# the library or hardware is unavailable.

from typing import List, Optional

try:
    from luma.core.interface.serial import spi
    from luma.core.interface.gpio import gpio
    from luma.lcd.device import st7735
    from luma.core.render import canvas
    from PIL import ImageFont
    _LUMA_AVAILABLE = True
except Exception:
    # Define dummies to satisfy static analysis when libs aren't installed
    spi = None  # type: ignore
    gpio = None  # type: ignore
    st7735 = None  # type: ignore
    canvas = None  # type: ignore
    ImageFont = None  # type: ignore
    _LUMA_AVAILABLE = False


class TFTDisplay:
    """Abstraction for 80x160 ST7735 TFT display.

    Default pin assignments (BCM numbering, Jetson/RPi-compatible):
    - SPI0: SCLK (pin 23), MOSI (pin 19), MISO (pin 21), CE0 (pin 24)
    - `dc_pin` (D/C): BCM25 (physical pin 22)
    - `rst_pin` (RESET): BCM24 (physical pin 18)
    
    Backlight can be tied to 3.3V, or driven via a spare PWM GPIO.
    """

    def __init__(self, dc_pin: int = 25, rst_pin: int = 24,
                 spi_port: int = 0, spi_device: int = 0,
                 width: int = 160, height: int = 80, rotate: int = 90):
        self.width = int(width)
        self.height = int(height)
        self.rotate = int(rotate)
        self._device = None
        self._font = None
        self._available = False

        if _LUMA_AVAILABLE and spi is not None and gpio is not None and st7735 is not None:
            try:
                serial = spi(port=spi_port, device=spi_device, gpio=gpio())
                # luma.lcd expects width/height and pin numbers for dc/rst
                self._device = st7735(serial, width=self.width, height=self.height,
                                       rotate=self.rotate, rst=rst_pin, dc=dc_pin)
                if ImageFont is not None:
                    try:
                        self._font = ImageFont.load_default()
                    except Exception:
                        self._font = None
                self._available = True
            except Exception:
                self._available = False

    @property
    def available(self) -> bool:
        return bool(self._available and self._device is not None)

    def draw_lines(self, lines: List[str]) -> None:
        """Draw up to 6 lines of status text."""
        if not self.available:
            return
        try:
            if canvas is None:
                return
            with canvas(self._device) as draw:
                y = 0
                for i, line in enumerate(lines[:6]):
                    draw.text((0, y), str(line), fill=255, font=self._font)
                    y += 12
        except Exception:
            # Best-effort rendering; ignore transient IO errors
            pass

    def draw_status(self, mode: str, explore: bool, safe: bool,
                    fps: float, distance_cm: Optional[float],
                    power: Optional[dict]) -> None:
        """Render a compact status view on the TFT."""
        lines = []
        lines.append(f"Mode: {mode}  Explore: {'ON' if explore else 'OFF'}")
        lines.append(f"Safe: {'ON' if safe else 'OFF'}  FPS: {fps:.1f}")
        if isinstance(distance_cm, (int, float)):
            lines.append(f"Dist: {distance_cm:.1f} cm")
        else:
            lines.append("Dist: --")
        if isinstance(power, dict):
            try:
                ml = power.get('motor_left', {}).get('current_a')
                mr = power.get('motor_right', {}).get('current_a')
                sa = power.get('system', {}).get('current_a')
                lines.append(f"I L/R: {ml or '--'}/{mr or '--'} A")
                lines.append(f"I SYS: {sa or '--'} A")
            except Exception:
                lines.append("Power: --")
        else:
            lines.append("Power: --")
        self.draw_lines(lines)
