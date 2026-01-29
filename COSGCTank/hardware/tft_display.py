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

# CircuitPython ST7735R fallback (works on Jetson via Blinka)
try:
    import board  # type: ignore
    import digitalio  # type: ignore
    import busio  # type: ignore
    from adafruit_rgb_display import st7735 as ada_st7735  # type: ignore
    from PIL import Image, ImageDraw
    _ADA_AVAILABLE = True
except Exception:
    board = None  # type: ignore
    digitalio = None  # type: ignore
    busio = None  # type: ignore
    ada_st7735 = None  # type: ignore
    Image = None  # type: ignore
    ImageDraw = None  # type: ignore
    _ADA_AVAILABLE = False


class TFTDisplay:
    """Abstraction for 80x160 ST7735 TFT display.

    Vendor label mapping (common on ST7735 boards):
    - SCL → SPI SCLK (BCM11, physical pin 23)
    - SDA → SPI MOSI (BCM10, physical pin 19)
    - CS  → SPI CE0/CE1 (device 0/1, CE0=BCM8 pin 24)
    - DEC → D/C (data/command) GPIO
    - RES → RESET GPIO

    Default pin assignments (BCM numbering):
    - `DEC` (D/C): BCM20 (physical pin 38)
    - `RES` (RESET): BCM21 (physical pin 40)
    - SPI0 uses system pins; select CS via `spi_device` (0=CE0, 1=CE1)

    Backlight can be tied to 3.3V, or driven via a spare PWM GPIO.
    """

    def __init__(self, dec_pin: int = 20, res_pin: int = 21,
                 spi_port: int = 0, spi_device: int = 0,
                 width: int = 160, height: int = 80, rotate: int = 90,
                 **kwargs):
        self.width = int(width)
        self.height = int(height)
        self.rotate = int(rotate)
        self._device = None
        self._font = None
        self._available = False
        self._ada_device = None
        # Backwards-compat: accept dc_pin/rst_pin kwargs
        if 'dc_pin' in kwargs and isinstance(kwargs['dc_pin'], int):
            dec_pin = int(kwargs['dc_pin'])
        if 'rst_pin' in kwargs and isinstance(kwargs['rst_pin'], int):
            res_pin = int(kwargs['rst_pin'])

        if _LUMA_AVAILABLE and spi is not None and gpio is not None and st7735 is not None:
            try:
                serial = spi(port=spi_port, device=spi_device, gpio=gpio())
                # luma.lcd expects width/height and pin numbers for dc/rst
                self._device = st7735(serial, width=self.width, height=self.height,
                                       rotate=self.rotate, rst=res_pin, dc=dec_pin)
                if ImageFont is not None:
                    try:
                        self._font = ImageFont.load_default()
                    except Exception:
                        self._font = None
                self._available = True
            except Exception:
                self._available = False
        # Fallback: Adafruit CircuitPython ST7735R via Blinka (e.g. Jetson Nano)
        if not self._available and _ADA_AVAILABLE and board is not None and digitalio is not None and busio is not None and ada_st7735 is not None:
            try:
                spi_hw = busio.SPI(board.SCLK, board.MOSI)
                cs_pin = board.CE0 if spi_device == 0 else board.CE1
                cs = digitalio.DigitalInOut(cs_pin)
                dc = digitalio.DigitalInOut(getattr(board, f"D{dec_pin}"))
                rst = digitalio.DigitalInOut(getattr(board, f"D{res_pin}"))
                self._ada_device = ada_st7735.ST7735R(spi_hw, cs=cs, dc=dc, rst=rst, rotation=self.rotate, width=self.width, height=self.height)
                # Use default font from PIL
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
        return bool(self._available and (self._device is not None or self._ada_device is not None))

    def draw_lines(self, lines: List[str]) -> None:
        """Draw up to 6 lines of status text."""
        if not self.available:
            return
        try:
            if self._device is not None and canvas is not None:
                with canvas(self._device) as draw:
                    y = 0
                    for i, line in enumerate(lines[:6]):
                        draw.text((0, y), str(line), fill=255, font=self._font)
                        y += 12
            elif self._ada_device is not None and Image is not None and ImageDraw is not None:
                img = Image.new('RGB', (self.width, self.height), color=(0, 0, 0))
                draw = ImageDraw.Draw(img)
                y = 0
                for i, line in enumerate(lines[:6]):
                    draw.text((0, y), str(line), fill=(255, 255, 255), font=self._font)
                    y += 12
                # Send image to display
                self._ada_device.image(img)
        except Exception:
            # Best-effort rendering; ignore transient IO errors
            pass

    def draw_status(self, mode: str, explore: bool, safe: bool,
                    fps: float, distance_cm: Optional[float],
                    power: Optional[dict], rc_status: Optional[str] = None,
                    rc_debug: Optional[str] = None) -> None:
        """Render a compact status view on the TFT."""
        lines = []
        lines.append(f"Mode: {mode}  Explore: {'ON' if explore else 'OFF'}")
        lines.append(f"Safe: {'ON' if safe else 'OFF'}  FPS: {fps:.1f}")
        if str(mode).upper() == 'RC' and rc_status:
            lines.append(f"RC: {rc_status}")
            if rc_debug:
                lines.append(str(rc_debug))
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
