#!/usr/bin/env bash
set -euo pipefail

# Raspberry Pi setup helper for COSGCTank
# - Installs system packages (OpenCV, GPIO, SPI, Picamera2)
# - Enables I2C and SPI via raspi-config (non-interactive)
# - Optionally verifies Python imports

echo "[+] Updating apt and installing system packages"
sudo apt update
sudo apt install -y \
  python3-pip python3-opencv python3-picamera2 \
  python3-rpi.gpio python3-spidev i2c-tools v4l-utils

echo "[+] Enabling I2C and SPI interfaces"
if command -v raspi-config >/dev/null 2>&1; then
  sudo raspi-config nonint do_i2c 0   # enable I2C
  sudo raspi-config nonint do_spi 0   # enable SPI
else
  echo "[!] raspi-config not found; please enable I2C/SPI manually"
fi

echo "[+] Upgrading pip and installing project Python deps"
python3 -m pip install --upgrade pip
python3 -m pip install -r "$(dirname "$0")/../requirements.txt"

echo "[+] Quick Python import check"
python3 - <<'PY'
ok = True
def check(mod):
    global ok
    try:
        __import__(mod)
        print(f"  - {mod}: OK")
    except Exception as e:
        ok = False
        print(f"  - {mod}: FAIL ({e})")

for m in ("cv2", "picamera2", "RPi.GPIO", "spidev", "smbus2"):
    check(m)

print("[=] Import summary:", "PASS" if ok else "SOME FAILURES")
PY

echo "[+] Done. If imports failed, ensure you reboot after enabling I2C/SPI."
echo "    You can now run: python3 COSGC-ROBOT/COSGCTank/main.py"
