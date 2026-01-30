#!/usr/bin/env bash
set -euo pipefail

# COSGCTank bootstrap script: prepare venv, install deps, and install systemd units.
# Usage:
#   chmod +x tools/strap.sh
#   ./tools/strap.sh install    # create venv, install deps, install+enable units
#   ./tools/strap.sh enable     # enable units (after manual edits)
#   ./tools/strap.sh disable    # disable units
#   ./tools/strap.sh status     # show unit status
#   ./tools/strap.sh start|stop # control units
#
# Overrides (pass as KEY=VALUE after the command, or export env):
#   CAMERA_INDEX (default 0)
#   WIDTH (default 640)
#   HEIGHT (default 480)
#   FPS (default 20)
#   EXTRA_ARGS (default "--headless")
# Examples:
#   ./tools/strap.sh install CAMERA_INDEX=0 WIDTH=640 HEIGHT=480 FPS=20 EXTRA_ARGS="--headless"
#   CAMERA_INDEX=1 EXTRA_ARGS="--headless --video /path.mp4" ./tools/strap.sh enable
#
# Notes:
# - Requires sudo for systemd installation (writes to /etc/systemd/system).
# - Detects repo root from this script's location.

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENVDIR="$ROOT_DIR/.venv"
PY="$VENVDIR/bin/python"
PIP="$VENVDIR/bin/pip"
USER_NAME="$(id -un)"

UNIT_DIR="/etc/systemd/system"
UNIT_PREFIX="cosgctank"

# Defaults (can be overridden via args/env)
CAMERA_INDEX="${CAMERA_INDEX:-0}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-20}"
EXTRA_ARGS="${EXTRA_ARGS:---headless}"

# Parse KEY=VALUE overrides from CLI args
for arg in "$@"; do
  case "$arg" in
    CAMERA_INDEX=*) CAMERA_INDEX="${arg#*=}" ;;
    WIDTH=*) WIDTH="${arg#*=}" ;;
    HEIGHT=*) HEIGHT="${arg#*=}" ;;
    FPS=*) FPS="${arg#*=}" ;;
    EXTRA_ARGS=*) EXTRA_ARGS="${arg#*=}" ;;
  esac
done

function ensure_venv() {
  if [[ ! -x "$PY" ]]; then
    echo "[+] Creating venv at $VENVDIR (system site-packages)"
    python3 -m venv --system-site-packages "$VENVDIR"
  fi
  echo "[+] Pinning pip/setuptools/wheel for Python 3.6 compatibility"
  "$PIP" install --upgrade 'pip==21.3.1' 'setuptools==58.3.0' 'wheel==0.37.1' 'setuptools_scm==6.4.2' || true
  # Ensure system SPI/GPIO are available to Python (avoid pip build failures on Jetson Python 3.6)
  echo "[+] Installing system packages for SPI/GPIO"
  sudo apt update -y || true
  sudo apt install -y python3-spidev python3-jetson-gpio || true
  echo "[+] Installing OpenCV + NumPy from apt"
  sudo apt install -y python3-opencv python3-numpy || true
  if [[ -f "$ROOT_DIR/requirements.txt" ]]; then
    echo "[+] Installing requirements.txt"
    "$PIP" install -r "$ROOT_DIR/requirements.txt" || true
  fi
  echo "[+] Installing extra packages (Flask, Pillow, luma, evdev)"
  "$PIP" install flask pillow luma.lcd luma.core evdev || true
  echo "[+] Optional: tflite-runtime"
  "$PIP" install tflite-runtime || true
  echo "[+] Verifying cv2 import"
  if ! "$PY" - <<'PY'
import cv2, numpy
print('cv2 OK', cv2.__version__)
print('numpy OK', numpy.__version__)
PY
  then
    echo "[!] OpenCV import failed; ensure python3-opencv is installed and venv uses --system-site-packages"
    exit 1
  fi
}

function write_unit() {
  local name="$1"; shift
  local exec="$1"; shift
  local after="$1"; shift
  local wants="$1"; shift
  local unit_path="$UNIT_DIR/${UNIT_PREFIX}-${name}.service"
  echo "[+] Writing unit $unit_path"
  sudo tee "$unit_path" >/dev/null <<UNIT
[Unit]
Description=COSGCTank ${name}
After=network-online.target ${after}
Wants=${wants}

[Service]
Type=simple
User=${USER_NAME}
WorkingDirectory=${ROOT_DIR}
Environment=PYTHONUNBUFFERED=1
ExecStart=${PY} ${exec}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
}

function install_units() {
  write_unit bridge "${ROOT_DIR}/slam/bridge.py" "" ""
  write_unit server "${ROOT_DIR}/server/rover_server.py" "" ""
  write_unit main "${ROOT_DIR}/main.py --camera-index ${CAMERA_INDEX} --width ${WIDTH} --height ${HEIGHT} --fps ${FPS} ${EXTRA_ARGS}" "${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service" "${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service"

  echo "[+] Reloading systemd"
  sudo systemctl daemon-reload
  echo "[+] Enabling units"
  sudo systemctl enable ${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-main.service
}

function start_units() {
  echo "[+] Starting units"
  sudo systemctl start ${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-main.service
}

function stop_units() {
  echo "[+] Stopping units"
  sudo systemctl stop ${UNIT_PREFIX}-main.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-bridge.service || true
}

function disable_units() {
  echo "[+] Disabling units"
  sudo systemctl disable ${UNIT_PREFIX}-main.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-bridge.service || true
}

function status_units() {
  systemctl status ${UNIT_PREFIX}-bridge.service || true
  systemctl status ${UNIT_PREFIX}-server.service || true
  systemctl status ${UNIT_PREFIX}-main.service   || true
}

case "${1:-install}" in
  install)
    ensure_venv
    install_units
    start_units
    ;;
  enable)
    install_units
    ;;
  disable)
    disable_units
    ;;
  start)
    start_units
    ;;
  stop)
    stop_units
    ;;
  status)
    status_units
    ;;
  *)
    echo "Usage: $0 [install|enable|disable|start|stop|status]"
    exit 1
    ;;
esac
