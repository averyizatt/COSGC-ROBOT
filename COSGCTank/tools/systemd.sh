#!/usr/bin/env bash
set -euo pipefail

# COSGCTank systemd helper: install/manage services only.
# Usage:
#   chmod +x tools/systemd.sh
#   ./tools/systemd.sh install               # write units and enable them
#   ./tools/systemd.sh install CAMERA_INDEX=0 WIDTH=640 HEIGHT=480 FPS=20 EXTRA_ARGS="--headless"
#   ./tools/systemd.sh start|stop|disable|status
#
# Environment overrides:
#   CAMERA_INDEX (default 0)
#   WIDTH (default 640)
#   HEIGHT (default 480)
#   FPS (default 20)
#   EXTRA_ARGS (default "--headless")

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENVDIR="$ROOT_DIR/.venv"
PY="$VENVDIR/bin/python"
USER_NAME="$(id -un)"

UNIT_DIR="/etc/systemd/system"
UNIT_PREFIX="cosgctank"

CAMERA_INDEX="${CAMERA_INDEX:-0}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-20}"
EXTRA_ARGS="${EXTRA_ARGS:---headless}"

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
Wants=network-online.target ${wants}

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
  if [[ ! -x "$PY" ]]; then
    echo "[!] venv not found at $VENVDIR; run tools/strap.sh install first"
    exit 1
  fi
  write_unit bridge "${ROOT_DIR}/slam/bridge.py" "" ""
  write_unit server "${ROOT_DIR}/server/rover_server.py" "" ""
  write_unit main "${ROOT_DIR}/main.py --camera-index ${CAMERA_INDEX} --width ${WIDTH} --height ${HEIGHT} --fps ${FPS} ${EXTRA_ARGS}" "${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service" "${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service"
  echo "[+] Reloading systemd"
  sudo systemctl daemon-reload
  echo "[+] Enabling units"
  sudo systemctl enable ${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-main.service
}

case "${1:-install}" in
  install)
    install_units
    ;;
  start)
    sudo systemctl start ${UNIT_PREFIX}-bridge.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-main.service
    ;;
  stop)
    sudo systemctl stop ${UNIT_PREFIX}-main.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-bridge.service || true
    ;;
  disable)
    sudo systemctl disable ${UNIT_PREFIX}-main.service ${UNIT_PREFIX}-server.service ${UNIT_PREFIX}-bridge.service || true
    ;;
  status)
    systemctl status ${UNIT_PREFIX}-bridge.service || true
    systemctl status ${UNIT_PREFIX}-server.service || true
    systemctl status ${UNIT_PREFIX}-main.service   || true
    ;;
  *)
    echo "Usage: $0 [install|start|stop|disable|status]"
    exit 1
    ;;
esac
