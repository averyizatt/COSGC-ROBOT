#!/usr/bin/env bash
set -euo pipefail

# Jetson Nano (JetPack 4.6.4) setup for COSGCTank
# - Installs system OpenCV (CUDA-enabled) and GStreamer plugins
# - Installs Python dependencies (skips opencv-python on aarch64)

sudo apt update
sudo apt install -y \
  python3-opencv python3-pip \
  gstreamer1.0-tools gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  v4l-utils python3-jetson-gpio

python3 -m pip install --upgrade pip
python3 -m pip install -r "$(dirname "$0")/../requirements.txt"

echo "Adding current user to gpio group (Jetson.GPIO)"
sudo groupadd -f gpio || true
sudo usermod -aG gpio "$USER"
echo "You may need to log out/in or run 'newgrp gpio' for group changes to take effect."

echo "Jetson setup complete. Try: python3 main.py"
