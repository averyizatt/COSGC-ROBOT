#!/usr/bin/env bash
set -euo pipefail

# Jetson Nano performance tuning helpers

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root: sudo $0"
  exit 1
fi

nvpmodel -m 0
jetson_clocks

echo "Exporting CUDA/OpenCV library paths for current shell"
export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

echo "Done. Current nvpmodel:" 
cat /etc/nvpmodel.conf | head -n 20 | sed -n '1,20p' >/dev/null || true
