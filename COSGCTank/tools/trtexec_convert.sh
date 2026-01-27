#!/usr/bin/env bash
set -euo pipefail

# Convert an ONNX SSD model to TensorRT engine on Jetson Nano
# Usage: ./trtexec_convert.sh path/to/model.onnx [output.engine]

ONNX=${1:-models/mobilenet_ssd.onnx}
ENGINE=${2:-models/mobilenet_ssd.engine}

if [[ ! -f "$ONNX" ]]; then
  echo "ONNX model not found: $ONNX"
  exit 1
fi

# Use trtexec to build a TensorRT engine (FP16 for speed on Nano)
trtexec --onnx="$ONNX" --saveEngine="$ENGINE" --explicitBatch --fp16 --workspace=1024

echo "Saved TensorRT engine to: $ENGINE"
