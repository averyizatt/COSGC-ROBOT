#!/usr/bin/env bash
set -euo pipefail

# Fetch a default SSD MobileNet ONNX model into models/mobilenet_ssd.onnx
# Tries a few public sources. If all fail, prints instructions.

OUT="COSGC-ROBOT/COSGCTank/models/mobilenet_ssd.onnx"
mkdir -p "$(dirname "$OUT")"

fetch() {
  local url="$1"
  echo "Attempting: $url"
  if curl -fsSL "$url" -o "$OUT"; then
    echo "Downloaded ONNX to: $OUT"
    return 0
  fi
  return 1
}

# Candidate sources (may change over time)
CANDIDATES=(
  "https://huggingface.co/onnxmodelzoo/ssd-mobilenetv1/resolve/main/model.onnx"
  "https://huggingface.co/onnxmodelzoo/ssd/resolve/main/model.onnx"
  "https://dl.jsdelivr.net/gh/onnx/models@main/validated/vision/object_detection/ssd/model/ssd_mobilenet_v1_coco.onnx"
)

for u in "${CANDIDATES[@]}"; do
  if fetch "$u"; then
    echo "Success."
    exit 0
  fi
done

echo "\nAll automatic sources failed."
echo "Please download an SSD MobileNet ONNX and place it at: $OUT"
echo "Suggested source: https://huggingface.co/onnxmodelzoo"
exit 1
