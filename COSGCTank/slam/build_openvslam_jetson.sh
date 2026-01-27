#!/usr/bin/env bash
set -euo pipefail

# Build OpenVSLAM on Jetson Nano with performance flags
# Assumes OpenVSLAM source is in COSGCTank/slam/openvslam (adjust paths as needed)

SRC_DIR=${1:-openvslam}
BUILD_DIR=${2:-openvslam/build_jetson}

mkdir -p "$BUILD_DIR"
pushd "$BUILD_DIR" >/dev/null

export CXXFLAGS="-O3 -march=armv8-a -fopenmp"
export CFLAGS="-O3 -march=armv8-a"

cmake "../$SRC_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DUSE_PANGOLIN_VIEWER=ON \
  -DPANGOLIN_USE_EGL=ON

make -j$(nproc)

popd >/dev/null

echo "Build complete in $BUILD_DIR"
