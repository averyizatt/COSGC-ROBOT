#!/usr/bin/env bash
# Example script to launch OpenVSLAM (adjust paths and flags for your build)
# Requires OpenVSLAM binaries in PATH and a config file pointing to camera intrinsics.

CONFIG=${1:-slam/openvslam_config.yaml}
VOCAB=${2:-vocab/orb_vocab.dbow2}
CAMERA_YAML=${3:-slam/camera.yaml}

# Example usage:
# ./launch_openvslam.sh slam/config.yaml vocab/orb_vocab.dbow2 slam/camera.yaml

echo "Starting OpenVSLAM with config=${CONFIG} vocab=${VOCAB} camera=${CAMERA_YAML}"

# Prefer 4 threads on Nano; override by exporting OMP_NUM_THREADS
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-4}
echo "OMP_NUM_THREADS=${OMP_NUM_THREADS}"

# This is an example command; adjust according to your OpenVSLAM build and sample apps.
# If OpenVSLAM provides an http map server plugin, configure it to POST poses to the bridge
openvslam_run \
  --config ${CONFIG} \
  --vocab ${VOCAB} \
  --camera ${CAMERA_YAML}
