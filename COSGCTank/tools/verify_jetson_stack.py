#!/usr/bin/env python3
"""
Quick Jetson Nano stack verifier.

Checks:
- Jetson detection (L4T)
- OpenCV build info and CUDA availability
- GStreamer presence
- TensorRT Python API
- PyCUDA (optional)
- TFLite runtime
- Jetson.GPIO
- evdev (for local controller)
- Presence of TensorRT engine and trtexec

Run:
  python3 COSGC-ROBOT/COSGCTank/tools/verify_jetson_stack.py
"""
import os, sys, shutil

report = []

# Jetson detection
is_jetson = False
try:
    with open('/proc/device-tree/model','r') as f:
        txt = f.read().lower()
        is_jetson = ('jetson' in txt)
except Exception:
    pass
report.append(f"Jetson detected: {'YES' if is_jetson else 'NO'}")

# OpenCV
try:
    import cv2
    ver = cv2.__version__
    info = cv2.getBuildInformation()
    use_cuda = 'Use of CUDA' in info and 'YES' in info.split('Use of CUDA')[-1][:80]
    gst = 'GStreamer' in info and 'YES' in info.split('GStreamer')[-1][:80]
    cuda_dev_count = -1
    try:
        cuda_dev_count = cv2.cuda.getCudaEnabledDeviceCount()
    except Exception:
        pass
    report.append(f"OpenCV: {ver}; CUDA: {'YES' if use_cuda else 'NO'}; cuda devices: {cuda_dev_count}; GStreamer: {'YES' if gst else 'NO'}")
except Exception as e:
    report.append(f"OpenCV: not available ({e})")

# TensorRT
try:
    import tensorrt as trt
    report.append(f"TensorRT: {trt.__version__}")
except Exception as e:
    report.append(f"TensorRT: not available ({e})")

# PyCUDA
try:
    import pycuda.driver as cuda
    import pycuda.autoinit  # noqa
    dev = cuda.Device(0)
    report.append(f"PyCUDA: OK; device 0 = {dev.name()}")
except Exception as e:
    report.append(f"PyCUDA: not available ({e})")

# TFLite runtime
try:
    import tflite_runtime.interpreter as tflite
    report.append("tflite-runtime: available")
except Exception:
    try:
        from tensorflow.lite import Interpreter as tflite  # noqa
        report.append("TensorFlow Lite: available")
    except Exception as e:
        report.append(f"TFLite: not available ({e})")

# Jetson.GPIO
try:
    import Jetson.GPIO as GPIO  # noqa
    report.append("Jetson.GPIO: available")
except Exception as e:
    report.append(f"Jetson.GPIO: not available ({e})")

# evdev
try:
    from evdev import list_devices  # noqa
    devs = list_devices()
    report.append(f"evdev: available; input devices found: {len(devs)}")
except Exception as e:
    report.append(f"evdev: not available ({e})")

# trtexec tool
trtexec_path = shutil.which('trtexec') or '/usr/src/tensorrt/bin/trtexec'
has_trtexec = os.path.exists(trtexec_path) or shutil.which('trtexec') is not None
report.append(f"trtexec: {'found' if has_trtexec else 'not found'} ({trtexec_path})")

# Engine presence
engine_candidates = [
    'COSGC-ROBOT/COSGCTank/models/mobilenet_ssd.engine',
    'models/mobilenet_ssd.engine',
]
engine_found = next((p for p in engine_candidates if os.path.exists(p)), None)
report.append(f"TRT engine: {'found at ' + engine_found if engine_found else 'not found'}")

print("\n==== Jetson Stack Report ====")
for line in report:
    print("- ", line)
print("============================\n")

# Exit nonzero if key accelerations are missing
missing = 0
if not is_jetson:
    missing += 1
if 'OpenCV:' in report[1] and 'CUDA: NO' in report[1]:
    missing += 1
if not any(line.startswith('TensorRT:') and 'not available' not in line for line in report):
    missing += 1
if not has_trtexec:
    missing += 1
sys.exit(0 if missing == 0 else 1)
