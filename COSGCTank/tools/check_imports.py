#!/usr/bin/env python3
"""Quick import sanity check for COSGCTank internal modules.

Verifies that package reorganization did not break imports.
Only checks internal modules; external dependencies (GPIO/TRT/etc.) are optional.
"""

import sys
import importlib
from pathlib import Path

root = Path(__file__).resolve().parent.parent
if str(root) not in sys.path:
    sys.path.insert(0, str(root))

MODULES = [
    # hardware package
    'hardware.camera',
    'hardware.motor_control',
    'hardware.tft_display',
    'hardware.hardware_switches',
    'hardware.ultrasonic',
    'hardware.imu',
    'hardware.rc_controller',
    'hardware.power_monitor',
    # perception + nav
    'slam.detector',
    'slam.decision',
    'slam.navigator',
    'objectDetection.boundaries',
    'objectDetection.terrain',
    'objectDetection.overlay',
    'objectDetection.scale_estimator',
    'objectDetection.ground_classifier',
    'objectDetection.barrier_detector',
]

OPTIONAL = [
    'server.rover_server',  # Flask server; may require cv2
]

ok = True
for m in MODULES:
    try:
        importlib.import_module(m)
        print(f"[ok] {m}")
    except Exception as e:
        print(f"[fail] {m}: {type(e).__name__}: {e}")
        ok = False

for m in OPTIONAL:
    try:
        importlib.import_module(m)
        print(f"[ok-optional] {m}")
    except Exception as e:
        print(f"[skip-optional] {m}: {type(e).__name__}: {e}")

sys.exit(0 if ok else 1)
