#!/usr/bin/env python3
"""Stereo calibration tool (two-device stereo pair).

Reads saved image pairs and writes a calibration JSON usable by runtime.

Usage (chessboard):
  python3 tools/calibrate_stereo.py \
    --images calib_images/session1 \
    --pattern 9x6 \
    --square-size-mm 25 \
    --out calib/stereo_60mm.json

Outputs:
  - K1, D1, K2, D2
  - R, T
  - R1, R2, P1, P2, Q
  - image size, baseline, RMS

Notes:
  - --pattern is inner corners: e.g. 9x6
  - square size must match your printed board.
"""

import argparse
import glob
import json
import os

import cv2
import numpy as np


def _parse_pattern(s: str):
    s = s.lower().replace(' ', '')
    if 'x' not in s:
        raise ValueError('pattern must be like 9x6')
    a, b = s.split('x', 1)
    return (int(a), int(b))


def _jsonify_mat(m):
    return np.asarray(m).reshape(-1).tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', required=True, help='Root folder containing left/ and right/ subfolders')
    ap.add_argument('--pattern', required=True, help='Chessboard inner corners WxH, e.g. 9x6')
    ap.add_argument('--square-size-mm', type=float, required=True, help='Chessboard square size in millimeters')
    ap.add_argument('--out', default='calib/stereo_60mm.json')
    ap.add_argument('--max-pairs', type=int, default=0, help='Limit number of pairs (0 = all)')
    ap.add_argument('--show', action='store_true', help='Show detected corners for debugging')
    args = ap.parse_args()

    pattern = _parse_pattern(args.pattern)
    square_size_m = float(args.square_size_mm) / 1000.0

    left_paths = sorted(glob.glob(os.path.join(args.images, 'left', '*.png')) + glob.glob(os.path.join(args.images, 'left', '*.jpg')))
    right_paths = sorted(glob.glob(os.path.join(args.images, 'right', '*.png')) + glob.glob(os.path.join(args.images, 'right', '*.jpg')))
    if not left_paths or not right_paths:
        raise SystemExit('No images found. Expect left/*.png and right/*.png')

    # Match by filename (recommended)
    right_by_name = {os.path.basename(p): p for p in right_paths}
    pairs = []
    for lp in left_paths:
        name = os.path.basename(lp)
        rp = right_by_name.get(name)
        if rp:
            pairs.append((lp, rp))

    if args.max_pairs and args.max_pairs > 0:
        pairs = pairs[: args.max_pairs]

    if len(pairs) < 8:
        raise SystemExit(f'Need more pairs for calibration; found {len(pairs)}')

    # Prepare object points
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)
    objp *= square_size_m

    objpoints = []
    imgpointsL = []
    imgpointsR = []

    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    used = 0
    for lp, rp in pairs:
        imgL = cv2.imread(lp)
        imgR = cv2.imread(rp)
        if imgL is None or imgR is None:
            continue
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (grayL.shape[1], grayL.shape[0])

        okL, cornersL = cv2.findChessboardCorners(grayL, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        okR, cornersR = cv2.findChessboardCorners(grayR, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not okL or not okR:
            continue

        cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
        cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)
        used += 1

        if args.show:
            visL = imgL.copy(); visR = imgR.copy()
            cv2.drawChessboardCorners(visL, pattern, cornersL, okL)
            cv2.drawChessboardCorners(visR, pattern, cornersR, okR)
            cv2.imshow('L', visL)
            cv2.imshow('R', visR)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break

    if args.show:
        cv2.destroyAllWindows()

    if used < 8:
        raise SystemExit(f'Only {used} valid stereo pairs with detected corners. Capture more / improve lighting.')

    # Calibrate each camera intrinsics first
    flags = 0
    retL, K1, D1, _, _ = cv2.calibrateCamera(objpoints, imgpointsL, image_size, None, None, flags=flags)
    retR, K2, D2, _, _ = cv2.calibrateCamera(objpoints, imgpointsR, image_size, None, None, flags=flags)

    # Stereo calibrate with fixed intrinsics
    stereo_flags = cv2.CALIB_FIX_INTRINSIC
    rms, K1s, D1s, K2s, D2s, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpointsL,
        imgpointsR,
        K1,
        D1,
        K2,
        D2,
        image_size,
        criteria=criteria,
        flags=stereo_flags,
    )

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)

    baseline_m = float(np.linalg.norm(T))

    out = {
        'version': 1,
        'image_width': int(image_size[0]),
        'image_height': int(image_size[1]),
        'pattern': {'type': 'chessboard', 'inner_corners_w': int(pattern[0]), 'inner_corners_h': int(pattern[1]), 'square_size_m': float(square_size_m)},
        'rms': float(rms),
        'baseline_m': baseline_m,
        'K1': _jsonify_mat(K1),
        'D1': _jsonify_mat(D1),
        'K2': _jsonify_mat(K2),
        'D2': _jsonify_mat(D2),
        'R': _jsonify_mat(R),
        'T': _jsonify_mat(T),
        'R1': _jsonify_mat(R1),
        'R2': _jsonify_mat(R2),
        'P1': _jsonify_mat(P1),
        'P2': _jsonify_mat(P2),
        'Q': _jsonify_mat(Q),
    }

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(out, f, indent=2)

    print(f'Wrote {args.out}')
    print(f'Used pairs: {used}')
    print(f'RMS: {rms:.4f}')
    print(f'Baseline: {baseline_m*1000.0:.1f} mm')


if __name__ == '__main__':
    main()
