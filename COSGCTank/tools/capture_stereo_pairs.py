#!/usr/bin/env python3
"""Capture synchronized-ish stereo pairs from two Linux video devices.

Assumes your dual-lens module appears as two V4L2 devices (e.g. /dev/video0 and /dev/video1)
that are hardware-synchronized.

Output structure:
  calib_images/<session>/left/000001.png
  calib_images/<session>/right/000001.png

Usage:
  python3 tools/capture_stereo_pairs.py --left 0 --right 1 --out calib_images/session1

Controls:
  - Space: capture a pair
  - q: quit

Notes:
  - This uses MJPG if supported.
  - Keep the board in view of both cameras.
"""

import argparse
import os
import time

import cv2


def _open_cam(dev, width, height, fps):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'Failed to open camera {dev}')
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    cap.set(cv2.CAP_PROP_FPS, float(fps))
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--left', default='0', help='Left camera index (or /dev/videoN)')
    ap.add_argument('--right', default='1', help='Right camera index (or /dev/videoN)')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30)
    ap.add_argument('--out', required=True, help='Output directory (will create left/ right/)')
    args = ap.parse_args()

    left_dev = int(args.left.replace('/dev/video', '')) if str(args.left).startswith('/dev/video') else int(args.left)
    right_dev = int(args.right.replace('/dev/video', '')) if str(args.right).startswith('/dev/video') else int(args.right)

    out_left = os.path.join(args.out, 'left')
    out_right = os.path.join(args.out, 'right')
    os.makedirs(out_left, exist_ok=True)
    os.makedirs(out_right, exist_ok=True)

    capL = _open_cam(left_dev, args.width, args.height, args.fps)
    capR = _open_cam(right_dev, args.width, args.height, args.fps)

    # Warm up
    for _ in range(10):
        capL.read(); capR.read()
        time.sleep(0.01)

    idx = 1
    print('Press SPACE to capture, q to quit.')

    try:
        while True:
            okL, frameL = capL.read()
            okR, frameR = capR.read()
            if not okL or not okR:
                time.sleep(0.01)
                continue

            preview = cv2.hconcat([frameL, frameR])
            cv2.putText(preview, f'Pairs: {idx-1}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            cv2.imshow('Stereo Capture (L|R)', preview)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            if key == 32:  # space
                fn = f'{idx:06d}.png'
                pL = os.path.join(out_left, fn)
                pR = os.path.join(out_right, fn)
                cv2.imwrite(pL, frameL)
                cv2.imwrite(pR, frameR)
                print(f'Saved {fn}')
                idx += 1
                time.sleep(0.08)
    finally:
        capL.release()
        capR.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
