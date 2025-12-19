#!/usr/bin/env python3
"""USB webcam probe + preview.

Usage:
  python3 tools/test_usb_cam.py --device 0
  python3 tools/test_usb_cam.py --device /dev/video2

This script forces common capture settings (V4L2 + MJPG) and prints the
negotiated width/height/fps. Press `q` to quit.
"""

import argparse
import os
import time

import cv2


def _as_device_index(device: str):
    device = str(device).strip()
    if device.startswith('/dev/video'):
        try:
            return int(device.replace('/dev/video', ''))
        except Exception:
            return device
    try:
        return int(device)
    except Exception:
        return device


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--device', default='0', help='Camera index (0) or /dev/videoN')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30)
    args = ap.parse_args()

    dev = _as_device_index(args.device)
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'Failed to open camera: {args.device}')

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))
    cap.set(cv2.CAP_PROP_FPS, float(args.fps))

    # Give driver a moment.
    time.sleep(0.1)

    w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = ''.join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])
    print(f'Opened {args.device}: {w:.0f}x{h:.0f} @ {fps:.1f} FPS, FOURCC={fourcc_str!r}')

    last = time.time()
    frames = 0
    fps_est = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue
            frames += 1
            now = time.time()
            dt = now - last
            if dt >= 1.0:
                fps_est = frames / dt
                frames = 0
                last = now

            cv2.putText(frame, f'FPS~{fps_est:.1f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            cv2.imshow('USB Cam', frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
