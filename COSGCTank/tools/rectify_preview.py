#!/usr/bin/env python3
"""Preview rectified stereo output using a saved calibration.

Usage:
  python3 tools/rectify_preview.py --left 0 --right 1 --calib calib/stereo_60mm.json

Press `q` to quit.
"""

import argparse
import cv2

from camera import StereoFrameProvider


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--left', default='0')
    ap.add_argument('--right', default='1')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30)
    ap.add_argument('--calib', required=True)
    args = ap.parse_args()

    left = int(args.left.replace('/dev/video', '')) if str(args.left).startswith('/dev/video') else int(args.left)
    right = int(args.right.replace('/dev/video', '')) if str(args.right).startswith('/dev/video') else int(args.right)

    prov = StereoFrameProvider(left_index=left, right_index=right, width=args.width, height=args.height, fps=args.fps, calib_path=args.calib)

    try:
        for d in prov.frames(throttle=True):
            L = d.get('left_rect')
            R = d.get('right_rect')
            if L is None or R is None:
                # fall back to raw
                L = d['left']
                R = d['right']
            # Convert back to BGR for display
            Lb = cv2.cvtColor(L, cv2.COLOR_RGB2BGR)
            Rb = cv2.cvtColor(R, cv2.COLOR_RGB2BGR)
            # Draw epipolar lines for sanity
            for y in range(40, Lb.shape[0], 80):
                cv2.line(Lb, (0, y), (Lb.shape[1]-1, y), (0, 255, 0), 1)
                cv2.line(Rb, (0, y), (Rb.shape[1]-1, y), (0, 255, 0), 1)
            vis = cv2.hconcat([Lb, Rb])
            cv2.imshow('Rectified (L|R)', vis)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        prov.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
