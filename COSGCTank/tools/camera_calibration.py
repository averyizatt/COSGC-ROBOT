"""Camera calibration helper using OpenCV.

Run this script with a connected camera or a video file to capture chessboard
images and compute camera intrinsics. It writes `camera.yaml` with the
intrinsic matrix and distortion coefficients suitable for SLAM engines.

Example:
    python3 slam/camera_calibration.py --output slam/camera.yaml --device 0
"""

import cv2
import numpy as np
import argparse
import time
import yaml

parser = argparse.ArgumentParser()
parser.add_argument('--device', default=0)
parser.add_argument('--output', default='slam/camera.yaml')
parser.add_argument('--rows', type=int, default=6)
parser.add_argument('--cols', type=int, default=9)
parser.add_argument('--square', type=float, default=0.024)  # meters
args = parser.parse_args()

CHESSBOARD = (args.cols, args.rows)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHESSBOARD[0]*CHESSBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
objp *= args.square

objpoints = []
imgpoints = []

cap = cv2.VideoCapture(int(args.device))
print('Press space to capture a frame with visible chessboard. Press q to finish.')

while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
    display = frame.copy()
    if found:
        cv2.drawChessboardCorners(display, CHESSBOARD, corners, found)
    cv2.imshow('calib', display)
    k = cv2.waitKey(1)
    if k == ord(' '):
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(f'Captured {len(objpoints)}')
        else:
            print('Chessboard not found in frame')
    elif k == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 5:
    print('Not enough calibration frames; need at least 5')
    exit(1)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print('Calibration RMS:', ret)

data = {
    'camera_matrix': mtx.tolist(),
    'dist_coeff': dist.tolist(),
    'rms': float(ret)
}

with open(args.output, 'w') as f:
    yaml.dump(data, f)

print('Wrote', args.output)
