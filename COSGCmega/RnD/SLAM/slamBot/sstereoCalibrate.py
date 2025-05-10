import cv2
import numpy as np
import glob
import pickle

# Checkerboard size (inner corners)
CHECKERBOARD = (9, 6)
square_size = 1.0  # Real-world size if known (e.g. in cm)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.indices(CHECKERBOARD).T.reshape(-1, 2) * square_size

objpoints = []
imgpointsL = []
imgpointsR = []

left_imgs = sorted(glob.glob('calib/left/*.png'))
right_imgs = sorted(glob.glob('calib/right/*.png'))

for imgL_path, imgR_path in zip(left_imgs, right_imgs):
    imgL = cv2.imread(imgL_path)
    imgR = cv2.imread(imgR_path)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, CHECKERBOARD, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, CHECKERBOARD, None)

    if retL and retR:
        objpoints.append(objp)
        cornersL2 = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
        cornersR2 = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
        imgpointsL.append(cornersL2)
        imgpointsR.append(cornersR2)

print(f"Calibrating from {len(objpoints)} image pairs...")

# Calibrate each camera individually
retL, mtxL, distL, _, _ = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
retR, mtxR, distR, _, _ = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)

# Stereo calibration
flags = cv2.CALIB_FIX_INTRINSIC
retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR,
    mtxL, distL, mtxR, distR, grayL.shape[::-1],
    criteria=criteria, flags=flags
)

# Stereo rectification
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, grayL.shape[::-1], R, T)

# Save everything
with open('stereo_params.pkl', 'wb') as f:
    pickle.dump({
        'mtxL': mtxL, 'distL': distL,
        'mtxR': mtxR, 'distR': distR,
        'R': R, 'T': T,
        'R1': R1, 'R2': R2, 'P1': P1, 'P2': P2, 'Q': Q
    }, f)

print("✅ Stereo calibration complete. Params saved to stereo_params.pkl")
