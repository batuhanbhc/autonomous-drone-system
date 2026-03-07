import cv2
import numpy as np
import glob
import json

chessboard_size = (9, 6)
square_size = 0.024

objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2) * square_size

objpoints = []
imgpoints = []

images = glob.glob('/home/batuhan/autonomous-drone-system/ros2_ws/camera_calibration/calibration_images/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size)
    if ret:
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        objpoints.append(objp)
        imgpoints.append(corners)

if len(objpoints) == 0:
    print("No chessboard corners found in any image!")
    exit()

print(f"Calibrating with {len(objpoints)} images...")

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("RMS reprojection error:", ret)
print("Camera matrix:\n", mtx)
print("Distortion:\n", dist)

calibration_data = {
    "rms_error": ret,
    "camera_matrix": mtx.tolist(),
    "distortion_coefficients": dist.tolist(),
    "image_size": [gray.shape[1], gray.shape[0]]  # W, H
}

with open("calibration.json", "w") as f:
    json.dump(calibration_data, f, indent=4)

print("Saved calibration.json")