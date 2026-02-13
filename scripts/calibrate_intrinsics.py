# --------------------------------------------------------------------------
# Purpose: Calibrate each camera independently using chessboard images.
# Usage:
#   python scripts/calibrate_intrinsics.py --left_dir data/raw/session_x/left \
#       --right_dir data/raw/session_x/right --board 8 5 --square 0.030
# Outputs:
#   calibration/intrinsics/left_camera.yaml
#   calibration/intrinsics/right_camera.yaml
# Notes:
#  - Make sure square_size is in meters (e.g., 0.025 for 25 mm).
# --------------------------------------------------------------------------
from typing import List, Tuple, Optional, Union
from cv2.typing import MatLike, TermCriteria
import cv2
import numpy as np
import glob
import os
import argparse

def ensure_dir(path: Union[str, os.PathLike]) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)

def find_corners(image_paths: List[str], board_size: Tuple[int, int], criteria: TermCriteria) -> Tuple[List[MatLike], List[MatLike], List[str], Optional[Tuple[int, int]]]:
    objpoints = []
    imgpoints = []
    used_paths = []
    sample_shape = None

    objp = np.zeros((board_size[0]*board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    for p in sorted(image_paths):
        img = cv2.imread(p)
        if img is None:
            print(f"Warning: Could not read {p}. Skipping.")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(
            gray,
            board_size,
            flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
        )
        if ret:
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            objpoints.append(objp.copy())
            used_paths.append(p)
            if sample_shape is None:
                sample_shape = gray.shape[::-1]
        else:
            print(f"Chessboard not found in {p}")

    return objpoints, imgpoints, used_paths, sample_shape

def calibrate_and_save(image_dir: str, board_size: Tuple[int, int], square_size: float, out_yaml: Union[str, os.PathLike]) -> None:
    image_paths = sorted(glob.glob(os.path.join(image_dir, "*.png")) + glob.glob(os.path.join(image_dir, "*.jpg")))

    if len(image_paths) == 0:
        raise RuntimeError(f"No images found in {image_dir}")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints, imgpoints, used, image_size = find_corners(image_paths, board_size, criteria)

    if not objpoints or image_size is None:
        raise RuntimeError(f"No valid calibration images in {image_dir}")
    
    for o in objpoints:
        o *= square_size

    # OpenCV will fill these with the calculated values.
    init_K = np.eye(3, dtype=np.float64)
    init_dist = np.zeros(5, dtype=np.float64)

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints=objpoints, 
        imagePoints=imgpoints, 
        imageSize=image_size, 
        cameraMatrix=init_K, 
        distCoeffs=init_dist
    )
    
    if not ret:
        raise RuntimeError("Calibration failed")

    # Compute reprojection error
    total_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        total_error += err
    mean_error = total_error / len(objpoints)
    print(f"Mean reprojection error: {mean_error:.5f} pixels")

    # Save using OpenCV FileStorage
    ensure_dir(os.path.dirname(out_yaml))
    fs = cv2.FileStorage(out_yaml, cv2.FILE_STORAGE_WRITE)
    fs.write("K", K)
    fs.write("dist", dist)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("reprojection_error", float(mean_error))
    fs.release()
    print(f"Wrote intrinsics to {out_yaml}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibrate single camera.")
    parser.add_argument("--left_dir", type=str, required=True)
    parser.add_argument("--right_dir", type=str, required=True)
    parser.add_argument("--board", type=int, nargs=2, default=[9,6])
    parser.add_argument("--square", type=float, default=0.025)
    parser.add_argument("--out_dir", type=str, default="calibration/intrinsics")
    args = parser.parse_args()

    ensure_dir(args.out_dir)
    calibrate_and_save(args.left_dir, tuple(args.board), args.square, os.path.join(args.out_dir, "left_camera.yaml"))
    calibrate_and_save(args.right_dir, tuple(args.board), args.square, os.path.join(args.out_dir, "right_camera.yaml"))