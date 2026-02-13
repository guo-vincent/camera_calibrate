# --------------------------------------------------------------------------
# Purpose: Calibrate each camera independently using charuco board images.
# Usage:
#   python scripts/calibrate_intrinsics.py --left_dir data/raw/session_x/left \
#       --right_dir data/raw/session_x/right --board 9 6 --square 0.025
# Outputs:
#   calibration/intrinsics/left_camera.yaml
#   calibration/intrinsics/right_camera.yaml
# Notes:
#  - Make sure square_size is in meters (e.g., 0.025 for 25 mm).
# --------------------------------------------------------------------------
import cv2
import numpy as np
import glob
import os
import argparse
from typing import List, Tuple, Optional, Union

def ensure_dir(path: Union[str, os.PathLike]) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)

def find_charuco_corners(
    image_paths: List[str], 
    charuco_detector: cv2.aruco.CharucoDetector, 
    board: cv2.aruco.CharucoBoard
) -> Tuple[List[np.ndarray], List[np.ndarray], List[str], Optional[Tuple[int, int]]]:
    
    all_objpoints: List[np.ndarray] = []
    all_imgpoints: List[np.ndarray] = []
    used_paths: List[str] = []
    sample_shape: Optional[Tuple[int, int]] = None

    board_3d_corners = np.array(board.getChessboardCorners())

    for p in sorted(image_paths):
        img = cv2.imread(p)
        if img is None:
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if sample_shape is None:
            sample_shape = gray.shape[::-1]

        charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)

        if charuco_corners is not None and charuco_ids is not None and len(charuco_corners) >= 4:
            obj_points = board_3d_corners[charuco_ids.ravel()]
            img_points = charuco_corners
            
            all_objpoints.append(obj_points)
            all_imgpoints.append(img_points)
            used_paths.append(p)
        else:
            print(f"Insufficient corners found in {p}")

    return all_objpoints, all_imgpoints, used_paths, sample_shape

def calibrate_and_save(
    image_dir: str, 
    board: cv2.aruco.CharucoBoard,
    charuco_detector: cv2.aruco.CharucoDetector,
    out_yaml: Union[str, os.PathLike]
) -> None:
    
    image_paths = sorted(glob.glob(os.path.join(image_dir, "*.png")) + glob.glob(os.path.join(image_dir, "*.jpg")))

    if len(image_paths) == 0:
        raise RuntimeError(f"No images found in {image_dir}")

    objpoints, imgpoints, used, image_size = find_charuco_corners(image_paths, charuco_detector, board)

    if not objpoints or image_size is None:
        raise RuntimeError(f"No valid calibration images in {image_dir}")

    camera_matrix = np.array([], dtype=np.float32)
    dist_coeffs = np.array([], dtype=np.float32)

    print(f"\nCalibrating using {len(used)} images from {image_dir}...")
    
    # Standard calibration
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objectPoints=objpoints, 
        imagePoints=imgpoints, 
        imageSize=image_size, 
        cameraMatrix=camera_matrix, 
        distCoeffs=dist_coeffs
    )
    
    if not ret:
        raise RuntimeError("Calibration failed")

    # Compute reprojection error
    total_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        total_error += err
    mean_error = total_error / len(objpoints)
    print(f"Mean reprojection error: {mean_error:.5f} pixels")

    # Save results
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
    parser = argparse.ArgumentParser(description="Calibrate camera with ChArUco.")
    parser.add_argument("--left_dir", type=str, required=True)
    parser.add_argument("--right_dir", type=str, required=True)
    parser.add_argument("--board", type=int, nargs=2, default=[9,6])
    parser.add_argument("--square", type=float, default=0.025)
    parser.add_argument("--marker", type=float, default=0.015)
    parser.add_argument("--out_dir", type=str, default="calibration/intrinsics")
    args = parser.parse_args()

    ensure_dir(args.out_dir)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((args.board[0], args.board[1]), args.square, args.marker, aruco_dict)
    charuco_detector = cv2.aruco.CharucoDetector(board)

    calibrate_and_save(args.left_dir, board, charuco_detector, os.path.join(args.out_dir, "left_camera.yaml"))
    calibrate_and_save(args.right_dir, board, charuco_detector, os.path.join(args.out_dir, "right_camera.yaml"))