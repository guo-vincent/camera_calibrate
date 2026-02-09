# --------------------------------------------------------------------------
# Purpose: Compute rectification transforms and maps, save to disk and show example.
# Usage:
#   python scripts/rectify_images.py --intr_left calibration/intrinsics/left_camera.yaml \
#       --intr_right calibration/intrinsics/right_camera.yaml --extr calibration/extrinsics/stereo_extrinsics.yaml \
#       --left_image data/raw/left/left_000.png --right_image data/raw/right/right_000.png \
#       --out_dir calibration/rectification
# Outputs:
#   rectification_maps.yaml (contains R1,R2,P1,P2,Q and map arrays optionally)
# --------------------------------------------------------------------------
import cv2
import numpy as np
import argparse
import os
from typing import Union
from cv2.typing import MatLike

def ensure_dir(path: Union[str, os.PathLike]) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)

def load_yaml_matrix(path: Union[str, os.PathLike], name: str) -> MatLike:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open {path}")
    mat = fs.getNode(name).mat()
    fs.release()
    if mat is None:
        raise RuntimeError(f"Node '{name}' not found in {path}")
    return mat

def save_rectification(out_path: Union[str, os.PathLike], R1: MatLike, R2: MatLike, P1: MatLike, P2: MatLike, Q: MatLike) -> None:
    ensure_dir(os.path.dirname(out_path))
    fs = cv2.FileStorage(out_path, cv2.FILE_STORAGE_WRITE)
    fs.write("R1", R1)
    fs.write("R2", R2)
    fs.write("P1", P1)
    fs.write("P2", P2)
    fs.write("Q", Q)
    fs.release()
    print(f"Wrote rectification to {out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rectify stereo based on intrinsics & extrinsics.")
    parser.add_argument("--intr_left", required=True)
    parser.add_argument("--intr_right", required=True)
    parser.add_argument("--extr", required=True)
    parser.add_argument("--left_image", required=True)
    parser.add_argument("--right_image", required=True)
    parser.add_argument("--out_dir", default="calibration/rectification")
    args = parser.parse_args()

    K1 = load_yaml_matrix(args.intr_left, "K")
    dist1 = load_yaml_matrix(args.intr_left, "dist")
    K2 = load_yaml_matrix(args.intr_right, "K")
    dist2 = load_yaml_matrix(args.intr_right, "dist")
    R = load_yaml_matrix(args.extr, "R")
    T = load_yaml_matrix(args.extr, "T")

    imgL = cv2.imread(args.left_image)
    imgR = cv2.imread(args.right_image)
    if imgL is None or imgR is None:
        raise RuntimeError("Could not read example images.")

    h, w = imgL.shape[:2]

    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
        K1, dist1, K2, dist2, (w,h), R, T, alpha=0
    )

    # Compute rectification maps for fast remap() at runtime
    map1x, map1y = cv2.initUndistortRectifyMap(K1, dist1, R1, P1, (w,h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, dist2, R2, P2, (w,h), cv2.CV_32FC1)

    # Save small YAML with R1/R2/P1/P2/Q (we avoid saving big map arrays to keep file small)
    out_yaml = os.path.join(args.out_dir, "rectification.yaml")
    save_rectification(out_yaml, R1, R2, P1, P2, Q)

    # Also save the maps in compressed npz for runtime speed
    ensure_dir(args.out_dir)
    np.savez_compressed(os.path.join(args.out_dir, "rect_maps.npz"),
                        map1x=map1x, map1y=map1y, map2x=map2x, map2y=map2y)

    # Visual sanity check: show rectified pair with horizontal lines
    rectL = cv2.remap(imgL, map1x, map1y, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map2x, map2y, cv2.INTER_LINEAR)
    vis = np.hstack((rectL, rectR))
    # draw horizontal lines every 50 px
    for y in range(0, vis.shape[0], 50):
        cv2.line(vis, (0,y), (vis.shape[1], y), (0,255,0), 1)
    cv2.imshow("Rectified (left | right)", cv2.resize(vis, (min(1600, vis.shape[1]), int(vis.shape[0]*min(1600, vis.shape[1])/vis.shape[1]))))
    print("Showing rectified images. Press any key to exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()