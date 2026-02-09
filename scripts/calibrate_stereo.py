# --------------------------------------------------------------------------
# Purpose: Perform stereo calibration using intrinsics and matched image pairs.
# Usage:
#   python scripts/calibrate_stereo.py --left_dir data/raw/left --right_dir data/raw/right \
#       --intr calibration/intrinsics/left_camera.yaml calibration/intrinsics/right_camera.yaml \
#       --board 9 6 --square 0.025 --out calibration/extrinsics/stereo_extrinsics.yaml
# Outputs:
#   calibration/extrinsics/stereo_extrinsics.yaml
# --------------------------------------------------------------------------

import cv2
import numpy as np
import os
import glob
import argparse
import re
from typing import Dict, List, Tuple, Union
from cv2.typing import MatLike

_DIGIT_RE = re.compile(r'(\d+)(?=\.[^.]+$)')  # captures last contiguous digits before the extension

def _extract_index(path: str) -> int:
    """Extract the last numeric group in the filename. Raise if not found."""
    name = os.path.basename(path)
    m = _DIGIT_RE.search(name)
    if not m:
        raise ValueError(f"Could not extract numeric index from filename '{name}'. "
                         f"Expected trailing digits like 'left_001.png'.")
    return int(m.group(1))

def _build_index_map(paths: List[str]) -> Dict[int, str]:
    """Return dict mapping extracted integer index -> path. If duplicate indices exist, last wins."""
    idx_map = {}
    for p in paths:
        try:
            idx = _extract_index(p)
        except ValueError:
            # Skip files without numeric suffix
            continue
        idx_map[idx] = p
    return idx_map

def find_matched_pairs_by_index(left_dir: str, right_dir: str) -> List[Tuple[str, str]]:
    """Find matched (left_path, right_path) pairs by numeric suffix intersection."""
    left_paths = sorted(glob.glob(os.path.join(left_dir, "*.*")))
    right_paths = sorted(glob.glob(os.path.join(right_dir, "*.*")))

    left_map = _build_index_map(left_paths)
    right_map = _build_index_map(right_paths)

    common_idxs = sorted(set(left_map.keys()).intersection(right_map.keys()))
    if len(common_idxs) == 0:
        raise RuntimeError("No matching file indices found between left and right directories. "
                           "Check naming convention and files.")

    # Log unmatched indices
    left_only = sorted(set(left_map.keys()) - set(right_map.keys()))
    right_only = sorted(set(right_map.keys()) - set(left_map.keys()))
    if left_only:
        print(f"Warning: left-only indices (will be ignored): {left_only[:10]}{'...' if len(left_only)>10 else ''}")
    if right_only:
        print(f"Warning: right-only indices (will be ignored): {right_only[:10]}{'...' if len(right_only)>10 else ''}")

    pairs = [(left_map[i], right_map[i]) for i in common_idxs]
    return pairs

def load_yaml_matrix(path: Union[str, os.PathLike], name: str) -> MatLike:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open {path}")
    mat = fs.getNode(name).mat()
    fs.release()
    if mat is None:
        raise RuntimeError(f"Node '{name}' not found in {path}")
    return mat

def save_stereo_extrinsics(out_path: Union[str, os.PathLike], R: MatLike, T: MatLike, E: MatLike, F: MatLike, stereo_size: Tuple[int, int]) -> None:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fs = cv2.FileStorage(out_path, cv2.FILE_STORAGE_WRITE)
    fs.write("R", R)
    fs.write("T", T)
    fs.write("E", E)
    fs.write("F", F)
    fs.write("image_width", int(stereo_size[0]))
    fs.write("image_height", int(stereo_size[1]))
    fs.release()
    print(f"Wrote stereo extrinsics to {out_path}")

def find_matched_corners_from_pairs(pairs: List[Tuple[str, str]], board_size: Tuple[int, int]) -> Tuple[List[MatLike], List[MatLike], List[MatLike], List[Tuple[str, str]], Tuple[int, int]]:
    objpoints = []
    imgpointsL = []
    imgpointsR = []
    objp = np.zeros((board_size[0]*board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    used_pairs = []
    sample_shape = None
    for l, r in pairs:
        imgL = cv2.imread(l)
        imgR = cv2.imread(r)
        if imgL is None or imgR is None:
            print(f"Warning: Could not read pair {l} / {r}; skipping.")
            continue
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
        retL, cornersL = cv2.findChessboardCorners(grayL, board_size, None)
        retR, cornersR = cv2.findChessboardCorners(grayR, board_size, None)
        if retL and retR:
            cornersL = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
            cornersR = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
            imgpointsL.append(cornersL)
            imgpointsR.append(cornersR)
            objpoints.append(objp.copy())
            used_pairs.append((l, r))
            if sample_shape is None:
                sample_shape = grayL.shape[::-1]
        else:
            print(f"Skipping pair (chessboard not found in both): {os.path.basename(l)} / {os.path.basename(r)}")
    if sample_shape is None:
        raise RuntimeError("No valid chessboard corners found in any image pair.")
    return objpoints, imgpointsL, imgpointsR, used_pairs, sample_shape

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stereo calibration from matched image pairs (robust pairing).")
    parser.add_argument("--left_dir", required=True)
    parser.add_argument("--right_dir", required=True)
    parser.add_argument("--intr", nargs=2, required=True, help="Left and right intrinsics YAMLs")
    parser.add_argument("--board", type=int, nargs=2, default=[9,6])
    parser.add_argument("--square", type=float, default=0.025)
    parser.add_argument("--out", type=str, default="calibration/extrinsics/stereo_extrinsics.yaml")
    args = parser.parse_args()

    # Load precomputed intrinsics
    K1 = load_yaml_matrix(args.intr[0], "K")
    dist1 = load_yaml_matrix(args.intr[0], "dist")
    K2 = load_yaml_matrix(args.intr[1], "K")
    dist2 = load_yaml_matrix(args.intr[1], "dist")

    # Build matched pairs by numeric index in filename
    pairs = find_matched_pairs_by_index(args.left_dir, args.right_dir)
    print(f"Found {len(pairs)} candidate pairs (after index intersection).")

    objpoints, imgpointsL, imgpointsR, used_pairs, (w, h) = find_matched_corners_from_pairs(pairs, tuple(args.board))
    if len(objpoints) < 6:
        raise RuntimeError(f"Not enough valid stereo pairs for stereo calibration (found {len(objpoints)}). Need >= 6.")

    # scale object points to meters
    for o in objpoints:
        o *= args.square

    # Stereo calibrate: keep intrinsics fixed (we trust the per-camera calibration)
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)

    retval, K1_new, dist1_new, K2_new, dist2_new, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpointsL,
        imgpointsR,
        K1, dist1,
        K2, dist2,
        (w,h),
        criteria=criteria,
        flags=flags
    )
    print(f"Stereo calibrate RMS error: {retval}")
    save_stereo_extrinsics(args.out, R, T, E, F, (w,h))