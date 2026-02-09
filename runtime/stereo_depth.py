# --------------------------------------------------------------------------
# Purpose: Real-time stereo depth pipeline using saved calibration/rectification.
# Usage:
#   python runtime/stereo_depth.py --left 0 --right 1 --rect calibration/rectification/rectification.yaml \
#       --maps calibration/rectification/rect_maps.npz --intr calibration/intrinsics/left_camera.yaml \
#       --num_disparities 128 --blocksize 5
# Notes:
#  - This loads precalculated rectification maps (map1x,map1y,map2x,map2y) saved by rectify_images.py.
#  - It uses StereoSGBM; tune parameters to your scene.
#  - Produces a realtime colored disparity and (optionally) 3D point cloud export.
# --------------------------------------------------------------------------
import cv2
import numpy as np
import argparse
import os
import time
from typing import Union
from cv2.typing import MatLike

def load_yaml_matrix(path: Union[str, os.PathLike], name: str) -> MatLike:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open {path}")
    mat = fs.getNode(name).mat()
    fs.release()
    if mat is None:
        raise RuntimeError(f"Node '{name}' not found in {path}")
    return mat

def ensure_dir(path: Union[str, os.PathLike]) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)

def save_ply_fast(path: str, verts: np.ndarray, colors: np.ndarray) -> None:
    """
    Save a point cloud to ASCII PLY using numpy.savetxt for speed.

    verts: (N,3) float32 or float64
    colors: (N,3) uint8 or int
    """
    assert verts.shape[0] == colors.shape[0], "verts/colors length mismatch"
    N = verts.shape[0]
    header = [
        "ply",
        "format ascii 1.0",
        f"element vertex {N}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header"
    ]
    with open(path, "w") as f:
        f.write("\n".join(header) + "\n")
        # Build Nx6 numeric array. Colors casted to integers to avoid scientific notation.
        data = np.hstack((verts.astype(np.float64), colors.astype(np.int32)))
        fmt = "%.6f %.6f %.6f %d %d %d"
        np.savetxt(f, data, fmt=fmt)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Real-time stereo depth using rectification maps (fast PLY save).")
    parser.add_argument("--left", type=int, default=0)
    parser.add_argument("--right", type=int, default=1)
    parser.add_argument("--maps", required=True, help="NPZ file containing map1x,map1y,map2x,map2y")
    parser.add_argument("--rect", required=True, help="YAML file containing R1,R2,P1,P2,Q")
    parser.add_argument("--intr_left", required=True, help="Left intrinsics YAML (for image size check)")
    parser.add_argument("--num_disparities", type=int, default=128)
    parser.add_argument("--blocksize", type=int, default=5)
    parser.add_argument("--save_ply", type=str, default="", help="Optional output PLY file prefix (without timestamp)")
    parser.add_argument("--resize", type=float, default=1.0, help="Resize preview (for performance)")
    args = parser.parse_args()

    # Load rect maps
    maps = np.load(args.maps)
    map1x = maps["map1x"]
    map1y = maps["map1y"]
    map2x = maps["map2x"]
    map2y = maps["map2y"]

    # Load Q matrix (reprojection)
    Q = load_yaml_matrix(args.rect, "Q")

    capL = cv2.VideoCapture(args.left, cv2.CAP_DSHOW)
    capR = cv2.VideoCapture(args.right, cv2.CAP_DSHOW)
    if not capL.isOpened() or not capR.isOpened():
        raise RuntimeError("Unable to open cameras.")

    # Create StereoSGBM matcher (ensure numDisparities is divisible by 16)
    num_disp = args.num_disparities
    if num_disp % 16 != 0:
        num_disp = (num_disp // 16 + 1) * 16
    stereo = cv2.StereoSGBM.create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=args.blocksize,
        P1=8 * 3 * args.blocksize**2,
        P2=32 * 3 * args.blocksize**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=2
    )

    print("Press 'q' or ESC to quit. Press 'p' to save the current disparity and (optionally) point cloud.")

    while True:
        # Grab & retrieve
        capL.grab()
        capR.grab()
        retL, frameL = capL.retrieve()
        retR, frameR = capR.retrieve()
        if not retL or not retR:
            time.sleep(0.01)
            continue

        # Remap (rectify)
        rectL = cv2.remap(frameL, map1x, map1y, cv2.INTER_LINEAR)
        rectR = cv2.remap(frameR, map2x, map2y, cv2.INTER_LINEAR)

        grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

        # stereo.compute returns fixed-point disparities: disparity*16 (int). Convert to float pixels.
        disp_fixed = stereo.compute(grayL, grayR)  # typically int16
        disp = disp_fixed.astype(np.float32) / 16.0  # now in pixel units; required by reprojectImageTo3D

        # clamp negatives
        disp_vis = np.copy(disp)
        disp_vis[disp_vis < 0] = 0
        disp_norm = np.empty(disp_vis.shape, dtype=np.uint8)
        cv2.normalize(src=disp_vis, dst=disp_norm, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_norm.astype(np.uint8), cv2.COLORMAP_JET)

        # Reproject to 3D
        points_3d = cv2.reprojectImageTo3D(disp, Q)
        depth = points_3d[:, :, 2]

        # Compose display
        if args.resize != 1.0:
            display_left = cv2.resize(rectL, (0,0), fx=args.resize, fy=args.resize)
            display_disp = cv2.resize(disp_color, (0,0), fx=args.resize, fy=args.resize)
        else:
            display_left = rectL
            display_disp = disp_color

        combined = np.hstack((display_left, display_disp))
        cv2.imshow("Left | Disparity", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('p'):
            timestamp = int(time.time())
            disp_path = f"disparity_{timestamp}.png"
            cv2.imwrite(disp_path, disp_norm)
            print(f"Saved disparity to {disp_path}")

            if args.save_ply:
                # Create mask for valid points: disparity > 0 and finite depth
                mask = (disp > 0) & np.isfinite(points_3d[:, :, 2])
                verts = points_3d[mask]
                colors = rectL[mask]  # BGR
                # Convert BGR->RGB for PLY.
                colors = colors[:, ::-1]  # BGR -> RGB

                if verts.shape[0] == 0:
                    print("No valid 3D points to save.")
                else:
                    ply_path = f"{args.save_ply}_{timestamp}.ply" if not args.save_ply.endswith(".ply") else args.save_ply
                    print(f"Saving point cloud with {verts.shape[0]} points to {ply_path} (this may take a few seconds)...")
                    # Use fast saver
                    save_ply_fast(ply_path, verts, colors)
                    print(f"Saved point cloud to {ply_path}")

    capL.release()
    capR.release()
    cv2.destroyAllWindows()