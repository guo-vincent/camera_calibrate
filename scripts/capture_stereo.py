# --------------------------------------------------------------------------
# Purpose: Capture synchronized-ish stereo image pairs from two webcams.
# Usage:
#   python scripts/capture_stereo.py --left 0 --right 1 --out data/raw --prefix sess1 \
#       --width 1280 --height 720 --fps 30 --count 60
# Notes:
#  - Cap.grab() / cap.retrieve() to minimize temporal skew.
#  - Set camera indices appropriately for your system. On Windows, try using
#    cv2.CAP_DSHOW backend (recommended) by setting cv2.VideoCapture(index, cv2.CAP_DSHOW).
#  - Make sure autofocus/auto exposure/auto white balance are disabled
#  - If you have a laptop camera (like in my case: --left 1 --right 2 would correspond to the cameras instead). 
#    Run utils\check_camera_ids.py first.
#
# Operating Instructions:
#  - Press 's' to save a stereo pair (left and right images will be
#    saved with matching numeric suffixes in separate left/ and right/ folders).
#  - Press 'q' or ESC to quit.
#  LEFT CAMERA IMAGES AND RIGHT CAMERA IMAGES SHOULD NOT BE MIXED. EVER. ONCE YOU DETERMINE WHICH CAMERA IS THE LEFT ONE KEEP IT CONSISTENT.
# 
#
#  LAST TESTED ON 2/9/26. Code successfully executed.
# --------------------------------------------------------------------------
import cv2
import os
import argparse
from datetime import datetime
from typing import Union

def set_camera_properties(cap: cv2.VideoCapture, width: int, height: int, fps: int) -> None:
    # Try to set resolution and FPS. Not all webcams will honor settings, but the webcam we are using should. In theory.
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    except Exception:
        pass

def ensure_dir(path: Union[str, os.PathLike]) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)

def main(args):
    left_idx = args.left
    right_idx = args.right

    left_cap = cv2.VideoCapture(left_idx, cv2.CAP_DSHOW)
    right_cap = cv2.VideoCapture(right_idx, cv2.CAP_DSHOW)

    if not left_cap.isOpened() or not right_cap.isOpened():
        print("ERROR: Could not open one or both video captures.")
        print("Check camera indices and that no other app is using them.")
        return

    set_camera_properties(left_cap, args.width, args.height, args.fps)
    set_camera_properties(right_cap, args.width, args.height, args.fps)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out, args.prefix + "_" + timestamp)
    left_dir = os.path.join(out_dir, "left")
    right_dir = os.path.join(out_dir, "right")
    ensure_dir(left_dir)
    ensure_dir(right_dir)

    print(f"Saving pairs to: {out_dir}")
    print("Press 's' to save a stereo pair, 'q' or ESC to quit.")

    idx = 0
    while True:
        # Grab both frames first, then retrieve
        # USB cams aren't hardware synced, so we do this to reduce time delay between frames.
        left_cap.grab()
        right_cap.grab()
        retL, frameL = left_cap.retrieve()
        retR, frameR = right_cap.retrieve()

        if not retL or frameL is None:
            print("Warning: left frame not received")
            continue
        if not retR or frameR is None:
            print("Warning: right frame not received")
            continue

        # Show side-by-side preview
        combined = cv2.hconcat([cv2.resize(frameL, (args.width//2, args.height//2)),
                                 cv2.resize(frameR, (args.width//2, args.height//2))])
        cv2.putText(combined, "LEFT", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.putText(combined, "RIGHT", (args.width//2 + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow("Stereo Capture (press 's' to save)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            left_path = os.path.join(left_dir, f"left_{idx:03d}.png")
            right_path = os.path.join(right_dir, f"right_{idx:03d}.png")
            cv2.imwrite(left_path, frameL)
            cv2.imwrite(right_path, frameR)
            print(f"Saved pair {idx} -> {left_path}, {right_path}")
            idx += 1
            if args.count and idx >= args.count:
                print("Reached requested count. Exiting.")
                break
        elif key == ord('q') or key == 27:
            print("User requested exit.")
            break

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture synchronized-ish stereo pairs.")
    parser.add_argument("--left", type=int, default=0, help="Left camera index")
    parser.add_argument("--right", type=int, default=1, help="Right camera index")
    parser.add_argument("--out", type=str, default="data/raw", help="Output base directory")
    parser.add_argument("--prefix", type=str, default="session", help="Folder prefix")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--fps", type=int, default=30, help="Requested FPS")
    parser.add_argument("--count", type=int, default=0, help="Number of pairs to capture (0 = unlimited)")
    args = parser.parse_args()
    main(args)