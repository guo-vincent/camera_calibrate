# Stereo Camera Calibration with OpenCV (Windows)

This repository contains a complete, reproducible pipeline for calibrating two monocular webcams (e.g. Logitech C920x) into a stereo vision system using OpenCV. The primary goal is to obtain accurate depth perception suitable for real-time remote operation.

The repo is structured to clearly separate:

- raw calibration data
- calibration artifacts (intrinsics, extrinsic, rectification)
- calibration scripts
- runtime stereo depth code

Once calibration is complete, runtime systems can load the saved YAML files without ever re-running calibration.

---

## Hardware Assumptions

- Two identical or similar webcams (Logitech C920x recommended)
- Rigid stereo mount (no movement after calibration)
- Fixed camera settings (we used logitune to adjust the C920X cameras with the following settings):
  - Anti-flicker: NTSC 60 Hz
  - auto exposure OFF
    - Exposure: -6
    - Gain: 192
  - auto white balance OFF
    - Temperature: 4500 K
    - Brightness: 128
    - Contrast: 32
    - Saturation: 32
    - Sharpness: 0
- Known and fixed resolution (e.g. 1280x720)

If *any* camera setting or physical alignment changes, calibration must be redone.

---

## Software Requirements

These will be the same as the alamgirakash2000/AeroPiper repo.

- Windows 10/11
- Python 3.9+
- OpenCV (opencv-python)
- NumPy

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Repository Structure Overview

stereo_calibration/
├── data/ # Input data (images, chessboards)
├── calibration/ # Generated calibration artifacts (YAML)
├── scripts/ # Calibration and verification scripts
├── runtime/ # Real-time stereo + depth code
├── utils/ # Shared helper functions
└── README.md

Each directory has a single responsibility. Calibration outputs are stored in `calibration/` and are intended to be checked into version control.

---

## Calibration Workflow (Required Order)

1. **Print calibration chessboard**
   - See `data/chessboard/README.md`
   - Measure square size accurately

2. **Capture stereo image pairs**
python scripts/capture_stereo.py

Images are saved to:
data/raw/left/
data/raw/right/

3. **Calibrate intrinsics (each camera independently)**
python scripts/calibrate_intrinsics.py

Output:
calibration/intrinsics/left_camera.yaml
calibration/intrinsics/right_camera.yaml

4. **Stereo calibration (extrinsics)**
python scripts/calibrate_stereo.py

Output:
calibration/extrinsics/stereo_extrinsics.yaml

5. **Rectification**
python scripts/rectify_images.py

Output:
calibration/rectification/

6. **Validate**

- Check rectified image alignment
- Verify epipolar lines are horizontal
- Test depth estimation

---

## Runtime Usage

Once calibration is complete:

```bash
python runtime/stereo_depth.py
```

This script:

- loads all calibration YAMLs
- rectifies incoming frames
- computes disparity
- outputs depth (meters)

No calibration images are required at runtime.

---

## Units and Conventions

- All distances are in **meters**
- Chessboard square size must match printed measurement
- Baseline is derived from stereo calibration `T` vector
- Depth is positive forward from the camera plane

---

## License

This repository is intended for research and engineering use. No warranty is provided.
