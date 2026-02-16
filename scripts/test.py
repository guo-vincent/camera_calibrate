import cv2
import numpy as np

def corner_sharpness(gray, corners, patch_size=9):
    """
    Compute mean Laplacian variance in patches around each corner.
    """
    h, w = gray.shape
    r = patch_size // 2
    scores = []

    for pt in corners:
        x, y = int(pt[0][0]), int(pt[0][1])

        x1 = max(0, x - r)
        x2 = min(w, x + r + 1)
        y1 = max(0, y - r)
        y2 = min(h, y + r + 1)

        patch = gray[y1:y2, x1:x2]

        if patch.size < 9:  # too small to be meaningful
            continue

        lap_var = cv2.Laplacian(patch, cv2.CV_64F).var()
        scores.append(lap_var)

    if len(scores) == 0:
        return 0.0

    return float(np.mean(scores))

for k in range(17):
    path = f"data/raw/test_20260216_150656/right/right_{k:03d}.png"
    img = cv2.imread(path)

    if img is None:
        raise FileNotFoundError(f"file not found: {path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    norm = np.empty_like(gray)
    cv2.normalize(gray, norm, 0, 255, cv2.NORM_MINMAX)
    gray = norm

    rows, cols = 5, 9
    pattern_size = (rows, cols)

    ret, corners = cv2.findChessboardCornersSB(
        gray,
        pattern_size,
        flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    )

    if not ret:
        pattern_size = (cols, rows)
        ret, corners = cv2.findChessboardCornersSB(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        )

    sharp = 0.0
    if ret and corners is not None:
        sharp = corner_sharpness(gray, corners)

    print(f"board {k}: found={ret} dims={pattern_size} sharpness={sharp:.2f}")