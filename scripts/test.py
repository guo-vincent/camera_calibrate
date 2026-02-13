import cv2

# filename
img = cv2.imread(r"data\raw\session_20260213_152433\right\right_000.png")

if img is None:
    raise FileNotFoundError("file not found")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

for i in range(6, 11):
    for j in range(3, 8):
        ret, corners = cv2.findChessboardCornersSB(gray, (i,j))
        print(f"found: {ret} with dimensions {i, j}")
        vis = img.copy()
        cv2.drawChessboardCorners(vis, (i, j), corners, ret)
        cv2.imwrite("debug_chess_vis.png", vis)
