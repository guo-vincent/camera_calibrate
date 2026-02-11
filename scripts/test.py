import cv2

img = cv2.imread(r"C:\GREAT_LAB\camera_calibrate\data\raw\test\_20260211_155002\right\right_000.png")

if img is None:
    raise FileNotFoundError("file not found")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

for i in range(6, 11):
    for j in range(3, 8):
        ret, corners = cv2.findChessboardCornersSB(gray, (i,j))
        print(ret)
