import cv2
import os

squares_x = 7
squares_y = 5
square_size_mm = 25
marker_size_mm = 18
dpi = 300

def mm_to_px(mm, dpi):
    return int((mm / 25.4) * dpi)

square_px = mm_to_px(square_size_mm, dpi)
marker_px = mm_to_px(marker_size_mm, dpi)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
board = cv2.aruco.CharucoBoard(
    (squares_x, squares_y), 
    square_px, 
    marker_px, 
    aruco_dict
)

width_px = squares_x * square_px
height_px = squares_y * square_px

img = board.generateImage((width_px, height_px))

output_path = "data/chessboard/charuco_25mm_300dpi.png"
cv2.imwrite(output_path, img)

print(f"Success! Created {output_path}")
print(f"Physical Size: {squares_x * square_size_mm}mm x {squares_y * square_size_mm}mm")
print(f"Pixel Size: {width_px} x {height_px}")