from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib import colors

# === CONFIGURATION ===
squares_x = 9
squares_y = 6
square_size_mm = 30

filename = "data\chessboard\opencv_chessboard_9x6_30mm.pdf"

page_width, page_height = landscape(A4)

c = canvas.Canvas(filename, pagesize=(page_width, page_height))

square_size = square_size_mm * mm
board_width = squares_x * square_size
board_height = squares_y * square_size

# Center board on page
start_x = (page_width - board_width) / 2
start_y = (page_height - board_height) / 2

for y in range(squares_y):
    for x in range(squares_x):
        if (x + y) % 2 == 0:
            c.setFillColor(colors.black)
        else:
            c.setFillColor(colors.white)

        c.rect(
            start_x + x * square_size,
            start_y + y * square_size,
            square_size,
            square_size,
            stroke=0,
            fill=1
        )

c.save()

print("Generated:", filename)
