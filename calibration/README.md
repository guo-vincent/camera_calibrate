# Calibration Chessboard

This directory is a WORK IN PROGRESS.
This directory contains printable calibration targets used for camera and stereo calibration.

---

## Recommended Pattern

**Pattern type:** Standard OpenCV chessboard  
**Inner corners:** 9 x 6  
**Squares:** 10 x 7  
**Square size:** 25 mm (recommended)

File: chessboard_9x6_25mm.pdf (<https://github.com/opencv/opencv/blob/4.x/doc/pattern.png>)

This pattern works well for webcams with resolutions from 720p to 1080p and typical working distances of 0.3–2.0 m.

---

## Printing Instructions

1. Print at **100% scale**
   - Disable “Fit to Page”
   - Disable scaling
2. Use a **laser printer** if possible
3. Print on matte paper (avoid gloss)
4. Measure the printed square size with calipers or a ruler
5. Use the *measured* value in calibration code

If the printed square size is not accurate, your depth scale will be wrong.

---

## Mounting Instructions

- Mount the printed sheet on:
  - foam board
  - acrylic sheet
  - plywood
- Ensure the board is:
  - flat
  - rigid
  - not warped
- Do NOT fold or bend the paper

---

## During Image Capture

When capturing calibration images:

- Show the board at different depths
- Tilt and rotate the board
- Cover all image regions
- Ensure corners are sharp and well-lit
- Avoid motion blur

Aim for **20–30 valid stereo pairs**.

---

## Corner Definition Reminder

For OpenCV:

- A “9x6” board means **9 inner corners horizontally**
- NOT the number of squares
- The calibration code must match this exactly

---

## If You Need a Different Board

You may generate custom boards using online generators (A4 / US Letter). If you do:

- Record number of inner corners
- Record square size in mm
- Update calibration scripts accordingly

---
