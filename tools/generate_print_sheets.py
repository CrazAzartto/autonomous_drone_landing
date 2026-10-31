"""
Generate exact-size printable A4 PDFs for the ArUco marker and the calibration checkerboard,
using the dictionary / ID / sizes in config/camera_config.yaml.

Usage (from the autonomous_drone_landing folder, with the .venv active):
    python tools\\generate_print_sheets.py
Output:
    results/print_aruco_marker.pdf      marker with white margin, cut line and a 100 mm check ruler
    results/print_checkerboard.pdf      checkerboard with the configured inner-corner count and square size
    results/aruco_marker.png            plain marker image (for viewing only - print the PDF)

PRINT AT 100 % / "Actual size" - never "Fit to page". Then measure with a ruler.
"""

import argparse
import os
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path      # noqa: E402
from common.aruco_utils import get_dictionary, check_opencv_aruco   # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Generate printable marker and checkerboard PDFs.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--dpi", type=int, default=300)
args = parser.parse_args()

config = load_config(args.config)
check_opencv_aruco()

DPI = args.dpi
A4_MM = (210.0, 297.0)
OUT_DIR = repo_path("results")
os.makedirs(OUT_DIR, exist_ok=True)


def mm(value_mm):
    """Millimetres -> pixels at the chosen DPI."""
    return int(round(value_mm / 25.4 * DPI))


def new_page():
    return np.full((mm(A4_MM[1]), mm(A4_MM[0])), 255, np.uint8)


def text(page, string, x_mm, y_mm, scale=1.0, thickness=2):
    cv2.putText(page, string, (mm(x_mm), mm(y_mm)), cv2.FONT_HERSHEY_SIMPLEX, scale * DPI / 300, 0,
                thickness, cv2.LINE_AA)


def dashed_rectangle(page, x0, y0, x1, y1, dash_mm=3.0):
    for (xa, ya, xb, yb) in [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]:
        length = np.hypot(xb - xa, yb - ya)
        steps = int(length / dash_mm)
        for i in range(0, steps, 2):
            p = (xa + (xb - xa) * i / steps, ya + (yb - ya) * i / steps)
            q = (xa + (xb - xa) * min(i + 1, steps) / steps, ya + (yb - ya) * min(i + 1, steps) / steps)
            cv2.line(page, (mm(p[0]), mm(p[1])), (mm(q[0]), mm(q[1])), 140, 2)


def ruler(page, x_mm, y_mm, length_mm=100):
    cv2.line(page, (mm(x_mm), mm(y_mm)), (mm(x_mm + length_mm), mm(y_mm)), 0, 2)
    for i in range(0, length_mm + 1, 5):
        tick = 4 if i % 10 == 0 else 2
        cv2.line(page, (mm(x_mm + i), mm(y_mm)), (mm(x_mm + i), mm(y_mm - tick)), 0, 2)
    text(page, f"{length_mm} mm check ruler - measure it!", x_mm, y_mm + 6, 0.8)


def save_pdf(page, path):
    Image.fromarray(page).convert("L").save(path, "PDF", resolution=float(DPI))
    print(f"Saved {path}")


# ============================================================
# ARUCO MARKER SHEET
# ============================================================

dictionary_name = config["aruco"]["dictionary"]
marker_id = int(config["aruco"]["marker_id"])
marker_mm = float(config["aruco"]["marker_length_mm"])
dictionary = get_dictionary(dictionary_name)

marker_px = mm(marker_mm)
marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_px)
cell_mm = marker_mm / (dictionary.markerSize + 2)       # +2 for the black border cells
margin_mm = max(cell_mm, 5.0)                            # white quiet zone around the black square

page = new_page()
x0, y0 = (A4_MM[0] - marker_mm) / 2.0, 70.0
page[mm(y0):mm(y0) + marker_px, mm(x0):mm(x0) + marker_px] = marker_image
dashed_rectangle(page, x0 - margin_mm, y0 - margin_mm, x0 + marker_mm + margin_mm, y0 + marker_mm + margin_mm)

text(page, "ArUco marker  -  print at 100% (Actual size), NOT 'Fit to page'", 15, 20, 1.0)
text(page, f"dictionary {dictionary_name}   ID {marker_id}   black square = {marker_mm:.1f} mm", 15, 30, 1.0)
text(page, f"cut on the dashed line (white margin {margin_mm:.1f} mm) - keep the white border!", 15, 40, 0.9)
text(page, "TOP of marker = this side ^  = DRONE FRONT (mark an arrow on the back before cutting)", 15, 55, 0.9)
text(page, "Measure the BLACK square after printing and put that value in", 15, y0 + marker_mm + margin_mm + 15, 0.9)
text(page, "config/camera_config.yaml -> aruco.marker_length_mm", 15, y0 + marker_mm + margin_mm + 23, 0.9)
ruler(page, 15, 270)

save_pdf(page, os.path.join(OUT_DIR, "print_aruco_marker.pdf"))
cv2.imwrite(os.path.join(OUT_DIR, "aruco_marker.png"),
            cv2.copyMakeBorder(cv2.aruco.generateImageMarker(dictionary, marker_id, 600), 75, 75, 75, 75,
                               cv2.BORDER_CONSTANT, value=255))


# ============================================================
# CHECKERBOARD SHEET
# ============================================================

cols = int(config["checkerboard"]["inner_corners_cols"])
rows = int(config["checkerboard"]["inner_corners_rows"])
square_mm = float(config["checkerboard"]["square_size_mm"])
squares_x, squares_y = cols + 1, rows + 1
board_w, board_h = squares_x * square_mm, squares_y * square_mm

if board_w > A4_MM[0] - 20 or board_h > A4_MM[1] - 40:
    print(f"ERROR: board {board_w:.0f} x {board_h:.0f} mm does not fit on A4. Use smaller squares or fewer corners.")
    sys.exit(1)

page = new_page()
bx, by = (A4_MM[0] - board_w) / 2.0, 35.0
for r in range(squares_y):
    for c in range(squares_x):
        if (r + c) % 2 == 0:
            page[mm(by + r * square_mm):mm(by + (r + 1) * square_mm),
                 mm(bx + c * square_mm):mm(bx + (c + 1) * square_mm)] = 0

text(page, f"Calibration checkerboard: {cols} x {rows} INNER corners, {square_mm:.1f} mm squares", 12, 15, 0.95)
text(page, "Print at 100%. Glue FLAT on stiff card. Measure 6 squares, divide by 6 -> square_size_mm.", 12, 24, 0.8)
save_pdf(page, os.path.join(OUT_DIR, "print_checkerboard.pdf"))

print("\nPrint both PDFs at 100 % scale and check the sizes with a ruler.")
