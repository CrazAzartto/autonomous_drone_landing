"""
PROGRAM 2 - capture_calibration_images.py
Capture checkerboard photos for camera calibration.

Usage (from the autonomous_drone_landing folder, with the .venv active):
    python calibration\\capture_calibration_images.py
    python calibration\\capture_calibration_images.py --clear     # delete old images first

Keys:
    SPACE   save the current frame (only if the WHOLE board is detected and the image is sharp)
    u       undo: delete the last saved image
    q / ESC quit

Aim: 20-30 images that together cover the whole image, at several distances and tilts.
The blue overlay shows where the board has already been - keep going until it covers everything.
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path      # noqa: E402
from common.camera import open_camera                 # noqa: E402
from common.overlay import draw_text                   # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Capture checkerboard images for calibration.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--clear", action="store_true", help="delete existing calibration images first")
args = parser.parse_args()

config = load_config(args.config)
camera_config = config["camera"]

PATTERN_SIZE = (int(config["checkerboard"]["inner_corners_cols"]),
                int(config["checkerboard"]["inner_corners_rows"]))
IMAGES_DIR = repo_path(config["calibration"]["images_dir"])
MIN_IMAGES = int(config["calibration"]["min_images"])
TARGET_IMAGES = 25
MIN_SHARPNESS = 60.0          # variance of Laplacian; below this the image is probably blurred

# Suggested sequence of board poses, shown on screen one after another.
POSE_HINTS = [
    "centre, board flat (facing camera), ~60 cm",
    "top-left corner of image",
    "top-right corner of image",
    "bottom-left corner of image",
    "bottom-right corner of image",
    "left edge, middle",
    "right edge, middle",
    "top edge, middle",
    "bottom edge, middle",
    "centre, tilted LEFT ~30 deg",
    "centre, tilted RIGHT ~30 deg",
    "centre, tilted UP (top away) ~30 deg",
    "centre, tilted DOWN (bottom away) ~30 deg",
    "centre, far (~1.2 m)",
    "centre, near (board almost fills image)",
    "corner of image + tilted",
    "rotate board ~20 deg in its own plane",
    "anywhere not yet covered (blue overlay)",
]


# ============================================================
# PREPARE OUTPUT FOLDER
# ============================================================

os.makedirs(IMAGES_DIR, exist_ok=True)
existing = sorted(glob.glob(os.path.join(IMAGES_DIR, "*.png")))
if args.clear and existing:
    for path in existing:
        os.remove(path)
    print(f"Deleted {len(existing)} old images.")
    existing = []
elif existing:
    print(f"NOTE: {len(existing)} images already in {IMAGES_DIR}. New images are added to them.")
    print("      Use --clear to start over (do this whenever resolution, focus or zoom changed!).")

saved_paths = list(existing)


# ============================================================
# OPEN CAMERA
# ============================================================

try:
    capture, actual = open_camera(camera_config)
except RuntimeError as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)

if (actual["width"], actual["height"]) != (int(camera_config["width"]), int(camera_config["height"])):
    print("ERROR: camera resolution differs from the config. Fix the config or the camera first.")
    capture.release()
    sys.exit(1)

coverage = np.zeros((actual["height"], actual["width"], 3), dtype=np.uint8)
for path in saved_paths:          # show coverage of images captured in an earlier session
    gray_old = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray_old is None or gray_old.shape[::-1] != (actual["width"], actual["height"]):
        continue
    found_old, corners_old = cv2.findChessboardCorners(gray_old, PATTERN_SIZE)
    if found_old:
        cv2.fillConvexPoly(coverage, cv2.convexHull(corners_old.reshape(-1, 2).astype(np.int32)), (255, 120, 0))

print(f"\nBoard: {PATTERN_SIZE[0]} x {PATTERN_SIZE[1]} inner corners. Saving to: {IMAGES_DIR}")
print("SPACE = save   u = undo last   q = quit")


# ============================================================
# MAIN LOOP
# ============================================================

FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
message = ""
message_until = 0.0
failed_reads = 0

try:
    while True:
        ok, frame = capture.read()
        if not ok or frame is None:
            failed_reads += 1
            if failed_reads > config["detection"]["max_failed_reads"]:
                print("ERROR: camera stopped delivering frames.")
                break
            continue
        failed_reads = 0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, FIND_FLAGS)
        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()

        view = cv2.addWeighted(frame, 1.0, coverage, 0.35, 0)
        if found:
            cv2.drawChessboardCorners(view, PATTERN_SIZE, corners.reshape(-1, 1, 2), found)

        count = len(saved_paths)
        hint = POSE_HINTS[min(count, len(POSE_HINTS) - 1)]
        colour = (0, 255, 0) if found else (0, 0, 255)
        draw_text(view, f"saved {count}/{TARGET_IMAGES} (min {MIN_IMAGES})   board: {'FOUND' if found else 'not found'}"
                  f"   sharpness: {sharpness:.0f}", (10, 30), colour)
        draw_text(view, f"next pose: {hint}", (10, 60), (255, 255, 0))
        if time.time() < message_until:
            draw_text(view, message, (10, 90), (0, 200, 255))
        cv2.imshow("capture calibration images  (SPACE = save, u = undo, q = quit)", view)

        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break

        elif key == ord(" "):
            if not found:
                message = "NOT saved: whole board must be visible and detected"
            elif sharpness < MIN_SHARPNESS:
                message = "NOT saved: image blurred - hold the board still"
            else:
                path = os.path.join(IMAGES_DIR, f"calib_{int(time.time() * 1000)}.png")
                cv2.imwrite(path, frame)          # the CLEAN frame, without drawings
                saved_paths.append(path)
                cv2.fillConvexPoly(coverage, cv2.convexHull(corners.reshape(-1, 2).astype(np.int32)), (255, 120, 0))
                message = f"saved image {len(saved_paths)}"
                print(f"Saved {path}")
            message_until = time.time() + 1.5

        elif key == ord("u") and saved_paths:
            removed = saved_paths.pop()
            if os.path.exists(removed):
                os.remove(removed)
            message = "last image deleted (coverage overlay is not undone)"
            message_until = time.time() + 1.5
            print(f"Deleted {removed}")

finally:
    capture.release()
    cv2.destroyAllWindows()

print(f"\n{len(saved_paths)} images in {IMAGES_DIR}")
if len(saved_paths) < MIN_IMAGES:
    print(f"Capture at least {MIN_IMAGES} (20-30 recommended) before calibrating.")
else:
    print("Next step:  python calibration\\calibrate_camera.py")
