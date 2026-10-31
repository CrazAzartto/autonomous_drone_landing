"""
PROGRAM 3 - calibrate_camera.py
Compute the camera matrix K and the distortion coefficients from the checkerboard images.

Usage (from the repository folder):
    python3 calibration/calibrate_camera.py
    python3 calibration/calibrate_camera.py --show      # also show each detection + an undistortion preview

Output:
    results/camera_calibration.npz     (path set in config/camera_config.yaml)
    results/calibration_report.txt     (human-readable copy of the results)
    results/undistort_check.png        (one image: original | undistorted, side by side)
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path           # noqa: E402
from common.calibration_io import save_calibration          # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Calibrate the camera from checkerboard images.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--show", action="store_true", help="show detected corners for every image")
args = parser.parse_args()

config = load_config(args.config)

PATTERN_SIZE = (int(config["checkerboard"]["inner_corners_cols"]),
                int(config["checkerboard"]["inner_corners_rows"]))
SQUARE_SIZE_M = float(config["checkerboard"]["square_size_mm"]) / 1000.0
IMAGES_DIR = repo_path(config["calibration"]["images_dir"])
OUTPUT_FILE = repo_path(config["calibration"]["output_file"])
MIN_IMAGES = int(config["calibration"]["min_images"])
MAX_IMAGE_ERROR_PX = float(config["calibration"]["max_image_error_px"])
EXPECTED_SIZE = (int(config["camera"]["width"]), int(config["camera"]["height"]))

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)


# ============================================================
# 3-D CHECKERBOARD POINTS
# ============================================================
# The board defines its own coordinate frame: origin at the first inner corner, x along the
# rows, y down the columns, z = 0 (the board is flat). Units are METRES, so the calibration
# (and later every pose) comes out in metres.

board_points = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
board_points[:, :2] = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE_M


# ============================================================
# FIND CORNERS IN EVERY IMAGE
# ============================================================

image_files = sorted(glob.glob(os.path.join(IMAGES_DIR, "*.png")) + glob.glob(os.path.join(IMAGES_DIR, "*.jpg")))
print(f"Found {len(image_files)} images in {IMAGES_DIR}")

object_points = []     # 3-D board points, one array per image
image_points = []      # matching 2-D detected corners, one array per image
used_files = []

for path in image_files:
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        print(f"  SKIP {os.path.basename(path)}: unreadable")
        continue
    size = (gray.shape[1], gray.shape[0])
    if size != EXPECTED_SIZE:
        print(f"  SKIP {os.path.basename(path)}: size {size} differs from config {EXPECTED_SIZE}")
        continue

    found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE,
                                               cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        print(f"  SKIP {os.path.basename(path)}: board not found")
        continue

    # Refine every corner to sub-pixel accuracy (about 0.1 px instead of 1 px).
    corners = cv2.cornerSubPix(gray, corners.reshape(-1, 1, 2).astype(np.float32), (11, 11), (-1, -1),
                               SUBPIX_CRITERIA)

    object_points.append(board_points)
    image_points.append(corners)
    used_files.append(path)

    if args.show:
        view = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(view, PATTERN_SIZE, corners, True)
        cv2.imshow("detected corners (any key = next)", view)
        cv2.waitKey(0)

if args.show:
    cv2.destroyAllWindows()

print(f"Board detected in {len(used_files)} images.")
if len(used_files) < MIN_IMAGES:
    print(f"ERROR: need at least {MIN_IMAGES} usable images. Capture more with "
          f"calibration/capture_calibration_images.py")
    sys.exit(1)


# ============================================================
# CALIBRATE
# ============================================================

def calibrate(obj_pts, img_pts):
    """Run OpenCV calibration and compute each image's own RMS reprojection error."""
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, EXPECTED_SIZE, None, None)
    errors = []
    for points_3d, points_2d, rvec, tvec in zip(obj_pts, img_pts, rvecs, tvecs):
        projected, _ = cv2.projectPoints(points_3d, rvec, tvec, camera_matrix, dist_coeffs)
        difference = projected.reshape(-1, 2) - points_2d.reshape(-1, 2)
        errors.append(float(np.sqrt(np.mean(np.sum(difference ** 2, axis=1)))))
    return rms, camera_matrix, dist_coeffs, np.array(errors)


print("\nCalibrating ...")
rms, camera_matrix, dist_coeffs, errors = calibrate(object_points, image_points)

# Drop clearly bad images (blur, mis-detections) once, then recalibrate.
bad = [i for i, e in enumerate(errors) if e > MAX_IMAGE_ERROR_PX]
if bad and len(used_files) - len(bad) >= MIN_IMAGES:
    print(f"Removing {len(bad)} image(s) with error > {MAX_IMAGE_ERROR_PX} px and recalibrating:")
    for i in bad:
        print(f"  {os.path.basename(used_files[i])}  ({errors[i]:.2f} px)")
    keep = [i for i in range(len(used_files)) if i not in bad]
    object_points = [object_points[i] for i in keep]
    image_points = [image_points[i] for i in keep]
    used_files = [used_files[i] for i in keep]
    rms, camera_matrix, dist_coeffs, errors = calibrate(object_points, image_points)
elif bad:
    print(f"WARNING: {len(bad)} image(s) have error > {MAX_IMAGE_ERROR_PX} px, but removing them would leave "
          f"fewer than {MIN_IMAGES} images. Consider recapturing.")


# ============================================================
# REPORT
# ============================================================

fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
width, height = EXPECTED_SIZE
hfov = 2 * np.degrees(np.arctan(width / (2 * fx)))
vfov = 2 * np.degrees(np.arctan(height / (2 * fy)))
dfov = 2 * np.degrees(np.arctan(np.hypot(width / (2 * fx), height / (2 * fy))))

if rms < 0.5:
    verdict = "GOOD"
elif rms < 1.0:
    verdict = "ACCEPTABLE (try to get below 0.5 px)"
else:
    verdict = "POOR - recapture (blur, bent board, wrong square size or too few poses)"

np.set_printoptions(suppress=True, precision=4)
lines = [
    "==================== CAMERA CALIBRATION RESULT ====================",
    f"images used            : {len(used_files)}",
    f"image size             : {width} x {height}",
    f"checkerboard           : {PATTERN_SIZE[0]} x {PATTERN_SIZE[1]} inner corners, {SQUARE_SIZE_M * 1000:.2f} mm squares",
    "",
    "camera matrix K =",
    f"  [[{fx:10.3f}, {0:10.3f}, {cx:10.3f}],",
    f"   [{0:10.3f}, {fy:10.3f}, {cy:10.3f}],",
    f"   [{0:10.3f}, {0:10.3f}, {1:10.3f}]]",
    f"  fx = {fx:.2f} px, fy = {fy:.2f} px  (ratio fy/fx = {fy / fx:.4f}, expect ~1.00)",
    f"  cx = {cx:.2f} px, cy = {cy:.2f} px  (image centre = {width / 2:.0f}, {height / 2:.0f})",
    f"  field of view: horizontal {hfov:.1f} deg, vertical {vfov:.1f} deg, diagonal {dfov:.1f} deg",
    "",
    "distortion coefficients [k1, k2, p1, p2, k3] =",
    f"  {np.round(dist_coeffs.ravel(), 5).tolist()}",
    "",
    f"RMS reprojection error : {rms:.3f} px   -> {verdict}",
    f"per-image error        : min {errors.min():.3f}, mean {errors.mean():.3f}, max {errors.max():.3f} px",
    "",
    "per-image errors (px):",
]
lines += [f"  {errors[i]:6.3f}  {os.path.basename(used_files[i])}" for i in np.argsort(-errors)]
report = "\n".join(lines)
print("\n" + report)

if abs(fy / fx - 1.0) > 0.03:
    print("\nWARNING: fx and fy differ by more than 3 %. Usually a bent board or a wrong pattern size.")
if abs(cx - width / 2) > 0.1 * width or abs(cy - height / 2) > 0.1 * height:
    print("\nWARNING: principal point far from the image centre. Add images with the board near ALL edges/corners.")


# ============================================================
# SAVE
# ============================================================

save_calibration(OUTPUT_FILE, camera_matrix, dist_coeffs, EXPECTED_SIZE, rms, errors,
                 SQUARE_SIZE_M * 1000.0, PATTERN_SIZE, config["camera"]["fourcc"])

report_path = os.path.join(os.path.dirname(OUTPUT_FILE), "calibration_report.txt")
with open(report_path, "w") as file:
    file.write(report + "\n")

# Undistortion preview: straight edges of the board should look straight on the right side.
preview = cv2.imread(used_files[int(np.argmax(errors))])
new_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, EXPECTED_SIZE, 0)
undistorted = cv2.undistort(preview, camera_matrix, dist_coeffs, None, new_matrix)
side_by_side = np.hstack([preview, undistorted])
preview_path = os.path.join(os.path.dirname(OUTPUT_FILE), "undistort_check.png")
cv2.imwrite(preview_path, side_by_side)

print(f"\nSaved calibration : {OUTPUT_FILE}")
print(f"Saved report      : {report_path}")
print(f"Saved preview     : {preview_path}  (left = original, right = undistorted)")
print("Next step:  python3 detection/aruco_pose_estimation.py")

if args.show:
    cv2.imshow("left: original | right: undistorted  (any key = close)", cv2.resize(side_by_side, None, fx=0.5, fy=0.5))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
