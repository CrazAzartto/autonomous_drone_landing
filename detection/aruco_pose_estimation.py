"""
PROGRAM 4 - aruco_pose_estimation.py
Live ArUco detection + 3-D pose of the marker in the CAMERA frame.

Usage (from the repository folder):
    python3 detection/aruco_pose_estimation.py                 # live camera from the config
    python3 detection/aruco_pose_estimation.py --log           # also write results/pose_log.csv
    python3 detection/aruco_pose_estimation.py --all-ids       # estimate pose of every visible marker
    python3 detection/aruco_pose_estimation.py --image results/snapshot_123.png   # test on a still image

Keys:
    q / ESC  quit
    s        save a snapshot (raw frame) to results/
    p        pause / resume

Output (camera frame, OpenCV convention, metres -> shown in cm):
    X > 0 : marker is to the RIGHT of the optical axis in the image
    Y > 0 : marker is BELOW the optical axis in the image (image y points down)
    Z > 0 : distance ALONG the optical axis, in front of the lens
"""

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path                         # noqa: E402
from common.camera import open_camera                                    # noqa: E402
from common.calibration_io import load_calibration                       # noqa: E402
from common.aruco_utils import create_detector, detect_markers, estimate_marker_pose  # noqa: E402
from common.overlay import draw_text                                     # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Live ArUco marker pose estimation.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--image", default=None, help="process one image file instead of the camera")
parser.add_argument("--log", action="store_true", help="write every pose to the CSV log file")
parser.add_argument("--all-ids", action="store_true", help="estimate the pose of every detected marker")
args = parser.parse_args()

config = load_config(args.config)
camera_config = config["camera"]
aruco_config = config["aruco"]
detection_config = config["detection"]

TARGET_ID = int(aruco_config["marker_id"])
MARKER_LENGTH_M = float(aruco_config["marker_length_mm"]) / 1000.0
PRINT_INTERVAL = 1.0 / float(detection_config["print_rate_hz"])
AXIS_LENGTH_M = MARKER_LENGTH_M * float(detection_config["axis_length_ratio"])
MAX_FAILED_READS = int(detection_config["max_failed_reads"])
LOG_FILE = repo_path(detection_config["log_file"])

GREEN, RED, YELLOW, WHITE, CYAN = (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 255), (255, 255, 0)


# ============================================================
# LOAD CALIBRATION
# ============================================================

try:
    calibration = load_calibration(repo_path(config["calibration"]["output_file"]))
except FileNotFoundError as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)

CAMERA_MATRIX = calibration["camera_matrix"]
DIST_COEFFS = calibration["dist_coeffs"]
CALIB_SIZE = (calibration["image_width"], calibration["image_height"])

print(f"Calibration loaded: {CALIB_SIZE[0]}x{CALIB_SIZE[1]}, RMS {calibration['rms_error_px']:.3f} px, "
      f"fx={CAMERA_MATRIX[0, 0]:.1f} fy={CAMERA_MATRIX[1, 1]:.1f}")
print(f"Marker: {aruco_config['dictionary']}, ID {TARGET_ID}, side {MARKER_LENGTH_M * 1000:.1f} mm")

if CALIB_SIZE != (int(camera_config["width"]), int(camera_config["height"])):
    print(f"ERROR: calibration was made at {CALIB_SIZE[0]}x{CALIB_SIZE[1]} but the config asks for "
          f"{camera_config['width']}x{camera_config['height']}. Recalibrate or fix the config.")
    sys.exit(1)

detector = create_detector(aruco_config["dictionary"])


# ============================================================
# PROCESS ONE FRAME
# ============================================================

def process_frame(frame):
    """Detect markers, estimate poses, draw everything. Returns (annotated_frame, list_of_results)."""
    view = frame.copy()
    corners_list, ids = detect_markers(detector, frame)
    results = []

    if len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(view, [c.reshape(1, 4, 2) for c in corners_list], ids.reshape(-1, 1))

    for corners, marker_id in zip(corners_list, ids):
        # Number the four corners (0 = top-left of the printed marker, then clockwise).
        for index, (u, v) in enumerate(corners):
            cv2.circle(view, (int(round(u)), int(round(v))), 4, CYAN, -1)
            draw_text(view, str(index), (int(u) + 6, int(v) - 6), CYAN, 0.5, 1)

        if marker_id != TARGET_ID and not args.all_ids:
            continue

        pose = estimate_marker_pose(corners, MARKER_LENGTH_M, CAMERA_MATRIX, DIST_COEFFS)
        if pose is None:
            continue
        cv2.drawFrameAxes(view, CAMERA_MATRIX, DIST_COEFFS, pose["rvec"], pose["tvec"], AXIS_LENGTH_M, 2)
        pose["id"] = int(marker_id)
        pose["center_px"] = corners.mean(axis=0)
        pose["side_px"] = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]))
        results.append(pose)

    # ---------------- text overlay ----------------
    target = next((r for r in results if r["id"] == TARGET_ID), None)
    if target is not None:
        x, y, z = target["tvec"] * 100.0
        distance = float(np.linalg.norm(target["tvec"])) * 100.0
        lines = [
            (f"ID {TARGET_ID}   X = {x:+7.1f} cm   Y = {y:+7.1f} cm   Z = {z:7.1f} cm", GREEN),
            (f"distance = {distance:.1f} cm   yaw = {target['yaw_deg']:+.1f} deg   "
             f"marker = {target['side_px']:.0f} px   reproj = {target['reproj_error_px']:.2f} px", WHITE),
        ]
        if target["reproj_error_px"] > 1.0:
            lines.append(("WARNING: high reprojection error - pose unreliable (blur/size/calibration?)", YELLOW))
    elif len(ids) > 0:
        lines = [(f"Marker {TARGET_ID} NOT detected (visible IDs: {ids.tolist()})", YELLOW)]
    else:
        lines = [("NO MARKER DETECTED", RED)]

    for row, (text, colour) in enumerate(lines):
        draw_text(view, text, (10, 30 + 28 * row), colour)

    # Image centre cross = where the optical axis (X = 0, Y = 0) approximately is.
    cx, cy = int(CAMERA_MATRIX[0, 2]), int(CAMERA_MATRIX[1, 2])
    cv2.drawMarker(view, (cx, cy), WHITE, cv2.MARKER_CROSS, 20, 1)
    return view, results


def format_result(result):
    x, y, z = result["tvec"] * 100.0
    return (f"ID {result['id']}: X={x:+7.1f} cm  Y={y:+7.1f} cm  Z={z:7.1f} cm  "
            f"yaw={result['yaw_deg']:+6.1f} deg  reproj={result['reproj_error_px']:.2f} px")


# ============================================================
# STILL IMAGE MODE
# ============================================================

if args.image:
    frame = cv2.imread(args.image)
    if frame is None:
        print(f"ERROR: cannot read image {args.image}")
        sys.exit(1)
    if (frame.shape[1], frame.shape[0]) != CALIB_SIZE:
        print(f"WARNING: image is {frame.shape[1]}x{frame.shape[0]}, calibration is {CALIB_SIZE[0]}x{CALIB_SIZE[1]}. "
              "The pose will be WRONG unless sizes match.")
    view, results = process_frame(frame)
    if not results:
        print("No target marker detected in the image.")
    for result in results:
        print(format_result(result))
    out_path = repo_path(os.path.join("results", "pose_result.png"))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, view)
    print(f"Annotated image saved to {out_path}")
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        cv2.imshow("pose (any key = close)", view)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    sys.exit(0)


# ============================================================
# OPEN CAMERA
# ============================================================

def open_checked_camera():
    capture, actual = open_camera(camera_config)
    if (actual["width"], actual["height"]) != CALIB_SIZE:
        capture.release()
        raise RuntimeError(f"camera delivers {actual['width']}x{actual['height']} but calibration is "
                           f"{CALIB_SIZE[0]}x{CALIB_SIZE[1]}; the pose would be wrong.")
    return capture


try:
    capture = open_checked_camera()
except RuntimeError as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)


# ============================================================
# CSV LOGGER
# ============================================================

log_file = None
writer = None
if args.log:
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    log_file = open(LOG_FILE, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["time_s", "marker_id", "x_m", "y_m", "z_m", "distance_m",
                     "rvec_x", "rvec_y", "rvec_z", "yaw_deg", "reproj_error_px", "side_px"])
    print(f"Logging poses to {LOG_FILE}")


# ============================================================
# MAIN LOOP
# ============================================================

print("\nRunning. q = quit, s = snapshot, p = pause\n")

start_time = time.perf_counter()
last_print_time = 0.0
last_seen_time = None
failed_reads = 0
paused = False
frame = None
fps_display = 0.0
frame_count = 0
fps_timer = time.perf_counter()

try:
    while True:

        # ----------------------------------------------------
        # READ FRAME (with reconnect on failure)
        # ----------------------------------------------------
        if not paused:
            ok, new_frame = capture.read()
            if not ok or new_frame is None:
                failed_reads += 1
                if failed_reads >= MAX_FAILED_READS:
                    print("Camera stopped delivering frames - trying to reopen ...")
                    capture.release()
                    time.sleep(1.0)
                    try:
                        capture = open_checked_camera()
                        failed_reads = 0
                        print("Camera reopened.")
                    except RuntimeError as exc:
                        print(f"ERROR: could not reopen camera: {exc}")
                        break
                continue
            failed_reads = 0
            frame = new_frame

            frame_count += 1
            now = time.perf_counter()
            if now - fps_timer >= 1.0:
                fps_display = frame_count / (now - fps_timer)
                frame_count = 0
                fps_timer = now

        if frame is None:
            continue

        # ----------------------------------------------------
        # DETECT + POSE
        # ----------------------------------------------------
        view, results = process_frame(frame)
        elapsed = time.perf_counter() - start_time

        if any(r["id"] == TARGET_ID for r in results):
            last_seen_time = elapsed

        status = "PAUSED" if paused else f"{fps_display:.1f} fps"
        if last_seen_time is not None and not any(r["id"] == TARGET_ID for r in results):
            status += f"   marker lost for {elapsed - last_seen_time:.1f} s"
        draw_text(view, status, (10, view.shape[0] - 15), WHITE, 0.6)
        cv2.imshow("ArUco pose  (q = quit, s = snapshot, p = pause)", view)

        # ----------------------------------------------------
        # PRINT + LOG
        # ----------------------------------------------------
        if not paused and elapsed - last_print_time >= PRINT_INTERVAL:
            if results:
                for result in results:
                    print(f"[{elapsed:7.2f} s] {format_result(result)}")
            else:
                print(f"[{elapsed:7.2f} s] no target marker")
            last_print_time = elapsed

        if writer is not None and not paused:
            for result in results:
                t = result["tvec"]
                r = result["rvec"]
                writer.writerow([round(elapsed, 4), result["id"], t[0], t[1], t[2], float(np.linalg.norm(t)),
                                 r[0], r[1], r[2], result["yaw_deg"], result["reproj_error_px"], result["side_px"]])

        # ----------------------------------------------------
        # KEYS
        # ----------------------------------------------------
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("p"):
            paused = not paused
        if key == ord("s") and frame is not None:
            path = repo_path(os.path.join("results", f"snapshot_{int(time.time())}.png"))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            cv2.imwrite(path, frame)
            print(f"Saved {path}")

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    capture.release()
    cv2.destroyAllWindows()
    if log_file is not None:
        log_file.close()
        print(f"Pose log saved to {LOG_FILE}")
    print("Camera released.")
