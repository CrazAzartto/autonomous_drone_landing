"""
PROGRAM 4 - aruco_pose_estimation.py
Live ArUco detection + 3-D pose + (after tools\\set_target.py) world position and drone errors.

Usage (from the autonomous_drone_landing folder, with the .venv active):
    python detection\\aruco_pose_estimation.py                    # live camera from the config
    python detection\\aruco_pose_estimation.py --log              # also write results\\pose_log.csv
    python detection\\aruco_pose_estimation.py --video clip.mp4   # replay a recorded video
    python detection\\aruco_pose_estimation.py --image photo.png  # one still image

Keys:  q / ESC quit,   s snapshot,   r start/stop recording the RAW camera video (results\\recording_*.avi,
       replay it later with --video to test changes without flying)

Camera frame (OpenCV convention), shown as CAMERA X / Y / Z:
    X > 0 : marker to the RIGHT of the image centre     Y > 0 : marker BELOW the image centre
    Z     : distance along the optical axis = height of the marker above the lens (camera on the floor)
World frame (after set_target), shown as WORLD x / y / z:  x = forward, y = left, z = up, origin = target.
ERROR line: how far the drone must go forward / left / up and turn left (drone's own frame).
"""

import argparse
import csv
import math
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path                 # noqa: E402
from vision.marker_tracker import MarkerProcessor, MarkerTracker  # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Live ArUco marker pose estimation.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--image", default=None, help="process one image file instead of the camera")
parser.add_argument("--video", default=None, help="process a video file instead of the camera")
parser.add_argument("--log", action="store_true", help="write every frame's result to the CSV log")
args = parser.parse_args()

config = load_config(args.config)
PRINT_INTERVAL = 1.0 / float(config["detection"]["print_rate_hz"])
LOG_FILE = repo_path(config["detection"]["log_file"])


def describe(state):
    """One line of text for the terminal."""
    if not state.detected:
        return "no marker" + (f" ({state.rejected_reason})" if state.rejected_reason else "") + \
               (f" - predicting, lost {state.age_s * 1000:.0f} ms" if state.valid else "")
    x, y, z = state.camera_xyz_m * 100
    text = (f"CAM X={x:+6.1f} Y={y:+6.1f} Z={z:6.1f} cm  head={math.degrees(state.heading_cam_rad):+6.1f} deg"
            f"  reproj={state.reproj_error_px:.2f}px")
    if state.errors is not None and state.errors.valid:
        e = state.errors
        text += (f" | ERR fwd={e.e_forward_m * 100:+5.1f} left={e.e_left_m * 100:+5.1f} up={e.e_up_m * 100:+5.1f} cm"
                 f" yaw={math.degrees(e.e_yaw_rad):+5.1f} deg")
    return text


# ============================================================
# STILL IMAGE MODE
# ============================================================

if args.image:
    try:
        processor = MarkerProcessor(config)
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)
    frame = cv2.imread(args.image)
    if frame is None:
        print(f"ERROR: cannot read image {args.image}")
        sys.exit(1)
    calib = processor.calibration
    if (frame.shape[1], frame.shape[0]) != (calib["image_width"], calib["image_height"]):
        print(f"WARNING: image is {frame.shape[1]}x{frame.shape[0]}, calibration is "
              f"{calib['image_width']}x{calib['image_height']} - the pose will be WRONG.")
    state, view = processor.process(frame)
    print(describe(state))
    out_path = repo_path(os.path.join("results", "pose_result.png"))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, view)
    print(f"Annotated image saved to {out_path}")
    if not os.environ.get("NO_DISPLAY"):
        cv2.imshow("pose (any key = close)", view)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    sys.exit(0)


# ============================================================
# START TRACKER (live camera or video file)
# ============================================================

if args.video:
    config["camera"]["device"] = args.video

try:
    tracker = MarkerTracker(config)
    tracker.start()
except (RuntimeError, FileNotFoundError) as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)

print(f"Marker: {config['aruco']['dictionary']} ID {config['aruco']['marker_id']}, "
      f"{config['aruco']['marker_length_mm']} mm.  Target set: {tracker.processor.target is not None}")
print("Running.  q = quit,  s = snapshot,  r = record video\n")


# ============================================================
# CSV LOGGER
# ============================================================

log_file = None
writer = None
if args.log:
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    log_file = open(LOG_FILE, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["time_s", "detected", "valid", "age_s", "cam_x_m", "cam_y_m", "cam_z_m", "heading_cam_deg",
                     "reproj_px", "world_x_m", "world_y_m", "world_z_m", "yaw_world_deg",
                     "filt_x_m", "filt_y_m", "filt_z_m", "vel_x_mps", "vel_y_mps", "vel_z_mps",
                     "e_forward_m", "e_left_m", "e_up_m", "e_yaw_deg", "x_n", "y_n", "a_n", "alpha_deg"])
    print(f"Logging to {LOG_FILE}")


def nan_list(values, n):
    return list(values) if values is not None else [float("nan")] * n


# ============================================================
# MAIN LOOP (display + print + log; the tracking runs in its own thread)
# ============================================================

start_time = time.perf_counter()
last_print = 0.0
last_logged_index = -1
last_recorded_index = -1
video_writer = None

try:
    while tracker.is_running():
        state = tracker.get_state()
        view = tracker.get_frame()
        elapsed = time.perf_counter() - start_time

        if view is not None:
            if video_writer is not None:
                raw = tracker.get_raw_frame()
                if raw is not None and state.frame_index != last_recorded_index:
                    video_writer.write(raw)          # raw image -> can be replayed with --video
                    last_recorded_index = state.frame_index
            cv2.imshow("ArUco pose  (q quit | s snapshot | r record)", view)

        if elapsed - last_print >= PRINT_INTERVAL:
            print(f"[{elapsed:7.2f} s] {describe(state)}")
            last_print = elapsed

        if writer is not None and state.frame_index != last_logged_index:
            last_logged_index = state.frame_index
            e = state.errors
            f = state.features
            writer.writerow([round(state.t - start_time, 4), int(state.detected), int(state.valid), state.age_s,
                             *nan_list(state.camera_xyz_m, 3),
                             math.degrees(state.heading_cam_rad) if state.heading_cam_rad is not None else float("nan"),
                             state.reproj_error_px, *nan_list(state.world_xyz_m, 3),
                             math.degrees(state.yaw_world_rad) if state.yaw_world_rad is not None else float("nan"),
                             *nan_list(state.filtered_xyz_m, 3), *nan_list(state.filtered_vel_mps, 3),
                             *(nan_list(None, 3) if e is None else [e.e_forward_m, e.e_left_m, e.e_up_m]),
                             math.degrees(e.e_yaw_rad) if e is not None else float("nan"),
                             *(nan_list(None, 4) if f is None else [f["x_n"], f["y_n"], f["a_n"], math.degrees(f["alpha_rad"])])])

        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s") and view is not None:
            path = repo_path(os.path.join("results", f"snapshot_{int(time.time())}.png"))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            cv2.imwrite(path, view)
            print(f"Saved {path}")
        if key == ord("r") and view is not None:
            if video_writer is None:
                path = repo_path(os.path.join("results", f"recording_{int(time.time())}.avi"))
                video_writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (view.shape[1], view.shape[0]))
                print(f"Recording to {path} (press r again to stop)")
            else:
                video_writer.release()
                video_writer = None
                print("Recording stopped.")

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    tracker.stop()
    if video_writer is not None:
        video_writer.release()
    cv2.destroyAllWindows()
    if log_file is not None:
        log_file.close()
        print(f"Log saved to {LOG_FILE}")
    print("Camera released.")
