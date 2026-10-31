"""
Define the TARGET point and the WORLD frame used by the controller.

Procedure:
  1. Place the drone (marker underneath, printed TOP edge towards the drone's nose) exactly where it
     should hover/land, with its nose pointing in the direction you want to call "forward".
     Hold it level above the camera (by hand, ~40-80 cm high) or put it on a transparent stand.
  2. Run:   python tools\\set_target.py
  3. When the marker is detected steadily, press  t . The program averages 30 measurements and saves
     results\\target.json :
        world origin  = this marker position (x, y)
        world +X      = the drone's current forward direction,  +Y = left,  +Z = up
  4. Move the drone by hand and check the ERROR line: moving it BACKWARDS must make e_forward
     POSITIVE (it must go forward to return), moving it RIGHT must make e_left POSITIVE.

Usage:
    python tools\\set_target.py           # live: press t to record, q to quit
    python tools\\set_target.py --show    # print the saved target and exit
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path                    # noqa: E402
from common.overlay import draw_text                                # noqa: E402
from vision.marker_tracker import MarkerTracker                     # noqa: E402
from vision.target import load_target, save_target                  # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Record the target point and the forward direction.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--show", action="store_true", help="print the saved target and exit")
parser.add_argument("--samples", type=int, default=30, help="number of measurements to average")
args = parser.parse_args()

config = load_config(args.config)
TARGET_FILE = repo_path(config["target"]["file"])

if args.show:
    target = load_target(TARGET_FILE)
    if target is None:
        print(f"No target saved yet ({TARGET_FILE}).")
    else:
        print(f"Target file: {TARGET_FILE}")
        print(f"  recorded       : {target.get('created')}")
        print(f"  forward heading: {target['heading_ref_deg']:+.1f} deg in the camera frame")
        print(f"  target x, y    : {target['target_cam_xy_m'][0] * 100:+.1f}, {target['target_cam_xy_m'][1] * 100:+.1f} cm (camera frame)")
        print(f"  hover height   : {config['target']['hover_height_m'] * 100:.0f} cm (config target.hover_height_m)")
    sys.exit(0)


# ============================================================
# START TRACKER
# ============================================================

try:
    tracker = MarkerTracker(config)
    tracker.start()
except (RuntimeError, FileNotFoundError) as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)

print("Place the drone at the target point, nose pointing 'forward', hold still, press t.  q = quit")

collecting = False
samples_xy = []
samples_z = []
samples_heading = []
last_index = -1
message = ""
message_until = 0.0


# ============================================================
# MAIN LOOP
# ============================================================

try:
    while tracker.is_running():
        state = tracker.get_state()
        view = tracker.get_frame()

        if collecting and state.detected and state.frame_index != last_index:
            last_index = state.frame_index
            samples_xy.append(state.camera_xyz_m[0:2])
            samples_z.append(state.camera_xyz_m[2])
            samples_heading.append(state.heading_cam_rad)
            if len(samples_xy) >= args.samples:
                collecting = False
                xy = np.mean(samples_xy, axis=0)
                spread_cm = float(np.max(np.linalg.norm(np.array(samples_xy) - xy, axis=1)) * 100)
                # circular mean of the heading angles
                heading = math.atan2(np.mean(np.sin(samples_heading)), np.mean(np.cos(samples_heading)))
                data = save_target(TARGET_FILE, heading, xy, float(np.mean(samples_z)), len(samples_xy))
                tracker.set_target(load_target(TARGET_FILE))
                message = f"SAVED: heading {data['heading_ref_deg']:+.1f} deg, spread {spread_cm:.1f} cm"
                message_until = time.time() + 4
                print(f"Saved {TARGET_FILE}: forward heading {data['heading_ref_deg']:+.1f} deg, "
                      f"x={xy[0] * 100:+.1f} cm, y={xy[1] * 100:+.1f} cm, height when recorded "
                      f"{np.mean(samples_z) * 100:.1f} cm, max spread {spread_cm:.1f} cm")
                if spread_cm > 2.0:
                    print("WARNING: the marker moved more than 2 cm while recording - hold it stiller and record again.")
                print("Now move the drone by hand and check the ERROR line (see the top of this file).")

        if view is not None:
            if collecting:
                draw_text(view, f"recording {len(samples_xy)}/{args.samples} - hold still", (10, view.shape[0] - 45), (0, 200, 255), 0.7)
            elif time.time() < message_until:
                draw_text(view, message, (10, view.shape[0] - 45), (0, 255, 0), 0.7)
            else:
                draw_text(view, "t = record target here   q = quit", (10, view.shape[0] - 45), (255, 255, 255), 0.7)
            cv2.imshow("set target", view)

        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("t"):
            if not state.detected:
                print("Marker not detected - cannot record.")
            else:
                samples_xy, samples_z, samples_heading = [], [], []
                collecting = True
                print(f"Recording {args.samples} measurements ...")

finally:
    tracker.stop()
    cv2.destroyAllWindows()
