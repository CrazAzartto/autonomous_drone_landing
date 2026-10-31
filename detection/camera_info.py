"""
PROGRAM 1 - camera_info.py
Find the Logitech C920, check what it supports, and confirm the right camera is opened.

Usage (from the repository folder):
    python3 detection/camera_info.py --list        # list /dev/video* devices and stable names
    python3 detection/camera_info.py --formats     # pixel formats / resolutions / fps (v4l2-ctl)
    python3 detection/camera_info.py --controls    # camera controls (exposure, focus, zoom ...)
    python3 detection/camera_info.py               # open the configured camera and show live video
    python3 detection/camera_info.py --device /dev/video2   # override the device from the config

Keys in the video window:
    q / ESC  quit
    s        save a snapshot to results/
"""

import argparse
import os
import subprocess
import sys
import time

import cv2

# Allow `python3 detection/camera_info.py` from the repo folder to find the `common` package.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, parse_device, repo_path          # noqa: E402
from common.camera import (list_video_devices, list_stable_device_names, run_v4l2ctl,  # noqa: E402
                           v4l2ctl_available, open_camera, measure_fps, device_node)
from common.overlay import draw_text                                     # noqa: E402


# ============================================================
# ARGUMENTS
# ============================================================

parser = argparse.ArgumentParser(description="Inspect and test the webcam.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--device", default=None, help="override camera device (index or /dev path)")
parser.add_argument("--list", action="store_true", help="list video devices and exit")
parser.add_argument("--formats", action="store_true", help="print supported formats and exit")
parser.add_argument("--controls", action="store_true", help="print camera controls and exit")
args = parser.parse_args()

config = load_config(args.config)
camera_config = config["camera"]
if args.device is not None:
    camera_config["device"] = parse_device(args.device)
DEVICE = camera_config["device"]


# ============================================================
# LIST DEVICES
# ============================================================

def print_devices():
    print("\n=== Video devices (/dev/video*) ===")
    devices = list_video_devices()
    if not devices:
        print("  none found - is the webcam plugged in?  (check: lsusb | grep -i logitech)")
    for node, name in devices:
        print(f"  {node:14} {name}")
    print("  Note: one webcam creates 2 nodes; usually the LOWER number gives the images.")

    print("\n=== Stable names (/dev/v4l/by-id) - recommended for the config ===")
    stable = list_stable_device_names()
    if not stable:
        print("  none")
    for link in stable:
        print(f"  {link}  ->  {os.path.realpath(link)}")

    if v4l2ctl_available():
        print("\n=== v4l2-ctl --list-devices ===")
        try:
            print(subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True,
                                 timeout=5).stdout)
        except (subprocess.SubprocessError, OSError) as exc:
            print(f"  could not run v4l2-ctl: {exc}")
    else:
        print("\n(Install v4l-utils for more detail:  sudo apt install v4l-utils)")


if args.list:
    print_devices()
    sys.exit(0)


# ============================================================
# FORMATS / CONTROLS (needs v4l2-ctl)
# ============================================================

if args.formats or args.controls:
    if not v4l2ctl_available():
        print("v4l2-ctl not found. Install it with:  sudo apt install v4l-utils")
        sys.exit(1)
    node = device_node(DEVICE)
    if args.formats:
        print(f"\n=== Supported formats of {node} (v4l2-ctl --list-formats-ext) ===")
        print(run_v4l2ctl(DEVICE, "--list-formats-ext"))
        print("Look for 'MJPG' with 1280x720 at 30 fps - that is the configuration recommended in docs/02.")
    if args.controls:
        print(f"\n=== Camera controls of {node} (v4l2-ctl --list-ctrls-menus) ===")
        print(run_v4l2ctl(DEVICE, "--list-ctrls-menus"))
        print("Only controls listed above exist on YOUR camera. Use these exact names in config/camera_config.yaml.")
    sys.exit(0)


# ============================================================
# OPEN CAMERA + LIVE VIEW
# ============================================================

print_devices()

print("\n=== Opening the configured camera ===")
try:
    capture, actual = open_camera(camera_config)
except RuntimeError as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)

print("\n=== Properties reported by OpenCV ===")
properties = {
    "frame width": cv2.CAP_PROP_FRAME_WIDTH,
    "frame height": cv2.CAP_PROP_FRAME_HEIGHT,
    "fps (driver)": cv2.CAP_PROP_FPS,
    "brightness": cv2.CAP_PROP_BRIGHTNESS,
    "contrast": cv2.CAP_PROP_CONTRAST,
    "saturation": cv2.CAP_PROP_SATURATION,
    "gain": cv2.CAP_PROP_GAIN,
    "auto exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
    "exposure": cv2.CAP_PROP_EXPOSURE,
    "autofocus": cv2.CAP_PROP_AUTOFOCUS,
    "focus": cv2.CAP_PROP_FOCUS,
    "zoom": cv2.CAP_PROP_ZOOM,
    "buffer size": cv2.CAP_PROP_BUFFERSIZE,
}
for name, prop in properties.items():
    print(f"  {name:15} {capture.get(prop)}")
print(f"  {'fourcc':15} {actual['fourcc']}")
print("  (OpenCV's numbers for exposure/focus are backend-specific; trust v4l2-ctl --get-ctrl instead.)")

print("\nMeasuring the real frame rate for 2 s ...")
real_fps = measure_fps(capture, 2.0)
print(f"  measured: {real_fps:.1f} fps")
if real_fps < 0.8 * float(camera_config["fps"]):
    print("  WARNING: lower than configured. Common causes: YUYV instead of MJPG, dim light with auto "
          "exposure (the C920 lengthens exposure and drops fps), or a USB 2 hub.")

print("\nLive view: wave your hand in front of the C920 to confirm it is the right camera. q = quit, s = snapshot")

failed_reads = 0
frame_count = 0
fps_display = 0.0
fps_timer = time.perf_counter()

try:
    while True:
        ok, frame = capture.read()
        if not ok or frame is None:
            failed_reads += 1
            if failed_reads > config["detection"]["max_failed_reads"]:
                print("ERROR: camera stopped delivering frames (unplugged?).")
                break
            time.sleep(0.01)
            continue
        failed_reads = 0

        frame_count += 1
        now = time.perf_counter()
        if now - fps_timer >= 1.0:
            fps_display = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now

        text = f"{frame.shape[1]}x{frame.shape[0]}  {actual['fourcc']}  {fps_display:.1f} fps  device {DEVICE}"
        draw_text(frame, text, (10, 30), (0, 255, 0), 0.7)
        cv2.imshow("camera_info  (q = quit, s = snapshot)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            path = repo_path(os.path.join("results", f"snapshot_{int(time.time())}.png"))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            cv2.imwrite(path, frame)
            print(f"Saved {path}")

finally:
    capture.release()
    cv2.destroyAllWindows()
    print("Camera released.")
