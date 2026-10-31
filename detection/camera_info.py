"""
PROGRAM 1 - camera_info.py
Find the Logitech C920, check its resolution / frame rate, set its image settings.

Usage (from the autonomous_drone_landing folder, with the .venv active):
    python detection\\camera_info.py --list          # list cameras (index + name)
    python detection\\camera_info.py                 # open the camera from the config, live view
    python detection\\camera_info.py --device 1      # try another index without editing the config
    python detection\\camera_info.py --settings      # open the C920's own settings window

Keys in the video window:
    q / ESC  quit
    c        open the camera settings window (focus, exposure, gain, zoom, power-line frequency)
    p        print the current settings as config lines (copy them into camera_config.yaml)
    s        save a snapshot to results\\
"""

import argparse
import os
import sys
import time

import cv2

# Allow `python detection\camera_info.py` from the repo folder to find the `common` package.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, parse_device, repo_path              # noqa: E402
from common.camera import (list_camera_names, probe_cameras, open_camera,   # noqa: E402
                           measure_fps, read_controls, open_settings_dialog)
from common.overlay import draw_text                                        # noqa: E402


# ============================================================
# ARGUMENTS
# ============================================================

parser = argparse.ArgumentParser(description="Inspect and test the webcam.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--device", default=None, help="override the camera index from the config")
parser.add_argument("--list", action="store_true", help="list cameras and exit")
parser.add_argument("--settings", action="store_true", help="open the camera's settings window first")
args = parser.parse_args()

config = load_config(args.config)
camera_config = config["camera"]
if args.device is not None:
    camera_config["device"] = parse_device(args.device)
DEVICE = camera_config["device"]


# ============================================================
# HELPERS
# ============================================================

def print_cameras():
    print("\n=== Cameras on this PC ===")
    names = list_camera_names()
    if names:
        for index, name in enumerate(names):
            marker = "   <-- configured" if index == DEVICE else ""
            print(f"  index {index}: {name}{marker}")
    else:
        print("  (camera names need the 'pygrabber' package: pip install pygrabber)")
    print("\n  Probing indices 0-5 (which ones deliver images) ...")
    found = probe_cameras(5, camera_config.get("backend", "dshow"))
    if not found:
        print("  no camera delivered an image - is the C920 plugged in? Is another app using it?")
    for index, width, height in found:
        print(f"  index {index}: works, default size {width}x{height}")
    print("\n  Put the C920's index into config\\camera_config.yaml -> camera.device")


def print_controls_as_config(capture):
    values = read_controls(capture)
    print("\n=== Current camera settings (as reported by the Windows driver) ===")
    print("Copy the lines you want to fix into config\\camera_config.yaml -> camera.controls:")
    print("  controls:")
    for name in ["autofocus", "focus", "auto_exposure", "exposure", "gain", "zoom"]:
        print(f"    {name}: {values[name]:g}")
    print("  (other values: " + ", ".join(f"{k}={v:g}" for k, v in values.items()
                                          if k not in ["autofocus", "focus", "auto_exposure", "exposure", "gain", "zoom"]) + ")")


# ============================================================
# LIST
# ============================================================

if args.list:
    print_cameras()
    sys.exit(0)


# ============================================================
# OPEN CAMERA
# ============================================================

print_cameras()
print(f"\n=== Opening camera {DEVICE} ===")
try:
    capture, actual = open_camera(camera_config)
except RuntimeError as exc:
    print(f"ERROR: {exc}")
    sys.exit(1)

print(f"  pixel format: {actual['fourcc']}   (want MJPG for 30 fps at 1280x720)")
print("\nMeasuring the real frame rate for 2 s ...")
real_fps = measure_fps(capture, 2.0)
print(f"  measured: {real_fps:.1f} fps")
if real_fps < 0.8 * float(camera_config["fps"]):
    print("  WARNING: lower than configured. Common causes: not MJPG, auto-exposure in dim light "
          "(the C920 then lengthens the exposure and drops the frame rate), or a USB hub.")

print_controls_as_config(capture)

if args.settings:
    if open_settings_dialog(capture):
        print("\nSettings window opened. Adjust focus/exposure/zoom there, close it, then press 'p' in the video window.")
    else:
        print("\nThe settings window is only available with backend 'dshow'.")

print("\nLive view:  q = quit,  c = settings window,  p = print settings,  s = snapshot")


# ============================================================
# LIVE VIEW
# ============================================================

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

        text = f"camera {DEVICE}   {frame.shape[1]}x{frame.shape[0]}  {actual['fourcc']}  {fps_display:.1f} fps"
        draw_text(frame, text, (10, 30), (0, 255, 0), 0.7)
        draw_text(frame, "q quit | c settings | p print settings | s snapshot", (10, 60), (255, 255, 255), 0.6)
        cv2.imshow("camera_info", frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("c"):
            if not open_settings_dialog(capture):
                print("Settings window not available (needs backend 'dshow').")
        if key == ord("p"):
            print_controls_as_config(capture)
        if key == ord("s"):
            path = repo_path(os.path.join("results", f"snapshot_{int(time.time())}.png"))
            os.makedirs(os.path.dirname(path), exist_ok=True)
            cv2.imwrite(path, frame)
            print(f"Saved {path}")

finally:
    capture.release()
    cv2.destroyAllWindows()
    print("Camera released.")
