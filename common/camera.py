"""
Opening the webcam on Ubuntu with OpenCV's V4L2 backend.

Why a shared function: calibration and pose estimation are only valid if the camera
delivers exactly the same resolution, pixel format and controls in both programs.
open_camera() requests the configured settings, applies optional v4l2 controls and
then CHECKS what the driver actually delivers.
"""

import glob
import os
import shutil
import subprocess
import time

import cv2


# ============================================================
# DEVICE DISCOVERY (Linux)
# ============================================================

def list_video_devices():
    """
    Return [(device_path, name)] for every /dev/videoN node, using the names in /sys.
    Note: one physical webcam usually creates TWO nodes (e.g. /dev/video0 for video and
    /dev/video1 for metadata). Only the first one delivers images.
    """
    devices = []
    for node in sorted(glob.glob("/dev/video*"), key=lambda p: int("".join(c for c in p if c.isdigit()) or 0)):
        index = node.replace("/dev/video", "")
        name_file = f"/sys/class/video4linux/video{index}/name"
        name = "unknown"
        if os.path.isfile(name_file):
            with open(name_file) as file:
                name = file.read().strip()
        devices.append((node, name))
    return devices


def list_stable_device_names():
    """Return the /dev/v4l/by-id/... links (stable across reboots and USB ports)."""
    return sorted(glob.glob("/dev/v4l/by-id/*"))


def device_node(device):
    """Return a /dev path for v4l2-ctl from an int index or a path."""
    if isinstance(device, int):
        return f"/dev/video{device}"
    return os.path.realpath(device)


def v4l2ctl_available():
    return shutil.which("v4l2-ctl") is not None


def run_v4l2ctl(device, *arguments):
    """Run v4l2-ctl and return its text output ('' if v4l2-ctl is not installed or fails)."""
    if not v4l2ctl_available():
        return ""
    command = ["v4l2-ctl", "-d", device_node(device), *arguments]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
        return result.stdout + result.stderr
    except (subprocess.SubprocessError, OSError) as exc:
        return f"v4l2-ctl failed: {exc}"


def available_controls(device):
    """Return the set of control names the camera reports (empty set if v4l2-ctl is missing)."""
    names = set()
    for line in run_v4l2ctl(device, "--list-ctrls").splitlines():
        line = line.strip()
        if " 0x" in line:
            names.add(line.split()[0])
    return names


def apply_v4l2_controls(device, controls, verbose=True):
    """
    Apply {control_name: value} in the given order, skipping controls the camera does not have,
    then read every value back so you can SEE that it actually changed.
    Returns {control_name: value_read_back}.
    """
    if not controls:
        return {}
    if not v4l2ctl_available():
        print("WARNING: v4l2-ctl not found (sudo apt install v4l-utils); camera controls NOT applied.")
        return {}

    supported = available_controls(device)
    read_back = {}

    for name, value in controls.items():
        if value is None:
            continue
        if name not in supported:
            print(f"WARNING: control '{name}' is not reported by this camera - skipped.")
            continue
        run_v4l2ctl(device, f"--set-ctrl={name}={int(value)}")
        time.sleep(0.05)
        output = run_v4l2ctl(device, f"--get-ctrl={name}").strip()
        read_back[name] = output.split(":")[-1].strip() if ":" in output else output
        if verbose:
            status = "OK" if read_back[name] == str(int(value)) else "CHECK"
            print(f"  [{status:5}] {name:30} requested {int(value):6}   read back {read_back[name]}")
    return read_back


# ============================================================
# OPEN + VERIFY
# ============================================================

def fourcc_to_text(value):
    value = int(value)
    return "".join(chr((value >> (8 * i)) & 0xFF) for i in range(4))


def open_camera(camera_config, apply_controls=True, verbose=True):
    """
    Open the webcam with the V4L2 backend and the configured settings.

    Returns (capture, actual) where actual = {"width", "height", "fps", "fourcc"} as reported
    by the driver. Raises RuntimeError if the camera cannot be opened or no frame arrives.
    """
    device = camera_config["device"]

    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(
            f"Could not open camera '{device}'. Check the device with "
            f"`python3 detection/camera_info.py --list`, that no other program uses it, "
            f"and that your user is in the 'video' group."
        )

    # Order matters for V4L2: pixel format first, then size, then frame rate.
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*camera_config["fourcc"]))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera_config["width"]))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera_config["height"]))
    capture.set(cv2.CAP_PROP_FPS, int(camera_config["fps"]))
    capture.set(cv2.CAP_PROP_BUFFERSIZE, int(camera_config.get("buffer_size", 1)))

    # Controls are applied AFTER the format is set, because changing the format can reset them.
    if apply_controls and camera_config.get("v4l2_controls"):
        if verbose:
            print("Applying v4l2 controls from config:")
        apply_v4l2_controls(device, camera_config["v4l2_controls"], verbose=verbose)

    # Read a few frames: the first ones after opening can be dark or missing.
    frame = None
    for _ in range(10):
        ok, frame = capture.read()
        if ok and frame is not None:
            break
        time.sleep(0.05)
    if frame is None:
        capture.release()
        raise RuntimeError(f"Camera '{device}' opened but delivered no frames.")

    actual = {
        "width": frame.shape[1],
        "height": frame.shape[0],
        "fps": capture.get(cv2.CAP_PROP_FPS),
        "fourcc": fourcc_to_text(capture.get(cv2.CAP_PROP_FOURCC)),
    }

    if verbose:
        print(f"Camera '{device}': requested {camera_config['width']}x{camera_config['height']} "
              f"@ {camera_config['fps']} fps {camera_config['fourcc']}  ->  driver gives "
              f"{actual['width']}x{actual['height']} @ {actual['fps']:.0f} fps {actual['fourcc']}")

    if (actual["width"], actual["height"]) != (int(camera_config["width"]), int(camera_config["height"])):
        print("WARNING: the camera did NOT accept the configured resolution. "
              "Check supported sizes with: python3 detection/camera_info.py --formats")

    return capture, actual


def measure_fps(capture, seconds=2.0):
    """Count frames actually received during `seconds` (the driver's fps value can be wrong)."""
    count = 0
    start = time.perf_counter()
    while time.perf_counter() - start < seconds:
        ok, _ = capture.read()
        if ok:
            count += 1
    return count / (time.perf_counter() - start)
