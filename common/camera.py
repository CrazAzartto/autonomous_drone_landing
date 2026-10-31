"""
Opening the webcam on Windows with OpenCV (DirectShow by default).

Why a shared function: calibration and pose estimation are only valid if the camera delivers
exactly the same resolution, pixel format and image settings in every program. open_camera()
requests the configured settings, applies the configured camera controls, and then CHECKS what
the driver actually delivers.
"""

import os
import sys
import time

import cv2


# ============================================================
# BACKENDS + CONTROLS
# ============================================================

BACKENDS = {
    "dshow": cv2.CAP_DSHOW,     # Windows DirectShow (recommended for the C920)
    "msmf": cv2.CAP_MSMF,       # Windows Media Foundation
    "v4l2": cv2.CAP_V4L2,       # Linux (kept so the code can still run on Ubuntu if ever needed)
    "any": cv2.CAP_ANY,
}

# Friendly names used in camera_config.yaml -> OpenCV property IDs.
# Order matters when applying: switch an AUTO mode off BEFORE writing the manual value.
CONTROL_PROPERTIES = {
    "autofocus": cv2.CAP_PROP_AUTOFOCUS,
    "focus": cv2.CAP_PROP_FOCUS,
    "auto_exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
    "exposure": cv2.CAP_PROP_EXPOSURE,
    "gain": cv2.CAP_PROP_GAIN,
    "zoom": cv2.CAP_PROP_ZOOM,
    "brightness": cv2.CAP_PROP_BRIGHTNESS,
    "contrast": cv2.CAP_PROP_CONTRAST,
    "saturation": cv2.CAP_PROP_SATURATION,
    "sharpness": cv2.CAP_PROP_SHARPNESS,
    "auto_white_balance": cv2.CAP_PROP_AUTO_WB,
    "white_balance": cv2.CAP_PROP_WB_TEMPERATURE,
    "backlight": cv2.CAP_PROP_BACKLIGHT,
}
APPLY_ORDER = ["autofocus", "focus", "auto_exposure", "exposure", "gain", "zoom", "brightness",
               "contrast", "saturation", "sharpness", "auto_white_balance", "white_balance", "backlight"]


def default_backend():
    return "dshow" if sys.platform.startswith("win") else "v4l2"


def backend_id(name):
    name = (name or "any").lower()
    if name not in BACKENDS:
        raise ValueError(f"Unknown camera backend '{name}'. Use one of: {', '.join(BACKENDS)}")
    return BACKENDS[name]


# ============================================================
# DEVICE DISCOVERY (Windows)
# ============================================================

def list_camera_names():
    """
    Names of the cameras in DirectShow order (index 0, 1, 2 ...), using the optional `pygrabber`
    package. Returns None if pygrabber is not available (then use probe_cameras()).
    """
    if not sys.platform.startswith("win"):
        return None
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception:
        return None


def probe_cameras(max_index=5, backend="dshow"):
    """Try indices 0..max_index; return [(index, width, height)] for cameras that deliver a frame."""
    found = []
    for index in range(max_index + 1):
        capture = cv2.VideoCapture(index, backend_id(backend))
        if capture.isOpened():
            ok, frame = capture.read()
            if ok and frame is not None:
                found.append((index, frame.shape[1], frame.shape[0]))
        capture.release()
    return found


# ============================================================
# CONTROLS
# ============================================================

def read_controls(capture):
    """Current value of every known control as reported by the driver (-1 / 0 often = unsupported)."""
    return {name: capture.get(prop) for name, prop in CONTROL_PROPERTIES.items()}


def apply_controls(capture, controls, verbose=True):
    """
    Apply {name: value} from the config (None = skip), in a safe order, then READ BACK each value
    so you can see whether the driver accepted it. Returns {name: (requested, read_back)}.
    """
    results = {}
    for name in APPLY_ORDER + [n for n in controls if n not in APPLY_ORDER]:
        if name not in controls or controls[name] is None:
            continue
        if name not in CONTROL_PROPERTIES:
            print(f"WARNING: unknown camera control '{name}' in config - skipped. "
                  f"Known: {', '.join(CONTROL_PROPERTIES)}")
            continue
        requested = float(controls[name])
        accepted = capture.set(CONTROL_PROPERTIES[name], requested)
        time.sleep(0.05)
        read_back = capture.get(CONTROL_PROPERTIES[name])
        results[name] = (requested, read_back)
        if verbose:
            status = "OK" if abs(read_back - requested) < 1e-3 else ("SET?" if accepted else "REJECTED")
            print(f"  [{status:8}] {name:18} requested {requested:8.2f}   driver reports {read_back:8.2f}")
    return results


def open_settings_dialog(capture):
    """Open the camera driver's own settings window (DirectShow only). Returns True if it opened."""
    return bool(capture.set(cv2.CAP_PROP_SETTINGS, 1))


# ============================================================
# OPEN + VERIFY
# ============================================================

def fourcc_to_text(value):
    value = int(value)
    text = "".join(chr((value >> (8 * i)) & 0xFF) for i in range(4))
    return text if text.isprintable() and text.strip() else f"0x{value:08X}"


def _configure(capture, camera_config):
    fourcc = cv2.VideoWriter_fourcc(*camera_config["fourcc"])
    capture.set(cv2.CAP_PROP_FOURCC, fourcc)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(camera_config["width"]))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(camera_config["height"]))
    capture.set(cv2.CAP_PROP_FPS, int(camera_config["fps"]))
    # Some DirectShow drivers only switch to MJPG if FOURCC is set again AFTER the size.
    capture.set(cv2.CAP_PROP_FOURCC, fourcc)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, int(camera_config.get("buffer_size", 1)))


def open_camera(camera_config, apply_camera_controls=True, verbose=True):
    """
    Open the camera (or a video file) with the configured backend and settings.

    Returns (capture, actual) where actual = {"width", "height", "fps", "fourcc", "is_file"} as
    reported by the driver. Raises RuntimeError if the camera cannot be opened or no frame arrives.
    """
    device = camera_config["device"]
    is_file = isinstance(device, str)

    if is_file:
        if not os.path.isfile(device):
            raise RuntimeError(f"Video file not found: {device}")
        capture = cv2.VideoCapture(device)
    else:
        capture = cv2.VideoCapture(device, backend_id(camera_config.get("backend") or default_backend()))

    if not capture.isOpened():
        raise RuntimeError(
            f"Could not open camera {device!r}. Check the index with "
            f"`python detection\\camera_info.py --list`, close other apps using the camera "
            f"(Camera app, Teams, Zoom, browser), and check Windows Settings > Privacy > Camera."
        )

    if not is_file:
        _configure(capture, camera_config)
        if apply_camera_controls and camera_config.get("controls"):
            if any(v is not None for v in camera_config["controls"].values()):
                if verbose:
                    print("Applying camera controls from config:")
                apply_controls(capture, camera_config["controls"], verbose=verbose)

    # Read a few frames: the first ones after opening can be dark or missing.
    frame = None
    for _ in range(20):
        ok, frame = capture.read()
        if ok and frame is not None:
            break
        frame = None
        time.sleep(0.05)
    if frame is None:
        capture.release()
        raise RuntimeError(f"Camera {device!r} opened but delivered no frames.")

    actual = {
        "width": frame.shape[1],
        "height": frame.shape[0],
        "fps": capture.get(cv2.CAP_PROP_FPS),
        "fourcc": fourcc_to_text(capture.get(cv2.CAP_PROP_FOURCC)),
        "is_file": is_file,
    }

    if verbose:
        if is_file:
            print(f"Video file {device}: {actual['width']}x{actual['height']} @ {actual['fps']:.0f} fps")
        else:
            print(f"Camera {device} ({camera_config.get('backend')}): requested {camera_config['width']}x"
                  f"{camera_config['height']} @ {camera_config['fps']} fps {camera_config['fourcc']}  ->  "
                  f"driver gives {actual['width']}x{actual['height']} @ {actual['fps']:.0f} fps {actual['fourcc']}")

    if not is_file and (actual["width"], actual["height"]) != (int(camera_config["width"]), int(camera_config["height"])):
        print("WARNING: the camera did NOT accept the configured resolution. Is this the right camera? "
              "Check with: python detection\\camera_info.py --list")

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
