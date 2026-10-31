"""
Loads config/camera_config.yaml and resolves file paths relative to the repository root.

Every program calls load_config() so that calibration and detection always share
exactly the same camera, checkerboard and marker settings.
"""

import os

import yaml


# ============================================================
# PATHS
# ============================================================

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG_PATH = os.path.join(REPO_ROOT, "config", "camera_config.yaml")

REQUIRED_KEYS = {
    "camera": ["device", "width", "height", "fps", "fourcc"],
    "checkerboard": ["inner_corners_cols", "inner_corners_rows", "square_size_mm"],
    "aruco": ["dictionary", "marker_id", "marker_length_mm"],
    "calibration": ["images_dir", "output_file", "min_images", "max_image_error_px"],
    "detection": ["print_rate_hz", "axis_length_ratio", "max_failed_reads", "log_file"],
}


# ============================================================
# HELPERS
# ============================================================

def repo_path(relative_or_absolute):
    """Turn a path from the YAML file into an absolute path (relative = relative to repo root)."""
    if os.path.isabs(relative_or_absolute):
        return relative_or_absolute
    return os.path.join(REPO_ROOT, relative_or_absolute)


def parse_device(device):
    """
    The camera device may be written as 0, "0", "/dev/video2" or a /dev/v4l/by-id/... path.
    OpenCV wants an int for an index and a string for a path.
    """
    if isinstance(device, int):
        return device
    text = str(device).strip()
    if text.isdigit():
        return int(text)
    return text


def load_config(config_path=None):
    """Read the YAML configuration and check that every required key is present."""
    path = config_path or DEFAULT_CONFIG_PATH

    if not os.path.isfile(path):
        raise FileNotFoundError(f"Configuration file not found: {path}")

    with open(path, "r") as file:
        config = yaml.safe_load(file)

    if not isinstance(config, dict):
        raise ValueError(f"Configuration file is empty or invalid: {path}")

    missing = []
    for section, keys in REQUIRED_KEYS.items():
        if section not in config or not isinstance(config[section], dict):
            missing.append(section)
            continue
        for key in keys:
            if key not in config[section]:
                missing.append(f"{section}.{key}")
    if missing:
        raise KeyError(f"Missing entries in {path}: {', '.join(missing)}")

    # Normalise a few values so the programs do not have to.
    config["camera"]["device"] = parse_device(config["camera"]["device"])
    config["camera"]["fourcc"] = str(config["camera"]["fourcc"]).upper()
    config["camera"].setdefault("buffer_size", 1)
    config["camera"]["v4l2_controls"] = config["camera"].get("v4l2_controls") or {}

    if float(config["aruco"]["marker_length_mm"]) <= 0:
        raise ValueError("aruco.marker_length_mm must be a positive number (measure your printed marker).")
    if float(config["checkerboard"]["square_size_mm"]) <= 0:
        raise ValueError("checkerboard.square_size_mm must be a positive number (measure your print).")

    config["_config_path"] = os.path.abspath(path)
    return config
