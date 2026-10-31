"""
Save / load the camera calibration as a NumPy .npz file.

Stored keys:
  camera_matrix (3x3), dist_coeffs (1x5), image_width, image_height, rms_error_px,
  per_image_errors_px, num_images, square_size_mm, pattern_cols, pattern_rows,
  fourcc, created (ISO time string)
"""

import datetime
import os

import numpy as np


def save_calibration(path, camera_matrix, dist_coeffs, image_size, rms_error, per_image_errors,
                     square_size_mm, pattern_size, fourcc):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    np.savez(
        path,
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        dist_coeffs=np.asarray(dist_coeffs, dtype=np.float64),
        image_width=int(image_size[0]),
        image_height=int(image_size[1]),
        rms_error_px=float(rms_error),
        per_image_errors_px=np.asarray(per_image_errors, dtype=np.float64),
        num_images=int(len(per_image_errors)),
        square_size_mm=float(square_size_mm),
        pattern_cols=int(pattern_size[0]),
        pattern_rows=int(pattern_size[1]),
        fourcc=str(fourcc),
        created=datetime.datetime.now().isoformat(timespec="seconds"),
    )


def load_calibration(path):
    """Return a dict with the calibration; raises FileNotFoundError with a helpful message."""
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"Calibration file not found: {path}\n"
            "Run the calibration first:\n"
            "  python3 calibration/capture_calibration_images.py\n"
            "  python3 calibration/calibrate_camera.py"
        )
    data = np.load(path, allow_pickle=False)
    calibration = {key: data[key] for key in data.files}
    calibration["camera_matrix"] = calibration["camera_matrix"].astype(np.float64)
    calibration["dist_coeffs"] = calibration["dist_coeffs"].astype(np.float64)
    calibration["image_width"] = int(calibration["image_width"])
    calibration["image_height"] = int(calibration["image_height"])
    calibration["rms_error_px"] = float(calibration["rms_error_px"])
    return calibration
