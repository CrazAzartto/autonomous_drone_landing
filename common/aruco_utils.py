"""
ArUco detection and marker pose estimation (OpenCV >= 4.7 API only).

API notes (tested with opencv-contrib-python 4.10 and 4.14):
  - Detection uses the object API:  cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers()
    (the old free function cv2.aruco.detectMarkers() is deprecated and is NOT used here).
  - Pose uses cv2.solvePnP(..., flags=cv2.SOLVEPNP_IPPE_SQUARE) on the four marker corners.
    (cv2.aruco.estimatePoseSingleMarkers() is deprecated since OpenCV 4.7 - it still exists in the
     contrib build but OpenCV recommends solvePnP, so it is NOT used.)
"""

import math

import cv2
import numpy as np


# ============================================================
# VERSION CHECK
# ============================================================

def check_opencv_aruco():
    """Fail early with a clear message if the installed OpenCV lacks the new ArUco API."""
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise RuntimeError(
            f"OpenCV {cv2.__version__} does not provide cv2.aruco.ArucoDetector. "
            "Install the contrib build, version 4.10 or newer:  "
            "pip uninstall -y opencv-python && pip install -r requirements.txt"
        )


# ============================================================
# DETECTOR
# ============================================================

def get_dictionary(dictionary_name):
    """Turn 'DICT_4X4_50' into the OpenCV dictionary object."""
    if not hasattr(cv2.aruco, dictionary_name):
        valid = [n for n in dir(cv2.aruco) if n.startswith("DICT_")]
        raise ValueError(f"Unknown ArUco dictionary '{dictionary_name}'. Valid names: {', '.join(valid)}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))


def create_detector(dictionary_name):
    """ArUco detector with sub-pixel corner refinement (more precise corners -> better pose)."""
    check_opencv_aruco()
    dictionary = get_dictionary(dictionary_name)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def detect_markers(detector, image_bgr):
    """
    Returns (corners_list, ids) where corners_list[i] is a (4, 2) float32 array in the order
    top-left, top-right, bottom-right, bottom-left (of the marker as printed), and ids is a
    1-D int array. Returns ([], empty array) when nothing is found.
    """
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY) if image_bgr.ndim == 3 else image_bgr
    corners, ids, _rejected = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return [], np.array([], dtype=int)
    corners_list = [np.asarray(c, dtype=np.float32).reshape(4, 2) for c in corners]
    return corners_list, np.asarray(ids).reshape(-1).astype(int)


# ============================================================
# POSE
# ============================================================

def marker_object_points(marker_length_m):
    """
    3-D corner coordinates in the MARKER frame (metres), in the same order ArUco returns
    image corners. Marker frame: origin at the marker centre, +x to the marker's right,
    +y to the marker's top, +z out of the printed face (towards the camera).
    This order is the one required by cv2.SOLVEPNP_IPPE_SQUARE.
    """
    half = marker_length_m / 2.0
    return np.array([
        [-half,  half, 0.0],   # top-left
        [ half,  half, 0.0],   # top-right
        [ half, -half, 0.0],   # bottom-right
        [-half, -half, 0.0],   # bottom-left
    ], dtype=np.float32)


def estimate_marker_pose(corners_px, marker_length_m, camera_matrix, dist_coeffs):
    """
    Solve the Perspective-n-Point problem for one square marker.

    Returns dict with:
      rvec, tvec      : marker pose in the CAMERA frame (tvec in metres)
      reproj_error_px : RMS distance between detected and re-projected corners (quality check)
      yaw_deg         : rotation of the marker about the camera's optical axis
    or None if solvePnP fails.
    """
    object_points = marker_object_points(marker_length_m)
    image_points = np.asarray(corners_px, dtype=np.float32).reshape(4, 2)

    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None

    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    reproj_error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - image_points) ** 2, axis=1))))

    rotation, _ = cv2.Rodrigues(rvec)
    # Angle of the marker's +x axis projected onto the camera's x-y plane.
    yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))

    return {
        "rvec": rvec.reshape(3),
        "tvec": tvec.reshape(3),
        "reproj_error_px": reproj_error,
        "yaw_deg": yaw_deg,
    }
