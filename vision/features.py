"""
Image-moment features of Papers 1 and 2 (Hegazy/Boby/Klimchik; Ingale/Paulson/Boby):

    s = [x_n, y_n, a_n, alpha]

computed from the four ArUco corners:
    normalised coordinates  x = (u - cx)/fx, y = (v - cy)/fy   (here: undistorted with the calibration)
    centroid                x_g = mean(x), y_g = mean(y)
    "area"                  a = mu20 + mu02  (centred second-order moments)
    desired area            a* = 2 L^2 / Z*^2  (square of side L seen face-on at depth Z*)
    normalised features     a_n = Z* sqrt(a*/a),  x_n = a_n x_g,  y_n = a_n y_g
    orientation             alpha = atan2(y_1 - y_0, x_1 - x_0)   (corner-based: moment orientation
                                                                   is undefined for a square)
For a camera parallel to the marker: a_n = Z, x_n = X, y_n = Y (metres, camera frame).
They are provided so the controller can be written in the papers' feature space if wanted.
"""

import math

import cv2
import numpy as np


def image_moment_features(corners_px, camera_matrix, dist_coeffs, marker_length_m, desired_depth_m):
    corners = np.asarray(corners_px, dtype=np.float64).reshape(-1, 1, 2)
    normalised = cv2.undistortPoints(corners, camera_matrix, dist_coeffs).reshape(-1, 2)

    x_g, y_g = normalised.mean(axis=0)
    mu20 = float(np.sum((normalised[:, 0] - x_g) ** 2))
    mu02 = float(np.sum((normalised[:, 1] - y_g) ** 2))
    area = mu20 + mu02
    if area <= 0:
        return None

    desired_area = 2.0 * marker_length_m ** 2 / desired_depth_m ** 2
    a_n = desired_depth_m * math.sqrt(desired_area / area)
    alpha = math.atan2(normalised[1, 1] - normalised[0, 1], normalised[1, 0] - normalised[0, 0])

    return {
        "x_n": a_n * x_g,
        "y_n": a_n * y_g,
        "a_n": a_n,
        "alpha_rad": alpha,
        "x_g": float(x_g),
        "y_g": float(y_g),
        "area": area,
        "desired_area": desired_area,
    }
