"""Shared synthetic geometry for the tests (marker facing the upward camera, parallel to the image)."""
import math

import numpy as np


def marker_corners_cam(center, alpha, length):
    """3-D corners (TL, TR, BR, BL) of a marker whose x_M axis points at angle alpha in the camera x-y plane."""
    x_m = np.array([math.cos(alpha), math.sin(alpha), 0.0])
    y_m = np.array([math.sin(alpha), -math.cos(alpha), 0.0])      # x_M rotated by -90 deg (facing the camera)
    h = length / 2.0
    return np.array([center - h * x_m + h * y_m, center + h * x_m + h * y_m,
                     center + h * x_m - h * y_m, center - h * x_m - h * y_m], dtype=float)


def project(points):
    points = np.asarray(points, dtype=float)
    return points[:, :2] / points[:, 2:3]
