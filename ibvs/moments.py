"""
Discrete image moments of a set of n image points, exactly as defined in [P1] Eq.(2), p.3:

    m_ij  = sum_k  x_k^i * y_k^j
    mu_ij = sum_k (x_k - x_g)^i * (y_k - y_g)^j        with  x_g = m10/n,  y_g = m01/n,  m00 = n

The points are the 4 ArUco corners in NORMALISED image coordinates (x = X/Z, y = Y/Z), i.e. already
undistorted with the camera calibration (see features.py). Centred moments are invariant to 2-D
translation, and mu20 + mu02 is invariant to 2-D rotation as well ([P1] p.4, citing Tahri & Chaumette).
"""

import numpy as np


def discrete_moments(points):
    """
    points : (n, 2) array of normalised image points (x, y).
    Returns a dict with m00, m10, m01, m20, m02, m11, x_g, y_g, mu20, mu02, mu11.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if points.shape[0] < 1:
        raise ValueError("discrete_moments needs at least one point")
    x = points[:, 0]
    y = points[:, 1]
    n = float(points.shape[0])

    m10 = float(np.sum(x))
    m01 = float(np.sum(y))
    x_g = m10 / n
    y_g = m01 / n
    dx = x - x_g
    dy = y - y_g

    return {
        "m00": n,
        "m10": m10,
        "m01": m01,
        "m20": float(np.sum(x * x)),
        "m02": float(np.sum(y * y)),
        "m11": float(np.sum(x * y)),
        "x_g": x_g,
        "y_g": y_g,
        "mu20": float(np.sum(dx * dx)),
        "mu02": float(np.sum(dy * dy)),
        "mu11": float(np.sum(dx * dy)),
    }
