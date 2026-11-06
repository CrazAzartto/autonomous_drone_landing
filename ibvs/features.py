"""
Visual features of [P1] (Hegazy, Boby, Klimchik), p.4-5:

    a      = mu20 + mu02                                  "area" (invariant to 2-D rotation/translation)
    a_n    = z* * sqrt(a* / a)                            normalised area
    x_n    = a_n * x_g ,   y_n = a_n * y_g                normalised centre of gravity
    alpha  = atan2(y_b - y_a, x_b - x_a)                  marker orientation from two corners
    s      = [x_n, y_n, a_n, alpha]                       feature vector (4 x 1)
    s*     = [z* x_g*, z* y_g*, z*, alpha*]               desired features (at the desired image a = a*)
    e      = s - s*                                       error (alpha difference wrapped to (-pi, pi])

a* is the area of the marker in the DESIRED image and z* the desired depth. They are either taken from a
"teach" recording (the learning step of [P1] Sec. 4) or from the model of a square of side L seen face-on
at depth z*: four corners at (+-L/2z*, +-L/2z*) give mu20 = mu02 = L^2/z*^2, so a* = 2 L^2 / z*^2.

Property used throughout (proof in docs/06 §3): when the marker plane is parallel to the image plane and
a*, z* are consistent, a_n = Z, x_n = X, y_n = Y, i.e. the features equal the marker position in the
camera frame in metres. With a taught a* the controller converges to the taught IMAGE regardless of small
calibration/size errors, which is the robustness argument of IBVS in [P1] Sec. 1.

Orientation ambiguity in [P1]: the text says "the angle between the first corner and the third corner",
the printed formula is atan2(y2 - y1, x2 - x1). Both are supported (alpha_definition):
    "edge_0_1"     : corners 0 -> 1 (formula as printed, 1-based indices 1,2)  [default]
    "diagonal_0_2" : corners 0 -> 2 (text: first and third corner)
The choice only adds a constant offset, which cancels in alpha - alpha* as long as alpha* is computed the
same way (it always is in this code).
"""

import json
import math
import os
from dataclasses import asdict, dataclass, field
from typing import Optional

import cv2
import numpy as np

from ibvs.moments import discrete_moments


ALPHA_DEFINITIONS = ("edge_0_1", "diagonal_0_2")


def wrap_angle(angle):
    """Wrap to (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ============================================================
# DESIRED FEATURES
# ============================================================

@dataclass
class DesiredFeatures:
    z_star: float                 # desired depth = height of the marker above the lens (m)
    a_star: float                 # desired area mu20* + mu02* (normalised image units)
    x_g_star: float = 0.0         # desired centre of gravity (normalised image coordinates)
    y_g_star: float = 0.0
    alpha_star: float = 0.0       # desired orientation (rad), same alpha_definition as the features
    source: str = "model"         # "model" | "taught" | "config"
    note: str = ""

    def vector(self):
        """s* = [z* x_g*, z* y_g*, z*, alpha*]   ([P1] p.5, error definition)."""
        return np.array([self.z_star * self.x_g_star, self.z_star * self.y_g_star, self.z_star, self.alpha_star])

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as file:
            json.dump(asdict(self), file, indent=2)

    @staticmethod
    def load(path):
        with open(path, "r", encoding="utf-8") as file:
            data = json.load(file)
        return DesiredFeatures(**data)


def model_desired_area(marker_length_m, z_star):
    """a* of a square of side L seen face-on at depth z* (4 corner points): 2 L^2 / z*^2."""
    if marker_length_m <= 0 or z_star <= 0:
        raise ValueError("marker length and z* must be positive")
    return 2.0 * marker_length_m ** 2 / z_star ** 2


def desired_from_model(marker_length_m, z_star, x_g_star=0.0, y_g_star=0.0, alpha_star=0.0, note=""):
    return DesiredFeatures(z_star=float(z_star), a_star=model_desired_area(marker_length_m, z_star),
                           x_g_star=float(x_g_star), y_g_star=float(y_g_star), alpha_star=float(alpha_star),
                           source="model", note=note)


# ============================================================
# CURRENT FEATURES
# ============================================================

@dataclass
class FeatureSet:
    x_n: float
    y_n: float
    a_n: float
    alpha: float
    x_g: float
    y_g: float
    a: float
    moments: dict = field(default_factory=dict)
    points_norm: Optional[np.ndarray] = None     # (4, 2) undistorted normalised corners

    def vector(self):
        return np.array([self.x_n, self.y_n, self.a_n, self.alpha])

    def is_finite(self):
        return bool(np.all(np.isfinite(self.vector())) and math.isfinite(self.a) and self.a > 0)


def normalized_points(corners_px, camera_matrix, dist_coeffs):
    """Pixel corners (4, 2) -> undistorted normalised coordinates x = X/Z, y = Y/Z (4, 2)."""
    corners = np.asarray(corners_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(corners, camera_matrix, dist_coeffs).reshape(-1, 2)


def orientation(points_norm, alpha_definition="edge_0_1"):
    p = np.asarray(points_norm, dtype=np.float64).reshape(-1, 2)
    if alpha_definition == "edge_0_1":
        a, b = p[0], p[1]
    elif alpha_definition == "diagonal_0_2":
        a, b = p[0], p[2]
    else:
        raise ValueError(f"alpha_definition must be one of {ALPHA_DEFINITIONS}")
    return math.atan2(b[1] - a[1], b[0] - a[0])


def features_from_points(points_norm, desired, alpha_definition="edge_0_1"):
    """Features of [P1] from normalised points and the DesiredFeatures (for a*, z*)."""
    mom = discrete_moments(points_norm)
    a = mom["mu20"] + mom["mu02"]
    if not (a > 0 and math.isfinite(a)):
        raise ValueError("degenerate marker image (zero area)")
    a_n = desired.z_star * math.sqrt(desired.a_star / a)
    return FeatureSet(
        x_n=a_n * mom["x_g"],
        y_n=a_n * mom["y_g"],
        a_n=a_n,
        alpha=orientation(points_norm, alpha_definition),
        x_g=mom["x_g"],
        y_g=mom["y_g"],
        a=a,
        moments=mom,
        points_norm=np.asarray(points_norm, dtype=np.float64).reshape(-1, 2),
    )


def compute_features(corners_px, camera_matrix, dist_coeffs, desired, alpha_definition="edge_0_1"):
    """Pixel corners -> FeatureSet."""
    return features_from_points(normalized_points(corners_px, camera_matrix, dist_coeffs), desired,
                                alpha_definition)


def feature_error(s, s_star):
    """e = s - s*, with the orientation difference wrapped to (-pi, pi]."""
    e = np.asarray(s, dtype=np.float64) - np.asarray(s_star, dtype=np.float64)
    e[3] = wrap_angle(e[3])
    return e


def teach_desired(feature_sets, z_star, alpha_definition="edge_0_1", note=""):
    """
    The learning step of [P1] Sec. 4: average several observations taken with the drone held at the
    desired pose. z* is the measured depth at that pose (e.g. the solvePnP Z averaged over the same frames).
    """
    if not feature_sets:
        raise ValueError("no observations to teach from")
    points = np.array([f.points_norm for f in feature_sets])           # (N, 4, 2)
    mean_points = points.mean(axis=0)
    mom = discrete_moments(mean_points)
    alphas = np.array([f.alpha for f in feature_sets])
    alpha_star = math.atan2(np.mean(np.sin(alphas)), np.mean(np.cos(alphas)))   # circular mean
    return DesiredFeatures(z_star=float(z_star), a_star=float(mom["mu20"] + mom["mu02"]),
                           x_g_star=float(mom["x_g"]), y_g_star=float(mom["y_g"]),
                           alpha_star=float(alpha_star), source="taught", note=note)
