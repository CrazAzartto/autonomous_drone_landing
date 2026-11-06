"""
Frames and the eye-to-hand adaptation (hardware modification of [P1]; derivation in docs/06 §3-4).

FRAMES (all right-handed)
  C  camera (OpenCV): x_C = image right, y_C = image down, z_C = out of the lens. The C920 lies on the
     floor looking UP, so z_C = vertical up [assumption: camera levelled, docs/08 check C2].
     Seen from ABOVE, x_C -> y_C is COUNTER-CLOCKWISE.
  M  ArUco marker: origin = centre, x_M = printed right, y_M = printed top edge, z_M = out of the printed
     face. Glued under the drone, printed face DOWN, top edge towards the nose: z_M = down.
  B  drone body, FLU: x_B = forward (nose) = y_M,  y_B = left = x_M,  z_B = up = -z_M.
  Pluto RC (your plutocontrol library; signs MUST be verified, docs/08 tests B3/F1):
     pitch > 1500 -> forward (+x_B),  roll < 1500 -> left (+y_B),
     throttle > 1500 -> up (+z_B, altitude-hold velocity),  yaw < 1500 -> turn left (+r about z_B).

EYE-IN-HAND vs EYE-TO-HAND
  [P1] Eq.(1) gives v_c: the velocity the CAMERA should have, relative to a static marker, so that
  s_dot = L_s v_c = -lambda L_s L_hat^+ e. Here the camera is static and the marker (drone) moves.
  For a marker parallel to the image plane, s = [X, Y, Z, alpha] exactly (features.py), so the marker
  twist u_m = [v_mx, v_my, v_mz, w_m] (velocity of the marker centre and yaw rate about its own vertical
  axis, both in C) gives s_dot = I u_m. Requiring the SAME feature motion as the paper's law gives

        u_m = L_s(s) v_c                     (4x4 times 4x1)

  which is identical to the rigid-body relation  v_m = -v_c + w_m x P,  w_m = -w_cz  (docs/06 §4).
  Consequently the closed loop is s_dot = -lambda L_s L_hat^+ e exactly as in [P1]: the paper's error
  dynamics, decoupling and gain schedule are preserved; only the actuator changes.

HEADING FROM THE IMAGE
  With the marker facing the camera, y_M is x_M rotated by -90 deg in the camera x-y plane, so the
  drone heading (direction of x_B = y_M, CCW from x_C seen from above) is
        psi = alpha - 90 deg        (alpha_definition "edge_0_1": alpha = direction of x_M)
        psi = alpha - 135 deg       ("diagonal_0_2": corner 0 -> 2 points along x_M - y_M)
  so the whole controller uses image features only; solvePnP is used for logging/validation.
"""

import math
from dataclasses import dataclass

import numpy as np

from ibvs.features import wrap_angle
from ibvs.interaction import interaction_matrix


@dataclass
class BodyVelocity:
    """Drone-frame quantities. Positive = forward / left / up / turning left (CCW from above)."""
    forward: float
    left: float
    up: float
    yaw_rate: float

    def as_array(self):
        return np.array([self.forward, self.left, self.up, self.yaw_rate])

    def is_finite(self):
        return bool(np.all(np.isfinite(self.as_array())))


def camera_twist_to_marker_twist(v_c, s):
    """u_m = L_s(s) v_c : marker (drone) twist in the camera frame producing the paper's feature motion."""
    return interaction_matrix(s) @ np.asarray(v_c, dtype=np.float64)


def heading_from_alpha(alpha, alpha_definition="edge_0_1"):
    """Drone heading psi (direction of the nose in the camera x-y plane, CCW from x_C seen from above)."""
    offset = math.pi / 2.0 if alpha_definition == "edge_0_1" else 3.0 * math.pi / 4.0
    return wrap_angle(alpha - offset)


def camera_xy_to_body(vector_xy, psi):
    """Rotate a horizontal camera-frame vector into the body frame: returns (forward, left)."""
    c, s = math.cos(psi), math.sin(psi)
    vx, vy = float(vector_xy[0]), float(vector_xy[1])
    return c * vx + s * vy, -s * vx + c * vy


def body_to_camera_xy(forward, left, psi):
    c, s = math.cos(psi), math.sin(psi)
    return c * forward - s * left, s * forward + c * left


def marker_twist_to_body(u_m, psi):
    """u_m = [v_mx, v_my, v_mz, w_m] in C  ->  BodyVelocity (forward, left, up, yaw-rate-left)."""
    forward, left = camera_xy_to_body(u_m[0:2], psi)
    return BodyVelocity(forward=forward, left=left, up=float(u_m[2]), yaw_rate=float(u_m[3]))


def camera_velocity_to_body(v_cam, yaw_rate, psi):
    """Measured/estimated marker velocity (camera frame) -> BodyVelocity."""
    forward, left = camera_xy_to_body(v_cam[0:2], psi)
    return BodyVelocity(forward=forward, left=left, up=float(v_cam[2]), yaw_rate=float(yaw_rate))
