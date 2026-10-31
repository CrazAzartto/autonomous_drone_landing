"""
World (room) frame, target point and drone-body errors.

Setup: camera on the floor looking UP. OpenCV camera frame: x = image right, y = image down,
z = out of the lens = UP. Seen from above, (x, y) is a normal counter-clockwise (CCW) frame.

The marker is glued under the drone with its printed TOP edge towards the drone's FRONT, so the
marker's +y axis = drone forward direction.

tools/set_target.py records, with the drone placed at the desired point and pointing "forward":
    heading_ref_rad : drone heading in the camera frame at that moment  -> defines world +X
    target_cam_xy   : marker position in the camera frame               -> world origin
World frame W (used by the controller):
    X_w = forward (as set), Y_w = left, Z_w = up (height above the camera lens), origin at target.
Body errors (what a PID needs):  how far to go FORWARD / LEFT / UP and how much to turn LEFT,
measured in the drone's own current frame, so pitch acts on e_forward and roll on e_left.
"""

import datetime
import json
import math
import os
from dataclasses import dataclass

import numpy as np


def wrap_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def heading_in_camera(rotation_matrix):
    """
    Heading of the drone's forward direction (= marker +y axis) in the camera x-y plane,
    measured CCW from the camera +x axis (seen from above). Robust to tilt (uses the projection).
    """
    forward = rotation_matrix[:, 1]
    return math.atan2(forward[1], forward[0])


# ============================================================
# TARGET FILE
# ============================================================

def save_target(path, heading_ref_rad, target_cam_xy, target_height_seen_m, samples):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    data = {
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "heading_ref_rad": float(heading_ref_rad),
        "heading_ref_deg": math.degrees(heading_ref_rad),
        "target_cam_xy_m": [float(target_cam_xy[0]), float(target_cam_xy[1])],
        "height_when_recorded_m": float(target_height_seen_m),
        "samples_averaged": int(samples),
        "meaning": "World X = drone forward at recording, Y = left, Z = up; origin = recorded marker x,y.",
    }
    with open(path, "w", encoding="utf-8") as file:
        json.dump(data, file, indent=2)
    return data


def load_target(path):
    """Return the target dict, or None if the file does not exist yet."""
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as file:
        data = json.load(file)
    data["heading_ref_rad"] = float(data["heading_ref_rad"])
    data["target_cam_xy_m"] = np.asarray(data["target_cam_xy_m"], dtype=float)
    return data


# ============================================================
# TRANSFORMS
# ============================================================

def camera_to_world(cam_xyz, target):
    """Marker position camera frame -> world frame (origin at target point, X forward, Z up)."""
    c, s = math.cos(target["heading_ref_rad"]), math.sin(target["heading_ref_rad"])
    dx = cam_xyz[0] - target["target_cam_xy_m"][0]
    dy = cam_xyz[1] - target["target_cam_xy_m"][1]
    # Rotate by -heading_ref so that the recorded forward direction becomes +X.
    return np.array([c * dx + s * dy, -s * dx + c * dy, cam_xyz[2]])


def world_yaw(heading_cam_rad, target):
    """Drone yaw in the world frame (0 = facing world +X, positive = turned LEFT / CCW)."""
    return wrap_angle(heading_cam_rad - target["heading_ref_rad"])


def world_to_body(vector_xy, yaw_rad):
    """Rotate a horizontal world vector into the drone frame: returns (forward, left)."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return c * vector_xy[0] + s * vector_xy[1], -s * vector_xy[0] + c * vector_xy[1]


# ============================================================
# CONTROL ERRORS (input of the PID controller)
# ============================================================

@dataclass
class ControlErrors:
    """
    All in the DRONE's frame. Positive error = the drone must move/turn in that direction.
      e_forward_m  : go FORWARD by this much  (-> pitch)
      e_left_m     : go LEFT by this much     (-> roll)
      e_up_m       : go UP by this much       (-> throttle)
      e_yaw_rad    : turn LEFT (CCW) by this much (-> yaw)
    Velocities (filtered, drone frame) for the D-term. For a fixed target the error rate is
    d(e_forward)/dt = -v_forward, etc.
    """
    valid: bool
    t: float
    e_forward_m: float = 0.0
    e_left_m: float = 0.0
    e_up_m: float = 0.0
    e_yaw_rad: float = 0.0
    v_forward_mps: float = 0.0
    v_left_mps: float = 0.0
    v_up_mps: float = 0.0
    yaw_rate_radps: float = 0.0
    x_world_m: float = 0.0
    y_world_m: float = 0.0
    z_world_m: float = 0.0
    yaw_world_rad: float = 0.0


def compute_errors(t, world_xyz, world_velocity, yaw_rad, yaw_rate, target_height_m, valid=True):
    """Body-frame errors to the target point (world origin) at height target_height_m."""
    if world_xyz is None or yaw_rad is None:
        return ControlErrors(valid=False, t=t)
    e_world = (-world_xyz[0], -world_xyz[1])                 # target is at the world origin
    e_forward, e_left = world_to_body(e_world, yaw_rad)
    v_forward, v_left = (0.0, 0.0)
    v_up = 0.0
    if world_velocity is not None:
        v_forward, v_left = world_to_body(world_velocity[0:2], yaw_rad)
        v_up = float(world_velocity[2])
    return ControlErrors(
        valid=valid, t=t,
        e_forward_m=e_forward, e_left_m=e_left, e_up_m=target_height_m - world_xyz[2],
        e_yaw_rad=wrap_angle(-yaw_rad),
        v_forward_mps=v_forward, v_left_mps=v_left, v_up_mps=v_up, yaw_rate_radps=yaw_rate or 0.0,
        x_world_m=float(world_xyz[0]), y_world_m=float(world_xyz[1]), z_world_m=float(world_xyz[2]),
        yaw_world_rad=yaw_rad,
    )
