"""
SYNTHETIC plant and camera (docs/06 §8). Every parameter comes from config/ibvs_config.yaml -> simulation.

Plant, in the camera frame C (z up), state p (3), v (3), yaw psi, tilts:
  RC is applied after a transport delay (Wi-Fi + firmware + camera, [P3] Table 3 measured 341 ms total)
  pitch/roll : tilt set-point = gain * (RC - 1500) [deg/us], max 20 deg (MagisV2 config.cpp l.447),
               first-order attitude response (tau_att), acceleration = g * tan(tilt), linear drag,
               constant disturbance acceleration (trim / draft) + random gusts
  throttle   : altitude-hold model from the firmware: v_z* = (RC-1500)/4 cm/s outside +-40 us dead band,
               clamped -100..120 cm/s, first-order response (tau_z); baro drift while holding
  yaw        : rate = -k * (RC - 1500) (yaw < 1500 turns left)
  land       : descends at land_speed until z <= ground_z, then disarms
Camera: pinhole + distortion with YOUR calibration (or defaults), pixel noise, detection only when the
whole marker is inside the image and at least min_side_px wide.
"""

import math
from collections import deque

import cv2
import numpy as np

from common.aruco_utils import estimate_marker_pose
from ibvs.sources import Observation
from pluto_interface.adapters import BaseAdapter
from pluto_interface.command_mapper import NEUTRAL, RCCommand

G = 9.81


class PlutoSimPlant:

    def __init__(self, p, rng=None):
        self.p = p
        self.rng = rng or np.random.default_rng(int(p.get("seed", 1)))
        self.pos = np.array(p["initial_position_m"], dtype=float)
        self.vel = np.zeros(3)
        self.yaw = math.radians(p["initial_yaw_deg"])
        self.tilt = np.zeros(2)                 # forward, left (rad)
        self.rc = RCCommand()
        self._queue = deque()
        self.armed = True
        self.flying = True
        self.landing = False
        self.t = 0.0
        self.baro_drift = 0.0
        self.gust = np.zeros(2)
        self.disturbance = np.array(p["disturbance_accel_mps2"][:2], dtype=float)

    def command(self, t, rc):
        self._queue.append((t + self.p["delay_s"], rc))

    def _active_rc(self):
        while self._queue and self._queue[0][0] <= self.t:
            self.rc = self._queue.popleft()[1]
        return self.rc

    def step(self, dt):
        p = self.p
        rc = self._active_rc()
        if not self.flying:
            self.vel[:] = 0
            self.t += dt
            return
        # attitude
        max_tilt = math.radians(p["max_tilt_deg"])
        cmd_fwd = np.clip(math.radians(p["tilt_deg_per_us"] * (rc.pitch - NEUTRAL)), -max_tilt, max_tilt)
        cmd_left = np.clip(math.radians(-p["tilt_deg_per_us"] * (rc.roll - NEUTRAL)), -max_tilt, max_tilt)
        if self.landing:
            cmd_fwd = cmd_left = 0.0
        self.tilt += (np.array([cmd_fwd, cmd_left]) - self.tilt) * min(1.0, dt / p["attitude_tau_s"])
        a_body = G * np.tan(self.tilt)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        a_xy = np.array([c * a_body[0] - s * a_body[1], s * a_body[0] + c * a_body[1]])
        # gusts: Ornstein-Uhlenbeck acceleration (std gust_accel_std, correlation time gust_tau_s),
        # independent of the integration step
        tau_g = p["gust_tau_s"]
        self.gust += -self.gust / tau_g * dt + p["gust_accel_std_mps2"] * math.sqrt(2.0 * dt / tau_g) * \
            self.rng.normal(0.0, 1.0, 2)
        a_xy += self.disturbance + self.gust - p["drag_per_s"] * self.vel[:2]
        self.vel[:2] += a_xy * dt
        # vertical: firmware altitude-hold velocity mode
        offset = rc.throttle - NEUTRAL
        if self.landing:
            vz_set = -p["land_speed_mps"]
        elif abs(offset) > 40:
            vz_set = float(np.clip(int(offset / 4), -100, 120)) / 100.0
        else:
            vz_set = self.baro_drift              # firmware holds the BAROMETRIC altitude, which drifts
        tau_b = p["baro_drift_tau_s"]
        self.baro_drift += -self.baro_drift / tau_b * dt + p["baro_drift_std_mps"] * \
            math.sqrt(2.0 * dt / tau_b) * self.rng.normal()
        self.vel[2] += (vz_set - self.vel[2]) * min(1.0, dt / p["vertical_tau_s"])
        # yaw rate (yaw < 1500 = turn left = psi increases)
        self.yaw += -p["yaw_radps_per_us"] * (rc.yaw - NEUTRAL) * dt
        self.pos += self.vel * dt
        if self.pos[2] <= p["ground_z_m"]:
            self.pos[2] = p["ground_z_m"]
            self.vel[:] = 0
            self.flying = False                      # touched the ground (landing pad)
            self.armed = False
        self.t += dt

    def advance_to(self, t, substep=0.002):
        while self.t < t - 1e-12:
            self.step(min(substep, t - self.t))

    def truth(self):
        return {"x": self.pos[0], "y": self.pos[1], "z": self.pos[2], "yaw": self.yaw,
                "vx": self.vel[0], "vy": self.vel[1], "vz": self.vel[2],
                "tilt_fwd": self.tilt[0], "tilt_left": self.tilt[1]}


class SyntheticCamera:

    def __init__(self, p, camera_matrix, dist_coeffs, image_size, marker_length_m, rng=None):
        self.p = p
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.size = image_size
        self.L = marker_length_m
        self.rng = rng or np.random.default_rng(int(p.get("seed", 1)) + 7)

    def corners(self, truth):
        """Pixel corners (TL, TR, BR, BL of the printed marker) of the marker under the drone."""
        yaw = truth["yaw"]
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        # small tilts (forward tilt = nose down rotates the plane about the left axis)
        rot_left = cv2.Rodrigues(left * truth["tilt_fwd"])[0]
        rot_fwd = cv2.Rodrigues(-fwd * truth["tilt_left"])[0]
        R = rot_fwd @ rot_left
        x_m, y_m = R @ left, R @ fwd                     # x_M = drone left, y_M = drone forward
        c = np.array([truth["x"], truth["y"], truth["z"]])
        h = self.L / 2.0
        obj = np.array([c - h * x_m + h * y_m, c + h * x_m + h * y_m, c + h * x_m - h * y_m, c - h * x_m - h * y_m])
        pixels, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), self.K, self.dist)
        pixels = pixels.reshape(4, 2) + self.rng.normal(0.0, self.p["pixel_noise_px"], (4, 2))
        return pixels

    def observe(self, plant, t, index):
        truth = plant.truth()
        markers = {}
        if truth["z"] > 0.05:
            px = self.corners(truth)
            w, h = self.size
            side = np.mean([np.linalg.norm(px[i] - px[(i + 1) % 4]) for i in range(4)])
            inside = px[:, 0].min() > 2 and px[:, 1].min() > 2 and px[:, 0].max() < w - 2 and px[:, 1].max() < h - 2
            dropout = self.rng.random() < self.p["dropout_probability"]
            if inside and side >= self.p["min_side_px"] and not dropout:
                markers[int(self.p["marker_id"])] = px
        return Observation(t_capture=t, frame_index=index, markers=markers, frame=None,
                           image_size=self.size, truth=truth)

    def pnp(self, corners_px):
        return estimate_marker_pose(corners_px.astype(np.float32), self.L, self.K, self.dist)


class SimAdapter(BaseAdapter):
    """Adapter interface on top of the synthetic plant (labelled SYNTHETIC everywhere)."""
    name = "sim"
    live = False

    def __init__(self, plant, clock, event_callback=None):
        super().__init__(event_callback)
        self.plant = plant
        self.clock = clock                    # callable returning simulation time
        self.armed = plant.armed
        self.flying = plant.flying

    def send_rc(self, rc):
        self.last_rc = rc
        self.plant.command(self.clock(), rc)

    def land(self):
        self.plant.landing = True

    def disarm(self):
        self.plant.flying = False
        self.plant.armed = False
        self.armed = self.flying = False

    def telemetry(self):
        t = self.plant.truth()
        return {"age_s": 0.0, "roll_deg": math.degrees(t["tilt_left"]), "pitch_deg": math.degrees(t["tilt_fwd"]),
                "baro_alt_cm": 100 * t["z"], "vario_cmps": 100 * t["vz"], "vbat_v": 3.9,
                "rc_echo_latency_s": self.plant.p["delay_s"]}
