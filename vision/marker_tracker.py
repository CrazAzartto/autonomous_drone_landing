"""
Complete marker-based state estimation for the drone.

    camera frame -> ArUco detection -> solvePnP pose -> quality check
      -> camera frame to world frame (target) -> Kalman filter (position + velocity)
      -> image-moment features (Papers 1/2) -> drone-body errors for the controller

Two classes:
  MarkerProcessor : pure processing of one frame (no camera, no threads) - used by tests,
                    still images and video files.
  MarkerTracker   : runs a MarkerProcessor on the live camera in a BACKGROUND THREAD.
                    Your control loop simply calls  tracker.get_state()  whenever it wants the
                    newest result - it never waits for the camera.

Typical use in the controller:
    tracker = MarkerTracker(config); tracker.start()
    state = tracker.get_state()
    if state.valid: use state.errors.e_forward_m, state.errors.v_forward_mps, ...
    tracker.stop()
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from common.aruco_utils import create_detector, detect_markers, estimate_marker_pose
from common.calibration_io import load_calibration
from common.camera import open_camera
from common.config import repo_path
from common.overlay import draw_text
from vision.features import image_moment_features
from vision.kalman import ConstantVelocityKalman3D
from vision.target import (ControlErrors, camera_to_world, compute_errors, heading_in_camera,
                           load_target, world_yaw, wrap_angle)


GREEN, RED, YELLOW, WHITE, CYAN = (0, 255, 0), (0, 0, 255), (0, 255, 255), (255, 255, 255), (255, 255, 0)


# ============================================================
# STATE (one result per camera frame)
# ============================================================

@dataclass
class MarkerState:
    t: float                                  # time.perf_counter() when the frame was received
    frame_index: int = 0
    detected: bool = False                    # marker found AND pose accepted in THIS frame
    valid: bool = False                       # filter has a recent measurement (age < max_lost_time)
    age_s: float = float("inf")               # time since the last accepted measurement
    camera_xyz_m: Optional[np.ndarray] = None # raw marker position, camera frame (x right, y down-image, z up)
    rvec: Optional[np.ndarray] = None
    reproj_error_px: float = float("nan")
    marker_side_px: float = float("nan")
    corners_px: Optional[np.ndarray] = None
    heading_cam_rad: Optional[float] = None   # drone heading in the camera frame
    has_target: bool = False                  # results/target.json loaded -> world values available
    world_xyz_m: Optional[np.ndarray] = None  # raw position in the world frame (X fwd, Y left, Z up)
    yaw_world_rad: Optional[float] = None
    filtered_xyz_m: Optional[np.ndarray] = None   # Kalman position (world frame if target, else camera)
    filtered_vel_mps: Optional[np.ndarray] = None # Kalman velocity (same frame)
    features: Optional[dict] = None           # x_n, y_n, a_n, alpha_rad (Papers 1/2)
    errors: Optional[ControlErrors] = None    # body-frame errors for the PID (needs target)
    fps: float = 0.0
    rejected_reason: str = ""


# ============================================================
# PROCESSOR (one frame at a time)
# ============================================================

class MarkerProcessor:

    def __init__(self, config, calibration=None, target=None, use_target_file=True):
        self.config = config
        self.calibration = calibration or load_calibration(repo_path(config["calibration"]["output_file"]))
        self.camera_matrix = self.calibration["camera_matrix"]
        self.dist_coeffs = self.calibration["dist_coeffs"]
        self.marker_id = int(config["aruco"]["marker_id"])
        self.marker_length_m = float(config["aruco"]["marker_length_mm"]) / 1000.0
        self.max_reproj_px = float(config["detection"]["max_reproj_error_px"])
        self.max_lost_s = float(config["tracking"]["max_lost_time_s"])
        self.hover_height_m = float(config["target"]["hover_height_m"])
        self.axis_length_m = self.marker_length_m * float(config["detection"]["axis_length_ratio"])
        self.detector = create_detector(config["aruco"]["dictionary"])
        if target is None and use_target_file:
            target = load_target(repo_path(config["target"]["file"]))
        self.target = target
        self.kalman = ConstantVelocityKalman3D(config["tracking"]["process_accel_std"],
                                               config["tracking"]["measurement_std_xy"],
                                               config["tracking"]["measurement_std_z"])
        self.last_measurement_time = None
        self.last_yaw = None
        self.last_yaw_time = None
        self.yaw_rate = 0.0
        self.frame_index = 0

    def set_target(self, target):
        """Switch to a new target (e.g. after tools/set_target.py) - restarts the filter."""
        self.target = target
        self.kalman.reset()
        self.last_yaw = None

    def process(self, frame, t=None, draw=True):
        """Process one BGR frame. Returns (MarkerState, annotated_frame or None)."""
        t = time.perf_counter() if t is None else t
        self.frame_index += 1
        state = MarkerState(t=t, frame_index=self.frame_index, has_target=self.target is not None)
        view = frame.copy() if draw else None

        corners_list, ids = detect_markers(self.detector, frame)
        if draw and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(view, [c.reshape(1, 4, 2) for c in corners_list], ids.reshape(-1, 1))

        measurement = None
        for corners, marker_id in zip(corners_list, ids):
            if marker_id != self.marker_id:
                continue
            pose = estimate_marker_pose(corners, self.marker_length_m, self.camera_matrix, self.dist_coeffs)
            if pose is None:
                state.rejected_reason = "solvePnP failed"
                continue
            if pose["reproj_error_px"] > self.max_reproj_px:
                state.rejected_reason = f"reprojection error {pose['reproj_error_px']:.2f} px too high"
                continue
            measurement = (corners, pose)
            break

        # ---------------- measurement accepted ----------------
        if measurement is not None:
            corners, pose = measurement
            rotation, _ = cv2.Rodrigues(pose["rvec"])
            state.detected = True
            state.camera_xyz_m = pose["tvec"].copy()
            state.rvec = pose["rvec"].copy()
            state.reproj_error_px = pose["reproj_error_px"]
            state.corners_px = corners
            state.marker_side_px = float(np.mean([np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]))
            state.heading_cam_rad = heading_in_camera(rotation)
            state.features = image_moment_features(corners, self.camera_matrix, self.dist_coeffs,
                                                   self.marker_length_m, self.hover_height_m)
            if self.target is not None:
                state.world_xyz_m = camera_to_world(state.camera_xyz_m, self.target)
                state.yaw_world_rad = world_yaw(state.heading_cam_rad, self.target)
                filter_input = state.world_xyz_m
                self._update_yaw_rate(t, state.yaw_world_rad)
            else:
                filter_input = state.camera_xyz_m
            self.kalman.update(t, filter_input)
            self.last_measurement_time = t
            if draw:
                cv2.drawFrameAxes(view, self.camera_matrix, self.dist_coeffs, pose["rvec"], pose["tvec"],
                                  self.axis_length_m, 2)
                for index, (u, v) in enumerate(corners):
                    cv2.circle(view, (int(round(u)), int(round(v))), 4, CYAN, -1)
                    draw_text(view, str(index), (int(u) + 6, int(v) - 6), CYAN, 0.5, 1)
        else:
            self.kalman.predict(t)

        # ---------------- filter output ----------------
        if self.last_measurement_time is not None:
            state.age_s = t - self.last_measurement_time
        state.valid = self.kalman.initialized and state.age_s <= self.max_lost_s
        state.filtered_xyz_m = self.kalman.position
        state.filtered_vel_mps = self.kalman.velocity

        if self.target is not None:
            yaw = state.yaw_world_rad if state.detected else self.last_yaw
            state.errors = compute_errors(t, state.filtered_xyz_m, state.filtered_vel_mps, yaw,
                                          self.yaw_rate, self.hover_height_m, valid=state.valid)
            if state.yaw_world_rad is None:
                state.yaw_world_rad = self.last_yaw

        if draw:
            self._draw_overlay(view, state)
        return state, view

    def _update_yaw_rate(self, t, yaw):
        if self.last_yaw is not None and t > self.last_yaw_time:
            raw_rate = wrap_angle(yaw - self.last_yaw) / (t - self.last_yaw_time)
            self.yaw_rate = 0.7 * self.yaw_rate + 0.3 * raw_rate     # light low-pass filter
        self.last_yaw = yaw
        self.last_yaw_time = t

    def _draw_overlay(self, view, state):
        lines = []
        if state.detected:
            x, y, z = state.camera_xyz_m * 100.0
            lines.append((f"CAMERA  X={x:+6.1f}  Y={y:+6.1f}  Z={z:6.1f} cm   heading={math.degrees(state.heading_cam_rad):+6.1f} deg"
                          f"   reproj={state.reproj_error_px:.2f}px  size={state.marker_side_px:.0f}px", GREEN))
        elif state.valid:
            lines.append((f"marker not seen for {state.age_s * 1000:.0f} ms - predicting", YELLOW))
        else:
            lines.append(("NO MARKER" + (f" ({state.rejected_reason})" if state.rejected_reason else ""), RED))

        if state.has_target and state.errors is not None and state.errors.valid:
            e = state.errors
            lines.append((f"WORLD   x={e.x_world_m * 100:+6.1f}  y={e.y_world_m * 100:+6.1f}  z={e.z_world_m * 100:6.1f} cm"
                          f"   yaw={math.degrees(e.yaw_world_rad):+6.1f} deg", WHITE))
            lines.append((f"ERROR   fwd={e.e_forward_m * 100:+6.1f}  left={e.e_left_m * 100:+6.1f}  up={e.e_up_m * 100:+6.1f} cm"
                          f"   yaw={math.degrees(e.e_yaw_rad):+6.1f} deg", YELLOW))
            lines.append((f"VEL     fwd={e.v_forward_mps * 100:+6.1f}  left={e.v_left_mps * 100:+6.1f}  up={e.v_up_mps * 100:+6.1f} cm/s",
                          WHITE))
        elif not state.has_target:
            lines.append(("no target set - run tools\\set_target.py for world frame + errors", YELLOW))

        for row, (text, colour) in enumerate(lines):
            draw_text(view, text, (10, 28 + 26 * row), colour, 0.55)
        cx, cy = int(self.camera_matrix[0, 2]), int(self.camera_matrix[1, 2])
        cv2.drawMarker(view, (cx, cy), WHITE, cv2.MARKER_CROSS, 20, 1)


# ============================================================
# TRACKER (live camera in a background thread)
# ============================================================

class MarkerTracker:

    def __init__(self, config, draw=True, use_target_file=True):
        self.config = config
        self.draw = draw
        self.processor = MarkerProcessor(config, use_target_file=use_target_file)
        calib_size = (self.processor.calibration["image_width"], self.processor.calibration["image_height"])
        if calib_size != (int(config["camera"]["width"]), int(config["camera"]["height"])):
            raise RuntimeError(f"Calibration was made at {calib_size[0]}x{calib_size[1]} but the config asks for "
                               f"{config['camera']['width']}x{config['camera']['height']}. Recalibrate or fix the config.")
        self.calib_size = calib_size
        self._lock = threading.Lock()
        self._state = MarkerState(t=time.perf_counter())
        self._frame = None
        self._raw_frame = None
        self._running = False
        self._thread = None
        self._pending_target = None
        self.capture = None
        self.error = None

    # ---------------- public API ----------------
    def start(self, timeout_s=10.0):
        """Open the camera and start tracking. Raises RuntimeError if the camera does not work."""
        self.capture, actual = open_camera(self.config["camera"])
        if not actual["is_file"] and (actual["width"], actual["height"]) != self.calib_size:
            self.capture.release()
            raise RuntimeError(f"Camera delivers {actual['width']}x{actual['height']} but the calibration is "
                               f"{self.calib_size[0]}x{self.calib_size[1]} - the pose would be wrong.")
        self.is_file = actual["is_file"]
        self.file_fps = actual["fps"] if actual["is_file"] and actual["fps"] > 0 else 30.0
        self._running = True
        self._thread = threading.Thread(target=self._run, name="MarkerTracker", daemon=True)
        self._thread.start()
        start = time.perf_counter()
        while self.get_state().frame_index == 0 and time.perf_counter() - start < timeout_s:
            if self.error:
                raise RuntimeError(self.error)
            time.sleep(0.01)
        return self

    def get_state(self):
        """Newest MarkerState (never blocks for long)."""
        with self._lock:
            return self._state

    def get_frame(self):
        """Newest annotated image (or None) - show it from your MAIN thread with cv2.imshow."""
        with self._lock:
            return None if self._frame is None else self._frame

    def get_raw_frame(self):
        """Newest camera image WITHOUT drawings (for recording videos that can be replayed)."""
        with self._lock:
            return self._raw_frame

    def set_target(self, target):
        """Use a new target; applied by the worker thread before the next frame."""
        with self._lock:
            self._pending_target = target

    def is_running(self):
        return self._running

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()

    # ---------------- worker thread ----------------
    def _run(self):
        failed_reads = 0
        frames = 0
        fps = 0.0
        fps_timer = time.perf_counter()
        max_failed = int(self.config["detection"]["max_failed_reads"])
        while self._running:
            ok, frame = self.capture.read()
            t = time.perf_counter()
            if not ok or frame is None:
                if self.is_file:
                    self._running = False          # end of the video file
                    break
                failed_reads += 1
                if failed_reads >= max_failed:
                    self._reopen()
                    failed_reads = 0
                time.sleep(0.005)
                continue
            failed_reads = 0

            frames += 1
            if t - fps_timer >= 1.0:
                fps = frames / (t - fps_timer)
                frames, fps_timer = 0, t

            with self._lock:
                pending, self._pending_target = self._pending_target, None
            if pending is not None:
                self.processor.set_target(pending)
            state, view = self.processor.process(frame, t, draw=self.draw)
            state.fps = fps
            if self.draw:
                draw_text(view, f"{fps:.1f} fps   {'VALID' if state.valid else 'INVALID'}",
                          (10, view.shape[0] - 15), GREEN if state.valid else RED, 0.6)
            with self._lock:
                self._state = state
                self._frame = view
                self._raw_frame = frame
            if self.is_file:
                time.sleep(1.0 / self.file_fps)    # play files at their real speed

    def _reopen(self):
        print("Camera stopped delivering frames - trying to reopen ...")
        try:
            self.capture.release()
            time.sleep(1.0)
            self.capture, _ = open_camera(self.config["camera"], verbose=False)
            print("Camera reopened.")
        except RuntimeError as exc:
            self.error = f"camera lost: {exc}"
            print(f"ERROR: {self.error}")
            time.sleep(1.0)
