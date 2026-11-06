"""
Where observations come from. All sources return the same Observation object, so the controller,
safety supervisor and logger are identical for live camera, video replay and simulation.

  CameraSource : live C920 in a background thread (reuses common/camera.open_camera and
                 common/aruco_utils detection - the completed and tested Stage-1 code, unchanged)
  ReplaySource : a recorded video (.avi/.mp4), frame by frame, timestamps = frame_index / fps
  (sim/pluto_sim.SimSource : synthetic corners from the simulated drone)

Unlike vision/marker_tracker.MarkerTracker (which keeps one configured marker ID), these sources keep
ALL detected markers, because [P1] Algorithm 1 may use a different marker in each stage.
"""

import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import cv2
import numpy as np

from common.aruco_utils import create_detector, detect_markers
from common.camera import open_camera


@dataclass
class Observation:
    t_capture: float                               # time.perf_counter() (live) or video/sim time (s)
    frame_index: int
    markers: Dict[int, np.ndarray] = field(default_factory=dict)   # id -> (4, 2) pixel corners
    frame: Optional[np.ndarray] = None             # BGR image (None in headless sim)
    image_size: tuple = (1280, 720)
    truth: Optional[dict] = None                   # simulator ground truth only


class CameraSource:
    """Live camera, detection in a worker thread; read() returns the newest observation (never blocks long)."""

    def __init__(self, camera_config, dictionary_name, keep_frames=True):
        self.camera_config = camera_config
        self.detector = create_detector(dictionary_name)
        self.keep_frames = keep_frames
        self.capture = None
        self.actual = None
        self._lock = threading.Lock()
        self._latest = None
        self._raw = None
        self._running = False
        self._thread = None
        self.error = None
        self.fps = 0.0

    def start(self, timeout_s=10.0):
        self.capture, self.actual = open_camera(self.camera_config)
        self._running = True
        self._thread = threading.Thread(target=self._run, name="CameraSource", daemon=True)
        self._thread.start()
        deadline = time.perf_counter() + timeout_s
        while self._latest is None and time.perf_counter() < deadline and self._running:
            time.sleep(0.01)
        if self._latest is None:
            self.stop()
            raise RuntimeError(self.error or "camera delivered no frame")
        return self

    def read(self):
        with self._lock:
            return self._latest

    def raw_frame(self):
        with self._lock:
            return self._raw

    def is_running(self):
        return self._running

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()

    def _run(self):
        index = 0
        failures = 0
        frames, fps_timer = 0, time.perf_counter()
        while self._running:
            ok, frame = self.capture.read()
            t = time.perf_counter()
            if not ok or frame is None:
                failures += 1
                if failures > 30:
                    self.error = "camera stopped delivering frames"
                    self._running = False
                time.sleep(0.005)
                continue
            failures = 0
            index += 1
            corners, ids = detect_markers(self.detector, frame)
            markers = {int(i): c.astype(np.float64) for c, i in zip(corners, ids)}
            obs = Observation(t_capture=t, frame_index=index, markers=markers,
                              frame=frame if self.keep_frames else None,
                              image_size=(frame.shape[1], frame.shape[0]))
            frames += 1
            if t - fps_timer >= 1.0:
                self.fps = frames / (t - fps_timer)
                frames, fps_timer = 0, t
            with self._lock:
                self._latest = obs
                self._raw = frame


class ReplaySource:
    """Recorded video, processed frame by frame (deterministic, faster than real time when headless)."""

    def __init__(self, path, dictionary_name, keep_frames=True):
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise RuntimeError(f"cannot open video {path}")
        self.fps = self.capture.get(cv2.CAP_PROP_FPS) or 30.0
        if not np.isfinite(self.fps) or self.fps <= 1:
            self.fps = 30.0
        self.detector = create_detector(dictionary_name)
        self.keep_frames = keep_frames
        self.index = 0

    def next(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            return None
        self.index += 1
        corners, ids = detect_markers(self.detector, frame)
        markers = {int(i): c.astype(np.float64) for c, i in zip(corners, ids)}
        return Observation(t_capture=self.index / self.fps, frame_index=self.index, markers=markers,
                           frame=frame if self.keep_frames else None, image_size=(frame.shape[1], frame.shape[0]))

    def stop(self):
        self.capture.release()
