# Autonomous Drone Landing: vision-based position hold and landing of a Pluto drone (Windows)

BTP project: a **Logitech C920 on the floor looking up** sees an **ArUco marker under a Pluto nano drone**.
The code turns every camera frame into the drone's position, velocity and heading relative to a target point.
From these it computes the **errors a PID controller needs**. The approach follows IIT Bombay's Vicon-based
Pluto controller (`RC = 1500 + PID(error)`), with the camera + marker replacing the Vicon system.

| Part | Status |
|---|---|
| Camera setup on Windows (DirectShow, MJPG 1280×720@30, focus/exposure locked and read back) | **done** |
| Camera calibration (capture with coverage/blur checks, calibration, report) | **done** |
| ArUco detection + 3-D pose (IPPE_SQUARE, bad poses rejected) | **done** |
| Target / world frame, heading, drone-frame errors, Kalman velocity, image-moment features | **done** |
| Control loop: modes, keys, safety (marker lost → neutral → land), logging, plots, Pluto connection | **done** |
| **PID law** (`control/pid_controller.py`) and **RC mapping** (`compute_rc()` in `control/run_controller.py`) | **your remaining work**, marked `### TODO ###` |

**Start here → [docs/04_windows_step_by_step.md](docs/04_windows_step_by_step.md)** (complete click-by-click guide).

## Quick start (PowerShell)

```powershell
cd "D:\Study Materials\BTP\codebase\autonomous_drone_landing"
..\.venv\Scripts\Activate.ps1                         # (.venv) appears
python -m pip install -r requirements.txt             # first time only

python detection\camera_info.py --list                # find the C920 index -> config camera.device
python detection\camera_info.py --settings            # lock focus/exposure, press p, copy into config
python calibration\capture_calibration_images.py --clear
python calibration\calibrate_camera.py
python detection\aruco_pose_estimation.py             # tape-measure check of Z
python tools\set_target.py                            # define target point + "forward", check signs
python detection\aruco_pose_estimation.py --log       # live world position, errors, velocities
python control\run_controller.py                      # controller DRY RUN (sends nothing)
```

## Repository layout

```
autonomous_drone_landing/
├── config/camera_config.yaml        ONE config for every program (camera, board, marker, filter, target, loop)
├── common/                          shared low-level code
│   ├── config.py                    load + validate the YAML
│   ├── camera.py                    open the camera on Windows (DirectShow, MJPG, controls with read-back)
│   ├── aruco_utils.py               detector, solvePnP pose, reprojection error
│   ├── calibration_io.py            save/load results/camera_calibration.npz
│   └── overlay.py                   readable on-screen text
├── vision/                          state estimation (the "sensor" of the controller)
│   ├── marker_tracker.py            MarkerProcessor (one frame) + MarkerTracker (camera thread)
│   ├── target.py                    target file, world frame, ControlErrors (drone-frame errors)
│   ├── kalman.py                    constant-velocity Kalman filter (position + velocity)
│   └── features.py                  image-moment features x_n, y_n, a_n, alpha (Papers 1/2)
├── calibration/
│   ├── capture_calibration_images.py
│   ├── calibrate_camera.py
│   └── calibration_images/          your photos (not uploaded)
├── detection/
│   ├── camera_info.py               list cameras, check fps, settings window, print controls
│   └── aruco_pose_estimation.py     live pose / world / errors, CSV log, raw-video recording, replay
├── tools/
│   ├── generate_print_sheets.py     exact-size PDFs of the marker and the checkerboard
│   └── set_target.py                record the target point and the forward direction
├── control/
│   ├── run_controller.py            control loop template: YOUR RC mapping goes in compute_rc()
│   ├── pid_controller.py            PID class: YOUR PID law goes in update()
│   └── plotting.py                  plots of results/control_log.csv
├── docs/
│   ├── 01_camera_calibration_guide.md   theory + procedure of calibration
│   ├── 02_c920_configuration.md         the C920 on Windows (DirectShow, MJPG, settings)
│   ├── 03_aruco_and_pose_estimation.md  pose, frames, world frame, Kalman filter, features
│   ├── 04_windows_step_by_step.md       THE complete procedure
│   └── 05_adding_pid_and_rc.md          how to finish the controller (signs, gains, tuning, safety)
└── results/                         calibration, target, logs, recordings (not uploaded)
```

## How the pieces fit

```
camera thread:  frame -> ArUco -> solvePnP -> reject if reprojection > 2 px
                      -> camera frame -> world frame (target.json) -> Kalman filter
                      -> ControlErrors(e_forward, e_left, e_up, e_yaw, velocities)
control loop (30 Hz):  errors -> PID (yours) -> RC = 1500 + sign*PID (yours) -> Pluto (Wi-Fi, MSP)
```

Using it in your own script:

```python
from common.config import load_config
from vision.marker_tracker import MarkerTracker

tracker = MarkerTracker(load_config(None)); tracker.start()
state = tracker.get_state()                  # newest result, never waits for the camera
if state.errors is not None and state.errors.valid:
    print(state.errors.e_forward_m, state.errors.v_forward_mps)
tracker.stop()
```

## What has been verified and what not

| Item | Status |
|---|---|
| All programs with opencv-contrib-python **4.10.0** (Linux test machine) | **Tested on a synthetic camera:** calibration fx 910.6 vs 912 true, RMS 0.10 px; pose X/Y/Z 12.1/−5.0/90.5 cm vs 12/−5/90 cm |
| World frame and errors | **Tested (synthetic):** 10 cm behind → e_forward +10.0 cm; 10 cm right → e_left +10.2 cm; 20° left → e_yaw −20.4°; rotated 90° + displaced → correct body-frame errors |
| Kalman velocity | **Tested (synthetic):** 0.20 m/s → −0.203 m/s (σ 0.007 m/s); marker lost 0.6 s → state invalid |
| Controller loop | **Tested (dry run on a synthetic video, simulated keys):** arm/take-off/hold/landing ramp/emergency stop; log written |
| Your laptop: Python 3.12.10 venv, OpenCV 4.14.0, C920 listed as index 1 ("HD Pro Webcam C920") | **Checked** |
| Same synthetic image and video **on your laptop** (pose, target, Kalman velocity, MJPG video decoding), `plutocontrol` import from `pluto_course` | **Tested**, same results as above. The camera itself was not opened |
| Live C920 run, calibration quality, real pose accuracy, fps on your laptop | **Not yet tested.** That is your first run (docs/04) |
| Pluto RC signs (forward = pitch 1600, left = roll 1400, left_yaw = 1300) | taken from your `plutocontrol` library. **Verify on the drone with the propellers off** (docs/05) |

## Rules for the repository
- Generated files (`results/`, calibration images, `.npz`, logs, recordings) are git-ignored. Never commit them.
- No passwords or tokens in the code or the config.
