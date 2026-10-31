# Autonomous Drone Landing: Stage 1 (camera calibration + ArUco pose)

Visual-sensing module for a BTP project on autonomous landing of a Pluto nano drone with an ArUco marker.
**Stage 1** turns a **Logitech C920** on **Ubuntu** into a calibrated sensor that reports the marker's
3-D position `(X, Y, Z)` in the camera frame, live.

> Out of scope for this stage (on purpose): interaction matrix, IBVS controller, velocity or RC commands,
> drone communication.

---

## What is verified and what is not

| Item | Status |
|---|---|
| Programs run end-to-end with **opencv-contrib-python 4.10.0 and 4.14.0** (Python 3.13, NumPy 2.x) | **Tested** with a *synthetic* camera: calibration recovered fx 910.6 vs 912 true; pose of a known marker: X 12.1 / Y −5.0 / Z 90.5 cm vs 12 / −5 / 90 cm |
| Generated print sheets | **Tested**: marker measures 50.0 mm, checkerboard squares 24.97 mm in the rendered PDF |
| v4l2 control handling (`scripts/configure_c920.sh`, `common/camera.py`) | **Tested** against a mock `v4l2-ctl` that uses a real C920 control list |
| On a real C920 under Ubuntu 22.04 / 24.04 (Python 3.10 / 3.12) | **Not yet tested.** This is your first run; follow the guide below |
| C920 formats / controls / FOV | from cited sources (docs/02); **confirm on your unit** with the commands given |

## Repository layout

```
autonomous_drone_landing/
├── README.md
├── requirements.txt
├── .gitignore / .gitattributes
├── config/
│   └── camera_config.yaml            # ONE config for every program (camera, board, marker, paths)
├── common/                           # shared code (added: avoids 4 copies of the same logic)
│   ├── config.py                     # load + validate the YAML
│   ├── camera.py                     # open C920 via V4L2 (MJPG, size, fps), v4l2-ctl controls, checks
│   ├── aruco_utils.py                # detector, solvePnP pose, reprojection error
│   ├── calibration_io.py             # save/load camera_calibration.npz
│   └── overlay.py                    # readable on-screen text
├── calibration/
│   ├── capture_calibration_images.py # PROGRAM 2
│   ├── calibrate_camera.py           # PROGRAM 3
│   └── calibration_images/           # your photos (git-ignored)
├── detection/
│   ├── camera_info.py                # PROGRAM 1
│   └── aruco_pose_estimation.py      # PROGRAM 4
├── tools/
│   └── generate_print_sheets.py      # exact-size PDFs of the marker and checkerboard (added)
├── scripts/
│   └── configure_c920.sh             # inspect / lock focus, exposure, zoom with v4l2-ctl (added)
├── docs/
│   ├── 01_camera_calibration_guide.md
│   ├── 02_c920_configuration.md
│   └── 03_aruco_and_pose_estimation.md
└── results/                          # calibration file, reports, logs, snapshots (git-ignored)
```

Changes from the suggested layout, and why:
- **`common/`**: the four programs share opening the camera, the config and ArUco code. One copy means they
  cannot drift apart. In particular, calibration and detection then open the camera **identically**.
- **`tools/`** and **`scripts/`**: printing the marker at an exact size and locking the C920 controls are part of
  getting correct metric results, so they live in the repository.
- **`docs/`**: the theory and the hardware notes are kept with the code they explain.
- `camera_calibration.npz` is written to **`results/`** (git-ignored), because it is personal to your camera.

Programs are run **from the repository folder** as `python3 <folder>/<program>.py`. Each adds the repository
root to `sys.path`, so `common/` is found without installing anything.

---

## Ubuntu execution guide

### 1. Clone the repository
```bash
sudo apt update
sudo apt install git
cd ~
git clone https://github.com/CrazAzartto/autonomous_drone_landing.git
cd autonomous_drone_landing
```

### 2. Install Python dependencies (in a virtual environment)
```bash
sudo apt install python3-venv python3-pip
python3 -m venv .venv
source .venv/bin/activate              # do this in every new terminal
pip install --upgrade pip
pip install -r requirements.txt
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco, 'ArucoDetector'))"   # expect 4.x  True
```
A virtual environment keeps this project's NumPy/OpenCV separate from ROS 2's system packages.
Do **not** also install `opencv-python` (it conflicts with `opencv-contrib-python`).

### 3. Install the camera utilities
```bash
sudo apt install v4l-utils             # v4l2-ctl
sudo apt install guvcview              # optional GUI
sudo usermod -aG video $USER           # only if you get "permission denied"; then log out/in
```

### 4. Check that the C920 is detected
```bash
lsusb | grep -i logitech
v4l2-ctl --list-devices
```

### 5. Identify the correct `/dev/video*`
```bash
python3 detection/camera_info.py --list
```
Use the lower of the two C920 nodes, or better the `/dev/v4l/by-id/...-video-index0` path, and write it into
`config/camera_config.yaml → camera.device`.

### 6. Check supported resolutions and controls
```bash
python3 detection/camera_info.py --formats      # look for MJPG 1280x720 @ 30 fps
python3 detection/camera_info.py --controls     # exact control names on YOUR camera
```

### 7. Configure the webcam (details: docs/02)
```bash
bash scripts/configure_c920.sh /dev/video0 autofocus        # hold target at working distance ~3 s
v4l2-ctl -d /dev/video0 --get-ctrl=focus_absolute            # note the value, e.g. 30
bash scripts/configure_c920.sh /dev/video0 lock 30 100 64    # focus=30, exposure 10 ms, gain 64
python3 detection/camera_info.py                             # live view: check brightness, ~30 fps measured
```
Then copy the values that work into `camera.v4l2_controls` in the config, so every program re-applies them.

### 8. Print, measure, capture calibration images
```bash
python3 tools/generate_print_sheets.py       # results/print_checkerboard.pdf + results/print_aruco_marker.pdf
# print at 100 %, measure: 6 squares / 6 -> checkerboard.square_size_mm ; black square -> aruco.marker_length_mm
python3 calibration/capture_calibration_images.py --clear     # SPACE save, u undo, q quit; 20-30 images
```

### 9. Run the calibration
```bash
python3 calibration/calibrate_camera.py      # add --show to step through detections
```

### 10. Check calibration quality
- RMS reprojection error **< 0.5 px** (under 1.0 is acceptable), `fy/fx ≈ 1.00`, `(cx, cy)` near the centre
- open `results/undistort_check.png`: straight board edges must be straight on the right half
- full report: `results/calibration_report.txt`

### 11. Run ArUco detection and pose estimation
```bash
python3 detection/aruco_pose_estimation.py           # live; q quit, s snapshot, p pause
python3 detection/aruco_pose_estimation.py --log     # also save results/pose_log.csv
```
Tape-measure test: marker parallel to the camera at 50 cm and 100 cm from the lens → **Z within about 2–3 %**.

---

## Configuration reference (`config/camera_config.yaml`)

| Key | Meaning | Default | Must you measure it? |
|---|---|---|---|
| `camera.device` | index, `/dev/videoN` or `/dev/v4l/by-id/...` | `0` | check (step 5) |
| `camera.width/height/fps/fourcc` | capture mode, identical for all programs | 1280×720, 30, MJPG | no |
| `camera.v4l2_controls` | controls applied on every open | `{}` | yes (docs/02) |
| `checkerboard.inner_corners_cols/rows` | inner corners of the board | 6 × 9 | match your print |
| `checkerboard.square_size_mm` | one square | 25.0 | **yes** |
| `aruco.dictionary`, `aruco.marker_id` | marker type and ID | DICT_4X4_50, 0 | no |
| `aruco.marker_length_mm` | black square side | 50.0 | **yes** |

## Troubleshooting

| Symptom | Likely cause → fix |
|---|---|
| `Could not open camera` | wrong device (step 5); another app uses it; not in `video` group |
| only ~10 fps at 1280×720 | YUYV instead of MJPG (`camera_info.py` shows the fourcc); exposure too long in dim light; `exposure_dynamic_framerate=1` |
| `camera did NOT accept the configured resolution` | mode not supported; check `--formats` |
| pose program refuses to start: size mismatch | calibration made at another resolution → recalibrate |
| Z wrong by a constant % | `marker_length_mm` or `square_size_mm` not measured correctly |
| marker flickers in and out | blur or exposure: lock exposure, add light, larger marker |
| `ImportError ... _ARRAY_API` / NumPy errors | old OpenCV with NumPy 2 → `pip install -r requirements.txt` (needs OpenCV ≥ 4.10) |
| `cv2.aruco has no attribute ArucoDetector` | `opencv-python` installed instead of contrib → `pip uninstall -y opencv-python && pip install -r requirements.txt` |

## Next stages (not in this repository yet)
image-moment features (x_n, y_n, a_n, α) → feature error → controller → RC commands → Pluto.
