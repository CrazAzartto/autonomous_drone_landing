# 04 · Step-by-step guide on Windows: from a fresh terminal to live drone errors

This is the complete procedure, in order. Do not skip steps. Each step says **what you should see**.
If you see something else, look in the "If it goes wrong" line under that step.

Folders on your laptop:

```
D:\Study Materials\BTP\codebase\
├── .venv\                         <- Python 3.12 environment with all libraries (already set up)
├── pluto_course\                  <- your manual-flight code + the plutocontrol library
└── autonomous_drone_landing\      <- THIS project
```

---

## Part A: once per terminal

### A1. Open a terminal in the project folder
In VS Code: **File → Open Folder… → `D:\Study Materials\BTP\codebase\autonomous_drone_landing`**, then
**Terminal → New Terminal** (a PowerShell terminal opens at the bottom).

Or press Win, type `powershell`, Enter, and:
```powershell
cd "D:\Study Materials\BTP\codebase\autonomous_drone_landing"
```
(the quotes are needed because of the space in "Study Materials").

### A2. Activate the Python environment
```powershell
..\.venv\Scripts\Activate.ps1
```
You should see **`(.venv)`** at the start of the prompt. Do this in every new terminal.

*If it goes wrong:* "running scripts is disabled on this system" → run this once, answer `Y`, then repeat A2:
```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```
Alternative without activation: write `..\.venv\Scripts\python.exe` instead of `python` in every command below.
In **cmd.exe** (not PowerShell) activation is `..\.venv\Scripts\activate.bat`.

In VS Code also choose the interpreter once: **Ctrl+Shift+P → "Python: Select Interpreter" → Enter interpreter path →
`D:\Study Materials\BTP\codebase\.venv\Scripts\python.exe`**.

### A3. Check (first time only)
```powershell
python --version
python -m pip install -r requirements.txt
python -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco, 'ArucoDetector'))"
```
Expected: `Python 3.12.x`, pip says everything is already satisfied (or installs `pygrabber`), then `4.x.x True`.

**All commands below are typed in this terminal, in the `autonomous_drone_landing` folder, with `(.venv)` active.**

---

## Part B: print and measure (once)

### B1. Generate the sheets
```powershell
python tools\generate_print_sheets.py
```
Creates `results\print_aruco_marker.pdf` and `results\print_checkerboard.pdf`.
(If you already printed the 6 × 9 checkerboard with 25 mm squares and the 50 mm DICT_4X4_50 ID 0 marker, you can reuse them.)

### B2. Print correctly
Open each PDF → Print → **Scale: 100 % / "Actual size"** (not "Fit to page"), matte paper.
Check the **100 mm ruler** printed on the marker sheet with a real ruler. If it is not 100 mm, the printer scaled it, so print again.

### B3. Checkerboard
1. Glue it **completely flat** on stiff cardboard or a clipboard. No bubbles, no bending.
2. With a steel ruler measure across **6 squares** (left edge of square 1 → left edge of square 7), divide by 6.
3. Open `config\camera_config.yaml` and set `checkerboard → square_size_mm` to that value, e.g. `24.9`.

### B4. Marker
1. Glue the marker sheet on thin card. Before cutting, draw an **arrow on the back** pointing to the printed TOP edge.
2. Cut on the dashed line. **Keep the white margin** all around the black square.
3. Measure the **black square** (outer edge to outer edge) in both directions; average.
4. Set `aruco → marker_length_mm` in the config, e.g. `49.8`. This number sets the scale of every position.

---

## Part C: the camera (once, and again whenever you change focus/zoom/resolution)

### C1. Plug in the C920 and find its index
```powershell
python detection\camera_info.py --list
```
Expected (your laptop, checked on 31 Oct 2026):
```
=== Cameras on this PC ===
  index 0: USB2.0 HD UVC WebCam
  index 1: HD Pro Webcam C920
```
followed by the probe of which indices deliver images. The config already has `device: 1`. Change it only if the C920 shows another index.

*If it goes wrong:* no C920 in the list → another USB port, wait 5 s, repeat. "Could not open" → close the Windows
Camera app / Teams / Zoom / browser tabs that use the camera; check **Settings → Privacy & security → Camera →
Let desktop apps access your camera = On**.

### C2. Check resolution and frame rate
```powershell
python detection\camera_info.py
```
Expected in the terminal: `1280x720`, `pixel format: MJPG`, `measured: ~30 fps`, then a live video window.
Press `q` to close.

### C3. Fix focus, exposure, zoom (docs/02 §4 explains why)
```powershell
python detection\camera_info.py --settings
```
1. Hold the checkerboard about **70 cm** from the lens (your flying height).
2. In the settings window, tab **Camera Control**: wait until the image is sharp with *Auto* focus on, then **untick
   Focus → Auto**. Untick **Exposure → Auto** and drag the slider down until a waved hand is not blurred.
   Zoom = 100.
3. Tab **Video Proc Amp**: raise **Gain** if the image is too dark. **PowerLine Frequency = 50 Hz**.
4. Click OK. Click the **video window** (so it has keyboard focus) and press **`p`**.
5. The terminal prints a `controls:` block. Copy the numbers into `config\camera_config.yaml → camera → controls`
   (replace the `null`s), save the file.
6. Run `python detection\camera_info.py` again. Each control line should end with **OK**, and the image should look the same.

From now on **never change focus or zoom** (exposure and gain may change later). If you do change them, redo Part D.

---

## Part D: camera calibration

### D1. Capture 20–30 checkerboard images
```powershell
python calibration\capture_calibration_images.py --clear
```
A live window opens. When the **whole** board is visible, coloured corners are drawn on it.
Press **SPACE** to save (it is refused when the image is blurred: hold still). `u` = undo the last one, `q` = finish.

Poses to collect (the blue overlay shows where the board has been. Fill the **whole** image, including the corners):

| # | Pose |
|---|---|
| 1–4 | board flat-on, in each of the 4 image corners, ~60 cm |
| 5–8 | board touching the top, bottom, left and right edges |
| 9–14 | centre and edges, board **tilted** ~30° left, right, up, down |
| 15–18 | close (~40 cm) and far (~100 cm) |
| 19–24 | rotated in-plane by ~15–30°, mixed positions |

Tips: move the **board**, not the camera; light from the side, no glare on the board.

*If it goes wrong:* corners never appear → whole board not visible, glare, or `inner_corners_cols/rows` do not match
your board (count the **inner** corners: 6 × 9 for the generated sheet).

### D2. Compute the calibration
```powershell
python calibration\calibrate_camera.py
```
Read the printed report:

| Line | Good value |
|---|---|
| `RMS reprojection error` | **< 0.5 px → GOOD** (0.5–1.0 acceptable) |
| `fx`, `fy` | nearly equal (`fy/fx ≈ 1.00`). *Illustrative:* ~900–950 px at 1280×720 for a C920. Your value is what counts |
| `cx, cy` | near (640, 360) |
| `field of view` | diagonal in the region of 78° (C920 specification) |

Open `results\undistort_check.png`: on the right half, the straight board edges must look straight.
Files written: `results\camera_calibration.npz` (used by all later programs), `results\calibration_report.txt`.

*If it goes wrong:* RMS > 1 px → recapture (blur, bent board). Warning "principal point far from centre" →
add images with the board in the image corners. Images dropped automatically are listed with their error.

### D3. Verify with a tape measure (important for your report)
```powershell
python detection\aruco_pose_estimation.py
```
Hold the marker **parallel** to the camera, facing it:
1. at **50 cm** from the front of the lens (tape measure) → `Z` should read **48.5–51.5 cm**;
2. at **100 cm** → `Z` ≈ **97–103 cm**;
3. move it right in the image → `X` grows; move it down in the image → `Y` grows.

Write the readings down (a table for your report). If Z is off by the **same %** at both distances, re-measure
`marker_length_mm` (B4) first.

---

## Part E: the real setup (camera on the floor, marker under the drone)

### E1. Mount the marker under the drone
- Flat, centred under the drone, printed side **down**, the **TOP arrow pointing to the drone's FRONT**.
- Double-sided tape on the battery / frame underside. Nothing (legs, wires) may cover the white margin.
- Weigh it: card + tape should be a few grams (the Pluto has little payload margin). **[Assumption: check hover throttle afterwards]**

### E2. Place the camera
- C920 on the floor (or a low box), **lens straight up**, fixed with tape so it **cannot move or rotate**.
- Optional protection: a clear acrylic/glass sheet on spacers above it. A thin flat sheet barely changes the image;
  keep it clean and free of reflections (for best accuracy, calibrate with it in place).
- No ceiling lamp directly in the camera image (docs/02 §6).
- In `config\camera_config.yaml → target → hover_height_m`, set the desired hover height **above the lens** (default 0.80 m).

### E3. Record the target and the "forward" direction
```powershell
python tools\set_target.py
```
1. Hold the drone (motors off) level, **directly above the spot where it should hover**, at roughly the hover height,
   **nose pointing in the direction you want to call forward** (e.g. towards a wall).
2. When `CAMERA X/Y/Z` is shown in green, hold still and press **`t`**. It averages 30 measurements and saves
   `results\target.json`. The terminal shows the spread; above 2 cm → press `t` again holding stiller.
3. **Check the signs now (most important check of the whole project).** Watch the yellow **ERROR** line while you move
   the drone by hand:

| You move the drone… | Must happen |
|---|---|
| backwards (towards its tail) | `fwd` becomes **positive** |
| forwards | `fwd` becomes negative |
| to its **right** | `left` becomes **positive** |
| lower | `up` becomes **positive** |
| turn its nose to the **right** | `yaw` becomes **positive** (it must turn left to recover) |

(Positive error = the direction the drone must move to get back to the target.)
`python tools\set_target.py --show` prints the saved target. **If the camera is moved or rotated, run E3 again.**

### E4. Live pose, world position and errors
```powershell
python detection\aruco_pose_estimation.py --log
```
On screen: `CAMERA` (raw camera frame), `WORLD` (x forward, y left, z up, from the target), `ERROR` (what the
PID will get), `VEL` (Kalman velocity). Keys: `q` quit, `s` snapshot, **`r` record the raw video**.
The log goes to `results\pose_log.csv` (one row per frame: raw, world, filtered, velocity, errors, image features).

Use `r` to record a manual flight once. Afterwards you can replay it as often as you like **without flying**:
```powershell
python detection\aruco_pose_estimation.py --video results\recording_XXXXXXXX.avi
```
This is the best way to test your controller code (Part F) on real data.

---

## Part F: the controller (pose is finished; PID + RC are yours)

### F1. Dry run (no drone, safe)
```powershell
python control\run_controller.py
```
Click the camera window, then: `h` = start HOLD (needs E3 and a visible marker), `l` = LANDING, `SPACE` = stop,
`q` = quit. The bottom of the window and the terminal show the mode, the errors and the RC values.
At this point they stay **1500** and the terminal says `[PID pitch] update() not implemented yet`. This is expected.
`results\control_log.csv` is written; `python control\plotting.py` plots it.

### F2. Your work
Follow **docs/05_adding_pid_and_rc.md**: write `PID.update()` (control/pid_controller.py) and the RC mapping in
`compute_rc()` (control/run_controller.py), check every sign with the propellers **off**, then tune.

### F3. Flying
```powershell
python control\run_controller.py --fly --plot
```
Connect the laptop to the **Pluto's Wi-Fi** first. The camera is USB, so it keeps working. Keys as in your manual
scripts: `y` arm, `t` take off, then `h` for autonomous hold over the target, `l` to land, `SPACE` to stop the
autonomous mode, `q` to land, disarm and quit. The library is used directly from `..\pluto_course\plutocontrol`.

---

## Quick reference

| Task | Command |
|---|---|
| list cameras | `python detection\camera_info.py --list` |
| live camera + settings | `python detection\camera_info.py --settings` |
| print sheets | `python tools\generate_print_sheets.py` |
| capture calibration images | `python calibration\capture_calibration_images.py --clear` |
| calibrate | `python calibration\calibrate_camera.py` (`--show` to step through) |
| live pose (+ log) | `python detection\aruco_pose_estimation.py --log` |
| pose of one photo / a video | `... --image file.png` / `... --video file.avi` |
| record the target | `python tools\set_target.py` (`--show` to print it) |
| controller dry run / flight | `python control\run_controller.py` / `... --fly --plot` |
| plot a control log | `python control\plotting.py [file.csv]` |

All settings: `config\camera_config.yaml`. All outputs: `results\` (not uploaded to GitHub).
