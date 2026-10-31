# 01 · Camera calibration (Logitech C920, Ubuntu)

> **Tags used in these docs:** **[Verified]** = checked in a source or in tests,
> **[Recommendation]** = engineering advice, **[Assumption]** = depends on your setup, **you must measure it**.
> All numbers marked *illustrative* are made up to explain the maths. They are **not** measurements.

---

## 1. Why calibrate at all?

ArUco detection alone gives you **pixels**: four corners at, say, (700, 205), (830, 240)…
Your controller needs **metres**: "the marker is 12 cm to the right and 90 cm away".

The bridge between pixels and metres is the **camera model**. For every 3-D point
`(X, Y, Z)` in the camera frame, the ideal (pinhole) camera produces the pixel

```
u = fx · X/Z + cx
v = fy · Y/Z + cy
```

Without `fx, fy, cx, cy` (and the lens distortion) you cannot invert this. You would know
*where* the marker is in the image, but not *how far* it is or how many centimetres one pixel means.
Calibration measures these numbers for **your** camera at **your** resolution and zoom.

## 2. Intrinsic parameters and the camera matrix

```
        [ fx   0   cx ]
    K = [  0  fy   cy ]
        [  0   0    1 ]
```

| Symbol | Meaning | Unit |
|---|---|---|
| `fx`, `fy` | focal length expressed in pixels: (focal length in mm) / (pixel size in mm), horizontally and vertically | px |
| `cx`, `cy` | principal point: the pixel where the optical axis hits the sensor, usually near the image centre | px |

`fx ≈ fy` for square pixels. A large `fx` means a narrow field of view, because each degree covers more pixels.
The horizontal field of view follows from `fx`:
`HFOV = 2·atan(width / (2·fx))`. `calibrate_camera.py` prints it as a sanity check.

These are **intrinsic** parameters: they belong to the camera and do not change when the camera moves.
They **do** change if you change resolution, digital zoom or focus (see §8).

## 3. Lens distortion

Real lenses bend straight lines, especially near the image edges. OpenCV models this with five
coefficients `[k1, k2, p1, p2, k3]` applied to the normalised coordinates `x = X/Z, y = Y/Z`, `r² = x² + y²`:

```
x_d = x(1 + k1 r² + k2 r⁴ + k3 r⁶) + 2 p1 x y + p2 (r² + 2x²)      (radial + tangential)
y_d = y(1 + k1 r² + k2 r⁴ + k3 r⁶) + p1 (r² + 2y²) + 2 p2 x y
u = fx · x_d + cx ,   v = fy · y_d + cy
```

- `k1, k2, k3` **radial** distortion: barrel (lines bulge outwards) or pin-cushion.
- `p1, p2` **tangential** distortion: the lens is slightly tilted relative to the sensor.

**Why it matters for you:** a marker near the image edge would appear shifted by a few pixels.
A corner error of a few pixels becomes centimetres of position error at 1 m. The pose code passes
the distortion coefficients to `solvePnP`, which removes this effect.

## 4. Why a checkerboard?

Calibration needs many points whose **true 3-D positions are known exactly**. A printed checkerboard:

- has corners on a perfect grid with a spacing you can measure (the square size);
- has corners (X-junctions) that can be located to about **0.1 px** with `cornerSubPix`;
- is detected automatically and in a fixed order by `cv2.findChessboardCorners`.

Each photo gives about 54 point correspondences (3-D board point ↔ 2-D pixel). Twenty photos give over 1000 equations
for the handful of unknowns (`K`, distortion, and one board pose per photo). OpenCV solves this by
non-linear least squares (Zhang's method).

## 5. Choosing the board and the inner-corner count

- **Inner corners** are the points where four squares meet. A board of 7 × 10 **squares** has
  **6 × 9 inner corners**. OpenCV wants the inner-corner count (`inner_corners_cols`, `inner_corners_rows`).
- Use **different** counts in the two directions (even × odd, e.g. 6 × 9) so the detected orientation is unambiguous.
- Square size: big enough to be sharp at your working distance. 25 mm on A4 works for 0.4–1.2 m.
  **[Recommendation]**
- Generate the board for your config with `python3 tools/generate_print_sheets.py`. The default is 6 × 9 inner corners
  with 25 mm squares, the same board as the sheet already printed. **[Verified]**: the generated PDF measures 24.97 mm per square at 150 dpi rendering.

## 6. Measuring the square size accurately

1. Print at **100 % / Actual size** (never "Fit to page").
2. Glue the print **flat** onto stiff card or a clipboard. A bent board is the most common cause of poor calibration.
3. With a steel ruler, measure across **6 squares** (e.g. from the left edge of square 1 to the left edge of square 7),
   then divide by 6. Measuring many squares divides your reading error by 6.
4. Put the result into `config/camera_config.yaml → checkerboard.square_size_mm`.

The square size sets the **scale** of everything. If it is 2 % wrong, `K` stays correct, but
every board distance reported during calibration is 2 % wrong. The marker poses use the separate
`marker_length_mm`, so measure **both** carefully.

## 7. Which images to capture

**[Recommendation]** 20–30 sharp images, with the **whole** board visible in each:

| Variation | Why |
|---|---|
| board in **all four corners and along all edges** of the image | distortion is strongest at the edges; without edge data `k1..k3` are guessed |
| board **tilted** about 20–40° left/right/up/down | tilt is what makes `fx`, `fy` observable separately from distance |
| a few **near** and a few **far** (0.4–1.2 m) | covers your working range |
| a small **in-plane rotation** | breaks symmetry, helps `cx`, `cy` |

`capture_calibration_images.py` shows a "next pose" hint and a blue overlay of the areas already covered.
Keep going until the whole image has been covered at least once.

### Common mistakes to avoid
- **Motion blur**: hold the board still; the program refuses images below a sharpness threshold.
- **Glare** on glossy paper: use matte paper and avoid lights reflecting straight into the lens.
- **Changing settings between calibration and use**: resolution, digital zoom or focus (see §8).
- **Only frontal images in the centre**: gives a poor `K` even if the RMS error looks small.
- **Board partially out of frame**: not saved (the whole pattern is required).
- **Autofocus hunting during capture**: lock focus first (docs/02).

## 8. What invalidates a calibration

| Change | Effect | Recalibrate? |
|---|---|---|
| different **resolution** (e.g. 1280×720 → 1920×1080) | `fx, fy, cx, cy` scale and may change if the sensor crop differs | **yes** |
| **digital zoom** ≠ 100 | the camera crops the sensor: effective `fx` grows | **yes** |
| **focus** moved a lot | focal length changes slightly (lens "breathing") | yes, for best accuracy |
| exposure / gain / white balance | no geometric effect | no |
| camera moved or rotated | intrinsics unchanged (only *where* the camera is changes) | no |

The programs enforce the resolution rule: `aruco_pose_estimation.py` **refuses to run** if the camera delivers
a different resolution than the one stored in `camera_calibration.npz`. **[Verified in code]**

## 9. Reprojection error: what it is and what is good

After calibration, OpenCV knows `K`, the distortion and the pose of the board in every image. It then
**re-projects** every 3-D board corner `X_j` into image `i` and compares with the detected corner `x_ij`:

```
e_RMS = sqrt( (1/N) · Σ_i Σ_j || x_ij − π(K, d, R_i, t_i, X_j) ||² )      [pixels]
```

`π(·)` is the camera model of §2–3 and `N` the total number of corners. `calibrate_camera.py` prints the
overall value and the value for each image, sorted worst first.

| RMS error | Verdict for this project **[Recommendation]** |
|---|---|
| < 0.3 px | excellent |
| 0.3 – 0.5 px | good |
| 0.5 – 1.0 px | acceptable; try to improve |
| > 1.0 px | redo: blur, bent board, wrong pattern size or bad images |

Low error is **necessary but not sufficient**. 15 nearly identical frontal images can give 0.2 px and a wrong `K`.
Coverage (§7) matters as much. Always run the tape-measure check in §11.

Sanity checks printed by the program: `fy/fx ≈ 1.00`; `(cx, cy)` within about 10 % of the image centre;
a field of view plausible for the camera. Logitech specifies a **78° diagonal** FOV for the C920 [1]. Expect a
value in that region at 16:9 resolutions. Your measured value is what counts, so do not force it.

## 10. Saving and loading

`calibrate_camera.py` writes `results/camera_calibration.npz` (a NumPy archive) with:
`camera_matrix`, `dist_coeffs`, `image_width`, `image_height`, `rms_error_px`, `per_image_errors_px`,
`square_size_mm`, `pattern_cols`, `pattern_rows`, `fourcc`, `created`.

Loading in your own code:

```python
import numpy as np
data = np.load("results/camera_calibration.npz")
K = data["camera_matrix"]; dist = data["dist_coeffs"]
size = (int(data["image_width"]), int(data["image_height"]))
```

or simply `from common.calibration_io import load_calibration`.

It also writes `results/calibration_report.txt` (a readable copy) and `results/undistort_check.png`
(original | undistorted side by side; straight board edges must look straight on the right).

These files are personal to your camera and are **not** committed to Git (`.gitignore`).

## 11. Physical procedure, step by step

```bash
cd ~/autonomous_drone_landing
source .venv/bin/activate

# 0. print + measure (once)
python3 tools/generate_print_sheets.py          # -> results/print_checkerboard.pdf
#    print at 100 %, glue flat, measure 6 squares / 6 -> edit checkerboard.square_size_mm

# 1. lock the camera settings (docs/02): resolution in config, focus/exposure locked, zoom 100

# 2. capture 20-30 images (SPACE = save, u = undo, q = quit)
python3 calibration/capture_calibration_images.py --clear

# 3. calibrate
python3 calibration/calibrate_camera.py
#    check: RMS < 0.5 px, fy/fx ~ 1, (cx, cy) near the centre, look at results/undistort_check.png

# 4. verify with a tape measure
python3 detection/aruco_pose_estimation.py
#    hold the marker parallel to the camera at 50 cm and 100 cm (tape from the lens front);
#    Z should read within about 2-3 %.
```

If Z is wrong by the **same percentage** at every distance, the cause is a size problem, not the calibration:
re-measure `marker_length_mm` first, then `square_size_mm`.

## References
1. Logitech C920 HD Pro product page: 1080p/30 fps, 720p/30 fps, 78° diagonal FOV, autofocus.
   <https://logitech.com/en-za/products/webcams/c920-pro-hd-webcam.html>
2. OpenCV camera calibration tutorial: <https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html>
3. Z. Zhang, "A flexible new technique for camera calibration", IEEE TPAMI 22(11), 2000.
