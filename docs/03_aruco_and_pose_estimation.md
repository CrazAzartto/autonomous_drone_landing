# 03 · ArUco markers and 3-D pose estimation

> *Illustrative* numbers are invented to explain the maths, not measurements.
> **[Verified]** items were checked in the OpenCV versions listed in the README or in tests.

---

## 1. Dictionaries and IDs

An ArUco marker is a black square border around an `n × n` grid of black/white cells (bits).
A **dictionary** is a fixed list of such bit patterns, chosen to be far apart from each other
(so a misread bit does not turn one ID into another). The marker **ID** is simply the index in that list.

Name format: `DICT_<n>X<n>_<count>`, e.g. `DICT_4X4_50` = 4 × 4 data bits, 50 different markers.

| Choice | Effect |
|---|---|
| fewer bits (4×4) | bigger cells for the same printed size → readable from **farther away** and with more blur |
| more bits (6×6) | more IDs, more error correction, but smaller cells |
| fewer markers in the dictionary (`_50`) | patterns more distinct from each other → fewer false IDs |

**Chosen: `DICT_4X4_50`, ID 0 [Recommendation].** You need only a few IDs (drone marker, later a second
small marker for the final approach). A 4×4 code keeps cells as large as possible on a small marker on a
nano drone. Both values are set in `config/camera_config.yaml` and can be changed freely.
If you change them, **regenerate and reprint** the marker.

## 2. Generating, printing and measuring the marker

```
python tools\generate_print_sheets.py
# -> results\print_aruco_marker.pdf  (dictionary, ID and size from the config)
```

1. Print at **100 % / Actual size**. Check the 100 mm ruler on the sheet with a real ruler.
2. Use matte paper glued flat on thin card. Cut on the dashed line, **keeping the white margin**
   (the detector needs white around the black border).
3. **Measure the black square**, from the outer edge of the black border to the opposite outer edge, in both
   directions, with a steel ruler. Enter the average in `aruco.marker_length_mm`.

### Why the marker size must be exact

A camera cannot tell "small and near" from "big and far". The only metric information in
`solvePnP` is the marker size you supply. If the marker really is 48 mm but you type 50 mm, every coordinate is scaled
by 50/48: **X, Y and Z are all 4.2 % too large** (a 75 cm distance reads about 78 cm). The orientation is not affected.

## 3. Coordinate frames

### Camera frame (OpenCV convention) **[Verified: OpenCV pinhole model]**
- origin at the camera's optical centre (inside the lens)
- **+Z** points **out of the lens** along the optical axis (straight ahead)
- **+X** points to the **right** in the image
- **+Y** points **down** in the image
- units: whatever unit you give the marker size in. This code uses **metres** and displays **centimetres**.

### Marker frame **[Verified in tests]**
- origin at the **centre** of the black square
- **+x** to the marker's right, **+y** to the marker's top (as printed)
- **+z** out of the **printed face**. When the marker faces the camera, its +z points **towards** the camera.
- Corners are returned in the order 0 = top-left, 1 = top-right, 2 = bottom-right, 3 = bottom-left (as printed).
  The live view numbers them, so you can check this.

```python
# common/aruco_utils.py
object_points = [[-L/2,  L/2, 0],   # 0 top-left
                 [ L/2,  L/2, 0],   # 1 top-right
                 [ L/2, -L/2, 0],   # 2 bottom-right
                 [-L/2, -L/2, 0]]   # 3 bottom-left
```

## 4. What `rvec` and `tvec` mean

`solvePnP` returns the transformation that takes a point from the **marker** frame into the **camera** frame:

```
P_camera = R · P_marker + t          with  R = Rodrigues(rvec)
```

- **`tvec = t = (X, Y, Z)`** is the position of the **marker centre in the camera frame**. This is the X/Y/Z the program shows.
- **`rvec`** is an axis–angle vector: its direction is the rotation axis and its length the angle in radians.
  `cv2.Rodrigues(rvec)` turns it into the 3×3 rotation matrix `R`, whose **columns** are the marker's x, y, z axes
  expressed in camera coordinates.
- A marker lying flat and **facing** the camera has its +z pointing back at the camera, opposite to the camera's +Z.
  So `R` is about a **180° rotation about X**, not the identity. This is expected. (Our own test generator first
  got this wrong and produced a mirror image, which ArUco correctly refused to decode.)
- The program also shows a **heading**: the direction of the marker's **+y axis** (= the drone's nose, because the
  printed TOP edge points to the drone's front) projected on the camera's x–y plane,
  `heading = atan2(R[1,1], R[0,1])`. With the camera looking up, this angle is counter-clockwise **seen from above**,
  so a larger heading = the drone turned LEFT. Using the projection keeps it correct when the drone is tilted.

## 5. How the pose is computed

1. **Detection:** `cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)`. It thresholds the image,
   finds square contours, reads the bits, checks them against the dictionary, and returns the 4 corners
   (refined to sub-pixel with `CORNER_REFINE_SUBPIX`) and the ID.
2. **Perspective-n-Point (PnP):** with 4 known 3-D points (§3), their 4 pixels, and `K` plus distortion,
   find the `R, t` that best reproject the 3-D corners onto the detected pixels:
   ```
   min over R,t   Σ_k || x_k − π(K, d, R·P_k + t) ||²
   ```
   For a square planar target, `cv2.SOLVEPNP_IPPE_SQUARE` (Infinitesimal Plane-based Pose Estimation) gives an
   analytic solution from the plane homography, then refines it. It is the method OpenCV recommends for a single marker.
3. **Quality check:** the 4 corners are re-projected with the found pose. The RMS pixel difference is shown as
   `reproj`. Above `detection.max_reproj_error_px` (2 px) the pose is **rejected**: it never reaches the filter
   or the controller, and the screen shows the reason.

**API note [Verified with opencv-contrib-python 4.10.0 and 4.14.0]:** `cv2.aruco.ArucoDetector`,
`cv2.aruco.generateImageMarker`, `cv2.solvePnP(..., SOLVEPNP_IPPE_SQUARE)` and `cv2.drawFrameAxes` all exist and
work in both versions. The older `cv2.aruco.estimatePoseSingleMarkers` still exists but is **deprecated**, so it is not used.
The old free function `cv2.aruco.detectMarkers` is also not used.

## 6. Numerical example (illustrative values only)

Assume a calibration of `fx = fy = 900 px`, `cx = 640`, `cy = 360` (1280×720), a 50 mm marker, and a program output of

```
tvec = (X, Y, Z) = (+0.080, −0.040, +0.750) m   →  shown as X = +8.0 cm, Y = −4.0 cm, Z = 75.0 cm
```

Interpretation:
- **Z = 75.0 cm**: the marker centre is 75 cm in front of the lens, measured **along the optical axis**.
- **X = +8.0 cm**: 8 cm to the **right** of the optical axis, as seen in the image.
- **Y = −4.0 cm**: 4 cm **above** the optical axis in the image (negative Y = towards the top of the image).
- Straight-line distance = √(0.08² + 0.04² + 0.75²) = **75.5 cm**.

Check with the pinhole equations (where the marker centre should appear):
```
u = 900 · 0.080 / 0.750 + 640 = 736 px      (right of the 640 centre)
v = 900 · (−0.040) / 0.750 + 360 = 312 px   (above the 360 centre)
marker width in the image ≈ 900 · 0.050 / 0.750 = 60 px
```

**For your setup (camera on the floor looking UP at a marker under the drone):**
Z is the height of the marker above the lens. Which room direction is camera +X or +Y depends on how the
webcam happens to be rotated on the floor. **You do not need to work this out:** `tools\set_target.py` (§8)
records the drone's forward direction once, and all errors are then given in the drone's own frame.

## 7. Limitations of monocular marker pose

| Effect | What happens | Size of the effect |
|---|---|---|
| **Depth is the weakest axis** | Z comes from the marker's apparent size: `Z ≈ fx·L / s_px` | pixel noise σ on the size gives σ_Z ≈ Z²·σ/(fx·L). *Illustrative:* Z = 0.75 m, fx = 900, L = 0.05 m, σ = 0.3 px → 3.8 mm; at 1.5 m → 15 mm |
| Lateral axes are much better | X ≈ Z·(u − cx)/fx | same example: σ_X ≈ Z·σ/fx ≈ 0.25 mm |
| **Wrong marker size** | everything scales | 1 % size error = 1 % X, Y, Z error |
| **Calibration error** in fx | Z scales with fx | 1 % fx error ≈ 1 % Z error; distortion errors grow towards image edges |
| **Blur / rolling shutter** | corners smeared or skewed while moving | shorter exposure (docs/02). Consumer webcams typically use rolling-shutter sensors **[Assumption]** |
| **Viewing angle** | a small, nearly frontal marker has an ambiguous tilt (two IPPE solutions); orientation can flip | position stays stable, but roll/pitch of the marker become noisy. Prefer larger markers or several markers for orientation |
| **Too small in the image** | below about 20–25 px the bits are unreadable | 50 mm at 1.5 m with fx = 900 → 30 px (marginal) |
| **Partial visibility** | if any corner leaves the frame, there is no detection | keep the working area inside the field of view |
| **Lighting** | backlight from ceiling lamps makes a dark marker on a bright background; glare on glossy paper | matte print, fixed exposure, avoid lamps directly above an upward camera |

## 8. World frame, target and drone errors (`vision/target.py`)

The camera frame is awkward for control: its axes depend on how the webcam lies on the floor, and pitch/roll act in
the **drone's** frame, which rotates with yaw. So the code defines:

1. **Target / world frame** - recorded once with `python tools\set_target.py`: put the drone where it should hover,
   nose pointing in the direction you call "forward", press `t`. Saved in `results\target.json`:
   - origin = that marker position (x, y); **X_w = forward** (the drone's heading at that moment), **Y_w = left**,
     **Z_w = up** (= height above the lens).
2. **World position** of the drone: `p_w = Rz(−heading_ref) · (p_cam − p_target)`, and **yaw** `ψ = heading − heading_ref`.
3. **Errors in the drone's own frame** (what the PID needs; positive = the drone must go/turn that way):

   ```
   e_forward =  cos ψ · (−x_w) + sin ψ · (−y_w)      -> pitch
   e_left    = −sin ψ · (−x_w) + cos ψ · (−y_w)      -> roll
   e_up      = hover_height − z_w                    -> throttle
   e_yaw     = −ψ   (wrapped to ±180°)               -> yaw   (positive = turn LEFT)
   ```

**[Verified in synthetic tests]** (`tracker_test.py`): drone 10 cm *behind* the target → `e_forward = +10.0 cm`;
10 cm to the *right* → `e_left = +10.2 cm`; turned 20° left → `e_yaw = −20.4°`; turned 90° left and 10 cm behind
the target in the world → `e_forward ≈ 0`, `e_left = −10.0 cm` (the target is now on its right).

## 9. Kalman filter: smooth position and velocity (`vision/kalman.py`)

The PID's D-term needs velocity. Differentiating a noisy position frame-to-frame amplifies the noise, so a
**constant-velocity Kalman filter** (state = position and velocity in x, y, z) is run on every accepted pose,
in the world frame. Settings in `tracking:` of the config:

| Key | Meaning |
|---|---|
| `process_accel_std` (m/s²) | how fast the velocity may change; larger = reacts faster, noisier |
| `measurement_std_xy`, `measurement_std_z` (m) | noise of one camera measurement (Z is noisier, §7) |
| `max_lost_time_s` | if the marker is not seen for longer, the state is **invalid** and the controller must stop correcting |

When a frame has no marker, the filter **predicts** (keeps moving with the last velocity) until `max_lost_time_s`.
**[Verified in synthetic tests]** a marker moving at 0.20 m/s backwards gave a filtered `v_forward = −0.203 m/s`
(standard deviation 0.007 m/s) and `v_up = +0.109 m/s` for a true 0.10 m/s.
For a fixed target, the error rate the D-term needs is simply `de/dt = −v`.

## 10. Image-moment features of Papers 1/2 (`vision/features.py`)

For the IBVS formulation the same detection also gives the normalised image-moment features, computed from the
undistorted corners (`x_g, y_g` = centroid in normalised coordinates, `a` = area):

```
a* = 2·L² / Z*²   (desired area at the desired depth Z* = hover_height)
a_n = Z* · sqrt(a* / a),     x_n = a_n · x_g,     y_n = a_n · y_g,     α = angle of the edge corner 0 -> 1
```

Here `a = μ20 + μ02` (centred second-order moments of the 4 corners), which equals `2L²/Z²` for a square seen
face-on, hence the factor 2 in `a*`. For a marker parallel to the image plane, **`a_n = Z`, `x_n = X`, `y_n = Y`**
(metres, camera frame). These features are therefore the metric position, which is why the IIT Bombay position PID
and the IBVS law of Papers 1/2 lead to the same controller structure (see the report). They are logged
(`x_n, y_n, a_n, alpha_deg` in `pose_log.csv`) and available as `state.features`. The controller template uses the
metric errors of §8, which are also valid when the drone is tilted.

## Sources
- OpenCV ArUco tutorial: <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>
- OpenCV `solvePnP` / `SOLVEPNP_IPPE_SQUARE`: <https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html>
- S. Garrido-Jurado et al., "Automatic generation and detection of highly reliable fiducial markers under occlusion",
  Pattern Recognition 47(6), 2014.
- T. Collins, A. Bartoli, "Infinitesimal Plane-Based Pose Estimation", IJCV 109, 2014.
