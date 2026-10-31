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

```bash
python3 tools/generate_print_sheets.py
# -> results/print_aruco_marker.pdf  (dictionary, ID and size from the config)
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
- The program also shows a **yaw** angle: the rotation of the marker's x axis about the camera's optical axis,
  `atan2(R[1,0], R[0,0])`. Later this becomes the α feature (heading).

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
   `reproj`. Above about 1 px, the program warns that the pose is unreliable.

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

**For your setup (camera on the floor looking UP at a marker under the drone)** **[Assumption about your mounting]:**
Z is the height of the marker above the lens. Which room direction is +X or +Y depends on how the
webcam is rotated on the floor. Mark the room directions once: move the marker by hand towards one wall and watch
which of X or Y changes, and with which sign. Write it down; the controller will need it.

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

The poses are **raw**, one frame at a time. Filtering (for example the Kalman filter of Paper 2) is a later stage.

## Sources
- OpenCV ArUco tutorial: <https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html>
- OpenCV `solvePnP` / `SOLVEPNP_IPPE_SQUARE`: <https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html>
- S. Garrido-Jurado et al., "Automatic generation and detection of highly reliable fiducial markers under occlusion",
  Pattern Recognition 47(6), 2014.
- T. Collins, A. Bartoli, "Infinitesimal Plane-Based Pose Estimation", IJCV 109, 2014.
