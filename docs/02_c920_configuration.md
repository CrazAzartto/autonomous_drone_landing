# 02 · Configuring the Logitech C920 on Ubuntu

> **[Verified]** = from a cited source or tested; **[Recommendation]** = advice;
> **[Assumption]** = depends on your unit/kernel, **check with the commands shown**.
> Control names and ranges below are what C920 users have reported. **Your camera's own
> `v4l2-ctl --list-ctrls` output is the only authority.** Do not copy values blindly.

---

## 0. Tools

```bash
sudo apt update
sudo apt install v4l-utils          # provides v4l2-ctl
sudo apt install guvcview           # optional GUI to play with controls
groups | grep -q video || sudo usermod -aG video $USER    # then log out and back in
```

## 1. Find the camera

```bash
lsusb | grep -i logitech                 # is it on the USB bus?
v4l2-ctl --list-devices                  # which /dev/video* nodes belong to it
ls -l /dev/v4l/by-id/                    # stable names (do not change between reboots/ports)
python3 detection/camera_info.py --list  # the same, from this repository
```

One webcam normally creates **two** nodes (e.g. `/dev/video0` = video, `/dev/video1` = metadata).
Use the one that delivers images, normally the lower number. **[Verified: common Linux UVC behaviour]**

Put the device in `config/camera_config.yaml → camera.device`. The `/dev/v4l/by-id/...-video-index0` path is
the most robust choice, because it never changes when other cameras are plugged in. **[Recommendation]**

## 2. Resolution and pixel format

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
python3 detection/camera_info.py --formats      # same
```

What C920 owners report **[Verified, NVIDIA forum listing of a C920]**:

| Format | 640×480 | 1280×720 | 1920×1080 |
|---|---|---|---|
| **YUYV** (uncompressed) | up to 30 fps | **10 fps** | **5 fps** |
| **MJPG** (compressed) | 30 fps | **30 fps** | **30 fps** |
| H264 (some units only) | 30 fps | 30 fps | 30 fps |

**Consequence:** at 720p or 1080p you only get 30 fps in **MJPG**. OpenCV may pick YUYV unless asked, so the
config sets `fourcc: "MJPG"` and `common/camera.py` requests it **before** setting the size (V4L2 order matters).
`camera_info.py` prints the requested and the actual format, and **measures** the real frame rate.

### Which resolution? **[Recommendation]**

| | 640×480 | **1280×720 (chosen)** | 1920×1080 |
|---|---|---|---|
| fps (MJPG) | 30 | 30 | 30 |
| marker pixels at 1 m (50 mm marker, illustrative fx) | ~30 px | ~45 px | ~68 px |
| depth noise at 1 m (relative) | 1.5× | 1× | 0.67× |
| detection CPU time | lowest | medium | ~2.25× of 720p |
| aspect ratio / field of view | 4:3, may use a different sensor crop | 16:9 | 16:9 |

1280×720 is the balanced choice for a first stage. 1920×1080 improves depth precision by about 1.5×
(σ_Z ∝ 1/fx; see docs/03) at the cost of CPU time and latency. You can switch later, but then you must **recalibrate**.

**Keep calibration and detection identical.** All programs read `width/height/fps/fourcc` from the same YAML.
The calibration file stores the size, and `aruco_pose_estimation.py` refuses to run on a mismatch.
Changing the resolution means: edit the YAML → `capture_calibration_images.py --clear` → recalibrate.

## 3. Camera controls: inspect first

```bash
v4l2-ctl -d /dev/video0 --list-ctrls-menus
python3 detection/camera_info.py --controls          # same
bash scripts/configure_c920.sh /dev/video0 show      # formats + controls + current format
```

Names reported for a C920 on a recent kernel (Linux 6.x, Raspberry Pi OS) **[Verified, crowsnest issue #96]**:

| Control | Type / range (as reported) | Purpose |
|---|---|---|
| `auto_exposure` | menu 0–3, default 3 | 1 = Manual Mode, 3 = Aperture Priority (auto) |
| `exposure_time_absolute` | 3–2047, default 156 | exposure in units of **100 µs** (UVC); *inactive* while auto |
| `exposure_dynamic_framerate` | bool | if 1 the camera may **lower the fps** in dim light |
| `gain` | 0–255 | sensor amplification (more gain = more noise) |
| `focus_automatic_continuous` | bool, default 1 | autofocus on/off |
| `focus_absolute` | 0–250 | manual focus position; *inactive* while autofocus is on |
| `zoom_absolute` | 100–500, default 100 | **digital** zoom (crop); 100 = full field of view |
| `white_balance_automatic` / `white_balance_temperature` | bool / 2800–7500 K | colour |
| `power_line_frequency` | menu 0–2 | anti-flicker: 1 = 50 Hz, 2 = 60 Hz |
| `brightness`, `contrast`, `saturation`, `sharpness`, `backlight_compensation`, `pan_absolute`, `tilt_absolute` | | image tuning / digital pan-tilt |

**[Assumption: kernel-dependent names]** Older kernels used `exposure_auto`,
`exposure_absolute`, `exposure_auto_priority`, `focus_auto` and `white_balance_temperature_auto` for the same controls.
`scripts/configure_c920.sh` checks which names your camera reports and uses those.
In the Python config, write the names exactly as **your** `--list-ctrls` shows them; unknown names are skipped with a warning.

Ranges and steps can differ between C920 revisions. Respect the `min/max/step` your camera prints.

## 4. Exposure, gain, focus, white balance: what to set for ArUco **[Recommendation]**

Goal: **sharp** corners (no motion blur), **constant** brightness, **constant** frame rate.

1. **Exposure: manual and short.** Blur length = speed × exposure. A drone marker moving at 0.5 m/s with a 33 ms
   auto exposure smears by 1.6 cm, about 15 px at 1 m. At 10 ms that drops to about 5 px.
   - Under 50 Hz mains lighting (India), lights flicker at 100 Hz. Use exposure **100** (10 ms) or **200** (20 ms),
     integer multiples of 10 ms, plus `power_line_frequency=1`, to avoid moving dark bands.
   - Then raise `gain` until the image is bright enough; add light rather than gain if you can.
2. **`exposure_dynamic_framerate = 0`**, otherwise the C920 silently drops to 15 fps in dim light.
3. **Focus: locked.** Autofocus "hunting" changes the image scale slightly and blurs frames.
   1. `bash scripts/configure_c920.sh /dev/video0 autofocus`
   2. hold the marker or board at your working distance (about the drone height above the camera) for about 3 s
   3. `v4l2-ctl -d /dev/video0 --get-ctrl=focus_absolute` and write the number down
   4. `bash scripts/configure_c920.sh /dev/video0 lock <focus> 100 <gain>`
4. **White balance:** irrelevant for ArUco geometry (it uses grey levels), but locking it keeps images consistent.
5. **Zoom:** keep `zoom_absolute = 100`.

### Verify that a setting really changed

```bash
v4l2-ctl -d /dev/video0 --get-ctrl=auto_exposure,exposure_time_absolute,focus_automatic_continuous,focus_absolute,zoom_absolute
v4l2-ctl -d /dev/video0 --list-ctrls | grep -E "exposure|focus|zoom"   # 'inactive' flag must be gone for manual values
```

Both the script and `common/camera.py` read every control back after setting it and print
`OK` or `CHECK`. Then look at the image: a manual-exposure change must visibly change brightness.

### Make the settings permanent for this project

Controls reset when the camera is unplugged or the PC reboots. Copy the values that worked into
`config/camera_config.yaml → camera.v4l2_controls`, in the order **auto-switch first, manual value second**.
Every program then applies them when it opens the camera, so calibration and detection always match:

```yaml
  v4l2_controls:
    focus_automatic_continuous: 0
    focus_absolute: 30            # your measured value
    zoom_absolute: 100
    auto_exposure: 1
    exposure_time_absolute: 100   # 10 ms
    exposure_dynamic_framerate: 0
    gain: 64                      # your value
    power_line_frequency: 1       # 50 Hz
```

## 5. Zoom and field of view

- **[Verified]** Logitech lists the C920 as 1080p/30 fps with a **78° diagonal** field of view and autofocus.
  The product page lists **no optical zoom**. The lens has a fixed focal length.
- **[Verified, user-reported control list]** Linux exposes `zoom_absolute` (100–500). This is a **digital** zoom:
  the camera crops the centre of the sensor and scales it up.
- Digital zoom = crop = a different `fx, fy, cx, cy`. A calibration made at zoom 100 is **invalid** at any other zoom.
- Some other apps (video-call software) can change zoom or other controls. Always start experiments with
  `bash scripts/configure_c920.sh /dev/video0 lock ...` or the `v4l2_controls` in the config, and check with
  `--get-ctrl zoom_absolute`.

## 6. Fixed-configuration checklist (every session)

- [ ] same `/dev/v4l/by-id/...` device, same USB port preferably (a USB 2.0 port is enough for MJPG)
- [ ] `camera_info.py` shows **1280×720 MJPG** and a measured fps of about 30
- [ ] zoom 100, focus locked at the calibrated value, manual exposure
- [ ] calibration file created with these settings (date in `results/calibration_report.txt`)

## Sources
- C920 formats (YUYV/MJPG/H264 frame rates): NVIDIA developer forum,
  <https://forums.developer.nvidia.com/t/unable-to-get-logitech-c920-to-work-with-gstreamer-solved/65203/5>
- C920 control list on Linux 6.1 (names, ranges): <https://github.com/mainsail-crew/crowsnest/issues/96>
- Older control names (`exposure_auto`, `focus_auto`): <https://www.christitus.com/logitech-c920-linux-driver/>
- Logitech C920 specifications: <https://logitech.com/en-za/products/webcams/c920-pro-hd-webcam.html>
