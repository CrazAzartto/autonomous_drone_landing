# 02 · Setting up the Logitech C920 on Windows

> **Tags:** **[Verified]** = checked in a source or in tests, **[Recommendation]** = engineering advice,
> **[Assumption]** = depends on your camera/driver, **you must check it on your PC**.

---

## 1. What we need from the camera

| Need | Why | Setting |
|---|---|---|
| the **same resolution** in calibration and flight | the calibration (`fx, fy, cx, cy`) is only valid for one resolution | `camera.width/height` in the config; every program checks it |
| **30 fps** | the controller can only react as often as it gets a new position | `fps: 30` + `fourcc: "MJPG"` |
| **sharp** marker at the flying height | blurred corners = noisy position | manual focus for about 50–100 cm |
| **short exposure** | a moving drone smears in a long exposure | manual exposure (§4) |
| **no digital zoom** | zoom crops the image and changes `fx` | `zoom: 100` (or leave it untouched at 100) |

## 2. Facts about the C920

- 1080p/30 fps and 720p/30 fps, **78° diagonal** field of view, autofocus. **[Verified: Logitech product page]**
- At 1280×720 the C920 reaches 30 fps only in the compressed **MJPG** format. The uncompressed YUY2 format is limited
  to roughly 10 fps at that size. **[Assumption: widely reported for the C920; check with `camera_info.py`, which measures the real frame rate]**
- The zoom is **digital** only (100 = no zoom). **[Verified: Logitech specs list no optical zoom]**

## 3. How the programs open the camera on Windows

All programs call `common/camera.py → open_camera()`, which:

1. opens `camera.device` (a number: 0, 1, 2 …) with the **DirectShow** driver (`backend: "dshow"`);
2. asks for `MJPG`, then `1280×720`, `30 fps` (the format is requested before and after the size, because some
   DirectShow drivers only accept MJPG once the size is set);
3. applies every non-`null` value in `camera.controls` and **reads it back**, printing `OK`, `SET?` (the driver
   accepted it but reports another value) or `REJECTED`;
4. checks the real size and refuses to continue if it is not the configured one.

**Why DirectShow and not Media Foundation (MSMF)?** **[Recommendation]** With DirectShow, OpenCV can open the
camera's own **settings window** (`CAP_PROP_SETTINGS`) and set MJPG reliably. MSMF is selectable in the config
(`backend: "msmf"`) if DirectShow ever fails on your PC.

### Which index is the C920?

```
python detection\camera_info.py --list
```

prints the camera names (needs the `pygrabber` package from `requirements.txt`) and tests indices 0–5.
A laptop's built-in webcam is usually index 0 and the C920 index 1. **[Assumption]** The order can change
when you plug the C920 into another USB port, so run `--list` again if the wrong camera opens.

## 4. Focus, exposure, gain, zoom: the easy way (settings window)

Units of these values differ between drivers, so the reliable way on Windows is to adjust them visually and let the
program read them back.

1. `python detection\camera_info.py --settings`
2. The C920's settings window opens (tabs "Video Proc Amp" and "Camera Control"). Hold the marker or the
   checkerboard about **70 cm** from the lens (your typical flying height).
3. **Focus:** leave *Auto* ticked for 3 seconds so it focuses, then **untick Auto**. The focus stays where it is.
4. **Exposure:** untick *Auto*. Lower the exposure until a waved hand is no longer smeared,
   then raise the *Gain* until the image is bright enough. Short exposure + some noise is better than motion blur.
   *Illustrative:* DirectShow exposure −6 ≈ 15.6 ms, −7 ≈ 7.8 ms, −8 ≈ 3.9 ms (the value is log₂ of seconds).
5. **Zoom:** 100. **Power line frequency** (Video Proc Amp tab): 50 Hz in India; this stops flicker under tube lights.
   **[Recommendation]**
6. Close the window and press **`p`** in the video window. The program prints lines such as

   ```
   controls:
     autofocus: 0
     focus: 30
     exposure: -7
     gain: 120
     zoom: 100
   ```

7. Copy them into `config\camera_config.yaml → camera.controls`. From now on **every** program
   (calibration, pose, controller) sets the same values when it opens the camera.
8. Run `python detection\camera_info.py` again. The control lines should say `OK`, and the measured frame rate should be about 30 fps.

`auto_exposure` is left `null` on purpose: on DirectShow, writing a manual `exposure` normally switches auto-exposure
off by itself. **[Assumption: driver dependent]** If the read-back shows the exposure still changing, set it in the
settings window instead, and check with `p`.

## 5. Which settings matter for the calibration?

| Setting | Changes the geometry (recalibrate if changed)? |
|---|---|
| resolution | **yes** |
| zoom (≠ 100) | **yes** |
| focus | **yes** for best accuracy (small change of the focal length) |
| exposure, gain, brightness, white balance, power-line frequency | no |

So: **fix the focus and zoom first, then calibrate.** Exposure can be re-tuned later without recalibrating.

## 6. Camera on the floor looking up: lighting

- Ceiling lamps directly above the camera make the marker a dark shape against a bright background. Move the setup
  so no lamp is in the image, or reduce the exposure until the marker's white cells are clearly white.
- Matte paper only (glossy paper reflects the lamps).
- The white margin around the black square must stay visible and white. The detector needs it.

## 7. Problems

| Symptom | Fix |
|---|---|
| `Could not open camera N` | wrong index (`--list`); close the Camera app, Teams, Zoom or the browser; Windows Settings → Privacy & security → Camera → allow desktop apps |
| the built-in webcam opens instead of the C920 | change `camera.device` (usually 1) |
| ~10 fps at 1280×720 | `fourcc` not MJPG, or exposure too long in a dark room (auto-exposure lowers the frame rate). Use manual exposure |
| control shows `REJECTED` | that camera has no such control; set it to `null` |
| settings window does not open | only with `backend: "dshow"` |
| image flickers / bands | power-line frequency 50 Hz in the settings window |
