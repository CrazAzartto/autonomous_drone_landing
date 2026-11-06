# 05 · Adding the PID controller and the RC commands (the remaining BTP work)

> **Update:** the project now implements Paper 1's IBVS controller (docs/06-09, `control/ibvs_run.py`). This document
> describes the earlier **position-PID template** (`control/run_controller.py`), which is kept as a Paper-3-style
> **baseline** for comparison. Its sign table and dead-band notes also apply to the IBVS program.

> **Tags:** **[Verified]** = checked in code, a source or tests, **[Recommendation]** = engineering advice,
> **[Assumption]** = must be checked on your drone. *Illustrative* numbers are examples, not tuned values.

Everything up to the error signals is finished and tested. What is left is exactly two places, both marked
`### TODO ###`:

| File | What to write |
|---|---|
| `control/pid_controller.py` → `PID.update(error, error_rate, dt)` | the PID law (about 10 lines) |
| `control/run_controller.py` → `compute_rc()` | turning the 4 PID outputs into 4 RC values (about 4 lines) |
| `control/run_controller.py` → the gains block | the tuned numbers |

Until you write them, the program runs and sends **1500 on every stick** (neutral).

---

## 1. What the controller receives every loop

`compute_rc(errors, target_height_m, dt)` gets a `ControlErrors` object (`vision/target.py`). All values are
**Kalman-filtered** and expressed in the **drone's own frame**:

| Field | Meaning | Unit | Drives |
|---|---|---|---|
| `e_forward_m` | how far the target is **ahead** of the drone | m | pitch |
| `e_left_m` | how far the target is to the drone's **left** | m | roll |
| `target_height_m − z_world_m` | how far the drone must go **up** (computed in `compute_rc`) | m | throttle |
| `e_yaw_rad` | how much the drone must turn **left** (CCW seen from above) | rad | yaw |
| `v_forward_mps`, `v_left_mps`, `v_up_mps` | velocity of the drone | m/s | D-terms |
| `yaw_rate_radps` | turn rate, positive = turning left | rad/s | yaw D-term |
| `x_world_m, y_world_m, z_world_m, yaw_world_rad` | position in the world frame of `set_target.py` | m, rad | logging |
| `valid` | `False` when the marker was lost for longer than `tracking.max_lost_time_s` | - | safety (already handled) |

The error **rate** for the D-term is already passed in: `error_rate = −velocity`, because the target does not move.
This is cleaner than differentiating the error yourself. **[Verified]** In the synthetic test, a 0.20 m/s motion
gave −0.203 m/s with 0.007 m/s noise.

## 2. The controller of IIT Bombay (Paper 3), which you are reproducing

For each axis:

```
RC = 1500 + SIGN · ( Kp · e  +  Ki · ∫e dt  +  Kd · de/dt )        clamped to 1500 ± MAX_CORRECTION
```

**Why this works with tilt commands [Verified in the MagisV2 firmware source; check your firmware version]:**
- **roll/pitch RC → tilt angle set-point** (angle mode, limited to about ±20°). A tilt is a horizontal
  **acceleration**, so `Kp·e` alone makes the drone swing. The `Kd·de/dt = −Kd·v` term brakes it. It is the
  velocity loop. In the IITB paper, `Kp/Kd` sets how fast the error decays, like λ in IBVS.
- **throttle RC in altitude-hold → vertical velocity set-point:** `v_z ≈ (RC − 1500)/4 cm/s`, with a **deadband of
  about ±40** around 1500. 1500 holds the height. So a throttle correction smaller than 40 does nothing:
  either add the deadband (`1500 + sign(c)·40 + c`) or accept a small steady height error. Because throttle
  already commands a *velocity*, the height loop needs little or no D-term.
- **yaw RC → yaw rate.**
- The stock firmware has **no horizontal velocity command**. That is why the position loop has to be closed here.

## 3. Writing `PID.update()`

The four steps are listed in the TODO in `control/pid_controller.py`:

```
integral = clamp(integral + error·dt, ±integral_limit)        # anti-windup
output   = kp·error + ki·integral + kd·error_rate
output   = clamp(output, ±output_limit)
```

Optional improvements **[Recommendation]**:
- a **deadband**: ignore errors below about 1–2 cm, so the drone does not twitch around the target;
- **conditional integration**: do not add to the integral while the output is saturated;
- start with `Ki = 0`. Add a small `Ki` only if a steady offset remains (for example from wind or a battery that is not centred).

## 4. Writing the RC mapping in `compute_rc()`

```
rc["pitch"]    = int(clamp(NEUTRAL_RC + PITCH_SIGN    * corrections["pitch"],    RC_MIN, RC_MAX))
rc["roll"]     = int(clamp(NEUTRAL_RC + ROLL_SIGN     * corrections["roll"],     RC_MIN, RC_MAX))
rc["throttle"] = ...   (+ deadband handling, §2)
rc["yaw"]      = ...
```

### The signs

They come from **your** manual-control code (`Pluto.py`: `forward()` = pitch 1600, `left()` = roll 1400,
`increase_height()` = throttle 1800, `left_yaw()` = yaw 1300):

| Error positive means | Command | Sign in the code |
|---|---|---|
| target is **ahead** (`e_forward > 0`) | pitch **> 1500** | `PITCH_SIGN = +1` |
| target is to the **left** (`e_left > 0`) | roll **< 1500** | `ROLL_SIGN = −1` |
| target is **higher** (`e_up > 0`) | throttle **> 1500** | `THROTTLE_SIGN = +1` |
| must turn **left** (`e_yaw > 0`) | yaw **< 1500** | `YAW_SIGN = −1` |

**[Assumption]** These names (forward/left/left_yaw) match the drone's real behaviour. **Check them with the
propellers removed:**

1. Implement a P-only `update()` and set small gains, e.g. *illustrative* `PITCH_KP = 300` (30 RC units for a 10 cm error).
2. `python control\run_controller.py` (dry run). Hold the drone level above the camera, press `h`.
3. Move the drone **backwards** by hand → the target is now ahead → the window must show **pitch > 1500**.
   Move it **right** → **roll < 1500**. Lower it → **throttle > 1500**. Turn it right → target heading is to the left → **yaw < 1500**.
4. Then the same with `--fly`, **propellers off**, armed. In this test you only watch the commands being sent.

## 5. Choosing the first gains (reasoning, not tested values)

- **Kp:** decide how many RC units you want for a given error. *Illustrative:* 50 RC at 20 cm → `Kp = 250 RC/m`.
  Your manual scripts used ±100 RC (1400/1600) for clearly visible motion, so `MAX_*_CORRECTION = 100` is a sensible first limit.
- **Kd:** start so that the D-term equals the P-term when the drone moves at a moderate speed.
  *Illustrative:* at 0.2 m/s and Kp·e = 50 → `Kd ≈ 250 RC/(m/s)`. Increase Kd if it overshoots, decrease if it jitters.
- **Delay limit [Verified: classic result for a delayed integrator, used in the report]:** with a total delay τ
  (camera exposure + processing + Wi-Fi), the loop `ė = −λ·e(t−τ)` stays free of oscillation only for `λ·τ ≤ 1/e ≈ 0.37` (and becomes unstable at `λ·τ = π/2`).
  *Illustrative:* τ = 0.1 s → λ below about 3.7 1/s. **Measure your τ** (§7) before pushing the gains.

## 6. Tuning order and safety [Recommendation]

1. **Height first:** only the throttle PID active (others 0). Hand-launch or take off, press `h`.
2. **Yaw:** small gains.
3. **Pitch, then roll**, one at a time, with `MAX_*_CORRECTION` = 50 at first.
4. Every flight: `--plot`, or `python control\plotting.py`, then look at error, velocity and RC together.
   An oscillation with a constant period = too much Kp or too little Kd. A slow drift = Ki or a sign problem.
5. Safety:
   - **SPACE** stops the autonomous mode instantly (neutral sticks, altitude-hold keeps the height).
   - The marker lost for more than 1 s → automatic `land()` (`LOST_LAND_S`).
   - Fly over a soft surface. Keep the camera protected (a sheet of clear acrylic on spacers over the lens).
   - Fresh battery: the hover throttle changes with the battery voltage.

## 7. Measuring the delay τ (useful for the report)

Record (`r` in `aruco_pose_estimation.py`) while you give one sharp manual pitch command, then compare the time
of the command in `control_log.csv` (or your flight logger) with the moment the position starts to change.
Repeat 5 times and average.

## 8. Landing

`l` in `run_controller.py` lowers the target height by `DESCEND_RATE_MPS` (10 cm/s) while the horizontal PID keeps
the drone over the target. At `LAND_HEIGHT_M` (25 cm), the Pluto's own `land()` takes over. The reason: the view
gets small near the lens. With a ~70° horizontal field of view, the image is only about 2·0.20·tan 35° ≈ 28 cm wide at
20 cm height, so a small drift loses the marker. Near the ground, the propeller ground effect also changes the
dynamics. **[Assumption: check with your mounting and landing pad]**
