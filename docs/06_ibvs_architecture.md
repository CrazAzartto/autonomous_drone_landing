# 06 · IBVS architecture: Paper 1 on the Pluto with a ground camera

> **References.** **[P1]** M. Hegazy, R. A. Boby, A. Klimchik, *Localization of a Drone for Landing using Image
> Based Visual Servoing with Image Moments*. **[P3]** S. Singh et al., *Vision-based indoor localization of nano
> drones in controlled environment with its applications*, arXiv:2412.08757 (IIT Bombay). **[FW]** DronaAviation
> MagisV2 firmware, commit 5982032. **[LIB]** your `pluto_course/plutocontrol/plutocontrol/Pluto.py`.
>
> **Tags.** **[Verified]** = checked in code, a source or a test. **[Modification]** = a change from [P1]
> required by the hardware, with its justification. **[Assumption]** = must be measured on your hardware (docs/08).

---

## 1. The feedback loop, step by step

```
  C920 (floor, looking up)                                              PLUTO
        |  image (30 fps, MJPG 1280x720)                                  ^
        v                                                                 |  MSP_SET_RAW_RC over Wi-Fi
 [1] ibvs/sources.py          capture + ArUco corners (common/aruco_utils, Stage 1)  |  (~45 Hz, plutocontrol thread)
 [2]                          4 corners of the stage marker (pixels)                 |
 [3] ibvs/features.py         undistort -> moments m_ij, mu_ij -> s = [x_n,y_n,a_n,alpha]      [P1] Eq.2, p.4
 [4]                          desired s* = [z* x_g*, z* y_g*, z*, alpha*] (model or taught)     [P1] p.5
 [5][6]                       e = s - s*  (alpha wrapped)                                       [P1] p.5
 [7] ibvs/interaction.py      L_s(s) (4x4), L_s(s*)                                              [P1] Eq.10
 [8]                          L_hat^+ = (L_s(s)^+ + L_s(s*)^+)/2, adaptive lambda                [P1] p.5
 [9] ibvs/controller.py       v_c = -lambda L_hat^+ e     (camera twist of the PAPER's model)    [P1] Eq.1
 [10] ibvs/frames.py          u_m = L_s(s) v_c (eye-to-hand)  ->  body frame via psi = alpha-90  [Modification]
      ibvs/runtime.py         limits on the body velocity set-point
 [11] pluto_interface/command_mapper.py   velocity PI loop (forward, left) with the CAMERA velocity
                              estimate; firmware velocity map (up); rate map (yaw)               [P3]-structure, [FW]
      safety/supervisor.py    may replace the command by neutral / LAND
 [12] pluto_interface/adapters.py  sets rcRoll/rcPitch/rcThrottle/rcYaw in plutocontrol  ----------+
 [13] next frame;  [14] until ||e|| < threshold (stage 1 -> stage 2 -> final action)             [P1] Alg. 1
      runlog/                 one CSV row per cycle + events
```

The program `control/ibvs_run.py` only does the operator window, keys and modes. All the computation in the box above is in
`ibvs/runtime.py → ControlPipeline.step()`. The same code runs in every mode: camera, compute, replay, sim, live and bench.

## 2. Paper 1 → code

| [P1] | Content | Where | Faithful? |
|---|---|---|---|
| Eq.(1), p.3 | `v_c = −λ L̂⁺ (s − s*)` | `controller.py: compute()` | yes |
| Eq.(2), p.3 | discrete moments m_ij, μ_ij of n points | `moments.py` | yes (n = 4 corners) |
| p.4 | `a = μ20 + μ02`, `a_n = z*√(a*/a)`, `x_n = a_n x_g`, `y_n = a_n y_g` | `features.py` | yes |
| p.4 | `α = atan2(y2 − y1, x2 − x1)` | `features.py: orientation()` | yes; ambiguity documented (§3.3) |
| Eq.(7)-(10), p.4-5 | 4×4 interaction matrix for (v_x, v_y, v_z, ω_z) | `interaction.py` | yes. **Verified**: equals the finite-difference derivative (`tests/`) |
| p.5 | `L̂⁺ = ½ (L⁺(s) + L⁺(s*))` | `interaction_estimate_pinv()` | yes ("current" and "desired" also available for comparison) |
| p.5 | error `e = [x_n − z*x_g*, y_n − z*y_g*, a_n − z*, α − α*]` | `features.feature_error()` | yes |
| p.5 | adaptive `λ = (λmax − λmin)‖e‖/‖e_max‖ + λmin` | `AdaptiveGain` | yes; scaled by `lambda_scale` [Modification §6] |
| p.6 | λmax = 1.0, λmin = 0.5 | config | yes (values kept; scale separate) |
| p.5 | thresholds 0.1 / 0.015 | config | kept as defaults; see §7 |
| Alg. 1, p.7 | stage 1 → stage 2 → turn off motors | `controller.advance_stage()`, `runtime._final_action()` | yes; final action default = firmware LAND [Modification] |
| Sec. 4 | "learning step" of z*, image moments | key **P** → `teach_desired()` | yes |

Notes on [P1] itself:
- Algorithm 1 cites equation numbers (3), (4), (16), (18), (26), (27) that do not exist in the 10-equation paper. They come from a
  longer version. The mapping above follows the meaning of each step.
- The printed Eq.(5) has a typo (`y_g ω_x` twice). It is not used: Eq.(7)/(10) is the form used.

## 3. Features: what they mean physically

### 3.1 Normalised features equal the metric position

Take the marker, a square of side L, parallel to the image plane at depth Z, centre (X, Y). Its corners in normalised
coordinates are `(X ± L/2, Y ± L/2)/Z`. Then:
- `x_g = X/Z` and `y_g = Y/Z`;
- `μ20 = μ02 = 4·(L/2Z)² = L²/Z²`, so `a = 2L²/Z²`;
- with `a* = 2L²/z*²`: `a_n = z*·√(a*/a) = z*·(Z/z*) = Z`, then `x_n = a_n x_g = X` and `y_n = Y`.

So **s = [X, Y, Z, α]** in metres in the camera frame (the IIT Bombay controller works on the same quantities, see docs/07).
**[Verified]** in `tests/test_features_and_interaction.py` to 1e-12 for arbitrary poses. If the drone tilts by θ, `a` shrinks by
about cos θ (1.5 % at 10°). This is a small, second-order effect.

### 3.2 Desired features and the learning step

`s* = [z*x_g*, z*y_g*, z*, α*]`.
- **Model** (`desired_source: model`): a* is computed from L and z*. Correct only if the size and the calibration are correct.
- **Taught** (key **P**, [P1]'s learning step): hold the drone at the desired pose; 30 frames are averaged. z* is taken from
  solvePnP. This makes the controller converge to that exact *image*, so calibration errors cancel. That is the robustness
  argument of IBVS ([P1] Sec. 1).

### 3.3 Orientation α

[P1]'s text says "first and third corner"; the printed formula uses corners 1→2. The default is `edge_0_1` (the formula);
`diagonal_0_2` (the text) is available. The two differ by exactly 45° (**[Verified]** test), and the offset cancels in α − α*.

### 3.4 Numerical example (computed by the code, values rounded)

Marker L = 40 mm at X = +0.10, Y = −0.05, Z = 0.90 m, α = 30°. Stage 1: z* = 0.80 m, x_g* = y_g* = 0, α* = 30°,
λ_max = 1.0, λ_min = 0.5, lambda_scale = 0.45.

| Quantity | Value |
|---|---|
| normalised corners | (0.1030, −0.0859), (0.1415, −0.0637), (0.1192, −0.0252), (0.0808, −0.0474) |
| x_g, y_g | 0.1111, −0.0556 (= X/Z, Y/Z) |
| μ20, μ02, μ11 | 0.001975, 0.001975, 0 |
| a, a* | 0.003951, 0.005000 (= 2·0.04²/0.8²) |
| a_n, x_n, y_n | 0.900, 0.100, −0.050 m |
| e | [0.100, −0.050, 0.100, 0], ‖e‖ = 0.150 |
| λ (first cycle: ‖e‖ = ‖e_max‖) | 0.45 · 1.0 = 0.45 |
| L(s) | rows [−1 0 0 −0.05], [0 −1 0 −0.10], [0 0 −1 0], [0 0 0 −1] |
| L̂⁺ | rows [−1 0 0 0.025], [0 −1 0 0.05], [0 0 −1 0], [0 0 0 −1] |
| v_c = −λL̂⁺e | [+0.045, −0.0225, +0.045, 0] (camera of the paper should move *towards* the marker offset) |
| u_m = L(s)v_c | [−0.045, +0.0225, −0.045, 0] m/s: the **drone** must move −X, +Y, down |
| heading ψ = α − 90° | −60° |
| body set-point | forward −0.042, left −0.028, up −0.045 m/s |
| RC (estimate 0, no slew) | roll 1503 (right), pitch 1495 (back), throttle 1459 (down; dead-band compensated), yaw 1500 |

Check by hand: forward unit vector (cos −60°, sin −60°) = (0.5, −0.866), so `v_fwd = 0.5·(−0.045) − 0.866·0.0225 = −0.042`. ✓

## 4. Eye-in-hand (paper) vs eye-to-hand (your setup)

[P1] Eq.(1) assumes the camera is ON the drone and the marker is static. Here the camera is static and the marker moves.
Writing u_m = [v_mx, v_my, v_mz, ω_m] for the marker twist (velocity of its centre and yaw rate about its own vertical axis,
in the camera frame), §3.1 gives directly `ṡ = u_m` (identity matrix). Requiring the **same feature motion as the paper's law**:

```
ṡ = L_s v_c = −λ L_s L̂⁺ e        =>        u_m = L_s(s) v_c
```

This is the same as the rigid-body relation between "camera moves" and "object moves", `v_m = −v_c + ω_m × P` with
`ω_m = −ω_cz`. **[Verified]** in a test. Consequences:
- The closed-loop error dynamics `ė = −λ L_s L̂⁺ e` are **identical** to [P1], so the decoupling, the gain schedule and the
  convergence properties carry over. The only change is the actuator.
- Near s*, `L_s L̂⁺ ≈ I`, so `ė ≈ −λe`. **[Verified]**: the ideal loop (both forms) decays monotonically at rate λ.
- When `L̂⁺ = L(s)⁻¹`, u_m translation = −λ·e_translation, independent of e_α. So disabling yaw does not disturb the
  translation (§6).

**Body frame.** Marker under the drone, printed face down, top edge = nose: x_B = y_M, y_B = x_M, z_B = −z_M. In the camera
x-y plane (counter-clockwise when seen from above, because z_C points up), y_M is x_M rotated by −90°, so the nose direction is
`ψ = α − 90°`. The body velocity is then `forward = cosψ·v_x + sinψ·v_y`, `left = −sinψ·v_x + cosψ·v_y`, `up = v_z`,
`yaw-left rate = ω_m`. The controller therefore needs **image features only**; solvePnP is logged for validation (fig04/05).

## 5. Six quantities that are NOT interchangeable

| Symbol / column | What it is | Frame | Source |
|---|---|---|---|
| `v_c` (vc_*) | camera twist of [P1] Eq.(1). A *virtual* quantity here (the camera does not move) | camera | controller |
| `u_m` (um_*) | drone/marker twist that produces the same ṡ | camera | `L_s v_c` |
| body set-point (cmd_v_*) | desired drone velocity after limits | body FLU | frames + limits |
| RC command (rc_*) | stick values 1000-2000 µs. Roll/pitch = **tilt**, throttle = **vertical velocity**, yaw = **rate** | stick axes | command_mapper |
| actual velocity | what the drone really does; **not measured directly** (only the simulator knows it: true_v*) | – | – |
| estimated velocity (est_v_*) | Kalman filter on [x_n, y_n, a_n] from the camera (latency included) | camera → body | velocity_estimation |

The Pluto's own vertical speed estimate (`tel_vario_cmps`, barometer + accelerometer) is logged when telemetry arrives. It is
not used, because the barometer drifts (±50 cm, docs/07 §6).

## 6. Modifications to [P1] required by the hardware

| # | Modification | Why | Turn it off / reproduce [P1] |
|---|---|---|---|
| M1 | eye-to-hand conversion `u_m = L_s v_c` | camera on the floor, marker on the drone | not applicable (geometry) |
| M2 | velocity PI loop for forward/left using the camera velocity estimate | Pluto roll/pitch = tilt, no velocity mode (docs/07 §2) | not applicable (actuator) |
| M3 | `lambda_scale` = 0.45 | [P1]'s λ assumed a fast PX4 velocity controller; Pluto + Wi-Fi + camera delay ≈ 0.2–0.35 s ([P3] Table 3: 341 ms) | `lambda_scale: 1.0` |
| M4 | limits, slew rate, safety | real hardware | (safety limits stay) |
| M5 | dwell time for the stopping test | single noisy frames | `dwell_s: 0` |
| M6 | α excluded from the **stopping** test while yaw is not actuated | an uncontrolled DOF cannot converge | enable yaw (after identification) |
| M7 | final action = firmware LAND instead of "turn off motors" | dropping the drone damages it | `final_action: disarm` |
| M8 | one marker for both stages | the marker is on the drone and grows in the image as the drone descends; [P1] needed a second, smaller marker because its camera approached a ground marker | set another `marker_id` for stage 2 if you add a marker |
| M9 | vertical dead-band compensation | the firmware ignores |RC − 1500| ≤ 40 (docs/07 §4) | `deadband_mode: none` |

## 7. Convergence of the image ≠ a safe landing

- ‖e‖ < threshold means *the image* matches s*. It says nothing about obstacles, the landing surface, the battery or whether the
  drone is level. The final LAND is done by the firmware, which lands where it is.
- The thresholds 0.1 / 0.015 of [P1] were obtained in **simulation** (Gazebo). On hardware the residual is limited by:
  (a) camera noise (measure: `analysis/feature_noise.py`);
  (b) the firmware's smallest vertical speed of 10 cm/s (vertical limit cycle of a few cm);
  (c) draft and trim drift against a latency-limited loop.
  In the **SYNTHETIC** simulation (plant assumptions in config), stage 1 (0.1) converged in 7.7–9.8 s in 3/3 seeds. Stage 2
  stayed at ‖e‖ ≈ 0.02–0.13 and never reached 0.015 in 40 s. With a stage-2 threshold of 0.06 (5 seeds), both stages
  converged every time: stage 1 in 3.5–10.7 s, stage 2 in 3.5–12.8 s. The drone touched down 6–11 cm from the camera axis,
  because the firmware's own descent from 0.55 m has no horizontal control (`tests/test_sim_closed_loop.py`).
  These numbers describe the simulator, **not** the Pluto.
- Recommendation: keep [P1]'s values for the report's "paper setting". For flights, set each threshold to at least
  3× the measured ‖e‖ noise in hover, and report both.

## 8. The SYNTHETIC simulator (`sim/pluto_sim.py`)

A point-mass model in the camera frame:
- RC is applied after a transport delay.
- Tilt = gain·(RC − 1500), with a first-order attitude lag; acceleration = g·tan(tilt), plus linear drag, constant draft and
  Ornstein-Uhlenbeck gusts.
- Vertical axis: the exact firmware altitude-hold map (dead band, 10 cm/s steps), plus barometer drift.
- Yaw rate ∝ RC.
- Camera: your calibration, pixel noise, dropouts and field-of-view loss.

There is no Pluto SITL in the DronaAviation repositories, so the firmware itself is not simulated. Every parameter is an
**[Assumption]** (config `simulation:`). The simulator checks signs, frames, logic and failure handling. It is not evidence of
flight performance.

## 9. Dimensions (checked in code and tests)

`corners (4,2)` → `points_norm (4,2)` → `s, s*, e (4,)` → `L, L(s*), L̂⁺ (4,4)` → `v_c (4,)` → `u_m = L v_c (4,)` →
`body (4,)` → `RC (4 ints)`. Also `det L_s = 1` for every s, so the 4×4 matrix is always invertible. Its condition number is
logged (`L_condition`) and checked (safety `max_condition`).

## 10. Limitations

- Monocular depth comes from the marker area. a_n noise grows with Z²; heavy tilt biases a_n slightly.
- The workspace is the camera's view: vertical half-view ≈ 0.37·Z for the C920 at 16:9. Near the camera the drone can leave
  the view with a small drift. The fence and the marker-loss LAND exist for that reason.
- The horizontal loop is latency-limited. Measure the delay before raising gains.
- The yaw scale is unknown until identified; yaw control is off by default.
