# 07 · The Pluto command interface: from IBVS velocities to RC values

> Sources: **[FW]** DronaAviation MagisV2 firmware, commit 5982032 (paths below are under `src/main/`). The older Magis
> (commit 2e55152) has the same RX-loss and altitude-hold logic. **[LIB]** your plutocontrol `Pluto.py`.
> **[P3]** IIT Bombay paper. **Which firmware is on your drone has not been checked**, so every number tagged [FW] must be
> confirmed by the tests in docs/08.

---

## 1. What the Pluto accepts

| Question | Answer | Source |
|---|---|---|
| Interface | MSP v1 over Wi-Fi TCP 192.168.4.1:23 (ESP bridge) | [LIB] `connect()`; [P3] Sec. 3.1 |
| Commands | `MSP_SET_RAW_RC` (200): 8 channels Roll, Pitch, Throttle, Yaw, AUX1-4; `MSP_SET_COMMAND` (217): 1 = take-off, 2 = land | [LIB]; [FW] `io/serial_msp.cpp` l.1428, l.1584 |
| Range / neutral | 1000-2000 µs, neutral 1500 | [P3] Sec. 3.1; [LIB] |
| Velocity command? | **No** for roll/pitch. A velocity mode exists in `flight/posControl.cpp` l.117-185, but it needs optical flow (`OPTIC_FLOW` is commented out in every `target/*/target.h` l.180) | [FW] |
| Roll / pitch | **tilt angle set-point** (angle mode): `errorAngle = constrain(2·rcCommand, ±max_angle) − angle`, max_angle = 200 = 20° | [FW] `flight/pid.cpp` l.252-255, `config/config.cpp` l.447 |
| RC → rcCommand | expo curve `lookupPitchRollRC` (rcRate 90, expo 65, but 130 in one target profile), no dead band (deadband = 0) | [FW] `io/rc_curves.cpp` l.36, `config.cpp` l.341-342, l.355, l.634 |
| Throttle (altitude hold) | **vertical velocity set-point** `v_z = (RC − 1500)/4 cm/s` if \|RC − 1500\| > 40, clamped −100..120 cm/s; inside ±40 the barometric altitude is held | [FW] `flight/altitudehold.cpp` l.231-238, `config.cpp` l.357 |
| Yaw | **yaw-rate** demand, `rc·(2·rate+30)>>5` in gyro units; scale depends on the rate profile | [FW] `flight/pid.cpp` l.422; **identify** |
| Your stick conventions | forward() pitch 1600, backward() 1400, left() roll 1400, right() 1600, left_yaw() 1300, right_yaw() 1600, increase_height() 1800, decrease_height() 1300 | [LIB] |
| Stabilisation | inner attitude PID on the drone (IMU), altitude-hold PID with baro + accelerometer | [FW]; [P3] Sec. 3.2 |
| Telemetry | MSP_ATTITUDE (roll/pitch 0.1°, heading °), MSP_ALTITUDE (cm, vario cm/s), MSP_ANALOG (battery mV), MSP_RC (echo); **no horizontal position/velocity** | [FW] `serial_msp.cpp` l.983-1025 |
| Command rate | plutocontrol re-sends the current RC every ~22 ms (~45 Hz) from its own thread | [LIB] `write_function()` |
| Commands stop arriving | no MSP RC for ~0.5 s (`needRxSignalBefore = now + 200 ms + 300 ms`) → RX lost → if armed: **firmware LAND**; then disarm | [FW] `rx/rx.cpp` l.368, `flight/failsafe.cpp` l.239-247 |
| …but | plutocontrol keeps re-sending the LAST values while the Python process lives. A frozen control loop does **not** trigger the failsafe; killing the process or losing Wi-Fi does | [LIB] |
| Take-off | `take_off()` = disarm, box_arm (throttle **1800**, AUX4 1500) + command 1. The firmware arms and sets the altitude target +120 cm; throttle 1800 in altitude hold = climb 75 cm/s until changed | [LIB]; [FW] `command/command.cpp` l.86-131 |
| Land | command 2: descends, disarms when the vertical speed > −8 cm/s after 2 s, on impact, or after 30 s; ignores new commands while landing | [FW] `command.cpp` l.132-182, `serial_msp.cpp` l.1586 |
| Armed with throttle ≤ 1500 after a stick arm | motors stay at idle until throttle > 1500 once | [FW] `altitudehold.cpp` l.248-252 |
| Headfree mode | exists (BOXHEADFREE); its AUX mapping is set by the app, so whether pitch/roll act in the body or a heading-fixed frame **must be checked** (test E4) | [FW] `API-Src/RxConfig.cpp` l.80-95 |

## 2. What IIT Bombay did ([P3]) and what it does not tell us

- Three **parallel PID controllers** on the position error in WhyCon coordinates → roll, pitch, throttle RC directly
  (Sec. 4.1, Fig. 3). Yaw is left to the drone. The default value is 1500; range 1000-2000 (Sec. 3.1).
- The sample time is chosen as `1/FPS + PID time + buffer` (Eq. 2).
- Gains from Ziegler-Nichols relay tuning: `Ku = 4d/(πa)` (Eq. 3) and Table 1 (Eq. 4-5).
- Measured total latency **341.33 ms** (Table 3).
- **Not given:** numerical gains, any velocity layer, any RC-to-velocity calibration, dead-band handling, sign conventions
  or failsafe behaviour. Those are filled in here from the firmware and must be measured.

## 3. The interface used here (and why it is [P3]'s structure)

[P1] expects a "high level control" that tracks (v_x, v_y, v_z, ω_z) ([P1] p.4). On the Pluto:

```
forward / left (tilt -> acceleration):   RC − 1500 = sign · ( Kv (v* − v̂) + Ki ∫(v* − v̂) dt )      clamp ±40 µs, slew 300 µs/s
up (altitude-hold velocity):              RC − 1500 = sign · 400 · v*   [+ dead-band compensation]    clamp ±60 µs, slew 400 µs/s
yaw (rate):                               RC − 1500 = sign · k_yaw · r*  (disabled until k_yaw is identified)
```

v̂ is the camera estimate (Kalman filter on [x_n, y_n, a_n] rotated into the body frame). The firmware closes the attitude
and vertical-velocity loops; this program closes the horizontal velocity loop.

**Equivalence with [P3].** With [P1]'s outer loop `v* = λe` (near s*), the horizontal law becomes
`RC − 1500 = Kv(λe − v̂) = (Kv·λ)·e + Kv·ė` (because ė = −v̂ for a fixed target). That is exactly [P3]'s PD with
`Kp = Kv·λ`, `Kd = Kv`; the Ki term plays the role of their integral. So the hardware layer *is* the IIT Bombay controller
structure, with [P1]'s IBVS supplying the set-point, unchanged.

## 4. The vertical dead band

Throttle inside 1500 ± 40 means "hold altitude". The smallest commandable climb or descent is 41 µs ⇒ (41/4 = 10) cm/s.
- `deadband_mode: none`: small IBVS corrections (|v*| < 0.10 m/s) are lost. The height error is corrected only when
  λ·e_z > 0.10 m/s, which can be 20+ cm.
- `deadband_mode: compensate` (default): if |v*| ≥ `min_effective_mps` (1 cm/s), send at least ±41 µs.
  The result is a small vertical limit cycle (10 cm/s × delay ≈ a few cm, visible as the saw-tooth in fig10 of the
  simulation) instead of a large steady error.
- Not implemented (possible future experiment): dithering between 0 and 41 µs. The firmware resets its altitude target and
  velocity integrator every time the stick leaves the dead band (`altitudehold.cpp` l.236-241), and the effect of doing that
  at 30 Hz is unknown.

## 5. Where the default gains come from (to be re-tuned after identification)

- **Tilt model** (assumption from the firmware curve, docs/03 of the report): RC 1600 ≈ 6.6° ⇒ 0.066°/µs ⇒
  `a ≈ g·tan(0.066°) = 0.0113 m/s²` per µs.
- **Inner loop.** `Kv = 120 µs/(m/s)` ⇒ crossover `ω_c = Kv·0.0113 ≈ 1.36 rad/s`. Phase margin
  `90° − ω_c·τ − atan(ω_c·τ_att)` is ≥ 40° for a total delay τ up to 0.5 s (τ_att ≈ 0.12 s). [P3] measured 0.34 s.
- **Ki = 50** ⇒ integral time Kv/Ki = 2.4 s > 3/ω_c. It removes trim/draft offsets: without it, a 0.05 m/s² drift needs
  4.4 µs, which with a P-only cascade costs about 4.4/(Kv·λ) ≈ 8–16 cm of steady error.
- **Outer loop** must be about 3× slower than the inner: λ_max,eff ≤ 0.45 ⇒ `lambda_scale = 0.45` on [P1]'s λ ∈ [0.5, 1.0].
  The classic delay rule for a delayed integrator, λτ < 1/e (no overshoot), gives λ < 1.08 at τ = 0.34 s, so [P1]'s
  λ = 1.0 would be at the limit even with an ideal inner loop.
- **Limits.** ±40 µs ≈ 2.6° ≈ 0.45 m/s². Speed 0.20 m/s horizontal, 0.15 m/s vertical: at 0.34 s delay that is ≤ 7 cm
  travelled before the camera sees it.

After the identification tests (§7), replace 0.0113 with the measured gain and τ with the measured delay, then recompute the
numbers above.

## 6. What "neutral" (1500) really means on the Pluto

Neutral = level attitude + hold barometric altitude + zero yaw rate. It is **not** a position hold: draft and trim keep
moving the drone (in [P3] Fig. 6 the drone drifts without the external controller). The barometer drifts by up to ~±50 cm over
minutes. So:
- HOLD (neutral) is a short-term state only. Marker lost for more than `marker_lost_land_s` ⇒ firmware LAND.
- After a stage timeout the default is to **continue** IBVS (`on_timeout: continue`), not to HOLD.

## 7. Identifying the real mapping (what must be measured before flying IBVS)

All of these use the program itself in `--mode live`, flying MANUALLY with the keys while everything is logged.
For identification, set the `manual:` values in the config to small pulses (for example pitch 1540/1460, roll 1460/1540).
Your normal ±100 µs moves the drone about 0.5 m in a second. Exact procedure: docs/08 Part E.

| Quantity | Experiment | Tool |
|---|---|---|
| Sign of each axis and the frame (body / headfree) | manual pulses W/S/A/D with the nose along +x_C, then with the nose rotated 90° | `identify_rc_response.py` (sign, k) |
| Tilt gain k (m/s² per µs), asymmetry (+/−), drag, trim bias | 10-20 manual pulses per direction, 0.5 s each | same, k(+), k(−), bias |
| Delay τ | the same pulses (best-fit delay) + `rc_echo_latency_s` (Wi-Fi round trip) + frame age | same + summary |
| Vertical gain and dead band | Q/E pulses with `manual.throttle_up/down` set to small steps (e.g. 1520, 1560, 1440) in live mode | same: compare with 400 µs/(m/s), ±40 |
| Yaw rate scale | Z/X pulses | same ⇒ `yaw.us_per_radps`, then `yaw.enabled: true` |
| Noise floor | marker still, 60 s | `feature_noise.py` |
| Neutral behaviour | hover, no keys, 20 s: drift speed and direction | `plot_ibvs_run.py` (est_v_* with rc_source = neutral) |

Saturation, motion coupling (pitch pulse causing a left motion = misaligned marker or headfree), non-linearity (k changes
with amplitude) and direction asymmetry all show up in these fits. Repeat each test 3 times and use the median.
