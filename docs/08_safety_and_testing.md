# 08 · Safety, hazard analysis and the staged test plan

> Nothing in this repository has been tested on the real Pluto. Everything marked [Verified] in docs/06-07 was verified in
> code, firmware source, unit tests or the SYNTHETIC simulator. Do the stages below **in order**. Do not skip one because the
> previous one "looked fine".

---

## 1. Built-in protections (what the software does)

| Protection | Implementation |
|---|---|
| Never arms or takes off at start | arm = key Y, take-off = key T, IBVS = key I. All are operator actions |
| Live control disabled by default | `safety.live_control_enabled: false`; `--mode live`; typing `FLY` (or `BENCH`) at a prompt. All three are required |
| Camera-only / compute-only / replay / simulation modes | `--mode camera / compute / replay / sim`: they use `DryRunAdapter` (no network code is even imported) |
| Command limits | body velocity limits; RC ±40 µs (tilt), ±60 µs (throttle), ±50 µs (yaw), clamped to 1000-2000 |
| Slew-rate limits | 300 µs/s (roll, pitch, yaw), 400 µs/s (throttle) |
| Stale frames | frame older than 0.15 s ⇒ HOLD that cycle |
| Marker loss | immediate HOLD (neutral), firmware LAND after 1.0 s |
| Invalid numbers | NaN/inf features or commands ⇒ neutral; invalid RC never reaches the drone (`PlutoAdapter.send_rc`) |
| Envelope | too high ⇒ HOLD; too low (< 0.20 m above the lens) ⇒ LAND; outside the 0.6 m fence ⇒ HOLD; marker near the image border ⇒ warning |
| Drone link | plutocontrol writer thread dead / disconnected ⇒ LAND requested (the firmware failsafe lands anyway after ~0.5-1.5 s) |
| Battery, tilt | from telemetry, when fresh: battery < 3.45 V ⇒ LAND; tilt > 25° ⇒ HOLD |
| Frozen control loop | watchdog thread: no `kick()` for 0.5 s ⇒ neutral + LAND, independent of the loop |
| Program exit / crash | `finally`: neutral + LAND, wait up to 8 s, disarm, disconnect. If the process is killed, the socket closes and the **firmware failsafe** lands the drone |
| Operator take-over | any manual key (W/S/A/D/Q/E/Z/X) immediately leaves IBVS; your manual-script RC values |
| Emergency stop | SPACE = neutral + firmware LAND; **K = disarm now** (motors stop) |
| Independent of this program | (1) firmware RX-loss failsafe when the RC stream stops (close the laptop lid / turn Wi-Fi off / end the process); (2) physical: net, tether, hand catch, battery unplug. **There is no second radio transmitter**, so these are the only paths that do not depend on this program. Test (1) on the bench (D5) |

## 2. Hazard analysis

| # | Failure | Consequence | Detection | Mitigation |
|---|---|---|---|---|
| H1 | wrong sign on an axis | drone accelerates AWAY from the target | compute mode: the arrow / RC disagree with the table in B3; flight: error grows | bench sign tests B3/D3, identification E2 (sign printed), small limits on the first flight, K / SPACE |
| H2 | pitch/roll act in a heading-fixed frame (headfree) | wrong direction after yawing | E4 | arm with the nose along the chosen forward direction; yaw disabled; E4 before enabling yaw |
| H3 | marker glued rotated / mirrored | forward ≠ nose: coupled, wrong motion | B2 heading check; E2 coupling | glue with the arrow to the nose; B2 |
| H4 | camera not level / moved after setup | biased horizontal error, coupling with height | C2; sudden constant offset | fix the camera; re-teach |
| H5 | latency larger than assumed | oscillation that grows | fig03/fig07 oscillation, identification delay | measure (E2), lower λ_scale and Kv; small limits |
| H6 | control loop freezes (camera driver, exception) | plutocontrol keeps sending the last command | watchdog | watchdog ⇒ neutral + LAND (test D4) |
| H7 | Wi-Fi drops | no commands | firmware RX loss; `link_ok` | firmware LAND (test D5) |
| H8 | marker lost (out of view, glare, blur) | no feedback | `detected` | HOLD then LAND after 1 s; fence; lower lights / exposure |
| H9 | false detection / wrong marker ID | wrong command | reprojection error, ID filter | DICT_4X4_50 ID filter; PnP reprojection logged; NaN guard |
| H10 | neutral ≠ hover (drift) | drone drifts during HOLD | est_v in HOLD | HOLD is short; `on_timeout: continue`; LAND on loss |
| H11 | take-off throttle 1800 left on | climbs at 75 cm/s | Z rising after T | neutral after `takeoff_neutral_after_s`; operator watches; max height ⇒ HOLD |
| H12 | baro drift | slow height change | e_an | IBVS corrects height from the image |
| H13 | low battery | sudden loss of thrust | telemetry vbat | LAND below 3.45 V (verify telemetry arrives: D1) |
| H14 | firmware LAND ignores new commands | cannot abort a landing | — | expected; K still disarms |
| H15 | lens hit by a landing drone | camera damage / blind | — | clear acrylic sheet on spacers over the camera; min height LAND at 0.20 m |
| H16 | person near the drone | injury (propellers) | — | net or cage, one operator + one observer, eye protection, small room-scale limits |

## 3. Staged test plan

Every run writes a log folder. Keep all of them; they are your report data. **Pass** means the listed condition is met.

### Stage A: offline (no hardware)

| ID | Do | Pass |
|---|---|---|
| A1 | `python -m pytest -q tests` | 42 passed |
| A2 | `python control/ibvs_run.py --mode sim --headless`, then `python analysis/plot_ibvs_run.py <run>` | stage 1 converges; plots labelled SYNTHETIC |
| A3 | record a video (key R in camera mode), then `--mode replay --video <file> --headless --auto-engage` | runs to the end; plots labelled REPLAY |

### Stage B: camera only (no drone link)

| ID | Do | Pass |
|---|---|---|
| B1 | `--mode camera`, marker flat at a measured height | `an` = PnP Z within 2 %; `xn, yn` = PnP X, Y |
| B2 | marker glued under the drone; hold the drone level, nose towards a wall | `a` and the heading follow when you rotate it; turning the nose LEFT (CCW seen from above) increases α |
| B3 | `--mode compute`; press P at the target pose (teach) and then I; move the drone **by hand** | drone 20 cm BEHIND its target ⇒ `cmd fwd > 0` and **RC p > 1500**; 20 cm to its RIGHT ⇒ `cmd left > 0` and **RC r < 1500**; too LOW ⇒ **RC t > 1500**; the red arrow points back to the orange square. Check with the nose in 4 directions |

### Stage C: camera installation

| ID | Do | Pass |
|---|---|---|
| C1 | camera taped to the floor, protective sheet if used | does not move when touched |
| C2 | level check: marker flat on a glass/acrylic shelf above the camera, the shelf levelled with a spirit level; `--mode camera` shows `tilt` (PnP) | tilt ≤ 2° (it then equals the camera's tilt); otherwise shim the camera |
| C3 | marker still for 60 s (camera mode) → `python analysis/feature_noise.py <run>` | note the noise; set the stage thresholds ≥ the recommended value |

### Stage D: bench, PROPELLERS REMOVED (`--mode live --bench`, `live_control_enabled: true`)

| ID | Do | Pass |
|---|---|---|
| D1 | connect, wait | `link OK`; telemetry vbat/roll/pitch shown (otherwise battery/tilt checks are inactive) |
| D2 | Y (arm), I (engage), move the drone by hand as in B3 | RC sent = the same signs as B3; `echo` latency shown in ms (record it) |
| D3 | watch the motors while moving the drone back/forward | the motor speeds change accordingly (e.g. back motors faster for pitch forward). Confirms the drone executes the signs |
| D4 | key F (freeze 1 s) | event `watchdog: ... LAND` within ~0.5 s |
| D5 | armed, close the program window **forcefully** (end the process in Task Manager) or switch the laptop Wi-Fi off | motors stop within ~2 s (firmware failsafe LAND → disarm). Record the time |
| D6 | SPACE; then reconnect and test K | SPACE ⇒ LAND command; K ⇒ disarmed immediately |

### Stage E: manual identification flights (propellers ON; net/clear area; 30-60 cm height; your manual keys)

Set small pulse values in `manual:` for E2/E3 (docs/07 §7). Every flight: `--mode live`, Y, T, then keys only, never I.

| ID | Do | Pass / output |
|---|---|---|
| E1 | hover 20 s without keys | drift speed and direction (est_v, plot fig07) |
| E2 | 10-20 short pitch and roll pulses, nose along +x of the camera image | `identify_rc_response.py`: sign_pitch = +1, sign_roll = −1, R² > 0.5; note k, delay, asymmetry |
| E3 | yaw pulses Z/X | `us_per_radps`, sign_yaw |
| E4 | rotate the nose 90°, repeat a few pitch pulses | motion along the NEW nose direction (body frame). If along the old direction ⇒ headfree is on: switch it off in the app or arm with the nose fixed and keep yaw disabled |

Update the config: `python analysis/suggest_gains.py <E2 run 1> <run 2> <run 3>` prints the signs, `kv`, `ki`,
`lambda_scale`, throttle and yaw values computed from YOUR measurements (docs/07 §5). Paste them into the config. Re-run the simulator with
the measured `tilt_deg_per_us` and `delay_s` to check the new gains.

### Stage F: first IBVS flights (propellers on, net, observer, hand on K)

| ID | Do | Pass |
|---|---|---|
| F1 | hover manually over the target, `cmd`/arrow shown but **not engaged** | arrow points to the target as you drift |
| F2 | `max_correction_us: 20`, `max_horizontal_mps: 0.10`; engage I for 10 s, then H | error shrinks or stays; no growing oscillation |
| F3 | normal limits, stage 1 only (delete stage 2 in a copy of the config, `final_action: hold`) | ‖e‖ decreases; record the convergence |
| F4 | full sequence with final LAND | lands; 5 repetitions for the report |

### Stage G: report data

For each configuration, run 5 times and keep the log folders. Run `plot_ibvs_run.py` on each and tabulate from `summary.json`
the convergence time, final residual, RMS of the last 2 s and the saturation fraction. Compare with [P1] Fig. 2 (≈8 s
coarse, ≈6 s fine in Gazebo) and say clearly which parts differ and why (docs/06 §6-7).
