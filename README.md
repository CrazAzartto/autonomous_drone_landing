# Autonomous Drone Landing: image-moment IBVS on a Pluto nano drone

BTP project. A **Logitech C920 on the floor, looking up**, watches an **ArUco marker under a Pluto drone**. The drone is
brought to a desired image configuration and landed with the **image-based visual servoing (IBVS) of Paper 1** (Hegazy,
Boby, Klimchik: image moments, interaction matrix, adaptive gain, two-stage algorithm). The Pluto-specific command layer
follows the **IIT Bombay** controller (Paper 3: RC = 1500 + PID on the error) and the Pluto firmware.

| Part | Status |
|---|---|
| Stage 1: camera on Windows, calibration, ArUco pose (`calibration/`, `detection/`, `common/`) | done, used on your C920 (RMS 0.43 px) |
| Paper 1 features, interaction matrix, control law, adaptive gain, stages, learning step (`ibvs/`) | done; Eq.(10) **verified** against finite differences |
| Eye-to-hand adaptation (camera on the floor) and drone body frame (`ibvs/frames.py`) | done; derivation in docs/06 §4 |
| Pluto interface: velocity → RC, firmware maps, MSP telemetry, dry-run / live / sim adapters (`pluto_interface/`) | done; **signs and gains must be verified on your drone** (docs/08) |
| Safety supervisor, independent watchdog, live-control gate, operator take-over (`safety/`, `control/ibvs_run.py`) | done; tested in software only |
| Logging (145 columns/cycle), publication plots, RC identification, noise floor (`runlog/`, `analysis/`) | done |
| SYNTHETIC simulator + 42 unit/integration tests (`sim/`, `tests/`) | 42 passed (Linux, OpenCV 4.10; Windows, OpenCV 4.14) |
| **Hardware validation** (bench, identification flights, IBVS flights) | **not done yet**: follow docs/08 in order |

> Nothing here has flown. No result in this repository is a flight result. Simulator outputs are labelled SYNTHETIC
> everywhere.

## Documents
| | |
|---|---|
| [docs/04_windows_step_by_step.md](docs/04_windows_step_by_step.md) | Stage 1: printing, calibration, pose (click by click) |
| [docs/06_ibvs_architecture.md](docs/06_ibvs_architecture.md) | **the method**: data flow, Paper 1 ↔ code, frames, eye-to-hand proof, worked example, modifications |
| [docs/07_pluto_command_interface.md](docs/07_pluto_command_interface.md) | what the Pluto accepts (firmware citations), Paper 3, velocity → RC, gain derivation, identification |
| [docs/08_safety_and_testing.md](docs/08_safety_and_testing.md) | **hazard analysis and the staged test plan (read before any live run)** |
| [docs/09_running_ibvs.md](docs/09_running_ibvs.md) | commands, modes, keys, plots, git |
| docs/01-03, 05 | calibration theory, C920 settings, pose theory, the earlier position-PID template (baseline) |

## Quick start (PowerShell)
```powershell
cd "D:\Study Materials\BTP\codebase\autonomous_drone_landing"
..\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python -m pytest -q tests                                   # 42 passed
python control\ibvs_run.py --mode sim                       # SYNTHETIC closed loop with the operator window
python control\ibvs_run.py --mode camera                    # your C920: features + PnP, nothing sent
python control\ibvs_run.py --mode compute                   # full controller, nothing sent (press I, move the drone by hand)
python analysis\plot_ibvs_run.py results\runs\<run folder>  # figures + convergence summary
```
`--mode live` (and `--mode live --bench` with propellers removed) works only after you set
`safety.live_control_enabled: true` and type a confirmation word. Do this after docs/08 stages A-D.

## Repository layout
```
config/camera_config.yaml      camera, calibration, marker (Stage 1)
config/ibvs_config.yaml        IBVS stages, gains, Pluto interface, safety, logging, simulator (every value tagged with its basis)
common/                        camera opening (DirectShow), ArUco + solvePnP, calibration I/O          (Stage 1)
calibration/  detection/       Stage-1 programs (capture, calibrate, camera info, live pose)
vision/                        Kalman filter, world-frame tools, marker tracker                           (Stage 1, reused)
ibvs/  moments.py features.py interaction.py controller.py frames.py sources.py runtime.py
pluto_interface/  command_mapper.py msp_parser.py adapters.py
safety/  supervisor.py watchdog.py
runlog/  run_logger.py columns.py
sim/  pluto_sim.py             SYNTHETIC plant + camera
control/ibvs_run.py            PROGRAM 6: the IBVS program (all modes)
control/run_controller.py      earlier position-PID template (Paper 3-style baseline, still works)
analysis/  plot_ibvs_run.py identify_rc_response.py suggest_gains.py feature_noise.py
tools/                         print sheets, set_target (for the PID baseline)
tests/                         pytest suite
results/                       calibration, runs, plots (git-ignored)
```

## How one control cycle works (details: docs/06)
```
C920 frame -> ArUco corners -> moments -> s=[x_n,y_n,a_n,alpha] -> e=s-s* -> L_s (Eq.10), L_hat+ = (L(s)+ + L(s*)+)/2
 -> v_c = -lambda L_hat+ e (Eq.1, adaptive lambda) -> u_m = L_s v_c (eye-to-hand) -> body (fwd,left,up,yaw) via psi=alpha-90deg
 -> limits -> velocity PI with camera-estimated velocity (pitch/roll), firmware velocity map (throttle), rate map (yaw)
 -> safety supervisor -> RC -> plutocontrol (MSP over Wi-Fi) ;  stage 1 -> stage 2 -> firmware LAND   (Algorithm 1)
```

## Verified vs assumed (summary)
- **Verified in code/tests:** Paper 1 feature, interaction-matrix and control-law equations; the eye-to-hand conversion;
  the sign chain (drone behind / right / low / turned right ⇒ pitch > 1500 / roll < 1500 / throttle > 1500 / yaw < 1500,
  consistent with your plutocontrol stick functions); safety and watchdog logic; MSP telemetry parsing; identification on
  synthetic data.
- **From firmware source (your firmware version not checked):** roll/pitch = tilt, throttle = vertical velocity
  ((RC−1500)/4 cm/s, ±40 dead band), yaw = rate, RX-loss ⇒ LAND.
- **Assumptions to measure (docs/08):** tilt gain, delay, yaw scale, headfree off, camera level, noise floor, battery threshold.
