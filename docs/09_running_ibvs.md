# 09 · Running the IBVS controller (Windows)

Before this: Stage 1 must be finished, i.e. calibration done (`results\camera_calibration.npz`) and the marker size measured
(docs/04). The theory is in docs/06, the Pluto interface in docs/07, and the safety rules and the test order in docs/08.

---

## 1. Setup

```powershell
cd "D:\Study Materials\BTP\codebase\autonomous_drone_landing"
..\.venv\Scripts\Activate.ps1                      # prompt shows (.venv) - the Python 3.12 environment
python -m pip install -r requirements.txt           # adds pytest (pygame etc. are already installed)
python -m pytest -q tests                           # expect: 42 passed
```
Note: there is also an **empty** `autonomous_drone_landing\.venv` (Python 3.15, no packages) inside the repository folder.
Do not select it in VS Code. Use `..\.venv` (codebase\.venv). You can delete the inner one.

## 2. The modes, in the order of docs/08

| Command | What happens | Sends to the drone? |
|---|---|---|
| `python control\ibvs_run.py --mode sim --headless` | SYNTHETIC closed loop (80 s of simulated time in a few seconds) | no |
| `python control\ibvs_run.py --mode sim` | same with the operator window (real-time) | no |
| `python control\ibvs_run.py --mode camera` | C920 + detection + features + PnP; teach with P; record with R | no |
| `python control\ibvs_run.py --mode compute` | + full controller; press I and move the drone by hand: arrow, `cmd`, `RC` show what WOULD be sent | no |
| `python control\ibvs_run.py --mode replay --video <file.avi>` | recorded video instead of the camera (`--headless --auto-engage` for batch) | no |
| `python control\ibvs_run.py --mode live --bench` | real link, **propellers removed**, take-off disabled | yes |
| `python control\ibvs_run.py --mode live` | flight | yes |

Live and bench modes need `safety.live_control_enabled: true` in `config\ibvs_config.yaml`, and you must type `FLY` / `BENCH`.

## 3. Keys (click the program window first)

| Key | Action |
|---|---|
| **I** | engage IBVS (current stage; live: only after take-off) |
| **H** | hold: pause IBVS, neutral sticks (the drone may drift) |
| **N** / **C** | next stage / restart at stage 1 |
| **L** | firmware LAND |
| **SPACE** | ABORT: neutral + LAND |
| **K** | KILL: disarm immediately (motors stop) |
| **Y** / **T** | arm / take off (live) |
| **W S A D Q E Z X** | manual take-over (your manual-script values); leaves IBVS at once |
| **P** | teach the desired features of the current stage (hold the drone at the desired pose; 30 frames) |
| **R** | record the raw camera video into the run folder (for replay) |
| **F** | bench only: freeze the loop 1 s (watchdog test) |
| **ESC / B** | quit (live: LAND, wait, disarm) |

Window: left = camera image with the detected marker (green), corners 0-3, the **desired** marker (orange), the commanded
drone motion (red arrow) and the optical axis (white cross). Right = state, safety, s, s*, e, ‖e‖, λ, cond(L), v_c,
body command vs estimate, PnP, RC, link.

## 4. Using taught desired features ([P1] learning step)

1. `--mode camera` (or compute), hold the drone at the desired pose of stage 1, press **P**. This saves
   `results\ibvs_desired_coarse.json` and the event line shows z*, a*, x_g*, y_g*, α*.
2. Press **N**, hold the drone at the stage-2 pose, press **P** (`results\ibvs_desired_fine.json`).
3. In `config\ibvs_config.yaml` set `desired_source: "taught"` for those stages.

## 5. Results and plots

Every run creates `results\runs\<date_time>_<mode>\` containing `meta.json` (configs, calibration, software versions,
REAL/REPLAY/SYNTHETIC), `ibvs_log.csv` (one row per frame, 145 columns: runlog/columns.py) and `events.csv`.

```powershell
python analysis\plot_ibvs_run.py results\runs\<run folder>          # 10 figures (PDF + PNG) + summary.txt/json
python analysis\feature_noise.py results\runs\<still-marker run>     # noise floor -> thresholds
python analysis\identify_rc_response.py results\runs\<manual-pulse flight>   # signs, gains, delay, dead band
```

## 6. Updating your GitHub repository (you do this yourself)

From the repository folder (PowerShell):
```bash
git status                       # check: no results/, no calibration images, no .venv
git add -A
git commit -m "IBVS (Paper 1) controller, Pluto interface, safety, logging, plots, tests"
git push origin main             # use your branch name if it is not main
```
`results/` and the calibration images are excluded by `.gitignore`. Your own changes to `config/camera_config.yaml` (measured
values) will be committed too. That is intended: they contain no secrets.
