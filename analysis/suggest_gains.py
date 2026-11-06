"""
Turn MEASURED drone responses into controller gains (docs/07 §5, roadmap step 6).

Input : one or more run folders on which analysis/identify_rc_response.py was run (rc_identification.json).
        Use 3 identification flights; the median of each quantity is used.
Output: the config lines to paste into config/ibvs_config.yaml, and suggested_gains.txt in the first folder.

Rules (the same derivation as the defaults, but with YOUR numbers):
  k   = horizontal acceleration per RC microsecond (largest of pitch/roll = most conservative)
  tau = total delay (largest of pitch/roll best-fit delays) x safety factor (default 1.3)
  inner velocity loop crossover  w_c = (90 deg - phase_margin) / tau          (delay-dominated loop)
  Kv = w_c / k,   Ki = Kv * w_c / 3   (integral time 3/w_c)
  outer IBVS loop: lambda_max_eff = w_c / separation (default 3)  ->  lambda_scale = lambda_max_eff / 1.0
  signs = sign of the measured gains;  throttle us_per_mps = 1/k_z;  yaw us_per_radps from the yaw fit

Usage:
    python analysis/suggest_gains.py results/runs/<flight1> results/runs/<flight2> results/runs/<flight3>
"""

import argparse
import json
import math
import os
import sys

import numpy as np

MIN_R2 = 0.5


def load(run_dirs):
    reports = []
    for run in run_dirs:
        path = os.path.join(run, "rc_identification.json")
        if not os.path.isfile(path):
            sys.exit(f"{path} missing - run analysis/identify_rc_response.py on that folder first")
        with open(path, encoding="utf-8") as file:
            reports.append(json.load(file))
    return reports


def collect(reports, axis, key, min_r2=MIN_R2):
    values = []
    for rep in reports:
        r = rep.get(axis)
        if r and r.get("r2") is not None and r["r2"] >= min_r2 and r.get(key) is not None:
            values.append(float(r[key]))
    return values


def suggest(reports, phase_margin_deg=45.0, separation=3.0, tau_factor=1.3, lambda_max_paper=1.0):
    notes = []
    k_p, k_r = collect(reports, "pitch", "k"), collect(reports, "roll", "k")
    d_p, d_r = collect(reports, "pitch", "delay_s"), collect(reports, "roll", "delay_s")
    if not k_p or not k_r:
        raise ValueError(f"pitch and roll need at least one fit with R^2 >= {MIN_R2} - repeat the pulse flights")
    k_pitch, k_roll = float(np.median(k_p)), float(np.median(k_r))
    signs = {"sign_pitch": int(np.sign(k_pitch)), "sign_roll": int(np.sign(k_roll))}
    if signs["sign_pitch"] != 1:
        notes.append("WARNING: pitch sign is NOT +1 (pitch > 1500 did not accelerate forward). Check the marker "
                     "orientation and headfree mode BEFORE changing the sign.")
    if signs["sign_roll"] != -1:
        notes.append("WARNING: roll sign is NOT -1 (roll < 1500 did not accelerate left). Check the marker orientation "
                     "and headfree mode BEFORE changing the sign.")
    if np.sign(np.max(k_p)) != np.sign(np.min(k_p)) or np.sign(np.max(k_r)) != np.sign(np.min(k_r)):
        notes.append("WARNING: the sign differs between flights - do not fly IBVS until this is explained.")

    k = max(abs(k_pitch), abs(k_roll))
    tau_measured = float(max(np.median(d_p), np.median(d_r)))
    tau = max(0.05, tau_measured * tau_factor)
    w_c = math.radians(90.0 - phase_margin_deg) / tau
    kv = w_c / k
    ki = kv * w_c / 3.0
    lambda_eff = w_c / separation
    lambda_scale = min(1.0, lambda_eff / lambda_max_paper)
    out = {"k_pitch": k_pitch, "k_roll": k_roll, "k_used": k, "tau_measured_s": tau_measured, "tau_design_s": tau,
           "crossover_radps": w_c, "kv_us_per_mps": kv, "ki_us_per_m": ki, "lambda_scale": lambda_scale,
           "lambda_tau_check": lambda_eff * tau_measured, **signs}
    if lambda_eff * tau_measured >= 1 / math.e:
        notes.append("NOTE: lambda_max_eff * tau >= 1/e: expect some overshoot of the outer loop.")
    asym = [r.get("pitch", {}) for r in reports] + [r.get("roll", {}) for r in reports]
    for a in asym:
        kp, kn = a.get("k_positive"), a.get("k_negative")
        if kp and kn and max(abs(kp), abs(kn)) > 1.5 * min(abs(kp), abs(kn)):
            notes.append("NOTE: strong +/- asymmetry in an axis (trim?). Re-trim the drone; the integral term "
                         "compensates small asymmetries only.")
            break

    k_z = collect(reports, "throttle", "k_mps_per_us")
    if k_z:
        out["throttle_us_per_mps"] = 1.0 / float(np.median(k_z))
        out["throttle_deadband_us"] = float(np.median(collect(reports, "throttle", "deadband_us")))
    else:
        notes.append("throttle: no good fit - keep the firmware values 400 us/(m/s), dead band 40.")
    k_y = collect(reports, "yaw", "k_radps_per_us")
    if k_y:
        out["yaw_us_per_radps"] = 1.0 / abs(float(np.median(k_y)))
        out["sign_yaw"] = int(np.sign(1.0 / float(np.median(k_y))))
    else:
        notes.append("yaw: no good fit - keep yaw.enabled: false.")
    return out, notes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+")
    parser.add_argument("--phase-margin", type=float, default=45.0)
    parser.add_argument("--separation", type=float, default=3.0)
    parser.add_argument("--tau-factor", type=float, default=1.3)
    args = parser.parse_args()

    reports = load(args.run_dirs)
    if any(r.get("data_kind") == "SYNTHETIC" for r in reports):
        print("NOTE: SYNTHETIC identification data - these gains are for the simulator, not the drone.\n")
    try:
        g, notes = suggest(reports, args.phase_margin, args.separation, args.tau_factor)
    except ValueError as exc:
        sys.exit(f"Cannot suggest gains: {exc}")

    lines = [
        f"Measured: k_pitch {g['k_pitch']:+.5f}, k_roll {g['k_roll']:+.5f} m/s^2 per us, delay {g['tau_measured_s'] * 1000:.0f} ms "
        f"(design {g['tau_design_s'] * 1000:.0f} ms), crossover {g['crossover_radps']:.2f} rad/s, "
        f"lambda_max_eff * tau = {g['lambda_tau_check']:.2f} (1/e = 0.37)",
        "",
        "Paste into config/ibvs_config.yaml:",
        "ibvs:",
        f"  lambda_scale: {g['lambda_scale']:.2f}",
        "command_interface:",
        "  horizontal:",
        f"    kv_us_per_mps: {g['kv_us_per_mps']:.0f}",
        f"    ki_us_per_m: {g['ki_us_per_m']:.0f}",
        f"    sign_pitch: {g['sign_pitch']}",
        f"    sign_roll: {g['sign_roll']}",
    ]
    if "throttle_us_per_mps" in g:
        lines += ["  vertical:", f"    us_per_mps: {g['throttle_us_per_mps']:.0f}",
                  f"    deadband_us: {g['throttle_deadband_us']:.0f}"]
    if "yaw_us_per_radps" in g:
        lines += ["  yaw:", f"    us_per_radps: {g['yaw_us_per_radps']:.0f}", f"    sign_yaw: {g['sign_yaw']}",
                  "    enabled: true      # only after roadmap step 7 passed with yaw disabled"]
    lines += ["simulation:   # to re-check the new gains in the simulator first",
              f"  tilt_deg_per_us: {math.degrees(math.atan(g['k_used'] / 9.81)):.4f}",
              f"  delay_s: {g['tau_measured_s']:.2f}"]
    text = "\n".join(lines + [""] + notes)
    print(text)
    with open(os.path.join(args.run_dirs[0], "suggested_gains.txt"), "w", encoding="utf-8") as file:
        file.write(text + "\n")


if __name__ == "__main__":
    main()
