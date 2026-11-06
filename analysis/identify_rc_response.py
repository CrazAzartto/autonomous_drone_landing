"""
Experimental identification of the Pluto's response to RC commands (docs/07 §7, docs/08 tests E1-E3).

Data: a run of control/ibvs_run.py --mode live in which the drone is flown MANUALLY with the keys
(W/S, A/D, Q/E, Z/X) in short pulses while the camera sees the marker. The log contains the RC actually
set and the camera-estimated body velocities, so no extra hardware is needed. Works on any run folder
(also SYNTHETIC ones, which is how the script itself is tested).

Models fitted (least squares, delay tau found by a grid search, u = RC - 1500):
  pitch    : d(v_forward)/dt = k * u(t - tau) - c * v_forward + b        (tilt -> acceleration)
  roll     : d(v_left)/dt    = k * u(t - tau) - c * v_left    + b
  throttle : v_up = k_z * u  for |u| > db, else 0 (hold)   [FW: setVelocity = (RC-1500)/4 outside +-40]
  yaw      : yaw_rate = k_yaw * u(t - tau)
Reported: sign, gain (overall and separately for u > 0 / u < 0 = asymmetry), dead band, delay, bias, R^2,
and the config values they imply (sign_*, us_per_mps, us_per_radps). Plots: <run>/plots/ident_*.png

Usage:
    python analysis/identify_rc_response.py results/runs/<run folder> [--min-us 20]
"""

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_ibvs_run import load_log  # noqa: E402

G = 9.81


def uniform(t, x, dt):
    grid = np.arange(t[0], t[-1], dt)
    ok = np.isfinite(x)
    if ok.sum() < 10:
        return grid, np.full_like(grid, np.nan)
    return grid, np.interp(grid, t[ok], x[ok])


def shift(u, steps):
    if steps <= 0:
        return u.copy()
    return np.concatenate([np.full(steps, u[0]), u[:-steps]])


def fit_linear(columns, target):
    A = np.column_stack(columns)
    ok = np.all(np.isfinite(A), axis=1) & np.isfinite(target)
    if ok.sum() < 20:
        return None, np.nan
    coef, *_ = np.linalg.lstsq(A[ok], target[ok], rcond=None)
    pred = A[ok] @ coef
    ss = np.sum((target[ok] - target[ok].mean()) ** 2)
    r2 = 1 - np.sum((target[ok] - pred) ** 2) / ss if ss > 0 else np.nan
    return coef, r2


def identify_tilt_axis(t, rc, v, dt, max_delay, min_us):
    tg, u = uniform(t, rc - 1500.0, dt)
    _, vel = uniform(t, v, dt)
    acc = np.gradient(vel, dt)
    # light smoothing of the differentiated velocity (moving average 5 samples)
    acc = np.convolve(acc, np.ones(5) / 5, mode="same")
    best = None
    for steps in range(0, int(max_delay / dt) + 1):
        us = shift(u, steps)
        coef, r2 = fit_linear([us, -vel, np.ones_like(us)], acc)
        if coef is not None and (best is None or r2 > best["r2"]):
            best = {"delay_s": steps * dt, "k": coef[0], "drag": coef[1], "bias": coef[2], "r2": r2, "steps": steps}
    if best is None:
        return None
    us = shift(u, best["steps"])
    for name, mask in (("k_positive", us > min_us), ("k_negative", us < -min_us)):
        if mask.sum() < 20:                      # not enough excitation in this direction
            best[name] = None
            continue
        sel = mask | (np.abs(us) < 1)
        coef, _ = fit_linear([us[sel], -vel[sel], np.ones(sel.sum())], acc[sel])
        best[name] = None if coef is None else float(coef[0])
    best["excitation_samples"] = int(np.sum(np.abs(u) > min_us))
    best["tilt_deg_per_us"] = math.degrees(math.atan(abs(best["k"]) / G))
    best["series"] = (tg, us, acc, vel)
    return best


def identify_throttle(t, rc, v_up, dt, max_delay):
    tg, u = uniform(t, rc - 1500.0, dt)
    _, vel = uniform(t, v_up, dt)
    best = None
    for steps in range(0, int(max_delay / dt) + 1):
        us = shift(u, steps)
        for db in range(0, 81, 5):
            x = np.where(np.abs(us) > db, us, 0.0)
            coef, r2 = fit_linear([x], vel)
            if coef is not None and (best is None or r2 > best["r2"]):
                best = {"delay_s": steps * dt, "deadband_us": db, "k_mps_per_us": coef[0], "r2": r2}
    if best is not None:
        best["implied_us_per_mps"] = 1.0 / best["k_mps_per_us"] if best["k_mps_per_us"] != 0 else np.nan
        best["firmware_value_us_per_mps"] = 400.0
        best["excitation_samples"] = int(np.sum(np.abs(u) > 40))
    return best


def identify_yaw(t, rc, rate, dt, max_delay):
    _, u = uniform(t, rc - 1500.0, dt)
    _, r = uniform(t, rate, dt)
    best = None
    for steps in range(0, int(max_delay / dt) + 1):
        coef, r2 = fit_linear([shift(u, steps)], r)
        if coef is not None and (best is None or r2 > best["r2"]):
            best = {"delay_s": steps * dt, "k_radps_per_us": coef[0], "r2": r2}
    if best is not None and best["k_radps_per_us"] != 0:
        best["implied_us_per_radps"] = abs(1.0 / best["k_radps_per_us"])
        best["implied_sign_yaw"] = int(np.sign(1.0 / best["k_radps_per_us"]))
        best["excitation_samples"] = int(np.sum(np.abs(u) > 20))
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--dt", type=float, default=1 / 30)
    parser.add_argument("--max-delay", type=float, default=0.8)
    parser.add_argument("--min-us", type=float, default=20)
    args = parser.parse_args()

    d = load_log(os.path.join(args.run_dir, "ibvs_log.csv"))
    with open(os.path.join(args.run_dir, "meta.json"), encoding="utf-8") as file:
        kind = json.load(file).get("data_kind", "?")
    t = d["t_s"]
    report = {"data_kind": kind}
    pitch = identify_tilt_axis(t, d["rc_pitch"], d["est_v_forward"], args.dt, args.max_delay, args.min_us)
    roll = identify_tilt_axis(t, d["rc_roll"], d["est_v_left"], args.dt, args.max_delay, args.min_us)
    throttle = identify_throttle(t, d["rc_throttle"], d["est_v_up"], args.dt, args.max_delay)
    yaw = identify_yaw(t, d["rc_yaw"], d["est_yaw_rate"], args.dt, args.max_delay)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out_dir = os.path.join(args.run_dir, "plots")
    os.makedirs(out_dir, exist_ok=True)

    lines = [f"RC RESPONSE IDENTIFICATION [{kind}] {os.path.basename(os.path.normpath(args.run_dir))}"]
    for name, res, sign_key, body in (("pitch", pitch, "sign_pitch", "forward"), ("roll", roll, "sign_roll", "left")):
        if res is None:
            lines.append(f"{name}: not enough data")
            continue
        sign = int(np.sign(res["k"])) if res["k"] != 0 else 0
        lines.append(f"{name}: a_{body} = {res['k']:+.5f} m/s^2 per us (tilt {res['tilt_deg_per_us']:.4f} deg/us), "
                     f"delay {res['delay_s'] * 1000:.0f} ms, drag {res['drag']:.2f} 1/s, bias {res['bias']:+.3f} m/s^2, "
                     f"R^2 {res['r2']:.2f}, k(+) {res['k_positive']}, k(-) {res['k_negative']}, "
                     f"excitation samples {res['excitation_samples']}")
        lines.append(f"   -> config {sign_key}: {sign}   (code expects +1 for pitch, -1 for roll)")
        tg, us, acc, vel = res.pop("series")
        fig, ax = plt.subplots(figsize=(8, 3))
        ax.plot(tg - tg[0], acc, lw=0.8, label=f"measured d(v_{body})/dt")
        ax.plot(tg - tg[0], res["k"] * us - res["drag"] * vel + res["bias"], lw=1, label="model")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("m/s^2")
        ax.set_title(f"[{kind}] {name}: R^2 = {res['r2']:.2f}, delay {res['delay_s'] * 1000:.0f} ms")
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"ident_{name}.png"), dpi=200)
        plt.close(fig)
        report[name] = res
    if throttle:
        lines.append(f"throttle: v_up = {throttle['k_mps_per_us']:.5f} m/s per us x u, zero inside +-{throttle['deadband_us']} us, "
                     f"delay {throttle['delay_s'] * 1000:.0f} ms, R^2 {throttle['r2']:.2f} -> us_per_mps "
                     f"{throttle['implied_us_per_mps']:.0f} (firmware: 400, dead band 40)")
        report["throttle"] = throttle
    if yaw and "implied_us_per_radps" in yaw:
        lines.append(f"yaw: rate = {yaw['k_radps_per_us']:+.5f} rad/s per us, delay {yaw['delay_s'] * 1000:.0f} ms, "
                     f"R^2 {yaw['r2']:.2f} -> us_per_radps {yaw['implied_us_per_radps']:.0f}, sign_yaw {yaw['implied_sign_yaw']}")
        report["yaw"] = yaw
    lines.append("Use only fits with R^2 > 0.5 and enough excitation; repeat 3 times and compare (docs/08).")
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(args.run_dir, "rc_identification.txt"), "w", encoding="utf-8") as file:
        file.write(text + "\n")
    with open(os.path.join(args.run_dir, "rc_identification.json"), "w", encoding="utf-8") as file:
        json.dump(report, file, indent=2, default=float)


if __name__ == "__main__":
    main()
