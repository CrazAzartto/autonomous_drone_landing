"""
Publication-quality plots and a convergence summary for ONE run folder written by control/ibvs_run.py.

Usage:
    python analysis/plot_ibvs_run.py results/runs/<run folder>            # writes <run>/plots/*.pdf/.png
    python analysis/plot_ibvs_run.py results/runs/<run folder> --show     # also open the windows

Figures (every title starts with the data kind: REAL / REPLAY / SYNTHETIC)
  fig01_features        s and s* vs time                      ([P1] features x_n, y_n, a_n, alpha)
  fig02_errors          e = s - s* along the axes             (as [P1] Fig. 2b/2e)
  fig03_error_norm      ||e|| (linear + log), thresholds       (as [P1] Fig. 2a/2d)
  fig04_pnp_position    solvePnP X, Y, Z (validation; + truth in SYNTHETIC runs)
  fig05_orientation     alpha, image heading, PnP heading, drone attitude telemetry
  fig06_gain            adaptive lambda                        ([P1] p.5)
  fig07_velocities      v_c of Eq.(1) (as [P1] Fig. 2c/2f) and body set-point vs camera estimate
  fig08_rc              RC channels with saturation / slew markers
  fig09_status          detection, safety level, controller state, stage, link
  fig10_paper_layout    per stage: ||e||, errors, control signals - the 3-panel layout of [P1] Fig. 2
  summary.json / summary.txt   convergence time, final residual, statistics
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

STATES = ["MANUAL", "IBVS", "HOLD", "LANDING", "DONE", "KILLED"]
LEVELS = ["OK", "WARN", "HOLD", "LAND", "KILL"]


def load_log(path):
    with open(path, newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        raise SystemExit(f"{path} is empty")
    columns = {}
    for key in rows[0]:
        values = [r[key] for r in rows]
        try:
            columns[key] = np.array([float(v) if v not in ("", None) else np.nan for v in values])
        except ValueError:
            columns[key] = np.array(values, dtype=object)
    return columns


def load_events(path):
    if not os.path.isfile(path):
        return []
    with open(path, newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def stage_segments(d):
    """[(stage_index, start_index, end_index)] of contiguous rows with the same stage, IBVS active."""
    active = d["ibvs_active"] == 1
    stage = d["stage_index"]
    segments = []
    start = None
    for i in range(len(stage)):
        if active[i] and (start is None or stage[i] != stage[start]):
            if start is not None:
                segments.append((int(stage[start]), start, i - 1))
            start = i
        elif not active[i] and start is not None:
            segments.append((int(stage[start]), start, i - 1))
            start = None
    if start is not None:
        segments.append((int(stage[start]), start, len(stage) - 1))
    # merge segments of the same stage interrupted by short pauses (keep first start)
    merged = []
    for seg in segments:
        if merged and merged[-1][0] == seg[0]:
            merged[-1] = (seg[0], merged[-1][1], seg[2])
        else:
            merged.append(seg)
    return merged


def summarise(d, meta, events):
    stages_cfg = meta.get("ibvs_config", {}).get("ibvs", {}).get("stages", [])
    t = d["t_s"]
    summary = {"data_kind": meta.get("data_kind", "?"), "mode": meta.get("mode", "?"),
               "duration_s": float(t[-1] - t[0]), "rows": int(len(t)),
               "detection_rate": float(np.nanmean(d["detected"])),
               "frame_age_ms": {"median": float(np.nanmedian(d["frame_age_s"]) * 1000),
                                "p95": float(np.nanpercentile(d["frame_age_s"], 95) * 1000)},
               "loop_dt_ms": {"median": float(np.nanmedian(d["loop_dt_s"][1:]) * 1000),
                              "p95": float(np.nanpercentile(d["loop_dt_s"][1:], 95) * 1000)},
               "stages": []}
    if "rc_echo_latency_s" in d and np.any(np.isfinite(d["rc_echo_latency_s"])):
        summary["rc_echo_latency_ms_median"] = float(np.nanmedian(d["rc_echo_latency_s"]) * 1000)
    for stage_index, i0, i1 in stage_segments(d):
        cfg = stages_cfg[stage_index] if stage_index < len(stages_cfg) else {}
        thr = float(cfg.get("error_threshold", np.nan))
        dwell = float(cfg.get("dwell_s", 0.0))
        tt, norm = t[i0:i1 + 1], d["e_norm_control"][i0:i1 + 1]
        conv_time = None
        below_since = None
        for k in range(len(tt)):
            if np.isfinite(norm[k]) and norm[k] < thr:
                below_since = tt[k] if below_since is None else below_since
                if tt[k] - below_since >= dwell:
                    conv_time = float(tt[k] - tt[0])
                    break
            else:
                below_since = None
        last = (tt >= tt[-1] - 2.0)
        e_last = np.vstack([d[k][i0:i1 + 1][last] for k in ("e_xn", "e_yn", "e_an", "e_alpha")]).T
        sat = np.nanmean(np.maximum.reduce([d[k][i0:i1 + 1] for k in ("sat_forward", "sat_left", "sat_up")]))
        summary["stages"].append({
            "stage_index": stage_index, "name": cfg.get("name", str(stage_index)),
            "threshold": thr, "dwell_s": dwell, "duration_s": float(tt[-1] - tt[0]),
            "convergence_time_s": conv_time,
            "converged": conv_time is not None,
            "initial_error_norm": float(d["e_norm"][i0]),
            "final_error_norm": float(d["e_norm"][i1]),
            "final_e": [float(d[k][i1]) for k in ("e_xn", "e_yn", "e_an", "e_alpha")],
            "last_2s_mean_abs_e": np.nanmean(np.abs(e_last), axis=0).tolist(),
            "last_2s_rms_error_norm": float(np.sqrt(np.nanmean(d["e_norm"][i0:i1 + 1][last] ** 2))),
            "saturation_fraction": float(sat) if np.isfinite(sat) else None,
        })
    summary["events"] = [f"{e['time_s']} {e['kind']}: {e['message']}" for e in events
                         if e["kind"] in ("converged", "timeout", "land", "kill", "finished", "ibvs", "watchdog",
                                          "override", "comm_error")]
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 100,
                         "savefig.dpi": 300, "font.family": "serif", "legend.fontsize": 8,
                         "axes.titlesize": 11})

    run = args.run_dir
    d = load_log(os.path.join(run, "ibvs_log.csv"))
    with open(os.path.join(run, "meta.json"), encoding="utf-8") as file:
        meta = json.load(file)
    events = load_events(os.path.join(run, "events.csv"))
    kind = meta.get("data_kind", "?")
    tag = f"[{kind}] {os.path.basename(os.path.normpath(run))}"
    out_dir = os.path.join(run, "plots")
    os.makedirs(out_dir, exist_ok=True)
    t = d["t_s"] - d["t_s"][0]
    segments = stage_segments(d)
    stages_cfg = meta.get("ibvs_config", {}).get("ibvs", {}).get("stages", [])

    def shade(ax):
        colours = ["#dbe9f6", "#f6e6db", "#e2f6db"]
        for k, (si, i0, i1) in enumerate(segments):
            ax.axvspan(t[i0], t[i1], color=colours[si % 3], alpha=0.6, lw=0)

    def save(fig, name):
        fig.tight_layout()
        for ext in ("pdf", "png"):
            fig.savefig(os.path.join(out_dir, f"{name}.{ext}"))
        if not args.show:
            plt.close(fig)

    # 1 features
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(7.5, 8))
    labels = [("s_xn", "sd_xn", "$x_n$ [m]"), ("s_yn", "sd_yn", "$y_n$ [m]"), ("s_an", "sd_an", "$a_n$ [m]"),
              ("s_alpha", "sd_alpha", r"$\alpha$ [deg]")]
    for ax, (key, dkey, label) in zip(axes, labels):
        scale = 180 / math.pi if key == "s_alpha" else 1.0
        shade(ax)
        ax.plot(t, d[key] * scale, lw=1.0, label="current $s$")
        ax.plot(t, d[dkey] * scale, "--", lw=1.2, label="desired $s^*$")
        ax.set_ylabel(label)
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{tag}\nVisual features vs desired features (shaded: IBVS stages)")
    axes[-1].set_xlabel("time [s]")
    save(fig, "fig01_features")

    # 2 errors along the axes
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    shade(ax)
    for key, label in (("e_xn", "$e_{x_n}$ [m]"), ("e_yn", "$e_{y_n}$ [m]"), ("e_an", "$e_{a_n}$ [m]"),
                       ("e_alpha", r"$e_\alpha$ [rad]")):
        ax.plot(t, d[key], lw=1.0, label=label)
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error")
    ax.set_title(f"{tag}\nFeature errors $e = s - s^*$")
    ax.legend(ncol=4, loc="upper right")
    save(fig, "fig02_errors")

    # 3 error norm
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7.5, 5.5))
    for ax, log in zip(axes, (False, True)):
        shade(ax)
        ax.plot(t, d["e_norm"], lw=1.1, label=r"$\|e\|$ (all 4 features, as [P1])")
        ax.plot(t, d["e_norm_control"], lw=0.9, alpha=0.8, label=r"$\|e\|$ used for stopping")
        for si, i0, i1 in segments:
            thr = stages_cfg[si]["error_threshold"] if si < len(stages_cfg) else np.nan
            ax.hlines(thr, t[i0], t[i1], colors="r", linestyles="--", lw=1)
        for e in events:
            if e["kind"] == "converged":
                ax.axvline(float(e["time_s"]) - d["t_s"][0], color="g", lw=1)
        if log:
            ax.set_yscale("log")
        ax.set_ylabel(r"$\|e\|$")
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{tag}\nError norm (dashed: stage thresholds, green: convergence)")
    axes[-1].set_xlabel("time [s]")
    save(fig, "fig03_error_norm")

    # 4 PnP position
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(7.5, 6))
    for ax, key, true_key, label in zip(axes, ("pnp_x", "pnp_y", "pnp_z"), ("true_x", "true_y", "true_z"),
                                        ("X [m]", "Y [m]", "Z [m]")):
        shade(ax)
        ax.plot(t, d[key], lw=1.0, label="solvePnP (camera frame)")
        if np.any(np.isfinite(d.get(true_key, np.array([np.nan])))):
            ax.plot(t, d[true_key], "k--", lw=0.8, label="truth (simulation)")
        ax.set_ylabel(label)
    axes[0].legend(loc="upper right")
    axes[0].set_title(f"{tag}\nEstimated marker position in the camera frame (validation, not used by IBVS)")
    axes[-1].set_xlabel("time [s]")
    save(fig, "fig04_pnp_position")

    # 5 orientation
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7.5, 5))
    shade(axes[0])
    axes[0].plot(t, np.degrees(d["s_alpha"]), lw=1, label=r"$\alpha$ (image feature)")
    axes[0].plot(t, np.degrees(d["heading_img_rad"]), lw=1, label=r"heading $\psi=\alpha-90^\circ$")
    axes[0].plot(t, np.degrees(d["pnp_heading_rad"]), "--", lw=0.8, label="heading from solvePnP")
    if np.any(np.isfinite(d.get("true_yaw_rad", np.array([np.nan])))):
        axes[0].plot(t, np.degrees(d["true_yaw_rad"]), "k:", lw=0.8, label="truth (simulation)")
    axes[0].set_ylabel("angle [deg]")
    axes[0].legend(loc="upper right")
    shade(axes[1])
    plotted = False
    for key, label in (("tel_roll_deg", "roll (drone)"), ("tel_pitch_deg", "pitch (drone)")):
        if np.any(np.isfinite(d[key])):
            axes[1].plot(t, d[key], lw=1, label=label)
            plotted = True
    axes[1].set_ylabel("attitude [deg]")
    axes[1].set_xlabel("time [s]")
    if plotted:
        axes[1].legend(loc="upper right")
    else:
        axes[1].text(0.5, 0.5, "no drone telemetry in this run", transform=axes[1].transAxes, ha="center")
    axes[0].set_title(f"{tag}\nMarker orientation and drone attitude (telemetry, if available)")
    save(fig, "fig05_orientation")

    # 6 gain
    fig, ax = plt.subplots(figsize=(7.5, 3))
    shade(ax)
    ax.plot(t, d["lambda"], lw=1.2)
    ax.set_ylabel(r"$\lambda$ [1/s]")
    ax.set_xlabel("time [s]")
    ax.set_title(f"{tag}\nAdaptive gain " + r"$\lambda=(\lambda_{max}-\lambda_{min})\|e\|/\|e_{max}\|+\lambda_{min}$")
    save(fig, "fig06_gain")

    # 7 velocities
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(7.5, 8.5))
    shade(axes[0])
    for key, label in (("vc_x", "$v_x$"), ("vc_y", "$v_y$"), ("vc_z", "$v_z$"), ("vc_wz", r"$\omega_z$")):
        axes[0].plot(t, d[key], lw=1, label=label)
    axes[0].set_ylabel("$v_c$ (Eq. 1)")
    axes[0].legend(ncol=4, loc="upper right")
    for ax, cmd, est, label in zip(axes[1:], ("cmd_v_forward", "cmd_v_left", "cmd_v_up"),
                                   ("est_v_forward", "est_v_left", "est_v_up"),
                                   ("forward [m/s]", "left [m/s]", "up [m/s]")):
        shade(ax)
        ax.plot(t, d[cmd], lw=1.1, label="commanded (IBVS, limited)")
        ax.plot(t, d[est], lw=0.9, alpha=0.8, label="estimated (camera, Kalman)")
        ax.set_ylabel(label)
    axes[1].legend(loc="upper right")
    axes[0].set_title(f"{tag}\nIBVS camera twist $v_c$ and drone-frame velocity set-point vs estimate")
    axes[-1].set_xlabel("time [s]")
    save(fig, "fig07_velocities")

    # 8 RC
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(7.5, 8))
    for ax, key, sat, label in zip(axes, ("rc_pitch", "rc_roll", "rc_throttle", "rc_yaw"),
                                   ("sat_forward", "sat_left", "sat_up", "sat_yaw"),
                                   ("pitch", "roll", "throttle", "yaw")):
        shade(ax)
        ax.plot(t, d[key], lw=1)
        mask = d[sat] == 1
        if np.any(mask):
            ax.plot(t[mask], d[key][mask], "r.", ms=3, label="saturated")
            ax.legend(loc="upper right")
        ax.axhline(1500, color="k", lw=0.5)
        ax.set_ylabel(f"{label} [us]")
    axes[0].set_title(f"{tag}\nRC channel commands")
    axes[-1].set_xlabel("time [s]")
    save(fig, "fig08_rc")

    # 9 status
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(7.5, 6))
    axes[0].step(t, d["detected"], where="post")
    axes[0].set_ylabel("detected")
    axes[1].step(t, [LEVELS.index(v) if v in LEVELS else -1 for v in d["safety_level"]], where="post")
    axes[1].set_yticks(range(len(LEVELS)), LEVELS)
    axes[1].set_ylabel("safety")
    axes[2].step(t, [STATES.index(v) if v in STATES else -1 for v in d["controller_state"]], where="post")
    axes[2].set_yticks(range(len(STATES)), STATES)
    axes[2].set_ylabel("state")
    axes[3].step(t, d["stage_index"] + 1, where="post", label="stage")
    axes[3].step(t, d["link_ok"], where="post", label="link ok")
    axes[3].legend(loc="upper right")
    axes[0].set_title(f"{tag}\nDetection, safety level, controller state, stage")
    axes[-1].set_xlabel("time [s]")
    save(fig, "fig09_status")

    # 10 paper layout: one row per stage, columns = ||e||, errors, control signals
    if segments:
        fig, axes = plt.subplots(len(segments), 3, figsize=(11, 3.2 * len(segments)), squeeze=False)
        for row, (si, i0, i1) in enumerate(segments):
            tt = t[i0:i1 + 1] - t[i0]
            name = stages_cfg[si]["name"] if si < len(stages_cfg) else str(si)
            axes[row, 0].plot(tt, d["e_norm"][i0:i1 + 1])
            if si < len(stages_cfg):
                axes[row, 0].axhline(stages_cfg[si]["error_threshold"], color="r", ls="--", lw=1)
            axes[row, 0].set_title(f"({'abcdef'[3 * row]}) error convergence, stage '{name}'")
            for key, label in (("e_xn", "$e_{x_n}$"), ("e_yn", "$e_{y_n}$"), ("e_an", "$e_{a_n}$"),
                               ("e_alpha", r"$e_\alpha$")):
                axes[row, 1].plot(tt, d[key][i0:i1 + 1], label=label)
            axes[row, 1].legend(ncol=2)
            axes[row, 1].set_title(f"({'abcdef'[3 * row + 1]}) errors along the axes")
            for key, label in (("vc_x", "$v_x$"), ("vc_y", "$v_y$"), ("vc_z", "$v_z$"), ("vc_wz", r"$\omega_z$")):
                axes[row, 2].plot(tt, d[key][i0:i1 + 1], label=label)
            axes[row, 2].legend(ncol=2)
            axes[row, 2].set_title(f"({'abcdef'[3 * row + 2]}) control signals $v_c$")
            for ax in axes[row]:
                ax.set_xlabel("time since stage start [s]")
        fig.suptitle(f"{tag} - layout of [P1] Fig. 2", y=1.0)
        save(fig, "fig10_paper_layout")

    summary = summarise(d, meta, events)
    with open(os.path.join(run, "summary.json"), "w", encoding="utf-8") as file:
        json.dump(summary, file, indent=2)
    if summary["data_kind"] == "REPLAY":
        summary["note"] = ("REPLAY is OPEN LOOP: the recorded drone did not receive these commands, so 'convergence' "
                           "here only tests the software, not the controller.")
    lines = [f"RUN {os.path.basename(os.path.normpath(run))}   data: {summary['data_kind']}   mode: {summary['mode']}",
             summary.get("note", ""),
             f"duration {summary['duration_s']:.1f} s, {summary['rows']} cycles, detection rate {summary['detection_rate'] * 100:.1f} %",
             f"frame age median {summary['frame_age_ms']['median']:.1f} ms (p95 {summary['frame_age_ms']['p95']:.1f}), "
             f"loop dt median {summary['loop_dt_ms']['median']:.1f} ms"]
    for s in summary["stages"]:
        conv = f"{s['convergence_time_s']:.2f} s" if s["converged"] else "NOT converged"
        lines.append(f"stage '{s['name']}': threshold {s['threshold']}, convergence {conv}, "
                     f"||e|| {s['initial_error_norm']:.4f} -> {s['final_error_norm']:.4f}, "
                     f"last-2-s RMS ||e|| {s['last_2s_rms_error_norm']:.4f}, "
                     f"mean |e| last 2 s = {np.round(s['last_2s_mean_abs_e'], 4).tolist()}")
    lines += ["events:"] + ["  " + e for e in summary["events"]]
    text = "\n".join(lines)
    with open(os.path.join(run, "summary.txt"), "w", encoding="utf-8") as file:
        file.write(text + "\n")
    print(text)
    print(f"\nPlots written to {out_dir}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())
