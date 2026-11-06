"""
Noise floor of the visual features (docs/08 test C3) -> basis for the convergence thresholds and the
Kalman measurement noise.

Record a run with the marker NOT moving (drone resting on a transparent stand / held in a clamp above
the camera at the stage height):
    python control/ibvs_run.py --mode camera          (30-60 s, then ESC)
    python analysis/feature_noise.py results/runs/<that run>

Reports the standard deviation of x_n, y_n, a_n, alpha and of the PnP position, the noise of ||e||
around the mean feature vector, and recommends:
  error threshold  >= 3 x (noise of ||e||)          a threshold below that can only be met by luck
  velocity_estimation.measurement_std_xy / _z      = measured std of x_n,y_n / a_n
"""

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_ibvs_run import load_log  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--skip-s", type=float, default=1.0, help="ignore the first seconds")
    args = parser.parse_args()

    d = load_log(os.path.join(args.run_dir, "ibvs_log.csv"))
    with open(os.path.join(args.run_dir, "meta.json"), encoding="utf-8") as file:
        meta = json.load(file)
    t = d["t_s"] - d["t_s"][0]
    sel = (t >= args.skip_s) & (d["detected"] == 1)
    if sel.sum() < 30:
        sys.exit("fewer than 30 frames with a detected marker - record longer")
    s = np.vstack([d["s_xn"][sel], d["s_yn"][sel], d["s_an"][sel], d["s_alpha"][sel]]).T
    mean = s.mean(axis=0)
    dev = s - mean
    dev[:, 3] = (dev[:, 3] + math.pi) % (2 * math.pi) - math.pi
    std = dev.std(axis=0)
    norm_noise = np.linalg.norm(dev, axis=1)
    norm_noise_xyz = np.linalg.norm(dev[:, :3], axis=1)
    pnp = np.vstack([d["pnp_x"][sel], d["pnp_y"][sel], d["pnp_z"][sel]]).T
    result = {
        "data_kind": meta.get("data_kind"), "frames": int(sel.sum()), "duration_s": float(t[sel][-1] - t[sel][0]),
        "detection_rate": float(np.mean(d["detected"][t >= args.skip_s])),
        "mean_features": mean.tolist(), "std_features": std.tolist(),
        "std_alpha_deg": math.degrees(std[3]),
        "pnp_mean_m": np.nanmean(pnp, axis=0).tolist(), "pnp_std_m": np.nanstd(pnp, axis=0).tolist(),
        "norm_noise_rms": float(np.sqrt(np.mean(norm_noise ** 2))),
        "norm_noise_p95": float(np.percentile(norm_noise, 95)),
        "norm_noise_xyz_rms": float(np.sqrt(np.mean(norm_noise_xyz ** 2))),
    }
    result["recommended_min_threshold_all4"] = 3 * result["norm_noise_rms"]
    result["recommended_min_threshold_xyz"] = 3 * result["norm_noise_xyz_rms"]
    result["recommended_measurement_std_xy"] = float(max(std[0], std[1]))
    result["recommended_measurement_std_z"] = float(std[2])
    text = (f"FEATURE NOISE [{result['data_kind']}] {result['frames']} frames, {result['duration_s']:.1f} s, "
            f"detection {result['detection_rate'] * 100:.1f} %\n"
            f"mean s = x_n {mean[0]:+.4f} m, y_n {mean[1]:+.4f} m, a_n {mean[2]:.4f} m, alpha {math.degrees(mean[3]):+.2f} deg\n"
            f"std  s = {std[0] * 1000:.2f} mm, {std[1] * 1000:.2f} mm, {std[2] * 1000:.2f} mm, {math.degrees(std[3]):.3f} deg\n"
            f"std PnP X,Y,Z = {np.round(np.array(result['pnp_std_m']) * 1000, 2).tolist()} mm\n"
            f"noise of ||e||: RMS {result['norm_noise_rms']:.4f}, p95 {result['norm_noise_p95']:.4f}\n"
            f"=> stopping thresholds should be >= {result['recommended_min_threshold_all4']:.4f} "
            f"(all 4 features) / {result['recommended_min_threshold_xyz']:.4f} (x_n, y_n, a_n)\n"
            f"=> velocity_estimation.measurement_std_xy: {result['recommended_measurement_std_xy']:.4f}, "
            f"measurement_std_z: {result['recommended_measurement_std_z']:.4f}\n"
            f"Note: in flight the noise is larger (vibration, tilt, motion blur); repeat with a hovering drone.")
    print(text)
    with open(os.path.join(args.run_dir, "feature_noise.json"), "w", encoding="utf-8") as file:
        json.dump(result, file, indent=2)
    with open(os.path.join(args.run_dir, "feature_noise.txt"), "w", encoding="utf-8") as file:
        file.write(text + "\n")


if __name__ == "__main__":
    main()
