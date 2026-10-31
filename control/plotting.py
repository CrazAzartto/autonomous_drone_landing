"""
Plots of results/control_log.csv : errors, velocities and RC commands against time.

    python control\\plotting.py                     # plots results\\control_log.csv
    python control\\plotting.py my_old_log.csv
"""

import csv
import os
import sys

import matplotlib.pyplot as plt


def read_log(path):
    with open(path, newline="") as file:
        rows = list(csv.DictReader(file))
    columns = {}
    for name in rows[0].keys() if rows else []:
        values = []
        for row in rows:
            try:
                values.append(float(row[name]))
            except ValueError:
                values.append(float("nan"))
        columns[name] = values
    return columns


def plot_control_log(path):
    if not os.path.isfile(path):
        print(f"No log file {path}")
        return
    data = read_log(path)
    if not data:
        print("Log is empty.")
        return
    t = data["time_s"]

    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 10))
    figure.suptitle(os.path.basename(path))

    axes[0].plot(t, [v * 100 for v in data["e_forward_m"]], label="e_forward")
    axes[0].plot(t, [v * 100 for v in data["e_left_m"]], label="e_left")
    axes[0].plot(t, [v * 100 for v in data["e_up_m"]], label="e_up")
    axes[0].set_ylabel("error (cm)")

    axes[1].plot(t, data["e_yaw_deg"], label="e_yaw")
    axes[1].set_ylabel("yaw error (deg)")

    axes[2].plot(t, [v * 100 for v in data["v_forward_mps"]], label="v_forward")
    axes[2].plot(t, [v * 100 for v in data["v_left_mps"]], label="v_left")
    axes[2].plot(t, [v * 100 for v in data["v_up_mps"]], label="v_up")
    axes[2].set_ylabel("velocity (cm/s)")

    for name in ("rc_roll", "rc_pitch", "rc_throttle", "rc_yaw"):
        axes[3].plot(t, data[name], label=name)
    axes[3].set_ylabel("RC (PWM)")
    axes[3].set_xlabel("time (s)")

    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from common.config import load_config, repo_path
    log_path = sys.argv[1] if len(sys.argv) > 1 else repo_path(load_config(None)["control"]["log_file"])
    plot_control_log(log_path)
