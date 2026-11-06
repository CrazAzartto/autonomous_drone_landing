"""
One run = one folder:

  results/runs/2026-11-05_14-03-22_compute/
      meta.json      mode, SYNTHETIC flag, configs, calibration summary, software versions, start time
      ibvs_log.csv   one row per control cycle (columns: runlog/columns.py)
      events.csv     time, kind, message (safety transitions, saturation, comm errors, key presses ...)
      summary.json   written by analysis/plot_ibvs_run.py

Data recorded from the simulator or replay are marked in meta.json ("data_kind") and every plot title
says SYNTHETIC / REPLAY, so they can never be mistaken for flight results.
"""

import csv
import datetime
import json
import os
import platform
import subprocess
import sys

import numpy as np


def _jsonable(value):
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def git_commit(repo_root):
    try:
        out = subprocess.run(["git", "-C", repo_root, "rev-parse", "--short", "HEAD"], capture_output=True,
                             text=True, timeout=3)
        dirty = subprocess.run(["git", "-C", repo_root, "status", "--porcelain"], capture_output=True,
                               text=True, timeout=3)
        if out.returncode == 0:
            return out.stdout.strip() + ("-dirty" if dirty.stdout.strip() else "")
    except Exception:
        pass
    return "unknown"


class RunLogger:

    def __init__(self, base_dir, mode, columns, data_kind, meta=None, run_name=None, flush_every=20):
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        name = f"{stamp}_{mode}" + (f"_{run_name}" if run_name else "")
        self.run_dir = os.path.join(base_dir, name)
        os.makedirs(self.run_dir, exist_ok=True)
        self.columns = list(columns)
        self.flush_every = flush_every
        self._rows = 0

        import cv2
        meta = dict(meta or {})
        meta.update({
            "mode": mode,
            "data_kind": data_kind,                     # "REAL" | "SYNTHETIC" | "REPLAY"
            "started": datetime.datetime.now().isoformat(timespec="seconds"),
            "python": sys.version.split()[0],
            "opencv": cv2.__version__,
            "numpy": np.__version__,
            "platform": platform.platform(),
            "git_commit": git_commit(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        })
        with open(os.path.join(self.run_dir, "meta.json"), "w", encoding="utf-8") as file:
            json.dump(_jsonable(meta), file, indent=2)

        self._log_file = open(os.path.join(self.run_dir, "ibvs_log.csv"), "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._log_file, fieldnames=self.columns, extrasaction="ignore")
        self._writer.writeheader()
        self._event_file = open(os.path.join(self.run_dir, "events.csv"), "w", newline="", encoding="utf-8")
        self._event_writer = csv.writer(self._event_file)
        self._event_writer.writerow(["time_s", "kind", "message"])

    def row(self, values):
        clean = {}
        for key in self.columns:
            value = values.get(key, "")
            if isinstance(value, (bool, np.bool_)):
                value = int(value)
            elif isinstance(value, (float, np.floating)):
                value = float(value) if np.isfinite(value) else ("nan" if np.isnan(value) else value)
            clean[key] = value
        self._writer.writerow(clean)
        self._rows += 1
        if self._rows % self.flush_every == 0:
            self._log_file.flush()

    def event(self, time_s, kind, message):
        self._event_writer.writerow([f"{time_s:.4f}", kind, message])
        self._event_file.flush()

    def close(self):
        for file in (self._log_file, self._event_file):
            try:
                file.flush()
                file.close()
            except Exception:
                pass
