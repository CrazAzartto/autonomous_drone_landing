"""
Per-cycle safety supervisor. It decides what the drone is ALLOWED to do this cycle; the control program
then takes the most severe action.

Levels (ordered):  OK < WARN < HOLD < LAND < KILL
  OK    autonomous command may be sent
  WARN  command sent, but something should be looked at (logged)
  HOLD  autonomous command suspended: NEUTRAL sticks (1500). On the Pluto this means level attitude and
        firmware altitude hold - NOT a position hold: the drone may drift (docs/07 §6).
  LAND  firmware LAND command (MSP_SET_COMMAND 2): the firmware descends and disarms on touchdown
  KILL  disarm immediately (motors stop, the drone falls) - only on operator request

Every check is configurable in config/ibvs_config.yaml -> safety (each default value is explained there).
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

OK, WARN, HOLD, LAND, KILL = 0, 1, 2, 3, 4
LEVEL_NAMES = {OK: "OK", WARN: "WARN", HOLD: "HOLD", LAND: "LAND", KILL: "KILL"}


@dataclass
class SafetyInputs:
    now: float
    frame_age_s: float                    # now - capture time of the frame used
    detected: bool
    lost_time_s: float                    # time since the last valid detection (0 if detected)
    features: Optional[np.ndarray] = None # s = [x_n, y_n, a_n, alpha]
    corners_px: Optional[np.ndarray] = None
    image_size: tuple = (1280, 720)
    condition: float = 1.0
    command: Optional[np.ndarray] = None  # body velocity command
    link_ok: bool = True
    live: bool = False
    telemetry: dict = field(default_factory=dict)
    loop_dt_s: float = 0.0


@dataclass
class SafetyDecision:
    level: int = OK
    reasons: List[str] = field(default_factory=list)

    @property
    def name(self):
        return LEVEL_NAMES[self.level]

    def raise_to(self, level, reason):
        self.reasons.append(f"{LEVEL_NAMES[level]}: {reason}")
        self.level = max(self.level, level)


class SafetySupervisor:

    def __init__(self, config):
        self.c = config

    def evaluate(self, x: SafetyInputs) -> SafetyDecision:
        c = self.c
        d = SafetyDecision()

        # ---- timing ----
        if not math.isfinite(x.frame_age_s) or x.frame_age_s > c["max_frame_age_s"]:
            d.raise_to(HOLD, f"stale camera frame ({x.frame_age_s * 1000:.0f} ms)")
        if x.loop_dt_s > c["max_loop_dt_s"]:
            d.raise_to(HOLD, f"control loop too slow ({x.loop_dt_s * 1000:.0f} ms)")

        # ---- marker visibility ----
        if not x.detected:
            if x.lost_time_s >= c["marker_lost_land_s"]:
                d.raise_to(LAND, f"marker lost for {x.lost_time_s:.2f} s")
            else:
                d.raise_to(HOLD, f"marker not detected ({x.lost_time_s:.2f} s)")

        # ---- numerical sanity ----
        if x.detected:
            if x.features is None or not np.all(np.isfinite(x.features)):
                d.raise_to(HOLD, "invalid (NaN/inf) features")
            else:
                height = float(x.features[2])
                if height > c["max_height_m"]:
                    d.raise_to(HOLD, f"height {height:.2f} m above max {c['max_height_m']:.2f} m")
                if height < c["min_height_m"]:
                    d.raise_to(LAND, f"height {height:.2f} m below min {c['min_height_m']:.2f} m")
                radius = math.hypot(x.features[0], x.features[1])
                if radius > c["fence_radius_m"]:
                    d.raise_to(HOLD, f"outside fence ({radius:.2f} m from the camera axis)")
            if x.corners_px is not None:
                w, h = x.image_size
                corners = np.asarray(x.corners_px).reshape(-1, 2)
                margin = min(corners[:, 0].min(), corners[:, 1].min(), w - corners[:, 0].max(), h - corners[:, 1].max())
                if margin < c["image_border_margin_px"]:
                    d.raise_to(WARN, f"marker {margin:.0f} px from the image border")
            if not math.isfinite(x.condition) or x.condition > c["max_condition"]:
                d.raise_to(HOLD, f"interaction matrix condition {x.condition:.1f}")
        if x.command is not None and not np.all(np.isfinite(x.command)):
            d.raise_to(HOLD, "invalid (NaN/inf) command")

        # ---- drone link and telemetry (live only) ----
        if x.live:
            if not x.link_ok:
                d.raise_to(LAND, "drone link lost (firmware failsafe should land it)")
            tel = x.telemetry or {}
            fresh = math.isfinite(tel.get("age_s", float("nan"))) and tel.get("age_s", 1e9) < c["telemetry_max_age_s"]
            if fresh:
                vbat = tel.get("vbat_v")
                if vbat is not None and 0.5 < vbat < c["battery_land_v"]:
                    d.raise_to(LAND, f"battery {vbat:.2f} V")
                roll, pitch = tel.get("roll_deg"), tel.get("pitch_deg")
                if roll is not None and pitch is not None and max(abs(roll), abs(pitch)) > c["max_tilt_deg"]:
                    d.raise_to(HOLD, f"tilt roll {roll:.1f} pitch {pitch:.1f} deg")
            elif c.get("require_telemetry", False):
                d.raise_to(HOLD, "no fresh telemetry")
            else:
                d.raise_to(WARN, "telemetry not fresh (battery/tilt checks inactive)")
        return d
