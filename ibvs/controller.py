"""
IBVS controller of [P1] (pure computation, no I/O).

    Control law      v_c = -lambda * L_hat^+ (s - s*)                              [P1] Eq.(1), p.3
    Estimate         L_hat^+ = 1/2 (L_s(s)^+ + L_s(s*)^+)                          [P1] p.5
    Adaptive gain    lambda = (lambda_max - lambda_min) * ||e|| / ||e_max|| + lambda_min   [P1] p.5
                     ||e_max|| = largest error norm since the stage (IBVS block) was initialised
    Stopping         a stage ends when ||e|| < error threshold                       [P1] Alg. 1, p.7
                     ([P1] used 0.1 for marker 1 at h = 0.6 m and 0.015 for marker 2 at h = 0.25 m, p.5)
    Sequence         stage 1 (coarse) -> stage 2 (fine) -> final action ("turn off motors" in [P1])

Hardware additions (documented in docs/06 §6, all configurable, all logged):
  * lambda_scale   : multiplies lambda_min and lambda_max (latency rule lambda*tau < 1/e, docs/07 §5).
                     lambda_scale = 1 reproduces [P1] exactly.
  * dwell_s        : ||e|| must stay below the threshold for dwell_s seconds (camera noise would otherwise
                     end a stage on a single lucky frame). dwell_s = 0 reproduces [P1].
  * timeout_s      : a stage that does not converge in time is reported (the runner then HOLDS).
  * norm_mask      : components included in the convergence norm. When the yaw axis is not actuated, the
                     orientation error cannot be reduced, so it is excluded from the STOPPING test only
                     (it is still part of e and of the control law). All-ones reproduces [P1].
"""

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

from ibvs.features import DesiredFeatures, feature_error
from ibvs.interaction import interaction_estimate_pinv, interaction_matrix, matrix_diagnostics


# ============================================================
# CONFIGURATION OBJECTS
# ============================================================

@dataclass
class StageConfig:
    name: str
    marker_id: int
    marker_length_m: float
    desired: DesiredFeatures
    error_threshold: float              # [P1]: 0.1 (stage 1), 0.015 (stage 2)
    lambda_max: float = 1.0             # [P1] p.6: lambda_max = 1.0
    lambda_min: float = 0.5             # [P1] p.6: lambda_min = 0.5
    dwell_s: float = 0.0
    timeout_s: float = 0.0              # 0 = no timeout
    alpha_mode: str = "desired"         # "desired" | "hold_at_engage"


@dataclass
class IBVSOutput:
    t: float
    stage_index: int
    stage_name: str
    s: np.ndarray
    s_star: np.ndarray
    e: np.ndarray
    error_norm: float                   # ||e|| over all 4 components (as in [P1])
    control_norm: float                 # ||e|| over norm_mask (used for the stopping test)
    error_norm_max: float
    lam: float
    L: np.ndarray
    L_hat_pinv: np.ndarray
    v_c: np.ndarray                     # [vx, vy, vz, wz] camera twist of Eq.(1)
    condition: float
    below_threshold: bool
    stage_converged: bool               # this call completed the stage
    finished: bool                      # all stages completed
    timed_out: bool                     # True only in the cycle in which the stage timeout expired
    stage_time_s: float
    diagnostics: dict = field(default_factory=dict)


# ============================================================
# ADAPTIVE GAIN
# ============================================================

class AdaptiveGain:
    """lambda = (lambda_max - lambda_min) * ||e|| / ||e_max|| + lambda_min   ([P1] p.5)."""

    def __init__(self, lambda_min, lambda_max, scale=1.0):
        if lambda_min < 0 or lambda_max < lambda_min:
            raise ValueError("need 0 <= lambda_min <= lambda_max")
        self.lambda_min = float(lambda_min) * scale
        self.lambda_max = float(lambda_max) * scale
        self.error_norm_max = 0.0

    def reset(self):
        self.error_norm_max = 0.0

    def __call__(self, error_norm):
        self.error_norm_max = max(self.error_norm_max, float(error_norm))
        if self.error_norm_max <= 0.0:
            return self.lambda_min
        ratio = min(1.0, float(error_norm) / self.error_norm_max)
        return (self.lambda_max - self.lambda_min) * ratio + self.lambda_min


# ============================================================
# CONTROLLER
# ============================================================

class IBVSController:

    def __init__(self, stages: List[StageConfig], lambda_scale=1.0, estimate="mean_pinv",
                 norm_mask=(1, 1, 1, 1)):
        if not stages:
            raise ValueError("at least one stage is required")
        self.stages = stages
        self.lambda_scale = float(lambda_scale)
        self.estimate = estimate
        self.norm_mask = np.asarray(norm_mask, dtype=np.float64)
        self.stage_index = 0
        self.finished = False
        self.active = False
        self._gain = None
        self._stage_start = None
        self._below_since = None
        self._timed_out = False

    # ---------------- state ----------------
    @property
    def stage(self) -> StageConfig:
        return self.stages[min(self.stage_index, len(self.stages) - 1)]

    def engage(self, t, s=None):
        """Initialise the IBVS block for the current stage ([P1] Alg. 1: 'the IBVS module will initialize')."""
        stage = self.stage
        self._gain = AdaptiveGain(stage.lambda_min, stage.lambda_max, self.lambda_scale)
        self._stage_start = t
        self._below_since = None
        self._timed_out = False
        self.active = True
        if stage.alpha_mode == "hold_at_engage" and s is not None and math.isfinite(s[3]):
            stage.desired.alpha_star = float(s[3])

    def disengage(self):
        self.active = False
        self._below_since = None

    def reset(self):
        self.stage_index = 0
        self.finished = False
        self.disengage()

    def advance_stage(self, t, s=None):
        """Go to the next stage and re-initialise the IBVS block. Returns True if a stage remains."""
        if self.stage_index + 1 >= len(self.stages):
            self.finished = True
            self.active = False
            return False
        self.stage_index += 1
        self.engage(t, s)
        return True

    # ---------------- control law ----------------
    def compute(self, s, t):
        """One control step. s = [x_n, y_n, a_n, alpha] of the CURRENT stage's marker."""
        if self._gain is None:
            self.engage(t, s)
        stage = self.stage
        s = np.asarray(s, dtype=np.float64)
        s_star = stage.desired.vector()

        e = feature_error(s, s_star)                                   # (4,)
        error_norm = float(np.linalg.norm(e))
        control_norm = float(np.linalg.norm(e * self.norm_mask))
        lam = self._gain(error_norm)

        L = interaction_matrix(s)                                       # (4, 4)
        L_hat_pinv = interaction_estimate_pinv(s, s_star, self.estimate)  # (4, 4)
        v_c = -lam * (L_hat_pinv @ e)                                   # (4,)  Eq.(1)

        # stopping test (Algorithm 1) with optional dwell time
        below = control_norm < stage.error_threshold
        converged = False
        if below and self.active:
            if self._below_since is None:
                self._below_since = t
            if t - self._below_since >= stage.dwell_s:
                converged = True
        else:
            self._below_since = None

        stage_time = 0.0 if self._stage_start is None else t - self._stage_start
        newly_timed_out = False
        if (self.active and stage.timeout_s > 0 and stage_time > stage.timeout_s and not converged
                and not self._timed_out):
            self._timed_out = True               # reported once per stage
            newly_timed_out = True

        diag = matrix_diagnostics(L)
        out = IBVSOutput(
            t=t, stage_index=self.stage_index, stage_name=stage.name, s=s, s_star=s_star, e=e,
            error_norm=error_norm, control_norm=control_norm, error_norm_max=self._gain.error_norm_max,
            lam=lam, L=L, L_hat_pinv=L_hat_pinv, v_c=v_c, condition=diag["condition"],
            below_threshold=below, stage_converged=converged, finished=self.finished,
            timed_out=newly_timed_out, stage_time_s=stage_time,
            diagnostics={"rank": diag["rank"], "det": diag["det"],
                         "singular_values": diag["singular_values"],
                         "pinv_condition": float(np.linalg.cond(L_hat_pinv))},
        )
        return out
