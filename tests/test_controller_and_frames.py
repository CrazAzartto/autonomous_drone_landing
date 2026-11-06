"""Control law (Eq. 1), adaptive gain, stages (Algorithm 1), eye-to-hand conversion and the sign chain."""
import math

import numpy as np
import pytest
import yaml

from helpers import marker_corners_cam, project
from ibvs.controller import AdaptiveGain, IBVSController, StageConfig
from ibvs.features import desired_from_model, features_from_points
from ibvs.frames import (BodyVelocity, camera_twist_to_marker_twist, camera_xy_to_body, heading_from_alpha,
                         marker_twist_to_body)
from ibvs.interaction import interaction_estimate_pinv, interaction_matrix
from pluto_interface.command_mapper import VelocityToRCMapper

L = 0.04


def stage(thr=0.01, dwell=0.0, timeout=0.0, lam=(0.5, 1.0), z=0.8):
    return StageConfig("s", 0, L, desired_from_model(L, z), thr, lambda_max=lam[1], lambda_min=lam[0],
                       dwell_s=dwell, timeout_s=timeout)


def test_adaptive_gain_formula():
    g = AdaptiveGain(0.5, 1.0)
    assert g(0.4) == pytest.approx(1.0)          # first value defines ||e_max||
    assert g(0.2) == pytest.approx(0.75)         # (1-0.5)*0.5 + 0.5
    assert g(0.0) == pytest.approx(0.5)
    assert AdaptiveGain(0.5, 1.0, scale=0.4)(1.0) == pytest.approx(0.4)


def test_control_law_eq1():
    c = IBVSController([stage()])
    s = np.array([0.1, -0.05, 0.9, 0.2])
    out = c.compute(s, 0.0)
    e = s - np.array([0, 0, 0.8, 0])
    assert out.v_c == pytest.approx(-out.lam * interaction_estimate_pinv(s, [0, 0, 0.8, 0]) @ e)
    assert out.v_c.shape == (4,) and out.L.shape == (4, 4) and out.L_hat_pinv.shape == (4, 4)


@pytest.mark.parametrize("eye_to_hand", [False, True])
def test_ideal_closed_loop_converges_exponentially(eye_to_hand):
    """Integrate s_dot = L v_c (paper, eye-in-hand) and s_dot = u_m = L v_c (ours): identical decay."""
    c = IBVSController([stage(thr=1e-4, lam=(0.5, 0.5))])
    s = np.array([0.2, -0.1, 1.1, 0.5])
    dt = 0.01
    norms = []
    for k in range(600):
        out = c.compute(s, k * dt)
        s_dot = camera_twist_to_marker_twist(out.v_c, s) if eye_to_hand else interaction_matrix(s) @ out.v_c
        s = s + dt * s_dot
        norms.append(out.error_norm)
    assert norms[-1] < 0.06 * norms[0]                       # exp(-0.5*6) = 0.05
    assert all(b <= a + 1e-12 for a, b in zip(norms, norms[1:]))   # monotone


def test_stage_sequence_and_dwell():
    c = IBVSController([stage(thr=0.05, dwell=0.2), stage(thr=0.05, z=0.5)])
    c.engage(0.0)
    s_good = np.array([0.0, 0.0, 0.8, 0.0])
    assert not c.compute(s_good, 0.0).stage_converged
    assert not c.compute(s_good, 0.1).stage_converged
    assert c.compute(s_good, 0.21).stage_converged
    assert c.advance_stage(0.21) and c.stage_index == 1
    assert not c.advance_stage(0.3) and c.finished


def test_timeout_reported_once():
    c = IBVSController([stage(thr=1e-6, timeout=1.0)])
    c.engage(0.0)
    flags = [c.compute(np.array([0.1, 0, 0.8, 0]), t).timed_out for t in np.arange(0, 3, 0.1)]
    assert sum(flags) == 1


def test_norm_mask_excludes_alpha_from_stopping_only():
    c = IBVSController([stage(thr=0.05)], norm_mask=(1, 1, 1, 0))
    c.engage(0.0)
    out = c.compute(np.array([0, 0, 0.8, 1.0]), 0.0)
    assert out.control_norm == pytest.approx(0.0) and out.error_norm == pytest.approx(1.0)
    assert out.stage_converged and abs(out.v_c[3]) > 0


def test_eye_to_hand_matches_rigid_body_relation():
    s = np.array([0.15, -0.08, 0.9, 0.3])
    v_c = np.array([0.02, -0.01, 0.03, 0.4])
    u = camera_twist_to_marker_twist(v_c, s)
    w_m = -v_c[3]
    v_m = -v_c[:3] + np.cross([0, 0, w_m], s[:3])
    assert u[:3] == pytest.approx(v_m) and u[3] == pytest.approx(w_m)


@pytest.mark.parametrize("psi", [0.0, 1.2, -2.7])
def test_heading_from_alpha(psi):
    alpha = psi + math.pi / 2
    assert heading_from_alpha(alpha) == pytest.approx(psi)
    fwd, left = camera_xy_to_body([math.cos(psi), math.sin(psi)], psi)
    assert (fwd, left) == pytest.approx((1.0, 0.0))
    fwd, left = camera_xy_to_body([-math.sin(psi), math.cos(psi)], psi)
    assert (fwd, left) == pytest.approx((0.0, 1.0))


def _rc_for_offset(offset_body, height_error=0.0, yaw_offset=0.0, psi=0.7):
    """Drone displaced from the target by offset_body = (forward, left) in ITS frame -> RC command."""
    cfg = yaml.safe_load(open("config/ibvs_config.yaml", encoding="utf-8"))["command_interface"]
    cfg["slew_us_per_s"] = {k: 0 for k in cfg["slew_us_per_s"]}
    cfg["yaw"]["enabled"] = True
    cfg["yaw"]["us_per_radps"] = 100.0
    c, s_ = math.cos(psi), math.sin(psi)
    target = np.array([0.0, 0.0, 0.8])
    pos = target + np.array([c * offset_body[0] - s_ * offset_body[1], s_ * offset_body[0] + c * offset_body[1],
                             height_error])
    desired = desired_from_model(L, 0.8, alpha_star=psi + math.pi / 2)
    s = features_from_points(project(marker_corners_cam(pos, psi + math.pi / 2 + yaw_offset, L)), desired).vector()
    ctrl = IBVSController([StageConfig("s", 0, L, desired, 0.001)])
    out = ctrl.compute(s, 0.0)
    body = marker_twist_to_body(camera_twist_to_marker_twist(out.v_c, s), heading_from_alpha(s[3]))
    return VelocityToRCMapper(cfg).map(body, BodyVelocity(0, 0, 0, 0), 1 / 30)


def test_sign_chain_drone_behind_target_pitches_forward():
    rc = _rc_for_offset((-0.2, 0.0))                       # drone 20 cm BEHIND the target
    assert rc.pitch > 1500 and abs(rc.roll - 1500) <= 1


def test_sign_chain_drone_right_of_target_rolls_left():
    rc = _rc_for_offset((0.0, -0.2))                       # drone 20 cm to the RIGHT of the target
    assert rc.roll < 1500 and abs(rc.pitch - 1500) <= 1


def test_sign_chain_drone_too_low_climbs():
    assert _rc_for_offset((0, 0), height_error=-0.2).throttle > 1500
    assert _rc_for_offset((0, 0), height_error=+0.2).throttle < 1500


def test_sign_chain_drone_turned_right_yaws_left():
    rc = _rc_for_offset((0, 0), yaw_offset=-0.3)          # nose 17 deg to the right of the desired heading
    assert rc.yaw < 1500
