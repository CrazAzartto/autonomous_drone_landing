"""
SYNTHETIC end-to-end tests: the complete pipeline (features -> IBVS -> eye-to-hand -> body frame ->
RC mapping -> safety) flying the simulated plant. These check that every sign and frame transformation
works together for different start positions and headings. They say nothing about the real Pluto's
performance (plant parameters are assumptions).
"""
import copy
import math

import numpy as np
import pytest
import yaml

from common.config import load_config, repo_path
from ibvs.runtime import ControlPipeline
from identify_rc_response import identify_throttle, identify_tilt_axis
from pluto_interface.command_mapper import RCCommand
from sim.pluto_sim import PlutoSimPlant, SimAdapter, SyntheticCamera

K = np.array([[950.0, 0, 640.0], [0, 950.0, 360.0], [0, 0, 1.0]])
DIST = np.zeros(5)


def configs():
    cam = load_config(None)
    cfg = yaml.safe_load(open(repo_path("config/ibvs_config.yaml"), encoding="utf-8"))
    for stage in cfg["ibvs"]["stages"]:
        stage["desired_source"] = "model"
    return cam, cfg


def fly(initial, yaw_deg, duration=30.0, stages_thr=None, seed=3):
    cam, cfg = configs()
    sim = copy.deepcopy(cfg["simulation"])
    sim.update({"initial_position_m": list(initial), "initial_yaw_deg": yaw_deg, "seed": seed})
    if stages_thr:
        for stage, thr in zip(cfg["ibvs"]["stages"], stages_thr):
            stage["error_threshold"] = thr
    plant = PlutoSimPlant(sim)
    camera = SyntheticCamera(sim, K, DIST, (1280, 720), float(cam["aruco"]["marker_length_mm"]) / 1000)
    adapter = SimAdapter(plant, clock=lambda: plant.t)
    pipe = ControlPipeline(cfg, cam, K, DIST, adapter, None, repo_path, mode="sim")
    t, i = 0.0, 0
    pipe.event = lambda *a: None
    while t < duration:
        t += 1 / 30
        i += 1
        plant.advance_to(t)
        info = pipe.step(camera.observe(plant, t, i), t)
        if pipe.state == "MANUAL" and t > 0.5 and info["detected"] and not pipe.controller.finished:
            pipe.request_engage(t)
    return pipe, plant


@pytest.mark.parametrize("initial,yaw", [((0.25, -0.15, 1.1), 70), ((-0.2, 0.1, 1.0), 200), ((0.1, 0.2, 0.6), -35)])
def test_stage1_converges_from_different_poses_and_headings(initial, yaw):
    pipe, plant = fly(initial, yaw)
    assert pipe.stage_results and pipe.stage_results[0]["converged"], pipe.stage_results
    assert pipe.stage_results[0]["time_s"] < 20
    # the drone really moved towards the optical axis (truth, not the estimate)
    assert math.hypot(plant.pos[0], plant.pos[1]) < math.hypot(initial[0], initial[1]) or \
        math.hypot(initial[0], initial[1]) < 0.12


def test_full_sequence_lands_with_hardware_threshold():
    pipe, plant = fly((0.2, -0.1, 1.0), 30, duration=60, stages_thr=(0.10, 0.06))
    assert [r["converged"] for r in pipe.stage_results] == [True, True]
    assert pipe.state == "LANDING" and not plant.flying
    assert math.hypot(plant.pos[0], plant.pos[1]) < 0.15


def test_identification_recovers_known_plant():
    """Open-loop RC pulses on the plant -> the identification script must recover gain, sign and delay."""
    cam, cfg = configs()
    sim = copy.deepcopy(cfg["simulation"])
    sim.update({"gust_accel_std_mps2": 0.0, "disturbance_accel_mps2": [0, 0], "baro_drift_std_mps": 0.0,
                "initial_yaw_deg": 0.0, "drag_per_s": 0.4})
    plant = PlutoSimPlant(sim)
    rng = np.random.default_rng(0)
    t_log, rc_p, v_f, rc_t, v_u = [], [], [], [], []
    t, rc = 0.0, RCCommand()
    while t < 60:
        if int(t * 30) % 45 == 0:                       # new random pulse every 1.5 s
            rc = RCCommand(pitch=int(1500 + rng.choice([-60, -30, 0, 30, 60])),
                           throttle=int(1500 + rng.choice([-100, -60, -30, 0, 20, 30, 60, 100])))
        plant.command(t, rc)
        t += 1 / 30
        plant.advance_to(t)
        t_log.append(t)
        rc_p.append(rc.pitch)
        rc_t.append(rc.throttle)
        v_f.append(plant.vel[0])                        # yaw 0: forward = camera x
        v_u.append(plant.vel[2])
        if plant.pos[2] < 0.5 or plant.pos[2] > 2.0:
            plant.pos[2] = 1.2
    t_log = np.array(t_log)
    pitch = identify_tilt_axis(t_log, np.array(rc_p, float), np.array(v_f), 1 / 30, 0.8, 20)
    expected_k = 9.81 * math.tan(math.radians(sim["tilt_deg_per_us"]))
    assert pitch["k"] == pytest.approx(expected_k, rel=0.2)
    assert 0.15 <= pitch["delay_s"] <= 0.45                 # 0.20 s transport + 0.12 s attitude lag
    thr = identify_throttle(t_log, np.array(rc_t, float), np.array(v_u), 1 / 30, 0.8)
    assert thr["k_mps_per_us"] == pytest.approx(0.0025, rel=0.25)
    assert 30 <= thr["deadband_us"] <= 55                 # firmware: 40 us


def test_suggest_gains_from_identification():
    from suggest_gains import suggest
    rep = {"pitch": {"k": 0.0113, "delay_s": 0.32, "r2": 0.8, "k_positive": 0.011, "k_negative": 0.012},
           "roll": {"k": -0.0105, "delay_s": 0.30, "r2": 0.7},
           "throttle": {"k_mps_per_us": 0.0025, "deadband_us": 40, "r2": 0.9},
           "yaw": {"k_radps_per_us": -0.004, "r2": 0.8}}
    g, notes = suggest([rep, rep, rep])
    w_c = math.radians(45) / (0.32 * 1.3)
    assert g["kv_us_per_mps"] == pytest.approx(w_c / 0.0113)
    assert g["ki_us_per_m"] == pytest.approx(g["kv_us_per_mps"] * w_c / 3)
    assert g["lambda_scale"] == pytest.approx(w_c / 3)
    assert (g["sign_pitch"], g["sign_roll"], g["sign_yaw"]) == (1, -1, -1)
    assert g["throttle_us_per_mps"] == pytest.approx(400) and g["yaw_us_per_radps"] == pytest.approx(250)
    assert not any("WARNING" in n for n in notes)
    bad = {"pitch": {"k": -0.0113, "delay_s": 0.3, "r2": 0.8}, "roll": {"k": -0.01, "delay_s": 0.3, "r2": 0.8}}
    _, notes = suggest([bad])
    assert any("pitch sign is NOT +1" in n for n in notes)
    with pytest.raises(ValueError):
        suggest([{"pitch": {"k": 0.01, "delay_s": 0.3, "r2": 0.2}, "roll": {"k": -0.01, "delay_s": 0.3, "r2": 0.9}}])
