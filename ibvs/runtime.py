"""
One control cycle, identical in every mode (camera / compute / replay / sim / live / bench).

    Observation (corners of the stage marker, capture time)
      -> features s, moments            ibvs/features.py          [P1] Eq.(2), p.4-5
      -> velocity estimate (Kalman)     vision/kalman.py (reused) on [x_n, y_n, a_n] = marker position
      -> v_c = -lambda L_hat^+ e        ibvs/controller.py        [P1] Eq.(1), (10), p.5
      -> u_m = L_s(s) v_c               ibvs/frames.py            eye-to-hand adaptation (docs/06 §4)
      -> body command (fwd, left, up, yaw rate) via heading psi = alpha - 90 deg, limited
      -> safety decision                safety/supervisor.py
      -> RC (roll, pitch, throttle, yaw) pluto_interface/command_mapper.py   [P3]-style loop, [FW] maps
      -> adapter.send_rc (dry run / Pluto / simulator)
      -> one CSV row                    runlog/

Controller states
  MANUAL   operator in charge: manual-key RC, or neutral sticks when no key is pressed (live: only when armed)
  IBVS     autonomous; a safety HOLD in a cycle sends neutral for that cycle and IBVS resumes when it clears
  HOLD     autonomous paused by the operator (H) or by a stage timeout: neutral sticks
  LANDING  firmware LAND requested: neutral sticks, the firmware descends and disarms
  KILLED   disarmed by the operator (K) or by final_action "disarm"; nothing is sent
  DONE     all stages converged and final_action "hold" was chosen
"""

import math

import numpy as np

from common.aruco_utils import estimate_marker_pose
from ibvs.controller import IBVSController, StageConfig
from ibvs.features import (DesiredFeatures, compute_features, desired_from_model, teach_desired,
                           wrap_angle)
from ibvs.frames import BodyVelocity, camera_twist_to_marker_twist, camera_velocity_to_body, \
    heading_from_alpha, marker_twist_to_body
from pluto_interface.command_mapper import RCCommand, VelocityToRCMapper
from safety.supervisor import HOLD, LAND, SafetyInputs, SafetySupervisor
from vision.kalman import ConstantVelocityKalman3D
from vision.target import heading_in_camera

import cv2

NAN = float("nan")


# ============================================================
# BUILDING THE STAGES FROM THE CONFIG
# ============================================================

def build_stages(ibvs_cfg, camera_cfg, repo_path):
    default_length_m = float(camera_cfg["aruco"]["marker_length_mm"]) / 1000.0
    stages = []
    for raw in ibvs_cfg["ibvs"]["stages"]:
        length_m = default_length_m if raw.get("marker_length_mm") in (None, "null") \
            else float(raw["marker_length_mm"]) / 1000.0
        if raw["desired_source"] == "taught":
            desired = DesiredFeatures.load(repo_path(raw["taught_file"]))
        elif raw["desired_source"] == "model":
            desired = desired_from_model(length_m, float(raw["z_star_m"]), float(raw["x_g_star"]),
                                         float(raw["y_g_star"]), math.radians(float(raw["alpha_star_deg"])),
                                         note=f"model a* = 2L^2/z*^2, L = {length_m * 1000:.1f} mm")
        else:
            raise ValueError(f"stage {raw['name']}: desired_source must be 'model' or 'taught'")
        stages.append(StageConfig(
            name=str(raw["name"]), marker_id=int(raw["marker_id"]), marker_length_m=length_m, desired=desired,
            error_threshold=float(raw["error_threshold"]), lambda_max=float(raw["lambda_max"]),
            lambda_min=float(raw["lambda_min"]), dwell_s=float(raw["dwell_s"]), timeout_s=float(raw["timeout_s"]),
            alpha_mode=str(raw["alpha_mode"])))
    return stages


def pnp_tilt_deg(rotation):
    """Angle between the marker normal and the optical axis (0 = marker parallel to the image plane)."""
    return math.degrees(math.acos(min(1.0, abs(float(rotation[2, 2])))))


def limit_body_velocity(cmd: BodyVelocity, limits):
    """Clip the IBVS set-point. The horizontal vector is SCALED (direction kept), not clipped per axis."""
    horizontal = math.hypot(cmd.forward, cmd.left)
    scale = 1.0 if horizontal <= limits["max_horizontal_mps"] or horizontal == 0 else limits["max_horizontal_mps"] / horizontal
    up = max(-limits["max_down_mps"], min(limits["max_up_mps"], cmd.up))
    yaw = max(-limits["max_yaw_rate_radps"], min(limits["max_yaw_rate_radps"], cmd.yaw_rate))
    limited = BodyVelocity(cmd.forward * scale, cmd.left * scale, up, yaw)
    return limited, (scale < 1.0 or up != cmd.up or yaw != cmd.yaw_rate)


# ============================================================
# PIPELINE
# ============================================================

class ControlPipeline:

    def __init__(self, ibvs_cfg, camera_cfg, camera_matrix, dist_coeffs, adapter, logger, repo_path,
                 mode="compute", live=False, bench=False, allow_engage=True):
        self.cfg = ibvs_cfg
        self.K = camera_matrix
        self.dist = dist_coeffs
        self.adapter = adapter
        self.logger = logger
        self.mode = mode
        self.live = live
        self.bench = bench
        self.allow_engage = allow_engage
        self.repo_path = repo_path
        self.alpha_def = ibvs_cfg["ibvs"]["alpha_definition"]
        self.yaw_enabled = bool(ibvs_cfg["command_interface"]["yaw"]["enabled"])

        self.stages = build_stages(ibvs_cfg, camera_cfg, repo_path)
        norm_mask = (1, 1, 1, 1) if self.yaw_enabled else (1, 1, 1, 0)
        self.controller = IBVSController(self.stages, float(ibvs_cfg["ibvs"]["lambda_scale"]),
                                         ibvs_cfg["ibvs"]["interaction_estimate"], norm_mask)
        ve = ibvs_cfg["velocity_estimation"]
        self.kf = ConstantVelocityKalman3D(ve["process_accel_std"], ve["measurement_std_xy"], ve["measurement_std_z"])
        self.yaw_smoothing = float(ve["yaw_rate_smoothing"])
        self.mapper = VelocityToRCMapper(ibvs_cfg["command_interface"])
        self.limits = ibvs_cfg["body_velocity_limits"]
        self.supervisor = SafetySupervisor(ibvs_cfg["safety"])
        self.manual_cfg = ibvs_cfg["manual"]

        self.state = "MANUAL"
        self.manual_rc = None
        self.takeoff_until = None
        self.last_detection_t = None
        self.last_frame_index = None
        self.last_step_t = None
        self.last_alpha = None
        self.last_alpha_t = None
        self.yaw_rate = 0.0
        self.psi = NAN
        self.teach_buffer = None
        self.last_message = ""
        self.last = {}                      # last cycle results for the UI
        self.stage_results = []             # convergence summary per stage

    # ------------------------------------------------------------
    def event(self, t, kind, message):
        self.last_message = f"{kind}: {message}"
        print(f"[{t:8.2f} s] {kind}: {message}")
        if self.logger is not None:
            self.logger.event(t, kind, message)

    # ---------------- operator requests ----------------
    def request_arm(self, t):
        self.adapter.arm()
        if self.state in ("LANDING", "KILLED", "DONE"):
            # a new arm starts a new session: back to manual, stages from the beginning
            self.controller.reset()
            self.mapper.reset()
            self.state = "MANUAL"
        self.event(t, "operator", "ARM")

    def request_takeoff(self, t):
        if self.bench:
            self.event(t, "refused", "take-off is disabled in bench mode (propellers off)")
            return
        self.adapter.take_off()
        self.takeoff_until = t + float(self.cfg["pluto"]["takeoff_neutral_after_s"])
        self.state = "MANUAL"
        self.event(t, "operator", "TAKE-OFF (firmware command)")

    def request_engage(self, t):
        if not self.allow_engage:
            self.event(t, "refused", "IBVS cannot be engaged in this mode")
            return
        if self.controller.finished:
            self.event(t, "refused", "all stages already finished (C = restart from stage 1)")
            return
        if self.state in ("LANDING", "KILLED"):
            self.event(t, "refused", f"state is {self.state}")
            return
        if self.live and not self.bench and not self.adapter.flying:
            self.event(t, "refused", "the drone is not flying (take off with T first)")
            return
        s = self.last.get("s")
        if s is None or not self.last.get("detected", False):
            self.event(t, "refused", "marker of the current stage not detected")
            return
        self.controller.engage(t, s)
        self.mapper.reset()
        self.state = "IBVS"
        self.event(t, "ibvs", f"ENGAGED stage {self.controller.stage_index + 1} '{self.controller.stage.name}'")

    def request_hold(self, t, reason="operator"):
        if self.state == "IBVS":
            self.controller.disengage()
            self.mapper.reset()
            self.state = "HOLD"
            self.event(t, "hold", f"HOLD ({reason}) - neutral sticks; I resumes")

    def request_land(self, t, reason="operator"):
        if self.state in ("LANDING", "KILLED"):
            return
        self.controller.disengage()
        self.mapper.reset()
        self.adapter.send_neutral()
        self.adapter.land()
        self.state = "LANDING"
        self.event(t, "land", f"firmware LAND ({reason})")

    def request_kill(self, t, reason="operator"):
        self.controller.disengage()
        self.adapter.disarm()
        self.state = "KILLED"
        self.event(t, "kill", f"DISARM ({reason})")

    def request_next_stage(self, t):
        s = self.last.get("s")
        if self.controller.advance_stage(t, s):
            self.mapper.reset()
            self.event(t, "ibvs", f"operator skipped to stage {self.controller.stage_index + 1}")

    def request_restart(self, t):
        self.controller.reset()
        self.mapper.reset()
        if self.state in ("IBVS", "DONE", "HOLD"):
            self.state = "MANUAL" if not self.live else "HOLD"
        self.event(t, "ibvs", "controller reset to stage 1")

    def start_teach(self, t, frames=30):
        if self.state == "IBVS":
            self.event(t, "refused", "stop IBVS (H) before teaching")
            return
        self.teach_buffer = {"features": [], "z": [], "frames": frames}
        self.event(t, "teach", f"recording {frames} frames for stage '{self.controller.stage.name}' - hold still")

    def set_manual_keys(self, t, keys):
        """keys: dict with booleans forward/backward/left/right/up/down/yaw_left/yaw_right."""
        m = self.manual_cfg
        if not any(keys.values()):
            self.manual_rc = None
            return
        rc = RCCommand(flags={"mode": "manual"})
        if keys.get("forward"):
            rc.pitch = m["pitch_forward"]
        elif keys.get("backward"):
            rc.pitch = m["pitch_backward"]
        if keys.get("left"):
            rc.roll = m["roll_left"]
        elif keys.get("right"):
            rc.roll = m["roll_right"]
        if keys.get("up"):
            rc.throttle = m["throttle_up"]
        elif keys.get("down"):
            rc.throttle = m["throttle_down"]
        if keys.get("yaw_left"):
            rc.yaw = m["yaw_left"]
        elif keys.get("yaw_right"):
            rc.yaw = m["yaw_right"]
        self.manual_rc = rc
        if self.state in ("IBVS", "HOLD", "DONE"):
            self.controller.disengage()
            self.mapper.reset()
            self.state = "MANUAL"
            self.event(t, "override", "operator took over with the manual keys")

    # ------------------------------------------------------------
    # ONE CYCLE
    # ------------------------------------------------------------
    def step(self, obs, now):
        loop_dt = 0.0 if self.last_step_t is None else now - self.last_step_t
        self.last_step_t = now
        stage = self.controller.stage
        stage_index = self.controller.stage_index          # the stage THIS cycle computes (may advance below)
        row = {"t_s": now, "mode": self.mode}

        # ---------- detection of the stage marker ----------
        corners = None if obs is None else obs.markers.get(stage.marker_id)
        frame_age = float("inf") if obs is None else max(0.0, now - obs.t_capture)
        new_frame = obs is not None and obs.frame_index != self.last_frame_index
        if obs is not None:
            self.last_frame_index = obs.frame_index
        features = None
        if corners is not None:
            try:
                features = compute_features(corners, self.K, self.dist, stage.desired, self.alpha_def)
                if not features.is_finite():
                    features = None
            except (ValueError, cv2.error):
                features = None
        detected = features is not None
        s = features.vector() if detected else None

        pnp = None
        if detected:
            pnp = estimate_marker_pose(np.asarray(corners, dtype=np.float32), stage.marker_length_m, self.K, self.dist)

        # ---------- velocity estimate (camera frame) and heading ----------
        if detected and new_frame:
            self.last_detection_t = obs.t_capture
            self.kf.update(obs.t_capture, s[0:3])
            if self.last_alpha is not None and obs.t_capture > self.last_alpha_t:
                rate = wrap_angle(s[3] - self.last_alpha) / (obs.t_capture - self.last_alpha_t)
                self.yaw_rate = (1 - self.yaw_smoothing) * self.yaw_rate + self.yaw_smoothing * rate
            self.last_alpha, self.last_alpha_t = s[3], obs.t_capture
        if detected:
            self.psi = heading_from_alpha(s[3], self.alpha_def)
        lost_time = 0.0 if detected else (float("inf") if self.last_detection_t is None
                                          else (now - self.last_detection_t))
        v_cam = self.kf.velocity if self.kf.initialized else None
        est_body = camera_velocity_to_body(v_cam, self.yaw_rate, self.psi) \
            if (v_cam is not None and math.isfinite(self.psi)) else None

        # ---------- teach (learning step of [P1]) ----------
        if self.teach_buffer is not None and detected and new_frame and pnp is not None:
            self.teach_buffer["features"].append(features)
            self.teach_buffer["z"].append(float(pnp["tvec"][2]))
            if len(self.teach_buffer["features"]) >= self.teach_buffer["frames"]:
                self._finish_teach(now)

        # ---------- IBVS law (computed whenever the marker is seen, so it can be inspected) ----------
        out = None
        cmd_body = None
        cmd_limited = False
        u_m = None
        if detected and self.allow_engage:
            out = self.controller.compute(s, now)
            u_m = camera_twist_to_marker_twist(out.v_c, s)
            raw_cmd = marker_twist_to_body(u_m, self.psi)
            cmd_body, cmd_limited = limit_body_velocity(raw_cmd, self.limits)

        # ---------- safety ----------
        telemetry = self.adapter.telemetry() if (self.live or self.mode == "sim") else {}
        link = self.adapter.link_status()
        decision = self.supervisor.evaluate(SafetyInputs(
            now=now, frame_age_s=frame_age, detected=detected, lost_time_s=lost_time,
            features=s, corners_px=corners, image_size=(obs.image_size if obs is not None else (1280, 720)),
            condition=(out.condition if out is not None else 1.0),
            command=(cmd_body.as_array() if cmd_body is not None else None),
            link_ok=link["ok"], live=self.live, telemetry=telemetry, loop_dt_s=loop_dt))

        # ---------- state machine -> RC ----------
        rc = RCCommand(flags={"mode": "neutral"})
        rc_source = "neutral"
        send = True
        critical = [r for r in decision.reasons if r.startswith("LAND") and ("battery" in r or "link" in r)]

        if self.state == "IBVS":
            if decision.level >= LAND:
                self.request_land(now, "; ".join(decision.reasons))
            elif decision.level >= HOLD or cmd_body is None or est_body is None:
                self.mapper.reset()
                rc_source = "safety_hold"
            else:
                dt = loop_dt if 0 < loop_dt < 0.5 else 1.0 / 30.0
                rc = self.mapper.map(cmd_body, est_body, dt)
                rc_source = "ibvs"
                self._stage_logic(now, out, s)
        elif self.state == "MANUAL":
            if critical and self.adapter.flying:
                self.request_land(now, "; ".join(critical))
            elif self.takeoff_until is not None and now < self.takeoff_until:
                send = False
                rc_source = "takeoff"
            else:
                self.takeoff_until = None
                if self.manual_rc is not None:
                    rc, rc_source = self.manual_rc, "manual"
                if self.live and not self.adapter.armed:
                    send = False
        elif self.state in ("HOLD", "DONE", "LANDING"):
            # paused autonomy: the same LAND rules as in IBVS (marker lost too long, link, battery, too low)
            if decision.level >= LAND and self.state != "LANDING" and (self.adapter.flying or not self.live):
                self.request_land(now, "; ".join(decision.reasons))
            rc_source = self.state.lower()
        elif self.state == "KILLED":
            send = False
            rc_source = "killed"

        if self.state == "LANDING":
            rc = RCCommand(flags={"mode": "landing"})
        if send:
            self.adapter.send_rc(rc)

        # ---------- remember for the UI ----------
        self.last = {"detected": detected, "s": s, "out": out, "cmd": cmd_body, "est": est_body, "rc": rc,
                     "decision": decision, "corners": corners, "pnp": pnp, "u_m": u_m, "lost_time": lost_time,
                     "frame_age": frame_age, "telemetry": telemetry, "link": link, "rc_source": rc_source,
                     "features": features}

        # ---------- log row ----------
        if self.logger is not None:
            self._fill_row(row, obs, now, frame_age, loop_dt, decision, stage, stage_index, detected, lost_time,
                           corners, features, out, u_m, cmd_body, est_body, v_cam, pnp, rc, rc_source, link,
                           telemetry)
            self.logger.row(row)
        return self.last

    # ------------------------------------------------------------
    def _stage_logic(self, now, out, s):
        stage = self.controller.stage
        if out.stage_converged:
            self.stage_results.append({"stage": stage.name, "converged": True, "time_s": out.stage_time_s,
                                       "final_error_norm": out.error_norm, "final_e": out.e.tolist()})
            self.event(now, "converged", f"stage '{stage.name}' converged in {out.stage_time_s:.2f} s, "
                                         f"||e|| = {out.error_norm:.4f} (control norm {out.control_norm:.4f})")
            if self.controller.advance_stage(now, s):
                self.mapper.reset()
                self.event(now, "ibvs", f"stage {self.controller.stage_index + 1} '{self.controller.stage.name}' started")
            else:
                self._final_action(now)
        elif out.timed_out:
            self.stage_results.append({"stage": stage.name, "converged": False, "time_s": out.stage_time_s,
                                       "final_error_norm": out.error_norm, "final_e": out.e.tolist()})
            self.event(now, "timeout", f"stage '{stage.name}' not converged after {out.stage_time_s:.1f} s "
                                       f"(||e|| = {out.error_norm:.4f})")
            action = self.cfg["ibvs"]["on_timeout"]
            if action == "next_stage" and self.controller.advance_stage(now, s):
                self.mapper.reset()
            elif action == "hold":
                self.request_hold(now, "stage timeout")
            elif action == "land":
                self.request_land(now, "stage timeout")
            else:                                   # "continue": keep servoing, the operator decides (L)
                self.event(now, "timeout", "IBVS continues; press L to land or N for the next stage")

    def _final_action(self, now):
        action = self.cfg["ibvs"]["final_action"]
        self.event(now, "finished", f"all stages converged; final action = {action}")
        if not self.cfg["ibvs"]["auto_final_action"]:
            self.state = "DONE"
            self.event(now, "finished", "auto_final_action is false: HOLD - press L to land")
        elif action == "land":
            self.request_land(now, "IBVS finished")
        elif action == "disarm":
            self.request_kill(now, "IBVS finished ([P1]: turn off motors)")
        else:
            self.state = "DONE"

    def _finish_teach(self, now):
        buffer = self.teach_buffer
        self.teach_buffer = None
        stage = self.controller.stage
        z_star = float(np.median(buffer["z"]))
        desired = teach_desired(buffer["features"], z_star, self.alpha_def,
                                note=f"taught over {len(buffer['features'])} frames, z* from solvePnP")
        stage.desired = desired
        raw = next(r for r in self.cfg["ibvs"]["stages"] if r["name"] == stage.name)
        path = self.repo_path(raw["taught_file"])
        desired.save(path)
        self.event(now, "teach", f"stage '{stage.name}': z* = {z_star:.3f} m, a* = {desired.a_star:.5f}, "
                                 f"x_g* = {desired.x_g_star:+.4f}, y_g* = {desired.y_g_star:+.4f}, "
                                 f"alpha* = {math.degrees(desired.alpha_star):+.1f} deg -> saved {path}. "
                                 f"Set desired_source: taught in ibvs_config.yaml to use it next time.")

    # ------------------------------------------------------------
    def _fill_row(self, row, obs, now, frame_age, loop_dt, decision, stage, stage_index, detected, lost_time,
                  corners, features, out, u_m, cmd, est, v_cam, pnp, rc, rc_source, link, telemetry):
        row.update({
            "frame_index": obs.frame_index if obs is not None else "", "frame_age_s": frame_age, "loop_dt_s": loop_dt,
            "controller_state": self.state, "safety_level": decision.name, "safety_reasons": " | ".join(decision.reasons),
            "stage_index": stage_index, "stage_name": stage.name,
            "ibvs_active": rc_source == "ibvs" or self.state == "IBVS", "detected": detected, "marker_id": stage.marker_id,
            "lost_time_s": lost_time if math.isfinite(lost_time) else "",
            "n_markers": len(obs.markers) if obs is not None else 0,
            "a_star": stage.desired.a_star, "z_star": stage.desired.z_star,
        })
        if corners is not None:
            c = np.asarray(corners).reshape(4, 2)
            for i in range(4):
                row[f"corner{i}_u_px"], row[f"corner{i}_v_px"] = c[i, 0], c[i, 1]
        if features is not None:
            m = features.moments
            row.update({"m00": m["m00"], "m10": m["m10"], "m01": m["m01"], "mu20": m["mu20"], "mu02": m["mu02"],
                        "mu11": m["mu11"], "x_g": features.x_g, "y_g": features.y_g, "a": features.a,
                        "s_xn": features.x_n, "s_yn": features.y_n, "s_an": features.a_n, "s_alpha": features.alpha,
                        "heading_img_rad": self.psi})
        sd = stage.desired.vector()
        row.update({"sd_xn": sd[0], "sd_yn": sd[1], "sd_an": sd[2], "sd_alpha": sd[3]})
        if out is not None:
            row.update({"e_xn": out.e[0], "e_yn": out.e[1], "e_an": out.e[2], "e_alpha": out.e[3],
                        "e_norm": out.error_norm, "e_norm_control": out.control_norm, "e_norm_max": out.error_norm_max,
                        "lambda": out.lam, "below_threshold": out.below_threshold, "stage_time_s": out.stage_time_s,
                        "L_condition": out.condition, "L_det": out.diagnostics["det"],
                        "Lpinv_condition": out.diagnostics["pinv_condition"],
                        "vc_x": out.v_c[0], "vc_y": out.v_c[1], "vc_z": out.v_c[2], "vc_wz": out.v_c[3]})
            for r in range(4):
                for k in range(4):
                    row[f"L_{r}{k}"] = out.L[r, k]
                    row[f"Lpinv_{r}{k}"] = out.L_hat_pinv[r, k]
        if u_m is not None:
            row.update({"um_x": u_m[0], "um_y": u_m[1], "um_z": u_m[2], "um_wz": u_m[3]})
        if cmd is not None:
            row.update({"cmd_v_forward": cmd.forward, "cmd_v_left": cmd.left, "cmd_v_up": cmd.up,
                        "cmd_yaw_rate": cmd.yaw_rate})
        if est is not None:
            row.update({"est_v_forward": est.forward, "est_v_left": est.left, "est_v_up": est.up,
                        "est_yaw_rate": est.yaw_rate})
        if v_cam is not None:
            row.update({"est_vx_cam": v_cam[0], "est_vy_cam": v_cam[1], "est_vz_cam": v_cam[2]})
        if pnp is not None:
            rotation, _ = cv2.Rodrigues(pnp["rvec"])
            row.update({"pnp_x": pnp["tvec"][0], "pnp_y": pnp["tvec"][1], "pnp_z": pnp["tvec"][2],
                        "pnp_heading_rad": heading_in_camera(rotation), "pnp_tilt_deg": pnp_tilt_deg(rotation),
                        "pnp_reproj_px": pnp["reproj_error_px"]})
        flags = rc.flags or {}
        integrals = self.mapper.integrals()
        row.update({"rc_roll": rc.roll, "rc_pitch": rc.pitch, "rc_throttle": rc.throttle, "rc_yaw": rc.yaw,
                    "rc_source": rc_source,
                    "sat_forward": flags.get("sat_forward", False), "sat_left": flags.get("sat_left", False),
                    "sat_up": flags.get("sat_up", False), "sat_yaw": flags.get("sat_yaw", False),
                    "slew_pitch": flags.get("slew_pitch", False), "slew_roll": flags.get("slew_roll", False),
                    "slew_throttle": flags.get("slew_throttle", False), "slew_yaw": flags.get("slew_yaw", False),
                    "z_in_deadband": flags.get("z_in_deadband", False),
                    "int_forward": integrals["forward"], "int_left": integrals["left"],
                    "link_ok": link["ok"]})
        if telemetry:
            row.update({"tel_age_s": telemetry.get("age_s", NAN), "tel_roll_deg": telemetry.get("roll_deg", NAN),
                        "tel_pitch_deg": telemetry.get("pitch_deg", NAN),
                        "tel_heading_deg": telemetry.get("heading_deg", NAN),
                        "tel_baro_alt_cm": telemetry.get("baro_alt_cm", NAN),
                        "tel_vario_cmps": telemetry.get("vario_cmps", NAN), "tel_vbat_v": telemetry.get("vbat_v", NAN),
                        "rc_echo_latency_s": telemetry.get("rc_echo_latency_s", NAN)})
        if obs is not None and obs.truth is not None:
            tr = obs.truth
            row.update({"true_x": tr["x"], "true_y": tr["y"], "true_z": tr["z"], "true_yaw_rad": tr["yaw"],
                        "true_vx": tr["vx"], "true_vy": tr["vy"], "true_vz": tr["vz"]})
