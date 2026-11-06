"""
Column names of ibvs_log.csv (one row per control cycle). Units: metres, seconds, radians unless the
name says otherwise. 'cmd' = commanded (set-point), 'est' = estimated from the camera, 'tel' = reported
by the drone (telemetry). Empty cell = not available in that cycle.
"""

COLUMNS = (
    # time and state
    ["t_s", "frame_index", "frame_age_s", "loop_dt_s", "mode", "controller_state", "safety_level",
     "safety_reasons", "stage_index", "stage_name", "ibvs_active",
     # detection
     "detected", "marker_id", "lost_time_s", "n_markers"]
    + [f"corner{i}_{a}_px" for i in range(4) for a in ("u", "v")]
    # image moments of the 4 normalised corners ([P1] Eq.(2))
    + ["m00", "m10", "m01", "mu20", "mu02", "mu11", "x_g", "y_g", "a"]
    # features, desired features, error ([P1] p.4-5)
    + ["s_xn", "s_yn", "s_an", "s_alpha", "sd_xn", "sd_yn", "sd_an", "sd_alpha", "a_star", "z_star",
       "e_xn", "e_yn", "e_an", "e_alpha", "e_norm", "e_norm_control", "e_norm_max", "lambda",
       "below_threshold", "stage_time_s"]
    # interaction matrix, estimate and diagnostics ([P1] Eq.(10), p.5)
    + [f"L_{r}{c}" for r in range(4) for c in range(4)]
    + [f"Lpinv_{r}{c}" for r in range(4) for c in range(4)]
    + ["L_condition", "L_det", "Lpinv_condition"]
    # velocities
    + ["vc_x", "vc_y", "vc_z", "vc_wz",                       # [P1] Eq.(1) camera twist
       "um_x", "um_y", "um_z", "um_wz",                       # marker/drone twist in camera frame (eye-to-hand)
       "heading_img_rad",                                     # psi from alpha
       "cmd_v_forward", "cmd_v_left", "cmd_v_up", "cmd_yaw_rate",
       "est_v_forward", "est_v_left", "est_v_up", "est_yaw_rate",
       "est_vx_cam", "est_vy_cam", "est_vz_cam"]
    # PnP pose (validation / plots only - not used by the controller)
    + ["pnp_x", "pnp_y", "pnp_z", "pnp_heading_rad", "pnp_tilt_deg", "pnp_reproj_px"]
    # RC
    + ["rc_roll", "rc_pitch", "rc_throttle", "rc_yaw", "rc_source",
       "sat_forward", "sat_left", "sat_up", "sat_yaw", "slew_pitch", "slew_roll", "slew_throttle", "slew_yaw",
       "z_in_deadband", "int_forward", "int_left"]
    # drone link and telemetry
    + ["link_ok", "tel_age_s", "tel_roll_deg", "tel_pitch_deg", "tel_heading_deg", "tel_baro_alt_cm",
       "tel_vario_cmps", "tel_vbat_v", "rc_echo_latency_s",
       # simulator ground truth (SYNTHETIC runs only)
       "true_x", "true_y", "true_z", "true_yaw_rad", "true_vx", "true_vy", "true_vz"]
)
