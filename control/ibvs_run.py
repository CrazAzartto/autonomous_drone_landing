"""
PROGRAM 6 - ibvs_run.py
Image-moment IBVS of Paper 1 on the Pluto drone (camera on the floor, ArUco marker under the drone).

MODES (nothing is ever sent to a drone unless --mode live is used AND enabled in the config)
    camera   camera + detection + features + solvePnP only. No controller.         (test stage 2)
    compute  camera + full controller; commands are computed, shown, logged, NOT sent (test stage 3)
    replay   a recorded video instead of the camera (offline, test stage 1)       --video file.avi
    sim      SYNTHETIC simulated drone + camera, full closed loop (software test only)
    live     real Pluto over Wi-Fi. Needs safety.live_control_enabled: true in config/ibvs_config.yaml
             and typing FLY at the prompt.  Add --bench for propellers-removed bench tests (take-off
             disabled, IBVS may be engaged on the ground).

Usage (autonomous_drone_landing folder, .venv active):
    python control\\ibvs_run.py --mode camera
    python control\\ibvs_run.py --mode compute
    python control\\ibvs_run.py --mode replay --video results\\recording_123.avi [--headless --auto-engage]
    python control\\ibvs_run.py --mode sim [--headless]
    python control\\ibvs_run.py --mode live --bench
    python control\\ibvs_run.py --mode live

KEYS (click the program window first)
    I  engage IBVS (current stage)        H  hold (pause IBVS, neutral sticks)      N  next stage
    L  firmware LAND                      SPACE  ABORT = neutral + LAND             K  KILL = disarm now
    Y  arm (live)                         T  take off (live)                        C  restart at stage 1
    P  teach desired features (hold the drone at the desired pose; [P1] learning step)
    R  start/stop recording the raw camera video (for replay)
    F  (bench only) freeze the loop for 1 s to test the watchdog
    W/S pitch fwd/back, A/D roll left/right, Q/E up/down, Z/X yaw left/right = MANUAL TAKE-OVER (stops IBVS)
    ESC or B  quit (lands and disarms when flying)

Logs: results/runs/<time>_<mode>/  ->  python analysis/plot_ibvs_run.py <that folder>
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.calibration_io import load_calibration                    # noqa: E402
from common.config import load_config, repo_path                      # noqa: E402
from common.overlay import draw_text                                  # noqa: E402
from ibvs.runtime import ControlPipeline, pnp_tilt_deg                # noqa: E402
from ibvs.sources import CameraSource, ReplaySource                   # noqa: E402
from pluto_interface.adapters import DryRunAdapter, PlutoAdapter      # noqa: E402
from runlog.columns import COLUMNS                                    # noqa: E402
from runlog.run_logger import RunLogger                               # noqa: E402
from safety.watchdog import Watchdog                                  # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="Image-moment IBVS for the Pluto drone (Paper 1).")
parser.add_argument("--mode", required=True, choices=["camera", "compute", "replay", "sim", "live"])
parser.add_argument("--bench", action="store_true", help="live link with propellers REMOVED (no take-off)")
parser.add_argument("--video", default=None, help="video file for --mode replay")
parser.add_argument("--headless", action="store_true", help="no window (replay / sim only)")
parser.add_argument("--auto-engage", action="store_true", help="engage IBVS at the first detection (replay/sim)")
parser.add_argument("--duration", type=float, default=None, help="stop after this many seconds")
parser.add_argument("--run-name", default=None, help="text added to the log folder name")
parser.add_argument("--config", default=None, help="camera_config.yaml")
parser.add_argument("--ibvs-config", default=None, help="ibvs_config.yaml")
args = parser.parse_args()

camera_cfg = load_config(args.config)
with open(args.ibvs_config or repo_path("config/ibvs_config.yaml"), "r", encoding="utf-8") as file:
    ibvs_cfg = yaml.safe_load(file)

if args.headless and args.mode not in ("replay", "sim"):
    sys.exit("--headless is only allowed for replay and sim (a live camera needs the operator window).")
if args.bench and args.mode != "live":
    sys.exit("--bench is used together with --mode live.")
if args.auto_engage and args.mode not in ("replay", "sim"):
    sys.exit("--auto-engage is only allowed in replay and sim modes. Live engagement is always a key press.")
if args.mode == "replay" and not args.video:
    sys.exit("--mode replay needs --video <file>")

DATA_KIND = {"sim": "SYNTHETIC", "replay": "REPLAY"}.get(args.mode, "REAL")


# ============================================================
# LIVE-CONTROL GATE (three independent confirmations)
# ============================================================

def confirm_live():
    if not ibvs_cfg["safety"]["live_control_enabled"]:
        sys.exit("Live control is DISABLED: set safety.live_control_enabled: true in config/ibvs_config.yaml "
                 "after completing the bench tests of docs/08.")
    print("\n" + "=" * 70)
    if args.bench:
        print(" BENCH MODE: the program will CONNECT to the Pluto and SEND RC commands.")
        print(" PROPELLERS MUST BE REMOVED. Take-off is disabled.")
        word = "BENCH"
    else:
        print(" LIVE FLIGHT MODE: the program will CONNECT to the Pluto and SEND RC commands.")
        print(" Nothing happens until YOU press Y (arm), T (take off) and I (engage IBVS).")
        print(" Check: net/clear area, fresh battery, camera fixed, signs verified on the bench,")
        print("        a second person ready, SPACE = abort+land, K = kill (disarm).")
        word = "FLY"
    print("=" * 70)
    answer = input(f"Type {word} to continue: ").strip()
    if answer != word:
        sys.exit("Not confirmed - exiting without connecting.")


if args.mode == "live":
    confirm_live()


# ============================================================
# CALIBRATION
# ============================================================

calibration_path = repo_path(camera_cfg["calibration"]["output_file"])
if args.mode == "sim" and not (ibvs_cfg["simulation"]["use_calibration_file"] and os.path.isfile(calibration_path)):
    K = np.array([[950.0, 0, 640.0], [0, 950.0, 360.0], [0, 0, 1.0]])
    DIST = np.zeros(5)
    calib_note = "default synthetic intrinsics"
else:
    calibration = load_calibration(calibration_path)
    K, DIST = calibration["camera_matrix"], calibration["dist_coeffs"]
    calib_note = f"{calibration_path} (RMS {calibration['rms_error_px']:.3f} px)"
IMAGE_SIZE = (int(camera_cfg["camera"]["width"]), int(camera_cfg["camera"]["height"]))


# ============================================================
# LOGGER, ADAPTER, PIPELINE
# ============================================================

logger = RunLogger(repo_path(ibvs_cfg["logging"]["base_dir"]), args.mode, COLUMNS, DATA_KIND,
                   meta={"ibvs_config": ibvs_cfg, "camera_config": {k: v for k, v in camera_cfg.items()
                                                                    if not k.startswith("_")},
                         "calibration": calib_note, "camera_matrix": K, "dist_coeffs": DIST,
                         "bench": args.bench, "video": args.video, "argv": sys.argv},
                   run_name=args.run_name)
print(f"Logging to {logger.run_dir}  ({DATA_KIND})")

clock_now = time.perf_counter
plant = None
sim_camera = None
if args.mode == "sim":
    from sim.pluto_sim import PlutoSimPlant, SimAdapter, SyntheticCamera
    sim_cfg = ibvs_cfg["simulation"]
    plant = PlutoSimPlant(sim_cfg)
    stage0_length = float(camera_cfg["aruco"]["marker_length_mm"]) / 1000.0
    sim_camera = SyntheticCamera(sim_cfg, K, DIST, IMAGE_SIZE, stage0_length)
    adapter = SimAdapter(plant, clock=lambda: plant.t, event_callback=lambda k, m: logger.event(plant.t, k, m))
elif args.mode == "live":
    library_dir = ibvs_cfg["pluto"]["library_dir"]
    adapter = PlutoAdapter(repo_path(library_dir) if library_dir else None, ibvs_cfg["pluto"]["ip"],
                           int(ibvs_cfg["pluto"]["port"]),
                           event_callback=lambda k, m: logger.event(time.perf_counter(), k, m),
                           read_telemetry=bool(ibvs_cfg["pluto"]["read_telemetry"]))
else:
    adapter = DryRunAdapter()

pipeline = ControlPipeline(ibvs_cfg, camera_cfg, K, DIST, adapter, logger, repo_path, mode=args.mode,
                           live=(args.mode == "live"), bench=args.bench, allow_engage=(args.mode != "camera"))


# ============================================================
# SOURCE
# ============================================================

source = None
if args.mode in ("camera", "compute", "live"):
    source = CameraSource(camera_cfg["camera"], camera_cfg["aruco"]["dictionary"]).start()
elif args.mode == "replay":
    source = ReplaySource(args.video, camera_cfg["aruco"]["dictionary"], keep_frames=not args.headless)

if args.mode == "live":
    if not adapter.connect():
        source.stop()
        logger.close()
        sys.exit("Could not connect to the Pluto (join its Wi-Fi first).")


def emergency_from_watchdog():
    """Runs in the watchdog thread: the control loop has stopped."""
    print("\nWATCHDOG: control loop silent -> neutral sticks + firmware LAND")
    try:
        adapter.send_neutral()
        adapter.land()
    finally:
        logger.event(time.perf_counter(), "watchdog", "control loop silent: neutral + LAND")


watchdog = Watchdog(float(ibvs_cfg["safety"]["watchdog_timeout_s"]), emergency_from_watchdog)


# ============================================================
# OPERATOR WINDOW (pygame: one window = video + status + reliable key states)
# ============================================================

class Console:
    VIDEO_W, VIDEO_H, PANEL_W = 960, 540, 420

    def __init__(self, title):
        import pygame
        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.VIDEO_W + self.PANEL_W, self.VIDEO_H + 150))
        pygame.display.set_caption(title)
        self.font = pygame.font.SysFont("consolas", 16)
        self.big = pygame.font.SysFont("consolas", 22, bold=True)

    def events(self):
        """Returns (list of pressed key names this frame, manual-key state dict, quit flag)."""
        pg = self.pg
        pressed, quit_flag = [], False
        for event in pg.event.get():
            if event.type == pg.QUIT:
                quit_flag = True
            elif event.type == pg.KEYDOWN:
                pressed.append(pg.key.name(event.key))
        keys = pg.key.get_pressed()
        manual = {"forward": keys[pg.K_w], "backward": keys[pg.K_s], "left": keys[pg.K_a], "right": keys[pg.K_d],
                  "up": keys[pg.K_q], "down": keys[pg.K_e], "yaw_left": keys[pg.K_z], "yaw_right": keys[pg.K_x]}
        return pressed, manual, quit_flag

    def draw(self, image_bgr, lines, banner, banner_colour):
        pg = self.pg
        self.screen.fill((20, 20, 20))
        if image_bgr is not None:
            rgb = cv2.cvtColor(cv2.resize(image_bgr, (self.VIDEO_W, self.VIDEO_H)), cv2.COLOR_BGR2RGB)
            self.screen.blit(pg.surfarray.make_surface(np.swapaxes(rgb, 0, 1)), (0, 0))
        self.screen.blit(self.big.render(banner, True, banner_colour), (self.VIDEO_W + 10, 8))
        for i, (text, colour) in enumerate(lines):
            x = self.VIDEO_W + 10 if i < 30 else 10
            y = 40 + 19 * i if i < 30 else self.VIDEO_H + 8 + 19 * (i - 30)
            self.screen.blit(self.font.render(text, True, colour), (x, y))
        pg.display.flip()

    def close(self):
        self.pg.quit()


def overlay(image, info):
    """Draw detected corners, the desired marker outline and the commanded direction on the image."""
    view = image.copy()
    stage = pipeline.controller.stage
    if info.get("corners") is not None:
        c = np.asarray(info["corners"]).reshape(4, 2)
        cv2.polylines(view, [c.astype(np.int32)], True, (0, 255, 0), 2)
        for i, (u, v) in enumerate(c):
            cv2.circle(view, (int(u), int(v)), 4, (255, 255, 0), -1)
            draw_text(view, str(i), (int(u) + 6, int(v) - 6), (255, 255, 0), 0.5, 1)
    # desired marker (s*) drawn as a square of side fx*L/z* at (x_g*, y_g*), orientation alpha*
    d = stage.desired
    center = K @ np.array([d.x_g_star, d.y_g_star, 1.0])
    half = 0.5 * K[0, 0] * math.sqrt(d.a_star / 2.0)
    angle = d.alpha_star
    ex, ey = np.array([math.cos(angle), math.sin(angle)]), np.array([-math.sin(angle), math.cos(angle)])
    pts = [center[:2] + half * (sx * ex + sy * ey) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    cv2.polylines(view, [np.array(pts, dtype=np.int32)], True, (0, 165, 255), 2)
    # commanded marker motion (u_m, camera frame) as an arrow from the marker centre
    if info.get("u_m") is not None and info.get("corners") is not None:
        c = np.asarray(info["corners"]).reshape(4, 2).mean(axis=0)
        z = max(0.2, info["s"][2])
        tip = c + 2.0 * K[0, 0] / z * np.asarray(info["u_m"][0:2])     # 2 s of motion, in pixels
        cv2.arrowedLine(view, (int(c[0]), int(c[1])), (int(tip[0]), int(tip[1])), (0, 0, 255), 3, tipLength=0.25)
    cv2.drawMarker(view, (int(K[0, 2]), int(K[1, 2])), (255, 255, 255), cv2.MARKER_CROSS, 24, 1)
    return view


def status_lines(info):
    white, yellow, green, red, grey = (230, 230, 230), (255, 220, 0), (0, 230, 0), (255, 70, 70), (150, 150, 150)
    stage = pipeline.controller.stage
    d = info.get("decision")
    lines = [(f"state {pipeline.state}   stage {pipeline.controller.stage_index + 1}/{len(pipeline.stages)} "
              f"'{stage.name}'", white),
             (f"safety {d.name if d else '-'}", green if d and d.level == 0 else (yellow if d and d.level == 1 else red))]
    for reason in (d.reasons[:3] if d else []):
        lines.append(("  " + reason[:52], yellow))
    s, out = info.get("s"), info.get("out")
    if s is not None:
        lines.append((f"s  xn {s[0]:+.3f} yn {s[1]:+.3f} an {s[2]:.3f} a {math.degrees(s[3]):+6.1f}d", white))
    else:
        lines.append(("marker of this stage NOT detected", red))
    sd = stage.desired.vector()
    lines.append((f"s* xn {sd[0]:+.3f} yn {sd[1]:+.3f} an {sd[2]:.3f} a {math.degrees(sd[3]):+6.1f}d", grey))
    if out is not None:
        e = out.e
        lines.append((f"e  {e[0]:+.3f} {e[1]:+.3f} {e[2]:+.3f} {math.degrees(e[3]):+6.1f}d", yellow))
        lines.append((f"|e| {out.error_norm:.4f}  ctrl {out.control_norm:.4f}  thr {stage.error_threshold}", yellow))
        lines.append((f"lambda {out.lam:.3f}   cond(L) {out.condition:.2f}", white))
        lines.append((f"v_c  {out.v_c[0]:+.3f} {out.v_c[1]:+.3f} {out.v_c[2]:+.3f} {out.v_c[3]:+.3f}", white))
    cmd, est = info.get("cmd"), info.get("est")
    if cmd is not None:
        lines.append((f"cmd fwd {cmd.forward:+.3f} left {cmd.left:+.3f} up {cmd.up:+.3f} m/s", green))
    if est is not None:
        lines.append((f"est fwd {est.forward:+.3f} left {est.left:+.3f} up {est.up:+.3f} m/s", white))
    pnp = info.get("pnp")
    if pnp is not None:
        x, y, z = pnp["tvec"] * 100
        tilt = pnp_tilt_deg(cv2.Rodrigues(pnp["rvec"])[0])
        lines.append((f"PnP X {x:+6.1f} Y {y:+6.1f} Z {z:6.1f} cm  tilt {tilt:4.1f}d", grey))
    rc = info.get("rc")
    if rc is not None:
        lines.append((f"RC r {rc.roll} p {rc.pitch} t {rc.throttle} y {rc.yaw} ({info.get('rc_source')})",
                      green if info.get("rc_source") == "ibvs" else white))
    link, tel = info.get("link", {}), info.get("telemetry", {})
    if args.mode == "live":
        lines.append((f"link {'OK' if link.get('ok') else 'LOST ' + link.get('reason', '')}  "
                      f"vbat {tel.get('vbat_v', float('nan')):.2f} V  echo {1000 * tel.get('rc_echo_latency_s', float('nan')):.0f} ms",
                      green if link.get("ok") else red))
    lines.append((pipeline.last_message[:56], yellow))
    while len(lines) < 30:
        lines.append(("", white))
    lines += [("I engage  H hold  N next  L land  SPACE abort  K kill  Y arm  T takeoff  C restart", grey),
              ("P teach  R record  W/S A/D Q/E Z/X manual take-over   ESC/B quit", grey),
              (f"mode {args.mode}{' BENCH' if args.bench else ''}  data {DATA_KIND}  log {os.path.basename(logger.run_dir)}", grey)]
    return lines


def handle_keys(pressed, manual, now):
    """Returns True to quit."""
    for name in pressed:
        if name in ("escape", "b"):
            return True
        if name == "i":
            pipeline.request_engage(now)
        elif name == "h":
            pipeline.request_hold(now)
        elif name == "n":
            pipeline.request_next_stage(now)
        elif name == "l":
            pipeline.request_land(now)
        elif name == "space":
            pipeline.request_land(now, "ABORT key")
        elif name == "k":
            pipeline.request_kill(now)
        elif name == "y":
            pipeline.request_arm(now)
        elif name == "t":
            pipeline.request_takeoff(now)
        elif name == "c":
            pipeline.request_restart(now)
        elif name == "p":
            pipeline.start_teach(now)
        elif name == "r":
            toggle_recording(now)
        elif name == "f" and args.bench:
            # BENCH TEST of the watchdog (docs/08 D4): freeze this loop for 1 s; the watchdog thread must
            # send neutral + LAND after watchdog_timeout_s.
            pipeline.event(now, "test", "freezing the control loop for 1.0 s (watchdog test)")
            time.sleep(1.0)
    if args.mode in ("live", "compute", "camera"):
        pipeline.set_manual_keys(now, manual)
    return False


recorder = {"writer": None, "last_index": -1}


def toggle_recording(now):
    if not isinstance(source, CameraSource):
        pipeline.event(now, "refused", "recording is only for the live camera")
        return
    if recorder["writer"] is None:
        path = os.path.join(logger.run_dir, f"raw_{int(time.time())}.avi")
        recorder["writer"] = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), float(camera_cfg["camera"]["fps"]),
                                             IMAGE_SIZE)
        pipeline.event(now, "record", f"recording raw video to {path}")
    else:
        recorder["writer"].release()
        recorder["writer"] = None
        pipeline.event(now, "record", "recording stopped")


# ============================================================
# SHUTDOWN (always runs)
# ============================================================

def shutdown():
    print("\nStopping ...")
    watchdog.disarm()
    try:
        if args.mode == "live":
            if adapter.flying and pipeline.state != "KILLED":
                adapter.send_neutral()
                adapter.land()
                print("Firmware LAND sent; waiting for touchdown ...")
                deadline = time.perf_counter() + float(ibvs_cfg["safety"]["land_wait_s"])
                while time.perf_counter() < deadline:
                    time.sleep(0.1)
            if adapter.armed:
                print("Disarming ...")
                adapter.disarm()
            adapter.close()
    finally:
        watchdog.stop()
        if source is not None:
            source.stop()
        if recorder["writer"] is not None:
            recorder["writer"].release()
        logger.close()
        print(f"Log folder: {logger.run_dir}")
        print(f"Plots:      python analysis/plot_ibvs_run.py \"{logger.run_dir}\"")


# ============================================================
# MAIN LOOPS
# ============================================================

console = None if args.headless else Console(f"IBVS - {args.mode}{' BENCH' if args.bench else ''} - {DATA_KIND}")
print_interval = 1.0 / float(ibvs_cfg["logging"]["print_rate_hz"])
last_print = -1e9
quit_requested = False

try:
    # ---------------- simulation: simulated time, one cycle per synthetic frame ----------------
    if args.mode == "sim":
        sim_cfg = ibvs_cfg["simulation"]
        dt = 1.0 / float(sim_cfg["fps"])
        duration = args.duration or float(sim_cfg["duration_s"])
        engage_at = float(sim_cfg["auto_engage_after_s"])
        index, t = 0, 0.0
        while t < duration and not quit_requested:
            t += dt
            index += 1
            plant.advance_to(t)
            obs = sim_camera.observe(plant, t, index)
            info = pipeline.step(obs, t)
            if pipeline.state == "MANUAL" and t >= engage_at and info["detected"] and not pipeline.controller.finished:
                pipeline.request_engage(t)
            if pipeline.state == "LANDING" and not plant.flying:
                pipeline.event(t, "sim", "touchdown (synthetic)")
                break
            if console is not None:
                canvas = np.full((IMAGE_SIZE[1], IMAGE_SIZE[0], 3), 90, np.uint8)
                draw_text(canvas, "SYNTHETIC SIMULATION", (20, 40), (0, 255, 255), 1.0)
                pressed, manual, quit_flag = console.events()
                quit_requested = quit_flag or handle_keys(pressed, manual, t)
                console.draw(overlay(canvas, info), status_lines(info), f"SIM t={t:6.2f}s", (0, 255, 255))
                time.sleep(max(0.0, dt - 0.002))
            if t - last_print >= print_interval:
                last_print = t
                out = info["out"]
                print(f"[{t:7.2f}] {pipeline.state:7s} stage {pipeline.controller.stage_index + 1} "
                      f"|e|={out.error_norm if out else float('nan'):.4f} "
                      f"true z={obs.truth['z']:.3f} RC={info['rc'].as_list()}")

    # ---------------- replay: frame by frame ----------------
    elif args.mode == "replay":
        while not quit_requested:
            obs = source.next()
            if obs is None:
                print("End of video.")
                break
            now = obs.t_capture
            info = pipeline.step(obs, now)
            if args.auto_engage and pipeline.state == "MANUAL" and info["detected"] and not pipeline.controller.finished:
                pipeline.request_engage(now)
            if args.duration and now > args.duration:
                break
            if console is not None:
                pressed, manual, quit_flag = console.events()
                quit_requested = quit_flag or handle_keys(pressed, manual, now)
                console.draw(overlay(obs.frame, info), status_lines(info), f"REPLAY t={now:6.2f}s", (255, 200, 0))
                time.sleep(1.0 / source.fps)

    # ---------------- live camera (camera / compute / live) ----------------
    else:
        if args.mode == "live":
            watchdog.arm()
        start = time.perf_counter()
        last_index = -1
        max_age = float(ibvs_cfg["safety"]["max_frame_age_s"])
        while not quit_requested and source.is_running():
            obs = source.read()
            now = time.perf_counter()
            if obs is not None and obs.frame_index == last_index and now - obs.t_capture < max_age:
                time.sleep(0.002)
                pressed, manual, quit_flag = console.events()
                quit_requested = quit_flag or handle_keys(pressed, manual, now)
                continue
            last_index = obs.frame_index if obs is not None else last_index
            info = pipeline.step(obs, now)
            watchdog.kick()
            if watchdog.fired:
                # the watchdog already sent neutral + LAND from its own thread: make the pipeline agree,
                # then re-arm the watchdog so it protects the rest of the session too
                pipeline.request_land(now, "watchdog fired")
                watchdog.arm()
            if recorder["writer"] is not None:
                raw = source.raw_frame()
                if raw is not None:
                    recorder["writer"].write(raw)
            pressed, manual, quit_flag = console.events()
            quit_requested = quit_flag or handle_keys(pressed, manual, now)
            banner_colour = {"IBVS": (0, 255, 0), "LANDING": (255, 160, 0), "KILLED": (255, 60, 60)}.get(pipeline.state, (230, 230, 230))
            console.draw(overlay(obs.frame, info) if obs is not None else None, status_lines(info),
                         f"{pipeline.state}  {source.fps:4.1f} fps", banner_colour)
            if args.duration and now - start > args.duration:
                break

except KeyboardInterrupt:
    print("Interrupted (Ctrl+C).")
finally:
    shutdown()
    if console is not None:
        console.close()
    if pipeline.stage_results:
        print("Stage results:")
        for result in pipeline.stage_results:
            print(f"  {result}")
