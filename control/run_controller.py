"""
PROGRAM 5 - run_controller.py
Closed-loop position hold / landing of the Pluto drone using the ground camera + ArUco marker.

EVERYTHING here is ready EXCEPT the two pieces of the BTP that are still yours to write:
    1. control/pid_controller.py  ->  PID.update()                     (### TODO ###)
    2. compute_rc() below          ->  PID corrections -> RC commands  (### TODO ###)
Until then the program sends / prints NEUTRAL commands (1500), so it is safe to run.

Usage (from the autonomous_drone_landing folder, with the .venv active):
    python control\\run_controller.py            # DRY RUN: camera + errors + controller, NO drone
    python control\\run_controller.py --fly      # connects to the Pluto and SENDS the RC commands
    python control\\run_controller.py --plot     # show plots of the log at the end

Keys (click the camera window first so it has keyboard focus):
    y  arm               t  take off             h  start / stop autonomous HOLD over the target
    l  autonomous LANDING (target height ramps down, then the Pluto's own land command)
    SPACE  EMERGENCY: stop autonomous mode, neutral sticks (the Pluto keeps altitude-hold)
    b / q / ESC  quit (lands and disarms if flying)

Before the first --fly: run tools\\set_target.py, and check every sign in the SIGN CONVENTIONS
block with the propellers OFF (docs/05_adding_pid_and_rc.md, step by step).
"""

import argparse
import csv
import math
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common.config import load_config, repo_path           # noqa: E402
from common.overlay import draw_text                        # noqa: E402
from control.pid_controller import PID                      # noqa: E402
from vision.marker_tracker import MarkerTracker             # noqa: E402


# ============================================================
# SETTINGS
# ============================================================

parser = argparse.ArgumentParser(description="ArUco-based position hold / landing controller for the Pluto.")
parser.add_argument("--config", default=None, help="path to camera_config.yaml")
parser.add_argument("--fly", action="store_true", help="connect to the Pluto and send commands (default: dry run)")
parser.add_argument("--plot", action="store_true", help="plot the control log at the end")
args = parser.parse_args()

config = load_config(args.config)
DRY_RUN = not args.fly
LOOP_RATE_HZ = float(config["control"]["loop_rate_hz"])
LOG_FILE = repo_path(config["control"]["log_file"])
HOVER_HEIGHT_M = float(config["target"]["hover_height_m"])
PRINT_INTERVAL = 0.2

# Folder that contains your plutocontrol package (codebase\pluto_course\plutocontrol).
PLUTO_LIBRARY_DIR = repo_path(os.path.join("..", "pluto_course", "plutocontrol"))

NEUTRAL_RC = 1500
RC_MIN, RC_MAX = 1000, 2000

# Largest correction each axis may add to 1500 (RC units). Start SMALL and increase while tuning.
MAX_PITCH_CORRECTION = 100
MAX_ROLL_CORRECTION = 100
MAX_THROTTLE_CORRECTION = 150
MAX_YAW_CORRECTION = 100

# PID gains - ### TODO: tune ###  (units: RC per metre, RC per metre*second, RC per (m/s);
# for yaw: RC per radian ...). All zero = controller does nothing.
PITCH_KP, PITCH_KI, PITCH_KD = 0.0, 0.0, 0.0          # forward error  -> pitch
ROLL_KP, ROLL_KI, ROLL_KD = 0.0, 0.0, 0.0             # left error     -> roll
THROTTLE_KP, THROTTLE_KI, THROTTLE_KD = 0.0, 0.0, 0.0 # up error       -> throttle
YAW_KP, YAW_KI, YAW_KD = 0.0, 0.0, 0.0                # turn-left error -> yaw
INTEGRAL_LIMIT_M = 0.5                                 # anti-windup (metre*seconds)
INTEGRAL_LIMIT_RAD = 1.0

# ------------------------------------------------------------
# SIGN CONVENTIONS  (from YOUR manual-control code: forward() = pitch 1600, left() = roll 1400,
# increase_height() = throttle 1800, left_yaw() = yaw 1300).  VERIFY with propellers OFF.
#   positive e_forward (target is AHEAD)       -> pitch    ABOVE 1500
#   positive e_left    (target is to the LEFT) -> roll     BELOW 1500
#   positive e_up      (target is HIGHER)      -> throttle ABOVE 1500
#   positive e_yaw     (must turn LEFT / CCW)  -> yaw      BELOW 1500
# ------------------------------------------------------------
PITCH_SIGN = +1
ROLL_SIGN = -1
THROTTLE_SIGN = +1
YAW_SIGN = -1

# Safety
LOST_LAND_S = 1.0             # marker lost longer than this while autonomous -> land
MAX_TARGET_HEIGHT_M = 1.5
DESCEND_RATE_MPS = 0.10       # landing: target height decreases at this rate
LAND_HEIGHT_M = 0.25          # landing: below this target height the Pluto's own land() takes over


def clamp(value, low, high):
    return max(low, min(high, value))


# ============================================================
# PID OBJECTS (one per axis)
# ============================================================

pid_pitch = PID("pitch", PITCH_KP, PITCH_KI, PITCH_KD, MAX_PITCH_CORRECTION, INTEGRAL_LIMIT_M)
pid_roll = PID("roll", ROLL_KP, ROLL_KI, ROLL_KD, MAX_ROLL_CORRECTION, INTEGRAL_LIMIT_M)
pid_throttle = PID("throttle", THROTTLE_KP, THROTTLE_KI, THROTTLE_KD, MAX_THROTTLE_CORRECTION, INTEGRAL_LIMIT_M)
pid_yaw = PID("yaw", YAW_KP, YAW_KI, YAW_KD, MAX_YAW_CORRECTION, INTEGRAL_LIMIT_RAD)
ALL_PIDS = (pid_pitch, pid_roll, pid_throttle, pid_yaw)


def reset_pids():
    for pid in ALL_PIDS:
        pid.reset()


def neutral_rc():
    return {"roll": NEUTRAL_RC, "pitch": NEUTRAL_RC, "throttle": NEUTRAL_RC, "yaw": NEUTRAL_RC}


# ============================================================
# CONTROLLER:  errors -> PID corrections -> RC commands
# ============================================================

def compute_rc(errors, target_height_m, dt):
    """
    errors          : vision.target.ControlErrors (drone frame, metres / radians, Kalman-filtered)
    target_height_m : desired marker height above the camera (changes during landing)
    dt              : seconds since the last call
    Returns (rc dict, corrections dict).
    """
    # Errors and their rates (rate = -velocity because the target does not move).
    e_up = target_height_m - errors.z_world_m

    corrections = {
        "pitch": pid_pitch.update(errors.e_forward_m, -errors.v_forward_mps, dt),
        "roll": pid_roll.update(errors.e_left_m, -errors.v_left_mps, dt),
        "throttle": pid_throttle.update(e_up, -errors.v_up_mps, dt),
        "yaw": pid_yaw.update(errors.e_yaw_rad, -errors.yaw_rate_radps, dt),
    }

    rc = neutral_rc()

    # ### TODO (your BTP work): convert the corrections into RC commands ###
    # IIT Bombay form:  RC = 1500 + SIGN * correction, clamped. Example for pitch:
    #
    #     rc["pitch"] = int(clamp(NEUTRAL_RC + PITCH_SIGN * corrections["pitch"], RC_MIN, RC_MAX))
    #
    # Do the same for roll, throttle and yaw. Things to think about (docs/05):
    #   - Pluto altitude-hold has a throttle DEADBAND around 1500 (+-40 in the MagisV2 source):
    #     small throttle corrections do nothing unless you skip over the deadband.
    #   - Tilt commands act on ACCELERATION, so the D-term (velocity) is essential.
    #   - Start with one axis at a time (others neutral) and small MAX_*_CORRECTION.

    return rc, corrections


# ============================================================
# CONNECT PLUTO (only with --fly)
# ============================================================

pluto = None
if not DRY_RUN:
    # Uses YOUR copy of the library: codebase\pluto_course\plutocontrol (no pip install needed).
    if os.path.isdir(PLUTO_LIBRARY_DIR):
        sys.path.insert(0, PLUTO_LIBRARY_DIR)
    try:
        from plutocontrol import Pluto
    except ImportError:
        print(f"ERROR: cannot import plutocontrol. Expected it in {PLUTO_LIBRARY_DIR} "
              "(set PLUTO_LIBRARY_DIR at the top of this file).")
        sys.exit(1)
    pluto = Pluto()
    print("Connecting to Pluto (join the Pluto's Wi-Fi first)...")
    pluto.connect()
    print("Pluto connected.")
else:
    print("DRY RUN - no drone. Commands are only printed and logged. Use --fly to send them.")


def send_rc(rc):
    """Write the four RC values into the Pluto object (its own thread transmits them)."""
    if pluto is None:
        return
    with pluto._lock:
        pluto.rcRoll = int(rc["roll"])
        pluto.rcPitch = int(rc["pitch"])
        pluto.rcThrottle = int(rc["throttle"])
        pluto.rcYaw = int(rc["yaw"])
        pluto.command_type = pluto.NONE_COMMAND


# ============================================================
# START CAMERA TRACKER
# ============================================================

try:
    tracker = MarkerTracker(config)
    tracker.start()
except (RuntimeError, FileNotFoundError) as exc:
    print(f"ERROR: {exc}")
    if pluto is not None:
        pluto.disconnect()
    sys.exit(1)

if tracker.processor.target is None:
    print("WARNING: no target set (results\\target.json). Run tools\\set_target.py first - "
          "autonomous mode is disabled until then.")


# ============================================================
# CSV LOGGER
# ============================================================

os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
log_file = open(LOG_FILE, "w", newline="")
writer = csv.writer(log_file)
writer.writerow([
    "time_s", "mode", "dry_run", "valid", "detected", "age_s",
    "x_world_m", "y_world_m", "z_world_m", "yaw_world_deg",
    "e_forward_m", "e_left_m", "e_up_m", "e_yaw_deg",
    "v_forward_mps", "v_left_mps", "v_up_mps", "yaw_rate_dps",
    "target_height_m",
    "corr_pitch", "corr_roll", "corr_throttle", "corr_yaw",
    "rc_roll", "rc_pitch", "rc_throttle", "rc_yaw", "camera_fps",
])


# ============================================================
# MAIN LOOP
# ============================================================

NAN = float("nan")
armed = False
flying = False
mode = "MANUAL"               # MANUAL (neutral sticks) | HOLD | LANDING
target_height_m = HOVER_HEIGHT_M
last_control_time = None
lost_reset_done = False
rc = neutral_rc()
corrections = {"pitch": NAN, "roll": NAN, "throttle": NAN, "yaw": NAN}
start_time = time.perf_counter()
last_print = 0.0
period = 1.0 / LOOP_RATE_HZ

print("Keys: y arm | t take off | h hold | l land | SPACE emergency neutral | b/q quit")

try:
    while tracker.is_running():
        loop_start = time.perf_counter()
        elapsed = loop_start - start_time
        state = tracker.get_state()
        errors = state.errors

        # ----------------------------------------------------
        # CONTROL
        # ----------------------------------------------------
        if mode in ("HOLD", "LANDING"):
            if errors is not None and errors.valid:
                lost_reset_done = False
                if mode == "LANDING":
                    target_height_m = max(0.0, target_height_m - DESCEND_RATE_MPS * period)
                    if target_height_m <= LAND_HEIGHT_M:
                        print("LANDING: low enough - Pluto land command.")
                        mode = "MANUAL"
                        rc = neutral_rc()
                        send_rc(rc)
                        if pluto is not None and flying:
                            pluto.land()
                        flying = False
                        continue
                dt = period if last_control_time is None else loop_start - last_control_time
                if 0.0 < dt <= 0.5:
                    rc, corrections = compute_rc(errors, target_height_m, dt)
                else:
                    reset_pids()
                last_control_time = loop_start
            else:
                # Marker lost: neutral sticks (level + altitude hold), then land if it stays lost.
                rc = neutral_rc()
                corrections = {"pitch": NAN, "roll": NAN, "throttle": NAN, "yaw": NAN}
                last_control_time = None
                if not lost_reset_done:
                    reset_pids()
                    lost_reset_done = True
                    print("MARKER LOST - neutral sticks.")
                if state.age_s > LOST_LAND_S:
                    print(f"Marker lost for more than {LOST_LAND_S} s - LANDING.")
                    mode = "MANUAL"
                    if pluto is not None and flying:
                        pluto.land()
                    flying = False
            send_rc(rc)
        else:
            # MANUAL: this program does NOT touch the RC values (after take_off() the Pluto library
            # keeps what take_off() set, exactly like your manual scripts). Logged values = the
            # values currently stored in the Pluto object.
            rc = neutral_rc() if pluto is None else dict(zip(("roll", "pitch", "throttle", "yaw"), pluto.rc_values()[:4]))
            corrections = {"pitch": NAN, "roll": NAN, "throttle": NAN, "yaw": NAN}

        # ----------------------------------------------------
        # LOG
        # ----------------------------------------------------
        e = errors
        has_e = e is not None and e.valid
        writer.writerow([
            round(elapsed, 4), mode, int(DRY_RUN), int(state.valid), int(state.detected), round(state.age_s, 4),
            e.x_world_m if has_e else NAN, e.y_world_m if has_e else NAN, e.z_world_m if has_e else NAN,
            math.degrees(e.yaw_world_rad) if has_e else NAN,
            e.e_forward_m if has_e else NAN, e.e_left_m if has_e else NAN,
            (target_height_m - e.z_world_m) if has_e else NAN, math.degrees(e.e_yaw_rad) if has_e else NAN,
            e.v_forward_mps if has_e else NAN, e.v_left_mps if has_e else NAN, e.v_up_mps if has_e else NAN,
            math.degrees(e.yaw_rate_radps) if has_e else NAN,
            target_height_m,
            corrections["pitch"], corrections["roll"], corrections["throttle"], corrections["yaw"],
            rc["roll"], rc["pitch"], rc["throttle"], rc["yaw"], round(state.fps, 1),
        ])

        # ----------------------------------------------------
        # DISPLAY + PRINT
        # ----------------------------------------------------
        view = tracker.get_frame()
        if view is not None:
            view = view.copy()
            colour = (0, 255, 0) if mode != "MANUAL" else (255, 255, 255)
            draw_text(view, f"MODE {mode}{'  (DRY RUN)' if DRY_RUN else ''}   armed={armed} flying={flying}"
                      f"   target h={target_height_m * 100:.0f} cm", (10, view.shape[0] - 45), colour, 0.6)
            draw_text(view, f"RC  roll={rc['roll']}  pitch={rc['pitch']}  throttle={rc['throttle']}  yaw={rc['yaw']}",
                      (10, view.shape[0] - 18), colour, 0.6)
            cv2.imshow("Pluto controller  (y arm | t takeoff | h hold | l land | SPACE stop | q quit)", view)

        if elapsed - last_print >= PRINT_INTERVAL:
            last_print = elapsed
            if has_e:
                print(f"[{elapsed:7.2f} s] {mode:7s} err fwd={e.e_forward_m * 100:+6.1f} left={e.e_left_m * 100:+6.1f} "
                      f"up={(target_height_m - e.z_world_m) * 100:+6.1f} cm yaw={math.degrees(e.e_yaw_rad):+6.1f} deg"
                      f" | RC r={rc['roll']} p={rc['pitch']} t={rc['throttle']} y={rc['yaw']}")
            else:
                print(f"[{elapsed:7.2f} s] {mode:7s} no valid marker state | RC neutral")

        # ----------------------------------------------------
        # KEYS (the wait also paces the loop)
        # ----------------------------------------------------
        remaining_ms = int(max(1, (period - (time.perf_counter() - loop_start)) * 1000))
        key = cv2.waitKey(remaining_ms) & 0xFF

        if key in (ord("q"), ord("b"), 27):
            break

        elif key == ord(" "):
            mode = "MANUAL"
            rc = neutral_rc()
            send_rc(rc)
            reset_pids()
            print("EMERGENCY STOP of autonomous mode - neutral sticks.")

        elif key == ord("y") and not armed:
            if pluto is not None:
                pluto.arm()
            armed = True
            print("Armed.")

        elif key == ord("t") and armed and not flying:
            if pluto is not None:
                pluto.take_off()
            flying = True
            print("Take-off command sent.")

        elif key == ord("h"):
            if mode == "MANUAL":
                if tracker.processor.target is None:
                    print("HOLD refused: no target (run tools\\set_target.py).")
                elif not (errors is not None and errors.valid):
                    print("HOLD refused: marker not tracked.")
                else:
                    reset_pids()
                    last_control_time = None
                    target_height_m = HOVER_HEIGHT_M
                    mode = "HOLD"
                    print(f"HOLD engaged: target height {HOVER_HEIGHT_M * 100:.0f} cm above the camera.")
            else:
                mode = "MANUAL"
                rc = neutral_rc()
                send_rc(rc)
                print("HOLD stopped - neutral sticks.")

        elif key == ord("l"):
            if mode in ("HOLD", "LANDING") or (errors is not None and errors.valid and tracker.processor.target is not None):
                if mode == "MANUAL":
                    reset_pids()
                    last_control_time = None
                    target_height_m = clamp(errors.z_world_m, 0.0, MAX_TARGET_HEIGHT_M)
                mode = "LANDING"
                print(f"LANDING: descending at {DESCEND_RATE_MPS * 100:.0f} cm/s over the target.")
            else:
                if pluto is not None and flying:
                    pluto.land()
                flying = False
                print("Land command sent (no marker tracking).")

except KeyboardInterrupt:
    print("Interrupted.")

finally:
    print("Stopping controller...")
    if pluto is not None:
        send_rc(neutral_rc())
        if flying:
            print("Landing...")
            pluto.land()
            time.sleep(2)
        if armed:
            print("Disarming...")
            pluto.disarm()
        pluto.disconnect()
    tracker.stop()
    cv2.destroyAllWindows()
    log_file.close()
    print(f"Control log saved to: {LOG_FILE}")

if args.plot:
    from control.plotting import plot_control_log
    plot_control_log(LOG_FILE)
