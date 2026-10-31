#!/usr/bin/env bash
# ============================================================
# Inspect and configure the Logitech C920 with v4l2-ctl (Ubuntu)
# ============================================================
# Usage:
#   bash scripts/configure_c920.sh /dev/video0 show          # list formats + controls + current values
#   bash scripts/configure_c920.sh /dev/video0 autofocus     # autofocus ON (to find a focus value)
#   bash scripts/configure_c920.sh /dev/video0 lock FOCUS EXPOSURE GAIN
#        e.g.  bash scripts/configure_c920.sh /dev/video0 lock 30 150 64
#        -> autofocus OFF + focus=FOCUS, manual exposure=EXPOSURE (x100 us), gain=GAIN,
#           digital zoom off, auto white balance off, no dynamic frame rate, 50 Hz anti-flicker
#        Under 50 Hz mains lighting use EXPOSURE = 100 (10 ms) or 200 (20 ms) to avoid flicker bands.
#   bash scripts/configure_c920.sh /dev/video0 auto          # restore automatic exposure/focus/WB
#
# Every control is set ONLY if this camera reports it (names differ between kernel versions:
# newer kernels use auto_exposure / focus_automatic_continuous / exposure_time_absolute,
# older ones exposure_auto / focus_auto / exposure_absolute). Values are read back after setting.
# Settings are lost when the camera is unplugged - run this again after reconnecting
# (or put the values into config/camera_config.yaml -> v4l2_controls, which every program applies).
# ============================================================

set -u

DEV="${1:-/dev/video0}"
MODE="${2:-show}"

if ! command -v v4l2-ctl >/dev/null 2>&1; then
    echo "v4l2-ctl not found. Install it:  sudo apt install v4l-utils"
    exit 1
fi
if [ ! -e "$DEV" ]; then
    echo "Device $DEV does not exist. List devices with:  v4l2-ctl --list-devices"
    exit 1
fi

CONTROLS="$(v4l2-ctl -d "$DEV" --list-ctrls 2>/dev/null)"

has() {             # has <control_name>  -> true if the camera reports this control
    echo "$CONTROLS" | awk '{print $1}' | grep -qx "$1"
}

first_of() {        # first_of name1 name2 ...  -> prints the first name the camera supports
    for name in "$@"; do
        if has "$name"; then echo "$name"; return 0; fi
    done
    return 1
}

set_ctrl() {        # set_ctrl <name> <value>   (skips unknown controls, prints read-back value)
    local name="$1" value="$2"
    if ! has "$name"; then
        echo "  [skip ] $name  (not reported by this camera)"
        return
    fi
    v4l2-ctl -d "$DEV" --set-ctrl="$name=$value" 2>&1 | sed 's/^/  /'
    sleep 0.1
    local now
    now="$(v4l2-ctl -d "$DEV" --get-ctrl="$name" 2>/dev/null | awk -F': ' '{print $2}')"
    if [ "$now" = "$value" ]; then
        echo "  [ OK  ] $name = $now"
    else
        echo "  [CHECK] $name requested $value, camera reports '$now'"
    fi
}

AUTO_EXPOSURE="$(first_of auto_exposure exposure_auto || true)"
EXPOSURE="$(first_of exposure_time_absolute exposure_absolute || true)"
DYN_FPS="$(first_of exposure_dynamic_framerate exposure_auto_priority || true)"
AUTO_FOCUS="$(first_of focus_automatic_continuous focus_auto || true)"
AUTO_WB="$(first_of white_balance_automatic white_balance_temperature_auto || true)"

case "$MODE" in

  show)
    echo "=== Devices ==="
    v4l2-ctl --list-devices
    echo "=== Formats of $DEV (look for MJPG 1280x720 30 fps) ==="
    v4l2-ctl -d "$DEV" --list-formats-ext
    echo "=== Controls of $DEV (names, ranges, current values; 'inactive' = locked by an auto mode) ==="
    v4l2-ctl -d "$DEV" --list-ctrls-menus
    echo "=== Current format ==="
    v4l2-ctl -d "$DEV" --get-fmt-video
    ;;

  autofocus)
    echo "Autofocus ON. Hold the target at your working distance for ~3 s, then read the value:"
    [ -n "$AUTO_FOCUS" ] && set_ctrl "$AUTO_FOCUS" 1
    echo "  v4l2-ctl -d $DEV --get-ctrl=focus_absolute"
    ;;

  lock)
    if [ $# -lt 5 ]; then
        echo "Usage: bash scripts/configure_c920.sh $DEV lock FOCUS EXPOSURE GAIN"
        exit 1
    fi
    FOCUS="$3"; EXPOSURE_VALUE="$4"; GAIN="$5"
    echo "=== Locking focus / exposure / white balance / zoom on $DEV ==="
    # Order matters: switch the AUTO mode off first, then write the manual value.
    [ -n "$AUTO_FOCUS" ] && set_ctrl "$AUTO_FOCUS" 0
    set_ctrl focus_absolute "$FOCUS"
    [ -n "$AUTO_EXPOSURE" ] && set_ctrl "$AUTO_EXPOSURE" 1        # 1 = Manual Mode (UVC menu)
    [ -n "$EXPOSURE" ] && set_ctrl "$EXPOSURE" "$EXPOSURE_VALUE"
    [ -n "$DYN_FPS" ] && set_ctrl "$DYN_FPS" 0                    # keep the frame rate constant
    set_ctrl gain "$GAIN"
    [ -n "$AUTO_WB" ] && set_ctrl "$AUTO_WB" 0
    set_ctrl zoom_absolute 100                                    # 100 = minimum = no digital zoom
    set_ctrl power_line_frequency 1                               # 1 = 50 Hz mains (India/Europe); 2 = 60 Hz
    echo "Done. Verify any time with:  v4l2-ctl -d $DEV --list-ctrls"
    ;;

  auto)
    echo "=== Restoring automatic modes on $DEV ==="
    [ -n "$AUTO_FOCUS" ] && set_ctrl "$AUTO_FOCUS" 1
    [ -n "$AUTO_EXPOSURE" ] && set_ctrl "$AUTO_EXPOSURE" 3        # 3 = Aperture Priority (auto)
    [ -n "$AUTO_WB" ] && set_ctrl "$AUTO_WB" 1
    ;;

  *)
    echo "Unknown mode '$MODE'. Use: show | autofocus | lock FOCUS EXPOSURE GAIN | auto"
    exit 1
    ;;
esac
