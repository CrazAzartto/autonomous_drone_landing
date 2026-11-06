"""
Drone adapters with one common interface, so the control program never talks to the network directly.

  DryRunAdapter  : nothing is ever sent (camera-only, compute-only, replay modes). Default.
  PlutoAdapter   : your plutocontrol library (codebase/pluto_course/plutocontrol), used only in live/bench mode.
  SimAdapter     : in sim/pluto_sim.py (synthetic plant).

Facts about plutocontrol (read from YOUR Pluto.py) that shape this adapter:
  * connect() starts a background thread write_function() that sends MSP_SET_RAW_RC with the CURRENT
    rcRoll/rcPitch/rcThrottle/rcYaw/AUX values every ~22 ms (~45 Hz), forever, until disconnect().
    => if the control loop freezes, the LAST command keeps being sent. The watchdog (safety/watchdog.py)
       exists for exactly this reason.
  * an exception in that thread (socket error) kills it silently -> no more RC -> the firmware failsafe
    lands the drone after about 0.5-1.5 s (MagisV2 rx/rx.cpp l.368, flight/failsafe.cpp l.241-247).
    link_status() checks that the thread is alive.
  * command_type (TAKE_OFF / LAND) is sent once and then cleared by that thread. send_rc() therefore
    NEVER writes command_type, otherwise a LAND requested in the same cycle could be overwritten.
  * the getters (get_height(), get_roll() ...) block for up to ~1 s and print every call; they are NOT
    used. Telemetry is parsed from the reply stream in a separate thread (msp_parser.py).
"""

import os
import sys
import threading
import time

from pluto_interface.command_mapper import NEUTRAL, RCCommand
from pluto_interface.msp_parser import MSP_ALTITUDE, MSP_ANALOG, MSP_ATTITUDE, MSP_RC, MSPStreamParser, decode


class BaseAdapter:
    name = "base"
    live = False

    def __init__(self, event_callback=None):
        self._event = event_callback or (lambda kind, message: None)
        self.last_rc = RCCommand()
        self.armed = False
        self.flying = False

    def connect(self):
        return True

    def close(self):
        pass

    def link_status(self):
        return {"ok": True, "reason": "", "writer_alive": True, "telemetry_age_s": float("nan")}

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = False
        self.flying = False

    def take_off(self):
        self.flying = True

    def land(self):
        pass

    def send_rc(self, rc: RCCommand):
        self.last_rc = rc

    def send_neutral(self):
        self.send_rc(RCCommand())

    def telemetry(self):
        return {}


class DryRunAdapter(BaseAdapter):
    """Computes everything, sends nothing. arm/take_off only change flags so the UI flow can be rehearsed."""
    name = "dry_run"
    live = False


class PlutoAdapter(BaseAdapter):
    name = "pluto"
    live = True

    def __init__(self, library_dir, ip="192.168.4.1", port=23, event_callback=None, read_telemetry=True):
        super().__init__(event_callback)
        if library_dir and os.path.isdir(library_dir) and library_dir not in sys.path:
            sys.path.insert(0, library_dir)
        try:
            from plutocontrol import Pluto        # your library: pluto_course/plutocontrol/plutocontrol
        except ImportError as exc:
            raise ImportError(f"cannot import plutocontrol from {library_dir}: {exc}") from exc
        self.pluto = Pluto(ip, port)
        self.read_telemetry = read_telemetry
        self._parser = MSPStreamParser()
        self._telemetry = {}
        self._telemetry_time = None
        self._tel_lock = threading.Lock()
        self._tel_thread = None
        self._pending_echo = None
        self.last_echo_latency_s = float("nan")

    # ---------------- connection ----------------
    def connect(self):
        self.pluto.connect()                     # prints its own message; starts write_function thread
        if not self.pluto.connected:
            self._event("comm_error", "connect failed")
            return False
        with self.pluto._lock:                   # start from neutral sticks, do not touch AUX / command
            self.pluto.rcRoll = NEUTRAL
            self.pluto.rcPitch = NEUTRAL
            self.pluto.rcYaw = NEUTRAL
            self.pluto.rcThrottle = NEUTRAL
        if self.read_telemetry:
            self._tel_thread = threading.Thread(target=self._telemetry_loop, name="PlutoTelemetry", daemon=True)
            self._tel_thread.start()
        return True

    def close(self):
        try:
            self.pluto.disconnect()
        except Exception as exc:                 # never let shutdown raise
            self._event("comm_error", f"disconnect: {exc}")

    def link_status(self):
        writer = getattr(self.pluto, "thread", None)
        writer_alive = bool(writer is not None and writer.is_alive())
        connected = bool(getattr(self.pluto, "connected", False))
        age = float("nan") if self._telemetry_time is None else time.perf_counter() - self._telemetry_time
        ok = connected and writer_alive
        reason = "" if ok else ("not connected" if not connected else "RC writer thread stopped")
        return {"ok": ok, "reason": reason, "writer_alive": writer_alive, "telemetry_age_s": age}

    # ---------------- commands ----------------
    def arm(self):
        self.pluto.arm()
        self.armed = True

    def disarm(self):
        self.pluto.disarm()
        self.armed = False
        self.flying = False

    def take_off(self):
        self.pluto.take_off()
        self.armed = True
        self.flying = True

    def land(self):
        self.pluto.land()

    def send_rc(self, rc: RCCommand):
        if not rc.is_valid():
            self._event("command_rejected", f"invalid RC {rc.as_list()} -> neutral")
            rc = RCCommand()
        with self.pluto._lock:
            changed = (self.pluto.rcRoll, self.pluto.rcPitch, self.pluto.rcThrottle, self.pluto.rcYaw) != tuple(rc.as_list())
            self.pluto.rcRoll = rc.roll
            self.pluto.rcPitch = rc.pitch
            self.pluto.rcThrottle = rc.throttle
            self.pluto.rcYaw = rc.yaw
        if changed and self._pending_echo is None:
            self._pending_echo = (time.perf_counter(), sorted(rc.as_list()))
        self.last_rc = rc

    # ---------------- telemetry ----------------
    def telemetry(self):
        with self._tel_lock:
            data = dict(self._telemetry)
        data["age_s"] = float("nan") if self._telemetry_time is None else time.perf_counter() - self._telemetry_time
        data["rc_echo_latency_s"] = self.last_echo_latency_s
        data["checksum_errors"] = self._parser.checksum_errors
        return data

    def _telemetry_loop(self):
        while getattr(self.pluto, "connected", False):
            try:
                data = self.pluto.receive_packet()   # returns b'' on timeout / error
            except Exception as exc:
                self._event("comm_error", f"telemetry receive: {exc}")
                time.sleep(0.1)
                continue
            if not data:
                continue
            now = time.perf_counter()
            for cmd, payload in self._parser.feed(data):
                if cmd not in (MSP_RC, MSP_ATTITUDE, MSP_ALTITUDE, MSP_ANALOG):
                    continue
                values = decode(cmd, payload)
                if not values:
                    continue
                if cmd == MSP_RC:
                    self._check_echo(values["rc"], now)
                with self._tel_lock:
                    self._telemetry.update(values)
                    self._telemetry_time = now

    def _check_echo(self, rc_values, now):
        """Latency from setting new RC values to seeing them in the drone's MSP_RC reply (Wi-Fi round trip)."""
        if self._pending_echo is None:
            return
        t_set, wanted = self._pending_echo
        if sorted(rc_values[:4]) == wanted:
            self.last_echo_latency_s = now - t_set
            self._pending_echo = None
        elif now - t_set > 2.0:
            self._pending_echo = None             # give up (failsafe values or channel remapping)
