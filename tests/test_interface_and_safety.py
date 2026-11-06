"""RC mapping limits, MSP parsing, the plutocontrol adapter (with a fake library), safety and watchdog."""
import math
import struct
import sys
import threading
import time
import types

import numpy as np
import pytest
import yaml

from ibvs.frames import BodyVelocity
from pluto_interface.command_mapper import RCCommand, VelocityToRCMapper
from pluto_interface.msp_parser import MSPStreamParser, build_reply, decode
from safety.supervisor import HOLD, LAND, OK, WARN, SafetyInputs, SafetySupervisor
from safety.watchdog import Watchdog


def mapper_cfg(**changes):
    cfg = yaml.safe_load(open("config/ibvs_config.yaml", encoding="utf-8"))["command_interface"]
    for path, value in changes.items():
        node = cfg
        keys = path.split(".")
        for k in keys[:-1]:
            node = node[k]
        node[keys[-1]] = value
    return cfg


ZERO = BodyVelocity(0, 0, 0, 0)


def test_horizontal_saturation_and_slew():
    m = VelocityToRCMapper(mapper_cfg())
    rc = m.map(BodyVelocity(1.0, 0, 0, 0), ZERO, 1 / 30)       # huge request
    assert rc.pitch - 1500 <= 300 / 30 + 1                       # slew-limited first step
    assert rc.flags["sat_forward"] and rc.flags["slew_pitch"]
    for _ in range(60):
        rc = m.map(BodyVelocity(1.0, 0, 0, 0), ZERO, 1 / 30)
    assert rc.pitch == 1500 + 40                                 # max_correction_us


def test_vertical_deadband_modes():
    none = VelocityToRCMapper(mapper_cfg(**{"vertical.deadband_mode": "none", "slew_us_per_s.throttle": 0}))
    comp = VelocityToRCMapper(mapper_cfg(**{"slew_us_per_s.throttle": 0}))
    assert none.map(BodyVelocity(0, 0, 0.05, 0), ZERO, 0.03).throttle == 1520     # inside the 40 us dead band
    assert comp.map(BodyVelocity(0, 0, 0.05, 0), ZERO, 0.03).throttle == 1541     # smallest effective climb
    assert comp.map(BodyVelocity(0, 0, -0.05, 0), ZERO, 0.03).throttle == 1459
    assert comp.map(BodyVelocity(0, 0, 0.005, 0), ZERO, 0.03).throttle == 1500    # below min_effective
    assert comp.map(BodyVelocity(0, 0, 1.0, 0), ZERO, 0.03).throttle == 1560      # max_up_us


def test_yaw_disabled_stays_neutral_and_nan_guard():
    m = VelocityToRCMapper(mapper_cfg())
    assert m.map(BodyVelocity(0, 0, 0, 1.0), ZERO, 0.03).yaw == 1500
    rc = m.map(BodyVelocity(float("nan"), 0, 0, 0), ZERO, 0.03)
    assert rc.as_list() == [1500, 1500, 1500, 1500] and rc.flags["nan_guard"]


def test_integral_is_limited():
    m = VelocityToRCMapper(mapper_cfg(**{"slew_us_per_s.pitch": 0, "horizontal.kv_us_per_mps": 0}))
    for _ in range(3000):
        rc = m.map(BodyVelocity(0.2, 0, 0, 0), ZERO, 0.03)
    assert rc.pitch == 1500 + 25                                  # integral_limit_us


def test_msp_parser_chunks_and_resync():
    frames = (build_reply(108, struct.pack("<hhh", -123, 45, 270)) + b"\x00garbage" +
              build_reply(109, struct.pack("<ih", 152, -7)) + build_reply(110, struct.pack("<HHHHBB", 3870, 1, 2, 3, 80, 0)))
    bad = bytearray(build_reply(105, struct.pack("<8H", *range(1500, 1508))))
    bad[-1] ^= 0xFF
    parser = MSPStreamParser()
    out = []
    data = bytes(bad) + frames
    for i in range(0, len(data), 7):                              # arrives in small chunks
        out += parser.feed(data[i:i + 7])
    decoded = {cmd: decode(cmd, payload) for cmd, payload in out}
    assert decoded[108] == {"roll_deg": -12.3, "pitch_deg": 4.5, "heading_deg": 270.0}
    assert decoded[109]["baro_alt_cm"] == 152 and decoded[109]["vario_cmps"] == -7
    assert decoded[110]["vbat_v"] == pytest.approx(3.87)
    assert 105 not in decoded and parser.checksum_errors == 1


class FakePluto:
    """Minimal stand-in for plutocontrol.Pluto (same attributes the adapter uses)."""
    NONE_COMMAND, TAKE_OFF, LAND = 0, 1, 2

    def __init__(self, ip, port):
        self._lock = threading.Lock()
        self.rcRoll = self.rcPitch = self.rcYaw = 1500
        self.rcThrottle = 1000
        self.command_type = 0
        self.connected = False
        self.thread = None
        self._stream = [build_reply(105, struct.pack("<8H", 1490, 1510, 1500, 1500, 1500, 1000, 1500, 1500)),
                        build_reply(108, struct.pack("<hhh", 10, -20, 90))]

    def connect(self):
        self.connected = True
        self.thread = threading.Thread(target=lambda: time.sleep(5), daemon=True)
        self.thread.start()

    def disconnect(self):
        self.connected = False

    def land(self):
        with self._lock:
            self.command_type = self.LAND

    def receive_packet(self):
        time.sleep(0.01)
        return self._stream.pop(0) if self._stream else b""


def test_pluto_adapter_with_fake_library():
    sys.modules["plutocontrol"] = types.SimpleNamespace(Pluto=FakePluto)
    from pluto_interface.adapters import PlutoAdapter
    adapter = PlutoAdapter(library_dir=None)
    assert adapter.connect() and adapter.link_status()["ok"]
    assert adapter.pluto.rcThrottle == 1500                       # neutral at connect, not 1000
    adapter.land()
    adapter.send_rc(RCCommand(roll=1490, pitch=1510))
    assert adapter.pluto.command_type == FakePluto.LAND           # send_rc must not erase the LAND request
    adapter.send_rc(RCCommand(roll=999))                          # invalid -> neutral
    assert adapter.pluto.rcRoll == 1500
    time.sleep(0.2)
    tel = adapter.telemetry()
    assert tel["roll_deg"] == pytest.approx(1.0) and tel["heading_deg"] == 90
    adapter.pluto.connected = False
    assert not adapter.link_status()["ok"]
    del sys.modules["plutocontrol"]


SAFETY = yaml.safe_load(open("config/ibvs_config.yaml", encoding="utf-8"))["safety"]


def inputs(**kw):
    base = dict(now=1.0, frame_age_s=0.03, detected=True, lost_time_s=0.0, features=np.array([0, 0, 0.8, 0]),
                corners_px=np.array([[600, 300], [680, 300], [680, 380], [600, 380]]), condition=1.0,
                command=np.zeros(4), link_ok=True, live=False)
    base.update(kw)
    return SafetyInputs(**base)


def test_supervisor_levels():
    sup = SafetySupervisor(SAFETY)
    assert sup.evaluate(inputs()).level == OK
    assert sup.evaluate(inputs(frame_age_s=0.5)).level == HOLD
    assert sup.evaluate(inputs(detected=False, lost_time_s=0.2, features=None)).level == HOLD
    assert sup.evaluate(inputs(detected=False, lost_time_s=1.5, features=None)).level == LAND
    assert sup.evaluate(inputs(features=np.array([0, 0, math.nan, 0]))).level == HOLD
    assert sup.evaluate(inputs(features=np.array([0, 0, 0.1, 0]))).level == LAND       # too low
    assert sup.evaluate(inputs(features=np.array([0.7, 0, 0.8, 0]))).level == HOLD     # outside fence
    assert sup.evaluate(inputs(corners_px=np.array([[5, 300], [80, 300], [80, 380], [5, 380]]))).level == WARN
    assert sup.evaluate(inputs(command=np.array([math.inf, 0, 0, 0]))).level == HOLD
    assert sup.evaluate(inputs(live=True, link_ok=False)).level == LAND
    assert sup.evaluate(inputs(live=True, telemetry={"age_s": 0.1, "vbat_v": 3.3, "roll_deg": 0, "pitch_deg": 0})).level == LAND
    assert sup.evaluate(inputs(live=True, telemetry={"age_s": 0.1, "vbat_v": 3.9, "roll_deg": 30, "pitch_deg": 0})).level == HOLD


def test_watchdog_fires_once_only_when_not_kicked():
    fired = []
    wd = Watchdog(0.1, lambda: fired.append(time.perf_counter()))
    wd.arm()
    for _ in range(10):
        wd.kick()
        time.sleep(0.03)
    assert not fired
    time.sleep(0.3)
    assert len(fired) == 1
    wd.stop()


def test_watchdog_can_be_rearmed_after_firing():
    fired = []
    wd = Watchdog(0.05, lambda: fired.append(1))
    wd.arm()
    time.sleep(0.2)
    assert wd.fired and len(fired) == 1
    wd.arm()
    assert not wd.fired
    time.sleep(0.2)
    assert len(fired) == 2
    wd.stop()
