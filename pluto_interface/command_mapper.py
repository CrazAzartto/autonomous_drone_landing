"""
Velocity set-point -> Pluto RC channels (the Pluto-specific low-level interface, docs/07).

WHY THIS LAYER EXISTS
  [P1] assumes a "high level control" that tracks v_x, v_y, v_z and the yaw rate (p.4, Eq.(8)).
  The Pluto firmware provides only part of that:
    throttle (altitude-hold mode) -> vertical VELOCITY set-point  v_z = (RC - 1500)/4 cm/s,
                                     |RC - 1500| <= 40 is a dead band (v_z = 0, hold altitude),
                                     clamp -100..120 cm/s          [MagisV2 flight/altitudehold.cpp l.231-238]
    yaw                           -> yaw RATE demand (scale depends on firmware rates: identify)
    roll / pitch                  -> TILT ANGLE set-point (angle mode), NOT a velocity
                                     [MagisV2 flight/pid.cpp l.252-255; max 20 deg, config.cpp l.447]
  so the horizontal velocity loop must be closed here, with the velocity ESTIMATED from the camera.

HORIZONTAL AXES (forward -> pitch, left -> roll): velocity PI loop
      RC - 1500 = sign * ( Kv (v* - v_hat) + Ki * integral(v* - v_hat) )
  This is the structure of the IIT Bombay controller [P3] Sec. 4.1 / Fig. 3 (parallel PID on the
  position error e, RC = 1500 + PID): with v* = lambda e and v_hat = -de/dt,
      Kv (lambda e - v_hat) = (Kv lambda) e + Kv de/dt  = Kp e + Kd de/dt,   Kp = Kv lambda,  Kd = Kv,
  i.e. [P3]'s PD law is the special case of this cascade with [P1]'s proportional IBVS outer loop.

VERTICAL AXIS: open-loop inverse of the firmware map (the firmware closes the velocity loop with its own
  baro/accelerometer estimate):  RC - 1500 = sign * 400 us/(m/s) * v_z*.
  Dead band: commands |RC - 1500| <= 40 (|v_z*| < 0.10 m/s) produce NO motion. deadband_mode:
     "none"       : send the plain value (small corrections are lost)       [faithful to the firmware]
     "compensate" : if |v_z*| >= min_effective_mps, send at least 41 us (=10 cm/s, the smallest speed the
                    firmware can produce); below that, 1500 (hold)          [default, see docs/07 §4]

YAW: RC - 1500 = sign * k_yaw * r*. k_yaw is NOT known from documentation -> disabled until identified.

Every output is clamped (max_correction_us), slew-rate limited (slew_us_per_s) and reported with flags.
"""

import math
from dataclasses import dataclass, field

from ibvs.frames import BodyVelocity


NEUTRAL = 1500


@dataclass
class RCCommand:
    roll: int = NEUTRAL
    pitch: int = NEUTRAL
    throttle: int = NEUTRAL
    yaw: int = NEUTRAL
    flags: dict = field(default_factory=dict)

    def as_list(self):
        return [self.roll, self.pitch, self.throttle, self.yaw]

    def is_valid(self, low=1000, high=2000):
        return all(isinstance(v, int) and low <= v <= high for v in self.as_list())


def neutral_command(note="neutral"):
    return RCCommand(flags={"mode": note})


def _clamp(value, low, high):
    return max(low, min(high, value))


class _SlewLimiter:
    def __init__(self, rate_us_per_s):
        self.rate = float(rate_us_per_s)
        self.value = 0.0

    def reset(self, value=0.0):
        self.value = float(value)

    def __call__(self, target, dt):
        if self.rate <= 0 or dt <= 0:
            self.value = target
            return target, False
        step = self.rate * dt
        limited = _clamp(target, self.value - step, self.value + step)
        was_limited = abs(limited - target) > 1e-9
        self.value = limited
        return limited, was_limited


class VelocityToRCMapper:
    """
    config: the 'command_interface' section of config/ibvs_config.yaml (see that file for every parameter
    and the basis of its default value).
    """

    def __init__(self, config):
        xy = config["horizontal"]
        z = config["vertical"]
        yaw = config["yaw"]
        self.neutral = int(config.get("neutral_us", NEUTRAL))

        self.kv = float(xy["kv_us_per_mps"])
        self.ki = float(xy["ki_us_per_m"])
        self.integral_limit = float(xy["integral_limit_us"])
        self.max_xy = float(xy["max_correction_us"])
        self.sign_pitch = int(xy["sign_pitch"])
        self.sign_roll = int(xy["sign_roll"])

        self.z_gain = float(z["us_per_mps"])
        self.z_deadband = float(z["deadband_us"])
        self.z_mode = str(z["deadband_mode"])
        self.z_min_effective = float(z["min_effective_mps"])
        self.max_up_us = float(z["max_up_us"])
        self.max_down_us = float(z["max_down_us"])
        self.sign_throttle = int(z["sign_throttle"])

        self.yaw_enabled = bool(yaw["enabled"])
        self.k_yaw = float(yaw["us_per_radps"])
        self.max_yaw = float(yaw["max_correction_us"])
        self.sign_yaw = int(yaw["sign_yaw"])

        slew = config["slew_us_per_s"]
        self._slew = {"pitch": _SlewLimiter(slew["pitch"]), "roll": _SlewLimiter(slew["roll"]),
                      "throttle": _SlewLimiter(slew["throttle"]), "yaw": _SlewLimiter(slew["yaw"])}
        self._integral = {"forward": 0.0, "left": 0.0}

    # ------------------------------------------------------------
    def reset(self):
        """Call whenever the autonomous mode is (re-)engaged or interrupted."""
        for limiter in self._slew.values():
            limiter.reset(0.0)
        self._integral = {"forward": 0.0, "left": 0.0}

    def integrals(self):
        return dict(self._integral)

    # ------------------------------------------------------------
    def _horizontal(self, axis, v_star, v_hat, dt, flags):
        error = v_star - v_hat
        if self.ki > 0 and dt > 0:
            limit = self.integral_limit / self.ki
            self._integral[axis] = _clamp(self._integral[axis] + error * dt, -limit, limit)
        correction = self.kv * error + self.ki * self._integral[axis]
        clamped = _clamp(correction, -self.max_xy, self.max_xy)
        flags[f"sat_{axis}"] = abs(clamped - correction) > 1e-9
        return clamped

    def _vertical(self, v_star, flags):
        raw = self.z_gain * v_star
        flags["z_in_deadband"] = abs(raw) <= self.z_deadband
        if self.z_mode == "compensate":
            if abs(v_star) < self.z_min_effective:
                raw = 0.0
            elif abs(raw) <= self.z_deadband:
                raw = math.copysign(self.z_deadband + 1.0, raw)
        elif self.z_mode != "none":
            raise ValueError("vertical.deadband_mode must be 'none' or 'compensate'")
        clamped = _clamp(raw, -self.max_down_us, self.max_up_us)
        flags["sat_up"] = abs(clamped - raw) > 1e-9
        return clamped

    def map(self, command: BodyVelocity, measured: BodyVelocity, dt):
        """Returns RCCommand. command = set-point from IBVS, measured = camera estimate (body frame)."""
        flags = {"mode": "ibvs"}
        if not (command.is_finite() and measured.is_finite()):
            self.reset()
            out = neutral_command("nan_guard")
            out.flags["nan_guard"] = True
            return out

        pitch_corr = self._horizontal("forward", command.forward, measured.forward, dt, flags)
        roll_corr = self._horizontal("left", command.left, measured.left, dt, flags)
        thr_corr = self._vertical(command.up, flags)
        if self.yaw_enabled:
            raw = self.k_yaw * command.yaw_rate
            yaw_corr = _clamp(raw, -self.max_yaw, self.max_yaw)
            flags["sat_yaw"] = abs(yaw_corr - raw) > 1e-9
        else:
            yaw_corr = 0.0
            flags["sat_yaw"] = False

        # signs map body directions onto Pluto stick directions (verify on the bench, docs/08)
        # (sign_roll = -1 means "+left -> roll below 1500", sign_yaw = -1 "+turn left -> yaw below 1500")
        targets = {"pitch": self.sign_pitch * pitch_corr, "roll": self.sign_roll * roll_corr,
                   "throttle": self.sign_throttle * thr_corr, "yaw": self.sign_yaw * yaw_corr}

        values = {}
        for channel, target in targets.items():
            limited, was_limited = self._slew[channel](target, dt)
            flags[f"slew_{channel}"] = was_limited
            values[channel] = int(round(self.neutral + limited))
        out = RCCommand(roll=values["roll"], pitch=values["pitch"], throttle=values["throttle"],
                        yaw=values["yaw"], flags=flags)
        flags["corrections_us"] = {k: round(v, 2) for k, v in targets.items()}
        return out
