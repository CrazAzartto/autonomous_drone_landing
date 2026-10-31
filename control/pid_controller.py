"""
PID controller used by control/run_controller.py  -  ### YOUR BTP WORK: complete update() ###

One PID object per axis (forward/pitch, left/roll, up/throttle, yaw). run_controller.py already
gives you, every loop:
    error       : e.g. errors.e_forward_m     (m or rad, positive = drone must move/turn that way)
    error_rate  : e.g. -errors.v_forward_mps  (m/s or rad/s; from the Kalman filter, NOT a
                  finite difference of a noisy error, so the D-term is clean)
    dt          : seconds since the previous call (already checked to be 0 < dt <= 0.5)

and expects back ONE number: the correction in RC units (PWM microseconds, added to 1500).

IIT Bombay (Paper 3) used exactly this structure:   RC = 1500 + Kp*e + Ki*integral(e) + Kd*de/dt
The D-term on de/dt = -velocity is what makes it a position + velocity controller.
"""


class PID:

    def __init__(self, name, kp, ki, kd, output_limit, integral_limit):
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit        # |output| is clamped to this (RC units)
        self.integral_limit = integral_limit    # |integral| is clamped to this (anti-windup)
        self.integral = 0.0
        self.last_output = 0.0
        self._warned = False

    def reset(self):
        """Call when the controller is (re-)engaged or the marker was lost."""
        self.integral = 0.0
        self.last_output = 0.0

    def update(self, error, error_rate, dt):
        """
        Return the correction in RC units for this axis.

        ### TODO (your BTP work) ###
          1. integral += error * dt, clamped to +-integral_limit
          2. output = kp * error + ki * integral + kd * error_rate
          3. clamp output to +-output_limit
          4. (optional) deadband on small errors, conditional integration while saturated
          5. self.last_output = output; return output

        Until you implement it, it returns 0.0, so the drone gets NEUTRAL commands
        (1500 everywhere = level attitude and altitude hold on the Pluto).
        """
        if not self._warned:
            print(f"[PID {self.name}] update() not implemented yet - returning 0 (neutral). "
                  f"See control/pid_controller.py")
            self._warned = True
        self.last_output = 0.0
        return 0.0
