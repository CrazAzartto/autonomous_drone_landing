"""
Constant-velocity Kalman filter in 3-D.

State  x = [px, py, pz, vx, vy, vz]   (metres, metres/second)
Model  p(k+1) = p(k) + v(k)*dt,  v(k+1) = v(k) + a*dt  with a = random acceleration (white noise)
Measurement z = [px, py, pz] from the camera.

Why: the PID controller needs (1) a smooth position and (2) a VELOCITY for its D-term.
Differentiating raw camera positions amplifies noise (sigma_v = sqrt(2)*sigma_p/dt: 5 mm noise at
30 fps gives ~0.2 m/s of fake velocity). The filter gives a clean velocity and keeps predicting
for a short time when the marker is briefly lost.
"""

import numpy as np


class ConstantVelocityKalman3D:

    def __init__(self, accel_std=2.0, meas_std_xy=0.004, meas_std_z=0.010):
        self.accel_std = float(accel_std)
        self.R = np.diag([meas_std_xy ** 2, meas_std_xy ** 2, meas_std_z ** 2])
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.x = np.zeros(6)
        self.P = np.eye(6)
        self.initialized = False
        self.last_time = None

    def reset(self):
        self.initialized = False
        self.last_time = None

    def _predict_to(self, t):
        dt = 0.0 if self.last_time is None else max(0.0, t - self.last_time)
        if dt > 0:
            F = np.eye(6)
            F[0:3, 3:6] = np.eye(3) * dt
            q = self.accel_std ** 2
            # Discrete white-noise-acceleration model, per axis: [[dt^4/4, dt^3/2], [dt^3/2, dt^2]] * q
            Q = np.zeros((6, 6))
            Q[0:3, 0:3] = np.eye(3) * (dt ** 4 / 4.0) * q
            Q[0:3, 3:6] = np.eye(3) * (dt ** 3 / 2.0) * q
            Q[3:6, 0:3] = np.eye(3) * (dt ** 3 / 2.0) * q
            Q[3:6, 3:6] = np.eye(3) * (dt ** 2) * q
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q
        self.last_time = t

    def predict(self, t):
        """Advance the estimate to time t without a measurement (marker not seen)."""
        if self.initialized:
            self._predict_to(t)
        return self.position, self.velocity

    def update(self, t, measured_position):
        """Fuse one camera measurement taken at time t."""
        z = np.asarray(measured_position, dtype=float).reshape(3)
        if not self.initialized:
            self.x = np.concatenate([z, np.zeros(3)])
            self.P = np.diag([self.R[0, 0], self.R[1, 1], self.R[2, 2], 1.0, 1.0, 1.0])
            self.initialized = True
            self.last_time = t
            return self.position, self.velocity

        self._predict_to(t)
        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.position, self.velocity

    @property
    def position(self):
        return self.x[0:3].copy() if self.initialized else None

    @property
    def velocity(self):
        return self.x[3:6].copy() if self.initialized else None
