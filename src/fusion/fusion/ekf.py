from typing import Optional

import numpy as np
from filterpy.kalman import KalmanFilter


class TrackEKF:
    """6-state constant velocity EKF [e, n, u, ve, vn, vu]."""

    def __init__(self, dt: float = 0.05) -> None:
        self.dt = dt
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.kf.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=float,
        )
        self.kf.R = np.diag([25.0, 25.0, 64.0])  # 5m horizontal, 8m vertical
        self.kf.P = np.diag([100.0, 100.0, 100.0, 25.0, 25.0, 25.0])
        self.kf.Q = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0])
        self.kf.x = np.zeros(6, dtype=float)
        self.initialized = False
        self._set_motion_model(self.dt)

    def _set_motion_model(self, dt: float) -> None:
        self.dt = dt
        self.kf.F = np.array(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=float,
        )

    def initialize(self, pos_enu: np.ndarray) -> None:
        self.kf.x = np.array([pos_enu[0], pos_enu[1], pos_enu[2], 0.0, 0.0, 0.0], dtype=float)
        self.initialized = True

    def predict(self) -> None:
        if not self.initialized:
            return
        self.kf.predict()

    def update(self, pos_enu: np.ndarray) -> None:
        z = np.array([pos_enu[0], pos_enu[1], pos_enu[2]], dtype=float)
        if not self.initialized:
            self.initialize(pos_enu)
            return
        self.kf.update(z)

    def get_position(self) -> np.ndarray:
        return self.kf.x[:3].copy()

    def get_velocity(self) -> np.ndarray:
        return self.kf.x[3:6].copy()

    def predict_position(self, t: float) -> np.ndarray:
        return self.kf.x[:3] + self.kf.x[3:6] * float(t)
