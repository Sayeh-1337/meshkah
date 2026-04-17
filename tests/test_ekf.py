import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(ROOT, "src", "fusion"))

from fusion.ekf import TrackEKF  # noqa: E402


def test_ekf_smoothing_and_prediction() -> None:
    ekf = TrackEKF(dt=0.05)
    rng = np.random.default_rng(1234)

    t = 0.0
    truth_v = np.array([10.0, 0.0, 0.0], dtype=float)
    meas_errors = []
    est_errors = []

    for _ in range(400):
        truth_pos = np.array([truth_v[0] * t, 0.0, 80.0], dtype=float)
        noise = np.array([rng.normal(0, 5), rng.normal(0, 5), rng.normal(0, 8)], dtype=float)
        meas = truth_pos + noise
        ekf.predict()
        ekf.update(meas)
        est = ekf.get_position()
        meas_errors.append(np.linalg.norm(meas - truth_pos))
        est_errors.append(np.linalg.norm(est - truth_pos))
        t += 0.05

    assert float(np.mean(est_errors)) < float(np.mean(meas_errors))
    pred = ekf.predict_position(1.0)
    future_truth = np.array([truth_v[0] * t + 10.0, 0.0, 80.0], dtype=float)
    assert float(np.linalg.norm(pred - future_truth)) < 2.0
