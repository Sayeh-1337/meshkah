import math
import os
import sys
from dataclasses import dataclass

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(ROOT, "src", "fusion"))
sys.path.append(os.path.join(ROOT, "src", "node_sim"))

from fusion.triangulation import triangulate_target  # noqa: E402
from node_sim.sensor_model import enu_to_latlon, enu_to_az_el  # noqa: E402


@dataclass
class DummyReport:
    node_id: str
    lat: float
    lon: float
    alt: float
    azimuth_deg: float
    elevation_deg: float
    confidence: float = 1.0
    modality: str = "rgb"


def test_triangulation_error_below_5m() -> None:
    ref_lat, ref_lon, ref_alt = 30.0, 31.0, 0.0
    target = np.array([200.0, 150.0, 80.0], dtype=float)
    nodes = [
        ("A", np.array([0.0, 0.0, 0.0])),
        ("B", np.array([300.0, 0.0, 0.0])),
        ("C", np.array([150.0, 260.0, 0.0])),
    ]

    reports = []
    for node_id, node_enu in nodes:
        lat, lon, alt = enu_to_latlon(node_enu[0], node_enu[1], node_enu[2], ref_lat, ref_lon, ref_alt)
        az, el, _ = enu_to_az_el(target, node_enu)
        reports.append(
            DummyReport(node_id=node_id, lat=lat, lon=lon, alt=alt, azimuth_deg=az, elevation_deg=el)
        )
    est = triangulate_target(reports, ref_lat, ref_lon, ref_alt)
    err = float(np.linalg.norm(est - target))
    assert err < 5.0, f"Triangulation error {err:.3f} m exceeds limit"
