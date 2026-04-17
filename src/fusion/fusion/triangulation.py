import math
from typing import Iterable, List

import numpy as np
from scipy.optimize import minimize

EARTH_RADIUS_M = 6371000.0


def latlon_to_enu(
    lat: float,
    lon: float,
    alt: float,
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> np.ndarray:
    """GPS to ENU with spherical-Earth approximation."""
    ref_lat_rad = math.radians(ref_lat)
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    east = EARTH_RADIUS_M * d_lon * math.cos(ref_lat_rad)
    north = EARTH_RADIUS_M * d_lat
    up = alt - ref_alt
    return np.array([east, north, up], dtype=float)


def bearing_vector(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """Build ENU unit direction vector from azimuth/elevation in degrees."""
    az = math.radians(azimuth_deg)
    el = math.radians(elevation_deg)
    east = math.sin(az) * math.cos(el)
    north = math.cos(az) * math.cos(el)
    up = math.sin(el)
    vec = np.array([east, north, up], dtype=float)
    norm = np.linalg.norm(vec)
    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return vec / norm


def _objective(candidate: np.ndarray, sensor_positions: List[np.ndarray], bearings: List[np.ndarray]) -> float:
    sse = 0.0
    for p, d in zip(sensor_positions, bearings):
        r = candidate - p
        perp = r - np.dot(r, d) * d
        sse += float(np.dot(perp, perp))
    return sse


def triangulate_target(reports: Iterable, ref_lat: float, ref_lon: float, ref_alt: float) -> np.ndarray:
    """Triangulate target ENU position from bearing-only reports."""
    reports = list(reports)
    if len(reports) < 2:
        raise ValueError("Need at least two reports for triangulation")
    sensor_positions = []
    bearings = []
    for report in reports:
        pos = latlon_to_enu(report.lat, report.lon, report.alt, ref_lat, ref_lon, ref_alt)
        b = bearing_vector(report.azimuth_deg, report.elevation_deg)
        sensor_positions.append(pos)
        bearings.append(b)
    x0 = np.mean(np.array(sensor_positions), axis=0)
    res = minimize(
        _objective,
        x0=x0,
        args=(sensor_positions, bearings),
        method="Nelder-Mead",
        options={"maxiter": 2000, "xatol": 1e-4, "fatol": 1e-4},
    )
    if not res.success:
        # Return best effort even if optimizer reports non-convergence.
        return np.array(res.x, dtype=float)
    return np.array(res.x, dtype=float)
