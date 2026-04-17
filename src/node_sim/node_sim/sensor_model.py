import math
import random
from typing import Optional, Tuple

import numpy as np

EARTH_RADIUS_M = 6371000.0


def latlon_to_enu(
    lat: float,
    lon: float,
    alt: float,
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> np.ndarray:
    """Convert geodetic coordinates to ENU using spherical Earth approximation."""
    lat_rad = math.radians(lat)
    ref_lat_rad = math.radians(ref_lat)
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    east = EARTH_RADIUS_M * d_lon * math.cos(ref_lat_rad)
    north = EARTH_RADIUS_M * d_lat
    up = alt - ref_alt
    return np.array([east, north, up], dtype=float)


def enu_to_latlon(
    east: float,
    north: float,
    up: float,
    ref_lat: float,
    ref_lon: float,
    ref_alt: float,
) -> Tuple[float, float, float]:
    """Convert ENU to geodetic coordinates using spherical Earth approximation."""
    ref_lat_rad = math.radians(ref_lat)
    d_lat = north / EARTH_RADIUS_M
    d_lon = east / (EARTH_RADIUS_M * max(math.cos(ref_lat_rad), 1e-9))
    lat = ref_lat + math.degrees(d_lat)
    lon = ref_lon + math.degrees(d_lon)
    alt = ref_alt + up
    return lat, lon, alt


def enu_to_az_el(target_enu: np.ndarray, sensor_enu: np.ndarray) -> Tuple[float, float, float]:
    """Return azimuth deg, elevation deg, and range from sensor to target."""
    delta = target_enu - sensor_enu
    east, north, up = float(delta[0]), float(delta[1]), float(delta[2])
    horizontal = math.sqrt(east * east + north * north)
    rng = math.sqrt(horizontal * horizontal + up * up)
    azimuth_deg = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
    elevation_deg = math.degrees(math.atan2(up, max(horizontal, 1e-9)))
    return azimuth_deg, elevation_deg, rng


class SensorModel:
    """Shared sensor model for virtual node detections."""

    def __init__(
        self,
        az_sigma_deg: float = 0.5,
        el_sigma_deg: float = 1.0,
        horizontal_fov_deg: float = 120.0,
        max_elevation_deg: float = 60.0,
    ) -> None:
        self.az_sigma_deg = az_sigma_deg
        self.el_sigma_deg = el_sigma_deg
        self.horizontal_fov_deg = horizontal_fov_deg
        self.max_elevation_deg = max_elevation_deg

    def detection_probability(self, rng_m: float) -> float:
        return math.exp(-max(rng_m, 0.0) / 500.0)

    def in_fov(self, azimuth_deg: float, elevation_deg: float, heading_deg: float = 0.0) -> bool:
        rel_az = ((azimuth_deg - heading_deg + 540.0) % 360.0) - 180.0
        return (
            abs(rel_az) <= self.horizontal_fov_deg * 0.5
            and 0.0 <= elevation_deg <= self.max_elevation_deg
        )

    def sample_detection(
        self,
        azimuth_deg: float,
        elevation_deg: float,
        rng_m: float,
        heading_deg: float = 0.0,
    ) -> Optional[Tuple[float, float, float]]:
        """Return noisy az/el/confidence or None for missed detection."""
        if not self.in_fov(azimuth_deg, elevation_deg, heading_deg=heading_deg):
            return None
        p_detect = self.detection_probability(rng_m)
        if random.random() > p_detect:
            return None
        noisy_az = random.gauss(azimuth_deg, self.az_sigma_deg)
        noisy_el = random.gauss(elevation_deg, self.el_sigma_deg)
        confidence = float(np.clip(p_detect, 0.05, 1.0))
        return noisy_az, noisy_el, confidence
