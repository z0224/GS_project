"""WGS84 <-> local ENU coordinate conversion utilities."""

from __future__ import annotations

from functools import lru_cache
from typing import Any

import numpy as np
from pyproj import CRS, Transformer

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def build_wgs84_crs() -> CRS:
    """Build the WGS84 geographic 3D coordinate reference system."""
    return CRS.from_epsg(4979)


@lru_cache(maxsize=1)
def _ecef_to_wgs84_transformer() -> Transformer:
    return Transformer.from_crs(CRS.from_epsg(4978), CRS.from_epsg(4979), always_xy=True)


def _origin_rotation(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    return np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )


def wgs84_to_ecef(
    lat_deg: float,
    lon_deg: float,
    altitude_m: float,
) -> tuple[float, float, float]:
    """Convert WGS84 geodetic coordinates to ECEF coordinates."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    prime_vertical_radius = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (prime_vertical_radius + altitude_m) * cos_lat * np.cos(lon)
    y = (prime_vertical_radius + altitude_m) * cos_lat * np.sin(lon)
    z = (prime_vertical_radius * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return float(x), float(y), float(z)


def gps_to_enu(
    lat: float,
    lon: float,
    alt: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float = 0.0,
) -> tuple[float, float, float]:
    """Convert WGS84 coordinates to local East-North-Up coordinates in meters."""
    # CN: 先转到地心 ECEF，再减去原点并旋转到局部 ENU；这样经纬度就变成米。
    # EN: Convert to ECEF, subtract the origin, then rotate into ENU so lat/lon becomes meters.
    point_ecef = np.array(wgs84_to_ecef(lat, lon, alt), dtype=float)
    origin_ecef = np.array(wgs84_to_ecef(origin_lat, origin_lon, origin_alt), dtype=float)
    enu = _origin_rotation(origin_lat, origin_lon) @ (point_ecef - origin_ecef)
    return float(enu[0]), float(enu[1]), float(enu[2])


def enu_to_gps(
    e: float,
    n: float,
    u: float,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float = 0.0,
) -> tuple[float, float, float]:
    """Convert local East-North-Up coordinates in meters back to WGS84."""
    origin_ecef = np.array(wgs84_to_ecef(origin_lat, origin_lon, origin_alt), dtype=float)
    enu = np.array([e, n, u], dtype=float)
    point_ecef = origin_ecef + _origin_rotation(origin_lat, origin_lon).T @ enu
    lon, lat, alt = _ecef_to_wgs84_transformer().transform(
        point_ecef[0], point_ecef[1], point_ecef[2]
    )
    return float(lat), float(lon), float(alt)


def heading_to_rotation_matrix(heading_deg: float) -> np.ndarray:
    """Convert compass heading to a 3x3 rotation matrix in ENU coordinates."""
    # CN: heading 是平面方向角，这里只绕 Up 轴旋转，不改变高度方向。
    # EN: Heading is a horizontal bearing, so this rotates around the Up axis only.
    heading = np.radians(heading_deg)
    cos_h = np.cos(heading)
    sin_h = np.sin(heading)
    return np.array(
        [
            [cos_h, sin_h, 0.0],
            [-sin_h, cos_h, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def batch_gps_to_enu(coords: np.ndarray, origin_lat: float, origin_lon: float) -> np.ndarray:
    """Convert an array of lat/lon(/alt) points to an ``(N, 3)`` ENU array."""
    points = np.asarray(coords, dtype=float)
    if points.ndim != 2 or points.shape[1] not in (2, 3):
        raise ValueError("coords must have shape (N, 2) or (N, 3)")

    if points.shape[1] == 2:
        points = np.column_stack([points, np.zeros(points.shape[0], dtype=float)])

    return np.array(
        [gps_to_enu(lat, lon, alt, origin_lat, origin_lon, 0.0) for lat, lon, alt in points],
        dtype=float,
    )


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    altitude_m: float,
    origin_wgs84: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Convert WGS84 geodetic coordinates to local ENU coordinates."""
    return gps_to_enu(lat_deg, lon_deg, altitude_m, *origin_wgs84)


def enu_to_geodetic(
    east_m: float,
    north_m: float,
    up_m: float,
    origin_wgs84: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Convert local ENU coordinates to WGS84 geodetic coordinates."""
    return enu_to_gps(east_m, north_m, up_m, *origin_wgs84)


def transform_points_wgs84_to_enu(
    points_wgs84: np.ndarray,
    origin_wgs84: tuple[float, float, float],
) -> np.ndarray:
    """Convert an array of WGS84 points to local ENU coordinates."""
    points = np.asarray(points_wgs84, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_wgs84 must have shape (N, 3)")
    return np.array([geodetic_to_enu(lat, lon, alt, origin_wgs84) for lat, lon, alt in points])


def build_transformers(origin_wgs84: tuple[float, float, float]) -> dict[str, Transformer | Any]:
    """Build reusable pyproj transformers for the configured local origin."""
    return {
        "wgs84": build_wgs84_crs(),
        "ecef": CRS.from_epsg(4978),
        "ecef_to_wgs84": _ecef_to_wgs84_transformer(),
        "origin_wgs84": origin_wgs84,
        "origin_ecef": wgs84_to_ecef(*origin_wgs84),
        "ecef_to_enu_rotation": _origin_rotation(origin_wgs84[0], origin_wgs84[1]),
    }


if __name__ == "__main__":
    raise SystemExit("Use data/capture_pipeline.py for capture metadata processing.")
