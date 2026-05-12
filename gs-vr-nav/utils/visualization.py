"""Matplotlib visualization helpers for debugging GS-VR-Nav experiments.

This module defines plotting helpers for capture GPS traces, OSM geometry,
alignment correspondences, and generated navigation meshes. These figures are
intended for offline debugging rather than runtime VR rendering.
"""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry.base import BaseGeometry


def plot_gps_track(points_enu: np.ndarray, output_path: Path | None = None) -> Any:
    """Plot an ENU GPS track for capture debugging."""
    raise NotImplementedError()


def plot_osm_alignment(
    osm_geometry: BaseGeometry,
    splat_points_enu: np.ndarray,
    output_path: Path | None = None,
) -> Any:
    """Plot OSM geometry against aligned reconstruction points."""
    raise NotImplementedError()


def plot_nav_mesh(nav_mesh: BaseGeometry, output_path: Path | None = None) -> Any:
    """Plot the generated navigation mesh."""
    raise NotImplementedError()


if __name__ == "__main__":
    raise NotImplementedError("Command-line visualization is not implemented yet.")
