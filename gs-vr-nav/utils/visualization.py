"""Matplotlib visualization helpers for debugging GS-VR-Nav experiments.

This module defines plotting helpers for capture GPS traces and alignment
correspondences. These figures are intended for offline debugging rather than
runtime VR rendering.
"""

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np


def plot_gps_track(points_enu: np.ndarray, output_path: Path | None = None) -> Any:
    """Plot an ENU GPS track for capture debugging."""
    raise NotImplementedError()


if __name__ == "__main__":
    raise NotImplementedError("Command-line visualization is not implemented yet.")
