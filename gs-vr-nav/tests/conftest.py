from __future__ import annotations

import re
import shutil
from pathlib import Path

import numpy as np
import pytest


def _rotation_matrix_y(degrees: float) -> np.ndarray:
    theta = np.deg2rad(degrees)
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    return np.array(
        [
            [cos_t, 0.0, sin_t],
            [0.0, 1.0, 0.0],
            [-sin_t, 0.0, cos_t],
        ],
        dtype=float,
    )


def pytest_addoption(parser):
    parser.addoption(
        "--run-slow",
        action="store_true",
        default=False,
        help="Run slow tests that require network access",
    )


def pytest_collection_modifyitems(config, items):
    if not config.getoption("--run-slow"):
        skip_slow = pytest.mark.skip(reason="Need --run-slow option to run")
        for item in items:
            if "slow" in item.keywords:
                item.add_marker(skip_slow)


@pytest.fixture
def tmp_path(request):
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", request.node.name)
    base_dir = Path(__file__).resolve().parents[1] / ".pytest-tmp-local"
    path = base_dir / safe_name
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)
    return path


@pytest.fixture
def uq_origin():
    return {"lat": -27.4975, "lon": 153.0137, "alt": 0.0}


@pytest.fixture
def synthetic_enu_path():
    np.random.seed(42)
    positions = []
    for i in range(20):
        e = i * 10.0
        n = 3.0 * np.sin(i * 0.3)
        u = 1.6
        positions.append([e, n, u])
    return np.array(positions)


@pytest.fixture
def synthetic_colmap_transform():
    return {
        "scale": 0.5,
        "rotation": _rotation_matrix_y(30),
        "translation": np.array([100, -50, 20]),
    }
