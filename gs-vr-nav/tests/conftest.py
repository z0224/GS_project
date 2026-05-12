"""
Pytest 共享 fixtures。
"""

from __future__ import annotations

import re
import shutil
from pathlib import Path

import numpy as np
import pytest


def _rotation_matrix_y(degrees: float) -> np.ndarray:
    """生成绕 Y 轴旋转的 3x3 矩阵，避免 fixture 依赖可选 SciPy 运行时。"""

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
    """在项目工作区内创建临时目录，避免受限系统 temp 目录影响离线测试。"""

    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", request.node.name)
    base_dir = Path(__file__).resolve().parents[1] / ".pytest-tmp-local"
    path = base_dir / safe_name
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)
    return path


@pytest.fixture
def uq_origin():
    """UQ St Lucia 校园中心坐标。"""

    return {"lat": -27.4975, "lon": 153.0137, "alt": 0.0}


@pytest.fixture
def synthetic_enu_path():
    """
    生成一条合成的步行路径（20个点），
    模拟在 UQ 校园中沿东向步行 200m 的采集过程。
    """

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
    """
    一个已知的 COLMAP→ENU 相似变换参数。
    用于生成合成测试数据。
    """

    return {
        "scale": 0.5,
        "rotation": _rotation_matrix_y(30),
        "translation": np.array([100, -50, 20]),
    }


@pytest.fixture
def synthetic_osm_data():
    """
    合成的 OSM 数据：1条道路 + 1条人行道 + 2个建筑。
    """

    from geo_alignment.osm_loader import OSMData
    from shapely.geometry import LineString, Polygon

    return OSMData(
        buildings=[
            Polygon([(40, 10), (60, 10), (60, 30), (40, 30)]),
            Polygon([(140, -20), (170, -20), (170, -5), (140, -5)]),
        ],
        roads=[
            LineString([(0, 0), (200, 0)]),
        ],
        sidewalks=[
            LineString([(0, 8), (200, 8)]),
        ],
        origin_lat=-27.4975,
        origin_lon=153.0137,
        radius_m=500,
    )


@pytest.fixture
def synthetic_nav_mesh(synthetic_osm_data):
    """从合成 OSM 数据生成的导航网格。"""

    from geo_alignment.nav_mesh import NavMesh

    return NavMesh.from_osm_data(
        synthetic_osm_data,
        road_buffer_m=5.0,
        sidewalk_buffer_m=2.0,
        collision_margin_m=0.3,
    )
