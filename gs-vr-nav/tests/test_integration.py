"""
端到端集成测试。

验证完整管线：EXIF提取 → ENU转换 → (模拟COLMAP) → Procrustes对齐
              → OSM加载 → 导航网格 → 导航约束。
"""

from __future__ import annotations

import json

import numpy as np
import pytest
from shapely.geometry import LineString, Point, Polygon


def _rotation_matrix_y(degrees: float) -> np.ndarray:
    """生成绕 Y 轴旋转的 3x3 矩阵，用于构造合成 COLMAP 坐标系。"""

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


def _rotation_matrix_xyz(degrees_xyz: list[float]) -> np.ndarray:
    """按 scipy Rotation.from_euler('xyz', ...) 约定生成合成测试旋转。"""

    x_deg, y_deg, z_deg = degrees_xyz
    x, y, z = np.deg2rad([x_deg, y_deg, z_deg])
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(x), -np.sin(x)],
            [0.0, np.sin(x), np.cos(x)],
        ],
        dtype=float,
    )
    ry = np.array(
        [
            [np.cos(y), 0.0, np.sin(y)],
            [0.0, 1.0, 0.0],
            [-np.sin(y), 0.0, np.cos(y)],
        ],
        dtype=float,
    )
    rz = np.array(
        [
            [np.cos(z), -np.sin(z), 0.0],
            [np.sin(z), np.cos(z), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return rz @ ry @ rx


def _create_synthetic_splats(colmap_positions: np.ndarray, n_splats: int = 1000):
    """在 COLMAP 路径附近生成合成 Gaussian splats。"""

    from reconstruction.export import GaussianSplatData

    # 沿相机轨迹随机取基准点，再添加局部扰动，模拟路径附近的重建点云。
    anchor_indices = np.random.randint(0, len(colmap_positions), size=n_splats)
    positions = colmap_positions[anchor_indices] + np.random.normal(0.0, 3.0, (n_splats, 3))

    rotations = np.random.randn(n_splats, 4).astype(np.float32)
    rotations /= np.linalg.norm(rotations, axis=1, keepdims=True)
    return GaussianSplatData(
        positions=positions.astype(np.float32),
        colors_sh=np.random.randn(n_splats, 3).astype(np.float32),
        opacities=np.random.uniform(0.05, 0.95, n_splats).astype(np.float32),
        scales=np.random.randn(n_splats, 3).astype(np.float32),
        rotations=rotations,
    )


def test_full_pipeline_with_synthetic_data():
    """
    使用完全离线的合成数据验证端到端管线。

    评估指标：Procrustes 锚点 RMSE、scale 恢复误差、splat 与导航网格的空间一致性。
    合格标准：RMSE < 1m，scale 误差 < 5%，至少 30% splats 位于可行走区域或 10m 邻域内。
    """

    from geo_alignment.nav_mesh import NavMesh
    from geo_alignment.osm_loader import OSMData
    from geo_alignment.procrustes import umeyama_alignment
    from reconstruction.export import apply_transform

    np.random.seed(42)
    origin_lat, origin_lon = -27.4975, 153.0137

    # 在 UQ St Lucia 附近构造一条 200m 的轻微蛇形采集路径。
    enu_positions = []
    for i in range(20):
        e = i * 10.0
        n = 5.0 * np.sin(i * 0.3)
        u = 1.6
        enu_positions.append([e, n, u])
    enu_positions = np.array(enu_positions)

    # 模拟 COLMAP 的任意相似坐标系：ENU = scale * COLMAP * R.T + translation。
    gt_scale = 0.5
    gt_rotation = _rotation_matrix_y(30)
    gt_translation = np.array([100.0, -50.0, 20.0])
    colmap_positions = (enu_positions - gt_translation) @ gt_rotation / gt_scale
    colmap_positions += np.random.normal(0.0, 0.1, colmap_positions.shape)

    # 估计 COLMAP→ENU 对齐变换，并验证误差在合成噪声可解释范围内。
    transform, info = umeyama_alignment(colmap_positions, enu_positions)
    print(
        "\nAlignment report\n"
        f"  anchors: {info['num_anchors']}\n"
        f"  scale: {info['scale']:.6f}\n"
        f"  rmse: {info['rmse']:.3f} m\n"
        f"  max residual: {np.max(info['residuals']):.3f} m"
    )
    assert info["rmse"] < 1.0
    assert abs(info["scale"] - gt_scale) / gt_scale < 0.05

    # 将合成 splats 从 COLMAP 坐标系变换到 ENU 坐标系。
    splat_data = _create_synthetic_splats(colmap_positions, n_splats=1000)
    aligned_splats = apply_transform(splat_data, transform)

    # 构造一个离线 OSM 场景：一条道路和一个建筑障碍物。
    road = LineString([(0, 0), (200, 0)])
    building = Polygon([(80, 15), (120, 15), (120, 35), (80, 35)])
    osm_data = OSMData(
        buildings=[building],
        roads=[road],
        sidewalks=[],
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        radius_m=500,
    )
    nav_mesh = NavMesh.from_osm_data(osm_data, road_buffer_m=5.0)

    # 道路区域可行走，建筑和远离道路区域不可行走。
    assert nav_mesh.is_walkable(50, 0)
    assert nav_mesh.is_walkable(150, 0)
    assert not nav_mesh.is_walkable(100, 25)
    assert not nav_mesh.is_walkable(50, 50)

    # 对齐后的 splats 应主要落在路径附近，因此接近可行走区域。
    splat_positions_2d = aligned_splats.positions[:, [0, 1]]
    near_walkable = 0
    for pos in splat_positions_2d:
        if nav_mesh.is_walkable(pos[0], pos[1]):
            near_walkable += 1
        else:
            nearest = nav_mesh.clamp_to_walkable(pos[0], pos[1])
            dist = np.sqrt((pos[0] - nearest[0]) ** 2 + (pos[1] - nearest[1]) ** 2)
            if dist < 10.0:
                near_walkable += 1
    assert near_walkable / len(splat_positions_2d) > 0.3

    # 模拟从道路向建筑移动，约束结果必须留在可行走区域且不进入建筑。
    start = (50, 0)
    end = (100, 25)
    clamped = nav_mesh.clamp_movement(start[0], start[1], end[0], end[1])
    assert nav_mesh.is_walkable(clamped[0], clamped[1])
    assert not building.contains(Point(clamped[0], clamped[1]))


@pytest.mark.slow
def test_osm_download_and_nav_mesh():
    """
    使用真实 OSM 数据测试导航网格生成。

    评估指标：真实校园建筑/道路解析数量、可行走与障碍区域面积、随机采样一致性。
    合格标准：UQ 500m 半径内应解析出足够道路建筑，且 clamp 后的位置可行走。
    """

    from geo_alignment.nav_mesh import NavMesh
    from geo_alignment.osm_loader import download_osm_data

    np.random.seed(42)
    osm_data = download_osm_data(center_lat=-27.4975, center_lon=153.0137, radius_m=500)

    assert len(osm_data.buildings) > 10, "UQ 校园应该有至少 10 个建筑"
    assert len(osm_data.roads) > 5, "UQ 校园应该有至少 5 条道路"

    nav_mesh = NavMesh.from_osm_data(osm_data)
    assert nav_mesh.walkable_area.area > 0
    assert nav_mesh.obstacle_area.area > 0

    # 随机采样验证：可行走点不应落在任意建筑内部。
    min_e, min_n, max_e, max_n = nav_mesh.bounds
    walkable_count = 0
    for _ in range(100):
        e = np.random.uniform(min_e, max_e)
        n = np.random.uniform(min_n, max_n)
        if nav_mesh.is_walkable(e, n):
            walkable_count += 1
            p = Point(e, n)
            for bldg in osm_data.buildings:
                assert not bldg.contains(p), f"Walkable point ({e:.1f}, {n:.1f}) is inside a building!"

    assert walkable_count > 5, (
        f"Only {walkable_count}/100 random points are walkable - nav mesh may be too restrictive"
    )

    # 建筑中心点 clamp 后应回到建筑外的可行走区域。
    if len(osm_data.buildings) > 0:
        bldg_center = osm_data.buildings[0].centroid
        clamped = nav_mesh.clamp_to_walkable(bldg_center.x, bldg_center.y)
        assert nav_mesh.is_walkable(clamped[0], clamped[1])


def test_coordinate_pipeline_consistency():
    """
    验证 GPS → ENU → COLMAP → (Procrustes) → ENU' 的全程一致性。

    评估指标：恢复后 ENU 坐标 RMSE、最大误差、ENU→GPS 回转误差。
    合格标准：RMSE < 0.5m，最大 ENU 误差 < 1m，经纬度回转误差 < 0.001 度。
    """

    from geo_alignment.procrustes import umeyama_alignment
    from utils.coordinate_utils import enu_to_gps, gps_to_enu

    np.random.seed(42)
    origin_lat, origin_lon = -27.4975, 153.0137

    # 在原点附近 500m 范围内生成 GPS 锚点。
    lats = origin_lat + np.random.uniform(-0.003, 0.003, 15)
    lons = origin_lon + np.random.uniform(-0.003, 0.003, 15)

    # GPS 转为局部 ENU，作为真实世界目标坐标。
    enu_coords = np.array([gps_to_enu(lat, lon, 0, origin_lat, origin_lon) for lat, lon in zip(lats, lons)])

    # 施加随机相似变换，模拟 COLMAP 重建坐标。
    r_true = _rotation_matrix_xyz([15, -30, 45])
    s_true = 0.7
    t_true = np.array([50.0, -100.0, 30.0])
    colmap_coords = (enu_coords @ r_true.T) * s_true + t_true
    colmap_coords += np.random.normal(0.0, 0.05, colmap_coords.shape)

    # 恢复 COLMAP→ENU 的相似变换。
    transform, info = umeyama_alignment(colmap_coords, enu_coords)
    assert info["rmse"] < 0.5, f"RMSE too high: {info['rmse']:.3f}m"
    assert abs(info["scale"] - 1 / s_true) / (1 / s_true) < 0.05, (
        f"Scale error too high: {info['scale']:.4f} vs expected {1 / s_true:.4f}"
    )

    recovered_enu = (transform[:3, :3] @ colmap_coords.T).T + transform[:3, 3]
    errors = np.linalg.norm(recovered_enu - enu_coords, axis=1)
    assert np.max(errors) < 1.0, f"Max error: {np.max(errors):.3f}m"

    # 回转到 GPS，确认 Unity/地理管线中使用的 ENU 原点没有漂移。
    for i in range(len(enu_coords)):
        lat_r, lon_r, _alt_r = enu_to_gps(
            recovered_enu[i, 0],
            recovered_enu[i, 1],
            recovered_enu[i, 2],
            origin_lat,
            origin_lon,
        )
        assert abs(lat_r - lats[i]) < 0.001, f"Lat roundtrip error at point {i}"
        assert abs(lon_r - lons[i]) < 0.001, f"Lon roundtrip error at point {i}"


def test_nav_mesh_serialization_for_unity():
    """
    验证 NavMesh.to_json() 输出的格式可以被 Unity 端 GeoAlignmentLoader.cs 解析。

    评估指标：GeoJSON-like MultiPolygon 坐标结构、bounds、JSON roundtrip、反序列化行为一致性。
    合格标准：所有字段存在，坐标为 [x, y] 数值列表，from_json 后查询结果一致。
    """

    from geo_alignment.nav_mesh import NavMesh
    from geo_alignment.osm_loader import OSMData

    road = LineString([(0, 0), (100, 0)])
    building = Polygon([(40, 10), (60, 10), (60, 30), (40, 30)])
    osm_data = OSMData(
        buildings=[building],
        roads=[road],
        sidewalks=[],
        origin_lat=-27.4975,
        origin_lon=153.0137,
        radius_m=500,
    )

    nav_mesh = NavMesh.from_osm_data(osm_data)
    json_data = nav_mesh.to_json()

    assert "walkable_area" in json_data
    assert "obstacle_area" in json_data
    assert "bounds" in json_data

    wa = json_data["walkable_area"]
    assert wa["type"] == "MultiPolygon"
    assert isinstance(wa["coordinates"], list)
    assert len(wa["coordinates"]) > 0

    # Unity 端按 MultiPolygon → Polygon rings → [x, y] 逐层读取。
    for polygon_coords in wa["coordinates"]:
        assert isinstance(polygon_coords, list)
        assert len(polygon_coords) >= 1
        outer_ring = polygon_coords[0]
        assert isinstance(outer_ring, list)
        assert len(outer_ring) >= 4
        for point in outer_ring:
            assert isinstance(point, list)
            assert len(point) == 2
            assert isinstance(point[0], (int, float))
            assert isinstance(point[1], (int, float))

    assert len(json_data["bounds"]) == 4
    min_e, min_n, max_e, max_n = json_data["bounds"]
    assert max_e > min_e
    assert max_n > min_n

    json_str = json.dumps(json_data)
    reloaded = json.loads(json_str)
    assert reloaded == json_data

    nav_mesh_2 = NavMesh.from_json(reloaded)
    test_points = [(50, 0), (50, 20), (0, 0), (100, 50)]
    for x, y in test_points:
        assert nav_mesh.is_walkable(x, y) == nav_mesh_2.is_walkable(x, y), f"Mismatch at ({x}, {y})"


def test_ply_transform_and_export(tmp_path):
    """
    验证 splat 数据的 .ply 加载 → 变换 → 保存 → 重新加载完整流程。

    评估指标：PLY roundtrip 位置精度、相似变换位置结果、bounding box 尺寸变化。
    合格标准：位置误差在 1e-5 到 1e-4 内，scale=2 后 bounding box 尺寸增大。
    """

    from reconstruction.export import GaussianSplatData, apply_transform, get_bounding_box, load_ply, save_ply

    np.random.seed(42)

    # 创建合成 splat 数据，模拟 COLMAP 坐标系中的重建输出。
    n_splats = 500
    rotations = np.random.randn(n_splats, 4).astype(np.float32)
    rotations /= np.linalg.norm(rotations, axis=1, keepdims=True)
    data = GaussianSplatData(
        positions=(np.random.randn(n_splats, 3).astype(np.float32) * 10),
        colors_sh=np.random.randn(n_splats, 3).astype(np.float32),
        opacities=np.random.rand(n_splats).astype(np.float32),
        scales=np.random.randn(n_splats, 3).astype(np.float32),
        rotations=rotations,
    )

    original_path = tmp_path / "original.ply"
    save_ply(data, original_path)

    loaded = load_ply(original_path)
    np.testing.assert_allclose(loaded.positions, data.positions, atol=1e-5)

    # 构造 Procrustes 风格的 4x4 相似变换。
    transform = np.eye(4)
    transform[:3, :3] = 2.0 * np.eye(3)
    transform[:3, 3] = [100, 200, 300]

    transformed = apply_transform(loaded, transform)
    expected_positions = loaded.positions * 2.0 + np.array([100, 200, 300])
    np.testing.assert_allclose(transformed.positions, expected_positions, atol=1e-4)

    bb_orig = get_bounding_box(loaded)
    bb_trans = get_bounding_box(transformed)
    assert np.all(bb_trans[1] - bb_trans[0] > bb_orig[1] - bb_orig[0])

    transformed_path = tmp_path / "aligned.ply"
    save_ply(transformed, transformed_path)

    reloaded = load_ply(transformed_path)
    np.testing.assert_allclose(reloaded.positions, transformed.positions, atol=1e-5)


# 运行所有快速测试：
#   pytest tests/ -v --ignore=tests/test_integration.py -k "not slow"
#
# 运行包括慢速测试在内的所有测试：
#   pytest tests/ -v --run-slow
#
# 运行集成测试（离线部分）：
#   pytest tests/test_integration.py -v -k "not slow"
#
# 运行集成测试（含网络依赖）：
#   pytest tests/test_integration.py -v --run-slow
#
# 生成覆盖率报告：
#   pytest tests/ -v --cov=. --cov-report=html
