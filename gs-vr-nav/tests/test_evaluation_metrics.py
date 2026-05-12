"""
评估指标测试。

定义并验证项目的定量评估方法。
"""

from __future__ import annotations

import numpy as np


def test_alignment_accuracy_metric():
    """
    评估指标 1：对齐精度。

    度量：锚点 RMSE（米）。
    合格标准：RMSE < 5m（GPS精度限制下的合理阈值）；理想标准：RMSE < 2m。
    """

    from geo_alignment.procrustes import umeyama_alignment

    np.random.seed(42)

    source = np.random.randn(20, 3) * 50

    # 无噪声目标：相似变换应被精确恢复，RMSE 接近 0。
    transform_true = np.eye(4)
    transform_true[:3, :3] = 2.0 * np.eye(3)
    transform_true[:3, 3] = [10, 20, 30]
    target_clean = (transform_true[:3, :3] @ source.T).T + transform_true[:3, 3]
    _transform_clean, info_clean = umeyama_alignment(source, target_clean)
    assert info_clean["rmse"] < 0.001, "无噪声时 RMSE 应接近 0"

    # 1m GPS 噪声：合成误差应保持在项目理想标准附近。
    target_1m = target_clean + np.random.normal(0, 1.0, target_clean.shape)
    _transform_1m, info_1m = umeyama_alignment(source, target_1m)
    assert info_1m["rmse"] < 3.0, f"1m 噪声时 RMSE 应 < 3m, got {info_1m['rmse']:.2f}"

    # 3m GPS 噪声：允许 RMSE 上升，但仍应低于宽松验收阈值。
    target_3m = target_clean + np.random.normal(0, 3.0, target_clean.shape)
    _transform_3m, info_3m = umeyama_alignment(source, target_3m)
    assert info_3m["rmse"] < 8.0, f"3m 噪声时 RMSE 应 < 8m, got {info_3m['rmse']:.2f}"


def test_navigation_validity_metric(synthetic_nav_mesh):
    """
    评估指标 2：导航合理性。

    度量：随机采样 N 个可行走点中，真正位于非障碍区域的比例。
    合格标准：100%（by construction，因为可行走区域由道路/人行道 buffer 扣除建筑生成）。
    """

    from shapely.geometry import Point

    nav_mesh = synthetic_nav_mesh
    min_e, min_n, max_e, max_n = nav_mesh.bounds

    np.random.seed(42)
    walkable_points = []
    attempts = 0
    while len(walkable_points) < 100 and attempts < 10000:
        e = np.random.uniform(min_e, max_e)
        n = np.random.uniform(min_n, max_n)
        if nav_mesh.is_walkable(e, n):
            walkable_points.append((e, n))
        attempts += 1

    assert len(walkable_points) >= 50, f"只找到 {len(walkable_points)} 个可行走点，导航网格可能太小"

    # 所有可行走点都不应在建筑障碍区域内。
    for e, n in walkable_points:
        point = Point(e, n)
        assert not nav_mesh.obstacle_area.contains(point), f"可行走点 ({e:.1f}, {n:.1f}) 位于障碍区域内！"


def test_movement_constraint_metric(synthetic_nav_mesh):
    """
    评估指标 3：移动约束有效性。

    度量：模拟 1000 次随机移动，验证所有 clamp 后的位置都是可行走的。
    合格标准：0 次违规。
    """

    nav_mesh = synthetic_nav_mesh

    np.random.seed(42)

    # 从一个已知可行走点开始，模拟 Unity 中连续手柄/键盘移动。
    current = (50.0, 0.0)
    assert nav_mesh.is_walkable(*current)

    violations = 0
    for _ in range(1000):
        angle = np.random.uniform(0, 2 * np.pi)
        dist = np.random.uniform(0, 5)
        dx = dist * np.cos(angle)
        dy = dist * np.sin(angle)

        proposed = (current[0] + dx, current[1] + dy)
        clamped = nav_mesh.clamp_movement(current[0], current[1], proposed[0], proposed[1])

        if not nav_mesh.is_walkable(clamped[0], clamped[1]):
            violations += 1

        # 只从合法位置继续，避免一次失败污染后续随机游走。
        if nav_mesh.is_walkable(clamped[0], clamped[1]):
            current = clamped

    assert violations == 0, f"{violations}/1000 次 clamp 后仍不可行走--clamp_movement 实现有误"


def test_splat_coverage_metric(synthetic_nav_mesh):
    """
    评估指标 4：Splat 覆盖度。

    度量：可行走路径上每 1m 采样一个点，检查 10m 内是否有 splat。
    合格标准：>80% 路径有覆盖，保证用户沿路行走时有视觉内容。
    """

    np.random.seed(42)
    n_splats = 500

    # splats 集中在道路附近，模拟重建点覆盖主路径。
    splat_positions = np.column_stack(
        [
            np.random.uniform(0, 200, n_splats),
            np.random.normal(0, 5, n_splats),
            np.random.uniform(0, 5, n_splats),
        ]
    ).astype(np.float32)

    # 沿道路中心线每 1m 采样，统计可行走点的视觉覆盖比例。
    path_points = [(e, 0) for e in range(0, 200, 1)]

    covered = 0
    for px, py in path_points:
        if not synthetic_nav_mesh.is_walkable(px, py):
            continue
        dists = np.sqrt((splat_positions[:, 0] - px) ** 2 + (splat_positions[:, 1] - py) ** 2)
        if np.min(dists) < 10.0:
            covered += 1

    walkable_path_points = sum(1 for px, py in path_points if synthetic_nav_mesh.is_walkable(px, py))

    if walkable_path_points > 0:
        coverage = covered / walkable_path_points
        assert coverage > 0.8, f"Splat 覆盖度仅 {coverage:.1%}，应 > 80%"


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
