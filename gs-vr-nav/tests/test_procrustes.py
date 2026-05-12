"""Tests for GPS-to-scene Procrustes alignment."""

from __future__ import annotations

import math

import numpy as np
import pytest

from geo_alignment.procrustes import (
    AlignmentResult,
    compute_anchor_pairs,
    quat_to_rotation_matrix,
    umeyama_alignment,
)


def _random_points(count: int) -> np.ndarray:
    return np.random.normal(size=(count, 3))


def _rotation_z(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def _rotation_y(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ]
    )


@pytest.fixture(autouse=True)
def fixed_random_seed() -> None:
    """每个测试都固定随机种子，保证数值测试可复现。"""
    np.random.seed(42)


def test_identity_transform() -> None:
    """验证 source 与 target 完全相同时，应恢复单位相似变换。"""
    source = _random_points(10)
    target = source.copy()

    transform, info = umeyama_alignment(source, target)

    np.testing.assert_allclose(transform, np.eye(4), atol=1e-10)
    assert info["scale"] == pytest.approx(1.0, abs=1e-10)
    assert info["rmse"] == pytest.approx(0.0, abs=1e-10)


def test_known_translation() -> None:
    """验证纯平移场景下，算法能恢复正确的平移向量。"""
    source = _random_points(10)
    translation = np.array([100.0, 200.0, 300.0])
    target = source + translation

    _, info = umeyama_alignment(source, target)

    np.testing.assert_allclose(info["translation"], translation, atol=1e-8)
    np.testing.assert_allclose(info["rotation"], np.eye(3), atol=1e-8)
    assert info["scale"] == pytest.approx(1.0, abs=1e-8)
    assert info["rmse"] < 1e-6


def test_known_rotation() -> None:
    """验证绕 Z 轴 90 度旋转时，能恢复正确的旋转角度。"""
    source = _random_points(10)
    target = source @ _rotation_z(math.pi / 2.0).T

    _, info = umeyama_alignment(source, target)
    recovered_angle = math.atan2(info["rotation"][1, 0], info["rotation"][0, 0])

    assert info["rmse"] < 1e-6
    assert recovered_angle == pytest.approx(math.pi / 2.0, abs=1e-6)


def test_known_scale() -> None:
    """验证纯缩放场景下，算法能恢复真实比例因子。"""
    source = _random_points(10)
    target = source * 2.5

    _, info = umeyama_alignment(source, target)

    assert info["scale"] == pytest.approx(2.5, abs=1e-8)
    assert info["rmse"] < 1e-6


def test_combined_transform() -> None:
    """验证缩放、旋转、平移组合变换下，算法能恢复完整 4x4 矩阵。"""
    source = _random_points(20)
    true_scale = 3.0
    true_rotation = _rotation_y(math.radians(45.0))
    true_translation = np.array([10.0, -20.0, 30.0])
    target = true_scale * (source @ true_rotation.T) + true_translation

    true_transform = np.eye(4)
    true_transform[:3, :3] = true_scale * true_rotation
    true_transform[:3, 3] = true_translation

    transform, info = umeyama_alignment(source, target)

    assert abs(info["scale"] - true_scale) < 0.01
    assert info["rmse"] < 1e-4
    np.testing.assert_allclose(transform, true_transform, atol=0.01)


def test_with_noise() -> None:
    """验证带 0.5m GPS 高斯噪声时，RMSE 与缩放误差仍在合理范围内。"""
    source = _random_points(20)
    true_scale = 1.8
    true_rotation = _rotation_y(math.radians(-20.0))
    true_translation = np.array([5.0, 15.0, -2.0])
    clean_target = true_scale * (source @ true_rotation.T) + true_translation
    target = clean_target + np.random.normal(scale=0.5, size=clean_target.shape)

    _, info = umeyama_alignment(source, target)

    assert 0.3 <= info["rmse"] <= 1.0
    assert abs(info["scale"] - true_scale) / true_scale < 0.1


def test_insufficient_points() -> None:
    """验证少于 3 个锚点时会抛出 ValueError。"""
    source = _random_points(2)
    target = source.copy()

    with pytest.raises(ValueError):
        umeyama_alignment(source, target)


def test_collinear_points() -> None:
    """验证所有锚点共线时算法不会崩溃。"""
    x = np.linspace(-5.0, 5.0, 10)
    source = np.column_stack([x, np.zeros_like(x), np.zeros_like(x)])
    target = source * 2.0 + np.array([1.0, 2.0, 3.0])

    _, info = umeyama_alignment(source, target)

    assert np.isfinite(info["rmse"])
    assert info["num_anchors"] == 10


def test_quat_to_rotation_matrix() -> None:
    """验证 COLMAP 四元数能正确转换为旋转矩阵。"""
    np.testing.assert_allclose(quat_to_rotation_matrix(np.array([1.0, 0.0, 0.0, 0.0])), np.eye(3), atol=1e-10)

    qvec = np.array([math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0)])
    np.testing.assert_allclose(quat_to_rotation_matrix(qvec), _rotation_z(math.pi / 2.0), atol=1e-10)


def test_alignment_result_report() -> None:
    """验证 AlignmentResult.report() 返回包含核心质量指标的报告文本。"""
    result = AlignmentResult(
        transform=np.eye(4),
        rotation=np.eye(3),
        scale=1.0,
        translation=np.zeros(3),
        rmse=0.25,
        residuals=np.array([0.1, 0.2, 0.3]),
        num_anchors=3,
        anchor_source=np.zeros((3, 3)),
        anchor_target=np.zeros((3, 3)),
    )

    report = result.report()

    assert "GS-VR-Nav Alignment Report" in report
    assert "Anchor points used: 3" in report
    assert "Scale factor: 1.000000" in report
    assert "RMSE: 0.250 m" in report
    assert "Per-anchor residuals:" in report


def test_alignment_result_save_load(tmp_path) -> None:
    """验证 AlignmentResult.save() 生成的 npz 文件可正确读回。"""
    result = AlignmentResult(
        transform=np.eye(4),
        rotation=np.eye(3),
        scale=2.0,
        translation=np.array([1.0, 2.0, 3.0]),
        rmse=0.0,
        residuals=np.array([0.0, 0.1, 0.2]),
        num_anchors=3,
        anchor_source=_random_points(3),
        anchor_target=_random_points(3),
    )
    output_path = tmp_path / "alignment.npz"

    result.save(output_path)
    loaded = np.load(output_path)

    np.testing.assert_allclose(loaded["transform"], result.transform)
    np.testing.assert_allclose(loaded["rotation"], result.rotation)
    assert float(loaded["scale"]) == pytest.approx(result.scale)
    np.testing.assert_allclose(loaded["translation"], result.translation)
    np.testing.assert_allclose(loaded["residuals"], result.residuals)
    np.testing.assert_allclose(loaded["anchor_source"], result.anchor_source)
    np.testing.assert_allclose(loaded["anchor_target"], result.anchor_target)


def test_compute_anchor_pairs() -> None:
    """验证 transforms.json 与 COLMAP 字典只提取双方都存在的锚点对。"""
    transforms_json = {
        "frames": [
            {"file_path": "images/a.jpg", "enu": {"e": 1.0, "n": 2.0, "u": 3.0}},
            {"file_path": "images/b.jpg", "enu": {"e": 4.0, "n": 5.0, "u": 6.0}},
            {"file_path": "images/missing_colmap.jpg", "enu": {"e": 7.0, "n": 8.0, "u": 9.0}},
            {"file_path": "images/missing_enu.jpg"},
        ]
    }
    colmap_images = {
        "a.jpg": {"position": np.array([10.0, 20.0, 30.0])},
        "images/b.jpg": {"position": np.array([40.0, 50.0, 60.0])},
        "missing_enu.jpg": {"position": np.array([70.0, 80.0, 90.0])},
    }

    source, target = compute_anchor_pairs(transforms_json, colmap_images)

    assert source.shape == (2, 3)
    assert target.shape == (2, 3)
    np.testing.assert_allclose(source, np.array([[10.0, 20.0, 30.0], [40.0, 50.0, 60.0]]))
    np.testing.assert_allclose(target, np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))
