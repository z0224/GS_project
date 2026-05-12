from pathlib import Path

import numpy as np

from reconstruction.export import (
    GaussianSplatData,
    apply_transform,
    get_bounding_box,
    load_ply,
    save_ply,
    subsample,
)


def create_test_ply(tmp_path: Path, n_points: int, sh_degree: int = 0) -> tuple[Path, GaussianSplatData]:
    rng = np.random.default_rng(42)
    n_coefficients = 3 * (sh_degree + 1) ** 2
    rotations = rng.normal(size=(n_points, 4)).astype(np.float32)
    rotations /= np.linalg.norm(rotations, axis=1, keepdims=True)
    data = GaussianSplatData(
        positions=rng.normal(size=(n_points, 3)).astype(np.float32),
        colors_sh=rng.normal(size=(n_points, n_coefficients)).astype(np.float32),
        opacities=rng.uniform(0.05, 0.95, size=n_points).astype(np.float32),
        scales=rng.normal(size=(n_points, 3)).astype(np.float32),
        rotations=rotations,
    )
    ply_path = tmp_path / "test_splats.ply"
    save_ply(data, ply_path)
    return ply_path, data


def test_load_save_roundtrip(tmp_path) -> None:
    ply_path, expected = create_test_ply(tmp_path, 100, sh_degree=2)

    actual = load_ply(ply_path)

    assert np.allclose(actual.positions, expected.positions, atol=1e-6)
    assert np.allclose(actual.colors_sh, expected.colors_sh, atol=1e-6)
    assert np.allclose(actual.opacities, expected.opacities, atol=1e-5)
    assert np.allclose(actual.scales, expected.scales, atol=1e-6)
    assert np.allclose(actual.rotations, expected.rotations, atol=1e-6)


def test_apply_identity_transform(tmp_path) -> None:
    _, data = create_test_ply(tmp_path, 100, sh_degree=1)

    transformed = apply_transform(data, np.eye(4, dtype=np.float32))

    assert np.allclose(transformed.positions, data.positions)
    assert np.allclose(transformed.colors_sh, data.colors_sh)
    assert np.allclose(transformed.opacities, data.opacities)
    assert np.allclose(transformed.scales, data.scales)
    assert np.allclose(transformed.rotations, data.rotations)


def test_apply_translation(tmp_path) -> None:
    _, data = create_test_ply(tmp_path, 100)
    transform = np.eye(4, dtype=np.float32)
    transform[:3, 3] = [10.0, 20.0, 30.0]

    transformed = apply_transform(data, transform)

    assert np.allclose(transformed.positions, data.positions + [10.0, 20.0, 30.0])
    assert np.allclose(transformed.rotations, data.rotations)
    assert np.allclose(transformed.scales, data.scales)


def test_apply_rotation() -> None:
    data = GaussianSplatData(
        positions=np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32),
        colors_sh=np.zeros((2, 3), dtype=np.float32),
        opacities=np.ones(2, dtype=np.float32) * 0.5,
        scales=np.zeros((2, 3), dtype=np.float32),
        rotations=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
    )
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )

    transformed = apply_transform(data, transform)

    assert np.allclose(transformed.positions, [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]], atol=1e-6)


def test_bounding_box() -> None:
    data = GaussianSplatData(
        positions=np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0], [7.0, -8.0, 9.0]], dtype=np.float32),
        colors_sh=np.zeros((3, 3), dtype=np.float32),
        opacities=np.ones(3, dtype=np.float32),
        scales=np.zeros((3, 3), dtype=np.float32),
        rotations=np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (3, 1)),
    )

    min_xyz, max_xyz = get_bounding_box(data)

    assert np.allclose(min_xyz, [-4.0, -8.0, -6.0])
    assert np.allclose(max_xyz, [7.0, 5.0, 9.0])


def test_subsample(tmp_path) -> None:
    _, data = create_test_ply(tmp_path, 1000)

    sampled = subsample(data, fraction=0.1)

    assert 80 <= sampled.positions.shape[0] <= 120
    assert sampled.colors_sh.shape[0] == sampled.positions.shape[0]
    assert sampled.opacities.shape[0] == sampled.positions.shape[0]
    assert sampled.scales.shape[0] == sampled.positions.shape[0]
    assert sampled.rotations.shape[0] == sampled.positions.shape[0]
