import json
import math

import numpy as np
import pytest

from geo_alignment import icp_refinement
from geo_alignment.icp_refinement import (
    compute_map_to_scene_alignment,
    constrain_to_yaw_xy,
    evaluate_map_alignment_quality,
    extract_building_projection_points,
    initial_yaw_translation_transform,
    iter_building_rings,
    load_osm_building_boundary_points,
    require_open3d,
    run_multistart_yaw_icp,
    sample_polyline,
)
from reconstruction.export import load_ply
from tests.test_reconstruction import create_test_ply


def _rotation_z(angle_rad: float) -> np.ndarray:
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def test_sample_polyline_closes_open_polygon() -> None:
    ring = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], dtype=np.float32)

    sampled = sample_polyline(ring, spacing_m=1.0, closed=True)

    assert sampled.shape[1] == 2
    assert sampled.shape[0] >= 8
    assert any(np.allclose(point, [0.0, 1.0]) for point in sampled)


def test_iter_building_rings_handles_flat_and_nested_shapes() -> None:
    flat = [[0, 0], [1, 0], [1, 1]]
    nested = [[[2, 2], [3, 2], [3, 3]]]

    rings = iter_building_rings([flat, nested, [], [[1, 2]]])

    assert len(rings) == 2
    np.testing.assert_allclose(rings[0], flat)
    np.testing.assert_allclose(rings[1], nested[0])


def test_load_osm_building_boundary_points(tmp_path) -> None:
    osm_path = tmp_path / "osm_data.json"
    osm_path.write_text(
        json.dumps(
            {
                "buildings": [
                    [[0, 0], [2, 0], [2, 2], [0, 2]],
                    [[[4, 4], [5, 4], [5, 5], [4, 5]]],
                    [],
                ]
            }
        ),
        encoding="utf-8",
    )

    points = load_osm_building_boundary_points(osm_path, sample_spacing_m=1.0)

    assert points.shape[1] == 2
    assert points.shape[0] >= 12


def test_extract_building_projection_points_filters_height() -> None:
    positions = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.1],
            [2.0, 0.0, 2.5],
            [3.0, 0.0, 4.0],
        ],
        dtype=np.float32,
    )

    projected = extract_building_projection_points(
        positions,
        min_height_above_ground_m=1.0,
        max_height_above_ground_m=10.0,
        ground_percentile=0.0,
        voxel_size_m=0.0,
    )

    np.testing.assert_allclose(projected, [[2.0, 0.0], [3.0, 0.0]])


def test_constrain_to_yaw_xy_preserves_only_horizontal_components() -> None:
    raw = np.eye(4, dtype=float)
    raw[:3, :3] = _rotation_z(math.radians(15.0))
    raw[0, 3] = 4.5
    raw[1, 3] = -2.0
    raw[2, 3] = 99.0

    constrained, yaw, translation_xy = constrain_to_yaw_xy(raw)

    assert math.degrees(yaw) == pytest.approx(15.0)
    np.testing.assert_allclose(translation_xy, [4.5, -2.0])
    np.testing.assert_allclose(constrained[:2, 3], [4.5, -2.0])
    assert constrained[2, 3] == pytest.approx(0.0)
    np.testing.assert_allclose(constrained[2], [0.0, 0.0, 1.0, 0.0])


def test_initial_yaw_translation_transform_aligns_centroids() -> None:
    source = np.array([[0.0, 0.0], [2.0, 0.0]], dtype=np.float32)
    target = np.array([[10.0, -3.0], [10.0, -1.0]], dtype=np.float32)

    transform = initial_yaw_translation_transform(source, target, math.radians(90.0))

    source_3d = np.column_stack([source, np.zeros(source.shape[0]), np.ones(source.shape[0])])
    transformed = (transform @ source_3d.T).T[:, :2]
    np.testing.assert_allclose(transformed.mean(axis=0), target.mean(axis=0), atol=1e-6)


def test_run_multistart_yaw_icp_selects_best_seed(monkeypatch) -> None:
    source = np.array([[0.0, 0.0], [1.0, 0.0]], dtype=np.float32)
    target = np.array([[0.0, 0.0], [-1.0, 0.0]], dtype=np.float32)
    calls: list[float] = []

    def fake_icp(source_xy, target_xy, *, max_correspondence_distance_m, initial_transform=None):
        yaw = math.degrees(math.atan2(initial_transform[1, 0], initial_transform[0, 0]))
        calls.append(yaw)
        fitness = 0.9 if abs(abs(yaw) - 180.0) < 1e-4 else 0.1
        rmse = 0.2 if fitness > 0.5 else 9.0
        return initial_transform, fitness, rmse

    monkeypatch.setattr(icp_refinement, "run_open3d_icp", fake_icp)

    transform, fitness, rmse = run_multistart_yaw_icp(
        source,
        target,
        max_correspondence_distance_m=10.0,
        yaw_step_degrees=90.0,
    )

    assert len(calls) == 4
    assert math.degrees(math.atan2(transform[1, 0], transform[0, 0])) == pytest.approx(180.0)
    assert fitness == pytest.approx(0.9)
    assert rmse == pytest.approx(0.2)


def test_require_open3d_error_is_actionable(monkeypatch) -> None:
    def fake_import(name, *args, **kwargs):
        if name == "open3d":
            raise ImportError("missing")
        return real_import(name, *args, **kwargs)

    real_import = __import__
    monkeypatch.setattr("builtins.__import__", fake_import)

    with pytest.raises(RuntimeError, match="pip install -r requirements.txt"):
        require_open3d()


def test_refine_scene_with_icp_writes_refined_ply_and_transform(tmp_path, monkeypatch) -> None:
    input_ply, data = create_test_ply(tmp_path, 20, sh_degree=0)
    osm_path = tmp_path / "osm_data.json"
    output_ply = tmp_path / "refined_scene.ply"
    output_transform = tmp_path / "icp_refinement.npz"
    osm_path.write_text(
        json.dumps(
            {
                "origin_lat": -27.5,
                "origin_lon": 153.1,
                "origin_alt": 42.0,
                "buildings": [[[0, 0], [10, 0], [10, 10], [0, 10]]],
            }
        ),
        encoding="utf-8",
    )

    raw = np.eye(4, dtype=float)
    raw[:3, :3] = _rotation_z(math.radians(10.0))
    raw[:2, 3] = [1.0, -2.0]

    def fake_icp(source_xy, target_xy, *, max_correspondence_distance_m):
        assert source_xy.shape[1] == 2
        assert target_xy.shape[1] == 2
        return raw, 0.75, 0.5

    monkeypatch.setattr(icp_refinement, "run_open3d_icp", fake_icp)

    result = icp_refinement.refine_scene_with_icp(
        input_ply=input_ply,
        osm_json=osm_path,
        output_ply=output_ply,
        output_transform=output_transform,
        source_min_height_m=-10.0,
        source_max_height_m=100.0,
        voxel_size_m=0.0,
    )

    assert output_ply.exists()
    assert output_transform.exists()
    assert output_transform.with_suffix(".json").exists()
    assert result.fitness == pytest.approx(0.75)
    refined = load_ply(output_ply)
    expected_positions = data.positions @ result.transform[:3, :3].T + result.transform[:3, 3]
    np.testing.assert_allclose(refined.positions, expected_positions, atol=1e-5)


def test_compute_map_to_scene_alignment_writes_unity_json(tmp_path, monkeypatch) -> None:
    input_ply, _ = create_test_ply(tmp_path, 20, sh_degree=0)
    osm_path = tmp_path / "osm_data.json"
    output_json = tmp_path / "map_alignment.json"
    osm_path.write_text(
        json.dumps({"buildings": [[[0, 0], [10, 0], [10, 10], [0, 10]]]}),
        encoding="utf-8",
    )

    raw = np.eye(4, dtype=float)
    raw[:3, :3] = _rotation_z(math.radians(10.0))
    raw[:2, 3] = [23.2, -15.8]

    def fake_icp(source_xy, target_xy, *, max_correspondence_distance_m, yaw_step_degrees):
        assert source_xy.shape[1] == 2
        assert target_xy.shape[1] == 2
        return raw, 0.72, 1.34

    monkeypatch.setattr(icp_refinement, "run_multistart_yaw_icp", fake_icp)

    result = compute_map_to_scene_alignment(
        input_ply=input_ply,
        osm_json=osm_path,
        output_map_alignment=output_json,
        unity_y_offset=-1.0,
        source_min_height_m=-10.0,
        source_max_height_m=100.0,
        voxel_size_m=0.0,
    )

    payload = json.loads(output_json.read_text(encoding="utf-8"))
    assert payload["coordinate_space"] == "unity_xz"
    assert payload["alignment_mode"] == "map-to-scene"
    assert payload["accepted"] is True
    assert payload["origin_wgs84"] == {"lat": -27.5, "lon": 153.1, "alt": 42.0}
    assert len(payload["input_ply_hash"]) == 64
    assert len(payload["osm_json_hash"]) == 64
    assert payload["position"] == pytest.approx({"x": 23.2, "y": -1.0, "z": -15.8})
    assert payload["rotation_y_deg"] == pytest.approx(-10.0)
    assert payload["scale"] == pytest.approx(1.0)
    assert payload["fitness"] == pytest.approx(0.72)
    assert payload["rmse"] == pytest.approx(1.34)
    assert payload["source"] == "osm_buildings_to_splat_projection"
    assert payload["warnings"]
    assert result.source_count > 0
    assert result.target_count > 0


def test_map_alignment_quality_rejects_bad_icp_metrics() -> None:
    result = icp_refinement.UnityMapAlignmentResult(
        transform=np.eye(4),
        raw_icp_transform=np.eye(4),
        yaw_rad=math.radians(20.0),
        translation_xy=np.array([3.0, 4.0]),
        unity_y_offset=-1.0,
        fitness=0.4,
        rmse=4.5,
        source_count=10,
        target_count=12,
    )

    accepted, warnings = evaluate_map_alignment_quality(result)

    assert accepted is False
    assert any("fitness" in warning for warning in warnings)
    assert any("rmse" in warning for warning in warnings)
    assert any("yaw" in warning for warning in warnings)
