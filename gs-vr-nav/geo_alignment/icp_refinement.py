"""Refine an aligned 3DGS scene against OSM building footprints with Open3D ICP."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from plyfile import PlyData

try:
    from reconstruction.export import apply_transform, load_ply, save_ply
except ImportError:  # pragma: no cover - package import fallback
    from gs_vr_nav.reconstruction.export import apply_transform, load_ply, save_ply


@dataclass(frozen=True)
class IcpRefinementResult:
    """Summary of a horizontal ICP refinement."""

    transform: np.ndarray
    raw_icp_transform: np.ndarray
    yaw_rad: float
    translation_xy: np.ndarray
    fitness: float
    rmse: float
    source_count: int
    target_count: int


@dataclass(frozen=True)
class UnityMapAlignmentResult:
    """Map-to-scene alignment that Unity can apply to Blosm map geometry."""

    transform: np.ndarray
    raw_icp_transform: np.ndarray
    yaw_rad: float
    translation_xy: np.ndarray
    unity_y_offset: float
    fitness: float
    rmse: float
    source_count: int
    target_count: int


def require_open3d() -> Any:
    """Import Open3D or raise an actionable dependency error."""
    try:
        import open3d as o3d  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Open3D is required for ICP refinement but is not installed. "
            "Run `pip install -r requirements.txt` from the gs-vr-nav directory."
        ) from exc
    return o3d


def load_splat_positions(ply_path: str | Path) -> np.ndarray:
    """Load only XYZ positions from a 3DGS PLY."""
    vertex = PlyData.read(Path(ply_path))["vertex"].data
    return np.column_stack([vertex["x"], vertex["y"], vertex["z"]]).astype(np.float32)


def extract_building_projection_points(
    positions: np.ndarray,
    *,
    min_height_above_ground_m: float = 2.0,
    max_height_above_ground_m: float | None = 35.0,
    ground_percentile: float = 5.0,
    voxel_size_m: float = 1.0,
    max_points: int = 100_000,
) -> np.ndarray:
    """Project likely vertical/building splat points onto the EN plane."""
    points = np.asarray(positions, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"positions must have shape (N, 3), got {points.shape}")
    if points.size == 0:
        raise ValueError("positions must not be empty")

    ground_z = float(np.percentile(points[:, 2], ground_percentile))
    min_z = ground_z + float(min_height_above_ground_m)
    mask = points[:, 2] >= min_z
    if max_height_above_ground_m is not None:
        mask &= points[:, 2] <= ground_z + float(max_height_above_ground_m)

    projected = points[mask, :2]
    if projected.shape[0] == 0:
        raise ValueError("No source points survived building-height filtering")

    projected = voxel_downsample_2d(projected, voxel_size_m)
    if projected.shape[0] > max_points:
        rng = np.random.default_rng(42)
        indices = rng.choice(projected.shape[0], size=max_points, replace=False)
        projected = projected[np.sort(indices)]
    return projected.astype(np.float32)


def voxel_downsample_2d(points: np.ndarray, voxel_size_m: float) -> np.ndarray:
    """Downsample 2D points by keeping one point per metric voxel."""
    array = np.asarray(points, dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError(f"points must have shape (N, 2), got {array.shape}")
    if array.shape[0] == 0:
        return array.copy()
    if voxel_size_m <= 0.0:
        return array.copy()

    keys = np.floor(array / float(voxel_size_m)).astype(np.int64)
    _, unique_indices = np.unique(keys, axis=0, return_index=True)
    return array[np.sort(unique_indices)]


def load_osm_building_boundary_points(
    osm_json_path: str | Path,
    *,
    sample_spacing_m: float = 1.0,
) -> np.ndarray:
    """Load and sample OSM building footprint boundaries in EN coordinates."""
    payload = json.loads(Path(osm_json_path).read_text(encoding="utf-8"))
    sampled: list[np.ndarray] = []
    for ring in iter_building_rings(payload.get("buildings", [])):
        points = sample_polyline(ring, spacing_m=sample_spacing_m, closed=True)
        if points.shape[0] > 0:
            sampled.append(points)

    if not sampled:
        raise ValueError(f"No building footprint boundary points found in {osm_json_path}")
    return np.vstack(sampled).astype(np.float32)


def iter_building_rings(buildings: Any) -> list[np.ndarray]:
    """Return polygon rings from flexible OSM building JSON shapes."""
    rings: list[np.ndarray] = []
    if not isinstance(buildings, list):
        return rings

    for building in buildings:
        if _is_ring(building):
            ring = np.asarray(building, dtype=np.float32)
            if ring.shape[0] >= 3:
                rings.append(ring)
            continue

        if isinstance(building, list):
            for maybe_ring in building:
                if _is_ring(maybe_ring):
                    ring = np.asarray(maybe_ring, dtype=np.float32)
                    if ring.shape[0] >= 3:
                        rings.append(ring)
    return rings


def sample_polyline(points: np.ndarray, *, spacing_m: float = 1.0, closed: bool = False) -> np.ndarray:
    """Sample points along a 2D polyline or polygon boundary."""
    array = np.asarray(points, dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 2 or array.shape[0] < 2:
        return np.empty((0, 2), dtype=np.float32)

    if closed and not np.allclose(array[0], array[-1]):
        array = np.vstack([array, array[0]])

    samples: list[np.ndarray] = []
    step = max(float(spacing_m), 0.05)
    for start, end in zip(array[:-1], array[1:]):
        delta = end - start
        length = float(np.linalg.norm(delta))
        if length <= 1e-6:
            continue
        count = max(1, int(math.ceil(length / step)))
        for index in range(count):
            samples.append(start + delta * (index / count))

    if not samples:
        return np.empty((0, 2), dtype=np.float32)
    return np.asarray(samples, dtype=np.float32)


def run_open3d_icp(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
    *,
    max_correspondence_distance_m: float = 15.0,
    initial_transform: np.ndarray | None = None,
) -> tuple[np.ndarray, float, float]:
    """Run point-to-point ICP on 2D EN points embedded into Z=0."""
    o3d = require_open3d()
    source = _to_open3d_cloud(o3d, source_xy)
    target = _to_open3d_cloud(o3d, target_xy)
    init = np.eye(4, dtype=float) if initial_transform is None else np.asarray(initial_transform, dtype=float)

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        float(max_correspondence_distance_m),
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    )
    return np.asarray(result.transformation, dtype=float), float(result.fitness), float(result.inlier_rmse)


def run_multistart_yaw_icp(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
    *,
    max_correspondence_distance_m: float = 15.0,
    yaw_step_degrees: float = 15.0,
) -> tuple[np.ndarray, float, float]:
    """Run ICP from multiple yaw seeds so map-to-scene alignment avoids local minima."""
    source = np.asarray(source_xy, dtype=np.float64)
    target = np.asarray(target_xy, dtype=np.float64)
    if source.ndim != 2 or source.shape[1] != 2 or source.shape[0] == 0:
        raise ValueError(f"source_xy must have shape (N, 2), got {source.shape}")
    if target.ndim != 2 or target.shape[1] != 2 or target.shape[0] == 0:
        raise ValueError(f"target_xy must have shape (N, 2), got {target.shape}")

    step = max(float(yaw_step_degrees), 1.0)
    seed_count = max(1, int(math.ceil(360.0 / step)))
    seeds = [index * 360.0 / seed_count for index in range(seed_count)]
    if 0.0 not in seeds:
        seeds.insert(0, 0.0)

    best_transform: np.ndarray | None = None
    best_fitness = -1.0
    best_rmse = float("inf")
    for yaw_degrees in seeds:
        initial = initial_yaw_translation_transform(source, target, math.radians(yaw_degrees))
        transform, fitness, rmse = run_open3d_icp(
            source,
            target,
            max_correspondence_distance_m=max_correspondence_distance_m,
            initial_transform=initial,
        )
        if fitness > best_fitness or (math.isclose(fitness, best_fitness) and rmse < best_rmse):
            best_transform = transform
            best_fitness = fitness
            best_rmse = rmse

    if best_transform is None:
        raise RuntimeError("No ICP yaw seed produced a result")
    return best_transform, best_fitness, best_rmse


def initial_yaw_translation_transform(source_xy: np.ndarray, target_xy: np.ndarray, yaw_rad: float) -> np.ndarray:
    """Build an initial 2D yaw transform whose translation aligns source and target centroids."""
    source = np.asarray(source_xy, dtype=np.float64)
    target = np.asarray(target_xy, dtype=np.float64)
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=np.float64)
    translation = target_centroid - rotation @ source_centroid

    transform = np.eye(4, dtype=np.float64)
    transform[:2, :2] = rotation
    transform[:2, 3] = translation
    return transform


def constrain_to_yaw_xy(raw_transform: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
    """Constrain a 4x4 rigid transform to yaw around ENU Up plus XY translation."""
    matrix = np.asarray(raw_transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"raw_transform must have shape (4, 4), got {matrix.shape}")

    yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    translation_xy = matrix[:2, 3].astype(float).copy()

    constrained = np.eye(4, dtype=float)
    constrained[:3, :3] = np.array(
        [
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    constrained[0, 3] = translation_xy[0]
    constrained[1, 3] = translation_xy[1]
    return constrained, yaw, translation_xy


def refine_scene_with_icp(
    input_ply: str | Path,
    osm_json: str | Path,
    output_ply: str | Path,
    *,
    output_transform: str | Path | None = None,
    voxel_size_m: float = 1.0,
    osm_sample_spacing_m: float = 1.0,
    source_min_height_m: float = 2.0,
    source_max_height_m: float | None = 35.0,
    max_correspondence_distance_m: float = 15.0,
    max_source_points: int = 100_000,
) -> IcpRefinementResult:
    """Refine an aligned 3DGS PLY against OSM building footprints and save it."""
    positions = load_splat_positions(input_ply)
    source_xy = extract_building_projection_points(
        positions,
        min_height_above_ground_m=source_min_height_m,
        max_height_above_ground_m=source_max_height_m,
        voxel_size_m=voxel_size_m,
        max_points=max_source_points,
    )
    target_xy = load_osm_building_boundary_points(osm_json, sample_spacing_m=osm_sample_spacing_m)
    target_xy = crop_target_to_source_bounds(
        target_xy,
        source_xy,
        margin_m=max(50.0, max_correspondence_distance_m * 2.0),
    )

    raw_transform, fitness, rmse = run_open3d_icp(
        source_xy,
        target_xy,
        max_correspondence_distance_m=max_correspondence_distance_m,
    )
    transform, yaw, translation_xy = constrain_to_yaw_xy(raw_transform)

    splats = load_ply(input_ply)
    refined = apply_transform(splats, transform.astype(np.float32))
    save_ply(refined, output_ply)

    result = IcpRefinementResult(
        transform=transform,
        raw_icp_transform=raw_transform,
        yaw_rad=yaw,
        translation_xy=translation_xy,
        fitness=fitness,
        rmse=rmse,
        source_count=int(source_xy.shape[0]),
        target_count=int(target_xy.shape[0]),
    )
    if output_transform is not None:
        save_refinement_result(result, output_transform)
    return result


def compute_map_to_scene_alignment(
    input_ply: str | Path,
    osm_json: str | Path,
    output_map_alignment: str | Path,
    *,
    unity_y_offset: float = -1.0,
    voxel_size_m: float = 1.0,
    osm_sample_spacing_m: float = 1.0,
    source_min_height_m: float = 2.0,
    source_max_height_m: float | None = 35.0,
    max_correspondence_distance_m: float = 15.0,
    max_target_points: int = 100_000,
    coarse_yaw_step_degrees: float = 15.0,
) -> UnityMapAlignmentResult:
    """Align OSM/Blosm map geometry to the visible 3DGS scene and save Unity JSON."""
    positions = load_splat_positions(input_ply)
    target_xy = extract_building_projection_points(
        positions,
        min_height_above_ground_m=source_min_height_m,
        max_height_above_ground_m=source_max_height_m,
        voxel_size_m=voxel_size_m,
        max_points=max_target_points,
    )
    source_xy = load_osm_building_boundary_points(osm_json, sample_spacing_m=osm_sample_spacing_m)
    source_xy = crop_target_to_source_bounds(
        source_xy,
        target_xy,
        margin_m=max(50.0, max_correspondence_distance_m * 2.0),
    )

    raw_transform, fitness, rmse = run_multistart_yaw_icp(
        source_xy,
        target_xy,
        max_correspondence_distance_m=max_correspondence_distance_m,
        yaw_step_degrees=coarse_yaw_step_degrees,
    )
    transform, yaw, translation_xy = constrain_to_yaw_xy(raw_transform)

    result = UnityMapAlignmentResult(
        transform=transform,
        raw_icp_transform=raw_transform,
        yaw_rad=yaw,
        translation_xy=translation_xy,
        unity_y_offset=float(unity_y_offset),
        fitness=fitness,
        rmse=rmse,
        source_count=int(source_xy.shape[0]),
        target_count=int(target_xy.shape[0]),
    )
    save_map_alignment_result(result, output_map_alignment)
    return result


def crop_target_to_source_bounds(target_xy: np.ndarray, source_xy: np.ndarray, *, margin_m: float) -> np.ndarray:
    """Crop OSM targets to the source footprint bounds plus a margin."""
    target = np.asarray(target_xy, dtype=np.float32)
    source = np.asarray(source_xy, dtype=np.float32)
    if target.shape[0] == 0 or source.shape[0] == 0:
        return target

    lower = source.min(axis=0) - float(margin_m)
    upper = source.max(axis=0) + float(margin_m)
    mask = np.all((target >= lower) & (target <= upper), axis=1)
    cropped = target[mask]
    if cropped.shape[0] < 10:
        return target
    return cropped


def save_refinement_result(result: IcpRefinementResult, output_path: str | Path) -> None:
    """Save ICP refinement metrics as NPZ and JSON sidecar when appropriate."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = _result_json_payload(result)

    if path.suffix.lower() == ".json":
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return

    np.savez(
        path,
        transform=result.transform,
        raw_icp_transform=result.raw_icp_transform,
        yaw_rad=result.yaw_rad,
        translation_xy=result.translation_xy,
        fitness=result.fitness,
        rmse=result.rmse,
        source_count=result.source_count,
        target_count=result.target_count,
    )
    path.with_suffix(".json").write_text(json.dumps(payload, indent=2), encoding="utf-8")


def save_map_alignment_result(result: UnityMapAlignmentResult, output_path: str | Path) -> None:
    """Save a Unity-readable Blosm map alignment JSON."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_map_alignment_json_payload(result), indent=2), encoding="utf-8")


def _result_json_payload(result: IcpRefinementResult) -> dict[str, Any]:
    return {
        "transform": result.transform.tolist(),
        "raw_icp_transform": result.raw_icp_transform.tolist(),
        "yaw_rad": result.yaw_rad,
        "yaw_deg": math.degrees(result.yaw_rad),
        "translation_xy": result.translation_xy.tolist(),
        "fitness": result.fitness,
        "rmse": result.rmse,
        "source_count": result.source_count,
        "target_count": result.target_count,
    }


def _map_alignment_json_payload(result: UnityMapAlignmentResult) -> dict[str, Any]:
    return {
        "coordinate_space": "unity_xz",
        "position": {
            "x": float(result.translation_xy[0]),
            "y": result.unity_y_offset,
            "z": float(result.translation_xy[1]),
        },
        "rotation_y_deg": -math.degrees(result.yaw_rad),
        "scale": 1.0,
        "fitness": result.fitness,
        "rmse": result.rmse,
        "source_count": result.source_count,
        "target_count": result.target_count,
        "source": "osm_buildings_to_splat_projection",
    }


def _to_open3d_cloud(o3d: Any, points_xy: np.ndarray) -> Any:
    points = np.asarray(points_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError(f"points_xy must have shape (N, 2), got {points.shape}")
    if points.shape[0] == 0:
        raise ValueError("ICP point clouds must not be empty")
    points_3d = np.column_stack([points, np.zeros(points.shape[0], dtype=np.float64)])
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_3d)
    return cloud


def _is_ring(value: Any) -> bool:
    if not isinstance(value, list) or len(value) < 3:
        return False
    return all(_is_point(item) for item in value)


def _is_point(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return False
    return all(isinstance(component, (int, float)) for component in value)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Align 3DGS PLY and OSM building footprints with Open3D ICP.")
    parser.add_argument("--mode", choices=("refined-scene", "map-to-scene"), default="refined-scene")
    parser.add_argument("--input-ply", type=Path, required=True)
    parser.add_argument("--osm-json", type=Path, required=True)
    parser.add_argument("--output-ply", type=Path, default=None)
    parser.add_argument("--output-transform", type=Path, default=None)
    parser.add_argument("--output-map-alignment", type=Path, default=None)
    parser.add_argument("--unity-y-offset", type=float, default=-1.0)
    parser.add_argument("--voxel-size", type=float, default=1.0)
    parser.add_argument("--osm-sample-spacing", type=float, default=1.0)
    parser.add_argument("--source-min-height", type=float, default=2.0)
    parser.add_argument("--source-max-height", type=float, default=35.0)
    parser.add_argument("--max-correspondence-distance", type=float, default=15.0)
    parser.add_argument("--max-source-points", type=int, default=100_000)
    parser.add_argument("--coarse-yaw-step", type=float, default=15.0)
    return parser


def main(argv: list[str] | None = None) -> None:
    args = _build_parser().parse_args(argv)
    if args.mode == "map-to-scene":
        if args.output_map_alignment is None:
            raise SystemExit("--output-map-alignment is required when --mode map-to-scene")
        result = compute_map_to_scene_alignment(
            input_ply=args.input_ply,
            osm_json=args.osm_json,
            output_map_alignment=args.output_map_alignment,
            unity_y_offset=args.unity_y_offset,
            voxel_size_m=args.voxel_size,
            osm_sample_spacing_m=args.osm_sample_spacing,
            source_min_height_m=args.source_min_height,
            source_max_height_m=args.source_max_height,
            max_correspondence_distance_m=args.max_correspondence_distance,
            max_target_points=args.max_source_points,
            coarse_yaw_step_degrees=args.coarse_yaw_step,
        )
        print(
            "Map-to-scene ICP complete: "
            f"unity_position=({result.translation_xy[0]:.3f}, {result.unity_y_offset:.3f}, {result.translation_xy[1]:.3f}) m, "
            f"unity_rotation_y={-math.degrees(result.yaw_rad):.3f} deg, "
            f"fitness={result.fitness:.4f}, rmse={result.rmse:.3f} m, "
            f"source={result.source_count}, target={result.target_count}"
        )
        return

    if args.output_ply is None:
        raise SystemExit("--output-ply is required when --mode refined-scene")

    result = refine_scene_with_icp(
        input_ply=args.input_ply,
        osm_json=args.osm_json,
        output_ply=args.output_ply,
        output_transform=args.output_transform,
        voxel_size_m=args.voxel_size,
        osm_sample_spacing_m=args.osm_sample_spacing,
        source_min_height_m=args.source_min_height,
        source_max_height_m=args.source_max_height,
        max_correspondence_distance_m=args.max_correspondence_distance,
        max_source_points=args.max_source_points,
    )
    print(
        "ICP refinement complete: "
        f"yaw={math.degrees(result.yaw_rad):.3f} deg, "
        f"translation=({result.translation_xy[0]:.3f}, {result.translation_xy[1]:.3f}) m, "
        f"fitness={result.fitness:.4f}, rmse={result.rmse:.3f} m, "
        f"source={result.source_count}, target={result.target_count}"
    )


if __name__ == "__main__":
    main()
