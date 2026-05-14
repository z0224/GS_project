"""加载并操作 3D Gaussian Splatting .ply 文件。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from plyfile import PlyData, PlyElement


@dataclass(frozen=True)
class GaussianSplatData:
    positions: np.ndarray
    colors_sh: np.ndarray
    opacities: np.ndarray
    scales: np.ndarray
    rotations: np.ndarray


def load_ply(ply_path: str | Path) -> GaussianSplatData:
    """Load a 3DGS PLY file into structured NumPy arrays."""
    ply_path = Path(ply_path)
    ply = PlyData.read(ply_path)
    vertex = ply["vertex"].data
    names = vertex.dtype.names or ()

    required = ["x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2", "opacity"]
    required += [f"scale_{idx}" for idx in range(3)]
    required += [f"rot_{idx}" for idx in range(4)]
    missing = [name for name in required if name not in names]
    if missing:
        raise ValueError(f"PLY file is missing required 3DGS vertex properties: {missing}")

    rest_names = sorted(
        (name for name in names if name.startswith("f_rest_")),
        key=lambda name: int(name.rsplit("_", 1)[1]),
    )

    positions = _stack_fields(vertex, ["x", "y", "z"])
    colors_sh = _stack_fields(vertex, ["f_dc_0", "f_dc_1", "f_dc_2", *rest_names])
    opacities = _sigmoid(np.asarray(vertex["opacity"], dtype=np.float32))
    scales = _stack_fields(vertex, [f"scale_{idx}" for idx in range(3)])
    rotations = _normalize_quaternions(_stack_fields(vertex, [f"rot_{idx}" for idx in range(4)]))

    return GaussianSplatData(
        positions=positions,
        colors_sh=colors_sh,
        opacities=opacities,
        scales=scales,
        rotations=rotations,
    )


def save_ply(data: GaussianSplatData, output_path: str | Path) -> None:
    """Save splat data in a 3DGS viewer-compatible PLY layout."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    _validate_data(data)

    n_points = data.positions.shape[0]
    n_rest = data.colors_sh.shape[1] - 3
    fields = [
        ("x", "f4"),
        ("y", "f4"),
        ("z", "f4"),
        ("nx", "f4"),
        ("ny", "f4"),
        ("nz", "f4"),
        ("f_dc_0", "f4"),
        ("f_dc_1", "f4"),
        ("f_dc_2", "f4"),
    ]
    fields.extend((f"f_rest_{idx}", "f4") for idx in range(n_rest))
    fields.append(("opacity", "f4"))
    fields.extend((f"scale_{idx}", "f4") for idx in range(3))
    fields.extend((f"rot_{idx}", "f4") for idx in range(4))

    vertex = np.empty(n_points, dtype=fields)
    vertex["x"], vertex["y"], vertex["z"] = data.positions.astype(np.float32).T
    vertex["nx"] = 0.0
    vertex["ny"] = 0.0
    vertex["nz"] = 0.0
    for idx in range(3):
        vertex[f"f_dc_{idx}"] = data.colors_sh[:, idx].astype(np.float32)
    for idx in range(n_rest):
        vertex[f"f_rest_{idx}"] = data.colors_sh[:, idx + 3].astype(np.float32)
    vertex["opacity"] = _logit(data.opacities.astype(np.float32))
    for idx in range(3):
        vertex[f"scale_{idx}"] = data.scales[:, idx].astype(np.float32)
    rotations = _normalize_quaternions(data.rotations.astype(np.float32))
    for idx in range(4):
        vertex[f"rot_{idx}"] = rotations[:, idx]

    PlyData([PlyElement.describe(vertex, "vertex")], text=False).write(output_path)


def apply_transform(data: GaussianSplatData, transform: np.ndarray) -> GaussianSplatData:
    """Apply a 4x4 similarity transform and return a new splat data object."""
    _validate_data(data)
    transform = np.asarray(transform, dtype=np.float32)
    if transform.shape != (4, 4):
        raise ValueError(f"transform must have shape (4, 4), got {transform.shape}")

    # CN: 4x4 对齐矩阵同时包含旋转和统一尺度；位置、旋转、scale 都需要同步更新。
    # EN: The 4x4 alignment matrix contains rotation and uniform scale, so positions, rotations, and scales all change.
    linear = transform[:3, :3]
    scale = float(np.cbrt(abs(np.linalg.det(linear))))
    if scale <= 0.0:
        raise ValueError("transform must contain a non-zero uniform scale")

    rotation_matrix = linear / scale
    transform_quat = _matrix_to_quaternion(rotation_matrix)
    # CN: 位置直接做仿射变换；Gaussian 自身朝向用四元数乘法叠加全局旋转。
    # EN: Positions use the affine transform; Gaussian orientations compose the global rotation as quaternions.
    transformed_positions = (data.positions @ linear.T) + transform[:3, 3]
    transformed_rotations = _normalize_quaternions(
        _quaternion_multiply(
            np.broadcast_to(transform_quat, data.rotations.shape),
            data.rotations,
        )
    )
    transformed_scales = data.scales + np.float32(np.log(scale))

    return GaussianSplatData(
        positions=transformed_positions.astype(np.float32),
        colors_sh=data.colors_sh.copy(),
        opacities=data.opacities.copy(),
        scales=transformed_scales.astype(np.float32),
        rotations=transformed_rotations.astype(np.float32),
    )


def get_bounding_box(data: GaussianSplatData) -> tuple[np.ndarray, np.ndarray]:
    """Return the axis-aligned bounding box as ``(min_xyz, max_xyz)``."""
    if data.positions.size == 0:
        raise ValueError("Cannot compute a bounding box for empty splat data")
    return np.min(data.positions, axis=0), np.max(data.positions, axis=0)


def subsample(data: GaussianSplatData, fraction: float = 0.1) -> GaussianSplatData:
    """Randomly subsample a fraction of Gaussians for debugging."""
    _validate_data(data)
    if not 0.0 < fraction <= 1.0:
        raise ValueError("fraction must be in the range (0, 1]")

    n_points = data.positions.shape[0]
    n_keep = int(round(n_points * fraction))
    n_keep = min(n_points, max(1, n_keep))
    indices = np.random.default_rng().choice(n_points, size=n_keep, replace=False)

    return GaussianSplatData(
        positions=data.positions[indices].copy(),
        colors_sh=data.colors_sh[indices].copy(),
        opacities=data.opacities[indices].copy(),
        scales=data.scales[indices].copy(),
        rotations=data.rotations[indices].copy(),
    )


def _stack_fields(vertex: np.ndarray, field_names: list[str]) -> np.ndarray:
    return np.column_stack([np.asarray(vertex[name], dtype=np.float32) for name in field_names]).astype(np.float32)


def _sigmoid(values: np.ndarray) -> np.ndarray:
    return (1.0 / (1.0 + np.exp(-values))).astype(np.float32)


def _logit(values: np.ndarray) -> np.ndarray:
    clipped = np.clip(values, 1e-6, 1.0 - 1e-6)
    return np.log(clipped / (1.0 - clipped)).astype(np.float32)


def _validate_data(data: GaussianSplatData) -> None:
    n_points = data.positions.shape[0]
    expected = {
        "positions": (n_points, 3),
        "opacities": (n_points,),
        "scales": (n_points, 3),
        "rotations": (n_points, 4),
    }
    for name, shape in expected.items():
        value = getattr(data, name)
        if value.shape != shape:
            raise ValueError(f"{name} must have shape {shape}, got {value.shape}")
    if data.colors_sh.ndim != 2 or data.colors_sh.shape[0] != n_points or data.colors_sh.shape[1] < 3:
        raise ValueError("colors_sh must have shape (N, C) with C >= 3")


def _normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    norms = np.where(norms == 0.0, 1.0, norms)
    return (quaternions / norms).astype(np.float32)


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = np.moveaxis(left, -1, 0)
    rw, rx, ry, rz = np.moveaxis(right, -1, 0)
    return np.stack(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        axis=-1,
    ).astype(np.float32)


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float32)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * s,
                (matrix[2, 1] - matrix[1, 2]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[1, 0] - matrix[0, 1]) / s,
            ],
            dtype=np.float32,
        )
    else:
        idx = int(np.argmax(np.diag(matrix)))
        if idx == 0:
            s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quat = np.array(
                [
                    (matrix[2, 1] - matrix[1, 2]) / s,
                    0.25 * s,
                    (matrix[0, 1] + matrix[1, 0]) / s,
                    (matrix[0, 2] + matrix[2, 0]) / s,
                ],
                dtype=np.float32,
            )
        elif idx == 1:
            s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quat = np.array(
                [
                    (matrix[0, 2] - matrix[2, 0]) / s,
                    (matrix[0, 1] + matrix[1, 0]) / s,
                    0.25 * s,
                    (matrix[1, 2] + matrix[2, 1]) / s,
                ],
                dtype=np.float32,
            )
        else:
            s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quat = np.array(
                [
                    (matrix[1, 0] - matrix[0, 1]) / s,
                    (matrix[0, 2] + matrix[2, 0]) / s,
                    (matrix[1, 2] + matrix[2, 1]) / s,
                    0.25 * s,
                ],
                dtype=np.float32,
            )
    return _normalize_quaternions(quat.reshape(1, 4))[0]
