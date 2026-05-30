"""Umeyama similarity alignment from COLMAP reconstruction coordinates to ENU."""

from __future__ import annotations

import argparse
import csv
import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


def _as_points(points: np.ndarray, name: str) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3), got {array.shape}")
    return array


def umeyama_alignment(
    source: np.ndarray,
    target: np.ndarray,
    with_scale: bool = True,
) -> tuple[np.ndarray, dict]:
    """Estimate a 4x4 similarity transform that maps ``source`` to ``target``."""
    source = _as_points(source, "source")
    target = _as_points(target, "target")
    if source.shape != target.shape:
        raise ValueError(f"source and target must have the same shape, got {source.shape} and {target.shape}")

    n = source.shape[0]
    if n < 3:
        raise ValueError("At least 3 anchor points are required for 3D similarity alignment")

    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    src_centered = source - mu_s
    tgt_centered = target - mu_t

    var_s = np.mean(np.sum(src_centered**2, axis=1))
    if np.isclose(var_s, 0.0):
        raise ValueError("source points have near-zero variance; cannot estimate similarity transform")

    cov = (tgt_centered.T @ src_centered) / n
    u, d_values, vt = np.linalg.svd(cov)
    determinant_product = np.linalg.det(u) * np.linalg.det(vt)
    sign = 1.0 if determinant_product >= 0.0 else -1.0
    s_matrix = np.diag([1.0, 1.0, sign])

    rotation = u @ s_matrix @ vt
    scale = float(np.trace(np.diag(d_values) @ s_matrix) / var_s) if with_scale else 1.0
    translation = mu_t - scale * rotation @ mu_s

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = scale * rotation
    transform[:3, 3] = translation

    transformed = (scale * (source @ rotation.T)) + translation
    residuals = np.linalg.norm(transformed - target, axis=1)
    rmse = float(np.sqrt(np.mean(residuals**2)))
    return transform, {
        "rotation": rotation,
        "scale": scale,
        "translation": translation,
        "rmse": rmse,
        "residuals": residuals,
        "num_anchors": n,
    }


def quat_to_rotation_matrix(qvec: np.ndarray) -> np.ndarray:
    """Convert a COLMAP quaternion ``(w, x, y, z)`` to a 3x3 rotation matrix."""
    qvec = np.asarray(qvec, dtype=float)
    if qvec.shape != (4,):
        raise ValueError(f"qvec must have shape (4,), got {qvec.shape}")

    norm = np.linalg.norm(qvec)
    if np.isclose(norm, 0.0):
        raise ValueError("qvec must not be the zero quaternion")

    w, x, y, z = qvec / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _read_null_terminated_string(handle: Any) -> str:
    chars = bytearray()
    while True:
        value = handle.read(1)
        if value == b"":
            raise EOFError("Unexpected end of file while reading image name")
        if value == b"\x00":
            return chars.decode("utf-8", errors="replace")
        chars.extend(value)


def read_colmap_images_binary(path: str | Path) -> dict:
    """Read a COLMAP ``images.bin`` file and return per-image camera metadata."""
    path = Path(path)
    images: dict[str, dict[str, Any]] = {}

    with path.open("rb") as handle:
        num_images_data = handle.read(8)
        if len(num_images_data) != 8:
            raise ValueError(f"Invalid COLMAP images.bin file: {path}")
        (num_images,) = struct.unpack("<Q", num_images_data)

        for _ in range(num_images):
            image_header = handle.read(64)
            if len(image_header) != 64:
                raise EOFError("Unexpected end of file while reading COLMAP image header")

            unpacked = struct.unpack("<I4d3dI", image_header)
            image_id = int(unpacked[0])
            qvec = np.array(unpacked[1:5], dtype=float)
            tvec = np.array(unpacked[5:8], dtype=float)
            camera_id = int(unpacked[8])
            image_name = _read_null_terminated_string(handle)

            num_points_data = handle.read(8)
            if len(num_points_data) != 8:
                raise EOFError("Unexpected end of file while reading COLMAP point count")
            (num_points2d,) = struct.unpack("<Q", num_points_data)
            handle.seek(24 * num_points2d, 1)

            rotation_w2c = quat_to_rotation_matrix(qvec)
            position = -rotation_w2c.T @ tvec
            images[image_name] = {
                "image_id": image_id,
                "qvec": qvec,
                "tvec": tvec,
                "camera_id": camera_id,
                "position": position,
            }

    return images


def _frame_name(frame: dict) -> str | None:
    for key in ("file_path", "file_name", "image_name", "name"):
        value = frame.get(key)
        if value:
            return str(value).replace("\\", "/")
    return None


def _lookup_colmap_image(frame_name: str, colmap_images: dict) -> dict | None:
    normalized = frame_name.replace("\\", "/")
    if normalized in colmap_images:
        return colmap_images[normalized]

    basename = Path(normalized).name
    if basename in colmap_images:
        return colmap_images[basename]

    normalized_keys = {str(key).replace("\\", "/"): value for key, value in colmap_images.items()}
    if normalized in normalized_keys:
        return normalized_keys[normalized]

    for key, value in normalized_keys.items():
        if Path(key).name == basename:
            return value
    return None


def _enu_to_array(enu: Any) -> np.ndarray | None:
    if enu is None:
        return None
    if isinstance(enu, dict):
        if all(key in enu for key in ("e", "n", "u")):
            return np.array([enu["e"], enu["n"], enu["u"]], dtype=float)
        if all(key in enu for key in ("east", "north", "up")):
            return np.array([enu["east"], enu["north"], enu["up"]], dtype=float)
        if all(key in enu for key in ("x", "y", "z")):
            return np.array([enu["x"], enu["y"], enu["z"]], dtype=float)
    array = np.asarray(enu, dtype=float)
    if array.shape == (3,):
        return array
    raise ValueError(f"ENU coordinate must be a 3-vector or e/n/u dict, got shape {array.shape}")


def compute_anchor_pairs(
    transforms_json: dict,
    colmap_images: dict,
) -> tuple[np.ndarray, np.ndarray]:
    """Extract corresponding COLMAP camera centers and ENU coordinates."""
    source_points = []
    target_points = []
    for frame in transforms_json.get("frames", []):
        if not isinstance(frame, dict):
            continue

        name = _frame_name(frame)
        if name is None:
            continue

        colmap_image = _lookup_colmap_image(name, colmap_images)
        enu = _enu_to_array(frame.get("enu"))
        if colmap_image is None or enu is None:
            continue

        source_points.append(np.asarray(colmap_image["position"], dtype=float))
        target_points.append(enu)

    return np.asarray(source_points, dtype=float).reshape(-1, 3), np.asarray(target_points, dtype=float).reshape(-1, 3)


def canonical_origin_from_transforms_payload(transforms_json: dict, source_path: str | Path) -> dict[str, float]:
    """Read required canonical origin metadata from transforms.json payload."""
    required = ("origin_lat", "origin_lon", "origin_alt")
    missing = [key for key in required if key not in transforms_json]
    if missing:
        raise ValueError(f"{source_path} is missing canonical origin field(s): {', '.join(missing)}")
    return {
        "lat": float(transforms_json["origin_lat"]),
        "lon": float(transforms_json["origin_lon"]),
        "alt": float(transforms_json["origin_alt"]),
    }


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    points = _as_points(points, "points")
    return points @ transform[:3, :3].T + transform[:3, 3]


def robust_umeyama_alignment(
    source: np.ndarray,
    target: np.ndarray,
    *,
    with_scale: bool = True,
    ransac_threshold_m: float = 5.0,
    ransac_iterations: int = 500,
    random_seed: int = 42,
) -> tuple[np.ndarray, dict]:
    """Estimate Umeyama alignment after deterministic RANSAC outlier rejection."""
    source = _as_points(source, "source")
    target = _as_points(target, "target")
    if source.shape != target.shape:
        raise ValueError(f"source and target must have the same shape, got {source.shape} and {target.shape}")
    if source.shape[0] < 3:
        raise ValueError("At least 3 anchor points are required for robust alignment")

    count = source.shape[0]
    rng = np.random.default_rng(random_seed)
    threshold = float(ransac_threshold_m)
    best_mask: np.ndarray | None = None
    best_count = -1
    best_rmse = float("inf")

    for _ in range(max(1, int(ransac_iterations))):
        sample = rng.choice(count, size=3, replace=False)
        try:
            sample_transform, _sample_info = umeyama_alignment(source[sample], target[sample], with_scale=with_scale)
        except ValueError:
            continue
        residuals = np.linalg.norm(_transform_points(source, sample_transform) - target, axis=1)
        mask = residuals <= threshold
        inlier_count = int(np.count_nonzero(mask))
        if inlier_count < 3:
            continue
        rmse = float(np.sqrt(np.mean(residuals[mask] ** 2)))
        if inlier_count > best_count or (inlier_count == best_count and rmse < best_rmse):
            best_mask = mask
            best_count = inlier_count
            best_rmse = rmse

    if best_mask is None:
        best_mask = np.ones(count, dtype=bool)

    transform, info = umeyama_alignment(source[best_mask], target[best_mask], with_scale=with_scale)
    all_residuals = np.linalg.norm(_transform_points(source, transform) - target, axis=1)
    inlier_residuals = all_residuals[best_mask]
    info["residuals"] = all_residuals
    info["rmse"] = float(np.sqrt(np.mean(inlier_residuals**2)))
    info["num_anchors"] = count
    info["inlier_mask"] = best_mask
    return transform, info


def alignment_quality(rmse: float, residuals: np.ndarray) -> tuple[str, str]:
    """Return quality status and an actionable message for alignment metrics."""
    max_residual = float(np.max(residuals))
    if rmse > 5.0:
        return "fail", "FAIL: RMSE > 5 m. Do not use this alignment; inspect GPS anchors and remove bad frames."
    if rmse >= 3.0:
        return "warning", "WARNING: RMSE is 3-5 m. Use only for preview and inspect high-residual anchors."
    if max_residual > 10.0:
        return "warning", "WARNING: Max residual is large; inspect the highest-residual GPS frame even though RMSE passed."
    return "pass", "PASS: Alignment quality is within the default threshold."


@dataclass
class AlignmentResult:
    """Full output of the GPS-to-scene Procrustes alignment pipeline."""

    transform: np.ndarray
    rotation: np.ndarray
    scale: float
    translation: np.ndarray
    rmse: float
    residuals: np.ndarray
    num_anchors: int
    anchor_source: np.ndarray
    anchor_target: np.ndarray
    inlier_mask: np.ndarray | None = None
    quality_status: str = "pass"
    quality_message: str = ""
    origin_wgs84: dict[str, float] | None = None

    def report(self) -> str:
        """Generate a human-readable alignment report."""
        inlier_mask = self._resolved_inlier_mask()
        lines = [
            "=" * 60,
            "GS-VR-Nav Alignment Report",
            "=" * 60,
            f"Anchor points used: {self.num_anchors}",
            f"Inliers: {int(np.count_nonzero(inlier_mask))}",
            f"Outliers: {int(np.count_nonzero(~inlier_mask))}",
            f"Scale factor: {self.scale:.6f}",
            f"RMSE: {self.rmse:.3f} m",
            f"Max residual: {np.max(self.residuals):.3f} m",
            f"Min residual: {np.min(self.residuals):.3f} m",
            f"Mean residual: {np.mean(self.residuals):.3f} m",
            f"Quality: {self.quality_status}",
            f"Origin WGS84: {self.origin_wgs84}" if self.origin_wgs84 is not None else "Origin WGS84: unavailable",
            "",
            "Per-anchor residuals:",
        ]
        for index, residual in enumerate(self.residuals):
            state = "inlier" if bool(inlier_mask[index]) else "outlier"
            lines.append(f"  Anchor {index}: {residual:.3f} m ({state})")
        lines.append("=" * 60)
        lines.append(self.quality_message or alignment_quality(self.rmse, self.residuals)[1])
        return "\n".join(lines)

    def save(self, path: str | Path) -> None:
        """Save transform data to a compressed NumPy archive."""
        np.savez(
            path,
            transform=self.transform,
            rotation=self.rotation,
            scale=self.scale,
            translation=self.translation,
            residuals=self.residuals,
            anchor_source=self.anchor_source,
            anchor_target=self.anchor_target,
            inlier_mask=self._resolved_inlier_mask(),
            origin_wgs84=(
                np.array(
                    [self.origin_wgs84["lat"], self.origin_wgs84["lon"], self.origin_wgs84["alt"]],
                    dtype=float,
                )
                if self.origin_wgs84 is not None
                else np.empty((0,), dtype=float)
            ),
        )

    def write_quality_reports(self, output_dir: str | Path) -> None:
        """Write JSON and CSV residual reports next to alignment.npz."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        inlier_mask = self._resolved_inlier_mask()
        anchors = [
            {
                "index": int(index),
                "residual_m": float(residual),
                "status": "inlier" if bool(inlier_mask[index]) else "outlier",
                "source": self.anchor_source[index].astype(float).tolist(),
                "target": self.anchor_target[index].astype(float).tolist(),
            }
            for index, residual in enumerate(self.residuals)
        ]
        payload = {
            "quality_status": self.quality_status,
            "quality_message": self.quality_message,
            "rmse_m": float(self.rmse),
            "mean_residual_m": float(np.mean(self.residuals)),
            "max_residual_m": float(np.max(self.residuals)),
            "min_residual_m": float(np.min(self.residuals)),
            "num_anchors": int(self.num_anchors),
            "inlier_count": int(np.count_nonzero(inlier_mask)),
            "outlier_count": int(np.count_nonzero(~inlier_mask)),
            "scale": float(self.scale),
            "translation": self.translation.astype(float).tolist(),
            "origin_wgs84": self.origin_wgs84,
            "anchors": anchors,
        }
        (output_path / "alignment_report.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

        with (output_path / "alignment_report.csv").open("w", newline="", encoding="utf-8") as handle:
            fieldnames = (
                "index",
                "residual_m",
                "status",
                "source_x",
                "source_y",
                "source_z",
                "target_e",
                "target_n",
                "target_u",
            )
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for anchor in anchors:
                source = anchor["source"]
                target = anchor["target"]
                writer.writerow(
                    {
                        "index": anchor["index"],
                        "residual_m": anchor["residual_m"],
                        "status": anchor["status"],
                        "source_x": source[0],
                        "source_y": source[1],
                        "source_z": source[2],
                        "target_e": target[0],
                        "target_n": target[1],
                        "target_u": target[2],
                    }
                )

    def _resolved_inlier_mask(self) -> np.ndarray:
        if self.inlier_mask is None:
            return np.ones(self.num_anchors, dtype=bool)
        mask = np.asarray(self.inlier_mask, dtype=bool)
        if mask.shape != (self.num_anchors,):
            raise ValueError(f"inlier_mask must have shape ({self.num_anchors},), got {mask.shape}")
        return mask


def align_pipeline(
    transforms_json_path: str | Path,
    colmap_images_bin_path: str | Path,
    output_dir: str | Path | None = None,
    *,
    robust: bool = True,
    ransac_threshold_m: float = 5.0,
    ransac_iterations: int = 500,
) -> AlignmentResult:
    """Run the end-to-end COLMAP-to-ENU alignment pipeline."""
    transforms_path = Path(transforms_json_path)
    with transforms_path.open("r", encoding="utf-8") as handle:
        transforms_json = json.load(handle)
    origin_wgs84 = canonical_origin_from_transforms_payload(transforms_json, transforms_path)

    colmap_images = read_colmap_images_binary(colmap_images_bin_path)
    source, target = compute_anchor_pairs(transforms_json, colmap_images)
    if robust:
        transform, info = robust_umeyama_alignment(
            source,
            target,
            with_scale=True,
            ransac_threshold_m=ransac_threshold_m,
            ransac_iterations=ransac_iterations,
        )
    else:
        transform, info = umeyama_alignment(source, target, with_scale=True)
        info["inlier_mask"] = np.ones(source.shape[0], dtype=bool)

    quality_status, quality_message = alignment_quality(info["rmse"], info["residuals"])
    result = AlignmentResult(
        transform=transform,
        rotation=info["rotation"],
        scale=info["scale"],
        translation=info["translation"],
        rmse=info["rmse"],
        residuals=info["residuals"],
        num_anchors=info["num_anchors"],
        anchor_source=source,
        anchor_target=target,
        inlier_mask=info["inlier_mask"],
        quality_status=quality_status,
        quality_message=quality_message,
        origin_wgs84=origin_wgs84,
    )

    print(result.report())

    if output_dir is not None:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        result.save(output_path / "alignment.npz")
        result.write_quality_reports(output_path)

    return result


def visualize_alignment(
    result: AlignmentResult,
    save_path: str | Path | None = None,
) -> None:
    """Plot top-down, residual, 3D, and text report views of the alignment."""
    import matplotlib.pyplot as plt

    transformed_source = _transform_points(result.anchor_source, result.transform)
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Procrustes Alignment Result", fontsize=16)

    ax_top = fig.add_subplot(2, 2, 1)
    ax_top.scatter(result.anchor_target[:, 0], result.anchor_target[:, 1], c="tab:blue", label="ENU target")
    ax_top.scatter(transformed_source[:, 0], transformed_source[:, 1], c="tab:red", label="Aligned COLMAP")
    for i, (target, source) in enumerate(zip(result.anchor_target, transformed_source)):
        ax_top.plot([target[0], source[0]], [target[1], source[1]], "--", color="0.6", linewidth=1.0)
        ax_top.text(target[0], target[1], f"T{i}", color="tab:blue", fontsize=8)
        ax_top.text(source[0], source[1], f"S{i}", color="tab:red", fontsize=8)
    ax_top.set_xlabel("East (m)")
    ax_top.set_ylabel("North (m)")
    ax_top.set_title("Top-down EN view")
    ax_top.axis("equal")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend()

    ax_residual = fig.add_subplot(2, 2, 2)
    anchor_indices = np.arange(result.num_anchors)
    ax_residual.bar(anchor_indices, result.residuals, color="0.35")
    ax_residual.axhline(result.rmse, color="tab:red", linestyle="-", label=f"RMSE {result.rmse:.2f} m")
    ax_residual.axhline(3.0, color="goldenrod", linestyle="--", label="3 m")
    ax_residual.axhline(5.0, color="red", linestyle="--", label="5 m")
    ax_residual.set_xlabel("Anchor")
    ax_residual.set_ylabel("Residual (m)")
    ax_residual.set_title("Per-anchor residuals")
    ax_residual.set_xticks(anchor_indices)
    ax_residual.grid(True, axis="y", alpha=0.3)
    ax_residual.legend()

    ax_3d = fig.add_subplot(2, 2, 3, projection="3d")
    ax_3d.scatter(result.anchor_target[:, 0], result.anchor_target[:, 1], result.anchor_target[:, 2], c="tab:blue", label="ENU target")
    ax_3d.scatter(transformed_source[:, 0], transformed_source[:, 1], transformed_source[:, 2], c="tab:red", label="Aligned COLMAP")
    for target, source in zip(result.anchor_target, transformed_source):
        ax_3d.plot([target[0], source[0]], [target[1], source[1]], [target[2], source[2]], color="0.6")
    ax_3d.set_xlabel("East (m)")
    ax_3d.set_ylabel("North (m)")
    ax_3d.set_zlabel("Up (m)")
    ax_3d.set_title("3D alignment")
    ax_3d.legend()

    ax_text = fig.add_subplot(2, 2, 4)
    ax_text.axis("off")
    ax_text.text(0.0, 1.0, result.report(), va="top", ha="left", family="monospace", fontsize=9)

    fig.tight_layout()
    if save_path is not None:
        plt.savefig(save_path, bbox_inches="tight", dpi=150)
        plt.close(fig)
    else:
        plt.show()


def estimate_umeyama_similarity(
    source_points: np.ndarray,
    target_points: np.ndarray,
    with_scale: bool = True,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Backward-compatible wrapper returning ``(scale, rotation, translation)``."""
    _, info = umeyama_alignment(source_points, target_points, with_scale=with_scale)
    return info["scale"], info["rotation"], info["translation"]


def apply_similarity_transform(
    points: np.ndarray,
    scale: float,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Apply a similarity transform to 3D points."""
    points = _as_points(points, "points")
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError(f"rotation must have shape (3, 3), got {rotation.shape}")
    if translation.shape != (3,):
        raise ValueError(f"translation must have shape (3,), got {translation.shape}")
    return float(scale) * (points @ rotation.T) + translation


def save_alignment_transform(
    scale: float,
    rotation: np.ndarray,
    translation: np.ndarray,
    output_path: Path,
) -> Path:
    """Save an estimated alignment transform for renderer loading."""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = float(scale) * np.asarray(rotation, dtype=float)
    transform[:3, 3] = np.asarray(translation, dtype=float)
    np.savez(output_path, transform=transform, scale=float(scale), rotation=rotation, translation=translation)
    return output_path


def _main() -> None:
    parser = argparse.ArgumentParser(description="Align COLMAP reconstruction to ENU coordinates")
    parser.add_argument("transforms_json", help="Path to transforms.json from stage 1")
    parser.add_argument("colmap_images_bin", help="Path to COLMAP images.bin")
    parser.add_argument("--output-dir", "-o", default=None, help="Output directory for alignment results")
    parser.add_argument("--visualize", "-v", action="store_true", help="Show alignment visualization")
    parser.add_argument("--no-robust", action="store_true", help="Disable default RANSAC outlier rejection")
    parser.add_argument("--ransac-threshold", type=float, default=5.0, help="RANSAC inlier threshold in meters")
    parser.add_argument("--ransac-iterations", type=int, default=500, help="RANSAC iteration count")
    args = parser.parse_args()

    result = align_pipeline(
        args.transforms_json,
        args.colmap_images_bin,
        args.output_dir,
        robust=not args.no_robust,
        ransac_threshold_m=args.ransac_threshold,
        ransac_iterations=args.ransac_iterations,
    )
    if args.visualize:
        visualize_alignment(result)


if __name__ == "__main__":
    _main()
