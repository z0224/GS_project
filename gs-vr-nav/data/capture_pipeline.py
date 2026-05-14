"""Extract GPS metadata from phone photos and generate 3DGS transforms.json."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import piexif
from PIL import Image

try:
    from utils.coordinate_utils import gps_to_enu, heading_to_rotation_matrix
except ImportError:  # pragma: no cover - package import fallback
    from gs_vr_nav.utils.coordinate_utils import gps_to_enu, heading_to_rotation_matrix


DEFAULT_EXTENSIONS = ("jpg", "jpeg", "png")


def discover_images(input_dir: Path, extensions: Iterable[str]) -> list[Path]:
    """Discover capture images that match the configured file extensions."""
    normalized = {ext.lower().lstrip(".") for ext in extensions}
    return sorted(
        path
        for path in input_dir.rglob("*")
        if path.is_file() and path.suffix.lower().lstrip(".") in normalized
    )


def _ratio_to_float(value: Any) -> float:
    if isinstance(value, tuple):
        numerator, denominator = value
        return float(numerator) / float(denominator)
    return float(value)


def _dms_to_decimal(dms: Any, ref: bytes | str | None) -> float:
    degrees, minutes, seconds = (_ratio_to_float(part) for part in dms)
    decimal = degrees + minutes / 60.0 + seconds / 3600.0
    ref_text = ref.decode("ascii", errors="ignore") if isinstance(ref, bytes) else ref
    if ref_text in {"S", "W"}:
        decimal *= -1.0
    return float(decimal)


def extract_exif(image_path: Path) -> dict[str, Any]:
    """Extract raw EXIF metadata from an image file."""
    return piexif.load(str(image_path))


def parse_gps(exif_data: dict[str, Any]) -> tuple[float, float, float | None]:
    """Parse latitude, longitude, and optional altitude from EXIF metadata."""
    gps = exif_data.get("GPS") or {}
    lat = _dms_to_decimal(gps[piexif.GPSIFD.GPSLatitude], gps.get(piexif.GPSIFD.GPSLatitudeRef))
    lon = _dms_to_decimal(gps[piexif.GPSIFD.GPSLongitude], gps.get(piexif.GPSIFD.GPSLongitudeRef))

    alt = None
    if piexif.GPSIFD.GPSAltitude in gps:
        alt = _ratio_to_float(gps[piexif.GPSIFD.GPSAltitude])
        if gps.get(piexif.GPSIFD.GPSAltitudeRef, 0) == 1:
            alt *= -1.0
    return float(lat), float(lon), None if alt is None else float(alt)


def parse_heading(exif_data: dict[str, Any]) -> float | None:
    """Parse optional compass heading from EXIF metadata."""
    gps = exif_data.get("GPS") or {}
    if piexif.GPSIFD.GPSImgDirection not in gps:
        return None
    return float(_ratio_to_float(gps[piexif.GPSIFD.GPSImgDirection]))


def extract_exif_gps(image_path: str | Path) -> dict[str, float | None] | None:
    """Extract lat/lon/alt/heading from image EXIF GPS metadata."""
    try:
        exif_data = extract_exif(Path(image_path))
    except (FileNotFoundError, ValueError, piexif.InvalidImageDataError):
        return None

    gps = exif_data.get("GPS") or {}
    if (
        piexif.GPSIFD.GPSLatitude not in gps
        or piexif.GPSIFD.GPSLongitude not in gps
        or piexif.GPSIFD.GPSLatitudeRef not in gps
        or piexif.GPSIFD.GPSLongitudeRef not in gps
    ):
        return None

    lat, lon, alt = parse_gps(exif_data)
    return {"lat": lat, "lon": lon, "alt": 0.0 if alt is None else alt, "heading": parse_heading(exif_data)}


def estimate_intrinsics(image_path: str | Path) -> dict[str, float | int]:
    """Estimate simple pinhole intrinsics from image size."""
    with Image.open(image_path) as image:
        width, height = image.size
    focal = float(max(width, height) * 1.2)
    return {
        "w": int(width),
        "h": int(height),
        "fx": focal,
        "fy": focal,
        "cx": float(width) / 2.0,
        "cy": float(height) / 2.0,
    }


def _gps_accuracy_m(image_path: Path) -> float | None:
    try:
        gps = extract_exif(image_path).get("GPS") or {}
    except (FileNotFoundError, ValueError, piexif.InvalidImageDataError):
        return None
    tag = getattr(piexif.GPSIFD, "GPSHPositioningError", 31)
    if tag not in gps:
        return None
    return float(_ratio_to_float(gps[tag]))


def process_image_directory(
    image_dir: str | Path,
    output_path: str | Path = "transforms.json",
    config: dict | None = None,
) -> dict:
    """Extract capture metadata and write a 3DGS-friendly transforms.json file."""
    cfg = config or {}
    input_dir = Path(image_dir)
    output = Path(output_path)
    extensions = cfg.get("extensions", DEFAULT_EXTENSIONS)
    max_accuracy_m = cfg.get("max_gps_accuracy_m")

    # CN: 第一阶段只保留带可用 GPS 的图片；没有 GPS 的图片无法作为地理对齐锚点。
    # EN: First keep only images with usable GPS, because later alignment needs GPS anchors.
    captures: list[tuple[Path, dict[str, float | None]]] = []
    for image_path in discover_images(input_dir, extensions):
        gps = extract_exif_gps(image_path)
        if gps is None:
            continue
        accuracy = _gps_accuracy_m(image_path)
        if max_accuracy_m is not None and accuracy is not None and accuracy > float(max_accuracy_m):
            continue
        captures.append((image_path, gps))

    if not captures:
        raise ValueError(f"No images with usable GPS EXIF found in {input_dir}")

    # CN: ENU 是局部米制坐标系，需要选择一个 WGS84 原点；默认用第一张有效图片。
    # EN: ENU is a local metric frame, so we choose a WGS84 origin; by default, the first valid image.
    origin_wgs84 = cfg.get("origin_wgs84")
    if origin_wgs84 is None:
        origin_gps = captures[0][1]
        origin_lat = float(origin_gps["lat"])
        origin_lon = float(origin_gps["lon"])
        origin_alt = float(origin_gps["alt"] or 0.0)
    else:
        origin_lat, origin_lon, origin_alt = (float(value) for value in origin_wgs84)

    # CN: 每一帧同时保存原始 GPS、局部 ENU、粗略朝向和相机内参，供 COLMAP/对齐阶段使用。
    # EN: Each frame stores raw GPS, local ENU, rough heading, and camera intrinsics for reconstruction/alignment.
    frames = []
    for image_path, gps in captures:
        e, n, u = gps_to_enu(
            float(gps["lat"]),
            float(gps["lon"]),
            float(gps["alt"] or 0.0),
            origin_lat,
            origin_lon,
            origin_alt,
        )
        heading = 0.0 if gps["heading"] is None else float(gps["heading"])
        frames.append(
            {
                "file_path": image_path.relative_to(input_dir).as_posix(),
                "gps": {
                    "lat": float(gps["lat"]),
                    "lon": float(gps["lon"]),
                    "alt": float(gps["alt"] or 0.0),
                    "heading": heading,
                },
                "enu": {"e": e, "n": n, "u": u},
                "rotation_matrix": heading_to_rotation_matrix(heading).tolist(),
                "intrinsics": estimate_intrinsics(image_path),
            }
        )

    transforms = {
        "origin_lat": origin_lat,
        "origin_lon": origin_lon,
        "origin_alt": origin_alt,
        "frames": frames,
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(transforms, indent=2), encoding="utf-8")
    return transforms


def visualize_camera_positions(transforms: dict, save_path: str | Path | None = None) -> None:
    """Plot camera positions in a top-down ENU view."""
    frames = transforms.get("frames", [])
    east = [frame["enu"]["e"] for frame in frames]
    north = [frame["enu"]["n"] for frame in frames]

    plt.figure()
    plt.scatter(east, north, label="camera")
    for frame in frames:
        e = frame["enu"]["e"]
        n = frame["enu"]["n"]
        heading = frame.get("gps", {}).get("heading") or 0.0
        direction = heading_to_rotation_matrix(float(heading)) @ [0.0, 1.0, 0.0]
        plt.arrow(e, n, direction[0] * 2.0, direction[1] * 2.0, head_width=0.5, length_includes_head=True)

    plt.xlabel("East (m)")
    plt.ylabel("North (m)")
    plt.title("Camera Positions (ENU)")
    plt.axis("equal")
    plt.grid(True)
    if save_path is not None:
        plt.savefig(save_path, bbox_inches="tight")
        plt.close()
    else:
        plt.show()


def generate_transforms_json(
    image_dir: Path,
    output_path: Path,
    origin_wgs84: tuple[float, float, float] | None = None,
) -> Path:
    """Generate a transforms.json file from image metadata."""
    config = {"origin_wgs84": origin_wgs84} if origin_wgs84 is not None else None
    process_image_directory(image_dir, output_path, config=config)
    return output_path


def _main() -> None:
    parser = argparse.ArgumentParser(description="Generate transforms.json from geotagged images.")
    parser.add_argument("image_dir", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=Path("transforms.json"))
    args = parser.parse_args()
    process_image_directory(args.image_dir, args.output)


if __name__ == "__main__":
    _main()
