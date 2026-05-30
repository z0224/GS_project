"""Generate a Unity FBX map asset through Blender + Blosm."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

DEFAULT_CONFIG_PATH = Path("configs/default.yaml")
DEFAULT_OUTPUT_PATH = Path("../gs-vr-nav-unity/Assets/External/BlosmMap/Blosm_Map.fbx")
BLENDER_SCRIPT_PATH = Path(__file__).with_name("blosm_blender_export.py")


Runner = Callable[[Sequence[str]], subprocess.CompletedProcess]


@dataclass(frozen=True)
class BlosmMapRequest:
    """Resolved request passed to Blender."""

    center_lat: float
    center_lon: float
    origin_lat: float
    origin_lon: float
    origin_alt: float
    radius_m: float
    output_path: Path


def canonical_origin_from_transforms(transforms_json_path: str | Path) -> tuple[float, float, float]:
    """Return the canonical WGS84 origin from a transforms.json file."""

    path = Path(transforms_json_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    missing = [key for key in ("origin_lat", "origin_lon", "origin_alt") if key not in payload]
    if missing:
        raise ValueError(f"{path} is missing canonical origin field(s): {', '.join(missing)}")
    return float(payload["origin_lat"]), float(payload["origin_lon"]), float(payload["origin_alt"])


def center_from_transforms(transforms_json_path: str | Path) -> tuple[float, float]:
    """Return the average GPS latitude/longitude from a transforms.json file."""

    path = Path(transforms_json_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    frames = payload.get("frames", [])

    coordinates: list[tuple[float, float]] = []
    for frame in frames:
        gps = frame.get("gps") if isinstance(frame, dict) else None
        if not isinstance(gps, dict):
            continue
        if "lat" not in gps or "lon" not in gps:
            continue
        coordinates.append((float(gps["lat"]), float(gps["lon"])))

    if not coordinates:
        raise ValueError(f"No GPS lat/lon entries found in {path}")

    lat = sum(point[0] for point in coordinates) / len(coordinates)
    lon = sum(point[1] for point in coordinates) / len(coordinates)
    return lat, lon


def radius_from_config(config_path: str | Path = DEFAULT_CONFIG_PATH, default_radius_m: float = 500.0) -> float:
    """Read the OSM radius from the project config."""

    path = Path(config_path)
    if not path.exists():
        return float(default_radius_m)

    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore

        payload = yaml.safe_load(text) or {}
        geo_alignment = payload.get("geo_alignment", {})
        return float(geo_alignment.get("osm_radius_m", default_radius_m))
    except ImportError:
        return _radius_from_simple_yaml(text, default_radius_m)


def _radius_from_simple_yaml(text: str, default_radius_m: float) -> float:
    in_geo_alignment = False
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue

        if not line.startswith((" ", "\t")):
            key = line.split(":", 1)[0].strip()
            in_geo_alignment = key == "geo_alignment"
            continue

        if in_geo_alignment and ":" in line:
            key, value = line.split(":", 1)
            if key.strip() == "osm_radius_m":
                return float(value.strip())

    return float(default_radius_m)


def resolve_request(
    output_path: str | Path = DEFAULT_OUTPUT_PATH,
    *,
    transforms_json_path: str | Path | None = None,
    center_lat: float | None = None,
    center_lon: float | None = None,
    radius_m: float | None = None,
    config_path: str | Path = DEFAULT_CONFIG_PATH,
) -> BlosmMapRequest:
    """Resolve center/radius/output for the Blosm export."""

    if center_lat is None or center_lon is None:
        if transforms_json_path is None:
            raise ValueError("Provide either transforms_json_path or both center_lat and center_lon")
        origin_lat, origin_lon, origin_alt = canonical_origin_from_transforms(transforms_json_path)
        center_lat, center_lon = origin_lat, origin_lon
    elif transforms_json_path is not None:
        origin_lat, origin_lon, origin_alt = canonical_origin_from_transforms(transforms_json_path)
    else:
        origin_lat, origin_lon = float(center_lat), float(center_lon)
        origin_alt = 0.0

    resolved_radius = radius_from_config(config_path) if radius_m is None else float(radius_m)
    return BlosmMapRequest(
        center_lat=float(center_lat),
        center_lon=float(center_lon),
        origin_lat=float(origin_lat),
        origin_lon=float(origin_lon),
        origin_alt=float(origin_alt),
        radius_m=resolved_radius,
        output_path=Path(output_path),
    )


def build_blender_command(
    request: BlosmMapRequest,
    *,
    blender_exe: str | Path = "blender",
    blender_script_path: str | Path = BLENDER_SCRIPT_PATH,
    config_json_path: str | Path,
) -> list[str]:
    """Build the Blender background command."""

    return [
        str(blender_exe),
        "--background",
        "--python",
        str(blender_script_path),
        "--",
        "--config",
        str(config_json_path),
    ]


def generate_blosm_map_asset(
    output_path: str | Path = DEFAULT_OUTPUT_PATH,
    *,
    transforms_json_path: str | Path | None = None,
    center_lat: float | None = None,
    center_lon: float | None = None,
    radius_m: float | None = None,
    config_path: str | Path = DEFAULT_CONFIG_PATH,
    blender_exe: str | Path = "blender",
    runner: Runner = subprocess.run,
) -> Path:
    """Run Blender + Blosm and return the generated FBX path."""

    request = resolve_request(
        output_path,
        transforms_json_path=transforms_json_path,
        center_lat=center_lat,
        center_lon=center_lon,
        radius_m=radius_m,
        config_path=config_path,
    )

    request.output_path.parent.mkdir(parents=True, exist_ok=True)
    blender_path = str(blender_exe)
    if blender_path == "blender" and shutil.which(blender_path) is None:
        raise FileNotFoundError("Blender was not found on PATH. Pass --blender-exe with the Blender executable path.")

    config_dir = request.output_path.parent / f".gs_vr_nav_blosm_{uuid.uuid4().hex}"
    command: list[str] | None = None
    try:
        config_dir.mkdir(parents=True, exist_ok=False)
        config_json_path = config_dir / "blosm_request.json"
        config_json_path.write_text(
            json.dumps(
                {
                    "center_lat": request.center_lat,
                    "center_lon": request.center_lon,
                    "origin_wgs84": {
                        "lat": request.origin_lat,
                        "lon": request.origin_lon,
                        "alt": request.origin_alt,
                    },
                    "blosm_origin_wgs84": {
                        "lat": request.center_lat,
                        "lon": request.center_lon,
                        "alt": request.origin_alt,
                    },
                    "radius_m": request.radius_m,
                    "output_path": str(request.output_path.resolve()),
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        command = build_blender_command(
            request,
            blender_exe=blender_path,
            blender_script_path=BLENDER_SCRIPT_PATH,
            config_json_path=config_json_path,
        )
        completed = runner(command)
    finally:
        shutil.rmtree(config_dir, ignore_errors=True)

    if completed.returncode != 0:
        command_text = " ".join(command or [])
        raise RuntimeError(f"Blender/Blosm export failed with exit code {completed.returncode}: {command_text}")

    return request.output_path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate Unity Blosm_Map.fbx through Blender + Blosm.")
    parser.add_argument("--transforms-json", type=Path, default=Path("transforms.json"))
    parser.add_argument("--center-lat", type=float, default=None)
    parser.add_argument("--center-lon", type=float, default=None)
    parser.add_argument("--radius-m", type=float, default=None)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--output", "-o", type=Path, default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--blender-exe", type=Path, default=Path(os.environ.get("BLENDER_EXE", "blender")))
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)

    transforms_json_path = args.transforms_json
    if args.center_lat is not None and args.center_lon is not None:
        transforms_json_path = None

    output = generate_blosm_map_asset(
        args.output,
        transforms_json_path=transforms_json_path,
        center_lat=args.center_lat,
        center_lon=args.center_lon,
        radius_m=args.radius_m,
        config_path=args.config,
        blender_exe=args.blender_exe,
    )
    print(output)


if __name__ == "__main__":
    main(sys.argv[1:])
