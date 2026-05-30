"""Download OpenStreetMap geometry and convert it to local EN coordinates."""

from __future__ import annotations

import argparse
import json
import sys
import urllib.parse
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

try:
    from geo_alignment.generate_blosm_map import canonical_origin_from_transforms, center_from_transforms, radius_from_config
    from utils.coordinate_utils import gps_to_enu
except ImportError:  # pragma: no cover - package import fallback
    from gs_vr_nav.geo_alignment.generate_blosm_map import (
        canonical_origin_from_transforms,
        center_from_transforms,
        radius_from_config,
    )
    from gs_vr_nav.utils.coordinate_utils import gps_to_enu


DEFAULT_OVERPASS_URL = "https://overpass-api.de/api/interpreter"
DEFAULT_CONFIG_PATH = Path("configs/default.yaml")
Fetcher = Callable[[str, str, float], dict[str, Any]]


@dataclass(frozen=True)
class OsmDataRequest:
    center_lat: float
    center_lon: float
    origin_lat: float
    origin_lon: float
    origin_alt: float
    radius_m: float
    output_path: Path
    overpass_url: str
    timeout_s: float


def build_overpass_query(center_lat: float, center_lon: float, radius_m: float) -> str:
    """Build an Overpass query for buildings and walk/navigation-relevant ways."""

    radius = max(1.0, float(radius_m))
    lat = float(center_lat)
    lon = float(center_lon)
    return f"""
[out:json][timeout:60];
(
  way["building"](around:{radius:.3f},{lat:.8f},{lon:.8f});
  relation["building"](around:{radius:.3f},{lat:.8f},{lon:.8f});
  way["highway"](around:{radius:.3f},{lat:.8f},{lon:.8f});
);
out body;
>;
out skel qt;
""".strip()


def fetch_overpass_json(query: str, overpass_url: str = DEFAULT_OVERPASS_URL, timeout_s: float = 60.0) -> dict[str, Any]:
    """POST an Overpass query and return the decoded JSON payload."""

    data = urllib.parse.urlencode({"data": query}).encode("utf-8")
    request = urllib.request.Request(
        overpass_url,
        data=data,
        headers={
            "Accept": "application/json",
            "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8",
            "User-Agent": "gs-vr-nav-osm-data-generator/1.0",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=float(timeout_s)) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        details = exc.read().decode("utf-8", errors="replace").strip()
        message = f"Overpass request failed with HTTP {exc.code}"
        if details:
            message = f"{message}: {details}"
        raise RuntimeError(message) from exc


def generate_osm_data(
    output_path: str | Path,
    *,
    transforms_json_path: str | Path | None = None,
    center_lat: float | None = None,
    center_lon: float | None = None,
    radius_m: float | None = None,
    config_path: str | Path = DEFAULT_CONFIG_PATH,
    overpass_url: str = DEFAULT_OVERPASS_URL,
    timeout_s: float = 60.0,
    fetcher: Fetcher = fetch_overpass_json,
) -> dict[str, Any]:
    """Generate project OSM JSON from Overpass geometry."""

    if transforms_json_path is not None:
        origin_lat, origin_lon, origin_alt = canonical_origin_from_transforms(transforms_json_path)
    elif center_lat is not None and center_lon is not None:
        origin_lat, origin_lon, origin_alt = float(center_lat), float(center_lon), 0.0
    else:
        raise ValueError("Provide either transforms_json_path or both center_lat and center_lon")

    if center_lat is None or center_lon is None:
        if transforms_json_path is None:
            raise ValueError("Provide either transforms_json_path or both center_lat and center_lon")
        center_lat, center_lon = center_from_transforms(transforms_json_path)

    radius = radius_from_config(config_path) if radius_m is None else float(radius_m)
    query = build_overpass_query(float(center_lat), float(center_lon), radius)
    overpass_payload = fetcher(query, overpass_url, float(timeout_s))
    osm_data = osm_payload_to_local_json(
        overpass_payload,
        origin_lat=float(origin_lat),
        origin_lon=float(origin_lon),
        origin_alt=float(origin_alt),
        query_center_lat=float(center_lat),
        query_center_lon=float(center_lon),
        radius_m=radius,
    )

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(osm_data, indent=2), encoding="utf-8")
    return osm_data


def osm_payload_to_local_json(
    payload: dict[str, Any],
    *,
    origin_lat: float,
    origin_lon: float,
    origin_alt: float = 0.0,
    query_center_lat: float | None = None,
    query_center_lon: float | None = None,
    radius_m: float,
) -> dict[str, Any]:
    """Convert Overpass JSON elements to this project's local OSM JSON shape."""

    elements = payload.get("elements", [])
    if not isinstance(elements, list):
        raise ValueError("Overpass payload must contain an elements list")

    nodes: dict[int, tuple[float, float]] = {}
    ways: dict[int, dict[str, Any]] = {}
    relations: list[dict[str, Any]] = []

    for element in elements:
        if not isinstance(element, dict):
            continue
        element_type = element.get("type")
        element_id = element.get("id")
        if element_type == "node" and isinstance(element_id, int):
            if "lat" in element and "lon" in element:
                nodes[element_id] = (float(element["lat"]), float(element["lon"]))
        elif element_type == "way" and isinstance(element_id, int):
            ways[element_id] = element
        elif element_type == "relation":
            relations.append(element)

    buildings: list[list[list[float]]] = []
    roads: list[list[list[float]]] = []

    for way in ways.values():
        tags = way.get("tags") if isinstance(way.get("tags"), dict) else {}
        ring = _way_to_enu_polyline(way, nodes, origin_lat, origin_lon)
        if len(ring) < 2:
            continue
        if _is_building(tags):
            if len(ring) >= 3:
                buildings.append(ring)
        elif _is_road(tags):
            roads.append(ring)

    for relation in relations:
        tags = relation.get("tags") if isinstance(relation.get("tags"), dict) else {}
        if not _is_building(tags):
            continue
        for ring in _relation_outer_rings(relation, ways, nodes, origin_lat, origin_lon):
            if len(ring) >= 3:
                buildings.append(ring)

    return {
        "origin_lat": float(origin_lat),
        "origin_lon": float(origin_lon),
        "origin_alt": float(origin_alt),
        "query_center_lat": float(origin_lat if query_center_lat is None else query_center_lat),
        "query_center_lon": float(origin_lon if query_center_lon is None else query_center_lon),
        "radius_m": float(radius_m),
        "buildings": buildings,
        "roads": roads,
    }


def _way_to_enu_polyline(
    way: dict[str, Any],
    nodes: dict[int, tuple[float, float]],
    origin_lat: float,
    origin_lon: float,
) -> list[list[float]]:
    node_ids = way.get("nodes", [])
    if not isinstance(node_ids, list):
        return []

    points: list[list[float]] = []
    for node_id in node_ids:
        if not isinstance(node_id, int) or node_id not in nodes:
            continue
        lat, lon = nodes[node_id]
        east, north, _up = gps_to_enu(lat, lon, 0.0, origin_lat, origin_lon, 0.0)
        points.append([east, north])
    return points


def _relation_outer_rings(
    relation: dict[str, Any],
    ways: dict[int, dict[str, Any]],
    nodes: dict[int, tuple[float, float]],
    origin_lat: float,
    origin_lon: float,
) -> list[list[list[float]]]:
    members = relation.get("members", [])
    if not isinstance(members, list):
        return []

    rings: list[list[list[float]]] = []
    for member in members:
        if not isinstance(member, dict):
            continue
        if member.get("type") != "way" or member.get("role", "outer") not in ("", "outer"):
            continue
        ref = member.get("ref")
        if not isinstance(ref, int) or ref not in ways:
            continue
        ring = _way_to_enu_polyline(ways[ref], nodes, origin_lat, origin_lon)
        if len(ring) >= 3:
            rings.append(ring)
    return rings


def _is_building(tags: dict[str, Any]) -> bool:
    value = tags.get("building")
    return value not in (None, "no", "false", "0")


def _is_road(tags: dict[str, Any]) -> bool:
    highway = tags.get("highway")
    return highway not in (None, "no", "construction", "proposed")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate local OSM JSON from OpenStreetMap/Overpass.")
    parser.add_argument("--transforms-json", type=Path, default=Path("transforms.json"))
    parser.add_argument("--center-lat", type=float, default=None)
    parser.add_argument("--center-lon", type=float, default=None)
    parser.add_argument("--radius-m", type=float, default=None)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--output", "-o", type=Path, required=True)
    parser.add_argument("--overpass-url", default=DEFAULT_OVERPASS_URL)
    parser.add_argument("--timeout-s", type=float, default=60.0)
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)

    transforms_json_path = args.transforms_json
    if args.center_lat is not None and args.center_lon is not None:
        transforms_json_path = None

    generate_osm_data(
        args.output,
        transforms_json_path=transforms_json_path,
        center_lat=args.center_lat,
        center_lon=args.center_lon,
        radius_m=args.radius_m,
        config_path=args.config,
        overpass_url=args.overpass_url,
        timeout_s=args.timeout_s,
    )
    print(args.output)


if __name__ == "__main__":
    main(sys.argv[1:])
