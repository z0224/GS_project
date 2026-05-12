"""Download and parse OpenStreetMap data for a local ENU frame."""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from shapely.geometry import (
    LineString,
    MultiLineString,
    MultiPolygon,
    Polygon,
    mapping,
    shape,
)
from shapely.geometry.base import BaseGeometry

from utils.coordinate_utils import gps_to_enu


@dataclass
class OSMData:
    """OpenStreetMap geometry transformed into local East-North coordinates."""

    buildings: list[Polygon]
    roads: list[LineString]
    sidewalks: list[LineString]
    origin_lat: float
    origin_lon: float
    radius_m: float


def _enu_xy(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    east, north, _up = gps_to_enu(lat, lon, 0.0, origin_lat, origin_lon, 0.0)
    return east, north


def _lon_lat(coord: tuple[float, ...]) -> tuple[float, float]:
    return float(coord[0]), float(coord[1])


def _line_to_enu(line: LineString, origin_lat: float, origin_lon: float) -> LineString | None:
    coords = [
        _enu_xy(lat=lat, lon=lon, origin_lat=origin_lat, origin_lon=origin_lon)
        for lon, lat in (_lon_lat(coord) for coord in line.coords)
    ]
    if len(coords) < 2:
        return None
    return LineString(coords)


def _polygon_to_enu(poly: Polygon, origin_lat: float, origin_lon: float) -> Polygon | None:
    exterior = [
        _enu_xy(lat=lat, lon=lon, origin_lat=origin_lat, origin_lon=origin_lon)
        for lon, lat in (_lon_lat(coord) for coord in poly.exterior.coords)
    ]
    if len(exterior) < 4:
        return None

    interiors = [
        [
            _enu_xy(lat=lat, lon=lon, origin_lat=origin_lat, origin_lon=origin_lon)
            for lon, lat in (_lon_lat(coord) for coord in ring.coords)
        ]
        for ring in poly.interiors
        if len(ring.coords) >= 4
    ]
    enu_poly = Polygon(exterior, interiors)
    if not enu_poly.is_valid:
        enu_poly = enu_poly.buffer(0)
    if enu_poly.is_empty:
        return None
    if isinstance(enu_poly, Polygon):
        return enu_poly
    if isinstance(enu_poly, MultiPolygon):
        return max(enu_poly.geoms, key=lambda geom: geom.area, default=None)
    return None


def _iter_polygons(geometry: BaseGeometry | None) -> Iterable[Polygon]:
    if geometry is None or geometry.is_empty:
        return
    if isinstance(geometry, Polygon):
        yield geometry
    elif isinstance(geometry, MultiPolygon):
        yield from geometry.geoms


def _iter_lines(geometry: BaseGeometry | None) -> Iterable[LineString]:
    if geometry is None or geometry.is_empty:
        return
    if isinstance(geometry, LineString):
        yield geometry
    elif isinstance(geometry, MultiLineString):
        yield from geometry.geoms


def _feature_geometries(features: Any) -> Iterable[BaseGeometry]:
    if features is None:
        return
    if hasattr(features, "geometry"):
        yield from features.geometry.dropna()
    elif isinstance(features, Iterable):
        yield from features


def _edges_to_lines(edges: Any) -> list[LineString]:
    lines: list[LineString] = []
    if edges is None or not hasattr(edges, "iterrows"):
        return lines

    for _idx, row in edges.iterrows():
        geometry = row.get("geometry")
        if isinstance(geometry, LineString):
            lines.append(geometry)
    return lines


def download_osm_data(center_lat: float, center_lon: float, radius_m: float = 500) -> OSMData:
    """Download buildings, walkable roads, and footways from OSM around a point."""

    try:
        import osmnx as ox
    except ImportError as exc:  # pragma: no cover - depends on optional runtime setup
        raise ImportError("download_osm_data requires osmnx to be installed") from exc

    center = (center_lat, center_lon)

    building_features = ox.features_from_point(center, tags={"building": True}, dist=radius_m)
    sidewalk_features = ox.features_from_point(
        center,
        tags={"highway": ["footway", "path", "pedestrian"]},
        dist=radius_m,
    )

    graph = ox.graph_from_point(center, dist=radius_m, network_type="walk", simplify=True)
    _nodes, edges = ox.graph_to_gdfs(graph, nodes=True, edges=True)

    buildings: list[Polygon] = []
    for geometry in _feature_geometries(building_features):
        for poly in _iter_polygons(geometry):
            enu_poly = _polygon_to_enu(poly, center_lat, center_lon)
            if enu_poly is not None:
                buildings.append(enu_poly)

    roads: list[LineString] = []
    for line in _edges_to_lines(edges):
        enu_line = _line_to_enu(line, center_lat, center_lon)
        if enu_line is not None:
            roads.append(enu_line)

    sidewalks: list[LineString] = []
    for geometry in _feature_geometries(sidewalk_features):
        for line in _iter_lines(geometry):
            enu_line = _line_to_enu(line, center_lat, center_lon)
            if enu_line is not None:
                sidewalks.append(enu_line)

    return OSMData(
        buildings=buildings,
        roads=roads,
        sidewalks=sidewalks,
        origin_lat=float(center_lat),
        origin_lon=float(center_lon),
        radius_m=float(radius_m),
    )


def _polygon_to_coordinates(poly: Polygon) -> list[list[list[float]]]:
    rings = [list(poly.exterior.coords)]
    rings.extend(list(ring.coords) for ring in poly.interiors)
    return [[[float(x), float(y)] for x, y in ring] for ring in rings]


def _line_to_coordinates(line: LineString) -> list[list[float]]:
    return [[float(x), float(y)] for x, y in line.coords]


def save_osm_data(data: OSMData, output_path: str | Path) -> None:
    """Serialize OSMData to JSON using ENU coordinates."""

    output = {
        "origin_lat": data.origin_lat,
        "origin_lon": data.origin_lon,
        "radius_m": data.radius_m,
        "buildings": [_polygon_to_coordinates(poly) for poly in data.buildings],
        "roads": [_line_to_coordinates(line) for line in data.roads],
        "sidewalks": [_line_to_coordinates(line) for line in data.sidewalks],
    }

    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output, indent=2), encoding="utf-8")


def load_osm_data(input_path: str | Path) -> OSMData:
    """Deserialize OSMData from JSON and restore shapely geometry objects."""

    payload = json.loads(Path(input_path).read_text(encoding="utf-8"))
    return OSMData(
        buildings=[
            Polygon(rings[0], rings[1:])
            for rings in payload.get("buildings", [])
            if rings and len(rings[0]) >= 4
        ],
        roads=[
            LineString(coords)
            for coords in payload.get("roads", [])
            if coords and len(coords) >= 2
        ],
        sidewalks=[
            LineString(coords)
            for coords in payload.get("sidewalks", [])
            if coords and len(coords) >= 2
        ],
        origin_lat=float(payload["origin_lat"]),
        origin_lon=float(payload["origin_lon"]),
        radius_m=float(payload["radius_m"]),
    )


def download_osm_features(
    center_wgs84: tuple[float, float],
    radius_m: float,
) -> dict[str, Any]:
    """Backward-compatible wrapper around download_osm_data."""

    data = download_osm_data(center_wgs84[0], center_wgs84[1], radius_m)
    return {
        "buildings": data.buildings,
        "roads": data.roads,
        "sidewalks": data.sidewalks,
        "origin_lat": data.origin_lat,
        "origin_lon": data.origin_lon,
        "radius_m": data.radius_m,
    }


def load_building_footprints(
    center_wgs84: tuple[float, float],
    radius_m: float,
) -> list[Polygon]:
    """Load OSM building footprints near a WGS84 center point."""

    return download_osm_data(center_wgs84[0], center_wgs84[1], radius_m).buildings


def load_road_network(
    center_wgs84: tuple[float, float],
    radius_m: float,
) -> list[LineString]:
    """Load OSM walk-network road geometry near a WGS84 center point."""

    return download_osm_data(center_wgs84[0], center_wgs84[1], radius_m).roads


def save_osm_cache(features: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Persist downloaded OSM features for repeatable experiments."""

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "osm_data.json"
    data = OSMData(
        buildings=list(features.get("buildings", [])),
        roads=list(features.get("roads", [])),
        sidewalks=list(features.get("sidewalks", [])),
        origin_lat=float(features["origin_lat"]),
        origin_lon=float(features["origin_lon"]),
        radius_m=float(features["radius_m"]),
    )
    save_osm_data(data, output_path)
    return {"osm_data": output_path}


def geometry_to_geojson_dict(geometry: BaseGeometry) -> dict[str, Any]:
    """Convert a shapely geometry to a GeoJSON-like dictionary."""

    return mapping(geometry)


def geometry_from_geojson_dict(data: dict[str, Any]) -> BaseGeometry:
    """Restore a shapely geometry from a GeoJSON-like dictionary."""

    return shape(data)


if __name__ == "__main__":
    raise SystemExit("Use download_osm_data from Python to fetch OpenStreetMap data.")
