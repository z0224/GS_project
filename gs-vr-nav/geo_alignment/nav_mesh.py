"""Generate a 2D navigation mesh from OpenStreetMap geometry."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import nearest_points, unary_union

from geo_alignment.osm_loader import OSMData


def _polygons_from_geometry(geometry: BaseGeometry | None) -> list[Polygon]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return list(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        polygons: list[Polygon] = []
        for child in geometry.geoms:
            polygons.extend(_polygons_from_geometry(child))
        return polygons
    return []


def _as_multipolygon(geometry: BaseGeometry | None) -> MultiPolygon:
    return MultiPolygon(_polygons_from_geometry(geometry))


def _union_polygons(polygons: Iterable[BaseGeometry]) -> MultiPolygon:
    polygon_list = [poly for poly in polygons if poly is not None and not poly.is_empty]
    if not polygon_list:
        return MultiPolygon([])
    return _as_multipolygon(unary_union(polygon_list))


def _polygon_to_coordinates(poly: Polygon) -> list[list[list[float]]]:
    rings = [list(poly.exterior.coords)]
    rings.extend(list(ring.coords) for ring in poly.interiors)
    return [[[float(x), float(y)] for x, y in ring] for ring in rings]


def _multipolygon_to_coordinates(multipolygon: MultiPolygon) -> list[list[list[list[float]]]]:
    return [_polygon_to_coordinates(poly) for poly in multipolygon.geoms]


def _coordinates_to_multipolygon(coordinates: list[Any]) -> MultiPolygon:
    polygons = [
        Polygon(rings[0], rings[1:])
        for rings in coordinates
        if rings and len(rings[0]) >= 4
    ]
    return MultiPolygon(polygons)


@dataclass
class NavMesh:
    """2D walkable and obstacle geometry in local ENU coordinates."""

    walkable_area: MultiPolygon
    obstacle_area: MultiPolygon
    bounds: tuple[float, float, float, float]

    @classmethod
    def from_osm_data(
        cls,
        osm_data: OSMData,
        road_buffer_m: float = 3.0,
        sidewalk_buffer_m: float = 2.0,
        collision_margin_m: float = 0.3,
    ) -> "NavMesh":
        """Build a navigation mesh from roads, sidewalks, and building obstacles."""

        road_surfaces = [
            line.buffer(road_buffer_m) for line in osm_data.roads if not line.is_empty
        ]
        sidewalk_surfaces = [
            line.buffer(sidewalk_buffer_m) for line in osm_data.sidewalks if not line.is_empty
        ]
        raw_walkable = unary_union(road_surfaces + sidewalk_surfaces)
        obstacles = _union_polygons(poly.buffer(collision_margin_m) for poly in osm_data.buildings)
        walkable = _as_multipolygon(raw_walkable.difference(obstacles))
        bounds = (
            tuple(float(value) for value in walkable.bounds)
            if not walkable.is_empty
            else (0.0, 0.0, 0.0, 0.0)
        )
        return cls(walkable_area=walkable, obstacle_area=obstacles, bounds=bounds)

    def is_walkable(self, x: float, y: float) -> bool:
        """Return True when a point is strictly inside the walkable area."""

        return bool(Point(x, y).within(self.walkable_area))

    def clamp_to_walkable(self, x: float, y: float) -> tuple[float, float]:
        """Return the input point or the nearest point on the walkable area."""

        if self.is_walkable(x, y):
            return float(x), float(y)
        if self.walkable_area.is_empty:
            return float(x), float(y)
        nearest = nearest_points(Point(x, y), self.walkable_area)[1]
        return float(nearest.x), float(nearest.y)

    def clamp_movement(
        self,
        from_x: float,
        from_y: float,
        to_x: float,
        to_y: float,
    ) -> tuple[float, float]:
        """Clamp movement to the last walkable point along a straight segment."""

        if self.is_walkable(to_x, to_y):
            return float(to_x), float(to_y)

        if not self.is_walkable(from_x, from_y):
            return self.clamp_to_walkable(from_x, from_y)

        low = np.array([from_x, from_y], dtype=float)
        high = np.array([to_x, to_y], dtype=float)

        for _ in range(40):
            mid = (low + high) * 0.5
            if self.is_walkable(float(mid[0]), float(mid[1])):
                low = mid
            else:
                high = mid

        return float(low[0]), float(low[1])

    def to_json(self) -> dict[str, Any]:
        """Serialize this navigation mesh to a GeoJSON-like dictionary."""

        return {
            "walkable_area": {
                "type": "MultiPolygon",
                "coordinates": _multipolygon_to_coordinates(self.walkable_area),
            },
            "obstacle_area": {
                "type": "MultiPolygon",
                "coordinates": _multipolygon_to_coordinates(self.obstacle_area),
            },
            "bounds": [float(value) for value in self.bounds],
        }

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> "NavMesh":
        """Deserialize a navigation mesh from a GeoJSON-like dictionary."""

        walkable_data = data["walkable_area"]
        obstacle_data = data["obstacle_area"]
        walkable = (
            _coordinates_to_multipolygon(walkable_data.get("coordinates", []))
            if isinstance(walkable_data, dict)
            else _coordinates_to_multipolygon(walkable_data)
        )
        obstacles = (
            _coordinates_to_multipolygon(obstacle_data.get("coordinates", []))
            if isinstance(obstacle_data, dict)
            else _coordinates_to_multipolygon(obstacle_data)
        )
        bounds = tuple(float(value) for value in data.get("bounds", walkable.bounds))
        return cls(walkable_area=walkable, obstacle_area=obstacles, bounds=bounds)


def build_walkable_area(
    road_geometries: Iterable[BaseGeometry],
    sidewalk_geometries: Iterable[BaseGeometry] | None = None,
    obstacle_geometries: Iterable[BaseGeometry] | None = None,
    road_buffer_m: float = 3.0,
    sidewalk_buffer_m: float = 2.0,
) -> BaseGeometry:
    """Backward-compatible helper returning the walkable area geometry."""

    osm_data = OSMData(
        buildings=[geom for geom in obstacle_geometries or [] if isinstance(geom, Polygon)],
        roads=[geom for geom in road_geometries if isinstance(geom, LineString)],
        sidewalks=[geom for geom in sidewalk_geometries or [] if isinstance(geom, LineString)],
        origin_lat=0.0,
        origin_lon=0.0,
        radius_m=0.0,
    )
    return NavMesh.from_osm_data(
        osm_data,
        road_buffer_m=road_buffer_m,
        sidewalk_buffer_m=sidewalk_buffer_m,
    ).walkable_area


def constrain_point_to_nav_mesh(
    point_xy: tuple[float, float],
    nav_mesh: BaseGeometry,
) -> tuple[float, float]:
    """Project a point onto a shapely navigation mesh geometry if needed."""

    point = Point(point_xy)
    if point.within(nav_mesh):
        return float(point.x), float(point.y)
    if nav_mesh.is_empty:
        return float(point.x), float(point.y)
    nearest = nearest_points(point, nav_mesh)[1]
    return float(nearest.x), float(nearest.y)


def export_nav_mesh_geojson(nav_mesh: BaseGeometry, output_path: Path) -> Path:
    """Export a navigation mesh geometry as GeoJSON for debugging or rendering."""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(nav_mesh.__geo_interface__, indent=2), encoding="utf-8")
    return output_path


def _plot_polygon(ax: Any, polygon: Polygon, color: str, alpha: float) -> None:
    x, y = polygon.exterior.xy
    ax.fill(x, y, color=color, alpha=alpha)
    for ring in polygon.interiors:
        hole_x, hole_y = ring.xy
        ax.fill(hole_x, hole_y, color="white", alpha=1.0)


def visualize_nav_mesh(
    nav_mesh: NavMesh,
    splat_positions: np.ndarray | None = None,
    save_path: str | Path | None = None,
) -> None:
    """Visualize walkable and obstacle areas, optionally with splat positions."""

    fig, ax = plt.subplots()

    for polygon in nav_mesh.walkable_area.geoms:
        _plot_polygon(ax, polygon, "#00ff00", 0.3)
    for polygon in nav_mesh.obstacle_area.geoms:
        _plot_polygon(ax, polygon, "#ff0000", 0.5)

    if splat_positions is not None:
        positions = np.asarray(splat_positions)
        if positions.ndim != 2 or positions.shape[1] < 2:
            raise ValueError("splat_positions must have shape (N, 2+) when provided")
        ax.scatter(positions[:, 0], positions[:, 1], color="blue", alpha=0.1, s=1)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(True)
    ax.set_title("Navigation Mesh Overview")

    if save_path is not None:
        fig.savefig(save_path, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    raise SystemExit("Use NavMesh.from_osm_data from Python to generate navigation meshes.")
