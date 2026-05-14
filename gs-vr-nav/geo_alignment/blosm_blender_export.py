"""Blender-side Blosm exporter.

Run only inside Blender:
    blender --background --python blosm_blender_export.py -- --config request.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import bpy


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Import an OSM region with Blosm and export FBX.")
    parser.add_argument("--config", type=Path, required=True)
    return parser


def _parse_blender_args() -> argparse.Namespace:
    import sys

    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []
    return _build_parser().parse_args(argv)


def _approx_extent(center_lat: float, center_lon: float, radius_m: float) -> dict[str, float]:
    lat_delta = radius_m / 111_320.0
    lon_scale = max(0.01, math.cos(math.radians(center_lat)))
    lon_delta = radius_m / (111_320.0 * lon_scale)
    return {
        "min_lat": center_lat - lat_delta,
        "max_lat": center_lat + lat_delta,
        "min_lon": center_lon - lon_delta,
        "max_lon": center_lon + lon_delta,
    }


def _clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()


def _enable_blosm_addon() -> None:
    candidates = ("blosm", "blender-osm", "blender_osm")
    for addon_name in candidates:
        try:
            bpy.ops.preferences.addon_enable(module=addon_name)
            return
        except Exception:
            continue


def _set_if_present(target: object, names: tuple[str, ...], value: object) -> None:
    for name in names:
        if hasattr(target, name):
            try:
                setattr(target, name, value)
            except Exception:
                pass


def _configure_blosm_properties(center_lat: float, center_lon: float, radius_m: float) -> None:
    extent = _approx_extent(center_lat, center_lon, radius_m)
    candidate_property_groups = []
    for name in ("blosm", "blender_osm", "osm", "osm_import"):
        if hasattr(bpy.context.scene, name):
            candidate_property_groups.append(getattr(bpy.context.scene, name))

    for props in candidate_property_groups:
        _set_if_present(props, ("lat", "latitude", "centerLat", "center_lat"), center_lat)
        _set_if_present(props, ("lon", "longitude", "centerLon", "center_lon"), center_lon)
        _set_if_present(props, ("radius", "radius_m", "dist", "distance"), radius_m)
        _set_if_present(props, ("minLat", "min_lat", "south", "bottom"), extent["min_lat"])
        _set_if_present(props, ("maxLat", "max_lat", "north", "top"), extent["max_lat"])
        _set_if_present(props, ("minLon", "min_lon", "west", "left"), extent["min_lon"])
        _set_if_present(props, ("maxLon", "max_lon", "east", "right"), extent["max_lon"])
        _set_if_present(props, ("importBuildings", "buildings", "import_buildings"), True)
        _set_if_present(props, ("importRoads", "roads", "import_roads", "importWays"), True)
        _set_if_present(props, ("importPaths", "paths", "import_paths"), True)
        _set_if_present(props, ("importWater", "water", "import_water"), False)
        _set_if_present(props, ("importForests", "forests", "import_forests"), False)
        _set_if_present(props, ("importVegetation", "vegetation", "import_vegetation"), False)
        _set_if_present(props, ("terrain", "importTerrain", "import_terrain"), False)


def _call_operator(category: str, operator_name: str) -> bool:
    category_ops = getattr(bpy.ops, category, None)
    if category_ops is None:
        return False
    operator = getattr(category_ops, operator_name, None)
    if operator is None:
        return False
    try:
        result = operator()
    except Exception:
        return False
    return "FINISHED" in result


def _run_blosm_import() -> None:
    attempts = (
        ("blosm", "import_data"),
        ("blosm", "import_osm"),
        ("blosm", "import_map"),
        ("blosm", "importOpenStreetMap"),
        ("blender_osm", "import_data"),
        ("blender_osm", "import_osm"),
        ("osm", "import_data"),
        ("osm", "import_osm"),
    )
    for category, operator_name in attempts:
        if _call_operator(category, operator_name):
            return

    raise RuntimeError(
        "Could not run a Blosm import operator. Install/enable Blosm in Blender and verify its operator name."
    )


def _mesh_objects() -> list[bpy.types.Object]:
    return [obj for obj in bpy.context.scene.objects if obj.type == "MESH" and obj.data is not None]


def _export_fbx(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    for obj in _mesh_objects():
        obj.select_set(True)
    bpy.ops.export_scene.fbx(
        filepath=str(output_path),
        use_selection=True,
        apply_unit_scale=True,
        bake_space_transform=False,
        object_types={"MESH"},
    )


def main() -> None:
    args = _parse_blender_args()
    request = json.loads(args.config.read_text(encoding="utf-8"))
    output_path = Path(request["output_path"])

    _enable_blosm_addon()
    _clear_scene()
    _configure_blosm_properties(
        float(request["center_lat"]),
        float(request["center_lon"]),
        float(request["radius_m"]),
    )
    _run_blosm_import()

    if not _mesh_objects():
        raise RuntimeError("Blosm import finished but produced no mesh objects.")

    _export_fbx(output_path)


if __name__ == "__main__":
    main()
