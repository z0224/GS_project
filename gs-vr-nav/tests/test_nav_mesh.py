from shapely.geometry import LineString, Point, Polygon

from geo_alignment.nav_mesh import NavMesh
from geo_alignment.osm_loader import OSMData


def _synthetic_osm_data() -> OSMData:
    return OSMData(
        buildings=[Polygon([(10, 10), (20, 10), (20, 20), (10, 20)])],
        roads=[LineString([(0, 0), (100, 0)])],
        sidewalks=[LineString([(0, 5), (100, 5)])],
        origin_lat=0.0,
        origin_lon=0.0,
        radius_m=100.0,
    )


def _synthetic_nav_mesh() -> NavMesh:
    return NavMesh.from_osm_data(
        _synthetic_osm_data(),
        road_buffer_m=3.0,
        sidewalk_buffer_m=2.0,
        collision_margin_m=0.3,
    )


def test_nav_mesh_from_synthetic() -> None:
    nav_mesh = _synthetic_nav_mesh()

    assert nav_mesh.is_walkable(50, 0)
    assert nav_mesh.is_walkable(50, 5)
    assert not nav_mesh.is_walkable(15, 15)
    assert not nav_mesh.is_walkable(50, 50)


def test_clamp_to_walkable() -> None:
    nav_mesh = _synthetic_nav_mesh()

    x, y = nav_mesh.clamp_to_walkable(15, 15)

    assert nav_mesh.walkable_area.covers(Point(x, y))
    assert not nav_mesh.obstacle_area.contains(Point(x, y))


def test_clamp_movement() -> None:
    nav_mesh = _synthetic_nav_mesh()

    x, y = nav_mesh.clamp_movement(5, 0, 15, 15)

    assert nav_mesh.walkable_area.covers(Point(x, y))
    assert not nav_mesh.obstacle_area.contains(Point(x, y))
    assert y < 10


def test_json_roundtrip() -> None:
    nav_mesh = _synthetic_nav_mesh()
    loaded = NavMesh.from_json(nav_mesh.to_json())
    test_points = [
        (50, 0),
        (50, 5),
        (15, 15),
        (50, 50),
        (5, 0),
        (0, 0),
        (100, 0),
        (12, 5),
        (12, 9),
        (25, 5),
    ]

    for x, y in test_points:
        assert loaded.is_walkable(x, y) == nav_mesh.is_walkable(x, y)
