import pytest
from shapely.geometry import LineString, Polygon

from geo_alignment.osm_loader import OSMData, download_osm_data, load_osm_data, save_osm_data


@pytest.mark.slow
def test_download_osm_data() -> None:
    data = download_osm_data(center_lat=-27.4975, center_lon=153.0137, radius_m=500)

    assert data.buildings
    assert data.roads
    assert all(isinstance(poly, Polygon) for poly in data.buildings)
    assert all(poly.is_valid for poly in data.buildings)


def test_save_load_roundtrip(tmp_path) -> None:
    data = OSMData(
        buildings=[
            Polygon([(0, 0), (4, 0), (4, 4), (0, 4)]),
            Polygon([(10, 0), (14, 0), (14, 4), (10, 4)]),
            Polygon([(20, 0), (24, 0), (24, 4), (20, 4)]),
        ],
        roads=[
            LineString([(0, 0), (10, 0)]),
            LineString([(0, 5), (10, 5)]),
            LineString([(0, 10), (10, 10)]),
            LineString([(0, 15), (10, 15)]),
            LineString([(0, 20), (10, 20)]),
        ],
        sidewalks=[LineString([(0, 2), (10, 2)]), LineString([(0, 7), (10, 7)])],
        origin_lat=-27.4975,
        origin_lon=153.0137,
        radius_m=500,
    )
    path = tmp_path / "osm_data.json"

    save_osm_data(data, path)
    loaded = load_osm_data(path)

    assert loaded.origin_lat == data.origin_lat
    assert loaded.origin_lon == data.origin_lon
    assert loaded.radius_m == data.radius_m
    assert len(loaded.buildings) == len(data.buildings)
    assert len(loaded.roads) == len(data.roads)
    assert len(loaded.sidewalks) == len(data.sidewalks)
    for expected, actual in zip(data.buildings, loaded.buildings):
        assert expected.equals_exact(actual, tolerance=1e-9)
    for expected, actual in zip(data.roads, loaded.roads):
        assert expected.equals_exact(actual, tolerance=1e-9)
    for expected, actual in zip(data.sidewalks, loaded.sidewalks):
        assert expected.equals_exact(actual, tolerance=1e-9)
