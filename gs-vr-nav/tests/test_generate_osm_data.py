import json

import pytest

from geo_alignment.generate_osm_data import build_overpass_query, generate_osm_data, osm_payload_to_local_json


def test_build_overpass_query_contains_center_radius_and_tags() -> None:
    query = build_overpass_query(-27.485, 153.003, 250)

    assert "around:250.000,-27.48500000,153.00300000" in query
    assert 'way["building"]' in query
    assert 'relation["building"]' in query
    assert 'way["highway"]' in query


def test_osm_payload_to_local_json_converts_buildings_and_roads() -> None:
    payload = {
        "elements": [
            {"type": "node", "id": 1, "lat": -27.0, "lon": 153.0},
            {"type": "node", "id": 2, "lat": -27.0, "lon": 153.0001},
            {"type": "node", "id": 3, "lat": -27.0001, "lon": 153.0001},
            {"type": "node", "id": 4, "lat": -27.0001, "lon": 153.0},
            {"type": "node", "id": 5, "lat": -27.0002, "lon": 153.0},
            {"type": "way", "id": 10, "nodes": [1, 2, 3, 4, 1], "tags": {"building": "yes"}},
            {"type": "way", "id": 11, "nodes": [4, 5], "tags": {"highway": "footway"}},
        ]
    }

    result = osm_payload_to_local_json(payload, origin_lat=-27.0, origin_lon=153.0, radius_m=250)

    assert result["origin_lat"] == -27.0
    assert result["origin_lon"] == 153.0
    assert result["origin_alt"] == 0.0
    assert result["query_center_lat"] == -27.0
    assert result["query_center_lon"] == 153.0
    assert result["radius_m"] == 250.0
    assert len(result["buildings"]) == 1
    assert len(result["buildings"][0]) == 5
    assert len(result["roads"]) == 1
    assert result["buildings"][0][0] == pytest.approx([0.0, 0.0], abs=1e-5)


def test_generate_osm_data_uses_transforms_center_and_writes_file(tmp_path) -> None:
    transforms_path = tmp_path / "transforms.json"
    transforms_path.write_text(
        json.dumps(
            {
                "origin_lat": -27.5,
                "origin_lon": 153.1,
                "origin_alt": 42.0,
                "frames": [
                    {"gps": {"lat": -27.0, "lon": 153.0}},
                    {"gps": {"lat": -27.2, "lon": 153.4}},
                ]
            }
        ),
        encoding="utf-8",
    )
    output_path = tmp_path / "osm_data.json"
    calls = []

    def fake_fetcher(query, overpass_url, timeout_s):
        calls.append((query, overpass_url, timeout_s))
        return {"elements": []}

    result = generate_osm_data(
        output_path,
        transforms_json_path=transforms_path,
        radius_m=123,
        overpass_url="https://example.test/overpass",
        timeout_s=7,
        fetcher=fake_fetcher,
    )

    assert result == json.loads(output_path.read_text(encoding="utf-8"))
    assert result["origin_lat"] == pytest.approx(-27.5)
    assert result["origin_lon"] == pytest.approx(153.1)
    assert result["origin_alt"] == pytest.approx(42.0)
    assert result["query_center_lat"] == pytest.approx(-27.1)
    assert result["query_center_lon"] == pytest.approx(153.2)
    assert result["radius_m"] == 123.0
    assert "around:123.000,-27.10000000,153.20000000" in calls[0][0]
    assert calls[0][1:] == ("https://example.test/overpass", 7.0)
