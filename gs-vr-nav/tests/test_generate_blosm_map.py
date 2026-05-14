import json
import subprocess

import pytest

from geo_alignment.generate_blosm_map import generate_blosm_map_asset, radius_from_config, resolve_request


def _write_transforms(path):
    payload = {
        "frames": [
            {"file_path": "frames/frame_000001.jpg", "gps": {"lat": -27.0, "lon": 153.0}},
            {"file_path": "frames/frame_000002.jpg", "gps": {"lat": -27.2, "lon": 153.4}},
        ]
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_radius_from_config(tmp_path) -> None:
    config_path = tmp_path / "default.yaml"
    config_path.write_text("geo_alignment:\n  osm_radius_m: 321\n", encoding="utf-8")

    assert radius_from_config(config_path) == 321.0


def test_resolve_request_from_transforms_and_config(tmp_path) -> None:
    transforms_path = tmp_path / "transforms.json"
    config_path = tmp_path / "default.yaml"
    output_path = tmp_path / "Blosm_Map.fbx"
    _write_transforms(transforms_path)
    config_path.write_text("geo_alignment:\n  osm_radius_m: 456\n", encoding="utf-8")

    request = resolve_request(
        output_path,
        transforms_json_path=transforms_path,
        config_path=config_path,
    )

    assert request.center_lat == pytest.approx(-27.1)
    assert request.center_lon == pytest.approx(153.2)
    assert request.radius_m == 456.0
    assert request.output_path == output_path


def test_generate_blosm_map_asset_builds_blender_command(tmp_path) -> None:
    output_path = tmp_path / "Assets" / "External" / "BlosmMap" / "Blosm_Map.fbx"
    calls = []
    configs = []

    def fake_runner(command):
        calls.append(command)
        config_index = command.index("--config") + 1
        configs.append(json.loads(open(command[config_index], encoding="utf-8").read()))
        return subprocess.CompletedProcess(command, 0)

    result = generate_blosm_map_asset(
        output_path,
        center_lat=-27.485,
        center_lon=153.0033,
        radius_m=250,
        blender_exe="C:/Program Files/Blender Foundation/Blender 4.2/blender.exe",
        runner=fake_runner,
    )

    assert result == output_path
    assert calls[0][1:3] == ["--background", "--python"]
    assert calls[0][-2] == "--config"
    assert configs == [
        {
            "center_lat": -27.485,
            "center_lon": 153.0033,
            "radius_m": 250.0,
            "output_path": str(output_path.resolve()),
        }
    ]
