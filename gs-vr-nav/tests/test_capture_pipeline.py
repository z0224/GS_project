import numpy as np
from PIL import Image

from data.capture_pipeline import estimate_intrinsics, extract_exif_gps
from utils.coordinate_utils import enu_to_gps, gps_to_enu, heading_to_rotation_matrix


def test_gps_to_enu_known_values() -> None:
    e, n, u = gps_to_enu(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    assert abs(e) < 1.0
    assert abs(u) < 1000.0
    assert abs(n - 110_574.0) / 110_574.0 < 0.01


def test_enu_roundtrip() -> None:
    lat, lon, alt = -27.4705, 153.0260, 18.0
    origin = (-27.4698, 153.0252, 12.0)

    e, n, u = gps_to_enu(lat, lon, alt, *origin)
    roundtrip_lat, roundtrip_lon, roundtrip_alt = enu_to_gps(e, n, u, *origin)

    assert abs(roundtrip_lat - lat) < 0.001
    assert abs(roundtrip_lon - lon) < 0.001
    assert abs(roundtrip_alt - alt) < 0.1


def test_heading_rotation() -> None:
    forward = np.array([0.0, 1.0, 0.0])

    assert np.allclose(heading_to_rotation_matrix(0.0), np.eye(3))
    assert np.allclose(heading_to_rotation_matrix(90.0) @ forward, [1.0, 0.0, 0.0], atol=1e-7)


def test_extract_exif_gps_no_gps(tmp_path) -> None:
    image_path = tmp_path / "plain.jpg"
    Image.new("RGB", (32, 24), color="white").save(image_path)

    assert extract_exif_gps(image_path) is None


def test_estimate_intrinsics(tmp_path) -> None:
    image_path = tmp_path / "phone.jpg"
    Image.new("RGB", (4000, 3000), color="white").save(image_path)

    intrinsics = estimate_intrinsics(image_path)

    assert intrinsics["w"] == 4000
    assert intrinsics["h"] == 3000
    assert intrinsics["fx"] == 4800.0
    assert intrinsics["fy"] == 4800.0
    assert intrinsics["cx"] == 2000.0
    assert intrinsics["cy"] == 1500.0
