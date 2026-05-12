import numpy as np

from utils.coordinate_utils import batch_gps_to_enu, gps_to_enu


def test_batch_gps_to_enu_shape() -> None:
    coords = np.column_stack(
        [
            np.linspace(-27.47, -27.46, 10),
            np.linspace(153.02, 153.03, 10),
        ]
    )

    enu = batch_gps_to_enu(coords, origin_lat=-27.47, origin_lon=153.02)

    assert enu.shape == (10, 3)


def test_origin_is_zero() -> None:
    enu = gps_to_enu(-27.4705, 153.0260, 18.0, -27.4705, 153.0260, 18.0)

    assert np.allclose(enu, (0.0, 0.0, 0.0), atol=1e-6)
