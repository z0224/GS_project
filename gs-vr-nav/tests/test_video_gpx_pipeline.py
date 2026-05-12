from __future__ import annotations

import json
import shutil
import subprocess

import numpy as np
import pytest
from PIL import Image

from data.video_gpx_pipeline import (
    generate_transforms_from_video_gpx,
    interpolate_gpx_point,
    load_gpx_points,
    parse_datetime_utc,
)


def _write_gpx(path, points: list[tuple[str, float, float, float]]) -> None:
    track_points = "\n".join(
        f"""
        <trkpt lat="{lat}" lon="{lon}">
          <ele>{alt}</ele>
          <time>{timestamp}</time>
        </trkpt>
        """
        for timestamp, lat, lon, alt in points
    )
    path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="gs-vr-nav-test" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <trkseg>
      {track_points}
    </trkseg>
  </trk>
</gpx>
""",
        encoding="utf-8",
    )


def test_load_gpx_points_parses_timestamped_track(tmp_path) -> None:
    gpx_path = tmp_path / "track.gpx"
    _write_gpx(
        gpx_path,
        [
            ("2026-05-11T10:00:01Z", -27.1, 153.1, 12.0),
            ("2026-05-11T10:00:00Z", -27.0, 153.0, 10.0),
        ],
    )

    points = load_gpx_points(gpx_path)

    assert len(points) == 2
    assert points[0].timestamp.isoformat() == "2026-05-11T10:00:00+00:00"
    assert points[0].lat == -27.0
    assert points[0].lon == 153.0
    assert points[0].alt == 10.0


def test_interpolate_gpx_point_between_samples(tmp_path) -> None:
    gpx_path = tmp_path / "track.gpx"
    _write_gpx(
        gpx_path,
        [
            ("2026-05-11T10:00:00Z", -27.0, 153.0, 10.0),
            ("2026-05-11T10:00:10Z", -27.1, 153.2, 20.0),
        ],
    )

    point = interpolate_gpx_point(load_gpx_points(gpx_path), parse_datetime_utc("2026-05-11T10:00:05Z"))

    assert point.lat == pytest.approx(-27.05)
    assert point.lon == pytest.approx(153.1)
    assert point.alt == pytest.approx(15.0)


def test_interpolate_gpx_point_rejects_out_of_range(tmp_path) -> None:
    gpx_path = tmp_path / "track.gpx"
    _write_gpx(
        gpx_path,
        [
            ("2026-05-11T10:00:00Z", -27.0, 153.0, 10.0),
            ("2026-05-11T10:00:10Z", -27.1, 153.2, 20.0),
        ],
    )

    with pytest.raises(ValueError, match="outside the GPX time range"):
        interpolate_gpx_point(load_gpx_points(gpx_path), parse_datetime_utc("2026-05-11T09:59:59Z"))


def test_generate_transforms_from_video_gpx_shape(tmp_path, monkeypatch) -> None:
    output_dir = tmp_path / "capture"
    frames_dir = output_dir / "frames"
    frames_dir.mkdir(parents=True)
    frame_1 = frames_dir / "frame_000001.jpg"
    frame_2 = frames_dir / "frame_000002.jpg"
    Image.new("RGB", (64, 48), color="white").save(frame_1)
    Image.new("RGB", (64, 48), color="black").save(frame_2)

    gpx_path = tmp_path / "track.gpx"
    _write_gpx(
        gpx_path,
        [
            ("2026-05-11T10:00:00Z", -27.0, 153.0, 10.0),
            ("2026-05-11T10:00:02Z", -27.0002, 153.0002, 12.0),
        ],
    )

    def fake_extract_video_frames(video_path, frames_dir, frame_rate=1.0):
        return [frame_1, frame_2]

    monkeypatch.setattr("data.video_gpx_pipeline.extract_video_frames", fake_extract_video_frames)

    transforms = generate_transforms_from_video_gpx(
        video_path=tmp_path / "video.mp4",
        gpx_path=gpx_path,
        output_dir=output_dir,
        video_start_time="2026-05-11T10:00:00Z",
        frame_rate=1.0,
    )

    saved = json.loads((output_dir / "transforms.json").read_text(encoding="utf-8"))
    assert saved == transforms
    assert len(transforms["frames"]) == 2
    assert transforms["frames"][0]["file_path"] == "frames/frame_000001.jpg"
    assert transforms["frames"][1]["gps"]["lat"] == pytest.approx(-27.0001)
    assert transforms["frames"][0]["gps"]["heading"] == 0.0
    np.testing.assert_allclose(transforms["frames"][0]["rotation_matrix"], np.eye(3))
    assert transforms["frames"][0]["intrinsics"]["w"] == 64
    assert transforms["frames"][0]["intrinsics"]["h"] == 48


def test_extract_video_frames_with_ffmpeg(tmp_path) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        pytest.skip("ffmpeg is not available")

    video_path = tmp_path / "video.mp4"
    subprocess.run(
        [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "lavfi",
            "-i",
            "testsrc=size=64x48:rate=2",
            "-t",
            "2",
            "-pix_fmt",
            "yuv420p",
            str(video_path),
        ],
        check=True,
    )

    gpx_path = tmp_path / "track.gpx"
    _write_gpx(
        gpx_path,
        [
            ("2026-05-11T10:00:00Z", -27.0, 153.0, 10.0),
            ("2026-05-11T10:00:05Z", -27.001, 153.001, 11.0),
        ],
    )

    transforms = generate_transforms_from_video_gpx(
        video_path=video_path,
        gpx_path=gpx_path,
        output_dir=tmp_path / "capture",
        video_start_time="2026-05-11T10:00:00Z",
        frame_rate=1.0,
    )

    assert len(transforms["frames"]) == 2
    assert (tmp_path / "capture" / "frames" / "frame_000001.jpg").exists()
    assert (tmp_path / "capture" / "frames" / "frame_000002.jpg").exists()
