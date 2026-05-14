"""Generate 3DGS capture metadata from a video and a GPX track."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from bisect import bisect_left
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree

from PIL import Image

try:
    from utils.coordinate_utils import gps_to_enu, heading_to_rotation_matrix
except ImportError:  # pragma: no cover - package import fallback
    from gs_vr_nav.utils.coordinate_utils import gps_to_enu, heading_to_rotation_matrix


@dataclass(frozen=True)
class GPXPoint:
    """A timestamped WGS84 point parsed from a GPX track."""

    timestamp: datetime
    lat: float
    lon: float
    alt: float = 0.0


def estimate_frame_intrinsics(image_path: str | Path) -> dict[str, float | int]:
    """Estimate simple pinhole intrinsics from an extracted video frame."""

    with Image.open(image_path) as image:
        width, height = image.size
    focal = float(max(width, height) * 1.2)
    return {
        "w": int(width),
        "h": int(height),
        "fx": focal,
        "fy": focal,
        "cx": float(width) / 2.0,
        "cy": float(height) / 2.0,
    }


def parse_datetime_utc(value: str) -> datetime:
    """Parse an ISO-like timestamp and normalize it to UTC."""

    text = value.strip()
    if text.endswith("Z"):
        text = f"{text[:-1]}+00:00"
    parsed = datetime.fromisoformat(text)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _local_name(tag: str) -> str:
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def _child_text(element: ElementTree.Element, child_name: str) -> str | None:
    for child in element:
        if _local_name(child.tag) == child_name:
            return child.text
    return None


def load_gpx_points(gpx_path: str | Path) -> list[GPXPoint]:
    """Load timestamped GPX track points sorted by time."""

    path = Path(gpx_path)
    root = ElementTree.parse(path).getroot()
    points: list[GPXPoint] = []

    for element in root.iter():
        if _local_name(element.tag) not in {"trkpt", "rtept", "wpt"}:
            continue
        if "lat" not in element.attrib or "lon" not in element.attrib:
            continue

        time_text = _child_text(element, "time")
        if time_text is None or not time_text.strip():
            continue

        ele_text = _child_text(element, "ele")
        points.append(
            GPXPoint(
                timestamp=parse_datetime_utc(time_text),
                lat=float(element.attrib["lat"]),
                lon=float(element.attrib["lon"]),
                alt=0.0 if ele_text is None or not ele_text.strip() else float(ele_text),
            )
        )

    if not points:
        raise ValueError(f"No timestamped GPX points found in {path}")

    deduplicated: dict[datetime, GPXPoint] = {}
    for point in sorted(points, key=lambda item: item.timestamp):
        deduplicated[point.timestamp] = point
    return list(deduplicated.values())


def interpolate_gpx_point(points: Iterable[GPXPoint], timestamp: datetime) -> GPXPoint:
    """Linearly interpolate a GPX point for an absolute UTC timestamp."""

    sorted_points = list(points)
    if not sorted_points:
        raise ValueError("At least one GPX point is required")

    target = timestamp.astimezone(timezone.utc) if timestamp.tzinfo is not None else timestamp.replace(tzinfo=timezone.utc)
    times = [point.timestamp for point in sorted_points]

    if target < times[0] or target > times[-1]:
        raise ValueError(
            "Frame timestamp is outside the GPX time range: "
            f"{target.isoformat()} not in [{times[0].isoformat()}, {times[-1].isoformat()}]"
        )

    index = bisect_left(times, target)
    if index < len(times) and times[index] == target:
        return sorted_points[index]
    if index == 0:
        return sorted_points[0]

    # CN: 视频帧时间通常不会刚好落在 GPX 采样点上，所以在相邻轨迹点之间线性插值。
    # EN: A video frame rarely lands exactly on a GPX sample, so interpolate between neighboring track points.
    before = sorted_points[index - 1]
    after = sorted_points[index]
    span_s = (after.timestamp - before.timestamp).total_seconds()
    if span_s <= 0.0:
        return before

    ratio = (target - before.timestamp).total_seconds() / span_s
    return GPXPoint(
        timestamp=target,
        lat=before.lat + (after.lat - before.lat) * ratio,
        lon=before.lon + (after.lon - before.lon) * ratio,
        alt=before.alt + (after.alt - before.alt) * ratio,
    )


def extract_video_frames(
    video_path: str | Path,
    frames_dir: str | Path,
    frame_rate: float = 1.0,
) -> list[Path]:
    """Extract JPEG frames from a video using ffmpeg."""

    if frame_rate <= 0.0:
        raise ValueError("frame_rate must be greater than zero")

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg was not found on PATH")

    video = Path(video_path)
    if not video.exists():
        raise FileNotFoundError(f"Video file does not exist: {video}")

    output_dir = Path(frames_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for existing in output_dir.glob("frame_*.jpg"):
        existing.unlink()

    command = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(video),
        "-vf",
        f"fps={frame_rate:g}",
        "-q:v",
        "2",
        str(output_dir / "frame_%06d.jpg"),
    ]

    try:
        subprocess.run(command, check=True, text=True, capture_output=True)
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else ""
        message = f"ffmpeg failed with exit code {exc.returncode}"
        if stderr:
            message = f"{message}: {stderr}"
        raise RuntimeError(message) from exc

    frames = sorted(output_dir.glob("frame_*.jpg"))
    if not frames:
        raise RuntimeError(f"ffmpeg did not extract any frames from {video}")
    return frames


def generate_transforms_from_video_gpx(
    video_path: str | Path,
    gpx_path: str | Path,
    output_dir: str | Path,
    video_start_time: str | datetime,
    frame_rate: float = 1.0,
    origin_wgs84: tuple[float, float, float] | None = None,
) -> dict:
    """Extract video frames, interpolate GPX positions, and write transforms.json."""

    output = Path(output_dir)
    frames_dir = output / "frames"
    # CN: 先把视频转换成图片序列，这样后续流程可以复用图片版 transforms.json 格式。
    # EN: Convert video to an image sequence first, so the rest of the pipeline can reuse the image format.
    frames = extract_video_frames(video_path, frames_dir, frame_rate=frame_rate)
    gpx_points = load_gpx_points(gpx_path)
    start_time = (
        parse_datetime_utc(video_start_time)
        if isinstance(video_start_time, str)
        else video_start_time.astimezone(timezone.utc)
    )

    # CN: 根据“视频开始绝对时间 + 帧序号/帧率”计算每帧的真实 UTC 时间。
    # EN: Compute each frame's UTC time from video start time plus frame index / frame rate.
    frame_gps: list[GPXPoint] = []
    for index, _frame_path in enumerate(frames):
        frame_time = start_time.timestamp() + index / frame_rate
        frame_gps.append(interpolate_gpx_point(gpx_points, datetime.fromtimestamp(frame_time, tz=timezone.utc)))

    if origin_wgs84 is None:
        first = frame_gps[0]
        origin_lat, origin_lon, origin_alt = first.lat, first.lon, first.alt
    else:
        origin_lat, origin_lon, origin_alt = (float(value) for value in origin_wgs84)

    frame_entries = []
    for frame_path, point in zip(frames, frame_gps):
        e, n, u = gps_to_enu(point.lat, point.lon, point.alt, origin_lat, origin_lon, origin_alt)
        heading = 0.0
        frame_entries.append(
            {
                "file_path": frame_path.relative_to(output).as_posix(),
                "gps": {
                    "lat": float(point.lat),
                    "lon": float(point.lon),
                    "alt": float(point.alt),
                    "heading": heading,
                },
                "enu": {"e": e, "n": n, "u": u},
                "rotation_matrix": heading_to_rotation_matrix(heading).tolist(),
                "intrinsics": estimate_frame_intrinsics(frame_path),
            }
        )

    transforms = {
        "origin_lat": float(origin_lat),
        "origin_lon": float(origin_lon),
        "origin_alt": float(origin_alt),
        "frames": frame_entries,
    }

    output.mkdir(parents=True, exist_ok=True)
    (output / "transforms.json").write_text(json.dumps(transforms, indent=2), encoding="utf-8")
    return transforms


def _main() -> None:
    parser = argparse.ArgumentParser(description="Generate transforms.json from a video and a GPX track.")
    parser.add_argument("video_path", type=Path)
    parser.add_argument("gpx_path", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--video-start-time", required=True, help="Absolute video start time, e.g. 2026-05-11T10:00:00Z")
    parser.add_argument("--frame-rate", type=float, default=1.0)
    args = parser.parse_args()

    generate_transforms_from_video_gpx(
        video_path=args.video_path,
        gpx_path=args.gpx_path,
        output_dir=args.output_dir,
        video_start_time=args.video_start_time,
        frame_rate=args.frame_rate,
    )


if __name__ == "__main__":
    _main()
