# GS-VR-Nav: Geographic-Aligned Gaussian Splatting for VR Navigation

GS-VR-Nav is a research prototype for turning outdoor 3D Gaussian Splatting
reconstructions into geographically aligned, walkable VR environments. The
system combines a Python offline pipeline for capture metadata, reconstruction,
OpenStreetMap alignment, and navigation-mesh generation with a Unity URP/XRI
demo project for continuous walking-style navigation.

The central idea is to align a visually reconstructed Gaussian Splatting scene
with a real-world local East-North-Up (ENU) coordinate frame, then use OSM roads,
sidewalks, and building footprints as invisible spatial constraints for VR
movement. The user sees the splat scene, while the navigation logic uses the
geographic mesh to keep movement plausible.

## Research Question

Can outdoor Gaussian Splatting reconstructions be aligned with known geographic
data to support continuous, realistic walkable navigation in VR environments?

## Repository Layout

This folder contains the Python research pipeline:

- `data`: image discovery, EXIF GPS/heading extraction, and `transforms.json`
  generation.
- `reconstruction`: wrappers for external COLMAP and official 3DGS training,
  plus PLY loading, transformation, and export utilities.
- `geo_alignment`: WGS84/ENU OSM loading, Procrustes/Umeyama alignment, and
  navigation-mesh generation.
- `navigation`: Python-side locomotion API placeholder for future simulation.
- `utils`: coordinate conversion helpers and visualization placeholders.
- `vr_renderer`: Unity script copies used by the Unity demo project.
- `tests`: unit and integration tests for the implemented Python pipeline.

The sibling folder `../gs-vr-nav-unity` is a Unity 6000.2.13f1 URP/XRI demo
project. It consumes exported assets from this Python pipeline:

- `../gs-vr-nav-unity/Assets/StreamingAssets/nav_mesh.json`
- `../gs-vr-nav-unity/Assets/StreamingAssets/SplatScenes/aligned_scene.ply`

## End-to-End Workflow

1. Capture outdoor photos with GPS metadata, or record `video.mp4` together
   with a timestamped `track.gpx`.
2. Generate `transforms.json` from image EXIF metadata or from interpolated
   GPX positions for extracted video frames.
3. Run COLMAP to estimate camera poses and sparse structure, then train a 3DGS
   model from the COLMAP workspace.
4. Estimate the COLMAP-to-ENU similarity transform using GPS-derived camera
   anchors, saving `alignment.npz`.
5. Download or load OSM data around the capture site and build `nav_mesh.json`.
6. Apply the alignment transform to the trained splat PLY and export
   `aligned_scene.ply`.
7. Copy `nav_mesh.json` and `aligned_scene.ply` into the Unity `StreamingAssets`
   paths listed above.
8. Open the Unity demo scene and run it with the XR Device Simulator or an
   appropriate XR setup.

## Environment Setup

Create a Python 3.10+ environment from this directory:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .
```

On Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
pip install -e .
```

COLMAP is required outside the Python environment for real reconstruction. The
official 3D Gaussian Splatting repository is also required for training. Install
both separately and make sure the `colmap` executable is available on `PATH`.

Unity rendering is handled by the sibling `../gs-vr-nav-unity` project. Its
package manifest includes XR Interaction Toolkit, Input System, URP,
Newtonsoft.Json, and UnityGaussianSplatting.

## Python Pipeline Examples

### 1. Generate Capture Metadata

From geotagged photos:

```python
from pathlib import Path

from data.capture_pipeline import generate_transforms_json

generate_transforms_json(
    image_dir=Path("captures/site_a/images"),
    output_path=Path("captures/site_a/transforms.json"),
)
```

This reads GPS EXIF metadata, chooses the first valid capture as the default ENU
origin, estimates simple camera intrinsics from image dimensions, and writes a
3DGS-friendly `transforms.json`.

From a video and GPX track:

```powershell
python -m data.video_gpx_pipeline video.mp4 track.gpx `
  --output-dir captures/site_a `
  --video-start-time 2026-05-11T10:00:00Z `
  --frame-rate 1
```

This extracts JPEG frames to `captures/site_a/frames`, interpolates GPX
`lat/lon/ele` by frame timestamp, and writes `captures/site_a/transforms.json`.
Heading is intentionally set to `0.0`; COLMAP and Umeyama alignment determine
the scene orientation later.

For video capture, start GPX logging first, wait 10-20 seconds for GPS to
stabilize, then start the video. Walk slowly through a 30-150 m route with
visible static features and avoid mostly sky, heavy motion blur, crowds, or
large reflective surfaces.

### 2. Run COLMAP and 3DGS Training

```python
from pathlib import Path

from reconstruction.train import GaussianSplattingTrainer

trainer = GaussianSplattingTrainer(
    gs_repo_path=Path("../external/gaussian-splatting"),
    colmap_workspace=Path("outputs/colmap/site_a"),
    output_dir=Path("outputs/3dgs/site_a"),
    config={"iterations": 30000, "sh_degree": 3},
)

colmap_model_dir = trainer.run_colmap(Path("captures/site_a/images"))
point_cloud_ply = trainer.train()
```

`run_colmap` calls `feature_extractor`, `exhaustive_matcher`, and `mapper`.
`train` delegates to the official 3DGS `train.py` script and returns the final
trained `point_cloud.ply`.

### 3. Estimate Geographic Alignment

```python
from geo_alignment.procrustes import align_pipeline

result = align_pipeline(
    transforms_json_path="captures/site_a/transforms.json",
    colmap_images_bin_path="outputs/colmap/site_a/images.bin",
    output_dir="outputs/alignment/site_a",
)

print(result.report())
```

The alignment step reads COLMAP camera centers, pairs them with GPS-derived ENU
coordinates from `transforms.json`, estimates a uniform-scale similarity
transform, reports residuals, and saves `alignment.npz` when `output_dir` is
provided.

### 4. Build and Export the Navigation Mesh

```python
import json
from pathlib import Path

from geo_alignment.nav_mesh import NavMesh
from geo_alignment.osm_loader import download_osm_data

osm_data = download_osm_data(
    center_lat=-27.4975,
    center_lon=153.0137,
    radius_m=500,
)

nav_mesh = NavMesh.from_osm_data(
    osm_data,
    road_buffer_m=3.0,
    sidewalk_buffer_m=2.0,
    collision_margin_m=0.3,
)

output_path = Path("../gs-vr-nav-unity/Assets/StreamingAssets/nav_mesh.json")
output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.write_text(json.dumps(nav_mesh.to_json(), indent=2), encoding="utf-8")
```

The Unity loader expects a GeoJSON-like `MultiPolygon` structure with
`walkable_area`, `obstacle_area`, and `bounds` fields.

### 5. Apply the Alignment to the Splat PLY

```python
from pathlib import Path

import numpy as np

from reconstruction.export import apply_transform, load_ply, save_ply

alignment = np.load("outputs/alignment/site_a/alignment.npz")
splats = load_ply("outputs/3dgs/site_a/point_cloud/iteration_30000/point_cloud.ply")
aligned = apply_transform(splats, alignment["transform"])

save_ply(
    aligned,
    Path("../gs-vr-nav-unity/Assets/StreamingAssets/SplatScenes/aligned_scene.ply"),
)
```

This produces the aligned splat scene consumed by the Unity demo.

## Unity Demo

The Unity demo project lives in `../gs-vr-nav-unity`. It has already been set up
as a URP/XRI project with a generated demo scene:

- `Assets/Scenes/GS_VR_Nav_Demo.unity`
- Editor menu: `GS-VR-Nav > Create Simulator Demo Scene`

Before running the scene, place the exported assets here:

- `../gs-vr-nav-unity/Assets/StreamingAssets/nav_mesh.json`
- `../gs-vr-nav-unity/Assets/StreamingAssets/SplatScenes/aligned_scene.ply`

Runtime responsibilities are split across the Unity scripts:

- `GeoAlignmentLoader` loads `nav_mesh.json`, parses polygons, and answers
  walkability and movement-clamping queries.
- `VRNavigationController` reads XR or keyboard movement input, moves the XR
  origin continuously, and constrains movement to the loaded navigation mesh.
- `GaussianSplatSetup` configures the UnityGaussianSplatting renderer and
  applies the ENU-to-Unity orientation convention.
- `MinimapController` renders a top-down minimap of the navigation mesh and
  player position.

Coordinate convention:

- Python ENU: `X = East`, `Y = North`, `Z = Up`
- Unity: `X = Right`, `Y = Up`, `Z = Forward`
- Mapping: `ENU(e, n, u) -> Unity(e, u, n)`

## Testing

Run the Python test suite from this directory:

```bash
python -m pytest tests
```

Slow tests that require network access or long-running external services are
marked with `slow`. Enable them explicitly when the environment supports OSM
downloads and external tooling:

```bash
python -m pytest tests --run-slow
```

The current local baseline is 38 passed and 2 skipped. A pytest cache permission
warning may appear in this workspace; it does not indicate a failure in the
project tests.

## Current Status and Known Gaps

Implemented and covered by tests:

- EXIF GPS extraction and `transforms.json` generation.
- WGS84, ECEF, and local ENU coordinate conversion.
- COLMAP image metadata parsing and Umeyama similarity alignment.
- OSM geometry loading, serialization, and navigation-mesh generation.
- 3DGS PLY load/save, splat transformation, bounding boxes, and subsampling.
- Synthetic integration tests for alignment, nav-mesh serialization, and PLY
  transformation.

Still incomplete:

- `navigation/locomotion.py` is still a Python-side locomotion placeholder.
  Runtime movement is currently implemented in Unity.
- `utils/visualization.py` contains plotting placeholders; detailed plotting is
  implemented in specific modules such as `geo_alignment.nav_mesh` and
  `geo_alignment.procrustes`.
- Real data runs depend on external COLMAP, the official 3DGS repository, OSM
  network access, and Unity runtime assets that are not committed here.
