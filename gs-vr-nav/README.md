# GS-VR-Nav

Geographic-aligned Gaussian Splatting for continuous VR navigation.

The current Unity workflow is Blosm-only for map geometry:

- Gaussian Splat rendering uses `aligned_scene.ply`.
- Unity can still load `refined_scene.ply`, but the recommended map-to-scene workflow keeps `aligned_scene.ply` as the visual target and moves Blosm navigation geometry with `map_alignment.json`.
- Blosm generates `Assets/External/BlosmMap/Blosm_Map.fbx`.
- Unity uses the Blosm FBX as the only map geometry, walkable surface source, and physical collision source.
- `transforms.json`, `osm_data.json`, Blosm export config, `alignment.npz`, and `map_alignment.json` must all agree on the canonical WGS84 origin from `transforms.json`.

## Quick Run: Site A Demo

Run these commands from PowerShell. Use Python 3.10 or 3.11 with the project dependencies installed.

```powershell
Set-Location D:\desktop\gs_nv\gs-vr-nav
conda create -n gs-vr-nav python=3.11 -y
conda activate gs-vr-nav
pip install -r requirements.txt
```

Regenerate OSM with the canonical origin from `captures/site_a/transforms.json`:

```powershell
python -m geo_alignment.generate_osm_data `
  --transforms-json ..\captures\site_a\transforms.json `
  --output ..\outputs\osm\site_a\osm_data.json `
  --radius-m 250
```

Regenerate the Procrustes alignment and residual report:

```powershell
python -m geo_alignment.procrustes `
  ..\captures\site_a\transforms.json `
  ..\captures\site_a\distorted\sparse\0\images.bin `
  --output-dir ..\outputs\alignment\site_a
```

Regenerate Blosm collision/map geometry. Adjust `--blender-exe` if Blender is installed elsewhere:

```powershell
python -m geo_alignment.generate_blosm_map `
  --transforms-json ..\captures\site_a\transforms.json `
  --output ..\gs-vr-nav-unity\Assets\External\BlosmMap\Blosm_Map.fbx `
  --blender-exe "C:\Program Files\Blender Foundation\Blender 4.2\blender.exe"
```

Open the Unity project:

```text
D:\desktop\gs_nv\gs-vr-nav-unity
```

Then open:

```text
Assets/Scenes/GS_VR_Nav_Demo.unity
```

Press Play. The demo controller is desktop-first:

- Mouse: look around.
- `W/A/S/D`: move forward/left/back/right.
- Height and movement bounds come from hidden Blosm road/path/sidewalk colliders.
- Use `Toggle Blosm Map Geometry` on `NavMeshManager/GeoAlignmentLoader` only for debugging.

Do the first Unity preview without ICP. Only run optional map-to-scene ICP if Blosm and the splat still have a small residual offset after the canonical-origin rebuild.

## Canonical Origin Workflow

Use the top-level `origin_lat`, `origin_lon`, and `origin_alt` in `captures/site_a/transforms.json` as the canonical origin.

- OSM still queries Overpass around the trajectory center, but converts OSM nodes to ENU with the canonical origin.
- Blosm uses the canonical origin as its import center.
- `osm_data.json` records both `origin_*` and `query_center_*` so a center/origin mismatch is visible.
- Regenerate OSM and Blosm before comparing Unity alignment if the canonical origin changes.

## Align Blosm Navigation Geometry With Open3D ICP

After generating the GPS-aligned splat and OSM JSON, compute a Unity map alignment:

```powershell
Set-Location D:\desktop\gs_nv\gs-vr-nav
python -m geo_alignment.icp_refinement `
  --mode map-to-scene `
  --input-ply ..\gs-vr-nav-unity\Assets\StreamingAssets\SplatScenes\aligned_scene.ply `
  --osm-json ..\outputs\osm\site_a\osm_data.json `
  --output-map-alignment ..\gs-vr-nav-unity\Assets\StreamingAssets\Alignment\map_alignment.json `
  --unity-y-offset -1
```

Unity reads `Assets/StreamingAssets/Alignment/map_alignment.json` and applies it to `NavMeshManager/Blosm_MapGeometry`. Keep `NavMeshManager` itself at identity.

Map-to-scene ICP is optional and gated. Unity rejects `map_alignment.json` unless it is accepted, uses `alignment_mode: "map-to-scene"`, includes `origin_wgs84`, and matches any configured expected hashes.

## Refine 3DGS Alignment With Open3D ICP

After generating the GPS-aligned splat, run an optional building-footprint ICP refinement:

```powershell
Set-Location D:\desktop\gs_nv\gs-vr-nav
python -m geo_alignment.icp_refinement `
  --mode refined-scene `
  --input-ply ..\gs-vr-nav-unity\Assets\StreamingAssets\SplatScenes\aligned_scene.ply `
  --osm-json ..\outputs\osm\site_a\osm_data.json `
  --output-ply ..\gs-vr-nav-unity\Assets\StreamingAssets\SplatScenes\refined_scene.ply `
  --output-transform ..\outputs\alignment\site_a\icp_refinement.npz
```

The refinement estimates only horizontal yaw plus east/north translation. It does not change scale, pitch, roll, or height.

Use either `refined_scene.ply` or `map_alignment.json` as the active correction owner. Do not stack both corrections unless the map alignment was generated against the exact splat file Unity loads.

Recommended correction owner: keep `aligned_scene.ply` unchanged and move only `Blosm_MapGeometry` with accepted `map_alignment.json`. Do not load `refined_scene.ply` while also applying a map alignment generated for a different splat.

## End-to-End Alignment Checklist

1. Confirm `captures/site_a/transforms.json` has the canonical `origin_lat/origin_lon/origin_alt`.
2. Regenerate `osm_data.json`; verify `origin_*` matches transforms and `query_center_*` is the trajectory center.
3. Regenerate `Blosm_Map.fbx`; verify Blosm config uses the canonical origin.
4. Run Procrustes and inspect `alignment_report.json/csv`.
5. Preview Unity without ICP and estimate the gross splat-vs-map offset.
6. Run optional map-to-scene ICP only if needed; use the generated JSON only when `accepted` is true.
7. In Unity final, record RMSE, fitness, yaw, and translation before presenting the demo.

## Generate Blosm Map Geometry

From Unity, select `NavMeshManager`, open the `GeoAlignmentLoader` component, set `Blender Executable Path` if `blender` is not on `PATH`, then use:

```text
Generate Blosm Map Asset
```

The expected output is:

```text
../gs-vr-nav-unity/Assets/External/BlosmMap/Blosm_Map.fbx
```

You can also run the Python command directly:

```bash
python -m geo_alignment.generate_blosm_map \
  --transforms-json transforms.json \
  --output ../gs-vr-nav-unity/Assets/External/BlosmMap/Blosm_Map.fbx \
  --blender-exe /path/to/blender
```

## Unity Runtime

`GeoAlignmentLoader` instantiates `Blosm_Map.fbx` under:

```text
NavMeshManager/Blosm_MapGeometry
```

Meshes whose hierarchy names contain `road`, `path`, `footway`, `pedestrian`, `sidewalk`, or `street` are registered as walkable surfaces. Meshes whose hierarchy names contain `building` are registered as building collision surfaces.

The runtime no longer loads or generates a separate JSON navigation mesh. If `Blosm_Map.fbx` is missing, Unity logs an error and movement remains disabled until the Blosm asset is available.
