# GS-VR-Nav

Geographic-aligned Gaussian Splatting for continuous VR navigation.

The current Unity workflow is Blosm-only for map geometry:

- Gaussian Splat rendering uses `aligned_scene.ply`.
- Unity can still load `refined_scene.ply`, but the recommended map-to-scene workflow keeps `aligned_scene.ply` as the visual target and moves Blosm navigation geometry with `map_alignment.json`.
- Blosm generates `Assets/External/BlosmMap/Blosm_Map.fbx`.
- Unity uses the Blosm FBX as the only map geometry, walkable surface source, and physical collision source.

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
