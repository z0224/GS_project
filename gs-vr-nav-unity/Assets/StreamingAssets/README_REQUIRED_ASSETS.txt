Required GS-VR-Nav demo assets:

1. aligned_scene.ply
   Place at:
   Assets/StreamingAssets/SplatScenes/aligned_scene.ply

2. Blosm_Map.fbx
   Generate through NavMeshManager/GeoAlignmentLoader > Generate Blosm Map Asset.
   Expected Unity asset path:
   Assets/External/BlosmMap/Blosm_Map.fbx

Coordinate convention:
Python ENU: X = East, Y = North, Z = Up
Unity: X = Right, Y = Up, Z = Forward
Mapping: ENU(e, n, u) -> Unity(e, u, n)
