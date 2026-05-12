# GS-VR-Nav Unity Demo Project

This folder is a Unity URP/XRI demo project scaffold for the `gs-vr-nav` Unity scripts.

It has been imported and compiled successfully with Unity 6000.2.13f1, which satisfies the original "Unity 2022.3 LTS or newer" requirement. Unity Package Manager installs the packages declared in `Packages/manifest.json`.

Generated scene:

- `Assets/Scenes/GS_VR_Nav_Demo.unity`

Imported XRI samples:

- `Assets/Samples/XR Interaction Toolkit/3.2.2/Starter Assets`
- `Assets/Samples/XR Interaction Toolkit/3.2.2/XR Device Simulator`

The scene can be rebuilt from Unity with:

- Menu: `GS-VR-Nav > Create Simulator Demo Scene`

Required runtime assets are not present in this workspace yet:

- Place `nav_mesh.json` at `Assets/StreamingAssets/nav_mesh.json`
- Place `aligned_scene.ply` at `Assets/StreamingAssets/SplatScenes/aligned_scene.ply`

Follow `SETUP_SIMULATOR.md` for the scene setup, XR Device Simulator controls, and common troubleshooting.
