# GS VR Navigation Project

This repository contains a Gaussian Splatting based VR navigation project, with a Python processing pipeline and a Unity VR viewer.

## Repository Structure

- `gs-vr-nav/`: Python source code, configs, utilities, and tests for data processing, reconstruction helpers, geospatial alignment, and navigation logic.
- `gs-vr-nav-unity/`: Unity project for viewing and navigating Gaussian Splatting scenes in VR.
- `gsproject.txt`: Project notes and high-level project information.

## External Dependency

This project uses `gaussian-splatting` as an external dependency:

https://github.com/graphdeco-inria/gaussian-splatting

The external dependency is not included in this repository. After cloning this project, clone it manually into:

```text
external/gaussian-splatting
```

Example:

```powershell
git clone https://github.com/graphdeco-inria/gaussian-splatting.git external/gaussian-splatting
```

## Python Setup

```powershell
cd gs-vr-nav
pip install -r requirements.txt
```

## Unity Setup

Open `gs-vr-nav-unity/` with Unity. Unity will regenerate local folders such as `Library/`, `Logs/`, and `UserSettings/`; these folders are intentionally excluded from Git.

Required runtime assets should be placed under:

```text
gs-vr-nav-unity/Assets/StreamingAssets/
```

