# GS-VR-Nav: Unity 编辑器模拟器设置指南

## 前置条件

- Unity 2022.3 LTS 或更新版本
- 3D URP（Universal Render Pipeline）模板项目

## 第一步：安装必需包

打开 Window > Package Manager，安装以下包：

| 包名                           | 版本    | 用途                           |
|--------------------------------|---------|--------------------------------|
| XR Interaction Toolkit         | ≥ 2.5  | VR 交互框架 + XR Device Simulator |
| XR Plugin Management           | ≥ 4.4  | XR 子系统管理                  |
| Input System                   | ≥ 1.7  | 新输入系统（XRI 依赖）        |
| Newtonsoft.Json                | ≥ 3.2  | JSON 解析（nav_mesh.json）     |

**注意**：不需要安装 Oculus XR Plugin 或 OpenXR Plugin。
MVP 阶段仅使用 XR Device Simulator，不需要任何硬件 XR 运行时。

安装 XR Interaction Toolkit 时，Unity 会提示导入 Starter Assets，
请选择导入——这包含了 XR Device Simulator 预制件和默认输入配置。

## 第二步：配置 XR Device Simulator

1. 在场景中添加 `XR Device Simulator` 预制件：
   - 路径：Packages/XR Interaction Toolkit/Runtime/
     XR Device Simulator/XR Device Simulator.prefab
   - 或搜索 Project 面板：`t:Prefab XR Device Simulator`
   - 拖入场景 Hierarchy 中。

2. XR Device Simulator 的默认控制方式：

   | 操作             | 键位                              |
   |------------------|-----------------------------------|
   | 头显旋转         | 鼠标右键 + 拖动                   |
   | 头显位置移动     | 鼠标右键 + WASD                   |
   | 左手控制器激活   | 按住 Left Shift                   |
   | 左手摇杆         | Left Shift + WASD                 |
   | 右手控制器激活   | 按住 Space                        |
   | 右手摇杆         | Space + WASD                      |
   | Grip 按键        | 鼠标左键（控制器激活时）          |
   | Trigger 按键     | 鼠标右键（控制器激活时）          |

   对于本项目的移动操作：
   - 按住 Left Shift，然后按 WASD 即可模拟左摇杆输入 → 触发步行移动。
   - 移动鼠标（不按 Shift）即可旋转头显视角。

## 第三步：场景设置

1. 删除默认 Main Camera。

2. 添加 XR Origin：
   - 右键 Hierarchy > XR > XR Origin (Action-based)
   - 这会创建 XR Origin + Camera Offset + Main Camera + 左右手 Controller

3. 在 XR Origin 上添加 `VRNavigationController` 组件：
   - 将 `moveInput` 指向 XRI Default Input Actions 中的
     `XRI LeftHand Locomotion/Move` Action
   - 将 `headCamera` 指向 XR Origin 下的 Main Camera
   - 将 `xrOrigin` 指向 XR Origin 自身的 Transform

4. 创建空 GameObject `NavMeshManager`：
   - 添加 `GeoAlignmentLoader` 组件
   - 设置 navMeshJsonPath 为 `nav_mesh.json`（将在 StreamingAssets 中查找）

5. 创建空 GameObject `GaussianSplatScene`：
   - 添加 GaussianSplatRenderer 组件（来自 UnityGaussianSplatting 插件）
   - 添加 `GaussianSplatSetup` 组件
   - 配置 .ply 文件路径

6. 将 `VRNavigationController` 中的 `geoAlignmentLoader` 引用
   拖拽指向 `NavMeshManager`。

## 第四步：导入资源

将以下文件放入 `Assets/StreamingAssets/`：

- `aligned_scene.ply` — 对齐后的 Gaussian Splat 点云
- `nav_mesh.json` — Python 管线导出的导航网格

## 第五步：导入 Gaussian Splatting 插件

1. 克隆 https://github.com/aras-p/UnityGaussianSplatting
2. 将 `package/` 文件夹复制到 Unity 项目的 `Packages/` 目录下
   并重命名为 `com.aras.gaussian-splatting/`
3. 或通过 Package Manager > Add package from disk > 选择 package.json

## 第六步：运行

1. 确保 Scene 视图和 Game 视图都打开。
2. 点击 Play。
3. 操作方式：
   - 鼠标移动 = 转动头显视角
   - 按住 Left Shift + WASD = 步行移动
   - M 键 = 切换小地图
   - 观察 Game 视图左上角的调试信息面板
   - 在 Scene 视图中可以看到导航网格的 Gizmo 可视化

## 常见问题

Q: 按 WASD 没有移动？
A: 确保按住了 Left Shift（激活左手控制器模拟），然后再按 WASD。
   或者检查 VRNavigationController 的 moveInput 是否正确绑定。

Q: 看不到 Gaussian Splat？
A: 检查 GaussianSplatRenderer 组件是否正确加载了 .ply 文件。
   确认 GaussianSplatSetup 的 applyENUToUnityRotation 设置正确。

Q: 导航网格边界不可见？
A: 确保 GeoAlignmentLoader 的 drawWalkableArea 和 drawObstacles 勾选。
   Gizmo 仅在 Scene 视图可见。要在 Game 视图中看到，
   使用 GeoAlignmentLoader 的右键菜单 "Generate Debug Walkable Mesh"。

Q: 性能很差 / FPS < 30？
A: 减少 GaussianSplatSetup 中的 maxSplatCount。
   关闭 Scene 视图（它会消耗额外的渲染性能）。
   确保 URP 质量设置中关闭了不必要的后处理。
