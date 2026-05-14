// Gaussian Splatting 场景设置。
// 配合 https://github.com/aras-p/UnityGaussianSplatting 使用。
// MVP：仅在 Unity 编辑器中运行，不做真机优化。
//
// 使用步骤：
// 1. 克隆 https://github.com/aras-p/UnityGaussianSplatting 到 Unity 项目的 Packages/ 或 Assets/ 中。
// 2. 将对齐后的 .ply 文件放入 Assets/StreamingAssets/SplatScenes/。
// 3. 在场景中创建空 GameObject，添加 GaussianSplatRenderer 组件（来自插件）。
// 4. 再添加此 GaussianSplatSetup 组件。
// 5. 在 Inspector 中配置参数。
// 6. 直接按 Play 在编辑器中运行。
//
// Unity 项目配置要点：
// - Project Settings > XR Plug-in Management 中不要勾选 Oculus 或 OpenXR；MVP 只使用 XR Device Simulator。
// - Project Settings > Player > Active Input Handling 设为 "Both" 或 "Input System Package (New)"。
// - 构建平台保持默认 Standalone (PC)，不需要切换到 Android。
//
// 坐标系映射参考：
// ┌──────────────────────────────────────────────────────┐
// │           坐标系映射参考                              │
// ├────────────┬──────────┬──────────┬──────────────────┤
// │            │  East    │  North   │  Up              │
// ├────────────┼──────────┼──────────┼──────────────────┤
// │ ENU        │  X (+)   │  Y (+)   │  Z (+)           │
// │ Unity      │  X (+)   │  Z (+)   │  Y (+)           │
// ├────────────┴──────────┴──────────┴──────────────────┤
// │ 变换：ENU → Unity                                    │
// │   unity.x = enu.east                                 │
// │   unity.y = enu.up                                   │
// │   unity.z = enu.north                                │
// │                                                      │
// │ 等价：对 ENU 点云绕 X 轴旋转 -90°                    │
// └──────────────────────────────────────────────────────┘

using System;
using System.IO;
using System.Reflection;
using UnityEngine;

namespace GsVrNav.Unity
{
    /// <summary>
    /// Editor-focused helper that applies ENU-to-Unity orientation and displays Gaussian Splat runtime stats.
    /// </summary>
    public sealed class GaussianSplatSetup : MonoBehaviour
    {
        [Header("Splat Configuration")]
        [SerializeField]
        private string splatFileName = "aligned_scene.ply";

        [SerializeField]
        private string fallbackSplatFileName = "refined_scene.ply";

        [SerializeField]
        private int maxSplatCount = 5_000_000;

        [SerializeField]
        private int shDegree = 3;

        [Header("Coordinate System")]
        [Tooltip("ENU(e,n,u) → Unity(x,y,z) = (e,u,n)。如果 .ply 已在 ENU 坐标系下，" +
                 "需要绕 X 轴旋转 -90° 将 ENU.Up(Z) 映射到 Unity.Up(Y)。" +
                 "如果 Python 导出时已做了 (e,n,u)→(e,u,n) 变换，则关闭此项。")]
        [SerializeField]
        private bool applyENUToUnityRotation = true;

        [Header("Editor Performance")]
        [SerializeField]
        private bool showPerformanceStats = true;

        private float fps;
        private float fpsAccumulator;
        private int fpsFrames;
        private float fpsTimer;
        private Component gaussianSplatRenderer;

        private void Awake()
        {
            gaussianSplatRenderer = FindGaussianSplatRenderer();
        }

        private void Start()
        {
            if (applyENUToUnityRotation)
            {
                // ENU: X=East, Y=North, Z=Up → Unity: X=East, Y=Up, Z=North。
                // 绕 X 轴旋转 -90° 将 ENU.Up(Z) 映射到 Unity.Up(Y)，并让 ENU.North(Y) 落到 Unity.Forward(Z)。
                transform.localRotation = Quaternion.Euler(90f, 0f, 0f);
                transform.localScale = new Vector3(1f, 1f, -1f);
            }
            else
            {
                transform.localRotation = Quaternion.identity;
                transform.localScale = Vector3.one;
            }

            transform.localPosition = Vector3.zero;

            ConfigureRendererBestEffort();
        }

        private void Update()
        {
            float delta = Time.unscaledDeltaTime;
            if (delta <= 0f)
            {
                return;
            }

            fpsAccumulator += 1f / delta;
            fpsFrames++;
            fpsTimer += delta;

            if (fpsTimer >= 0.25f)
            {
                fps = fpsAccumulator / Mathf.Max(1, fpsFrames);
                fpsAccumulator = 0f;
                fpsFrames = 0;
                fpsTimer = 0f;
            }
        }

        private Component FindGaussianSplatRenderer()
        {
            Component[] components = GetComponents<Component>();
            foreach (Component component in components)
            {
                if (component == null)
                {
                    continue;
                }

                string typeName = component.GetType().Name;
                if (typeName.IndexOf("GaussianSplatRenderer", StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return component;
                }
            }

            return null;
        }

        private void ConfigureRendererBestEffort()
        {
            if (gaussianSplatRenderer == null)
            {
                Debug.LogWarning("GaussianSplatRenderer component was not found on this GameObject. Add the UnityGaussianSplatting renderer before running the scene.");
                return;
            }

            string resolvedSplatFileName = ResolveSplatFileName();
            string splatPath = Path.Combine(Application.streamingAssetsPath, "SplatScenes", resolvedSplatFileName);

            // 插件版本间字段名可能会变化；这里用反射做宽松配置，避免把 MVP 脚本绑定到某个内部 API。
            TrySetMember(gaussianSplatRenderer, "m_FileName", resolvedSplatFileName);
            TrySetMember(gaussianSplatRenderer, "m_FilePath", splatPath);
            TrySetMember(gaussianSplatRenderer, "splatFileName", resolvedSplatFileName);
            TrySetMember(gaussianSplatRenderer, "splatFilePath", splatPath);
            TrySetMember(gaussianSplatRenderer, "maxSplatCount", maxSplatCount);
            TrySetMember(gaussianSplatRenderer, "m_MaxSplatCount", maxSplatCount);
            TrySetMember(gaussianSplatRenderer, "shDegree", shDegree);
            TrySetMember(gaussianSplatRenderer, "m_SHDegree", shDegree);
        }

        private string ResolveSplatFileName()
        {
            string primaryPath = Path.Combine(Application.streamingAssetsPath, "SplatScenes", splatFileName);
            if (File.Exists(primaryPath))
            {
                return splatFileName;
            }

            string fallbackPath = Path.Combine(Application.streamingAssetsPath, "SplatScenes", fallbackSplatFileName);
            if (File.Exists(fallbackPath))
            {
                Debug.LogWarning($"Splat file '{splatFileName}' was not found; falling back to '{fallbackSplatFileName}'.");
                return fallbackSplatFileName;
            }

            Debug.LogWarning($"Neither splat file '{splatFileName}' nor fallback '{fallbackSplatFileName}' was found in StreamingAssets/SplatScenes.");
            return splatFileName;
        }

        private static void TrySetMember(Component target, string memberName, object value)
        {
            Type type = target.GetType();
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

            FieldInfo field = type.GetField(memberName, flags);
            if (field != null && field.FieldType.IsAssignableFrom(value.GetType()))
            {
                field.SetValue(target, value);
                return;
            }

            PropertyInfo property = type.GetProperty(memberName, flags);
            if (property != null && property.CanWrite && property.PropertyType.IsAssignableFrom(value.GetType()))
            {
                property.SetValue(target, value, null);
            }
        }

        private int GetSplatCount()
        {
            if (gaussianSplatRenderer == null)
            {
                return maxSplatCount;
            }

            object value =
                TryGetMember(gaussianSplatRenderer, "splatCount") ??
                TryGetMember(gaussianSplatRenderer, "SplatCount") ??
                TryGetMember(gaussianSplatRenderer, "m_SplatCount");

            return value is int count ? count : maxSplatCount;
        }

        private static object TryGetMember(Component target, string memberName)
        {
            Type type = target.GetType();
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

            FieldInfo field = type.GetField(memberName, flags);
            if (field != null)
            {
                return field.GetValue(target);
            }

            PropertyInfo property = type.GetProperty(memberName, flags);
            return property != null && property.CanRead ? property.GetValue(target, null) : null;
        }

        private void OnGUI()
        {
#if UNITY_EDITOR
            if (!showPerformanceStats)
            {
                return;
            }

            Color textColor = fps < 30f ? Color.red : Color.white;
            GUIStyle style = new GUIStyle(GUI.skin.label)
            {
                fontSize = 16,
                normal = { textColor = textColor }
            };

            int drawCalls = GetEditorDrawCalls();
            string text =
                $"FPS: {Mathf.RoundToInt(fps)}\n" +
                $"Splat Count: {GetSplatCount():N0}\n" +
                $"Draw Calls: {drawCalls}";

            float height = 72f;
            GUI.Label(new Rect(12f, Screen.height - height - 12f, 280f, height), text, style);
#endif
        }

#if UNITY_EDITOR
        private static int GetEditorDrawCalls()
        {
            Type statsType = Type.GetType("UnityEditor.UnityStats, UnityEditor");
            if (statsType == null)
            {
                return 0;
            }

            PropertyInfo drawCallsProperty = statsType.GetProperty("drawCalls", BindingFlags.Static | BindingFlags.Public);
            if (drawCallsProperty == null)
            {
                drawCallsProperty = statsType.GetProperty("batches", BindingFlags.Static | BindingFlags.Public);
            }

            object value = drawCallsProperty != null ? drawCallsProperty.GetValue(null, null) : null;
            return value is int drawCalls ? drawCalls : 0;
        }
#endif
    }
}
