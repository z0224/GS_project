// 编辑器内小地图，显示玩家位置与导航网格。
// 按 M 键切换显示/隐藏（编辑器友好，不依赖 VR 控制器按钮）。
//
// Unity 项目配置要点：
// - Project Settings > XR Plug-in Management 中不要勾选 Oculus 或 OpenXR；MVP 只使用 XR Device Simulator。
// - Project Settings > Player > Active Input Handling 设为 "Both" 或 "Input System Package (New)"。
// - 构建平台保持默认 Standalone (PC)，不需要切换到 Android。
//
// 坐标系映射表（小地图渲染 Unity XZ 平面，对应 Python ENU East/North）：
// ┌──────────────────────────────────────────────────────┐
// │ Python ENU: X = East, Y = North, Z = Up（右手系）     │
// │ Unity:      X = Right, Y = Up,    Z = Forward（左手系）│
// ├────────────┬──────────┬──────────┬──────────────────┤
// │            │  East    │  North   │  Up              │
// ├────────────┼──────────┼──────────┼──────────────────┤
// │ ENU        │  X (+)   │  Y (+)   │  Z (+)           │
// │ Unity      │  X (+)   │  Z (+)   │  Y (+)           │
// ├────────────┴──────────┴──────────┴──────────────────┤
// │ ENU(e, n, u) → Unity(e, u, n)                        │
// │ Unity(x, y, z) = (ENU.east, ENU.up, ENU.north)        │
// └──────────────────────────────────────────────────────┘

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace GsVrNav.Unity
{
    /// <summary>
    /// Runtime minimap that renders the navigation mesh from an orthographic top-down camera.
    /// </summary>
    public sealed class MinimapController : MonoBehaviour
    {
        [SerializeField]
        private int mapSize = 256;

        [SerializeField]
        private float viewRange = 100f;

        [SerializeField]
        private float minViewRange = 50f;

        [SerializeField]
        private float maxViewRange = 500f;

        [SerializeField]
        private RawImage minimapImage;

        [SerializeField]
        private GeoAlignmentLoader geoLoader;

        [SerializeField]
        private Transform playerTransform;

        [SerializeField]
        private KeyCode toggleKey = KeyCode.M;

        private const string MinimapLayerName = "Minimap";
        private const float CameraHeight = 50f;
        private const float GeometryY = 0.05f;

        private Camera minimapCamera;
        private RenderTexture renderTexture;
        private GameObject minimapGeometryRoot;
        private GameObject playerArrow;
        private int minimapLayer;
        private bool isVisible = true;

        private void Awake()
        {
            if (geoLoader == null)
            {
#if UNITY_2023_1_OR_NEWER
                geoLoader = FindFirstObjectByType<GeoAlignmentLoader>();
#else
                geoLoader = FindObjectOfType<GeoAlignmentLoader>();
#endif
            }

            if (playerTransform == null)
            {
                Camera mainCamera = Camera.main;
                playerTransform = mainCamera != null ? mainCamera.transform : transform;
            }

            minimapLayer = LayerMask.NameToLayer(MinimapLayerName);
            if (minimapLayer < 0)
            {
                minimapLayer = 0;
                Debug.LogWarning("Layer 'Minimap' was not found. Minimap geometry will use Default layer; create a Minimap layer for cleaner camera culling.");
            }

            CreateRenderTexture();
            CreateCamera();
            CreateUiIfNeeded();
            BuildMinimapGeometry();
        }

        private void Start()
        {
            // GeoAlignmentLoader 也会在 Awake 自动加载；这里再重建一次，规避 Unity 组件 Awake 顺序不同导致的小地图空白。
            BuildMinimapGeometry();
        }

        private void OnDestroy()
        {
            if (renderTexture != null)
            {
                renderTexture.Release();
                Destroy(renderTexture);
            }
        }

        private void Update()
        {
            if (IsTogglePressed())
            {
                isVisible = !isVisible;
                if (minimapImage != null)
                {
                    minimapImage.enabled = isVisible;
                }

                if (minimapCamera != null)
                {
                    minimapCamera.enabled = isVisible;
                }
            }

            UpdateZoom();
            UpdateCameraFollow();
            UpdatePlayerArrow();
        }

        /// <summary>
        /// Rebuilds all minimap line and fill geometry from the current GeoAlignmentLoader data.
        /// </summary>
        public void BuildMinimapGeometry()
        {
            if (minimapGeometryRoot != null)
            {
                Destroy(minimapGeometryRoot);
            }

            minimapGeometryRoot = new GameObject("MinimapGeometry");
            minimapGeometryRoot.layer = minimapLayer;
            minimapGeometryRoot.transform.SetParent(transform, false);

            if (geoLoader == null)
            {
                return;
            }

            Material greenLineMaterial = CreateLineMaterial(new Color(0f, 1f, 0f, 1f));
            Material redFillMaterial = CreateFillMaterial(new Color(1f, 0f, 0f, 0.65f));

            foreach (GeoAlignmentLoader.Polygon2D polygon in geoLoader.WalkablePolygons)
            {
                CreateRingLine("WalkableBoundary", polygon.outerRing, greenLineMaterial, 0.08f);
                if (polygon.holes == null)
                {
                    continue;
                }

                foreach (Vector2[] hole in polygon.holes)
                {
                    CreateRingLine("WalkableHole", hole, greenLineMaterial, 0.06f);
                }
            }

            int obstacleIndex = 0;
            foreach (GeoAlignmentLoader.Polygon2D polygon in geoLoader.ObstaclePolygons)
            {
                CreateObstacleFill($"ObstacleFill_{obstacleIndex}", polygon.outerRing, redFillMaterial);
                obstacleIndex++;
            }

            playerArrow = CreatePlayerArrow();
        }

        private void CreateRenderTexture()
        {
            int safeSize = Mathf.Max(64, mapSize);
            renderTexture = new RenderTexture(safeSize, safeSize, 16, RenderTextureFormat.ARGB32)
            {
                name = "MinimapRenderTexture"
            };
            renderTexture.Create();
        }

        private void CreateCamera()
        {
            GameObject cameraObject = new GameObject("MinimapCamera");
            cameraObject.transform.SetParent(transform, false);
            cameraObject.layer = minimapLayer;

            minimapCamera = cameraObject.AddComponent<Camera>();
            minimapCamera.orthographic = true;
            minimapCamera.orthographicSize = viewRange * 0.5f;
            minimapCamera.clearFlags = CameraClearFlags.SolidColor;
            minimapCamera.backgroundColor = new Color(0f, 0f, 0f, 0.35f);
            minimapCamera.cullingMask = 1 << minimapLayer;
            minimapCamera.targetTexture = renderTexture;
            minimapCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
        }

        private void CreateUiIfNeeded()
        {
            if (minimapImage == null)
            {
                GameObject canvasObject = new GameObject("MinimapCanvas");
                Canvas canvas = canvasObject.AddComponent<Canvas>();
                canvas.renderMode = RenderMode.ScreenSpaceOverlay;
                canvas.sortingOrder = 100;
                canvasObject.AddComponent<CanvasScaler>();
                canvasObject.AddComponent<GraphicRaycaster>();

                GameObject imageObject = new GameObject("MinimapImage");
                imageObject.transform.SetParent(canvasObject.transform, false);
                minimapImage = imageObject.AddComponent<RawImage>();
            }

            minimapImage.texture = renderTexture;
            minimapImage.color = Color.white;

            RectTransform rectTransform = minimapImage.rectTransform;
            rectTransform.anchorMin = new Vector2(1f, 1f);
            rectTransform.anchorMax = new Vector2(1f, 1f);
            rectTransform.pivot = new Vector2(1f, 1f);
            rectTransform.anchoredPosition = new Vector2(-16f, -16f);
            rectTransform.sizeDelta = new Vector2(mapSize, mapSize);
        }

        private void CreateRingLine(string objectName, Vector2[] ring, Material material, float width)
        {
            if (ring == null || ring.Length < 2)
            {
                return;
            }

            GameObject lineObject = new GameObject(objectName);
            lineObject.layer = minimapLayer;
            lineObject.transform.SetParent(minimapGeometryRoot.transform, false);

            LineRenderer lineRenderer = lineObject.AddComponent<LineRenderer>();
            lineRenderer.useWorldSpace = true;
            lineRenderer.loop = true;
            lineRenderer.positionCount = ring.Length;
            lineRenderer.widthMultiplier = width;
            lineRenderer.material = material;
            lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lineRenderer.receiveShadows = false;

            for (int i = 0; i < ring.Length; i++)
            {
                // ENU (East, North) → Unity (x, z)，小地图从 y=50 向下看。
                lineRenderer.SetPosition(i, new Vector3(ring[i].x, GeometryY, ring[i].y));
            }
        }

        private void CreateObstacleFill(string objectName, Vector2[] ring, Material material)
        {
            Mesh mesh = CreateFlatMesh(ring, GeometryY + 0.01f, $"{objectName}_Mesh");
            if (mesh == null)
            {
                return;
            }

            GameObject fillObject = new GameObject(objectName);
            fillObject.layer = minimapLayer;
            fillObject.transform.SetParent(minimapGeometryRoot.transform, false);
            MeshFilter meshFilter = fillObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = fillObject.AddComponent<MeshRenderer>();
            meshFilter.sharedMesh = mesh;
            meshRenderer.sharedMaterial = material;
        }

        private GameObject CreatePlayerArrow()
        {
            GameObject arrowObject = new GameObject("PlayerArrow");
            arrowObject.layer = minimapLayer;
            arrowObject.transform.SetParent(minimapGeometryRoot.transform, false);

            Mesh mesh = new Mesh { name = "PlayerArrowMesh" };
            mesh.vertices = new[]
            {
                new Vector3(0f, GeometryY + 0.05f, 0.85f),
                new Vector3(-0.35f, GeometryY + 0.05f, -0.35f),
                new Vector3(0.35f, GeometryY + 0.05f, -0.35f)
            };
            mesh.triangles = new[] { 0, 1, 2 };
            mesh.RecalculateNormals();

            MeshFilter meshFilter = arrowObject.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = arrowObject.AddComponent<MeshRenderer>();
            meshFilter.sharedMesh = mesh;
            meshRenderer.sharedMaterial = CreateFillMaterial(Color.white);
            return arrowObject;
        }

        private void UpdateCameraFollow()
        {
            if (minimapCamera == null || playerTransform == null)
            {
                return;
            }

            Vector3 playerPosition = playerTransform.position;
            minimapCamera.transform.position = new Vector3(playerPosition.x, CameraHeight, playerPosition.z);
            minimapCamera.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
            minimapCamera.orthographicSize = viewRange * 0.5f;
        }

        private void UpdatePlayerArrow()
        {
            if (playerArrow == null || playerTransform == null)
            {
                return;
            }

            Vector3 playerPosition = playerTransform.position;
            playerArrow.transform.position = new Vector3(playerPosition.x, 0f, playerPosition.z);

            Vector3 forward = Vector3.ProjectOnPlane(playerTransform.forward, Vector3.up);
            if (forward.sqrMagnitude < 0.001f)
            {
                forward = Vector3.forward;
            }

            playerArrow.transform.rotation = Quaternion.LookRotation(forward.normalized, Vector3.up);
        }

        private bool IsTogglePressed()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard != null && toggleKey == KeyCode.M)
            {
                return keyboard.mKey.wasPressedThisFrame;
            }

            try
            {
                return Input.GetKeyDown(toggleKey);
            }
            catch (InvalidOperationException)
            {
                return false;
            }
        }

        private void UpdateZoom()
        {
            float scroll = 0f;
            Mouse mouse = Mouse.current;
            if (mouse != null)
            {
                scroll = mouse.scroll.ReadValue().y;
            }
            else
            {
                try
                {
                    scroll = Input.mouseScrollDelta.y;
                }
                catch (InvalidOperationException)
                {
                    scroll = 0f;
                }
            }

            if (Mathf.Abs(scroll) > 0.01f)
            {
                viewRange = Mathf.Clamp(viewRange - scroll * 0.1f, minViewRange, maxViewRange);
            }
        }

        private static Material CreateLineMaterial(Color color)
        {
            Shader shader = Shader.Find("Sprites/Default");
            Material material = new Material(shader)
            {
                name = "MinimapLineMaterial",
                color = color
            };
            return material;
        }

        private static Material CreateFillMaterial(Color color)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
            {
                shader = Shader.Find("Unlit/Color");
            }

            Material material = new Material(shader)
            {
                name = "MinimapFillMaterial",
                color = color
            };

            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", color);
            }

            material.SetOverrideTag("RenderType", "Transparent");
            material.renderQueue = 3000;
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            return material;
        }

        private static Mesh CreateFlatMesh(Vector2[] ring, float y, string meshName)
        {
            int[] triangles = TriangulateEarClipping(ring);
            if (triangles.Length < 3)
            {
                return null;
            }

            Vector3[] vertices = new Vector3[ring.Length];
            for (int i = 0; i < ring.Length; i++)
            {
                vertices[i] = new Vector3(ring[i].x, y, ring[i].y);
            }

            Mesh mesh = new Mesh { name = meshName };
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static int[] TriangulateEarClipping(Vector2[] ring)
        {
            if (ring == null || ring.Length < 3)
            {
                return Array.Empty<int>();
            }

            List<int> vertices = new List<int>();
            for (int i = 0; i < ring.Length; i++)
            {
                vertices.Add(i);
            }

            if (SignedArea(ring) < 0f)
            {
                vertices.Reverse();
            }

            List<int> triangles = new List<int>();
            int guard = 0;
            while (vertices.Count > 3 && guard < ring.Length * ring.Length)
            {
                bool earFound = false;
                for (int i = 0; i < vertices.Count; i++)
                {
                    int previousIndex = vertices[(i - 1 + vertices.Count) % vertices.Count];
                    int currentIndex = vertices[i];
                    int nextIndex = vertices[(i + 1) % vertices.Count];

                    if (!IsConvex(ring[previousIndex], ring[currentIndex], ring[nextIndex]))
                    {
                        continue;
                    }

                    bool containsPoint = false;
                    for (int j = 0; j < vertices.Count; j++)
                    {
                        int testIndex = vertices[j];
                        if (testIndex == previousIndex || testIndex == currentIndex || testIndex == nextIndex)
                        {
                            continue;
                        }

                        if (PointInTriangle(ring[testIndex], ring[previousIndex], ring[currentIndex], ring[nextIndex]))
                        {
                            containsPoint = true;
                            break;
                        }
                    }

                    if (containsPoint)
                    {
                        continue;
                    }

                    triangles.Add(previousIndex);
                    triangles.Add(currentIndex);
                    triangles.Add(nextIndex);
                    vertices.RemoveAt(i);
                    earFound = true;
                    break;
                }

                if (!earFound)
                {
                    break;
                }

                guard++;
            }

            if (vertices.Count == 3)
            {
                triangles.Add(vertices[0]);
                triangles.Add(vertices[1]);
                triangles.Add(vertices[2]);
            }

            return triangles.ToArray();
        }

        private static float SignedArea(Vector2[] ring)
        {
            float area = 0f;
            for (int i = 0; i < ring.Length; i++)
            {
                Vector2 a = ring[i];
                Vector2 b = ring[(i + 1) % ring.Length];
                area += a.x * b.y - b.x * a.y;
            }

            return area * 0.5f;
        }

        private static bool IsConvex(Vector2 previous, Vector2 current, Vector2 next)
        {
            Vector2 a = current - previous;
            Vector2 b = next - current;
            return a.x * b.y - a.y * b.x > 0f;
        }

        private static bool PointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            float area = Mathf.Abs(Cross(b - a, c - a));
            float area1 = Mathf.Abs(Cross(a - p, b - p));
            float area2 = Mathf.Abs(Cross(b - p, c - p));
            float area3 = Mathf.Abs(Cross(c - p, a - p));
            return Mathf.Abs(area - (area1 + area2 + area3)) <= 0.0001f;
        }

        private static float Cross(Vector2 a, Vector2 b)
        {
            return a.x * b.y - a.y * b.x;
        }
    }
}
