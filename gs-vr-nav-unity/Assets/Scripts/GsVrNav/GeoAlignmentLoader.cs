// 加载 Python 管线导出的 nav mesh JSON，提供高性能 2D 空间查询。
// 同时在 Scene 视图中提供可视化调试。
//
// Unity 项目配置要点：
// - Project Settings > XR Plug-in Management 中不要勾选 Oculus 或 OpenXR；MVP 只使用 XR Device Simulator。
// - Project Settings > Player > Active Input Handling 设为 "Both" 或 "Input System Package (New)"。
// - 构建平台保持默认 Standalone (PC)，不需要切换到 Android。
//
// 坐标系映射表（JSON 坐标为 ENU East/North，对应 Unity x/z）：
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
using System.IO;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace GsVrNav.Unity
{
    /// <summary>
    /// Loads a Python-exported ENU navigation mesh and answers Unity XZ-plane walkability queries.
    /// </summary>
    public sealed class GeoAlignmentLoader : MonoBehaviour
    {
        /// <summary>
        /// Serializable 2D polygon with an outer boundary and optional holes.
        /// </summary>
        [Serializable]
        public class Polygon2D
        {
            /// <summary>
            /// Exterior ring in ENU (East, North), mapped to Unity (x, z).
            /// </summary>
            public Vector2[] outerRing = Array.Empty<Vector2>();

            /// <summary>
            /// Interior rings in ENU (East, North), mapped to Unity (x, z).
            /// </summary>
            public Vector2[][] holes = Array.Empty<Vector2[]>();
        }

        [Header("NavMesh Source")]
        [SerializeField]
        private string navMeshJsonPath = "nav_mesh.json";

        [SerializeField]
        private float gridCellSize = 5.0f;

        [SerializeField]
        private bool loadOnAwake = true;

        [Header("Debug Visualization")]
        [SerializeField]
        private bool drawWalkableArea = true;

        [SerializeField]
        private bool drawObstacles = true;

        [SerializeField]
        private Color walkableColor = new Color(0, 1, 0, 0.3f);

        [SerializeField]
        private Color obstacleColor = new Color(1, 0, 0, 0.5f);

        private readonly List<Polygon2D> walkablePolygons = new List<Polygon2D>();
        private readonly List<Polygon2D> obstaclePolygons = new List<Polygon2D>();
        private Vector4 bounds; // (minE, minN, maxE, maxN)
        private Dictionary<(int, int), List<int>> gridIndex = new Dictionary<(int, int), List<int>>();
        private Dictionary<(int, int), List<int>> obstacleGridIndex = new Dictionary<(int, int), List<int>>();

        /// <summary>
        /// Gets the loaded walkable polygons in ENU East/North coordinates.
        /// </summary>
        public IReadOnlyList<Polygon2D> WalkablePolygons => walkablePolygons;

        /// <summary>
        /// Gets the loaded obstacle polygons in ENU East/North coordinates.
        /// </summary>
        public IReadOnlyList<Polygon2D> ObstaclePolygons => obstaclePolygons;

        /// <summary>
        /// Gets the navigation bounds as (minEast, minNorth, maxEast, maxNorth).
        /// </summary>
        public Vector4 Bounds => bounds;

        private void Awake()
        {
            if (!loadOnAwake)
            {
                return;
            }

            string resolvedPath = ResolveJsonPath(navMeshJsonPath);
            if (File.Exists(resolvedPath))
            {
                LoadNavMesh(resolvedPath);
            }
            else
            {
                Debug.LogWarning($"NavMesh JSON not found at {resolvedPath}. Use the context menu after placing nav_mesh.json in StreamingAssets.");
            }
        }

        /// <summary>
        /// Loads a nav mesh JSON file from an absolute path or from StreamingAssets.
        /// </summary>
        /// <param name="jsonPath">Absolute path or file name relative to Application.streamingAssetsPath.</param>
        public void LoadNavMesh(string jsonPath)
        {
            string resolvedPath = ResolveJsonPath(jsonPath);
            string json = File.ReadAllText(resolvedPath);
            JObject root = JObject.Parse(json);

            walkablePolygons.Clear();
            obstaclePolygons.Clear();

            walkablePolygons.AddRange(ParseMultiPolygon(root["walkable_area"]));
            obstaclePolygons.AddRange(ParseMultiPolygon(root["obstacle_area"]));
            bounds = ParseBounds(root["bounds"]);

            gridIndex = BuildGridIndex(walkablePolygons);
            obstacleGridIndex = BuildGridIndex(obstaclePolygons);

            Debug.Log($"NavMesh loaded: {walkablePolygons.Count} walkable polygons, {obstaclePolygons.Count} obstacles, bounds: {bounds}");
        }

        /// <summary>
        /// Returns true if a Unity XZ point is inside a walkable polygon and outside all obstacle polygons.
        /// </summary>
        /// <param name="x">Unity X coordinate, equivalent to ENU East.</param>
        /// <param name="z">Unity Z coordinate, equivalent to ENU North.</param>
        /// <returns>True when the point is navigable.</returns>
        public bool IsWalkable(float x, float z)
        {
            Vector2 point = new Vector2(x, z);
            if (!IsInsideBounds(point))
            {
                return false;
            }

            bool insideWalkable = false;
            foreach (int polygonIndex in QueryCandidatePolygons(point, gridIndex, walkablePolygons))
            {
                if (PointInPolygon(point, walkablePolygons[polygonIndex]))
                {
                    insideWalkable = true;
                    break;
                }
            }

            if (!insideWalkable)
            {
                return false;
            }

            foreach (int polygonIndex in QueryCandidatePolygons(point, obstacleGridIndex, obstaclePolygons))
            {
                if (PointInPolygon(point, obstaclePolygons[polygonIndex]))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Finds the nearest point on any walkable polygon boundary for a Unity XZ point.
        /// </summary>
        /// <param name="x">Unity X coordinate, equivalent to ENU East.</param>
        /// <param name="z">Unity Z coordinate, equivalent to ENU North.</param>
        /// <returns>The nearest Unity XZ point represented as (x, z).</returns>
        public Vector2 ClampToWalkable(float x, float z)
        {
            Vector2 point = new Vector2(x, z);
            if (IsWalkable(x, z))
            {
                return point;
            }

            float bestDistanceSq = float.PositiveInfinity;
            Vector2 bestPoint = point;

            foreach (Polygon2D polygon in walkablePolygons)
            {
                FindNearestPointOnRing(point, polygon.outerRing, ref bestPoint, ref bestDistanceSq);
                if (polygon.holes == null)
                {
                    continue;
                }

                foreach (Vector2[] hole in polygon.holes)
                {
                    FindNearestPointOnRing(point, hole, ref bestPoint, ref bestDistanceSq);
                }
            }

            return bestPoint;
        }

        /// <summary>
        /// Clamps a movement segment to the last walkable point along the attempted direction.
        /// </summary>
        /// <param name="from">Current Unity XZ position as (x, z).</param>
        /// <param name="to">Proposed Unity XZ position as (x, z).</param>
        /// <returns>The final allowed Unity XZ position as (x, z).</returns>
        public Vector2 ClampMovement(Vector2 from, Vector2 to)
        {
            if (IsWalkable(to.x, to.y))
            {
                return to;
            }

            if (!IsWalkable(from.x, from.y))
            {
                return ClampToWalkable(to.x, to.y);
            }

            Vector2 low = from;
            Vector2 high = to;

            // 二分搜索 from→to，保留最后一个仍处于可行走区域内的位置。
            for (int i = 0; i < 10; i++)
            {
                Vector2 mid = (low + high) * 0.5f;
                if (IsWalkable(mid.x, mid.y))
                {
                    low = mid;
                }
                else
                {
                    high = mid;
                }
            }

            return low;
        }

        /// <summary>
        /// Generates invisible MeshCollider objects from obstacle polygons.
        /// </summary>
        public void GenerateColliders()
        {
            const float colliderHeight = 10f;
            Transform existingRoot = transform.Find("GeneratedObstacleColliders");
            if (existingRoot != null)
            {
                DestroyUnityObject(existingRoot.gameObject);
            }

            GameObject root = new GameObject("GeneratedObstacleColliders");
            root.transform.SetParent(transform, false);

            for (int i = 0; i < obstaclePolygons.Count; i++)
            {
                Polygon2D polygon = obstaclePolygons[i];
                if (polygon.outerRing == null || polygon.outerRing.Length < 3)
                {
                    continue;
                }

                Mesh mesh = CreateExtrudedMesh(polygon.outerRing, colliderHeight, $"ObstacleCollider_{i}_Mesh");
                GameObject colliderObject = new GameObject($"ObstacleCollider_{i}");
                colliderObject.transform.SetParent(root.transform, false);

                MeshCollider meshCollider = colliderObject.AddComponent<MeshCollider>();
                meshCollider.sharedMesh = mesh;
            }
        }

        /// <summary>
        /// Loads nav_mesh.json from StreamingAssets using the Inspector path.
        /// </summary>
        [ContextMenu("Load NavMesh From StreamingAssets")]
        public void LoadNavMeshFromStreamingAssets()
        {
            LoadNavMesh(navMeshJsonPath);
        }

        /// <summary>
        /// Creates a translucent green runtime mesh for viewing the walkable area in the Game view.
        /// </summary>
        [ContextMenu("Generate Debug Walkable Mesh")]
        public void GenerateDebugWalkableMesh()
        {
            Transform existingRoot = transform.Find("DebugWalkableMesh");
            if (existingRoot != null)
            {
                DestroyUnityObject(existingRoot.gameObject);
            }

            GameObject root = new GameObject("DebugWalkableMesh");
            root.transform.SetParent(transform, false);

            Material material = CreateTransparentUnlitMaterial(walkableColor);

            for (int i = 0; i < walkablePolygons.Count; i++)
            {
                Polygon2D polygon = walkablePolygons[i];
                Mesh mesh = CreateFlatMesh(polygon.outerRing, 0.01f, $"WalkableDebug_{i}_Mesh");
                if (mesh == null)
                {
                    continue;
                }

                GameObject meshObject = new GameObject($"WalkableDebug_{i}");
                meshObject.transform.SetParent(root.transform, false);
                MeshFilter meshFilter = meshObject.AddComponent<MeshFilter>();
                MeshRenderer meshRenderer = meshObject.AddComponent<MeshRenderer>();
                meshFilter.sharedMesh = mesh;
                meshRenderer.sharedMaterial = material;
            }
        }

        private string ResolveJsonPath(string jsonPath)
        {
            if (string.IsNullOrWhiteSpace(jsonPath))
            {
                jsonPath = navMeshJsonPath;
            }

            return Path.IsPathRooted(jsonPath)
                ? jsonPath
                : Path.Combine(Application.streamingAssetsPath, jsonPath);
        }

        private static List<Polygon2D> ParseMultiPolygon(JToken token)
        {
            List<Polygon2D> polygons = new List<Polygon2D>();
            JToken coordinates = token?["coordinates"];
            if (coordinates == null)
            {
                return polygons;
            }

            foreach (JToken polygonToken in coordinates)
            {
                List<Vector2[]> rings = new List<Vector2[]>();
                foreach (JToken ringToken in polygonToken)
                {
                    Vector2[] ring = ParseRing(ringToken);
                    if (ring.Length >= 3)
                    {
                        rings.Add(ring);
                    }
                }

                if (rings.Count == 0)
                {
                    continue;
                }

                Polygon2D polygon = new Polygon2D
                {
                    outerRing = rings[0],
                    holes = rings.Count > 1 ? rings.GetRange(1, rings.Count - 1).ToArray() : Array.Empty<Vector2[]>()
                };
                polygons.Add(polygon);
            }

            return polygons;
        }

        private static Vector2[] ParseRing(JToken ringToken)
        {
            List<Vector2> points = new List<Vector2>();
            foreach (JToken pointToken in ringToken)
            {
                JArray pointArray = pointToken as JArray;
                if (pointArray != null && pointArray.Count >= 2)
                {
                    points.Add(new Vector2(pointArray[0].Value<float>(), pointArray[1].Value<float>()));
                }
            }

            return points.ToArray();
        }

        private static Vector4 ParseBounds(JToken token)
        {
            JArray array = token as JArray;
            if (array == null || array.Count < 4)
            {
                return Vector4.zero;
            }

            return new Vector4(array[0].Value<float>(), array[1].Value<float>(), array[2].Value<float>(), array[3].Value<float>());
        }

        private Dictionary<(int, int), List<int>> BuildGridIndex(IReadOnlyList<Polygon2D> polygons)
        {
            Dictionary<(int, int), List<int>> index = new Dictionary<(int, int), List<int>>();
            for (int i = 0; i < polygons.Count; i++)
            {
                if (!TryGetAabb(polygons[i], out Vector2 min, out Vector2 max))
                {
                    continue;
                }

                int minCellX = WorldToCell(min.x);
                int maxCellX = WorldToCell(max.x);
                int minCellY = WorldToCell(min.y);
                int maxCellY = WorldToCell(max.y);

                for (int cellX = minCellX; cellX <= maxCellX; cellX++)
                {
                    for (int cellY = minCellY; cellY <= maxCellY; cellY++)
                    {
                        (int, int) key = (cellX, cellY);
                        if (!index.TryGetValue(key, out List<int> polygonIndices))
                        {
                            polygonIndices = new List<int>();
                            index[key] = polygonIndices;
                        }

                        polygonIndices.Add(i);
                    }
                }
            }

            return index;
        }

        private IEnumerable<int> QueryCandidatePolygons(Vector2 point, Dictionary<(int, int), List<int>> index, IReadOnlyList<Polygon2D> polygons)
        {
            HashSet<int> candidates = new HashSet<int>();
            int centerX = WorldToCell(point.x);
            int centerY = WorldToCell(point.y);

            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    if (!index.TryGetValue((centerX + dx, centerY + dy), out List<int> polygonIndices))
                    {
                        continue;
                    }

                    foreach (int polygonIndex in polygonIndices)
                    {
                        candidates.Add(polygonIndex);
                    }
                }
            }

            // 空索引时回退到全量检查，避免未加载或极小网格配置导致误判。
            if (candidates.Count == 0 && polygons.Count > 0 && index.Count == 0)
            {
                for (int i = 0; i < polygons.Count; i++)
                {
                    candidates.Add(i);
                }
            }

            return candidates;
        }

        private int WorldToCell(float value)
        {
            return Mathf.FloorToInt(value / Mathf.Max(0.001f, gridCellSize));
        }

        private bool IsInsideBounds(Vector2 point)
        {
            if (bounds == Vector4.zero)
            {
                return true;
            }

            return point.x >= bounds.x && point.x <= bounds.z && point.y >= bounds.y && point.y <= bounds.w;
        }

        private static bool TryGetAabb(Polygon2D polygon, out Vector2 min, out Vector2 max)
        {
            min = new Vector2(float.PositiveInfinity, float.PositiveInfinity);
            max = new Vector2(float.NegativeInfinity, float.NegativeInfinity);

            if (polygon.outerRing == null || polygon.outerRing.Length == 0)
            {
                return false;
            }

            foreach (Vector2 point in polygon.outerRing)
            {
                min = Vector2.Min(min, point);
                max = Vector2.Max(max, point);
            }

            return true;
        }

        private bool PointInPolygon(Vector2 point, Polygon2D polygon)
        {
            bool inside = PointInRing(point, polygon.outerRing);
            if (inside && polygon.holes != null)
            {
                foreach (Vector2[] hole in polygon.holes)
                {
                    if (PointInRing(point, hole))
                    {
                        inside = false;
                        break;
                    }
                }
            }

            return inside;
        }

        private bool PointInRing(Vector2 point, Vector2[] ring)
        {
            if (ring == null || ring.Length < 3)
            {
                return false;
            }

            if (PointOnRing(point, ring, 0.001f))
            {
                return true;
            }

            bool inside = false;
            int j = ring.Length - 1;
            for (int i = 0; i < ring.Length; j = i++)
            {
                if ((ring[i].y > point.y) != (ring[j].y > point.y) &&
                    point.x < (ring[j].x - ring[i].x) * (point.y - ring[i].y)
                              / (ring[j].y - ring[i].y) + ring[i].x)
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        private static bool PointOnRing(Vector2 point, Vector2[] ring, float epsilon)
        {
            float epsilonSq = epsilon * epsilon;
            for (int i = 0; i < ring.Length; i++)
            {
                Vector2 a = ring[i];
                Vector2 b = ring[(i + 1) % ring.Length];
                if ((NearestPointOnSegment(point, a, b) - point).sqrMagnitude <= epsilonSq)
                {
                    return true;
                }
            }

            return false;
        }

        private static void FindNearestPointOnRing(Vector2 point, Vector2[] ring, ref Vector2 bestPoint, ref float bestDistanceSq)
        {
            if (ring == null || ring.Length < 2)
            {
                return;
            }

            for (int i = 0; i < ring.Length; i++)
            {
                Vector2 candidate = NearestPointOnSegment(point, ring[i], ring[(i + 1) % ring.Length]);
                float distanceSq = (candidate - point).sqrMagnitude;
                if (distanceSq < bestDistanceSq)
                {
                    bestDistanceSq = distanceSq;
                    bestPoint = candidate;
                }
            }
        }

        private static Vector2 NearestPointOnSegment(Vector2 p, Vector2 a, Vector2 b)
        {
            Vector2 ab = b - a;
            float denominator = Vector2.Dot(ab, ab);
            if (denominator <= 0.000001f)
            {
                return a;
            }

            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / denominator);
            return a + t * ab;
        }

        private static Mesh CreateFlatMesh(Vector2[] ring, float y, string meshName)
        {
            int[] indices = TriangulateEarClipping(ring);
            if (indices.Length < 3)
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
            mesh.triangles = indices;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Mesh CreateExtrudedMesh(Vector2[] ring, float height, string meshName)
        {
            int[] flatTriangles = TriangulateEarClipping(ring);
            List<Vector3> vertices = new List<Vector3>(ring.Length * 2);
            List<int> triangles = new List<int>();

            for (int i = 0; i < ring.Length; i++)
            {
                vertices.Add(new Vector3(ring[i].x, 0f, ring[i].y));
            }

            for (int i = 0; i < ring.Length; i++)
            {
                vertices.Add(new Vector3(ring[i].x, height, ring[i].y));
            }

            for (int i = 0; i < flatTriangles.Length; i += 3)
            {
                triangles.Add(flatTriangles[i + 2]);
                triangles.Add(flatTriangles[i + 1]);
                triangles.Add(flatTriangles[i]);

                triangles.Add(flatTriangles[i] + ring.Length);
                triangles.Add(flatTriangles[i + 1] + ring.Length);
                triangles.Add(flatTriangles[i + 2] + ring.Length);
            }

            for (int i = 0; i < ring.Length; i++)
            {
                int next = (i + 1) % ring.Length;
                int bottomA = i;
                int bottomB = next;
                int topA = i + ring.Length;
                int topB = next + ring.Length;

                triangles.Add(bottomA);
                triangles.Add(topA);
                triangles.Add(topB);
                triangles.Add(bottomA);
                triangles.Add(topB);
                triangles.Add(bottomB);
            }

            Mesh mesh = new Mesh { name = meshName };
            mesh.vertices = vertices.ToArray();
            mesh.triangles = triangles.ToArray();
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

        private static Material CreateTransparentUnlitMaterial(Color color)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
            {
                shader = Shader.Find("Unlit/Color");
            }

            Material material = new Material(shader)
            {
                name = "WalkableDebugTransparentMaterial",
                color = color
            };

            if (material.HasProperty("_BaseColor"))
            {
                material.SetColor("_BaseColor", color);
            }

            material.SetFloat("_Surface", 1f);
            material.SetOverrideTag("RenderType", "Transparent");
            material.renderQueue = 3000;
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            return material;
        }

        private void OnDrawGizmos()
        {
            const float gizmoY = 0.01f;

            if (drawWalkableArea)
            {
                Gizmos.color = walkableColor;
                foreach (Polygon2D polygon in walkablePolygons)
                {
                    DrawRingLines(polygon.outerRing, gizmoY);
                    if (polygon.holes == null)
                    {
                        continue;
                    }

                    foreach (Vector2[] hole in polygon.holes)
                    {
                        DrawRingLines(hole, gizmoY);
                    }
                }
            }

            if (drawObstacles)
            {
                Gizmos.color = obstacleColor;
                foreach (Polygon2D polygon in obstaclePolygons)
                {
                    Mesh mesh = CreateFlatMesh(polygon.outerRing, gizmoY + 0.01f, "ObstacleGizmoMesh");
                    if (mesh != null)
                    {
                        Gizmos.DrawMesh(mesh);
                        DestroyUnityObject(mesh);
                    }

                    DrawRingLines(polygon.outerRing, gizmoY + 0.02f);
                }
            }

            if (bounds != Vector4.zero)
            {
                Gizmos.color = Color.cyan;
                Vector3 a = new Vector3(bounds.x, 0.05f, bounds.y);
                Vector3 b = new Vector3(bounds.z, 0.05f, bounds.y);
                Vector3 c = new Vector3(bounds.z, 0.05f, bounds.w);
                Vector3 d = new Vector3(bounds.x, 0.05f, bounds.w);
                Gizmos.DrawSphere(a, 0.2f);
                Gizmos.DrawSphere(b, 0.2f);
                Gizmos.DrawSphere(c, 0.2f);
                Gizmos.DrawSphere(d, 0.2f);
                Gizmos.DrawLine(a, b);
                Gizmos.DrawLine(b, c);
                Gizmos.DrawLine(c, d);
                Gizmos.DrawLine(d, a);
            }
        }

        private static void DrawRingLines(Vector2[] ring, float y)
        {
            if (ring == null || ring.Length < 2)
            {
                return;
            }

            for (int i = 0; i < ring.Length; i++)
            {
                Vector2 start = ring[i];
                Vector2 end = ring[(i + 1) % ring.Length];
                Gizmos.DrawLine(new Vector3(start.x, y, start.y), new Vector3(end.x, y, end.y));
            }
        }

        private static void DestroyUnityObject(UnityEngine.Object unityObject)
        {
            if (unityObject == null)
            {
                return;
            }

            if (Application.isPlaying)
            {
                UnityEngine.Object.Destroy(unityObject);
            }
            else
            {
                UnityEngine.Object.DestroyImmediate(unityObject);
            }
        }
    }
}
