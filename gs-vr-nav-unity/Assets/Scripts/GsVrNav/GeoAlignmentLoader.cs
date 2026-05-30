using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace GsVrNav.Unity
{
    /// <summary>
    /// Loads Blosm-exported map geometry and uses it as the authoritative navigation/collision source.
    /// </summary>
    public sealed class GeoAlignmentLoader : MonoBehaviour
    {
        [Header("Blosm Map Geometry")]
        [SerializeField]
        private bool loadOnAwake = true;

        [SerializeField]
        private bool showBlosmMapGeometryOnLoad = false;

        [SerializeField]
        private string blosmMapAssetPath = "Assets/External/BlosmMap/Blosm_Map.fbx";

        [SerializeField]
        private string blosmGeneratorWorkingDirectory = "../gs-vr-nav";

        [SerializeField]
        private string blosmTransformsJsonPath = "transforms.json";

        [SerializeField]
        private string blosmPythonExecutable = "python";

        [SerializeField]
        private string blenderExecutablePath = "";

        [Header("Map Alignment")]
        [SerializeField]
        private bool applyMapAlignmentOnAwake = true;

        [SerializeField]
        private string mapAlignmentJsonPath = "Alignment/map_alignment.json";

        [SerializeField]
        private float defaultMapYOffset = -1f;

        [SerializeField]
        private string expectedInputPlyHash = "";

        [SerializeField]
        private string expectedOsmJsonHash = "";

        [SerializeField]
        private bool validateExpectedOrigin = false;

        [SerializeField]
        private float expectedOriginLat = 0f;

        [SerializeField]
        private float expectedOriginLon = 0f;

        [SerializeField]
        private float expectedOriginAlt = 0f;

        [SerializeField]
        private float originValidationTolerance = 0.000001f;

        [Header("Blosm Navigation")]
        [SerializeField]
        private float walkableRaycastHeight = 20f;

        [SerializeField]
        private float walkableRaycastDistance = 60f;

        [SerializeField]
        private float safePointSearchRadius = 6f;

        [SerializeField]
        private float safePointSearchStep = 0.5f;

        private const string BlosmMapGeometryRootName = "Blosm_MapGeometry";

        private static readonly string[] WalkableKeywords =
        {
            "road", "roads", "path", "paths", "footway", "pedestrian", "sidewalk", "street"
        };

        private static readonly string[] BuildingKeywords =
        {
            "building", "buildings"
        };

        private readonly List<MeshCollider> walkableColliders = new List<MeshCollider>();
        private readonly List<MeshCollider> buildingColliders = new List<MeshCollider>();
        private bool blosmGeometryReady;

        public bool BlosmGeometryReady => blosmGeometryReady;

        private void Awake()
        {
            if (loadOnAwake)
            {
                AttachBlosmColliders();
            }
        }

        public bool IsWalkable(float x, float z)
        {
            return TryFindWalkableSurface(new Vector2(x, z), out _);
        }

        public bool TryGetWalkableSurface(float x, float z, out Vector3 hitPoint)
        {
            return TryFindWalkableSurface(new Vector2(x, z), out hitPoint);
        }

        public Vector2 ClampToWalkable(float x, float z)
        {
            return FindSafeWalkablePoint(x, z);
        }

        public Vector3 FindSafeWalkableSurfacePoint(float x, float z, float inwardDistance = 0.25f, float searchRadius = 2.0f)
        {
            Vector2 point = new Vector2(x, z);
            if (TryFindWalkableSurface(point, out Vector3 hitPoint))
            {
                return hitPoint;
            }

            float maxRadius = Mathf.Max(searchRadius, safePointSearchRadius, inwardDistance);
            float step = Mathf.Max(0.1f, safePointSearchStep);
            float bestDistanceSq = float.PositiveInfinity;
            Vector3 bestPoint = new Vector3(x, 0f, z);
            bool found = false;

            for (float radius = step; radius <= maxRadius + 0.001f; radius += step)
            {
                int samples = Mathf.Max(12, Mathf.CeilToInt(radius * 12f));
                for (int i = 0; i < samples; i++)
                {
                    float angle = Mathf.PI * 2f * i / samples;
                    Vector2 candidate = point + new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * radius;
                    if (!TryFindWalkableSurface(candidate, out Vector3 candidateHit))
                    {
                        continue;
                    }

                    Vector2 candidate2D = new Vector2(candidateHit.x, candidateHit.z);
                    float distanceSq = (candidate2D - point).sqrMagnitude;
                    if (distanceSq < bestDistanceSq)
                    {
                        bestDistanceSq = distanceSq;
                        bestPoint = candidateHit;
                        found = true;
                    }
                }

                if (found)
                {
                    return bestPoint;
                }
            }

            return bestPoint;
        }

        public Vector2 FindSafeWalkablePoint(float x, float z, float inwardDistance = 0.25f, float searchRadius = 2.0f)
        {
            Vector3 point = FindSafeWalkableSurfacePoint(x, z, inwardDistance, searchRadius);
            return new Vector2(point.x, point.z);
        }

        public Vector2 ClampMovement(Vector2 from, Vector2 to)
        {
            if (IsWalkable(to.x, to.y))
            {
                return to;
            }

            if (!IsWalkable(from.x, from.y))
            {
                return FindSafeWalkablePoint(to.x, to.y);
            }

            Vector2 low = from;
            Vector2 high = to;
            for (int i = 0; i < 8; i++)
            {
                Vector2 mid = Vector2.Lerp(low, high, 0.5f);
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

        public Vector3 ClampMovementToSurface(Vector3 from, Vector3 to)
        {
            if (TryGetWalkableSurface(to.x, to.z, out Vector3 targetSurface))
            {
                return targetSurface;
            }

            if (!TryGetWalkableSurface(from.x, from.z, out Vector3 fromSurface))
            {
                return FindSafeWalkableSurfacePoint(to.x, to.z);
            }

            Vector2 low = new Vector2(from.x, from.z);
            Vector2 high = new Vector2(to.x, to.z);
            Vector3 lowSurface = fromSurface;
            for (int i = 0; i < 8; i++)
            {
                Vector2 mid = Vector2.Lerp(low, high, 0.5f);
                if (TryFindWalkableSurface(mid, out Vector3 midSurface))
                {
                    low = mid;
                    lowSurface = midSurface;
                }
                else
                {
                    high = mid;
                }
            }

            return lowSurface;
        }

        [ContextMenu("Generate Blosm Map Asset")]
        public void GenerateBlosmMapAsset()
        {
#if UNITY_EDITOR
            string workingDirectory = ResolveProjectRelativePath(blosmGeneratorWorkingDirectory);
            string outputPath = ResolveProjectRelativePath(blosmMapAssetPath);
            Directory.CreateDirectory(Path.GetDirectoryName(outputPath));

            List<string> arguments = new List<string>
            {
                "-m",
                "geo_alignment.generate_blosm_map",
                "--transforms-json",
                blosmTransformsJsonPath,
                "--output",
                outputPath
            };

            if (!string.IsNullOrWhiteSpace(blenderExecutablePath))
            {
                arguments.Add("--blender-exe");
                arguments.Add(blenderExecutablePath);
            }

            System.Diagnostics.ProcessStartInfo startInfo = new System.Diagnostics.ProcessStartInfo
            {
                FileName = string.IsNullOrWhiteSpace(blosmPythonExecutable) ? "python" : blosmPythonExecutable,
                Arguments = JoinCommandLineArguments(arguments),
                WorkingDirectory = workingDirectory,
                UseShellExecute = false,
                RedirectStandardError = true,
                RedirectStandardOutput = true,
                CreateNoWindow = true
            };

            using (System.Diagnostics.Process process = System.Diagnostics.Process.Start(startInfo))
            {
                string stdout = process.StandardOutput.ReadToEnd();
                string stderr = process.StandardError.ReadToEnd();
                process.WaitForExit();

                if (process.ExitCode != 0)
                {
                    Debug.LogError($"Blosm map asset generation failed.\nCommand: {startInfo.FileName} {startInfo.Arguments}\n{stdout}\n{stderr}");
                    return;
                }

                Debug.Log($"Blosm map asset generated at {outputPath}\n{stdout}");
            }

            AssetDatabase.Refresh();
            AssetDatabase.ImportAsset(ProjectAbsolutePathToAssetPath(outputPath), ImportAssetOptions.ForceUpdate);
#else
            Debug.LogWarning("Generate Blosm Map Asset is only available in the Unity Editor.");
#endif
        }

        [ContextMenu("Attach Blosm Colliders")]
        public void AttachBlosmColliders()
        {
            ClearBlosmRuntimeGeometry();
            walkableColliders.Clear();
            buildingColliders.Clear();
            blosmGeometryReady = false;

            GameObject blosmPrefab = LoadBlosmMapPrefab();
            if (blosmPrefab == null)
            {
                Debug.LogError($"Blosm map asset is required but was not found at {blosmMapAssetPath}. Run 'Generate Blosm Map Asset' first.");
                return;
            }

            GameObject root = new GameObject(BlosmMapGeometryRootName);
            root.transform.SetParent(transform, false);
            ApplyMapAlignmentIfAvailable(root.transform);

            GameObject instance = Instantiate(blosmPrefab, root.transform);
            instance.name = Path.GetFileNameWithoutExtension(blosmMapAssetPath);
            instance.transform.localPosition = Vector3.zero;
            instance.transform.localRotation = Quaternion.identity;
            instance.transform.localScale = Vector3.one;

            AttachAndClassifyMeshColliders(instance.transform);
            SetRenderersVisible(root.transform, showBlosmMapGeometryOnLoad);

            if (walkableColliders.Count == 0)
            {
                Debug.LogError("Blosm map geometry loaded, but no road/path walkable meshes were detected. Check Blosm object names or export script prefixes.");
                return;
            }

            blosmGeometryReady = true;
            Debug.Log($"Blosm map geometry ready: {walkableColliders.Count} walkable colliders, {buildingColliders.Count} building colliders.");
        }

        private void ApplyMapAlignmentIfAvailable(Transform root)
        {
            if (!applyMapAlignmentOnAwake)
            {
                return;
            }

            string alignmentPath = ResolveStreamingAssetsPath(mapAlignmentJsonPath);
            if (!File.Exists(alignmentPath))
            {
                Debug.LogWarning($"Map alignment JSON was not found at {alignmentPath}; Blosm map geometry will use identity alignment.");
                root.localPosition = Vector3.zero;
                root.localRotation = Quaternion.identity;
                root.localScale = Vector3.one;
                return;
            }

            MapAlignmentJson alignment;
            try
            {
                alignment = JsonUtility.FromJson<MapAlignmentJson>(File.ReadAllText(alignmentPath));
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"Failed to read map alignment JSON at {alignmentPath}: {exception.Message}. Blosm map geometry will use identity alignment.");
                root.localPosition = Vector3.zero;
                root.localRotation = Quaternion.identity;
                root.localScale = Vector3.one;
                return;
            }

            if (alignment == null)
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} was empty; Blosm map geometry will use identity alignment.");
                root.localPosition = Vector3.zero;
                root.localRotation = Quaternion.identity;
                root.localScale = Vector3.one;
                return;
            }

            if (!IsMapAlignmentSafeToApply(alignment, alignmentPath))
            {
                root.localPosition = Vector3.zero;
                root.localRotation = Quaternion.identity;
                root.localScale = Vector3.one;
                return;
            }

            float scale = Mathf.Approximately(alignment.scale, 0f) ? 1f : alignment.scale;
            MapAlignmentPosition position = alignment.position ?? new MapAlignmentPosition();
            if (alignment.position == null)
            {
                position.y = defaultMapYOffset;
            }
            root.localPosition = new Vector3(position.x, position.y, position.z);
            root.localRotation = Quaternion.Euler(0f, alignment.rotation_y_deg, 0f);
            root.localScale = new Vector3(scale, scale, scale);

            Debug.Log(
                $"Applied map alignment from {alignmentPath}: " +
                $"position=({position.x:F3}, {position.y:F3}, {position.z:F3}), " +
                $"rotationY={alignment.rotation_y_deg:F3}, scale={scale:F3}, " +
                $"fitness={alignment.fitness:F4}, rmse={alignment.rmse:F3}");
        }

        private bool IsMapAlignmentSafeToApply(MapAlignmentJson alignment, string alignmentPath)
        {
            if (!alignment.accepted)
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} is not accepted; Blosm map geometry will use identity alignment.");
                return false;
            }

            if (alignment.coordinate_space != "unity_xz")
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} uses coordinate_space '{alignment.coordinate_space}', expected 'unity_xz'; identity alignment will be used.");
                return false;
            }

            if (alignment.alignment_mode != "map-to-scene")
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} uses alignment_mode '{alignment.alignment_mode}', expected 'map-to-scene'; identity alignment will be used.");
                return false;
            }

            if (alignment.origin_wgs84 == null)
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} has no origin_wgs84 metadata; identity alignment will be used.");
                return false;
            }

            if (!string.IsNullOrWhiteSpace(expectedInputPlyHash) && !string.Equals(expectedInputPlyHash, alignment.input_ply_hash, StringComparison.OrdinalIgnoreCase))
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} input_ply_hash does not match the expected hash; identity alignment will be used.");
                return false;
            }

            if (!string.IsNullOrWhiteSpace(expectedOsmJsonHash) && !string.Equals(expectedOsmJsonHash, alignment.osm_json_hash, StringComparison.OrdinalIgnoreCase))
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} osm_json_hash does not match the expected hash; identity alignment will be used.");
                return false;
            }

            if (validateExpectedOrigin && !ApproximatelyOrigin(alignment.origin_wgs84))
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} origin_wgs84 does not match the expected origin; identity alignment will be used.");
                return false;
            }

            if (alignment.warnings != null && alignment.warnings.Length > 0)
            {
                Debug.LogWarning($"Map alignment JSON at {alignmentPath} includes warnings: {string.Join("; ", alignment.warnings)}");
            }

            return true;
        }

        private bool ApproximatelyOrigin(MapAlignmentOrigin origin)
        {
            return Mathf.Abs(origin.lat - expectedOriginLat) <= originValidationTolerance
                && Mathf.Abs(origin.lon - expectedOriginLon) <= originValidationTolerance
                && Mathf.Abs(origin.alt - expectedOriginAlt) <= Mathf.Max(0.01f, originValidationTolerance);
        }

        [ContextMenu("Toggle Blosm Map Geometry")]
        public void ToggleBlosmMapGeometry()
        {
            Transform root = transform.Find(BlosmMapGeometryRootName);
            if (root == null)
            {
                AttachBlosmColliders();
                root = transform.Find(BlosmMapGeometryRootName);
            }

            if (root == null)
            {
                return;
            }

            SetRenderersVisible(root, !AnyRendererEnabled(root));
        }

        [ContextMenu("Clear Blosm Runtime Geometry")]
        public void ClearBlosmRuntimeGeometry()
        {
            Transform child = transform.Find(BlosmMapGeometryRootName);
            if (child != null)
            {
                DestroyUnityObject(child.gameObject);
            }

            walkableColliders.Clear();
            buildingColliders.Clear();
            blosmGeometryReady = false;
        }

        private void AttachAndClassifyMeshColliders(Transform root)
        {
            MeshFilter[] meshFilters = root.GetComponentsInChildren<MeshFilter>(true);
            for (int i = 0; i < meshFilters.Length; i++)
            {
                Mesh sharedMesh = meshFilters[i].sharedMesh;
                if (sharedMesh == null || sharedMesh.vertexCount == 0)
                {
                    continue;
                }

                MeshCollider meshCollider = meshFilters[i].GetComponent<MeshCollider>();
                if (meshCollider == null)
                {
                    meshCollider = meshFilters[i].gameObject.AddComponent<MeshCollider>();
                }

                meshFilters[i].gameObject.isStatic = true;
                meshCollider.sharedMesh = sharedMesh;
                meshCollider.convex = false;

                string hierarchyName = BuildHierarchyName(meshFilters[i].transform);
                if (ContainsAny(hierarchyName, WalkableKeywords))
                {
                    walkableColliders.Add(meshCollider);
                }
                else if (ContainsAny(hierarchyName, BuildingKeywords))
                {
                    buildingColliders.Add(meshCollider);
                }
                else
                {
                    Debug.LogWarning($"Blosm mesh '{hierarchyName}' was not classified as walkable or building; collider kept for physics only.");
                }
            }
        }

        private bool TryFindWalkableSurface(Vector2 point, out Vector3 hitPoint)
        {
            hitPoint = default;
            if (walkableColliders.Count == 0)
            {
                return false;
            }

            Ray ray = new Ray(new Vector3(point.x, walkableRaycastHeight, point.y), Vector3.down);
            float closestDistance = float.PositiveInfinity;
            bool found = false;

            for (int i = 0; i < walkableColliders.Count; i++)
            {
                MeshCollider collider = walkableColliders[i];
                if (collider == null || !collider.enabled)
                {
                    continue;
                }

                if (collider.Raycast(ray, out RaycastHit hit, walkableRaycastDistance) && hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                    hitPoint = hit.point;
                    found = true;
                }
            }

            return found;
        }

        private GameObject LoadBlosmMapPrefab()
        {
#if UNITY_EDITOR
            return AssetDatabase.LoadAssetAtPath<GameObject>(blosmMapAssetPath);
#else
            return null;
#endif
        }

        private static string BuildHierarchyName(Transform transform)
        {
            List<string> names = new List<string>();
            Transform current = transform;
            while (current != null)
            {
                names.Add(current.name);
                current = current.parent;
            }

            return string.Join("/", names).ToLowerInvariant();
        }

        private static bool ContainsAny(string value, IReadOnlyList<string> keywords)
        {
            for (int i = 0; i < keywords.Count; i++)
            {
                if (value.Contains(keywords[i]))
                {
                    return true;
                }
            }

            return false;
        }

        private static bool AnyRendererEnabled(Transform root)
        {
            MeshRenderer[] renderers = root.GetComponentsInChildren<MeshRenderer>(true);
            for (int i = 0; i < renderers.Length; i++)
            {
                if (renderers[i].enabled)
                {
                    return true;
                }
            }

            return false;
        }

        private static void SetRenderersVisible(Transform root, bool visible)
        {
            MeshRenderer[] renderers = root.GetComponentsInChildren<MeshRenderer>(true);
            for (int i = 0; i < renderers.Length; i++)
            {
                renderers[i].enabled = visible;
            }
        }

        private static string ResolveProjectRelativePath(string path)
        {
            if (Path.IsPathRooted(path))
            {
                return Path.GetFullPath(path);
            }

            string projectRoot = Directory.GetParent(Application.dataPath).FullName;
            return Path.GetFullPath(Path.Combine(projectRoot, path));
        }

        private static string ResolveStreamingAssetsPath(string path)
        {
            if (Path.IsPathRooted(path))
            {
                return Path.GetFullPath(path);
            }

            return Path.GetFullPath(Path.Combine(Application.streamingAssetsPath, path));
        }

        private static string ProjectAbsolutePathToAssetPath(string absolutePath)
        {
            string projectRoot = Directory.GetParent(Application.dataPath).FullName;
            string fullPath = Path.GetFullPath(absolutePath).Replace('\\', '/');
            string fullProjectRoot = Path.GetFullPath(projectRoot).Replace('\\', '/');
            if (fullPath.StartsWith(fullProjectRoot + "/", StringComparison.OrdinalIgnoreCase))
            {
                return fullPath.Substring(fullProjectRoot.Length + 1);
            }

            return absolutePath;
        }

        private static string JoinCommandLineArguments(IReadOnlyList<string> arguments)
        {
            List<string> quoted = new List<string>(arguments.Count);
            for (int i = 0; i < arguments.Count; i++)
            {
                quoted.Add(QuoteCommandLineArgument(arguments[i]));
            }

            return string.Join(" ", quoted);
        }

        private static string QuoteCommandLineArgument(string argument)
        {
            if (string.IsNullOrEmpty(argument))
            {
                return "\"\"";
            }

            if (argument.IndexOfAny(new[] { ' ', '\t', '"' }) < 0)
            {
                return argument;
            }

            return "\"" + argument.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
        }

        private static void DestroyUnityObject(UnityEngine.Object unityObject)
        {
            if (Application.isPlaying)
            {
                Destroy(unityObject);
            }
            else
            {
                DestroyImmediate(unityObject);
            }
        }

        [Serializable]
        private sealed class MapAlignmentJson
        {
            public string coordinate_space;
            public string alignment_mode;
            public bool accepted;
            public MapAlignmentOrigin origin_wgs84;
            public string input_ply;
            public string input_ply_hash;
            public string osm_json;
            public string osm_json_hash;
            public MapAlignmentPosition position;
            public float rotation_y_deg;
            public float scale = 1f;
            public float fitness;
            public float rmse;
            public int source_count;
            public int target_count;
            public string source;
            public string[] warnings;
        }

        [Serializable]
        private sealed class MapAlignmentOrigin
        {
            public float lat;
            public float lon;
            public float alt;
        }

        [Serializable]
        private sealed class MapAlignmentPosition
        {
            public float x;
            public float y;
            public float z;
        }
    }
}
