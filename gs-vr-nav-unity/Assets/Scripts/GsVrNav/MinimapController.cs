using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace GsVrNav.Unity
{
    /// <summary>
    /// Runtime minimap shell for Blosm-only navigation. V1 shows player position/orientation only.
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
        private Transform playerTransform;

        [SerializeField]
        private KeyCode toggleKey = KeyCode.M;

        [SerializeField]
        private bool rotateWithPlayerView = true;

        private const string MinimapLayerName = "Minimap";
        private const float CameraHeight = 50f;
        private const float GeometryY = 0.05f;

        private Camera minimapCamera;
        private RenderTexture renderTexture;
        private GameObject minimapGeometryRoot;
        private GameObject playerArrow;
        private GameObject playerMarker;
        private int minimapLayer;
        private bool isVisible = true;

        private void Awake()
        {
            if (playerTransform == null)
            {
                Camera mainCamera = Camera.main;
                playerTransform = mainCamera != null ? mainCamera.transform : transform;
            }

            minimapLayer = LayerMask.NameToLayer(MinimapLayerName);
            if (minimapLayer < 0)
            {
                minimapLayer = 0;
                Debug.LogWarning("Layer 'Minimap' was not found. Minimap geometry will use Default layer.");
            }

            CreateRenderTexture();
            CreateCamera();
            CreateUiIfNeeded();
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

        public void BuildMinimapGeometry()
        {
            if (minimapGeometryRoot != null)
            {
                Destroy(minimapGeometryRoot);
            }

            minimapGeometryRoot = new GameObject("MinimapGeometry");
            minimapGeometryRoot.layer = minimapLayer;
            minimapGeometryRoot.transform.SetParent(transform, false);

            playerMarker = CreatePlayerMarker();
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

        private GameObject CreatePlayerMarker()
        {
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            marker.name = "PlayerMarker";
            marker.transform.SetParent(minimapGeometryRoot.transform, false);
            marker.transform.localScale = new Vector3(1.2f, 0.1f, 1.2f);
            marker.layer = minimapLayer;

            Renderer renderer = marker.GetComponent<Renderer>();
            renderer.sharedMaterial = CreateFillMaterial(Color.cyan);
            Destroy(marker.GetComponent<Collider>());
            return marker;
        }

        private GameObject CreatePlayerArrow()
        {
            GameObject arrow = new GameObject("PlayerArrow");
            arrow.transform.SetParent(minimapGeometryRoot.transform, false);
            arrow.layer = minimapLayer;

            MeshFilter meshFilter = arrow.AddComponent<MeshFilter>();
            MeshRenderer meshRenderer = arrow.AddComponent<MeshRenderer>();
            meshFilter.sharedMesh = CreateArrowMesh();
            meshRenderer.sharedMaterial = CreateFillMaterial(new Color(0f, 0.8f, 1f, 1f));
            return arrow;
        }

        private static Mesh CreateArrowMesh()
        {
            Mesh mesh = new Mesh { name = "MinimapPlayerArrow" };
            mesh.vertices = new[]
            {
                new Vector3(0f, GeometryY, 2.0f),
                new Vector3(-0.75f, GeometryY, -0.8f),
                new Vector3(0.75f, GeometryY, -0.8f)
            };
            mesh.triangles = new[] { 0, 1, 2 };
            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            return mesh;
        }

        private void UpdateCameraFollow()
        {
            if (minimapCamera == null || playerTransform == null)
            {
                return;
            }

            Vector3 playerPosition = playerTransform.position;
            minimapCamera.transform.position = new Vector3(playerPosition.x, CameraHeight, playerPosition.z);
            float yaw = rotateWithPlayerView ? playerTransform.eulerAngles.y : 0f;
            minimapCamera.transform.rotation = Quaternion.Euler(90f, yaw, 0f);
        }

        private void UpdatePlayerArrow()
        {
            if (playerTransform == null)
            {
                return;
            }

            Vector3 markerPosition = new Vector3(playerTransform.position.x, GeometryY, playerTransform.position.z);
            if (playerMarker != null)
            {
                playerMarker.transform.position = markerPosition;
            }

            if (playerArrow != null)
            {
                playerArrow.transform.position = markerPosition;
                playerArrow.transform.rotation = Quaternion.Euler(0f, playerTransform.eulerAngles.y, 0f);
            }
        }

        private void UpdateZoom()
        {
            if (minimapCamera == null)
            {
                return;
            }

            float scroll = Mouse.current != null ? Mouse.current.scroll.ReadValue().y : 0f;
            if (Mathf.Abs(scroll) <= 0.01f)
            {
                return;
            }

            viewRange = Mathf.Clamp(viewRange - scroll * 0.1f, minViewRange, maxViewRange);
            minimapCamera.orthographicSize = viewRange * 0.5f;
        }

        private bool IsTogglePressed()
        {
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
            {
                return false;
            }

            return toggleKey == KeyCode.M && keyboard.mKey.wasPressedThisFrame;
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

            return material;
        }
    }
}
