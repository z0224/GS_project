// VR 连续移动控制器，带导航网格约束。
// 同时支持 XR Device Simulator（编辑器）和真机 XR 输入。
// MVP 阶段仅使用 XR Device Simulator 在 Unity 编辑器中测试，不部署真机 VR。
//
// Unity 项目配置要点：
// - Project Settings > XR Plug-in Management 中不要勾选 Oculus 或 OpenXR；MVP 只使用 XR Device Simulator。
// - Project Settings > Player > Active Input Handling 设为 "Both" 或 "Input System Package (New)"。
// - 构建平台保持默认 Standalone (PC)，不需要切换到 Android。
//
// 坐标系映射表（所有导航查询都使用 Unity XZ 平面，对应 Python ENU East/North）：
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

using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GsVrNav.Unity
{
    /// <summary>
    /// Head-directed continuous locomotion controller constrained by the loaded 2D navigation mesh.
    /// </summary>
    public sealed class VRNavigationController : MonoBehaviour
    {
        [Header("Movement Settings")]
        [SerializeField]
        private float moveSpeed = 3.0f;

        [SerializeField]
        private float deadZone = 0.15f;

        [SerializeField]
        private float boundaryLerpFactor = 0.3f;

        [SerializeField]
        private bool snapToWalkableOnStart = true;

        [SerializeField]
        private bool usePhysicsCharacterController = true;

        [Header("Input")]
        [SerializeField]
        private InputActionReference moveInput;

        [Header("References")]
        [SerializeField]
        private GeoAlignmentLoader geoAlignmentLoader;

        [SerializeField]
        private Transform xrOrigin;

        [SerializeField]
        private Camera headCamera;

        [Header("Debug")]
        [SerializeField]
        private bool showDebugGizmos = true;

        private const float PlayerRadiusMeters = 0.3f;
        private const float DebugLogIntervalSeconds = 1.0f;

        private Vector3 lastMovementWorld;
        private Vector3 lastClampPosition;
        private bool wasClampedLastFrame;
        private float currentSpeed;
        private float lastClampLogTime = -999f;
        private float fps;
        private CharacterController characterController;

        private void Awake()
        {
            if (xrOrigin == null)
            {
                xrOrigin = transform;
            }

            if (headCamera == null)
            {
                headCamera = Camera.main;
            }

            if (geoAlignmentLoader == null)
            {
#if UNITY_2023_1_OR_NEWER
                geoAlignmentLoader = FindFirstObjectByType<GeoAlignmentLoader>();
#else
                geoAlignmentLoader = FindObjectOfType<GeoAlignmentLoader>();
#endif
            }

            if (usePhysicsCharacterController)
            {
                characterController = xrOrigin.GetComponent<CharacterController>();
                if (characterController == null)
                {
                    characterController = xrOrigin.gameObject.AddComponent<CharacterController>();
                }

                characterController.radius = PlayerRadiusMeters;
                characterController.height = 1.8f;
                characterController.center = new Vector3(0f, 0.9f, 0f);
                characterController.skinWidth = 0.03f;
                characterController.minMoveDistance = 0f;
            }
        }

        private IEnumerator Start()
        {
            yield return null;
            SnapToSafeWalkableStart();
        }

        private void SnapToSafeWalkableStart()
        {
            if (!snapToWalkableOnStart || geoAlignmentLoader == null || xrOrigin == null)
            {
                return;
            }

            Vector3 position = xrOrigin.position;
            if (geoAlignmentLoader.IsWalkable(position.x, position.z))
            {
                return;
            }

            Vector2 snapped = geoAlignmentLoader.FindSafeWalkablePoint(position.x, position.z);
            SetPlayerPosition(new Vector3(snapped.x, 0f, snapped.y));
            Debug.Log($"Player start position was outside the OSM walkable area and was snapped to ({snapped.x:F2}, {snapped.y:F2}).");
        }

        private void OnEnable()
        {
            if (moveInput != null && moveInput.action != null && !moveInput.action.enabled)
            {
                moveInput.action.Enable();
            }
        }

        private void Update()
        {
            if (xrOrigin == null)
            {
                return;
            }

            fps = Time.unscaledDeltaTime > 0f ? 1f / Time.unscaledDeltaTime : 0f;

            Vector2 input = ReadMoveInput();
            Vector2 adjustedInput = ApplyDeadZone(input);
            Vector3 movementDirection = CalculateHeadDirectedMovement(adjustedInput);

            currentSpeed = moveSpeed * Mathf.Clamp01(adjustedInput.magnitude);
            lastMovementWorld = movementDirection * currentSpeed;
            wasClampedLastFrame = false;

            if (geoAlignmentLoader != null && !geoAlignmentLoader.BlosmGeometryReady)
            {
                currentSpeed = 0f;
                return;
            }

            Vector3 currentPosition = xrOrigin.position;
            currentPosition.y = 0f; // MVP 地面高度锁定：玩家始终在 Unity y=0 平面移动。

            Vector3 proposed = currentPosition + lastMovementWorld * Time.deltaTime;
            proposed.y = 0f;

            if (geoAlignmentLoader != null)
            {
                // Unity XZ 平面直接对应 ENU (East, North)，所以查询时传入 (x, z)。
                Vector2 proposed2D = new Vector2(proposed.x, proposed.z);
                if (!geoAlignmentLoader.IsWalkable(proposed2D.x, proposed2D.y))
                {
                    Vector2 clamped = geoAlignmentLoader.ClampMovement(
                        new Vector2(currentPosition.x, currentPosition.z),
                        proposed2D);

                    Vector3 clampedPosition = new Vector3(clamped.x, 0f, clamped.y);
                    proposed = Vector3.Lerp(currentPosition, clampedPosition, boundaryLerpFactor);
                    proposed.y = 0f;

                    wasClampedLastFrame = true;
                    lastClampPosition = clampedPosition;

                    if (Time.unscaledTime - lastClampLogTime >= DebugLogIntervalSeconds)
                    {
                        Debug.Log("Movement clamped at boundary");
                        lastClampLogTime = Time.unscaledTime;
                    }
                }
            }

            MovePlayer(proposed - xrOrigin.position);
        }

        private void MovePlayer(Vector3 delta)
        {
            delta.y = 0f;
            if (usePhysicsCharacterController && characterController != null && characterController.enabled)
            {
                characterController.Move(delta);
                Vector3 position = xrOrigin.position;
                position.y = 0f;
                xrOrigin.position = position;
                return;
            }

            xrOrigin.position += delta;
        }

        private void SetPlayerPosition(Vector3 position)
        {
            if (characterController != null)
            {
                characterController.enabled = false;
                xrOrigin.position = position;
                characterController.enabled = true;
                return;
            }

            xrOrigin.position = position;
        }

        private Vector2 ReadMoveInput()
        {
            if (moveInput != null && moveInput.action != null)
            {
                return moveInput.action.ReadValue<Vector2>();
            }

            // 回退路径：未配置 XRI InputActionReference 时，直接用键盘 WASD 便于编辑器快速冒烟测试。
            Vector2 keyboardInput = Vector2.zero;
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
            {
                return keyboardInput;
            }

            if (keyboard.aKey.isPressed)
            {
                keyboardInput.x -= 1f;
            }

            if (keyboard.dKey.isPressed)
            {
                keyboardInput.x += 1f;
            }

            if (keyboard.sKey.isPressed)
            {
                keyboardInput.y -= 1f;
            }

            if (keyboard.wKey.isPressed)
            {
                keyboardInput.y += 1f;
            }

            return Vector2.ClampMagnitude(keyboardInput, 1f);
        }

        private Vector2 ApplyDeadZone(Vector2 input)
        {
            float magnitude = input.magnitude;
            if (magnitude <= deadZone)
            {
                return Vector2.zero;
            }

            // 死区外线性重映射：deadZone 刚外侧为 0，摇杆满幅为 1。
            float normalizedMagnitude = Mathf.InverseLerp(deadZone, 1f, Mathf.Clamp01(magnitude));
            return input.normalized * normalizedMagnitude;
        }

        private Vector3 CalculateHeadDirectedMovement(Vector2 input)
        {
            if (input.sqrMagnitude <= 0f)
            {
                return Vector3.zero;
            }

            Transform referenceTransform = headCamera != null ? headCamera.transform : xrOrigin;

            Vector3 forward = Vector3.ProjectOnPlane(referenceTransform.forward, Vector3.up).normalized;
            Vector3 right = Vector3.ProjectOnPlane(referenceTransform.right, Vector3.up).normalized;

            if (forward.sqrMagnitude < 0.0001f)
            {
                forward = xrOrigin.forward;
                forward.y = 0f;
                forward.Normalize();
            }

            Vector3 movement = forward * input.y + right * input.x;
            movement.y = 0f;
            return movement.sqrMagnitude > 1f ? movement.normalized : movement;
        }

        private void OnDrawGizmos()
        {
            if (!showDebugGizmos)
            {
                return;
            }

            Transform origin = xrOrigin != null ? xrOrigin : transform;
            Vector3 position = origin.position;
            position.y = 0f;

            Gizmos.color = Color.green;
            Gizmos.DrawLine(position, position + lastMovementWorld);

            if (wasClampedLastFrame)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(lastClampPosition + Vector3.up * 0.05f, 0.08f);
            }

            Gizmos.color = Color.yellow;
            DrawWireCylinder(position, PlayerRadiusMeters, 1.8f, 32);
        }

        private void DrawWireCylinder(Vector3 center, float radius, float height, int segments)
        {
            float bottomY = center.y;
            float topY = center.y + height;

            for (int i = 0; i < segments; i++)
            {
                float angleA = Mathf.PI * 2f * i / segments;
                float angleB = Mathf.PI * 2f * (i + 1) / segments;

                Vector3 bottomA = new Vector3(center.x + Mathf.Cos(angleA) * radius, bottomY, center.z + Mathf.Sin(angleA) * radius);
                Vector3 bottomB = new Vector3(center.x + Mathf.Cos(angleB) * radius, bottomY, center.z + Mathf.Sin(angleB) * radius);
                Vector3 topA = new Vector3(bottomA.x, topY, bottomA.z);
                Vector3 topB = new Vector3(bottomB.x, topY, bottomB.z);

                Gizmos.DrawLine(bottomA, bottomB);
                Gizmos.DrawLine(topA, topB);
                if (i % 8 == 0)
                {
                    Gizmos.DrawLine(bottomA, topA);
                }
            }
        }

        private void OnGUI()
        {
            Transform origin = xrOrigin != null ? xrOrigin : transform;
            Vector3 position = origin.position;
            bool walkable = geoAlignmentLoader == null || geoAlignmentLoader.IsWalkable(position.x, position.z);

            GUIStyle style = new GUIStyle(GUI.skin.label)
            {
                fontSize = 16,
                normal = { textColor = Color.white }
            };

            string text =
                $"Position: ({position.x:F2}, {0f:F2}, {position.z:F2})\n" +
                $"Speed: {currentSpeed:F1} m/s\n" +
                $"Walkable: {(walkable ? "Yes" : "No")}\n" +
                $"FPS: {Mathf.RoundToInt(fps)}";

            GUI.Label(new Rect(12f, 12f, 260f, 100f), text, style);
        }
    }
}
