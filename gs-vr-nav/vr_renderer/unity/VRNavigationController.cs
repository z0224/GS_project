using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;

namespace GsVrNav.Unity
{
    /// <summary>
    /// Desktop first-person demo controller constrained to Blosm walkable surfaces.
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

        [Header("Mouse Look")]
        [SerializeField]
        private float mouseSensitivity = 0.12f;

        [SerializeField]
        private float minPitch = -70f;

        [SerializeField]
        private float maxPitch = 70f;

        [SerializeField]
        private bool lockCursorOnPlay = true;

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
        private float pitchDegrees;
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

            InitializePitchFromCamera();
            EnsureCharacterController();
        }

        private void OnEnable()
        {
            SetCursorLocked(lockCursorOnPlay);
        }

        private void OnDisable()
        {
            SetCursorLocked(false);
        }

        private IEnumerator Start()
        {
            yield return null;
            SnapToSafeWalkableStart();
        }

        private void Update()
        {
            if (xrOrigin == null)
            {
                return;
            }

            fps = Time.unscaledDeltaTime > 0f ? 1f / Time.unscaledDeltaTime : 0f;

            ApplyMouseLook();

            Vector2 input = ApplyDeadZone(ReadKeyboardMoveInput());
            Vector3 movementDirection = CalculateYawRelativeMovement(input);
            currentSpeed = moveSpeed * Mathf.Clamp01(input.magnitude);
            lastMovementWorld = movementDirection * currentSpeed;
            wasClampedLastFrame = false;

            if (geoAlignmentLoader != null && !geoAlignmentLoader.BlosmGeometryReady)
            {
                currentSpeed = 0f;
                return;
            }

            Vector3 currentPosition = xrOrigin.position;
            Vector3 proposed = currentPosition + lastMovementWorld * Time.deltaTime;

            if (geoAlignmentLoader != null)
            {
                Vector2 targetPoint = new Vector2(proposed.x, proposed.z);
                if (geoAlignmentLoader.TryGetWalkableSurface(targetPoint.x, targetPoint.y, out Vector3 proposedSurface))
                {
                    proposed = proposedSurface;
                }
                else
                {
                    Vector3 clampedPosition = geoAlignmentLoader.ClampMovementToSurface(currentPosition, proposed);
                    proposed = Vector3.Lerp(currentPosition, clampedPosition, boundaryLerpFactor);

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

        private void OnApplicationFocus(bool hasFocus)
        {
            if (hasFocus)
            {
                SetCursorLocked(lockCursorOnPlay);
            }
        }

        private void InitializePitchFromCamera()
        {
            if (headCamera == null)
            {
                pitchDegrees = 0f;
                return;
            }

            pitchDegrees = NormalizeAngle(headCamera.transform.localEulerAngles.x);
            pitchDegrees = Mathf.Clamp(pitchDegrees, minPitch, maxPitch);
            headCamera.transform.localRotation = Quaternion.Euler(pitchDegrees, 0f, 0f);
        }

        private void EnsureCharacterController()
        {
            if (!usePhysicsCharacterController || xrOrigin == null)
            {
                return;
            }

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

        private void SnapToSafeWalkableStart()
        {
            if (!snapToWalkableOnStart || geoAlignmentLoader == null || xrOrigin == null)
            {
                return;
            }

            Vector3 position = xrOrigin.position;
            if (geoAlignmentLoader.TryGetWalkableSurface(position.x, position.z, out Vector3 currentSurface))
            {
                SetPlayerPosition(currentSurface);
                return;
            }

            Vector3 snapped = geoAlignmentLoader.FindSafeWalkableSurfacePoint(position.x, position.z);
            SetPlayerPosition(snapped);
            Debug.Log($"Player start position was outside the OSM walkable area and was snapped to ({snapped.x:F2}, {snapped.y:F2}, {snapped.z:F2}).");
        }

        private void ApplyMouseLook()
        {
            Mouse mouse = Mouse.current;
            if (mouse == null || xrOrigin == null)
            {
                return;
            }

            Vector2 delta = mouse.delta.ReadValue();
            if (delta.sqrMagnitude <= 0f)
            {
                return;
            }

            xrOrigin.Rotate(0f, delta.x * mouseSensitivity, 0f, Space.World);

            if (headCamera != null)
            {
                pitchDegrees = Mathf.Clamp(pitchDegrees - delta.y * mouseSensitivity, minPitch, maxPitch);
                headCamera.transform.localRotation = Quaternion.Euler(pitchDegrees, 0f, 0f);
            }
        }

        private Vector2 ReadKeyboardMoveInput()
        {
            Vector2 input = Vector2.zero;
            Keyboard keyboard = Keyboard.current;
            if (keyboard == null)
            {
                return input;
            }

            if (keyboard.aKey.isPressed)
            {
                input.x -= 1f;
            }

            if (keyboard.dKey.isPressed)
            {
                input.x += 1f;
            }

            if (keyboard.sKey.isPressed)
            {
                input.y -= 1f;
            }

            if (keyboard.wKey.isPressed)
            {
                input.y += 1f;
            }

            return Vector2.ClampMagnitude(input, 1f);
        }

        private Vector2 ApplyDeadZone(Vector2 input)
        {
            float magnitude = input.magnitude;
            if (magnitude <= deadZone)
            {
                return Vector2.zero;
            }

            float normalizedMagnitude = Mathf.InverseLerp(deadZone, 1f, Mathf.Clamp01(magnitude));
            return input.normalized * normalizedMagnitude;
        }

        private Vector3 CalculateYawRelativeMovement(Vector2 input)
        {
            if (input.sqrMagnitude <= 0f || xrOrigin == null)
            {
                return Vector3.zero;
            }

            Vector3 forward = Vector3.ProjectOnPlane(xrOrigin.forward, Vector3.up).normalized;
            Vector3 right = Vector3.ProjectOnPlane(xrOrigin.right, Vector3.up).normalized;

            if (forward.sqrMagnitude < 0.0001f)
            {
                forward = Vector3.forward;
            }

            if (right.sqrMagnitude < 0.0001f)
            {
                right = Vector3.right;
            }

            Vector3 movement = forward * input.y + right * input.x;
            movement.y = 0f;
            return movement.sqrMagnitude > 1f ? movement.normalized : movement;
        }

        private void MovePlayer(Vector3 delta)
        {
            if (usePhysicsCharacterController && characterController != null && characterController.enabled)
            {
                characterController.Move(delta);
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

        private void SetCursorLocked(bool locked)
        {
            Cursor.lockState = locked ? CursorLockMode.Locked : CursorLockMode.None;
            Cursor.visible = !locked;
        }

        private static float NormalizeAngle(float angle)
        {
            while (angle > 180f)
            {
                angle -= 360f;
            }

            while (angle < -180f)
            {
                angle += 360f;
            }

            return angle;
        }

        private void OnDrawGizmos()
        {
            if (!showDebugGizmos)
            {
                return;
            }

            Transform origin = xrOrigin != null ? xrOrigin : transform;
            Vector3 position = origin.position;

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
                $"Position: ({position.x:F2}, {position.y:F2}, {position.z:F2})\n" +
                $"Speed: {currentSpeed:F1} m/s\n" +
                $"Walkable: {(walkable ? "Yes" : "No")}\n" +
                $"FPS: {Mathf.RoundToInt(fps)}";

            GUI.Label(new Rect(12f, 12f, 260f, 100f), text, style);
        }
    }
}
