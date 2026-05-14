// Editor-only helper for assembling the GS-VR-Nav simulator demo scene.
// Coordinate convention:
// Python ENU: X = East, Y = North, Z = Up
// Unity: X = Right, Y = Up, Z = Forward
// Mapping: ENU(e, n, u) -> Unity(e, u, n)

using System.IO;
using GsVrNav.Unity;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.SceneManagement;

/// <summary>
/// Creates a baseline XR Device Simulator scene after the required Unity packages are installed.
/// </summary>
public static class GsVrNavDemoSceneBuilder
{
    private const string ScenePath = "Assets/Scenes/GS_VR_Nav_Demo.unity";

    /// <summary>
    /// Builds or rebuilds the MVP demo scene with XR simulator-friendly references.
    /// </summary>
    [MenuItem("GS-VR-Nav/Create Simulator Demo Scene")]
    public static void CreateSimulatorDemoScene()
    {
        Directory.CreateDirectory("Assets/Scenes");

        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        SceneManager.SetActiveScene(scene);

        GameObject xrOrigin = InstantiatePrefabByNames("XR Origin (Action-based)", "XR Origin (XR Rig)") ?? new GameObject("XR Origin (Action-based)");
        xrOrigin.name = "XR Origin";
        xrOrigin.transform.position = Vector3.zero;

        InstantiatePrefabByNames("XR Device Simulator");

        Camera headCamera = xrOrigin.GetComponentInChildren<Camera>(true);
        if (headCamera == null)
        {
            GameObject cameraObject = new GameObject("Main Camera");
            cameraObject.transform.SetParent(xrOrigin.transform, false);
            headCamera = cameraObject.AddComponent<Camera>();
        }

        headCamera.tag = "MainCamera";

        GameObject navMeshManager = new GameObject("NavMeshManager");
        GeoAlignmentLoader geoLoader = navMeshManager.AddComponent<GeoAlignmentLoader>();

        VRNavigationController navigationController = xrOrigin.GetComponent<VRNavigationController>();
        if (navigationController == null)
        {
            navigationController = xrOrigin.AddComponent<VRNavigationController>();
        }

        AssignSerializedReference(navigationController, "geoAlignmentLoader", geoLoader);
        AssignSerializedReference(navigationController, "xrOrigin", xrOrigin.transform);
        AssignSerializedReference(navigationController, "headCamera", headCamera);
        AssignMoveInputReference(navigationController);

        GameObject splatScene = new GameObject("GaussianSplatScene");
        splatScene.AddComponent<GaussianSplatSetup>();

        GameObject minimap = new GameObject("MinimapController");
        MinimapController minimapController = minimap.AddComponent<MinimapController>();
        AssignSerializedReference(minimapController, "playerTransform", xrOrigin.transform);

        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        Debug.Log($"GS-VR-Nav demo scene created at {ScenePath}. If moveInput is empty, assign XRI LeftHand Locomotion/Move manually.");
    }

    private static GameObject InstantiatePrefabByNames(params string[] prefabNames)
    {
        foreach (string prefabName in prefabNames)
        {
            GameObject instance = InstantiatePrefabByName(prefabName);
            if (instance != null)
            {
                return instance;
            }
        }

        Debug.LogWarning($"None of the requested prefabs were found: {string.Join(", ", prefabNames)}. Import XR Interaction Toolkit Starter Assets, then run this menu again if needed.");
        return null;
    }

    private static GameObject InstantiatePrefabByName(string prefabName)
    {
        string[] guids = AssetDatabase.FindAssets($"{prefabName} t:Prefab");
        foreach (string guid in guids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            if (prefab == null || prefab.name != prefabName)
            {
                continue;
            }

            GameObject instance = PrefabUtility.InstantiatePrefab(prefab) as GameObject;
            return instance;
        }

        return null;
    }

    private static void AssignMoveInputReference(VRNavigationController navigationController)
    {
        InputActionReference moveReference = FindXriMoveInputReference();
        if (moveReference != null)
        {
            AssignSerializedReference(navigationController, "moveInput", moveReference);
        }
    }

    private static InputActionReference FindXriMoveInputReference()
    {
        string[] guids = AssetDatabase.FindAssets("XRI Default Input Actions t:InputActionAsset");
        foreach (string guid in guids)
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            InputActionAsset asset = AssetDatabase.LoadAssetAtPath<InputActionAsset>(path);
            if (asset == null)
            {
                continue;
            }

            InputAction action =
                asset.FindAction("XRI LeftHand Locomotion/Move", false) ??
                asset.FindAction("Move", false);

            if (action == null)
            {
                continue;
            }

            string referencePath = "Assets/Settings/XRI_LeftHand_Move.asset";
            Directory.CreateDirectory("Assets/Settings");

            InputActionReference reference = AssetDatabase.LoadAssetAtPath<InputActionReference>(referencePath);
            if (reference == null)
            {
                reference = InputActionReference.Create(action);
                AssetDatabase.CreateAsset(reference, referencePath);
            }

            return reference;
        }

        Debug.LogWarning("XRI Default Input Actions asset was not found. Import XRI Starter Assets and assign moveInput manually.");
        return null;
    }

    private static void AssignSerializedReference(Object target, string propertyName, Object value)
    {
        SerializedObject serializedObject = new SerializedObject(target);
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property == null)
        {
            Debug.LogWarning($"Serialized property '{propertyName}' was not found on {target.name}.");
            return;
        }

        property.objectReferenceValue = value;
        serializedObject.ApplyModifiedPropertiesWithoutUndo();
        EditorUtility.SetDirty(target);
    }
}
