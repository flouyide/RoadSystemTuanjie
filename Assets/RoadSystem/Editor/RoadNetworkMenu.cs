using RoadSystem.Core;
using UnityEditor;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RoadSystem.EditorTools
{
    public static class RoadNetworkMenu
    {
        const string MaterialDir = "Assets/RoadSystem/Materials";
        const string MaterialPath = MaterialDir + "/M_RoadSystem.mat";
        const string TexturePath = "Assets/RoadSystem/Textures/RoadLaneAtlas.png";
        const string InputAssetPath = "Assets/Input/IA_RoadBuilder.inputactions";
        const string InputMapName = "RoadBuilder";

        [MenuItem("GameObject/Road System/道路网络 (Road Network)", false, 10)]
        static void CreateNetwork(MenuCommand cmd)
        {
            var go = new GameObject("RoadNetwork");
            GameObjectUtility.SetParentAndAlign(go, cmd.context as GameObject);
            var net = go.AddComponent<RoadNetworkBehaviour>();
            net.RoadMaterial = GetOrCreateDefaultMaterial();
            var builder = go.AddComponent<RoadPlayerBuilder>(); // 运行时画路（Play 模式，Input System）
#if ENABLE_INPUT_SYSTEM
            AssignActionRefs(builder);
#endif
            Undo.RegisterCreatedObjectUndo(go, "Create Road Network");
            Selection.activeGameObject = go;
        }

#if ENABLE_INPUT_SYSTEM
        /// <summary>（重新）生成 IA_RoadBuilder 各 Action 的 InputActionReference 资产。</summary>
        [MenuItem("Tools/Road System/生成 IA_RoadBuilder Action 引用", false, 100)]
        static void GenerateActionRefs()
        {
            GetOrCreateActionRef("Position");
            GetOrCreateActionRef("Click");
            GetOrCreateActionRef("Cancel");
            GetOrCreateActionRef("Delete");
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            Debug.Log("[RoadSystem] 已生成/刷新 IA_RoadBuilder 的 Action 引用（Assets/Input/ 下）");
        }

        /// <summary>把 Position/Click/Cancel/Delete 四个引用指派到 builder（用 SerializedObject 写私有字段）。</summary>
        public static void AssignActionRefs(RoadPlayerBuilder builder)
        {
            var so = new SerializedObject(builder);
            so.FindProperty("positionActionRef").objectReferenceValue = GetOrCreateActionRef("Position");
            so.FindProperty("clickActionRef").objectReferenceValue = GetOrCreateActionRef("Click");
            so.FindProperty("cancelActionRef").objectReferenceValue = GetOrCreateActionRef("Cancel");
            so.FindProperty("deleteActionRef").objectReferenceValue = GetOrCreateActionRef("Delete");
            so.ApplyModifiedPropertiesWithoutUndo();
        }

        static InputActionReference GetOrCreateActionRef(string actionName)
        {
            string refPath = $"Assets/Input/IA_RoadBuilder_{actionName}.asset";
            var existing = AssetDatabase.LoadAssetAtPath<InputActionReference>(refPath);
            if (existing != null) return existing;

            var iaa = AssetDatabase.LoadAssetAtPath<InputActionAsset>(InputAssetPath);
            if (iaa == null)
            {
                Debug.LogWarning($"[RoadSystem] 未找到输入资产 {InputAssetPath}");
                return null;
            }
            var action = iaa.FindAction($"{InputMapName}/{actionName}");
            if (action == null)
            {
                Debug.LogWarning($"[RoadSystem] {InputAssetPath} 中未找到 Action '{InputMapName}/{actionName}'");
                return null;
            }
            var r = InputActionReference.Create(action);
            r.name = $"IA_RoadBuilder_{actionName}";
            AssetDatabase.CreateAsset(r, refPath);
            return r;
        }
#endif

        /// <summary>一键示例：直线 + 90° 转弯 + S 形平移，验证几何与渲染通路。</summary>
        [MenuItem("GameObject/Road System/示例路网 (Demo)", false, 11)]
        static void CreateDemo(MenuCommand cmd)
        {
            CreateNetwork(cmd);
            var net = Selection.activeGameObject.GetComponent<RoadNetworkBehaviour>();

            Undo.RecordObject(net, "Build Demo Roads");
            var fwd = new DVec2(0, 1);  // +Z
            var east = new DVec2(1, 0); // +X

            var p0 = net.PlaceProfile(new DVec2(0, 0), fwd);
            var p1 = net.PlaceProfile(new DVec2(0, 30), fwd);
            net.AddSegment(p0, p1);                    // 30m 直线

            var p2 = net.PlaceProfile(new DVec2(30, 40), east);
            net.AddSegment(p1, p2);                    // 90° 转弯（弧 r=10 + 直线 20m）

            var p3 = net.PlaceProfile(new DVec2(60, 48), east);
            net.AddSegment(p2, p3);                    // S 形平移（横向 8m）

            Debug.Log("[RoadSystem] 示例路网已创建：直线 + 90°转弯 + S形平移");
        }

        public static Material GetOrCreateDefaultMaterial()
        {
            var mat = AssetDatabase.LoadAssetAtPath<Material>(MaterialPath);
            if (mat != null) return mat;

            if (!AssetDatabase.IsValidFolder("Assets/RoadSystem"))
                AssetDatabase.CreateFolder("Assets", "RoadSystem");
            if (!AssetDatabase.IsValidFolder(MaterialDir))
                AssetDatabase.CreateFolder("Assets/RoadSystem", "Materials");

            var shader = Shader.Find("Universal Render Pipeline/Lit");
            mat = new Material(shader) { name = "M_RoadSystem" };
            var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(TexturePath);
            if (tex != null) mat.SetTexture("_BaseMap", tex);
            mat.SetFloat("_Smoothness", 0.25f);
            AssetDatabase.CreateAsset(mat, MaterialPath);
            AssetDatabase.SaveAssets();
            return mat;
        }
    }
}
