using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace RoadSystem.EditorTools
{
    [CustomEditor(typeof(RoadNetworkBehaviour))]
    public class RoadNetworkInspector : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            var net = (RoadNetworkBehaviour)target;
            EditorGUILayout.Space();
            net.GetStats(out int profiles, out int segments, out int intersections);
            EditorGUILayout.LabelField("Profiles", profiles.ToString());
            EditorGUILayout.LabelField("路段 (Segments)", segments.ToString());
            EditorGUILayout.LabelField("路口 (Intersections)", intersections.ToString());

            EditorGUILayout.Space();
            if (GUILayout.Button("激活道路放置工具"))
            {
                Selection.activeGameObject = net.gameObject;
                ToolManager.SetActiveTool<RoadPlacementTool>();
            }
            if (GUILayout.Button("重建全部网格"))
                net.ForceRebuild();
            if (GUILayout.Button("清空路网"))
            {
                if (EditorUtility.DisplayDialog("清空路网", "确定删除全部道路？（可 Undo）", "删除", "取消"))
                {
                    Undo.RecordObject(net, "Clear Road Network");
                    net.Graph.Clear();
                    net.CommitEdit();
                }
            }
        }
    }
}
