using System.Collections.Generic;
using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEngine;

namespace RoadSystem
{
    /// <summary>
    /// 路网宿主：持有 RoadGraph（JSON 序列化进场景，天然支持 Undo/Redo），
    /// 订阅 Changed 事件做增量网格重建，DirtyVersion 帧合并。
    /// 子物体为纯派生数据（HideFlags.DontSave），由图数据随时重建。
    /// </summary>
    [ExecuteAlways]
    public class RoadNetworkBehaviour : MonoBehaviour
    {
        [SerializeField, HideInInspector] string graphJson = "";
        [SerializeField] Material roadMaterial;
        [SerializeField, Tooltip("弧段采样弦高误差（米），越小越圆滑")] float sagitta = 0.05f;
        [SerializeField, Tooltip("路面贴图沿道路方向的重复周期（米）")] float texturePeriod = 6f;

        public RoadGraph Graph { get; private set; }

        string appliedJson;
        bool graphSubscribed;
        readonly HashSet<string> dirtyNodes = new HashSet<string>();
        readonly Dictionary<string, GameObject> nodeObjects = new Dictionary<string, GameObject>();

        public Material RoadMaterial
        {
            get
            {
                if (roadMaterial == null)
                {
                    var shader = Shader.Find("Universal Render Pipeline/Lit");
                    if (shader != null)
                    {
                        roadMaterial = new Material(shader) { name = "M_RoadSystem_Default" };
                        roadMaterial.SetColor("_BaseColor", new Color(0.35f, 0.35f, 0.35f));
                    }
                }
                return roadMaterial;
            }
            set => roadMaterial = value;
        }

        // ---------------- 编辑 API（编辑器层调用；调用前须 Undo.RecordObject） ----------------

        public Profile PlaceProfile(DVec2 pos, DVec2 dir, LaneDef[] lanes = null)
        {
            EnsureGraph();
            var p = Graph.CreateProfile(pos, dir, lanes);
            CommitEdit();
            return p;
        }

        public RoadSegment AddSegment(Profile a, Profile b)
        {
            EnsureGraph();
            var seg = Graph.AddSegment(a, b);
            CommitEdit();
            return seg;
        }

        public void RemoveSegment(string segId)
        {
            EnsureGraph();
            Graph.RemoveSegment(segId);
            CommitEdit();
        }

        public void RemoveLooseProfile(Profile p)
        {
            EnsureGraph();
            Graph.RemoveLooseProfile(p);
            CommitEdit();
        }

        public void MoveProfile(Profile p, DVec2 pos, DVec2 dir)
        {
            EnsureGraph();
            Graph.MoveProfile(p, pos, dir);
            CommitEdit();
        }

        /// <summary>把图状态写入序列化字段（Undo 快照基于该字段）。</summary>
        public void CommitEdit()
        {
            graphJson = RoadGraphJson.ToJson(Graph);
#if UNITY_EDITOR
            if (!Application.isPlaying)
                UnityEditor.EditorUtility.SetDirty(this);
#endif
        }

        /// <summary>强制下次 Update 全量重建（Inspector 按钮用）。</summary>
        public void ForceRebuild() => appliedJson = null;

        // ---------------- 生命周期 ----------------

        void OnEnable()
        {
            EnsureGraph();
            appliedJson = null; // 强制全量重建
        }

        void OnDisable()
        {
            if (Graph != null && graphSubscribed)
            {
                Graph.Changed -= OnGraphChanged;
                graphSubscribed = false;
            }
        }

        void Update()
        {
            if (graphJson != appliedJson)
                RebuildAll();   // 覆盖 Undo/Redo：JSON 被还原后自动重建
            else
                FlushDirty();   // 帧合并：同一帧多次变动只重建一次
        }

        // ---------------- 内部 ----------------

        void EnsureGraph()
        {
            if (Graph == null)
                Graph = RoadGraphJson.FromJson(graphJson);
            if (!graphSubscribed)
            {
                Graph.Changed += OnGraphChanged;
                graphSubscribed = true;
            }
        }

        void OnGraphChanged(IReadOnlyList<string> ids)
        {
            foreach (var id in ids) dirtyNodes.Add(id);
        }

        void FlushDirty()
        {
            if (dirtyNodes.Count == 0) return;
            var ids = new List<string>(dirtyNodes);
            dirtyNodes.Clear();
            foreach (var id in ids) RebuildNode(id);
        }

        void RebuildAll()
        {
            appliedJson = graphJson;
            // 换图：反订阅旧图 → 反序列化 → 订阅新图
            if (Graph != null && graphSubscribed) Graph.Changed -= OnGraphChanged;
            Graph = RoadGraphJson.FromJson(graphJson);
            Graph.Changed += OnGraphChanged;
            graphSubscribed = true;

            foreach (var go in nodeObjects.Values) DestroyChild(go);
            nodeObjects.Clear();
            dirtyNodes.Clear();

            foreach (var seg in Graph.Segments()) RebuildNode(seg.Id);
        }

        void RebuildNode(string nodeId)
        {
            var node = Graph.GetNode(nodeId);
            if (node is RoadSegment seg)
                RebuildSegment(seg);
            // Intersection 网格生成属于 M4，当前版本跳过
        }

        void RebuildSegment(RoadSegment seg)
        {
            if (seg.PortIds.Count != 2) return;
            var pa = Graph.GetProfile(seg.PortIds[0]);
            var pb = Graph.GetProfile(seg.PortIds[1]);
            if (pa == null || pb == null) return;

            var path = ProfileConnector.Connect(pa, pb);
            if (path == null)
            {
                seg.PathSolveFailed = true;
                return;
            }
            seg.PathSolveFailed = false;
            seg.Path = path;

            var go = GetOrCreateNodeObject(seg.Id, out var mf, out var mr, out var mc);
            go.transform.position = new Vector3((float)pa.Position.X, 0f, (float)pa.Position.Y);

            var oldMesh = mf.sharedMesh;
            var mesh = SegmentMeshBuilder.Build(path, pa.Lanes, pa.Position, sagitta, texturePeriod);
            mesh.name = $"RoadSegment_{seg.Id.Substring(0, 6)}";
            mf.sharedMesh = mesh;
            mr.sharedMaterial = RoadMaterial;
            if (mc != null) mc.sharedMesh = mesh;
            if (oldMesh != null) DestroyChild(oldMesh);
        }

        GameObject GetOrCreateNodeObject(string nodeId,
            out MeshFilter mf, out MeshRenderer mr, out MeshCollider mc)
        {
            if (!nodeObjects.TryGetValue(nodeId, out var go) || go == null)
            {
                go = new GameObject($"seg_{nodeId.Substring(0, 6)}")
                {
                    hideFlags = HideFlags.DontSave // 派生数据不入场景
                };
                go.transform.SetParent(transform, false);
                mf = go.AddComponent<MeshFilter>();
                mr = go.AddComponent<MeshRenderer>();
                mc = go.AddComponent<MeshCollider>();
                nodeObjects[nodeId] = go;
            }
            else
            {
                mf = go.GetComponent<MeshFilter>();
                mr = go.GetComponent<MeshRenderer>();
                mc = go.GetComponent<MeshCollider>();
            }
            return go;
        }

        static void DestroyChild(Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) Destroy(o);
            else DestroyImmediate(o);
        }

        /// <summary>统计信息（Inspector 显示）。</summary>
        public void GetStats(out int profiles, out int segments, out int intersections)
        {
            EnsureGraph();
            profiles = Graph.Profiles.Count;
            segments = 0;
            intersections = 0;
            foreach (var n in Graph.Nodes.Values)
            {
                if (n is RoadSegment) segments++;
                else if (n is Intersection) intersections++;
            }
        }
    }
}
