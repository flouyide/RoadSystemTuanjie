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
        /// <summary>整张路网图的 JSON 序列化快照（存档/Undo 数据源）；[HideInInspector] 不暴露在 Inspector。</summary>
        [SerializeField, HideInInspector] string graphJson = "";
        /// <summary>道路渲染材质；为空时由 RoadMaterial 懒创建默认 URP Lit 灰材质。</summary>
        [SerializeField] Material roadMaterial;
        /// <summary>弧段采样弦高误差（米），越小弧线越圆滑、三角面越多。</summary>
        [SerializeField, Tooltip("弧段采样弦高误差（米），越小越圆滑")] float sagitta = 0.05f;
        /// <summary>路面贴图沿道路方向的重复周期（米），控制贴图拉伸密度。</summary>
        [SerializeField, Tooltip("路面贴图沿道路方向的重复周期（米）")] float texturePeriod = 6f;

        /// <summary>内存中的活图（权威数据模型）；只读对外暴露，编辑须经下方编辑 API。</summary>
        public RoadGraph Graph { get; private set; }

        /// <summary>已应用到 mesh 的 JSON 快照；与 graphJson 不同即表示图已变动、需重建。</summary>
        string appliedJson;
        /// <summary>是否已订阅 Graph.Changed 事件，防止重复订阅。</summary>
        bool graphSubscribed;
        /// <summary>脏节点 Id 集合（增量重建用）；HashSet 自动去重，实现帧合并。</summary>
        readonly HashSet<string> dirtyNodes = new HashSet<string>();
        /// <summary>节点 Id → 派生 GameObject 的映射；便于重建/销毁时定位对应 mesh 物体。</summary>
        readonly Dictionary<string, GameObject> nodeObjects = new Dictionary<string, GameObject>();

        /// <summary>道路材质访问器：未配置时懒创建默认 URP Lit 灰材质（仅编辑器且有对应 shader 时）。</summary>
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

        /// <summary>编辑 API：在图里新建一个端点 Profile（悬空端），写存档并返回该 profile。</summary>
        public Profile PlaceProfile(DVec2 pos, DVec2 dir, LaneDef[] lanes = null)
        {
            EnsureGraph();
            var p = Graph.CreateProfile(pos, dir, lanes);
            CommitEdit();
            return p;
        }

        /// <summary>编辑 API：用两个 profile 连成一条 RoadSegment（断面一致才成功），写存档并返回。</summary>
        public RoadSegment AddSegment(Profile a, Profile b)
        {
            EnsureGraph();
            var seg = Graph.AddSegment(a, b);
            CommitEdit();
            return seg;
        }

        /// <summary>编辑 API：按 Id 删除一条路段及其端口附件，写存档。</summary>
        public void RemoveSegment(string segId)
        {
            EnsureGraph();
            Graph.RemoveSegment(segId);
            CommitEdit();
        }

        /// <summary>编辑 API：删除一个悬空端 Profile（仅当它未接入任何节点时生效），写存档。</summary>
        public void RemoveLooseProfile(Profile p)
        {
            EnsureGraph();
            Graph.RemoveLooseProfile(p);
            CommitEdit();
        }

        /// <summary>编辑 API：移动一个 Profile 的位置与朝向，写存档。</summary>
        public void MoveProfile(Profile p, DVec2 pos, DVec2 dir)
        {
            EnsureGraph();
            Graph.MoveProfile(p, pos, dir);
            CommitEdit();
        }

        /// <summary>把图状态序列化写入 graphJson（Undo 快照基于此字段）；编辑器非运行态额外标记物体 dirty。</summary>
        public void CommitEdit()
        {
            graphJson = RoadGraphJson.ToJson(Graph);
#if UNITY_EDITOR
            if (!Application.isPlaying)
                UnityEditor.EditorUtility.SetDirty(this);
#endif
        }

        /// <summary>强制下次 Update 全量重建：把 appliedJson 置空，使 graphJson != appliedJson 成立。</summary>
        public void ForceRebuild() => appliedJson = null;

        // ---------------- 生命周期 ----------------

        /// <summary>启用时确保图已就绪，并强制下次 Update 全量重建（appliedJson 置空）。</summary>
        void OnEnable()
        {
            EnsureGraph();
            appliedJson = null; // 强制全量重建
        }

        /// <summary>禁用时解绑 Graph.Changed 事件，避免悬空引用/重复触发。</summary>
        void OnDisable()
        {
            if (Graph != null && graphSubscribed)
            {
                Graph.Changed -= OnGraphChanged;
                graphSubscribed = false;
            }
        }

        /// <summary>每帧重建调度：graphJson 变动（含 Undo/Redo 还原）则全量重建，否则增量刷新脏节点。</summary>
        void Update()
        {
            if (graphJson != appliedJson)
                RebuildAll();   // 覆盖 Undo/Redo：JSON 被还原后自动重建
            else
                FlushDirty();   // 帧合并：同一帧多次变动只重建一次
        }

        // ---------------- 内部 ----------------

        /// <summary>确保活图存在并订阅 Changed 事件：图为空则反序列化 graphJson，未订阅则挂 OnGraphChanged。</summary>
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

        /// <summary>Graph.Changed 事件处理器：把受影响节点 Id 收集进 dirtyNodes，供 FlushDirty 增量重建。</summary>
        void OnGraphChanged(IReadOnlyList<string> ids)
        {
            foreach (var id in ids) dirtyNodes.Add(id);
        }

        /// <summary>增量重建：把本帧累积的脏节点一次性重建 mesh 后清空集合（帧合并，避免同帧重复）。</summary>
        void FlushDirty()
        {
            if (dirtyNodes.Count == 0) return;
            var ids = new List<string>(dirtyNodes);
            dirtyNodes.Clear();
            foreach (var id in ids) RebuildNode(id);
        }

        /// <summary>全量重建：反订阅旧图→重新反序列化→订阅新图，销毁旧派生物体，重建所有路段（覆盖换图/Undo）。</summary>
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

        /// <summary>按节点 Id 分派重建：路段重建 mesh，路口（M4 功能）当前跳过。</summary>
        void RebuildNode(string nodeId)
        {
            var node = Graph.GetNode(nodeId);
            if (node is RoadSegment seg)
                RebuildSegment(seg);
            // Intersection 网格生成属于 M4，当前版本跳过
        }

        /// <summary>重建单个路段：取两端 profile→解路型→生成 mesh→挂到派生物体（含 MeshCollider），并释放旧 mesh。</summary>
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
            // 几何在 XZ 2D 计算；Y 仅用于渲染抬升（取起点 profile 的 Y，避免与地面重叠）
            go.transform.position = new Vector3((float)pa.Position.X, pa.Y, (float)pa.Position.Y);

            var oldMesh = mf.sharedMesh;
            var mesh = SegmentMeshBuilder.Build(path, pa.Lanes, pa.Position, sagitta, texturePeriod);
            mesh.name = $"RoadSegment_{seg.Id.Substring(0, 6)}";
            mf.sharedMesh = mesh;
            mr.sharedMaterial = RoadMaterial;
            if (mc != null) mc.sharedMesh = mesh;
            if (oldMesh != null) DestroyChild(oldMesh);
        }

        /// <summary>获取或创建某节点的派生 GameObject（带 MeshFilter/Renderer/Collider）；派生数据标记 DontSave 不入场景。</summary>
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

        /// <summary>安全销毁子物体/资源：运行态用 Destroy，编辑态用 DestroyImmediate（供 Undo 正确回滚）。</summary>
        static void DestroyChild(Object o)
        {
            if (o == null) return;
            if (Application.isPlaying) Destroy(o);
            else DestroyImmediate(o);
        }

        /// <summary>统计信息（Inspector 显示）：返回 profile / segment / intersection 数量。</summary>
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
