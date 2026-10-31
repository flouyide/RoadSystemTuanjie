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
        /// <summary>路口专用材质；编辑器中未指定时自动引用 M_Intersection.mat。</summary>
        [SerializeField] Material intersectionMaterial;
        /// <summary>弧段采样弦高误差（米），越小弧线越圆滑、三角面越多。</summary>
        [SerializeField, Tooltip("弧段采样弦高误差（米），越小越圆滑")] float sagitta = 0.05f;
        /// <summary>路口转角期望圆角半径（米）；空间不足时自动缩小。</summary>
        [SerializeField, Min(0f), Tooltip("路口转角半径（米）；空间不足时自动缩小")]
        float intersectionCornerRadius = 5f;
        /// <summary>路口边缘形状：0 为现有圆角，1 为直边；由 RoadPlayerBuilder 配置。</summary>
        float intersectionCurve;
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
        /// <summary>道路分组 Id → 分组 GameObject（路段 mesh 的父物体）；一条道路（一次建造流程）一个组。</summary>
        readonly Dictionary<string, GameObject> roadObjects = new Dictionary<string, GameObject>();
        bool warnedMissingIntersectionMaterial;
        GameObject buildPreview;
        readonly Dictionary<Renderer, bool> previewHidden = new Dictionary<Renderer, bool>();
        RoadBuildPlan displayedPlan;

        public RoadBuildSettings BuildSettings => new RoadBuildSettings {
            Sagitta = System.Math.Max(.001, sagitta), CornerRadius = intersectionCornerRadius, Curve = intersectionCurve };

        public bool TrySnapIntersection(DVec2 point, float y, out string id, out DVec2 position, out float height)
        {
            EnsureGraph();
            return RoadBuildPlanner.TrySnapIntersection(Graph, point, y, BuildSettings, out id, out position, out height);
        }

        public bool TryPlanRoad(Profile start, Profile end, PathChain path, string roadId,
            string startIntersection, string endIntersection, out RoadBuildPlan plan, out string reason)
        {
            EnsureGraph();
            return RoadBuildPlanner.TryPlan(Graph, start, end, path, roadId, startIntersection,
                endIntersection, BuildSettings, out plan, out reason);
        }

        sealed class PreparedNode
        {
            public Mesh Mesh;
            public Vector3 Position;
            public string RoadId;
            public bool Intersection;
        }

        bool PrepareMeshes(RoadBuildPlan plan, out Dictionary<string, PreparedNode> meshes, out string reason)
        {
            meshes = new Dictionary<string, PreparedNode>();
            reason = null;
            try
            {
                foreach (var id in plan.ChangedNodes)
                {
                    var node = plan.Result.GetNode(id);
                    if (node == null) continue;
                    var item = new PreparedNode();
                    if (node is RoadSegment segment)
                    {
                        var p = plan.Result.GetProfile(segment.PortIds[0]);
                        var path = SegmentPathResolver.Resolve(plan.Result, segment);
                        item.Mesh = SegmentMeshBuilder.Build(path, p.Lanes, p.Position, sagitta, texturePeriod);
                        item.Position = new Vector3((float)p.Position.X, p.Y, (float)p.Position.Y);
                        item.RoadId = segment.RoadId;
                    }
                    else if (node is Intersection ix)
                    {
                        var ports = IntersectionBuilder.LivePorts(plan.Result, ix);
                        var center = ix.HasAnchor ? ix.Anchor : IntersectionBuilder.CenterOf(ports);
                        float y = ix.HasAnchor ? ix.AnchorY : ports[0].Y;
                        item.Mesh = IntersectionMeshBuilder.Build(plan.Boundaries[ix.Id], center, y, vPeriod: texturePeriod);
                        item.Position = new Vector3((float)center.X, y, (float)center.Y);
                        item.Intersection = true;
                    }
                    meshes[id] = item;
                    if (item.Mesh == null || item.Mesh.vertexCount == 0)
                        throw new System.InvalidOperationException("道路或路口网格生成失败");
                }
                return true;
            }
            catch (System.Exception e)
            {
                foreach (var item in meshes.Values) if (item.Mesh != null) DestroyChild(item.Mesh);
                meshes.Clear();
                reason = e.Message;
                return false;
            }
        }

        /// <summary>All meshes and serialization are prepared before replacing the live graph.</summary>
        public bool TryCommitRoadPlan(RoadBuildPlan plan, out string reason)
        {
            reason = null;
            if (plan == null || !plan.IsCurrent(Graph, BuildSettings))
            { reason = "路网或建造参数已变化，请重新预览"; return false; }
            string json = RoadGraphJson.ToJson(plan.Result);
            if (!PrepareMeshes(plan, out var meshes, out reason)) return false;
            ClearBuildPreview();
            if (graphSubscribed) Graph.Changed -= OnGraphChanged;
            Graph = plan.Result;
            Graph.Changed += OnGraphChanged;
            graphSubscribed = true;
            graphJson = appliedJson = json;
            dirtyNodes.Clear();
            foreach (var id in plan.ChangedNodes)
            {
                if (!meshes.TryGetValue(id, out var item))
                {
                    if (nodeObjects.TryGetValue(id, out var dead)) DestroyChild(dead);
                    nodeObjects.Remove(id);
                    continue;
                }
                var go = GetOrCreateNodeObject(id, item.RoadId, out var mf, out var mr, out var mc);
                if (item.Intersection) go.name = $"Inter_{id}";
                go.transform.position = item.Position;
                var old = mf.sharedMesh;
                mf.sharedMesh = item.Mesh;
                mr.sharedMaterial = item.Intersection ? IntersectionMaterial : RoadMaterial;
                if (mc != null) mc.sharedMesh = item.Mesh;
                if (old != null) DestroyChild(old);
            }
#if UNITY_EDITOR
            if (!Application.isPlaying) UnityEditor.EditorUtility.SetDirty(this);
#endif
            return true;
        }

        public bool ShowBuildPreview(RoadBuildPlan plan, Material material, float lift = .03f)
        {
            if (plan == displayedPlan && buildPreview != null) return true;
            ClearBuildPreview();
            if (plan == null || !plan.IsCurrent(Graph, BuildSettings)
                || !PrepareMeshes(plan, out var meshes, out _)) return false;
            var combines = new List<CombineInstance>();
            foreach (var item in meshes.Values)
                for (int sub = 0; sub < item.Mesh.subMeshCount; sub++)
                    combines.Add(new CombineInstance { mesh = item.Mesh, subMeshIndex = sub,
                        transform = Matrix4x4.Translate(item.Position + Vector3.up * lift) });
            var combined = new Mesh { name = "RoadBuildPreview", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            combined.CombineMeshes(combines.ToArray(), true, true);
            foreach (var item in meshes.Values) DestroyChild(item.Mesh);
            buildPreview = new GameObject("~RoadBuildPreview") { hideFlags = HideFlags.HideAndDontSave };
            buildPreview.AddComponent<MeshFilter>().sharedMesh = combined;
            buildPreview.AddComponent<MeshRenderer>().sharedMaterial = material;
            foreach (var id in plan.ChangedNodes)
                if (nodeObjects.TryGetValue(id, out var go) && go != null)
                {
                    var renderer = go.GetComponent<Renderer>();
                    if (renderer == null) continue;
                    previewHidden[renderer] = renderer.enabled;
                    renderer.enabled = false;
                }
            displayedPlan = plan;
            return true;
        }

        public void ClearBuildPreview()
        {
            foreach (var entry in previewHidden) if (entry.Key != null) entry.Key.enabled = entry.Value;
            previewHidden.Clear();
            if (buildPreview != null) DestroyChild(buildPreview);
            buildPreview = null;
            displayedPlan = null;
        }

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

        public Material IntersectionMaterial
        {
            get
            {
#if UNITY_EDITOR
                if (intersectionMaterial == null)
                    AssignDefaultIntersectionMaterial();
#endif
                if (intersectionMaterial == null && !warnedMissingIntersectionMaterial)
                {
                    Debug.LogWarning("[RoadSystem] 未指定 M_Intersection 路口材质，暂用道路材质显示。", this);
                    warnedMissingIntersectionMaterial = true;
                }
                return intersectionMaterial != null ? intersectionMaterial : RoadMaterial;
            }
            set
            {
                intersectionMaterial = value;
                warnedMissingIntersectionMaterial = false;
                ForceRebuild();
            }
        }

        /// <summary>路口边缘从现有圆角到直边的插值；变动时重建已有路口。</summary>
        public float IntersectionCurve
        {
            get => intersectionCurve;
            set
            {
                float clamped = Mathf.Clamp01(value);
                if (Mathf.Approximately(intersectionCurve, clamped)) return;
                intersectionCurve = clamped;
                ForceRebuild();
            }
        }

#if UNITY_EDITOR
        const string IntersectionMaterialPath = "Assets/RoadSystem/Materials/M_Intersection.mat";

        void OnValidate()
        {
            ForceRebuild();
            if (intersectionMaterial == null)
                UnityEditor.EditorApplication.delayCall += AssignDefaultIntersectionMaterial;
        }

        void AssignDefaultIntersectionMaterial()
        {
            if (this == null || intersectionMaterial != null) return;
            var material = UnityEditor.AssetDatabase.LoadAssetAtPath<Material>(IntersectionMaterialPath);
            if (material == null) return;
            intersectionMaterial = material;
            warnedMissingIntersectionMaterial = false;
            UnityEditor.EditorUtility.SetDirty(this);
            ForceRebuild();
        }
#endif

        // ---------------- 编辑 API（编辑器层调用；调用前须 Undo.RecordObject） ----------------

        /// <summary>编辑 API：在图里新建一个端点 Profile（悬空端），y 须随本调用写入（内部会立即序列化存档，事后再改 Y 会被下次整图重建还原成旧值），写存档并返回该 profile。</summary>
        public Profile PlaceProfile(DVec2 pos, DVec2 dir, LaneDef[] lanes = null, float y = 0f)
        {
            EnsureGraph();
            var p = Graph.CreateProfile(pos, dir, lanes);
            p.Y = y;
            CommitEdit();
            return p;
        }

        /// <summary>编辑 API：用两个 profile 连成一条 RoadSegment（断面一致才成功），roadId 为所属道路分组 Id，写存档并返回。</summary>
        public RoadSegment AddSegment(Profile a, Profile b, string roadId = null)
        {
            EnsureGraph();
            var seg = Graph.AddSegment(a, b, roadId);
            CommitEdit();
            return seg;
        }

        /// <summary>Commit a controlled curve as one graph edit after its path has passed preflight.</summary>
        public RoadSegment AddControlledCurve(Profile existingStart, DVec2 startPos, float startY,
            Profile existingEnd, DVec2 endPos, float endY, LaneDef[] lanes,
            PathChain path, string roadId)
        {
            EnsureGraph();
            if (path == null || path.Elements.Count == 0) return null;
            var start = existingStart ?? Graph.CreateProfile(startPos, path.StartTangent, lanes);
            if (existingStart == null) start.Y = startY;
            var end = existingEnd ?? Graph.CreateProfile(endPos, path.EndTangent, lanes);
            if (existingEnd == null) end.Y = endY;
            var segment = Graph.AddSegment(start, end, roadId);
            if (segment == null)
            {
                if (existingStart == null) Graph.RemoveLooseProfile(start);
                if (existingEnd == null) Graph.RemoveLooseProfile(end);
                return null;
            }
            segment.Path = path;
            segment.ExplicitPath = SegmentPathResolver.Store(path);
            CommitEdit();
            return segment;
        }

        /// <summary>编辑 API：按 Id 删除一条路段及其端口附件；若其端口挂着的路口因此退化（&lt;3 口），
        /// 自动合并回普通路段。写存档。</summary>
        public void RemoveSegment(string segId)
        {
            EnsureGraph();
            // 记录该段 ports 挂的路口（删段后可能退化）
            var ixIds = new List<string>();
            if (Graph.GetNode(segId) is RoadSegment seg)
            {
                foreach (var pid in seg.PortIds)
                {
                    var p = Graph.GetProfile(pid);
                    if (p == null) continue;
                    foreach (var nid in new[] { p.NodeAId, p.NodeBId })
                        if (!string.IsNullOrEmpty(nid)
                            && Graph.GetNode(nid) is Intersection && !ixIds.Contains(nid))
                            ixIds.Add(nid);
                }
            }
            Graph.RemoveSegment(segId);
            foreach (var ixId in ixIds) IntersectionBuilder.MergeDegenerate(Graph, ixId);
            CommitEdit();
        }

        /// <summary>M4：检测新段与全图的交汇并自动生成路口（拆分/截断/建 Intersection/车道配对）。
        /// 图有变更时返回 true 并写存档；无交汇返回 false。</summary>
        public bool AutoBuildIntersections(string newSegId)
        {
            EnsureGraph();
            if (IntersectionBuilder.DetectAndBuild(Graph, newSegId))
            {
                CommitEdit();
                return true;
            }
            return false;
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
#if UNITY_EDITOR
            if (intersectionMaterial == null)
                UnityEditor.EditorApplication.delayCall += AssignDefaultIntersectionMaterial;
#endif
        }

        /// <summary>禁用时解绑 Graph.Changed 事件，避免悬空引用/重复触发。</summary>
        void OnDisable()
        {
            ClearBuildPreview();
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
            ClearBuildPreview();
            appliedJson = graphJson;
            // 换图：反订阅旧图 → 反序列化 → 订阅新图
            if (Graph != null && graphSubscribed) Graph.Changed -= OnGraphChanged;
            Graph = RoadGraphJson.FromJson(graphJson);
            Graph.Changed += OnGraphChanged;
            graphSubscribed = true;

            foreach (var go in nodeObjects.Values) DestroyChild(go);
            nodeObjects.Clear();
            foreach (var go in roadObjects.Values) DestroyChild(go); // 道路分组父物体（子段已先销毁）
            roadObjects.Clear();
            dirtyNodes.Clear();

            foreach (var n in Graph.Nodes.Values) RebuildNode(n.Id); // 段 + 路口（M4）
        }

        /// <summary>按节点 Id 分派重建：路段重建 mesh、路口重建铺装面（M4）；
        /// 节点已被删除则销毁其派生物体（拆分/删除后的清理）。</summary>
        void RebuildNode(string nodeId)
        {
            var node = Graph.GetNode(nodeId);
            if (node == null)
            {
                if (nodeObjects.TryGetValue(nodeId, out var dead) && dead != null)
                {
                    DestroyChild(dead);
                    nodeObjects.Remove(nodeId);
                }
                return;
            }
            if (node is RoadSegment seg)
                RebuildSegment(seg);
            else if (node is Intersection ix)
                RebuildIntersection(ix);
        }

        /// <summary>重建单个路段：取两端 profile→解路型→生成 mesh→挂到派生物体（含 MeshCollider），并释放旧 mesh。</summary>
        void RebuildSegment(RoadSegment seg)
        {
            if (seg.PortIds.Count != 2) return;
            var pa = Graph.GetProfile(seg.PortIds[0]);
            var pb = Graph.GetProfile(seg.PortIds[1]);
            if (pa == null || pb == null) return;

            seg.Path = null;
            var path = SegmentPathResolver.Resolve(Graph, seg);
            if (path == null)
            {
                seg.PathSolveFailed = true;
                return;
            }
            seg.PathSolveFailed = false;
            seg.Path = path;

            var go = GetOrCreateNodeObject(seg.Id, seg.RoadId, out var mf, out var mr, out var mc);
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

        /// <summary>重建单个路口（M4）：收集接通 ports → 极角序边界环（转角 fillet）→ 耳切三角化铺装 mesh；
        /// 挂点取路口中心与平均 port 高度，路口物体直接挂 RoadNetwork 根（不归道路分组）。</summary>
        void RebuildIntersection(Intersection ix)
        {
            var ports = IntersectionBuilder.LivePorts(Graph, ix);
            if (ports.Count < 3) return; // 退化（待合并清理），不生成网格
            IntersectionBuilder.RefreshLanePaths(Graph, ix);

            var center = ix.HasAnchor ? ix.Anchor : IntersectionBuilder.CenterOf(ports);
            var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, sagitta,
                intersectionCornerRadius, intersectionCurve);
            if (loop == null || loop.Count < 3) return;

            double avgY = 0;
            foreach (var p in ports) avgY += p.Y;
            avgY /= ports.Count;

            var mesh = IntersectionMeshBuilder.Build(loop, center, (float)avgY, vPeriod: texturePeriod);
            if (mesh == null) return;
            var go = GetOrCreateNodeObject(ix.Id, null, out var mf, out var mr, out var mc);
            go.name = $"Inter_{ix.Id}";
            go.transform.position = new Vector3((float)center.X, (float)avgY, (float)center.Y);

            var oldMesh = mf.sharedMesh;
            mesh.name = $"Intersection_{ix.Id.Substring(0, 6)}";
            mf.sharedMesh = mesh;
            mr.sharedMaterial = IntersectionMaterial;
            if (mc != null) mc.sharedMesh = mesh;
            if (oldMesh != null) DestroyChild(oldMesh);
        }

        /// <summary>获取或创建某节点的派生 GameObject（带 MeshFilter/Renderer/Collider）；
        /// roadId 非空时挂到对应道路分组物体下（一条道路一个父 GameObject），否则直接挂 RoadNetwork 根；
        /// 派生数据标记 DontSave 不入场景。</summary>
        GameObject GetOrCreateNodeObject(string nodeId, string roadId,
            out MeshFilter mf, out MeshRenderer mr, out MeshCollider mc)
        {
            if (!nodeObjects.TryGetValue(nodeId, out var go) || go == null)
            {
                go = new GameObject($"seg_{nodeId.Substring(0, 6)}")
                {
                    hideFlags = HideFlags.DontSave // 派生数据不入场景
                };
                var parent = !string.IsNullOrEmpty(roadId) ? GetOrCreateRoadObject(roadId).transform : transform;
                go.transform.SetParent(parent, false);
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

        /// <summary>获取或创建道路分组 GameObject（road_{id 前 6 位}），作为该道路所有路段 mesh 的父物体。</summary>
        GameObject GetOrCreateRoadObject(string roadId)
        {
            if (!roadObjects.TryGetValue(roadId, out var go) || go == null)
            {
                go = new GameObject($"road_{roadId.Substring(0, 6)}")
                {
                    hideFlags = HideFlags.DontSave // 派生数据不入场景
                };
                go.transform.SetParent(transform, false);
                roadObjects[roadId] = go;
            }
            return go;
        }

        /// <summary>安全销毁子物体/资源：运行态用 Destroy，编辑态用 DestroyImmediate（供 Undo 正确回滚）。</summary>
        static void DestroyChild(Object o)
        {
            if (o == null) return;
            if (o is GameObject go)
            {
                go.SetActive(false);
                var filter = go.GetComponent<MeshFilter>();
                if (filter != null && filter.sharedMesh != null) DestroyChild(filter.sharedMesh);
            }
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
