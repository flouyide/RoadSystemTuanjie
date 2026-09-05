using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RoadSystem
{
    /// <summary>
    /// 运行时画路（Play 模式，Input System 输入）：
    ///  左键单击放置起点 → 移动鼠标实时半透明预览 → 再次左键确认建设，并自动连续延伸；
    ///  右键 / Esc 取消当前起点；
    ///  自动吸附既有悬空端（looseEndSnap 米），落在源后方半平面时强制同向（红色警示预览）。
    /// 挂在 RoadNetworkBehaviour 同一物体上即可；buildEnabled 可运行时开关。
    /// </summary>
    [RequireComponent(typeof(RoadNetworkBehaviour))]
    public class RoadPlayerBuilder : MonoBehaviour
    {
        // ---- Inspector 配置字段 ----

        [Header("输入/相机")]
        /// <summary>渲染相机，用于将鼠标屏幕坐标转成世界射线；未赋值则由 Awake 自动取 Camera.main。</summary>
        [SerializeField] Camera cam; // 留空自动取 Camera.main

        /// <summary>运行时画路总开关；为 false 时 Update 直接 return，可在运行时动态关闭画路功能。</summary>
        [SerializeField] public bool buildEnabled = true;

        [Header("地面检测")]
        /// <summary>地面射线检测的层掩码；Physics.Raycast 只命中该层物体以拾取地形高度。Awake 中若未赋值则自动取名为 "Ground" 的层。</summary>
        [Tooltip("射线检测的地面层（建议设为 Ground）。留空时 Awake 自动取名为 Ground 的层")]
        [SerializeField] LayerMask groundMask;

        /// <summary>道路 mesh 相对地面命中点的额外 Y 抬高量（整数米），防止与地面重叠产生 Z-fighting。</summary>
        [Tooltip("道路 mesh 相对地面命中点的 Y 抬高量，避免与 ground 重叠")]
        [SerializeField] int roadYOffset = 0;

#if ENABLE_INPUT_SYSTEM
        [Header("Input Action 引用（拖入 IA_RoadBuilder 资产里的 Action）")]
        /// <summary>鼠标位置动作引用（Vector2），每帧读取屏幕坐标用于地面拾取；对应 IA_RoadBuilder 的 Position。</summary>
        [Tooltip("鼠标位置（Value/Vector2），对应 IA_RoadBuilder 的 Position")]
        [SerializeField] InputActionReference positionActionRef;
        /// <summary>左键确认动作引用（Button），按下即放置一个路点/连段；对应 IA_RoadBuilder 的 Click。</summary>
        [Tooltip("左键确认（Button），对应 IA_RoadBuilder 的 Click")]
        [SerializeField] InputActionReference clickActionRef;
        /// <summary>取消当前待连起点动作引用（Button），可接右键或 Esc；对应 IA_RoadBuilder 的 Cancel。</summary>
        [Tooltip("取消起点（Button），可接 RightClick 或 Cancel（Esc）")]
        [SerializeField] InputActionReference cancelActionRef;
        /// <summary>删除最后一段动作引用（Button），可选；对应 IA_RoadBuilder 的 Delete。</summary>
        [Tooltip("删除最后一段（Button），对应 IA_RoadBuilder 的 Delete；可选")]
        [SerializeField] InputActionReference deleteActionRef;

        /// <summary>鼠标位置动作的便捷访问器，未配置引用时返回 null。</summary>
        InputAction PointAction => positionActionRef != null ? positionActionRef.action : null;
        /// <summary>左键确认动作的便捷访问器，未配置引用时返回 null。</summary>
        InputAction ClickAction => clickActionRef != null ? clickActionRef.action : null;
        /// <summary>取消动作的便捷访问器，未配置引用时返回 null。</summary>
        InputAction CancelAction => cancelActionRef != null ? cancelActionRef.action : null;
        /// <summary>删除动作的便捷访问器，未配置引用时返回 null。</summary>
        InputAction DeleteAction => deleteActionRef != null ? deleteActionRef.action : null;
#endif

        [Header("建造参数")]
        /// <summary>吸附既有悬空端 profile 的半径（米）；≤0 时关闭吸附。</summary>
        [SerializeField, Tooltip("吸附既有悬空端的半径（米），0 关闭")] float looseEndSnap = 3f;
        /// <summary>位置网格吸附步长（米）；≤0.001 视为关闭。运行时无键盘事件，故无 Ctrl 临时禁用。</summary>
        [SerializeField, Tooltip("位置网格吸附步长（米），0 关闭")] float gridSnap = 0f;
        /// <summary>最小路段长度（米）；短于此的点击被忽略。</summary>
        [SerializeField, Tooltip("最小路段长度（米）")] float minSegmentLength = 1f;
        /// <summary>预览 mesh 的 Y 抬高量，防止与正式道路 Z-fighting。</summary>
        [SerializeField, Tooltip("预览网格抬高量，防 Z-fighting")] float previewLift = 0.03f;

        [Header("预览颜色")]
        /// <summary>预览合法（可建）时的颜色，青色半透明。</summary>
        [SerializeField] Color previewOkColor = new Color(0.2f, 0.9f, 1f, 0.45f);
        /// <summary>预览非法（后方半平面或过短）时的警示颜色，红色半透明。</summary>
        [SerializeField] Color previewBadColor = new Color(1f, 0.35f, 0.2f, 0.45f);

        // ---- 运行时状态字段 ----

        /// <summary>同物体上的图宿主引用；所有图操作（PlaceProfile / AddSegment / Remove...）都通过它完成。</summary>
        RoadNetworkBehaviour net;
        /// <summary>当前待连接的起点 profile（已入图）；为 null 表示尚未开始画路。</summary>
        Profile pendingStart;
        /// <summary>标记起点是否由本工具新建；决定 CancelPending 时能否删除该 loose profile。</summary>
        bool pendingStartIsNew;

        /// <summary>预览路面物体的根 GameObject，挂载 MeshFilter / MeshRenderer。</summary>
        GameObject previewGo;
        /// <summary>预览物体的网格过滤器，承载临时生成的预览 mesh。</summary>
        MeshFilter previewMf;
        /// <summary>预览物体的网格渲染器，承载 ok / bad 材质。</summary>
        MeshRenderer previewMr;
        /// <summary>预览合法 / 非法两种状态的半透明材质，由 MakePreviewMaterial 创建。</summary>
        Material matOk, matBad;
        /// <summary>上帧预览的指纹（位置 + 状态），用于帧合并以避免每帧重建 mesh。</summary>
        string lastPreviewKey;

        // ---- 生命周期 ----

        /// <summary>初始化：获取宿主与相机、解析地面层、创建预览对象。仅在脚本实例化时调用一次。</summary>
        void Awake()
        {
            net = GetComponent<RoadNetworkBehaviour>();
            if (cam == null) cam = Camera.main;
            if (groundMask == 0)
                groundMask = LayerMask.GetMask("Ground"); // 未配置时自动取 Ground 层
            EnsurePreviewObjects();
        }

        /// <summary>启用时开启所有 Input Action；若缺少必需的 Position / Click 引用则报错提示。</summary>
        void OnEnable()
        {
#if ENABLE_INPUT_SYSTEM
            if (positionActionRef == null || clickActionRef == null)
                Debug.LogError("[RoadSystem] RoadPlayerBuilder 未配置 InputActionReference（Position/Click 至少需要），请在 Inspector 拖入 IA_RoadBuilder 的 Action 引用");
            PointAction?.Enable();
            ClickAction?.Enable();
            CancelAction?.Enable();
            DeleteAction?.Enable();
#endif
        }

        /// <summary>禁用时关闭所有 Input Action 并隐藏预览（不清除 pendingStart，便于重新启用后继续）。</summary>
        void OnDisable()
        {
#if ENABLE_INPUT_SYSTEM
            PointAction?.Disable();
            ClickAction?.Disable();
            CancelAction?.Disable();
            DeleteAction?.Disable();
#endif
            SetPreviewVisible(false);
        }

        /// <summary>每帧主循环：派发输入动作（放置 / 取消 / 删除），并在有待连起点时刷新预览。</summary>
        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            if (!buildEnabled || cam == null || ClickAction == null) return;

            if (ClickAction.WasPressedThisFrame()) OnLeftClick();
            if (CancelAction != null && CancelAction.WasPressedThisFrame()) CancelPending();
            if (DeleteAction != null && DeleteAction.WasPressedThisFrame()) DeleteLastSegment();

            if (pendingStart != null) UpdatePreview();
#else
            // 未启用 Input System 时静默不工作（Project Settings → Active Input Handler）
#endif
        }

        // ---------------- 交互 ----------------

        /// <summary>左键放置逻辑：首击落起点 profile，再次点击连段并链式延伸；两端均写入地形高度 Y。</summary>
        void OnLeftClick()
        {
            if (!GroundPoint(out DVec2 pos, out float groundY)) return;
            pos = SnapPos(pos);
            float roadY = groundY + roadYOffset;

            if (pendingStart == null)
            {
                // 起点：优先吸附既有悬空端
                var loose = FindLooseEnd(pos);
                if (loose != null)
                {
                    pendingStart = loose;
                    pendingStartIsNew = false;
                }
                else
                {
                    pendingStart = net.PlaceProfile(pos, DefaultDirection(), Profile.DefaultLanes());
                    pendingStart.Y = roadY;
                    pendingStartIsNew = true;
                }
            }
            else
            {
                if (DVec2.Distance(pos, pendingStart.Position) < minSegmentLength) return;

                var loose = FindLooseEnd(pos);
                bool reused = loose != null && loose.Id != pendingStart.Id;
                Profile end;
                if (reused)
                {
                    end = loose;
                }
                else
                {
                    DVec2 dir = ComputeEndDirection(pos);
                    end = net.PlaceProfile(pos, dir, (LaneDef[])pendingStart.Lanes.Clone());
                    end.Y = roadY;
                }

                var seg = net.AddSegment(pendingStart, end);
                if (seg == null)
                {
                    Debug.LogWarning("[RoadSystem] 路段创建失败");
                    return;
                }
                pendingStart = end;
                pendingStartIsNew = !reused;
            }
            lastPreviewKey = null;
        }

        /// <summary>取消当前待连起点：若起点为本工具新建则移除其 loose profile，并清空状态、隐藏预览。</summary>
        void CancelPending()
        {
            if (pendingStart != null && pendingStartIsNew)
                net.RemoveLooseProfile(pendingStart); // 仅从未接入任何段时生效
            pendingStart = null;
            pendingStartIsNew = false;
            SetPreviewVisible(false);
            lastPreviewKey = null;
        }

        /// <summary>删除最后一条路段（接 IA_RoadBuilder 的 Delete 动作）。</summary>
        void DeleteLastSegment()
        {
            string lastId = null;
            long lastVer = long.MinValue;
            foreach (var n in net.Graph.Nodes.Values)
                if (n is RoadSegment s && n.DirtyVersion > lastVer) { lastVer = n.DirtyVersion; lastId = n.Id; }
            if (lastId != null) net.RemoveSegment(lastId);
        }

        /// <summary>预览/提交共用的终点朝向规则：后方半平面强制同向，否则取弦向。</summary>
        DVec2 ComputeEndDirection(DVec2 endPos)
        {
            if (DVec2.Dot(endPos - pendingStart.Position, pendingStart.Direction) < 0)
                return pendingStart.Direction; // 半平面约束
            double dist = DVec2.Distance(endPos, pendingStart.Position);
            return dist > 0.1 ? (endPos - pendingStart.Position).Normalized
                              : pendingStart.Direction;
        }

        /// <summary>首点默认朝向：相机前向在 XZ 的投影，过于俯视时取 +Z。</summary>
        DVec2 DefaultDirection()
        {
            if (cam != null)
            {
                Vector3 f = cam.transform.forward;
                var d = new DVec2(f.x, f.z);
                if (d.Length > 0.3) return d.Normalized;
            }
            return DVec2.UnitY;
        }

        // ---------------- 预览 ----------------

        /// <summary>实时预览：用当前鼠标位置构造临时 profile，共享几何/网格管线生成青/红虚影路面。</summary>
        void UpdatePreview()
        {
            EnsurePreviewObjects();
            if (!GroundPoint(out DVec2 endPos, out _))
            {
                SetPreviewVisible(false);
                return;
            }
            endPos = SnapPos(endPos);

            double dist = DVec2.Distance(endPos, pendingStart.Position);
            bool behind = DVec2.Dot(endPos - pendingStart.Position, pendingStart.Direction) < 0;
            bool bad = behind || dist < minSegmentLength;

            string key = $"{endPos.X:F2},{endPos.Y:F2},{bad}";
            if (key == lastPreviewKey) return; // 帧合并：位置未变不重建
            lastPreviewKey = key;

            if (dist < minSegmentLength)
            {
                SetPreviewVisible(false);
                return;
            }

            DVec2 endDir = ComputeEndDirection(endPos);
            var tmpA = Profile.Create(pendingStart.Position, pendingStart.Direction, pendingStart.Lanes);
            tmpA.Y = pendingStart.Y;
            var tmpB = Profile.Create(endPos, endDir, pendingStart.Lanes);
            var path = ProfileConnector.Connect(tmpA, tmpB);
            if (path == null)
            {
                SetPreviewVisible(false);
                return;
            }

            var mesh = SegmentMeshBuilder.Build(path, pendingStart.Lanes, pendingStart.Position, 0.05f, 6f);
            // 预览抬到起点 Y 之上一点，防 Z-fighting
            previewGo.transform.position = new Vector3(
                (float)pendingStart.Position.X, pendingStart.Y + previewLift, (float)pendingStart.Position.Y);
            var old = previewMf.sharedMesh;
            previewMf.sharedMesh = mesh;
            previewMr.sharedMaterial = bad ? matBad : matOk;
            SetPreviewVisible(true);
            if (old != null) Destroy(old);
        }

        /// <summary>懒创建预览 GameObject 与 ok/bad 材质（仅首次），避免每帧重复分配。</summary>
        void EnsurePreviewObjects()
        {
            if (previewGo == null)
            {
                previewGo = new GameObject("~RoadPreview");
                previewMf = previewGo.AddComponent<MeshFilter>();
                previewMr = previewGo.AddComponent<MeshRenderer>();
                previewGo.SetActive(false);
            }
            if (matOk == null) matOk = MakePreviewMaterial(previewOkColor);
            if (matBad == null) matBad = MakePreviewMaterial(previewBadColor);
        }

        /// <summary>创建 URP Unlit 半透明预览材质，颜色由参数指定。</summary>
        static Material MakePreviewMaterial(Color c)
        {
            var shader = Shader.Find("Universal Render Pipeline/Unlit");
            var m = new Material(shader);
            m.SetFloat("_Surface", 1f); // Transparent
            m.SetFloat("_Blend", 0f);   // Alpha
            m.SetColor("_BaseColor", c);
            m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            return m;
        }

        /// <summary>切换预览物体显隐，比较 activeSelf 避免无谓 SetActive。</summary>
        void SetPreviewVisible(bool v)
        {
            if (previewGo != null && previewGo.activeSelf != v) previewGo.SetActive(v);
        }

        /// <summary>销毁时释放预览 mesh / GameObject / 材质，防止运行时资源泄漏。</summary>
        void OnDestroy()
        {
            if (previewMf != null && previewMf.sharedMesh != null) Destroy(previewMf.sharedMesh);
            if (previewGo != null) Destroy(previewGo);
            if (matOk != null) Destroy(matOk);
            if (matBad != null) Destroy(matBad);
        }

        // ---------------- 工具 ----------------

        /// <summary>在吸附半径内查找最近的既有悬空端（排除当前起点自身），用于续接/复用。</summary>
        Profile FindLooseEnd(DVec2 pos)
        {
            if (looseEndSnap <= 0) return null;
            Profile best = null;
            double bestD = looseEndSnap;
            foreach (var p in net.Graph.LooseEndProfiles())
            {
                if (pendingStart != null && p.Id == pendingStart.Id) continue;
                double d = DVec2.Distance(p.Position, pos);
                if (d < bestD) { bestD = d; best = p; }
            }
            return best;
        }

        /// <summary>按 gridSnap 步长对位置取整（网格吸附）；步长关闭时原样返回。</summary>
        DVec2 SnapPos(DVec2 p)
        {
            if (gridSnap <= 0.001f) return p;
            double s = gridSnap;
            return new DVec2(System.Math.Round(p.X / s) * s, System.Math.Round(p.Y / s) * s);
        }

        /// <summary>屏幕射线打地面层取世界点：命中写地形 Y，未命中/无层回退 y=0 平面。out y 为命中高度。</summary>
        bool GroundPoint(out DVec2 p, out float y)
        {
            p = DVec2.Zero;
            y = 0f;
#if ENABLE_INPUT_SYSTEM
            if (PointAction == null || cam == null) return false;
            Ray ray = cam.ScreenPointToRay(PointAction.ReadValue<Vector2>());
            // 射线打到 groundMask 层（默认 Ground）的第一个物体
            if (groundMask == 0 || !Physics.Raycast(ray, out var hit, 2000f, groundMask))
            {
                // 未配置地面层或没命中：回退到 y=0 平面
                if (Mathf.Abs(ray.direction.y) < 1e-6f) return false;
                float t = -ray.origin.y / ray.direction.y;
                if (t < 0) return false;
                Vector3 hit0 = ray.origin + ray.direction * t;
                p = new DVec2(hit0.x, hit0.z);
                y = 0f;
                return true;
            }
            p = new DVec2(hit.point.x, hit.point.z);
            y = hit.point.y;
            return true;
#else
            return false;
#endif
        }
    }
}
