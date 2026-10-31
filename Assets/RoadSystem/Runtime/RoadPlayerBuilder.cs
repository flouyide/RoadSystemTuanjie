using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RoadSystem
{
    /// <summary>建造模式：直路两击，弯路按起点、虚拟转角、终点三击。</summary>
    public enum RoadBuildMode
    {
        /// <summary>直路：第一击落起点，第二击落终点即完成一条直路。</summary>
        Straight,
        /// <summary>弯路：第三击一次提交直线、圆弧、直线路径。</summary>
        Curve
    }

    /// <summary>
    /// 运行时画路（Play 模式，Input System 输入），支持两种建造模式（可用 UI 按钮切换，默认直路）：
    ///  直路模式（两击）：① 左键落起点 → 移动鼠标实时预览 → ② 左键落终点，一条直路完成；
    ///  弯路模式（三击）：① 起点 → ② 虚拟转角 → ③ 终点，整条道路一次提交；
    ///  场景中名为 "Straight" / "Curve" 的 UI 按钮会在 Awake 时自动挂接模式切换（也可用 Inspector 的 buildMode 预设）；
    ///  右键 / Esc 或切换模式取消当前未完成道路。
    ///  自动吸附既有悬空端（looseEndSnap 米）。不可建造时显示红色预览与原因。
    ///  每条完成的道路使用一个道路分组 Id，场景中归到各自 road_xxxxxx 子 GameObject 下。
    /// 挂在 RoadNetworkBehaviour 同一物体上即可；buildEnabled 可运行时开关。
    /// </summary>
    [RequireComponent(typeof(RoadNetworkBehaviour))]
    public class RoadPlayerBuilder : MonoBehaviour
    {
        const double RoadCenterSnapDistance = 6.0;
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
        /// <summary>当前建造模式：直路（两击一条）或弯路（三击一条）；UI 按钮 / 代码可随时切换。</summary>
        [SerializeField] RoadBuildMode buildMode = RoadBuildMode.Straight;
        /// <summary>吸附既有悬空端 profile 的半径（米）；≤0 时关闭吸附。</summary>
        [SerializeField, Tooltip("吸附既有悬空端的半径（米），0 关闭")] float looseEndSnap = 3f;
        /// <summary>位置网格吸附步长（米）；≤0.001 视为关闭。运行时无键盘事件，故无 Ctrl 临时禁用。</summary>
        [SerializeField, Tooltip("位置网格吸附步长（米），0 关闭")] float gridSnap = 0f;
        /// <summary>最小路段长度（米）；短于此的点击被忽略。</summary>
        [SerializeField, Tooltip("最小路段长度（米）")] float minSegmentLength = 1f;
        /// <summary>预览 mesh 的 Y 抬高量，防止与正式道路 Z-fighting。</summary>
        [SerializeField, Tooltip("预览网格抬高量，防 Z-fighting")] float previewLift = 0.03f;
        /// <summary>路口边缘形状：0 为当前圆角，1 为直边。</summary>
        [SerializeField, Range(0f, 1f), Tooltip("路口边缘弧度：0 为当前圆角，1 为直边")]
        public float IntersectionCurve = 0f;

        [Header("预览颜色")]
        /// <summary>预览合法（可建）时的颜色，青色半透明。</summary>
        [SerializeField] Color previewOkColor = new Color(0.2f, 0.9f, 1f, 0.45f);
        /// <summary>预览非法（后方半平面或过短）时的警示颜色，红色半透明。</summary>
        [SerializeField] Color previewBadColor = new Color(1f, 0.35f, 0.2f, 0.45f);

        // ---- 运行时状态字段 ----

        /// <summary>同物体上的图宿主引用；所有图操作（PlaceProfile / AddSegment / Remove...）都通过它完成。</summary>
        RoadNetworkBehaviour net;
        /// <summary>既有起点存 Id；新起点保留在草稿中，提交前不写入路网。</summary>
        string pendingStartId;
        Profile pendingDraft;
        string pendingIntersectionId, curveIntersectionId;
        RoadBuildPlan candidatePlan;
        /// <summary>新建草稿起点可以朝向终点，既有端口保留原切向。</summary>
        bool pendingStartIsNew;
        /// <summary>弯路点击阶段：0 起点，1 控制点，2 终点。前两击只记录位置。</summary>
        int curveStage;
        DVec2 curveStartPos, curveControlPos;
        float curveStartY;
        string curveStartId;
        /// <summary>上一条已完成道路末尾 profile 的 Id；仅在下一条路的第一击吸附中被排除，避免新道路误接旧弯路末尾。</summary>
        string lastRoadEndId;
        /// <summary>直路当前分组 Id；完成或取消时清空。</summary>
        string currentRoadId;

        /// <summary>从活图解析当前起点 profile；Id 为空或已被删除时返回 null。</summary>
        Profile PendingStart => pendingDraft ?? (pendingStartId != null ? net.Graph.GetProfile(pendingStartId) : null);

        // ---------------- 建造模式 ----------------

        /// <summary>当前建造模式（只读）。</summary>
        public RoadBuildMode BuildMode => buildMode;

        /// <summary>切换建造模式；若正有未完成道路则取消之，避免两种模式的段混在同一条路里。</summary>
        public void SetBuildMode(RoadBuildMode mode)
        {
            if (buildMode == mode) return;
            buildMode = mode;
            if (Application.isPlaying) CancelPending();
            Debug.Log($"[RoadSystem] 建造模式切换为：{(mode == RoadBuildMode.Straight ? "直路（两击）" : "弯路（三击）")}");
        }

        /// <summary>UI 按钮 "Straight" 的回调：切换到直路模式。</summary>
        public void SetStraightMode() => SetBuildMode(RoadBuildMode.Straight);

        /// <summary>UI 按钮 "Curve" 的回调：切换到弯路模式。</summary>
        public void SetCurveMode() => SetBuildMode(RoadBuildMode.Curve);

        /// <summary>Awake 时按名称兜底挂接模式切换按钮（"Straight" / "Curve"）：
        /// 仅当按钮尚未在 Inspector 里手动绑定回调（persistentEventCount == 0）时才补挂，
        /// 已手动配置的按钮以场景设置为准，不做重复挂接。</summary>
        void WireModeButtons()
        {
            HookModeButton("Straight", SetStraightMode);
            HookModeButton("Curve", SetCurveMode);
        }

        /// <summary>按 GameObject 名称查找 UI Button；若没有任何手动绑定的回调则自动补挂，否则跳过。</summary>
        static void HookModeButton(string buttonName, UnityEngine.Events.UnityAction action)
        {
            var go = GameObject.Find(buttonName);
            var btn = go != null ? go.GetComponent<Button>() : null;
            if (btn == null)
            {
                Debug.LogWarning($"[RoadSystem] 未找到名为 {buttonName} 的 UI 按钮，无法兜底挂接建造模式切换");
                return;
            }
            if (btn.onClick.GetPersistentEventCount() > 0) return; // 已手动配置：以场景为准
            btn.onClick.AddListener(action);
        }

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
        /// <summary>当前预览无法提交的原因；在 Game 视图提示。</summary>
        string previewFailureReason;
        GUIStyle previewWarningStyle;

        // ---- 生命周期 ----

        /// <summary>初始化：获取宿主与相机、解析地面层、创建预览对象。仅在脚本实例化时调用一次。</summary>
        void Awake()
        {
            net = GetComponent<RoadNetworkBehaviour>();
            SyncIntersectionCurve();
            if (cam == null) cam = Camera.main;
            if (groundMask == 0)
                groundMask = LayerMask.GetMask("Ground"); // 未配置时自动取 Ground 层
            EnsurePreviewObjects();
            WireModeButtons();
        }

        /// <summary>启用时开启所有 Input Action；若缺少必需的 Position / Click 引用则报错提示。</summary>
        void OnEnable()
        {
            SyncIntersectionCurve();
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
            previewFailureReason = null;
        }

        void OnGUI()
        {
            if (!Application.isPlaying || !buildEnabled || previewGo == null
                || !previewGo.activeSelf || string.IsNullOrEmpty(previewFailureReason)) return;
            if (previewWarningStyle == null)
            {
                previewWarningStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 20,
                    fontStyle = FontStyle.Bold
                };
                previewWarningStyle.normal.textColor = new Color(1f, 0.25f, 0.2f);
            }
            GUI.Label(new Rect(16f, 16f, 1000f, 36f),
                $"无法建造道路：{previewFailureReason}", previewWarningStyle);
        }

        /// <summary>每帧主循环：派发输入动作（放置 / 取消 / 删除），并在有待连起点时刷新预览。</summary>
        void Update()
        {
            SyncIntersectionCurve();
#if ENABLE_INPUT_SYSTEM
            if (!buildEnabled || cam == null || ClickAction == null) { SetPreviewVisible(false); return; }

            // 鼠标悬停在 UI（如模式按钮）上时忽略本次点击，避免点按钮的同时误落路点
            bool overUi = EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();

            if (!overUi && ClickAction.WasPressedThisFrame()) OnLeftClick();
            if (!overUi && CancelAction != null && CancelAction.WasPressedThisFrame()) CancelPending();
            if (!overUi && DeleteAction != null && DeleteAction.WasPressedThisFrame()) DeleteLastSegment();

            if (buildMode == RoadBuildMode.Curve && curveStage > 0) UpdateCurvePreview();
            else if (pendingStartId != null) UpdatePreview();
#else
            // 未启用 Input System 时静默不工作（Project Settings → Active Input Handler）
#endif
        }

        void OnValidate()
        {
            IntersectionCurve = Mathf.Clamp01(IntersectionCurve);
            SyncIntersectionCurve();
        }

        void SyncIntersectionCurve()
        {
            if (net == null) net = GetComponent<RoadNetworkBehaviour>();
            if (net != null) net.IntersectionCurve = IntersectionCurve;
        }

        // ---------------- 交互 ----------------

        /// <summary>直路两击、弯路三击；弯路的前三击由 OnCurveClick 管理。</summary>
        void OnLeftClick()
        {
            if (!GroundPoint(out var pos, out float groundY)) return;
            pos = SnapPos(pos);
            float roadY = groundY + roadYOffset;
            if (buildMode == RoadBuildMode.Curve) { OnCurveClick(pos, roadY); return; }
            var start = PendingStart;
            if (start == null)
            {
                var loose = ResolveEndpoint(ref pos, ref roadY, out pendingIntersectionId);
                pendingDraft = loose == null ? Profile.Create(pos, DefaultDirection(), Profile.DefaultLanes()) : null;
                if (pendingDraft != null) pendingDraft.Y = roadY;
                pendingStartId = (loose ?? pendingDraft).Id;
                pendingStartIsNew = loose == null;
                lastRoadEndId = null;
                currentRoadId = System.Guid.NewGuid().ToString("N");
            }
            else
            {
                if (!TryStraightCandidate(pos, roadY, out _, out _, out _, out var reason)
                    || !net.TryCommitRoadPlan(candidatePlan, out reason))
                {
                    previewFailureReason = reason;
                    Debug.LogWarning($"[RoadSystem] 无法建造道路：{reason}");
                    return;
                }
                CompleteRoad(net.Graph.GetProfile(candidatePlan.EndProfileId));
            }
            lastPreviewKey = null;
        }

        // Intersection pavement takes priority over nearby segment ports/centerlines.
        Profile ResolveEndpoint(ref DVec2 pos, ref float y, out string intersection, string excludeId = null)
        {
            if (net.TrySnapIntersection(pos, y, out intersection, out var anchor, out var height))
            { pos = anchor; y = height; return null; }
            var loose = FindLooseEnd(pos, excludeId);
            if (loose != null) { pos = loose.Position; y = loose.Y; return loose; }
            if (IntersectionBuilder.TrySnapToRoad(net.Graph, pos, RoadCenterSnapDistance, out var roadPoint)) pos = roadPoint;
            return null;
        }

        bool TryStraightCandidate(DVec2 pos, float y, out Profile start, out Profile end,
            out PathChain path, out string reason)
        {
            candidatePlan = null; end = null; path = null; reason = null;
            start = PendingStart;
            if (start == null) { reason = "起点已不存在"; return false; }
            var loose = ResolveEndpoint(ref pos, ref y, out var endIntersection, start.Id);
            var direction = (pos - start.Position).Normalized;
            if (DVec2.Distance(pos, start.Position) < minSegmentLength)
            { reason = "道路长度不足"; return false; }
            if (pendingStartIsNew)
            {
                var draft = Profile.Create(start.Position, direction, start.Lanes);
                draft.Id = start.Id; draft.Y = start.Y; start = draft;
            }
            end = loose ?? Profile.Create(pos, ComputeEndDirection(start, pos), (LaneDef[])start.Lanes.Clone());
            if (loose == null) end.Y = y;
            path = ProfileConnector.Connect(start, end);
            return net.TryPlanRoad(start, end, path, currentRoadId, pendingIntersectionId,
                endIntersection, out candidatePlan, out reason);
        }

        void OnCurveClick(DVec2 pos, float roadY)
        {
            if (curveStage == 0)
            {
                var loose = ResolveEndpoint(ref pos, ref roadY, out curveIntersectionId);
                curveStartId = loose != null ? loose.Id : null;
                curveStartPos = pos;
                curveStartY = roadY;
                currentRoadId = System.Guid.NewGuid().ToString("N");
                curveStage = 1;
                lastRoadEndId = null;
                lastPreviewKey = null;
                return;
            }
            if (curveStage == 1)
            {
                if (DVec2.Distance(curveStartPos, pos) < minSegmentLength) return;
                curveControlPos = pos;
                curveStage = 2;
                lastPreviewKey = null;
                return;
            }

            if (!TryCurveCandidate(pos, roadY, out var start, out var end,
                out var path, out var reason))
            {
                previewFailureReason = reason;
                Debug.LogWarning($"[RoadSystem] 无法建造道路：{reason}");
                return;
            }
            if (!net.TryCommitRoadPlan(candidatePlan, out reason))
            { previewFailureReason = reason; lastPreviewKey = null; return; }
            CompleteRoad(net.Graph.GetProfile(candidatePlan.EndProfileId));
            lastPreviewKey = null;
        }

        bool TryCurveCandidate(DVec2 pos, float roadY, out Profile start,
            out Profile end, out PathChain path, out string reason)
        {
            candidatePlan = null;
            start = curveStartId != null ? net.Graph.GetProfile(curveStartId) : null;
            end = null;
            path = null;
            reason = null;
            if (curveStartId != null && (start == null || !start.IsLooseEnd))
            {
                reason = "起点已不可连接";
                return false;
            }
            if (start == null)
            {
                start = Profile.Create(curveStartPos, DVec2.UnitX, Profile.DefaultLanes());
                start.Y = curveStartY;
            }

            end = ResolveEndpoint(ref pos, ref roadY, out var endIntersection, curveStartId);

            path = ControlPointCurve.Solve(start.Position, curveControlPos, pos,
                minSegmentLength, start.TotalWidth * 0.5 + 0.5, out reason);
            if (path == null) return false;
            const double tangentTolerance = 0.966; // 15 degrees
            if (curveStartId != null
                && DVec2.Dot(start.Direction, path.StartTangent) < tangentTolerance)
            {
                reason = "起点与已有道路切线不连续";
                return false;
            }
            if (end != null && DVec2.Dot(end.Direction, path.EndTangent) < tangentTolerance)
            {
                reason = "终点与已有道路切线不连续";
                return false;
            }
            if (end == null)
            {
                end = Profile.Create(pos, path.EndTangent, (LaneDef[])start.Lanes.Clone());
                end.Y = roadY;
            }
            if (curveStartId == null) start.Direction = path.StartTangent;
            return net.TryPlanRoad(start, end, path, currentRoadId, curveIntersectionId,
                endIntersection, out candidatePlan, out reason);
        }

        string PreviewKey(DVec2 cursor, float y) => $"{cursor.X:R},{cursor.Y:R},{y:R}:" +
            $"{net.Graph.GetHashCode()}:{net.Graph.Revision}:{net.BuildSettings.Sagitta:R}:" +
            $"{net.BuildSettings.CornerRadius:R}:{net.BuildSettings.Curve:R}:{curveStage}:" +
            $"{curveControlPos.X:R},{curveControlPos.Y:R}:{pendingStartId}:{curveStartId}:{pendingIntersectionId}:{curveIntersectionId}";

        void UpdateCurvePreview()
        {
            EnsurePreviewObjects();
            if (!GroundPoint(out var cursor, out float groundY)) { SetPreviewVisible(false); return; }
            cursor = SnapPos(cursor);
            string key = PreviewKey(cursor, groundY);
            if (key == lastPreviewKey) return;
            var start = curveStartId != null ? net.Graph.GetProfile(curveStartId) : null;
            if (curveStartId != null && start == null) { CancelPending(); return; }
            var origin = start != null ? start.Position : curveStartPos;
            var lanes = start != null ? start.Lanes : Profile.DefaultLanes();
            float y = start != null ? start.Y : curveStartY;
            PathChain path;
            bool valid;
            if (curveStage == 1)
            {
                candidatePlan = null;
                path = StraightHint(origin, cursor);
                valid = DVec2.Distance(origin, cursor) >= minSegmentLength;
                previewFailureReason = valid ? null : "控制点距离起点过近";
            }
            else valid = TryCurveCandidate(cursor, groundY + roadYOffset, out _, out _, out path, out previewFailureReason);
            DisplayCandidate(valid, path ?? StraightHint(origin, cursor), lanes, origin, y);
            lastPreviewKey = key;
        }

        void DisplayCandidate(bool valid, PathChain path, LaneDef[] lanes, DVec2 origin, float y)
        {
            if (valid && candidatePlan != null && net.ShowBuildPreview(candidatePlan, matOk, previewLift))
            { previewGo.SetActive(false); return; }
            if (valid && candidatePlan != null)
            { valid = false; previewFailureReason = "道路预览网格生成失败，请重新预览"; }
            net.ClearBuildPreview();
            var mesh = SegmentMeshBuilder.Build(path, lanes, origin, .05f, 6f);
            previewGo.transform.position = new Vector3((float)origin.X, y + previewLift, (float)origin.Y);
            var old = previewMf.sharedMesh;
            previewMf.sharedMesh = mesh;
            previewMr.sharedMaterial = valid ? matOk : matBad;
            previewGo.SetActive(true);
            if (old != null) Destroy(old);
        }

        void CompleteRoad(Profile end)
        {
            lastRoadEndId = end != null ? end.Id : null;
            pendingDraft = null;
            pendingIntersectionId = curveIntersectionId = null;
            pendingStartId = null;
            pendingStartIsNew = false;
            candidatePlan = null;
            curveStage = 0;
            curveStartId = null;
            currentRoadId = null; // 道路完成，下一条路属于新分组
            SetPreviewVisible(false);
        }

        /// <summary>丢弃未提交草稿并恢复被预览暂时隐藏的道路。</summary>
        void CancelPending()
        {
            if (net == null) return; // 尚未初始化（不可能有未完成道路）
            pendingDraft = null;
            pendingIntersectionId = curveIntersectionId = null;
            candidatePlan = null;
            pendingStartId = null;
            pendingStartIsNew = false;
            curveStage = 0;
            curveStartId = null;
            currentRoadId = null; // 取消后下一条路从新分组开始
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

        /// <summary>预览/提交共用的终点朝向规则（直路段）：后方半平面强制同向，否则取弦向。</summary>
        DVec2 ComputeEndDirection(Profile start, DVec2 endPos)
        {
            if (DVec2.Dot(endPos - start.Position, start.Direction) < 0)
                return start.Direction; // 半平面约束
            double dist = DVec2.Distance(endPos, start.Position);
            return dist > 0.1 ? (endPos - start.Position).Normalized
                              : start.Direction;
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
            var start = PendingStart;
            if (start == null || !GroundPoint(out var cursor, out float groundY))
            { SetPreviewVisible(false); previewFailureReason = null; return; }
            cursor = SnapPos(cursor);
            string key = PreviewKey(cursor, groundY);
            if (key == lastPreviewKey) return;
            bool valid = TryStraightCandidate(cursor, groundY + roadYOffset,
                out _, out _, out var path, out previewFailureReason);
            DisplayCandidate(valid, path ?? StraightHint(start.Position, cursor), start.Lanes, start.Position, start.Y);
            lastPreviewKey = key;
        }

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
            if (!v) { if (net != null) net.ClearBuildPreview(); lastPreviewKey = null; }
            if (previewGo != null && previewGo.activeSelf != v) previewGo.SetActive(v);
        }

        /// <summary>几何无解时的红色提示路径：起点到鼠标的简单直线，只用于"此处无法建造"的视觉提示。</summary>
        static PathChain StraightHint(DVec2 a, DVec2 b)
        {
            var chain = new PathChain();
            chain.Add(new LineElement(a, b));
            return chain;
        }

        /// <summary>销毁时释放预览 mesh / GameObject / 材质，防止运行时资源泄漏。</summary>
        void OnDestroy()
        {
            if (net != null) net.ClearBuildPreview();
            if (previewMf != null && previewMf.sharedMesh != null) Destroy(previewMf.sharedMesh);
            if (previewGo != null) Destroy(previewGo);
            if (matOk != null) Destroy(matOk);
            if (matBad != null) Destroy(matBad);
        }

        // ---------------- 工具 ----------------

        /// <summary>在吸附半径内查找最近的既有悬空端（排除当前起点自身与上一条路末尾），用于续接/复用。</summary>
        Profile FindLooseEnd(DVec2 pos, string excludeId = null)
        {
            if (looseEndSnap <= 0) return null;
            Profile best = null;
            double bestD = looseEndSnap;
            foreach (var p in net.Graph.LooseEndProfiles())
            {
                if (pendingStartId != null && p.Id == pendingStartId) continue;
                if (excludeId != null && p.Id == excludeId) continue;
                if (lastRoadEndId != null && p.Id == lastRoadEndId) continue; // 上一条路末尾：下一条路不误接
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
