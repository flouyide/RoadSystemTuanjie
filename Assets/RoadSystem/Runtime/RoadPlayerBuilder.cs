using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using TMPro;
using System.Linq;
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
    ///  无路口的延伸结果为一个道路 GameObject；同组被路口拆开的多段使用 road_xxxxxx 分组。
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
        /// <summary>切换删除模式的可配置动作引用；对应 IA_RoadBuilder 的 Delete。</summary>
        [Tooltip("切换删除模式（Button），在 Input Action 中配置按键；对应 IA_RoadBuilder 的 Delete")]
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
        [SerializeField, Min(0), Tooltip("弯路第二击控制点辅助吸附距离；0 关闭辅助")]
        float controlPointSnap = 5f;
        [SerializeField, Min(1), Tooltip("控制点辅助搜索的目标端点范围（米）")]
        float controlTargetRange = 500f;
        string endpointHint, controlTargetId;
        DVec2? endpointMarker, controlMarker;
        float markerY;
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

        [Header("删除模式")]
        [SerializeField, Tooltip("删除拾取射线检测层；必须包含道路和遮挡物，防止穿透误删")]
        LayerMask deletionRaycastMask = Physics.DefaultRaycastLayers;
        [SerializeField] Color deleteHighlightColor = new Color(1f, .65f, .1f, .65f);
        public bool IsDeleteMode { get; private set; }
        string deleteTargetId, deleteReason;
        RoadDeletePlan deletePlan;
        RoadGraph deleteSource;
        long deleteRevision;
        RoadBuildSettings deleteSettings;
        Material matDelete;
        int suppressInputFrame = -1;

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
        /// <summary>上次提交后仍存活的可延伸外端；不排除连续续建。</summary>
        string lastRoadEndId;
        /// <summary>本次建造的候选分组 Id；延伸时由规划器决定继承或合并分组，结束后清空。</summary>
        string currentRoadId;

        /// <summary>从活图解析当前起点 profile；Id 为空或已被删除时返回 null。</summary>
        Profile PendingStart => pendingDraft ?? (pendingStartId != null ? net.Graph.GetProfile(pendingStartId) : null);

        // ---------------- 建造模式 ----------------

        /// <summary>当前建造模式（只读）。</summary>
        public RoadBuildMode BuildMode => buildMode;

        /// <summary>切换建造模式；若正有未完成道路则取消之，避免两种模式的段混在同一条路里。</summary>
        public void SetBuildMode(RoadBuildMode mode)
        {
            SetDeleteMode(false);
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

        /// <summary>禁用时关闭输入并丢弃未提交草稿。</summary>
        void OnDisable()
        {
#if ENABLE_INPUT_SYSTEM
            PointAction?.Disable();
            ClickAction?.Disable();
            CancelAction?.Disable();
            DeleteAction?.Disable();
#endif
            SetDeleteMode(false);
            SetPreviewVisible(false);
            CancelPending();
            previewFailureReason = null;
        }

        void OnGUI()
        {
            if (!Application.isPlaying || !buildEnabled) return;
            if (!IsDeleteMode)
            {
                if (endpointMarker.HasValue) DrawMarker(endpointMarker.Value, markerY, Color.cyan);
                if (controlMarker.HasValue)
                {
                    DrawMarker(controlMarker.Value, curveStartY, Color.yellow);
                    DrawGuide(curveStartPos, controlMarker.Value, curveStartY);
                    if (endpointMarker.HasValue) DrawGuide(controlMarker.Value, endpointMarker.Value, curveStartY);
                }
                if (!string.IsNullOrEmpty(endpointHint)) GUI.Label(new Rect(16, 60, 1000, 30), endpointHint);
            }
            if (!IsDeleteMode && (previewGo == null || !previewGo.activeSelf || string.IsNullOrEmpty(previewFailureReason))) return;
            if (previewWarningStyle == null) previewWarningStyle = new GUIStyle(GUI.skin.label)
                { fontSize = 20, fontStyle = FontStyle.Bold };
            bool warning = IsDeleteMode ? !string.IsNullOrEmpty(deleteReason) : true;
            previewWarningStyle.normal.textColor = warning ? new Color(1f, .25f, .2f) : deleteHighlightColor;
            string text = IsDeleteMode
                ? (warning ? (deleteTargetId != null ? "无法删除道路：" : "") + deleteReason
                    : "删除模式：悬停路段并单击左键删除；Esc / 右键退出")
                : "无法建造道路：" + previewFailureReason;
            GUI.Label(new Rect(16f, 16f, 1200f, 40f), text, previewWarningStyle);
        }

        void DrawMarker(DVec2 p, float y, Color color)
        {
            if (cam == null) return;
            var s = cam.WorldToScreenPoint(new Vector3((float)p.X, y + previewLift, (float)p.Y));
            if (s.z <= 0) return;
            var previous = GUI.color; GUI.color = color;
            GUI.DrawTexture(new Rect(s.x - 5, Screen.height - s.y - 5, 10, 10), Texture2D.whiteTexture);
            GUI.color = previous;
        }

        void DrawGuide(DVec2 a, DVec2 b, float y)
        {
            if (cam == null) return;
            var p = cam.WorldToScreenPoint(new Vector3((float)a.X, y + previewLift, (float)a.Y));
            var q = cam.WorldToScreenPoint(new Vector3((float)b.X, y + previewLift, (float)b.Y));
            if (p.z <= 0 || q.z <= 0) return;
            var start = new Vector2(p.x, Screen.height - p.y); var delta = new Vector2(q.x - p.x, p.y - q.y);
            var matrix = GUI.matrix; var color = GUI.color; GUI.color = Color.yellow;
            GUIUtility.RotateAroundPivot(Mathf.Atan2(delta.y, delta.x) * Mathf.Rad2Deg, start);
            GUI.DrawTexture(new Rect(start.x, start.y, delta.magnitude, 2), Texture2D.whiteTexture);
            GUI.matrix = matrix; GUI.color = color;
        }

        static bool TextInputFocused()
        {
            var selected = EventSystem.current != null ? EventSystem.current.currentSelectedGameObject : null;
            if (selected == null) return false;
            var legacy = selected.GetComponent<InputField>();
            var tmp = selected.GetComponent<TMP_InputField>();
            return (legacy != null && legacy.isFocused) || (tmp != null && tmp.isFocused);
        }

        public void SetDeleteMode(bool enabled)
        {
            enabled &= buildEnabled && isActiveAndEnabled;
            suppressInputFrame = Time.frameCount;
            if (enabled != IsDeleteMode) CancelPending();
            IsDeleteMode = enabled;
            ClearDeleteHover();
        }

        public void ToggleDeleteMode() => SetDeleteMode(!IsDeleteMode);

        void Update()
        {
            SyncIntersectionCurve();
#if ENABLE_INPUT_SYSTEM
            if (!buildEnabled || cam == null || ClickAction == null)
            { if (IsDeleteMode) SetDeleteMode(false); CancelPending(); ClearDeleteHover(); SetPreviewVisible(false); return; }
            if (suppressInputFrame == Time.frameCount) return;
            bool overUi = EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();
            if (TextInputFocused()) { ClearDeleteHover(); SetPreviewVisible(false); return; }
            bool cancel = (CancelAction != null && CancelAction.WasPressedThisFrame())
                || (Keyboard.current != null && Keyboard.current.escapeKey.wasPressedThisFrame)
                || (Mouse.current != null && Mouse.current.rightButton.wasPressedThisFrame);
            if (cancel)
            { if (IsDeleteMode) SetDeleteMode(false); else CancelPending(); return; }
            if (!overUi && DeleteAction != null && DeleteAction.WasPressedThisFrame())
            { ToggleDeleteMode(); return; }
            if (overUi) { ClearDeleteHover(); SetPreviewVisible(false); return; }
            if (IsDeleteMode)
            {
                UpdateDeleteHover();
                if (ClickAction.WasPressedThisFrame() && deletePlan != null)
                {
                    if (net.TryCommitDeletePlan(deletePlan, out var reason)) ClearDeleteHover();
                    else { deleteReason = reason; deletePlan = null; net.ClearBuildPreview(); }
                    return;
                }
                DrawDeleteHighlight();
                return;
            }
            if (ClickAction.WasPressedThisFrame()) OnLeftClick();
            if (buildMode == RoadBuildMode.Curve && curveStage > 0) UpdateCurvePreview();
            else if (pendingStartId != null) UpdatePreview();
            else if (GroundPoint(out var hover, out float hoverY))
            { hoverY += roadYOffset; ResolveEndpoint(ref hover, ref hoverY, out _); }
#endif
        }

        void ClearDeleteHover()
        {
            if (deleteTargetId != null && net != null) net.ClearBuildPreview();
            deleteTargetId = null; deletePlan = null; deleteSource = null; deleteReason = null;
        }

        void UpdateDeleteHover()
        {
#if ENABLE_INPUT_SYSTEM
            if (PointAction == null) { ClearDeleteHover(); return; }
            var ray = cam.ScreenPointToRay(PointAction.ReadValue<Vector2>());
            // Use the first physical hit including occluders. Never raycast through an intersection.
            if (!Physics.Raycast(ray, out var hit, 2000, deletionRaycastMask, QueryTriggerInteraction.Ignore))
            { ClearDeleteHover(); return; }
            if (!net.TryGetSegment(hit.collider, out var id, out var pickReason))
            { ClearDeleteHover(); deleteReason = pickReason; return; }
            if (id == deleteTargetId && ReferenceEquals(deleteSource, net.Graph)
                && deleteRevision == net.Graph.Revision && deleteSettings.Equals(net.BuildSettings)) return;
            ClearDeleteHover();
            deleteTargetId = id; deleteSource = net.Graph; deleteRevision = net.Graph.Revision;
            deleteSettings = net.BuildSettings;
            if (net.TryPlanDelete(id, out deletePlan, out deleteReason))
            {
                EnsurePreviewObjects();
                if (!net.ShowBuildPreview(deletePlan.Edit, matOk, previewLift))
                { deletePlan = null; deleteReason = "删除预览无法生成"; }
            }
#endif
        }

        void DrawDeleteHighlight()
        {
            if (deleteTargetId == null || !net.TryGetNodeVisual(deleteTargetId, out var mesh, out var matrix)) return;
            EnsurePreviewObjects();
            if (matDelete == null) matDelete = MakePreviewMaterial(deleteHighlightColor);
            var material = deletePlan != null ? matDelete : matBad;
            for (int sub = 0; sub < mesh.subMeshCount; sub++)
                Graphics.DrawMesh(mesh, Matrix4x4.Translate(Vector3.up * previewLift) * matrix,
                    material, 0, cam, sub, null, UnityEngine.Rendering.ShadowCastingMode.Off, false);
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
            endpointHint = null; endpointMarker = null;
            if (net.TrySnapIntersection(pos, y, out intersection, out var anchor, out var height))
            { pos = anchor; y = height; endpointHint = "连接路口"; endpointMarker = pos; markerY = y; return null; }
            var loose = FindLooseEnd(pos, excludeId);
            if (loose != null)
            { pos = loose.Position; y = loose.Y; endpointHint = "延伸道路"; endpointMarker = pos; markerY = y; return loose; }
            if (IntersectionBuilder.TrySnapToRoad(net.Graph, pos, RoadCenterSnapDistance, out var roadPoint))
            { pos = roadPoint; endpointHint = "新建路口"; endpointMarker = pos; markerY = y; }
            else pos = SnapPos(pos);
            return null;
        }

        bool TryStraightCandidate(DVec2 pos, float y, out Profile start, out Profile end,
            out PathChain path, out string reason)
        {
            candidatePlan = null; end = null; path = null; reason = null;
            start = PendingStart;
            if (start == null) { reason = "起点已不存在"; return false; }
            if (!pendingStartIsNew)
            {
                start = EndpointExtension.View(net.Graph, start.Id, true);
                if (start == null) { reason = "起点已不可延伸"; return false; }
            }
            var loose = ResolveEndpoint(ref pos, ref y, out var endIntersection, start.Id);
            var target = loose != null ? EndpointExtension.View(net.Graph, loose.Id, false) : null;
            if (loose != null && target == null) { reason = "目标道路路径已失效"; return false; }
            var direction = (pos - start.Position).Normalized;
            if (DVec2.Distance(pos, start.Position) < minSegmentLength)
            { reason = "道路长度不足"; return false; }
            if (pendingStartIsNew)
            {
                var draft = Profile.Create(start.Position, direction, target != null ? target.Lanes : start.Lanes);
                draft.Id = start.Id; draft.Y = start.Y; start = draft;
            }
            end = target ?? Profile.Create(pos, ComputeEndDirection(start, pos), (LaneDef[])start.Lanes.Clone());
            if (loose == null) end.Y = y;
            if (!pendingStartIsNew || target != null)
            {
                path = EndpointExtension.ConnectAutomatic(start, end, !pendingStartIsNew, target != null);
                if (path == null) { reason = "没有足够空间连接剩余道路"; return false; }
                if (pendingStartIsNew) start.Direction = path.StartTangent;
                if (target == null) end.Direction = path.EndTangent;
            }
            else path = ProfileConnector.Connect(start, end);
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
                if (!TryControlPoint(pos, out var control, out previewFailureReason)) return;
                curveControlPos = control;
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
            start = curveStartId != null ? EndpointExtension.View(net.Graph, curveStartId, true) : null;
            controlMarker = curveControlPos;
            end = null;
            path = null;
            reason = null;
            if (curveStartId != null && start == null)
            {
                reason = "起点已不可连接";
                return false;
            }
            if (start == null)
            {
                start = Profile.Create(curveStartPos, DVec2.UnitX, Profile.DefaultLanes());
                start.Y = curveStartY;
            }

            var target = ResolveEndpoint(ref pos, ref roadY, out var endIntersection, curveStartId);
            end = target != null ? EndpointExtension.View(net.Graph, target.Id, false) : null;
            if (target != null && end == null) { reason = "目标道路路径已失效"; return false; }
            if (curveStartId == null && end != null) start.Lanes = (LaneDef[])end.Lanes.Clone();

            path = ControlPointCurve.Solve(start.Position, curveControlPos, pos,
                minSegmentLength, start.TotalWidth * 0.5 + 0.5, out reason);
            if (path == null) return false;
            const double tangentTolerance = EndpointExtension.TangentDot;
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

        bool TryControlPoint(DVec2 cursor, out DVec2 control, out string reason)
        {
            reason = null; controlTargetId = null; endpointMarker = null; endpointHint = null;
            var start = curveStartId != null ? EndpointExtension.View(net.Graph, curveStartId, true) : null;
            control = SnapPos(cursor);
            if (curveStartId != null && start == null) { reason = "起点已不可延伸"; return false; }
            if (start != null) control = start.Position + start.Direction * DVec2.Dot(cursor - start.Position, start.Direction);
            double best = controlPointSnap;
            if (controlPointSnap > 0 && looseEndSnap > 0)
                foreach (var p in net.Graph.Profiles.Values.OrderBy(p => p.Id, System.StringComparer.Ordinal))
                {
                    if (p.Id == curveStartId || DVec2.Distance(p.Position, curveStartPos) > controlTargetRange
                        || System.Math.Abs(p.Y - curveStartY) > 1e-4
                        || EndpointExtension.SameChain(net.Graph, curveStartId, p.Id)) continue;
                    var end = EndpointExtension.View(net.Graph, p.Id, false);
                    if (end == null || (start != null && !EndpointExtension.Compatible(start, end))) continue;
                    if (!EndpointExtension.TryControl(curveStartPos, start?.Direction, end, cursor,
                        minSegmentLength, (start ?? end).TotalWidth * .5 + .5, out var candidate)) continue;
                    double distance = DVec2.Distance(cursor, candidate);
                    if (distance >= best) continue;
                    best = distance; control = candidate; controlTargetId = p.Id;
                    endpointMarker = p.Position; markerY = p.Y;
                }
            controlMarker = control;
            if (controlTargetId != null) endpointHint = "控制点已吸附目标切线交点；第三击选择标记端点";
            if (DVec2.Distance(curveStartPos, control) < minSegmentLength
                || (start != null && DVec2.Dot(control - start.Position, start.Direction) < minSegmentLength))
            { reason = "控制点须位于起点前方且距离足够"; return false; }
            return true;
        }

        string PreviewKey(DVec2 cursor, float y) => $"{cursor.X:R},{cursor.Y:R},{y:R}:" +
            $"{net.Graph.GetHashCode()}:{net.Graph.Revision}:{net.BuildSettings.Sagitta:R}:" +
            $"{net.BuildSettings.CornerRadius:R}:{net.BuildSettings.Curve:R}:{curveStage}:" +
            $"{curveControlPos.X:R},{curveControlPos.Y:R}:{pendingStartId}:{curveStartId}:{pendingIntersectionId}:{curveIntersectionId}:" +
            $"{looseEndSnap:R}:{gridSnap:R}:{minSegmentLength:R}:{controlPointSnap:R}:{controlTargetRange:R}:{buildMode}:{roadYOffset}:{previewLift:R}";

        void UpdateCurvePreview()
        {
            EnsurePreviewObjects();
            if (!GroundPoint(out var cursor, out float groundY)) { SetPreviewVisible(false); return; }
            string key = PreviewKey(cursor, groundY);
            if (key == lastPreviewKey) return;
            var start = curveStartId != null ? EndpointExtension.View(net.Graph, curveStartId, true) : null;
            if (curveStartId != null && start == null) { CancelPending(); return; }
            var origin = start != null ? start.Position : curveStartPos;
            var lanes = start != null ? start.Lanes : Profile.DefaultLanes();
            float y = start != null ? start.Y : curveStartY;
            PathChain path;
            bool valid;
            if (curveStage == 1)
            {
                candidatePlan = null;
                valid = TryControlPoint(cursor, out var control, out previewFailureReason);
                path = StraightHint(origin, control);
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
            if (!v)
            {
                if (net != null) net.ClearBuildPreview(); lastPreviewKey = null;
                endpointMarker = controlMarker = null; endpointHint = controlTargetId = null;
            }
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
            ClearDeleteHover();
            if (matDelete != null) Destroy(matDelete);
            if (net != null) net.ClearBuildPreview();
            if (previewMf != null && previewMf.sharedMesh != null) Destroy(previewMf.sharedMesh);
            if (previewGo != null) Destroy(previewGo);
            if (matOk != null) Destroy(matOk);
            if (matBad != null) Destroy(matBad);
        }

        // ---------------- 工具 ----------------

        /// <summary>在吸附半径内查找有效单段端点，允许连续延伸上一条道路。</summary>
        Profile FindLooseEnd(DVec2 pos, string excludeId = null)
        {
            if (looseEndSnap <= 0) return null;
            Profile best = null;
            double bestD = looseEndSnap;
            foreach (var p in net.Graph.Profiles.Values.OrderBy(p => p.Id, System.StringComparer.Ordinal))
            {
                if (!EndpointExtension.TryGetRoad(net.Graph, p.Id, out _)) continue;
                if (pendingStartId != null && p.Id == pendingStartId) continue;
                if (excludeId != null && p.Id == excludeId) continue;
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
