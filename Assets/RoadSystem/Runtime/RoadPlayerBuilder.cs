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
    /// <summary>建造模式：直路（两击一条）或 弯路（三击一条：直路+圆弧+直路）。</summary>
    public enum RoadBuildMode
    {
        /// <summary>直路：第一击落起点，第二击落终点即完成一条直路。</summary>
        Straight,
        /// <summary>弯路：第一击落起点，第二击建首段直路，第三击建弯路段（圆弧+后半段直路）。</summary>
        Curve
    }

    /// <summary>
    /// 运行时画路（Play 模式，Input System 输入），支持两种建造模式（可用 UI 按钮切换，默认直路）：
    ///  直路模式（两击）：① 左键落起点 → 移动鼠标实时预览 → ② 左键落终点，一条直路完成；
    ///  弯路模式（三击）：① 左键落起点 → ② 建首段直路 → ③ 建弯路段（圆弧 + 后半段直路），整条道路完成；
    ///  场景中名为 "Straight" / "Curve" 的 UI 按钮会在 Awake 时自动挂接模式切换（也可用 Inspector 的 buildMode 预设）；
    ///  右键 / Esc 取消当前未完成道路；切换模式同样会取消未完成道路。
    ///  右键 / Esc 取消当前未完成道路；
    ///  自动吸附既有悬空端（looseEndSnap 米）。半平面规则：直路阶段若起点为本工具新建，
    ///  允许在起点后方画直路（起点朝向自动转到鼠标方向）；吸附的既有端点与弯路阶段落在后方时
    ///  会被阻止。不可建造的状态一律显示红色虚影（几何无解时为起点到鼠标的红色直线提示）。
    ///  每条道路（三击建成的一组路段）共享一个道路分组 Id，场景中归到各自 road_xxxxxx 子 GameObject 下。
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

        [Header("预览颜色")]
        /// <summary>预览合法（可建）时的颜色，青色半透明。</summary>
        [SerializeField] Color previewOkColor = new Color(0.2f, 0.9f, 1f, 0.45f);
        /// <summary>预览非法（后方半平面或过短）时的警示颜色，红色半透明。</summary>
        [SerializeField] Color previewBadColor = new Color(1f, 0.35f, 0.2f, 0.45f);

        // ---- 运行时状态字段 ----

        /// <summary>同物体上的图宿主引用；所有图操作（PlaceProfile / AddSegment / Remove...）都通过它完成。</summary>
        RoadNetworkBehaviour net;
        /// <summary>当前待连起点 profile 的 Id（跨帧只存 Id 不存引用：宿主 RebuildAll 会用 JSON 整图替换对象，引用会失效）；null 表示尚未开始画路。</summary>
        string pendingStartId;
        /// <summary>标记起点是否由本工具新建；决定 CancelPending 时能否删除该 loose profile。</summary>
        bool pendingStartIsNew;
        /// <summary>弯路阶段标记：pendingStart 为首段直路末尾，下一次左键（第三击）将创建弯路段。</summary>
        bool buildingCurve;
        /// <summary>上一条已完成道路末尾 profile 的 Id；仅在下一条路的第一击吸附中被排除，避免新道路误接旧弯路末尾。</summary>
        string lastRoadEndId;
        /// <summary>当前正在建造的道路分组 Id：第一击建段时生成，同一条道路（第二击直路 + 第三击弯路）的路段共享；
        /// 道路完成 / 取消 / 新道路开始时清空。运行时渲染据此把每条道路归到各自的 GameObject 下。</summary>
        string currentRoadId;

        /// <summary>从活图解析当前起点 profile；Id 为空或已被删除时返回 null。</summary>
        Profile PendingStart => pendingStartId != null ? net.Graph.GetProfile(pendingStartId) : null;

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

        // ---- 生命周期 ----

        /// <summary>初始化：获取宿主与相机、解析地面层、创建预览对象。仅在脚本实例化时调用一次。</summary>
        void Awake()
        {
            net = GetComponent<RoadNetworkBehaviour>();
            if (cam == null) cam = Camera.main;
            if (groundMask == 0)
                groundMask = LayerMask.GetMask("Ground"); // 未配置时自动取 Ground 层
            EnsurePreviewObjects();
            WireModeButtons();
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

            // 鼠标悬停在 UI（如模式按钮）上时忽略本次点击，避免点按钮的同时误落路点
            bool overUi = EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();

            if (!overUi && ClickAction.WasPressedThisFrame()) OnLeftClick();
            if (!overUi && CancelAction != null && CancelAction.WasPressedThisFrame()) CancelPending();
            if (!overUi && DeleteAction != null && DeleteAction.WasPressedThisFrame()) DeleteLastSegment();

            if (pendingStartId != null) UpdatePreview();
#else
            // 未启用 Input System 时静默不工作（Project Settings → Active Input Handler）
#endif
        }

        // ---------------- 交互 ----------------

        /// <summary>左键放置逻辑（按模式分派）：
        /// 直路模式两击：① 落起点 profile（优先吸附既有悬空端）→ ② 建直路并完成整条道路；
        /// 弯路模式三击：① 落起点 → ② 建首段直路，pendingStart 移到直路末尾 → ③ 建弯路段并完成。
        /// 两端均写入地形高度 Y；直路阶段若起点为本工具新建且点击在后方，先转起点朝向（允许向后半平面画直路）。</summary>
        void OnLeftClick()
        {
            if (!GroundPoint(out DVec2 pos, out float groundY)) return;
            pos = SnapPos(pos);
            float roadY = groundY + roadYOffset;

            var start = PendingStart;
            if (start == null)
            {
                // 第一击：起点优先吸附既有悬空端（lastRoadEnd 已在下条路开始前排除）
                pendingStartId = null;
                var loose = FindLooseEnd(pos);
                if (loose != null)
                {
                    pendingStartId = loose.Id;
                    pendingStartIsNew = false;
                }
                else
                {
                    // y 在 PlaceProfile 内部序列化前写入，保证进入 JSON 快照（否则 RebuildAll 会把 Y 还原成 0）
                    var p = net.PlaceProfile(pos, DefaultDirection(), Profile.DefaultLanes(), roadY);
                    pendingStartId = p.Id;
                    pendingStartIsNew = true;
                }
                buildingCurve = false;
                lastRoadEndId = null; // 新路已开始，恢复对旧端点的正常吸附
                currentRoadId = null; // 新道路开始：清空上一条的分组 Id
            }
            else
            {
                // 第二击（首段直路）或第三击（弯路段）：从 start 建到当前点击处
                // 直路允许落在起点后方半平面：起点为本工具新建（未接任何路段）时，
                // 先把起点朝向转到弦向（与预览一致），路段即成纯直路；
                // 吸附的既有端点仍禁止后方创建（会与既有路段折返/重叠）
                if (!buildingCurve && pendingStartIsNew &&
                    DVec2.Distance(pos, start.Position) >= minSegmentLength &&
                    DVec2.Dot(pos - start.Position, start.Direction) < 0)
                {
                    net.MoveProfile(start, start.Position, (pos - start.Position).Normalized);
                }

                if (!TryBuildSegment(start, pos, roadY, out Profile end, out bool reused))
                    return;

                if (buildingCurve || buildMode == RoadBuildMode.Straight)
                {
                    // 第三击（弯路模式的弯路段）或第二击（直路模式的终点）：整条道路完成
                    CompleteRoad(end);
                }
                else
                {
                    // 第二击：首段直路已创建 → pendingStart 移到直路末尾，进入弯路阶段
                    pendingStartId = end.Id;
                    pendingStartIsNew = !reused;
                    buildingCurve = true;
                }
            }
            lastPreviewKey = null;
        }

        /// <summary>整条道路建造完成：复位全部状态并隐藏预览；
        /// 记住末尾 Id 使下一条路的第一击不吸附本条路的末尾。</summary>
        void CompleteRoad(Profile end)
        {
            lastRoadEndId = end.Id;
            pendingStartId = null;
            pendingStartIsNew = false;
            buildingCurve = false;
            currentRoadId = null; // 道路完成，下一条路属于新分组
            SetPreviewVisible(false);
        }

        /// <summary>从起点到点击处建一段路（直路段与弯路段共用）：
        /// 距离过短返回 false；否则按吸附/新建放置终点 profile 并 AddSegment。</summary>
        bool TryBuildSegment(Profile start, DVec2 pos, float roadY, out Profile end, out bool reused)
        {
            end = null;
            reused = false;
            if (DVec2.Distance(pos, start.Position) < minSegmentLength) return false;

            var loose = FindLooseEnd(pos);
            reused = loose != null && loose.Id != start.Id;
            if (reused)
            {
                end = loose;
            }
            else
            {
                // 弯路阶段：终点朝向按 1.5φ 规则，保证 Fillet 能解出 圆弧+后半段直路
                DVec2 dir = buildingCurve
                    ? ComputeCurveEndDirection(start, pos)
                    : ComputeEndDirection(start, pos);
                // y 随 PlaceProfile 一起提交（随后 AddSegment 会再次序列化，Y 不会丢）
                end = net.PlaceProfile(pos, dir, (LaneDef[])start.Lanes.Clone(), roadY);
            }

            // 提交前预检：几何解不出路径的位置直接拒绝，避免创建无网格的隐形路段
            var tmpA = Profile.Create(start.Position, start.Direction, start.Lanes);
            var tmpB = Profile.Create(end.Position, end.Direction, end.Lanes);
            if (ProfileConnector.Connect(tmpA, tmpB) == null)
            {
                Debug.LogWarning("[RoadSystem] 该位置无法求解路径，路段未创建（可尝试换个方向/位置）");
                if (!reused) net.RemoveLooseProfile(end); // 回滚刚新建的终点
                return false;
            }

            // 本条道路的第一次建段：生成道路分组 Id（后续段共用，归到同一 GameObject 下）
            if (currentRoadId == null) currentRoadId = System.Guid.NewGuid().ToString("N");
            if (net.AddSegment(start, end, currentRoadId) == null)
            {
                Debug.LogWarning("[RoadSystem] 路段创建失败");
                return false;
            }
            return true;
        }

        /// <summary>取消当前未完成道路：若起点为本工具新建则移除其 loose profile，并清空状态、隐藏预览。</summary>
        void CancelPending()
        {
            if (net == null) return; // 尚未初始化（不可能有未完成道路）
            var start = PendingStart;
            if (start != null && pendingStartIsNew)
                net.RemoveLooseProfile(start); // 仅从未接入任何段时生效
            pendingStartId = null;
            pendingStartIsNew = false;
            buildingCurve = false;
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

        /// <summary>弯路段（第三击）终点朝向：从起点朝向按弦向转角 φ 的 1.5 倍有符号旋转。
        /// 不能用弦向：弦向延续线恰好穿过起点，Fillet 交点与起点重合（cA=0）退化为纯直线；
        /// 圆弧(起点切向固定)+后半段直路 的可解范围是 ψ 在 φ 和 2φ 之间（含 2φ），取中值 1.5φ 两侧留等量余量。</summary>
        DVec2 ComputeCurveEndDirection(Profile start, DVec2 endPos)
        {
            DVec2 dA = start.Direction;
            DVec2 w = endPos - start.Position;
            if (w.Length < 0.1 || DVec2.Dot(w, dA) < 0)
                return dA; // 过近或后方半平面：同向兜底（红色警示）

            DVec2 chord = w.Normalized;
            double phi = System.Math.Atan2(DVec2.Cross(dA, chord), DVec2.Dot(dA, chord));
            double psi = phi * 1.5;
            double c = System.Math.Cos(psi), s = System.Math.Sin(psi);
            return new DVec2(dA.X * c - dA.Y * s, dA.X * s + dA.Y * c);
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
            if (start == null)
            {
                SetPreviewVisible(false);
                return;
            }
            if (!GroundPoint(out DVec2 endPos, out _))
            {
                SetPreviewVisible(false);
                return;
            }
            endPos = SnapPos(endPos);

            double dist = DVec2.Distance(endPos, start.Position);
            bool behind = DVec2.Dot(endPos - start.Position, start.Direction) < 0;

            // 直路阶段且起点为本工具新建（未接任何路段）：允许在起点后方画直路
            // —— 与提交一致：等效于把起点朝向转到弦向，两端口同向即成纯直路
            bool straightBehindOk = !buildingCurve && pendingStartIsNew && behind;
            bool bad = (behind && !straightBehindOk) || dist < minSegmentLength;

            string key = $"{endPos.X:F2},{endPos.Y:F2},{bad}";
            if (key == lastPreviewKey) return; // 帧合并：位置未变不重建
            lastPreviewKey = key;

            // 距起点过近（虚影与起点几乎重合，无提示意义）：隐藏
            if (dist < 0.1)
            {
                SetPreviewVisible(false);
                return;
            }

            DVec2 chordDir = (endPos - start.Position).Normalized;
            DVec2 startDir = straightBehindOk ? chordDir : start.Direction;
            DVec2 endDir = straightBehindOk ? chordDir
                : buildingCurve ? ComputeCurveEndDirection(start, endPos)
                : ComputeEndDirection(start, endPos);
            var tmpA = Profile.Create(start.Position, startDir, start.Lanes);
            tmpA.Y = start.Y;
            var tmpB = Profile.Create(endPos, endDir, start.Lanes);
            var path = ProfileConnector.Connect(tmpA, tmpB);

            // 几何无解：不可建造（bad）→ 红色直线虚影提示"此处无法建造"；
            // 合法状态却无解（理论罕见）→ 保守隐藏
            if (path == null && !bad)
            {
                SetPreviewVisible(false);
                return;
            }

            var mesh = SegmentMeshBuilder.Build(
                path ?? StraightHint(start.Position, endPos),
                start.Lanes, start.Position, 0.05f, 6f);
            // 预览抬到起点 Y 之上一点，防 Z-fighting
            previewGo.transform.position = new Vector3(
                (float)start.Position.X, start.Y + previewLift, (float)start.Position.Y);
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
            if (previewMf != null && previewMf.sharedMesh != null) Destroy(previewMf.sharedMesh);
            if (previewGo != null) Destroy(previewGo);
            if (matOk != null) Destroy(matOk);
            if (matBad != null) Destroy(matBad);
        }

        // ---------------- 工具 ----------------

        /// <summary>在吸附半径内查找最近的既有悬空端（排除当前起点自身与上一条路末尾），用于续接/复用。</summary>
        Profile FindLooseEnd(DVec2 pos)
        {
            if (looseEndSnap <= 0) return null;
            Profile best = null;
            double bestD = looseEndSnap;
            foreach (var p in net.Graph.LooseEndProfiles())
            {
                if (pendingStartId != null && p.Id == pendingStartId) continue;
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
