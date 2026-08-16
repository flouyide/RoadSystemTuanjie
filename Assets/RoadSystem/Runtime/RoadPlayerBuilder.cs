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
        [Header("输入/相机")]
        [SerializeField] Camera cam; // 留空自动取 Camera.main
        [SerializeField] public bool buildEnabled = true;

#if ENABLE_INPUT_SYSTEM
        [Header("Input Action 引用（拖入 IA_RoadBuilder 资产里的 Action）")]
        [Tooltip("鼠标位置（Value/Vector2），对应 IA_RoadBuilder 的 Position")]
        [SerializeField] InputActionReference positionActionRef;
        [Tooltip("左键确认（Button），对应 IA_RoadBuilder 的 Click")]
        [SerializeField] InputActionReference clickActionRef;
        [Tooltip("取消起点（Button），可接 RightClick 或 Cancel（Esc）")]
        [SerializeField] InputActionReference cancelActionRef;
        [Tooltip("删除最后一段（Button），对应 IA_RoadBuilder 的 Delete；可选")]
        [SerializeField] InputActionReference deleteActionRef;

        InputAction PointAction => positionActionRef != null ? positionActionRef.action : null;
        InputAction ClickAction => clickActionRef != null ? clickActionRef.action : null;
        InputAction CancelAction => cancelActionRef != null ? cancelActionRef.action : null;
        InputAction DeleteAction => deleteActionRef != null ? deleteActionRef.action : null;
#endif

        [Header("建造参数")]
        [SerializeField, Tooltip("吸附既有悬空端的半径（米），0 关闭")] float looseEndSnap = 3f;
        [SerializeField, Tooltip("位置网格吸附步长（米），0 关闭")] float gridSnap = 0f;
        [SerializeField, Tooltip("最小路段长度（米）")] float minSegmentLength = 1f;
        [SerializeField, Tooltip("预览网格抬高量，防 Z-fighting")] float previewLift = 0.03f;

        [Header("预览颜色")]
        [SerializeField] Color previewOkColor = new Color(0.2f, 0.9f, 1f, 0.45f);
        [SerializeField] Color previewBadColor = new Color(1f, 0.35f, 0.2f, 0.45f);

        RoadNetworkBehaviour net;
        Profile pendingStart;
        bool pendingStartIsNew;

        GameObject previewGo;
        MeshFilter previewMf;
        MeshRenderer previewMr;
        Material matOk, matBad;
        string lastPreviewKey;

        void Awake()
        {
            net = GetComponent<RoadNetworkBehaviour>();
            if (cam == null) cam = Camera.main;
            EnsurePreviewObjects();
        }

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

        void Update()
        {
#if ENABLE_INPUT_SYSTEM
            if (!buildEnabled || cam == null || ClickAction == null) return;

            if (ClickAction.WasPressedThisFrame()) OnLeftClick();
            if (CancelAction != null && CancelAction.WasPressedThisFrame()) CancelPending();
            if (DeleteAction != null && DeleteAction.WasPressedThisFrame()) DeleteLastSegment();

            if (pendingStart != null) UpdatePreview();
#else
            // 未启用 Input System 时静默不工作（Project Settings → Active Input Handling）
#endif
        }

        // ---------------- 交互 ----------------

        void OnLeftClick()
        {
            if (!GroundPoint(out DVec2 pos)) return;
            pos = SnapPos(pos);

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

        void UpdatePreview()
        {
            EnsurePreviewObjects();
            if (!GroundPoint(out DVec2 endPos))
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
            var tmpB = Profile.Create(endPos, endDir, pendingStart.Lanes);
            var path = ProfileConnector.Connect(tmpA, tmpB);
            if (path == null)
            {
                SetPreviewVisible(false);
                return;
            }

            var mesh = SegmentMeshBuilder.Build(path, pendingStart.Lanes, pendingStart.Position, 0.05f, 6f);
            previewGo.transform.position = new Vector3(
                (float)pendingStart.Position.X, previewLift, (float)pendingStart.Position.Y);
            var old = previewMf.sharedMesh;
            previewMf.sharedMesh = mesh;
            previewMr.sharedMaterial = bad ? matBad : matOk;
            SetPreviewVisible(true);
            if (old != null) Destroy(old);
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

        void SetPreviewVisible(bool v)
        {
            if (previewGo != null && previewGo.activeSelf != v) previewGo.SetActive(v);
        }

        void OnDestroy()
        {
            if (previewMf != null && previewMf.sharedMesh != null) Destroy(previewMf.sharedMesh);
            if (previewGo != null) Destroy(previewGo);
            if (matOk != null) Destroy(matOk);
            if (matBad != null) Destroy(matBad);
        }

        // ---------------- 工具 ----------------

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

        DVec2 SnapPos(DVec2 p)
        {
            if (gridSnap <= 0.001f) return p;
            double s = gridSnap;
            return new DVec2(System.Math.Round(p.X / s) * s, System.Math.Round(p.Y / s) * s);
        }

        bool GroundPoint(out DVec2 p)
        {
            p = DVec2.Zero;
#if ENABLE_INPUT_SYSTEM
            if (PointAction == null) return false;
            Ray ray = cam.ScreenPointToRay(PointAction.ReadValue<Vector2>());
            if (Mathf.Abs(ray.direction.y) < 1e-6f) return false;
            float t = -ray.origin.y / ray.direction.y;
            if (t < 0) return false;
            Vector3 hit = ray.origin + ray.direction * t;
            p = new DVec2(hit.x, hit.z);
            return true;
#else
            return false;
#endif
        }
    }
}
