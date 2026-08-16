using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;

namespace RoadSystem.EditorTools
{
    /// <summary>
    /// 道路放置工具（类《城市：天际线》）：
    ///  - 单击放置起点 profile，移动鼠标实时虚影预览，再单击放置终点，连续点击形成连续路段
    ///  - 按下并拖拽 ≥2m 可显式指定 profile 朝向（曲线模式）；直接单击则沿用弦向/前段切向
    ///  - 吸附：既有悬空端 profile（3m）、网格（EditorSnapSettings）、角度 15°；Ctrl 临时禁用
    ///  - 半平面约束：鼠标位于源 profile 后方半平面时，预览朝向强制与源同向并红色警示
    ///  - Esc 取消当前待放置起点；所有编辑经 Undo.RecordObject，支持 Undo/Redo
    /// </summary>
    [EditorTool("道路放置工具", typeof(RoadNetworkBehaviour))]
    public class RoadPlacementTool : EditorTool
    {
        const double LooseEndSnapDist = 3.0;   // 悬空端吸附半径（米）
        const double DragDirThreshold = 2.0;   // 拖拽定向阈值（米）
        const double MinSegmentLength = 1.0;   // 最小路段长度（米）
        const double AngleSnapDeg = 15.0;

        GUIContent icon;
        public override GUIContent toolbarIcon =>
            icon ??= new GUIContent(EditorGUIUtility.IconContent("Grid.MoveTool").image, "道路放置工具");

        // ---- 工具状态 ----
        Profile pendingStart;      // 待连接起点（已在图中）
        bool pendingStartIsNew;    // 由本工具新建（Esc 可移除）
        bool dragging;
        DVec2 downPos;
        DVec2 hoverPos;
        bool hoverValid;

        // ---- 预览 ----
        GameObject previewGo;
        MeshFilter previewMf;
        MeshRenderer previewMr;
        Material previewMatOk;
        Material previewMatBad;
        string lastPreviewKey;
        bool previewBad;           // 当前预览是否处于非法/警示状态

        RoadNetworkBehaviour Net => target as RoadNetworkBehaviour;

        void OnEnable()
        {
            EnsurePreviewObjects();
        }

        void OnDisable()
        {
            DestroyPreview();
            pendingStart = null;
            dragging = false;
        }

        public override void OnToolGUI(EditorWindow window)
        {
            var net = Net;
            if (net == null) return;
            Event e = Event.current;
            int id = GUIUtility.GetControlID(FocusType.Passive);

            switch (e.GetTypeForControl(id))
            {
                case EventType.Layout:
                    HandleUtility.AddDefaultControl(id);
                    break;

                case EventType.MouseMove:
                    hoverValid = GroundPoint(e.mousePosition, out hoverPos);
                    dragging = false;
                    UpdatePreview(net);
                    HandleUtility.Repaint();
                    break;

                case EventType.MouseDown when e.button == 0 && !e.alt:
                    if (GroundPoint(e.mousePosition, out downPos))
                    {
                        downPos = SnapPos(downPos, e);
                        dragging = true;
                        hoverPos = downPos;
                        hoverValid = true;
                        GUIUtility.hotControl = id;
                        e.Use();
                    }
                    break;

                case EventType.MouseDrag when e.button == 0 && GUIUtility.hotControl == id:
                    if (GroundPoint(e.mousePosition, out hoverPos))
                        hoverValid = true;
                    UpdatePreview(net);
                    HandleUtility.Repaint();
                    e.Use();
                    break;

                case EventType.MouseUp when e.button == 0 && GUIUtility.hotControl == id:
                    GUIUtility.hotControl = 0;
                    Commit(net, e);
                    dragging = false;
                    lastPreviewKey = null;
                    UpdatePreview(net);
                    HandleUtility.Repaint();
                    e.Use();
                    break;

                case EventType.KeyDown when e.keyCode == KeyCode.Escape:
                    CancelPending(net);
                    e.Use();
                    HandleUtility.Repaint();
                    break;
            }

            DrawOverlay(net, e);
        }

        // ---------------- 提交 ----------------

        void Commit(RoadNetworkBehaviour net, Event e)
        {
            DVec2 upPos = SnapPos(hoverPos, e);
            bool dragged = DVec2.Distance(upPos, downPos) >= DragDirThreshold;

            if (pendingStart == null)
            {
                // 起点：优先吸附既有悬空端
                var loose = FindLooseEnd(net, downPos);
                if (loose != null)
                {
                    pendingStart = loose;
                    pendingStartIsNew = false;
                }
                else
                {
                    DVec2 dir = dragged
                        ? SnapAngle((upPos - downPos).Normalized, e)
                        : DVec2.UnitY; // 默认 +Z
                    Undo.RecordObject(net, "Place Road Profile");
                    pendingStart = net.PlaceProfile(downPos, dir, Profile.DefaultLanes());
                    pendingStartIsNew = true;
                }
            }
            else
            {
                // 终点过近：忽略
                if (DVec2.Distance(downPos, pendingStart.Position) < MinSegmentLength)
                    return;

                // 终点：吸附既有悬空端则复用
                var loose = FindLooseEnd(net, downPos);
                Profile end;
                bool endIsReused = loose != null && loose.Id != pendingStart.Id;

                // 半平面约束：落点在源后方 → 强制同向
                bool behind = DVec2.Dot(downPos - pendingStart.Position, pendingStart.Direction) < 0;

                if (endIsReused)
                {
                    end = loose;
                }
                else
                {
                    DVec2 dir;
                    if (behind) dir = pendingStart.Direction;
                    else if (dragged) dir = SnapAngle((upPos - downPos).Normalized, e);
                    else dir = (downPos - pendingStart.Position).Normalized;

                    Undo.RecordObject(net, "Add Road Segment");
                    end = net.PlaceProfile(downPos, dir, (LaneDef[])pendingStart.Lanes.Clone());
                }

                var seg = net.AddSegment(pendingStart, end);
                if (seg == null)
                {
                    Debug.LogWarning("[RoadSystem] 路段创建失败（断面不一致或参数非法）");
                    return;
                }
                pendingStart = end;
                pendingStartIsNew = !endIsReused;
            }
        }

        void CancelPending(RoadNetworkBehaviour net)
        {
            if (pendingStart != null && pendingStartIsNew)
            {
                Undo.RecordObject(net, "Cancel Road Profile");
                net.RemoveLooseProfile(pendingStart); // 仅当从未接入任何段时生效
            }
            pendingStart = null;
            pendingStartIsNew = false;
            dragging = false;
            ClearPreviewMesh();
        }

        // ---------------- 预览 ----------------

        void UpdatePreview(RoadNetworkBehaviour net)
        {
            EnsurePreviewObjects();
            if (pendingStart == null || !hoverValid)
            {
                ClearPreviewMesh();
                previewGo.SetActive(false);
                return;
            }

            DVec2 endPos = SnapPos(hoverPos, Event.current);
            double dist = DVec2.Distance(endPos, pendingStart.Position);
            bool behind = DVec2.Dot(endPos - pendingStart.Position, pendingStart.Direction) < 0;
            bool tooShort = dist < MinSegmentLength;

            DVec2 endDir;
            if (behind) endDir = pendingStart.Direction;
            else if (dragging && DVec2.Distance(endPos, downPos) >= DragDirThreshold)
                endDir = SnapAngle((endPos - downPos).Normalized, Event.current);
            else if (dist > 0.1) endDir = (endPos - pendingStart.Position).Normalized;
            else endDir = pendingStart.Direction;

            previewBad = behind || tooShort;

            string key = $"{endPos.X:F2},{endPos.Y:F2},{endDir.X:F3},{endDir.Y:F3},{previewBad}";
            if (key == lastPreviewKey) return; // 帧合并：位置未变不重建
            lastPreviewKey = key;

            if (tooShort)
            {
                previewGo.SetActive(false);
                return;
            }

            var tmpA = Profile.Create(pendingStart.Position, pendingStart.Direction, pendingStart.Lanes);
            var tmpB = Profile.Create(endPos, endDir, pendingStart.Lanes);
            var path = ProfileConnector.Connect(tmpA, tmpB);
            if (path == null)
            {
                previewGo.SetActive(false);
                return;
            }

            var mesh = SegmentMeshBuilder.Build(path, pendingStart.Lanes, pendingStart.Position, 0.05f, 6f);
            previewGo.SetActive(true);
            previewGo.transform.position = new Vector3(
                (float)pendingStart.Position.X, 0.03f, (float)pendingStart.Position.Y); // 微抬防 Z-fighting
            var old = previewMf.sharedMesh;
            previewMf.sharedMesh = mesh;
            previewMr.sharedMaterial = previewBad ? previewMatBad : previewMatOk;
            if (old != null) DestroyImmediate(old);
        }

        void EnsurePreviewObjects()
        {
            if (previewGo == null)
            {
                previewGo = new GameObject("~RoadPreview")
                {
                    hideFlags = HideFlags.HideAndDontSave
                };
                previewMf = previewGo.AddComponent<MeshFilter>();
                previewMr = previewGo.AddComponent<MeshRenderer>();
                previewGo.SetActive(false);
            }
            if (previewMatOk == null) previewMatOk = MakePreviewMaterial(new Color(0.2f, 0.9f, 1f, 0.45f));
            if (previewMatBad == null) previewMatBad = MakePreviewMaterial(new Color(1f, 0.35f, 0.2f, 0.45f));
        }

        static Material MakePreviewMaterial(Color c)
        {
            var shader = Shader.Find("Universal Render Pipeline/Unlit");
            var m = new Material(shader) { hideFlags = HideFlags.HideAndDontSave };
            m.SetFloat("_Surface", 1f); // Transparent
            m.SetFloat("_Blend", 0f);   // Alpha
            m.SetColor("_BaseColor", c);
            m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            return m;
        }

        void ClearPreviewMesh()
        {
            lastPreviewKey = null;
            if (previewMf != null && previewMf.sharedMesh != null)
            {
                DestroyImmediate(previewMf.sharedMesh);
                previewMf.sharedMesh = null;
            }
        }

        void DestroyPreview()
        {
            ClearPreviewMesh();
            if (previewGo != null) DestroyImmediate(previewGo);
            if (previewMatOk != null) DestroyImmediate(previewMatOk);
            if (previewMatBad != null) DestroyImmediate(previewMatBad);
        }

        // ---------------- 绘制 ----------------

        void DrawOverlay(RoadNetworkBehaviour net, Event e)
        {
            // 所有悬空端：吸附提示圈
            Handles.color = new Color(0.2f, 0.9f, 1f, 0.9f);
            foreach (var p in net.Graph.LooseEndProfiles())
            {
                var wp = ToV3(p.Position);
                Handles.DrawSolidDisc(wp, Vector3.up, 0.6f);
            }

            // 待连接起点：profile 线 + 车道分界 + 朝向箭头
            if (pendingStart != null)
            {
                double w = pendingStart.TotalWidth;
                Handles.color = Color.green;
                Handles.DrawAAPolyLine(4f, ToV3(pendingStart.LeftEdge), ToV3(pendingStart.RightEdge));

                Handles.color = new Color(0f, 1f, 0f, 0.5f);
                var bounds = pendingStart.BoundaryOffsets();
                for (int i = 1; i < bounds.Length - 1; i++)
                {
                    var p = ToV3(pendingStart.LateralPoint(bounds[i]));
                    Handles.DrawDottedLine(p, p + Vector3.up * 0.8f, 2f);
                }

                // 朝向箭头
                var c = ToV3(pendingStart.Position);
                var fwd = ToDir(pendingStart.Direction);
                Handles.color = Color.yellow;
                Handles.ArrowHandleCap(0, c, Quaternion.LookRotation(fwd), 3f, EventType.Repaint);

                // 半平面警示
                if (hoverValid && DVec2.Dot(hoverPos - pendingStart.Position, pendingStart.Direction) < 0)
                {
                    Handles.color = new Color(1f, 0.3f, 0.2f, 0.9f);
                    var l = ToV3(pendingStart.Position + pendingStart.Left * 60);
                    var r = ToV3(pendingStart.Position - pendingStart.Left * 60);
                    Handles.DrawAAPolyLine(3f, l, r);
                    Handles.Label(ToV3(hoverPos) + Vector3.up * 2,
                        "后方半平面：朝向已锁定为源方向", EditorStyles.boldLabel);
                }
            }

            // 操作提示
            Handles.BeginGUI();
            GUILayout.BeginArea(new Rect(10, 10, 460, 60), GUI.skin.box);
            GUILayout.Label(pendingStart == null
                ? "道路工具：单击放置起点（拖拽≥2m 定向）"
                : "单击放置终点并连续延伸 / 拖拽定向 / Esc 取消起点 / Ctrl 禁用吸附");
            GUILayout.EndArea();
            Handles.EndGUI();
        }

        // ---------------- 工具 ----------------

        Profile FindLooseEnd(RoadNetworkBehaviour net, DVec2 pos)
        {
            Profile best = null;
            double bestD = LooseEndSnapDist;
            foreach (var p in net.Graph.LooseEndProfiles())
            {
                if (pendingStart != null && p.Id == pendingStart.Id) continue;
                double d = DVec2.Distance(p.Position, pos);
                if (d < bestD) { bestD = d; best = p; }
            }
            return best;
        }

        static bool GroundPoint(Vector2 guiPos, out DVec2 p)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(guiPos);
            p = DVec2.Zero;
            if (Mathf.Abs(ray.direction.y) < 1e-6f) return false;
            float t = -ray.origin.y / ray.direction.y;
            if (t < 0) return false;
            Vector3 hit = ray.origin + ray.direction * t;
            p = new DVec2(hit.x, hit.z);
            return true;
        }

        static DVec2 SnapPos(DVec2 p, Event e)
        {
            if (e != null && e.control) return p;
            float step = EditorSnapSettings.move.x;
            if (step <= 0.001f) step = 1f;
            return new DVec2(
                System.Math.Round(p.X / step) * step,
                System.Math.Round(p.Y / step) * step);
        }

        static DVec2 SnapAngle(DVec2 dir, Event e)
        {
            if (e != null && e.control) return dir;
            double ang = System.Math.Atan2(dir.Y, dir.X);
            double step = AngleSnapDeg * System.Math.PI / 180.0;
            ang = System.Math.Round(ang / step) * step;
            return new DVec2(System.Math.Cos(ang), System.Math.Sin(ang));
        }

        static Vector3 ToV3(DVec2 p) => new Vector3((float)p.X, 0f, (float)p.Y);
        static Vector3 ToDir(DVec2 d) => new Vector3((float)d.X, 0f, (float)d.Y);
    }
}
