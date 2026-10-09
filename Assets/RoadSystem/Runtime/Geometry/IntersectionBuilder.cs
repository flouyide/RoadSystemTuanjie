using System;
using System.Collections.Generic;
using System.Linq;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>路口边界环顶点（位置 + 高度），供 L2 三角化铺装面。</summary>
    public struct BoundaryVertex
    {
        public DVec2 Pos;
        public float Y;
    }

    /// <summary>
    /// M4 路口构建器（plan §5）：
    ///  1. 交汇检测：新段 PathChain 与全图段的 AABB 粗筛 + LineLine/LineArc/ArcArc O(1) 精判；
    ///  2. 交点处拆分：被穿越段在交点两侧各后退 corner 半径距离生成 port profile，拆成两半段；
    ///     新段横穿同样拆分（十字），新段端点落在旧路上则截断后退（T 型）；
    ///  3. 生成 Intersection 节点（N≥3 ports）+ LaneConnection 默认配对（显式存储）；
    ///  4. 转角边界：相邻 port（极角序）外缘端点间用 Fillet.Solve 两线圆角构造（复用 L1 fillet）；
    ///  5. 2-port 退化路口合并回 RoadSegment。
    /// 纯 C#（L1），图操作通过 RoadGraph 完成，结束后由调用方 CommitEdit 一次性序列化。
    /// </summary>
    public static class IntersectionBuilder
    {
        // ---- 常量 ----
        /// <summary>交点距段端点的容差（米）：以内视为端点连接（T 型端点或正常链式续接）。</summary>
        const double EndpointTol = 1.0;
        /// <summary>拆分后每半段的最小长度（米），防止极碎段。</summary>
        const double MinHalf = 2.0;
        /// <summary>路口中心区域的最小保留长度（米）：两 port 之间至少留出的路面。</summary>
        const double MinGap = 1.0;
        /// <summary>port 后退的额外余量（米）：backoff = 半宽 + 该值，决定转角圆角半径下限。</summary>
        const double CornerMargin = 3.0;
        /// <summary>直行判定阈值（弧度）：|turn| 小于该值按直行配对。</summary>
        const double StraightTurn = Math.PI / 4;
        /// <summary>递归检测深度上限（新段拆分出的半段继续检测穿越）。</summary>
        const int MaxDetectDepth = 2;

        // ================================================================
        // 公共入口
        // ================================================================

        /// <summary>
        /// 检测新段与全图的交汇并在交点处构建路口（拆分/截断/建 Intersection/配对）。
        /// 一次调用只建一个路口；拆分出的半段会递归检测（深度限制），覆盖连穿多条路的情形。
        /// 返回是否建了路口（图已变更，调用方须 CommitEdit）。
        /// </summary>
        public static bool DetectAndBuild(RoadGraph g, string newSegId, int depth = 0)
        {
            if (depth >= MaxDetectDepth) return false;
            var newSeg = g.GetNode(newSegId) as RoadSegment;
            if (newSeg == null) return false;
            var pathNew = PathOf(g, newSeg);
            if (pathNew == null) return false;

            // 快照防御：BuildOne 会增删段（枚举中修改字典抛异常），ToList 后遍历
            foreach (var other in g.Segments().ToList())
            {
                if (other.Id == newSegId) continue;
                // 同一条路（同 RoadId）之间不建口：弯路折返自交等情形跳过，避免自交路口
                if (!string.IsNullOrEmpty(newSeg.RoadId) && newSeg.RoadId == other.RoadId) continue;
                var pathOther = PathOf(g, other);
                if (pathOther == null) continue;
                if (!BoundsOverlap(pathNew, pathOther)) continue;

                foreach (var hit in IntersectChains(pathNew, pathOther))
                {
                    var kind = Classify(pathNew, pathOther, hit.SNew, hit.SOther);
                    if (kind == null) continue;
                    if (BuildOne(g, newSegId, other.Id, hit.SNew, hit.SOther, kind.Value, depth))
                        return true;
                }
            }
            return false;
        }

        /// <summary>ports 的几何中心（近似路口中心）。</summary>
        public static DVec2 CenterOf(List<Profile> ports)
        {
            var c = DVec2.Zero;
            if (ports == null || ports.Count == 0) return c;
            foreach (var p in ports) c += p.Position;
            return c / ports.Count;
        }

        // ================================================================
        // 检测：AABB 粗筛 + 精判求交 + 交点弧长归属
        // ================================================================

        struct Hit
        {
            public DVec2 Point;
            public double SNew;   // 交点在新段 path 上的弧长
            public double SOther; // 交点在既有段 path 上的弧长
        }

        enum CrossKind
        {
            Interior,      // 新段从既有段内部横穿（十字）
            TeeAtNewEnd,   // 新段终点落在既有段上（T 型，流入）
            TeeAtNewStart  // 新段起点落在既有段上（T 型，流出）
        }

        static bool BoundsOverlap(PathChain a, PathChain b)
        {
            if (!PathChainUtil.Bounds(a, out var aMin, out var aMax)) return false;
            if (!PathChainUtil.Bounds(b, out var bMin, out var bMax)) return false;
            return aMin.X <= bMax.X && aMax.X >= bMin.X && aMin.Y <= bMax.Y && aMax.Y >= bMin.Y;
        }

        /// <summary>两条 PathChain 的全部交点（元素两两 O(1) 闭式求交，含弧长归属）。</summary>
        static List<Hit> IntersectChains(PathChain a, PathChain b)
        {
            var hits = new List<Hit>();
            var tmp = new List<DVec2>();
            double accA = 0;
            foreach (var ea in a.Elements)
            {
                double accB = 0;
                foreach (var eb in b.Elements)
                {
                    tmp.Clear();
                    int n = IntersectElements(ea, eb, tmp);
                    for (int i = 0; i < n; i++)
                    {
                        hits.Add(new Hit
                        {
                            Point = tmp[i],
                            SNew = accA + ElementParam(ea, tmp[i]) * ea.Length,
                            SOther = accB + ElementParam(eb, tmp[i]) * eb.Length
                        });
                    }
                    accB += eb.Length;
                }
                accA += ea.Length;
            }
            return hits;
        }

        static int IntersectElements(PathElement ea, PathElement eb, List<DVec2> hits)
        {
            if (ea is LineElement la && eb is LineElement lb)
            {
                if (ArcIntersections.LineLine(la.A, la.B, lb.A, lb.B, out var p)) { hits.Add(p); return 1; }
                return 0;
            }
            if (ea is LineElement l1 && eb is ArcElement a2)
                return ArcIntersections.LineArc(l1.A, l1.B, a2, hits);
            if (ea is ArcElement a1 && eb is LineElement l2)
                return ArcIntersections.LineArc(l2.A, l2.B, a1, hits);
            if (ea is ArcElement aa && eb is ArcElement ab)
                return ArcIntersections.ArcArc(aa, ab, hits);
            return 0;
        }

        /// <summary>点在元素上的参数化位置 [0,1]（直线按投影，圆弧按角度）。</summary>
        static double ElementParam(PathElement e, DVec2 p)
        {
            if (e is LineElement line)
            {
                DVec2 d = line.B - line.A;
                double lenSq = d.LengthSquared;
                return lenSq > 1e-18 ? GeomConsts.Clamp(DVec2.Dot(p - line.A, d) / lenSq, 0, 1) : 0;
            }
            if (e is ArcElement arc)
            {
                double theta = Math.Atan2(p.Y - arc.Center.Y, p.X - arc.Center.X);
                double rel = ArcIntersections.NormalizeAngle(theta - arc.StartAngle); // (-π, π]
                if (Math.Abs(arc.Sweep) < 1e-12) return 0;
                return GeomConsts.Clamp(rel / arc.Sweep, 0, 1);
            }
            return 0;
        }

        /// <summary>交点分类：贴近既有段端点视为正常端点连接（不建口）；贴近新段端点为 T 型。</summary>
        static CrossKind? Classify(PathChain pathNew, PathChain pathOther, double sNew, double sOther)
        {
            if (sOther < EndpointTol || pathOther.TotalLength - sOther < EndpointTol) return null;
            if (pathNew.TotalLength - sNew < EndpointTol) return CrossKind.TeeAtNewEnd;
            if (sNew < EndpointTol) return CrossKind.TeeAtNewStart;
            return CrossKind.Interior;
        }

        // ================================================================
        // 建口：拆分 + Intersection + 配对
        // ================================================================

        static bool BuildOne(RoadGraph g, string newSegId, string otherId,
            double sNew, double sOther, CrossKind kind, int depth)
        {
            var segNew = g.GetNode(newSegId) as RoadSegment;
            var segOther = g.GetNode(otherId) as RoadSegment;
            if (segNew == null || segOther == null) return false;
            var pathNew = PathOf(g, segNew);
            var pathOther = PathOf(g, segOther);
            if (pathNew == null || pathOther == null) return false;

            // 可拆性预检（避免拆了一半另一段拆不动，留下断路）
            if (!CanSplitAt(pathOther, sOther, g, segOther)) return false;
            if (kind == CrossKind.Interior && !CanSplitAt(pathNew, sNew, g, segNew)) return false;

            // 1. 拆分被穿越的既有段 → 两个 port（流出半段终点 + 流入半段起点）
            if (!SplitInterior(g, segOther, pathOther, sOther,
                out var aOut, out var aIn, out _, out _))
                return false;

            // 2. 新段处理
            var ports = new List<Profile> { aOut, aIn };
            string rightHalfId = null;
            if (kind == CrossKind.Interior)
            {
                if (!SplitInterior(g, segNew, pathNew, sNew,
                    out var bOut, out var bIn, out var leftId, out rightHalfId))
                    return false;
                ports.Add(bOut);
                ports.Add(bIn);
                if (leftId != null) DetectAndBuild(g, leftId, depth + 1); // 左半段也可能穿越
            }
            else if (kind == CrossKind.TeeAtNewEnd)
            {
                ports.Add(TruncateEnd(g, segNew, pathNew, sNew));
            }
            else
            {
                ports.Add(TruncateStart(g, segNew, pathNew, sNew));
            }

            // 3. 创建 Intersection 节点并挂接 ports
            var ix = new Intersection();
            g.Nodes[ix.Id] = ix;
            foreach (var p in ports)
            {
                if (p == null) continue;
                ix.PortIds.Add(p.Id);
                AttachToNode(p, ix.Id);
            }
            if (ix.PortIds.Count < 3)
            {
                g.Nodes.Remove(ix.Id); // 不足 3 口：撤销（理论不可达，防御）
                return false;
            }

            // 4. 默认车道配对（显式存储进 LaneLinks）
            BuildLaneLinks(g, ix);

            // 5. 通知受影响节点重建（路口 + 全部半段）
            var affected = new List<string> { ix.Id };
            foreach (var pid in ix.PortIds)
            {
                var p = g.GetProfile(pid);
                if (p == null) continue;
                foreach (var nid in new[] { p.NodeAId, p.NodeBId })
                    if (!string.IsNullOrEmpty(nid) && nid != ix.Id && !affected.Contains(nid))
                        affected.Add(nid);
            }
            g.NotifyChanged(affected);

            // 6. 新段右半段（含原终点侧）继续检测：一条路连穿多条路的情形
            if (rightHalfId != null) DetectAndBuild(g, rightHalfId, depth + 1);
            return true;
        }

        /// <summary>段的 path（优先缓存，缺则现场解算并缓存）。</summary>
        static PathChain PathOf(RoadGraph g, RoadSegment seg)
        {
            if (seg.Path != null) return seg.Path;
            if (seg.PortIds.Count != 2) return null;
            var pa = g.GetProfile(seg.PortIds[0]);
            var pb = g.GetProfile(seg.PortIds[1]);
            if (pa == null || pb == null) return null;
            seg.Path = ProfileConnector.Connect(pa, pb);
            return seg.Path;
        }

        /// <summary>拆分预检：交点两侧留足半段最小长度与路口中心保留区。</summary>
        static bool CanSplitAt(PathChain path, double s, RoadGraph g, RoadSegment seg)
        {
            double L = path.TotalLength;
            var pa = g.GetProfile(seg.PortIds[0]);
            if (pa == null) return false;
            double backoff = pa.TotalWidth * 0.5 + CornerMargin;
            double sL = GeomConsts.Clamp(s - backoff, MinHalf, s);
            double sR = GeomConsts.Clamp(s + backoff, s, L - MinHalf);
            return sL >= MinHalf - 1e-6 && L - sR >= MinHalf - 1e-6 && sR - sL >= MinGap;
        }

        /// <summary>
        /// 内部拆分：交点 s 两侧各后退 backoff 生成两个 port profile（流出+流入），
        /// 原段删除、生成左右两半段（继承 RoadId 与 path 子段缓存）。
        /// </summary>
        static bool SplitInterior(RoadGraph g, RoadSegment seg, PathChain path, double s,
            out Profile portOut, out Profile portIn, out string leftId, out string rightId)
        {
            portOut = null; portIn = null; leftId = null; rightId = null;
            if (seg.PortIds.Count != 2) return false;
            var pa = g.GetProfile(seg.PortIds[0]);
            var pb = g.GetProfile(seg.PortIds[1]);
            if (pa == null || pb == null) return false;

            double L = path.TotalLength;
            double backoff = pa.TotalWidth * 0.5 + CornerMargin;
            double sL = GeomConsts.Clamp(s - backoff, MinHalf, s);
            double sR = GeomConsts.Clamp(s + backoff, s, L - MinHalf);
            if (sL < MinHalf - 1e-6 || L - sR < MinHalf - 1e-6 || sR - sL < MinGap)
                return false;

            if (!PathChainUtil.PointAt(path, sL, out var posL, out var tanL)) return false;
            if (!PathChainUtil.PointAt(path, sR, out var posR, out var tanR)) return false;

            portOut = g.CreateProfile(posL, tanL, (LaneDef[])pa.Lanes.Clone());
            portOut.Y = (float)(pa.Y + (pb.Y - pa.Y) * (sL / L));
            portIn = g.CreateProfile(posR, tanR, (LaneDef[])pa.Lanes.Clone());
            portIn.Y = (float)(pa.Y + (pb.Y - pa.Y) * (sR / L));

            string roadId = seg.RoadId;
            string segId = seg.Id;
            g.RemoveSegment(segId);
            var left = g.AddSegment(pa, portOut, roadId);
            var right = g.AddSegment(portIn, pb, roadId);
            if (left == null || right == null) return false;
            left.Path = PathChainUtil.SubPath(path, 0, sL);
            right.Path = PathChainUtil.SubPath(path, sR, L);
            leftId = left.Id;
            rightId = right.Id;
            return true;
        }

        /// <summary>
        /// T 型终点截断：新段终点 port 沿 path 后退 backoff 到路口边界。
        /// 终点 profile 为悬空端（本建造流程新建）时移动之；被共享或距离不足则原位作为 port。
        /// </summary>
        static Profile TruncateEnd(RoadGraph g, RoadSegment seg, PathChain path, double s)
        {
            if (seg.PortIds.Count != 2) return null;
            var pa = g.GetProfile(seg.PortIds[0]);
            var pb = g.GetProfile(seg.PortIds[1]);
            if (pa == null || pb == null) return null;

            double L = path.TotalLength;
            double backoff = pa.TotalWidth * 0.5 + CornerMargin;
            double sN = GeomConsts.Clamp(s - backoff, MinHalf, L);
            if (L - sN < MinGap) return pb; // 终点距交点太近：port 即终点原位
            if (IsSharedByOther(pb, seg.Id)) return pb;

            if (!PathChainUtil.PointAt(path, sN, out var pos, out var tan)) return pb;
            g.MoveProfile(pb, pos, tan);
            pb.Y = (float)(pa.Y + (pb.Y - pa.Y) * (sN / L));
            seg.Path = PathChainUtil.SubPath(path, 0, sN);
            return pb;
        }

        /// <summary>T 型起点截断：与 TruncateEnd 对称（起点 port 后退）。</summary>
        static Profile TruncateStart(RoadGraph g, RoadSegment seg, PathChain path, double s)
        {
            if (seg.PortIds.Count != 2) return null;
            var pa = g.GetProfile(seg.PortIds[0]);
            var pb = g.GetProfile(seg.PortIds[1]);
            if (pa == null || pb == null) return null;

            double L = path.TotalLength;
            double backoff = pa.TotalWidth * 0.5 + CornerMargin;
            double sN = GeomConsts.Clamp(s + backoff, 0, L - MinHalf);
            if (sN < MinGap) return pa;
            if (IsSharedByOther(pa, seg.Id)) return pa;

            if (!PathChainUtil.PointAt(path, sN, out var pos, out var tan)) return pa;
            g.MoveProfile(pa, pos, tan);
            pa.Y = (float)(pa.Y + (pb.Y - pa.Y) * (sN / L));
            seg.Path = PathChainUtil.SubPath(path, sN, L);
            return pa;
        }

        /// <summary>profile 除 segId 外是否还挂着其他节点（共享中间端）。</summary>
        static bool IsSharedByOther(Profile p, string segId)
            => (p.NodeAId != null && p.NodeAId != segId)
               || (p.NodeBId != null && p.NodeBId != segId);

        static void AttachToNode(Profile p, string nodeId)
        {
            if (string.IsNullOrEmpty(p.NodeAId)) p.NodeAId = nodeId;
            else if (string.IsNullOrEmpty(p.NodeBId)) p.NodeBId = nodeId;
        }

        static void DetachFromNode(Profile p, string nodeId)
        {
            if (p.NodeAId == nodeId) p.NodeAId = null;
            else if (p.NodeBId == nodeId) p.NodeBId = null;
        }

        static string OtherNode(Profile p, string excludeId)
        {
            if (!string.IsNullOrEmpty(p.NodeAId) && p.NodeAId != excludeId) return p.NodeAId;
            if (!string.IsNullOrEmpty(p.NodeBId) && p.NodeBId != excludeId) return p.NodeBId;
            return null;
        }

        // ================================================================
        // LaneConnection 默认配对（plan §5.2：直行角度最优 + 左转最左 + 右转最右）
        // ================================================================

        /// <summary>
        /// 生成默认车道连接：from = 流入 port（Direction 指向路口中心），to = 流出 port。
        /// 转角 turn = signedAngle(from.Direction, to.Direction)：
        /// |turn| &lt; 45° 直行（每条 Car 车道按同序号配对）；turn &gt; 0 左转（最左↔最左）；turn &lt; 0 右转（最右↔最右）。
        /// 配对显式写入 LaneLinks；Path 走 ProfileConnector 解算（缓存，暂不渲染）。
        /// </summary>
        public static void BuildLaneLinks(RoadGraph g, Intersection ix)
        {
            ix.LaneLinks.Clear();
            var ports = LivePorts(g, ix);
            if (ports.Count < 3) return;
            var center = CenterOf(ports);
            var inflow = new List<Profile>();
            var outflow = new List<Profile>();
            foreach (var p in ports)
                (DVec2.Dot(p.Direction, center - p.Position) > 0 ? inflow : outflow).Add(p);
            if (inflow.Count == 0 || outflow.Count == 0) return;

            foreach (var from in inflow)
            foreach (var to in outflow)
            {
                double turn = ArcIntersections.NormalizeAngle(Math.Atan2(
                    DVec2.Cross(from.Direction, to.Direction), DVec2.Dot(from.Direction, to.Direction)));
                if (Math.Abs(turn) < StraightTurn) ConnectStraight(g, ix, from, to);
                else ConnectTurn(g, ix, from, to, leftmost: turn > 0);
            }
        }

        static void ConnectStraight(RoadGraph g, Intersection ix, Profile from, Profile to)
        {
            var fromCar = CarIndices(from);
            var toCar = CarIndices(to);
            for (int m = 0; m < fromCar.Count && m < toCar.Count; m++)
                AddLink(g, ix, from, fromCar[m], to, toCar[m]);
        }

        static void ConnectTurn(RoadGraph g, Intersection ix, Profile from, Profile to, bool leftmost)
        {
            var fromCar = CarIndices(from);
            var toCar = CarIndices(to);
            if (fromCar.Count == 0 || toCar.Count == 0) return;
            int fk = leftmost ? fromCar[0] : fromCar[fromCar.Count - 1];
            int tk = leftmost ? toCar[0] : toCar[toCar.Count - 1];
            AddLink(g, ix, from, fk, to, tk);
        }

        static List<int> CarIndices(Profile p)
        {
            var list = new List<int>();
            if (p.Lanes == null) return list;
            for (int i = 0; i < p.Lanes.Length; i++)
                if (p.Lanes[i].Type == LaneType.Car) list.Add(i);
            return list;
        }

        static void AddLink(RoadGraph g, Intersection ix, Profile from, int fromLane, Profile to, int toLane)
        {
            var link = new LaneConnection
            {
                FromPortId = from.Id,
                ToPortId = to.Id,
                FromLane = fromLane,
                ToLane = toLane
            };
            // 车道中心间的转向路径（缓存解算，MVP 不渲染箭头）
            var bFrom = from.BoundaryOffsets();
            var bTo = to.BoundaryOffsets();
            DVec2 a = from.LateralPoint(bFrom[fromLane] + from.Lanes[fromLane].Width * 0.5);
            DVec2 b = to.LateralPoint(bTo[toLane] + to.Lanes[toLane].Width * 0.5);
            link.Path = ProfileConnector.ConnectCenters(a, from.Direction, b, to.Direction, 0);
            ix.LaneLinks.Add(link);
        }

        // ================================================================
        // 转角边界环（plan §5.3：相邻 port 外缘间 fillet 圆角）
        // ================================================================

        /// <summary>
        /// 构建路口边界环：ports 按绕中心极角排序（数学 CCW），
        /// 每个 port 贡献一条截面边（CW 端 → CCW 端），相邻 port 的相邻外缘端点间
        /// 用两线 fillet（Fillet.Solve，圆角半径由 port 后退距离自然决定）构造转角弧。
        /// 环顶点带高度（port.Y 线性插值）。失败返回 null。
        /// </summary>
        public static List<BoundaryVertex> BuildBoundaryLoop(List<Profile> ports, DVec2 center, double sagitta)
        {
            if (ports == null || ports.Count < 3) return null;
            var sorted = ports
                .OrderBy(p => Math.Atan2(p.Position.Y - center.Y, p.Position.X - center.X))
                .ToList();
            int n = sorted.Count;

            // 每个 port 截面线的 CW / CCW 端点（相对中心极角小 / 大侧）
            var cw = new DVec2[n];
            var ccw = new DVec2[n];
            for (int i = 0; i < n; i++)
            {
                var p = sorted[i];
                DVec2 left = p.Direction.PerpLeft;
                var e1 = p.Position + left * (p.TotalWidth * 0.5);
                var e2 = p.Position - left * (p.TotalWidth * 0.5);
                double a1 = Math.Atan2(e1.Y - center.Y, e1.X - center.X);
                double a2 = Math.Atan2(e2.Y - center.Y, e2.X - center.X);
                if (ArcIntersections.NormalizeAngle(a1 - a2) > 0) { ccw[i] = e1; cw[i] = e2; }
                else { ccw[i] = e2; cw[i] = e1; }
            }

            var loop = new List<BoundaryVertex>();
            void AddPt(DVec2 p, double y)
            {
                if (loop.Count == 0 || DVec2.Distance(loop[loop.Count - 1].Pos, p) > 1e-6)
                    loop.Add(new BoundaryVertex { Pos = p, Y = (float)y });
            }

            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                // 转角：port i 的 CCW 外缘端 → port j 的 CW 外缘端（两线 fillet，平行兜底直线）
                var corner = CornerPath(sorted[i], ccw[i], sorted[j], cw[j], center);
                double yFrom = sorted[i].Y, yTo = sorted[j].Y;
                double cornerLen = corner.TotalLength;
                foreach (var pt in PathSampler.Sample(corner, sagitta))
                    AddPt(pt.Position, yFrom + (yTo - yFrom) * (cornerLen > 1e-6 ? pt.ArcLength / cornerLen : 0));

                // port j 截面边：CW 端 → CCW 端
                AddPt(cw[j], sorted[j].Y);
                AddPt(ccw[j], sorted[j].Y);
            }

            // 闭合去尾（首点 = 第一个转角起点，尾点与之重合时删除）
            if (loop.Count > 1 && DVec2.Distance(loop[0].Pos, loop[loop.Count - 1].Pos) < 1e-6)
                loop.RemoveAt(loop.Count - 1);
            return loop.Count >= 3 ? loop : null;
        }

        /// <summary>
        /// 两外缘线间的转角路径：过两 port 外缘端点、沿各自 Direction 的直线求交 C，
        /// 以 (C−E1) / (E2−C) 为 fillet 切向解 Arc+Line（切点即端点，圆角半径 = min(CA, CB)）。
        /// 例外兜底（均退化为直线 E1→E2）：
        ///  · 外缘线平行；
        ///  · C 与路口中心落在弦 E1E2 的同侧 —— 此时圆角弧凹向路口中心（斜交钝角侧），
        ///    会把边界环变成非星形、破坏中心辐射三角化，改为直边转角。
        /// </summary>
        static PathChain CornerPath(Profile pi, DVec2 e1, Profile pj, DVec2 e2, DVec2 center)
        {
            DVec2 d1 = pi.Direction, d2 = pj.Direction;
            double denom = DVec2.Cross(d1, d2);
            if (Math.Abs(denom) < GeomConsts.ParallelEps)
                return Straight(e1, e2);

            double t = DVec2.Cross(e2 - e1, d2) / denom;
            DVec2 c = e1 + d1 * t;
            DVec2 dA = c - e1, dB = e2 - c;
            if (dA.Length < GeomConsts.DistEps || dB.Length < GeomConsts.DistEps)
                return Straight(e1, e2);

            // C 应在转角尖端（比弦 E1E2 更远离中心的一侧）；与中心同侧 → 内凹弧，退化为直线
            double sideC = DVec2.Cross(e2 - e1, c - e1);
            double sideO = DVec2.Cross(e2 - e1, center - e1);
            if (Math.Sign(sideC) == Math.Sign(sideO) && Math.Abs(sideO) > 1e-9)
                return Straight(e1, e2);

            return Fillet.Solve(e1, dA.Normalized, e2, dB.Normalized) ?? Straight(e1, e2);
        }

        static PathChain Straight(DVec2 a, DVec2 b)
        {
            var chain = new PathChain();
            chain.Add(new LineElement(a, b));
            return chain;
        }

        // ================================================================
        // 退化合并（plan §5.4：2-port Intersection 合并回 RoadSegment）
        // ================================================================

        /// <summary>ports 少于 3 个的路口合并回普通路段（删段后调用）；2 口时两半段重新连成一段。</summary>
        public static void MergeDegenerate(RoadGraph g, string intersectionId)
        {
            if (!(g.GetNode(intersectionId) is Intersection ix)) return;
            var ports = LivePorts(g, ix);
            if (ports.Count >= 3) return;

            if (ports.Count == 2)
            {
                var p1 = ports[0];
                var p2 = ports[1];
                string n1 = OtherNode(p1, intersectionId);
                string n2 = OtherNode(p2, intersectionId);
                var seg1 = g.GetNode(n1) as RoadSegment;
                var seg2 = g.GetNode(n2) as RoadSegment;
                if (seg1 != null && seg2 != null && seg1.Id != seg2.Id
                    && seg1.PortIds.Count == 2 && seg2.PortIds.Count == 2)
                {
                    // p1 须为某段终点（PortIds[1]）、p2 为另一段起点（PortIds[0]）
                    bool forward = seg1.PortIds[1] == p1.Id && seg2.PortIds[0] == p2.Id;
                    bool backward = seg2.PortIds[1] == p1.Id && seg1.PortIds[0] == p2.Id;
                    if (forward || backward)
                    {
                        var aSeg = forward ? seg1 : seg2; // p1 = 其终点
                        var bSeg = forward ? seg2 : seg1; // p2 = 其起点
                        var pa = g.GetProfile(aSeg.PortIds[0]);
                        var pb = g.GetProfile(bSeg.PortIds[1]);
                        string roadId = aSeg.RoadId ?? bSeg.RoadId;
                        var affected = new List<string> { intersectionId, aSeg.Id, bSeg.Id };
                        g.RemoveSegment(aSeg.Id);
                        g.RemoveSegment(bSeg.Id);
                        g.Nodes.Remove(intersectionId);
                        g.Profiles.Remove(p1.Id);
                        g.Profiles.Remove(p2.Id);
                        var merged = g.AddSegment(pa, pb, roadId);
                        if (merged != null) affected.Add(merged.Id);
                        g.NotifyChanged(affected);
                        return;
                    }
                }
            }

            // 兜底：拆除路口节点，ports 回归悬空端
            g.Nodes.Remove(intersectionId);
            foreach (var p in ports) DetachFromNode(p, intersectionId);
            g.NotifyChanged(new List<string> { intersectionId });
        }

        /// <summary>路口当前仍接通的 ports（profile 存在，且另一侧仍挂着非本路口的节点）；
        /// 半段被删除后的悬空 port 不计入（供退化判定 / 配对 / mesh 重建共用）。</summary>
        public static List<Profile> LivePorts(RoadGraph g, Intersection ix)
        {
            var ports = new List<Profile>();
            foreach (var pid in ix.PortIds)
            {
                var p = g.GetProfile(pid);
                if (p == null) continue;
                if (OtherNode(p, ix.Id) == null) continue; // 其入射半段已被删除
                ports.Add(p);
            }
            return ports;
        }
    }
}
