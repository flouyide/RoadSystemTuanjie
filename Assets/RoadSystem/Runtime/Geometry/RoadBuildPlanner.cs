using System;
using System.Collections.Generic;
using System.Linq;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    public struct RoadBuildSettings : IEquatable<RoadBuildSettings>
    {
        public double Sagitta, CornerRadius, Curve;
        public bool Equals(RoadBuildSettings other) => Sagitta == other.Sagitta
            && CornerRadius == other.CornerRadius && Curve == other.Curve;
        public static RoadBuildSettings Default => new RoadBuildSettings { Sagitta = .05, CornerRadius = 5 };
    }

    /// <summary>A private graph transaction. Preview and commit consume the same geometry.</summary>
    public sealed class RoadBuildPlan
    {
        public RoadGraph Source { get; internal set; }
        public long SourceRevision { get; internal set; }
        public RoadGraph Result { get; internal set; }
        public RoadBuildSettings Settings { get; internal set; }
        public string EndProfileId { get; internal set; }
        public readonly HashSet<string> ChangedNodes = new HashSet<string>();
        public readonly Dictionary<string, List<BoundaryVertex>> Boundaries = new Dictionary<string, List<BoundaryVertex>>();
        public bool IsCurrent(RoadGraph graph, RoadBuildSettings settings)
            => ReferenceEquals(Source, graph) && SourceRevision == graph.Revision && Settings.Equals(settings);
    }

    public static class RoadBuildPlanner
    {
        const double MinLength = 2;
        const double HeightTolerance = 1;

        public static List<BoundaryVertex> Boundary(RoadGraph graph, Intersection ix, RoadBuildSettings settings)
        {
            var ports = IntersectionBuilder.LivePorts(graph, ix);
            return IntersectionBuilder.BuildBoundaryLoop(ports,
                ix.HasAnchor ? ix.Anchor : IntersectionBuilder.CenterOf(ports),
                settings.Sagitta, settings.CornerRadius, settings.Curve);
        }

        // Legacy graphs acquire an interior anchor on the transaction copy only.
        static bool Anchor(RoadGraph graph, Intersection ix, RoadBuildSettings settings,
            out DVec2 position, out float y)
        {
            var ports = IntersectionBuilder.LivePorts(graph, ix);
            position = ix.HasAnchor ? ix.Anchor : IntersectionBuilder.CenterOf(ports);
            y = ix.HasAnchor ? ix.AnchorY : (ports.Count == 0 ? 0 : ports.Average(p => p.Y));
            var loop = Boundary(graph, ix, settings);
            var points = RoadFootprint.Points(loop);
            if (RoadFootprint.Contains(points, position)) return true;
            if (ix.HasAnchor) return false;
            var triangles = new List<int>();
            if (!PolygonTriangulator.TryTriangulate(loop, triangles) || triangles.Count < 3) return false;
            position = (loop[triangles[0]].Pos + loop[triangles[1]].Pos + loop[triangles[2]].Pos) / 3;
            return true;
        }

        public static bool TrySnapIntersection(RoadGraph graph, DVec2 point, float y,
            RoadBuildSettings settings, out string id, out DVec2 position, out float height)
        {
            id = null; position = point; height = y;
            double best = double.PositiveInfinity;
            foreach (var ix in graph.Nodes.Values.OfType<Intersection>().OrderBy(n => n.Id))
            {
                if (!Anchor(graph, ix, settings, out var anchor, out var anchorY)
                    || Math.Abs(y - anchorY) > HeightTolerance) continue;
                var polygon = RoadFootprint.Points(Boundary(graph, ix, settings));
                if (!RoadFootprint.Contains(polygon, point, 2)) continue;
                double distance = DVec2.Distance(anchor, point);
                if (distance >= best) continue;
                best = distance; id = ix.Id; position = anchor; height = anchorY;
            }
            return id != null;
        }

        public static bool TryPlan(RoadGraph source, Profile start, Profile end, PathChain path,
            string roadId, string startIntersection, string endIntersection, RoadBuildSettings settings,
            out RoadBuildPlan plan, out string reason)
        {
            plan = null; reason = null;
            if (source == null || start == null || end == null)
                return Fail("道路两端断面不一致", out reason);
            string startExtension = source.GetProfile(start.Id) != null ? start.Id : null;
            string endExtension = source.GetProfile(end.Id) != null ? end.Id : null;
            var orientedStart = startExtension != null ? EndpointExtension.View(source, startExtension, true) : null;
            var orientedEnd = endExtension != null ? EndpointExtension.View(source, endExtension, false) : null;
            if ((startExtension != null && (orientedStart == null || DVec2.Distance(start.Position, orientedStart.Position) > 1e-4
                    || Math.Abs(start.Y - orientedStart.Y) > 1e-4))
                || (endExtension != null && (orientedEnd == null || DVec2.Distance(end.Position, orientedEnd.Position) > 1e-4
                    || Math.Abs(end.Y - orientedEnd.Y) > 1e-4)))
                return Fail("道路端点已被占用或改变", out reason);
            if (start.Id == end.Id || EndpointExtension.SameChain(source, startExtension, endExtension))
                return Fail("暂不支持连接同一道路的两端形成闭环", out reason);
            start = orientedStart ?? Draft(start, orientedEnd?.Lanes);
            end = orientedEnd ?? Draft(end, orientedStart?.Lanes);
            bool extending = startExtension != null || endExtension != null;
            if (!EndpointExtension.Compatible(start, end)) return Fail("道路两端断面或行车方向不兼容", out reason);
            if (extending && Math.Abs(start.Y - end.Y) > 1e-4)
                return Fail("端点延伸仅支持同一高度的道路", out reason);
            if (startIntersection != null && startIntersection == endIntersection)
                return Fail("暂不支持道路两端连接同一路口", out reason);
            path = path ?? ProfileConnector.Connect(start, end);
            if (path == null || path.TotalLength <= 1e-6)
                return Fail("道路路径无法求解或长度不足", out reason);
            if (DVec2.Distance(path.StartPoint, start.Position) > .01
                || DVec2.Distance(path.EndPoint, end.Position) > .01)
                return Fail("道路路径与端点不一致", out reason);
            if (extending && (DVec2.Distance(path.StartPoint, start.Position) > 1e-4
                || DVec2.Distance(path.EndPoint, end.Position) > 1e-4
                || (startExtension != null && DVec2.Dot(path.StartTangent, start.Direction) < EndpointExtension.TangentDot)
                || (endExtension != null && DVec2.Dot(path.EndTangent, end.Direction) < EndpointExtension.TangentDot)))
                return Fail("连接路径与已有道路切线不连续", out reason);
            if (extending && path.Elements.OfType<ArcElement>().Any(arc => arc.Radius <= start.TotalWidth * .5 + .01))
                return Fail("延伸弯路的内缘半径不足", out reason);
            if (extending && !EndpointExtension.Continuous(path)) return Fail("延伸路径存在折角或断口", out reason);

            var copy = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(source));
            foreach (var node in source.Nodes.Values) copy.GetNode(node.Id).DirtyVersion = node.DirtyVersion;
            // Old programmatic graphs may only have a cached exact path.
            foreach (var s in source.Segments())
                if (s.Path != null && (s.ExplicitPath == null || s.ExplicitPath.Count == 0)
                    && copy.GetNode(s.Id) is RoadSegment target)
                    target.ExplicitPath = SegmentPathResolver.Store(s.Path);
            var transaction = new RoadBuildPlan { Source = source, SourceRevision = source.Revision,
                Result = copy, Settings = settings };
            Action<IReadOnlyList<string>> track = ids => { foreach (var id in ids) transaction.ChangedNodes.Add(id); };
            copy.Changed += track;
            try
            {
                EndpointExtension.TryGetRoad(copy, startExtension, out var oldStart);
                EndpointExtension.TryGetRoad(copy, endExtension, out var oldEnd);
                string mergedRoadId = oldStart == null ? oldEnd?.RoadId : oldEnd == null || oldStart.RoadId == oldEnd.RoadId
                    ? oldStart.RoadId : Guid.NewGuid().ToString("N");
                if (!EndpointExtension.Normalize(copy, startExtension, true, oldEnd?.Id)
                    || !EndpointExtension.Normalize(copy, endExtension, false, oldStart?.Id))
                    return Fail("道路方向无法安全规范化", out reason);
                var originalBoundaries = new Dictionary<string, List<DVec2>>();

                foreach (var ix in copy.Nodes.Values.OfType<Intersection>())
                {
                    bool atStart = ix.Id == startIntersection, atEnd = ix.Id == endIntersection;
                    var polygon = RoadFootprint.Points(Boundary(copy, ix, settings));
                    originalBoundaries[ix.Id] = polygon;
                    if (atStart || atEnd)
                    {
                        var endpoint = atStart ? start : end;
                        if (!Anchor(copy, ix, settings, out var anchor, out var y)
                            || !RoadFootprint.Contains(polygon, endpoint.Position, .01)
                            || Math.Abs(endpoint.Y - y) > HeightTolerance)
                            return Fail("路口吸附位置已失效", out reason);
                        ix.HasAnchor = true; ix.Anchor = anchor; ix.AnchorY = y;
                        // Only a contiguous endpoint portion may enter the existing junction.
                        var samples = PathSampler.Sample(path, settings.Sagitta);
                        if (atEnd) samples.Reverse();
                        bool exited = false;
                        foreach (var sample in samples)
                        {
                            bool inside = RoadFootprint.Contains(polygon, sample.Position);
                            if (inside && exited) return Fail("道路中段再次穿入已有路口", out reason);
                            if (!inside) exited = true;
                        }
                    }
                    else if (RoadFootprint.Overlaps(polygon, RoadFootprint.Sweep(path, start.TotalWidth, settings.Sagitta), .01))
                        return Fail("道路中段穿越或擦碰已有路口，请将端点吸附到路口", out reason);
                }
                if ((startIntersection != null && !(copy.GetNode(startIntersection) is Intersection))
                    || (endIntersection != null && !(copy.GetNode(endIntersection) is Intersection)))
                    return Fail("目标路口已不存在", out reason);

                var a = Endpoint(copy, start, startIntersection != null);
                var b = Endpoint(copy, end, endIntersection != null);
                if (a == null || b == null) return Fail("道路端口已被占用或改变", out reason);
                // A fresh detection group prevents legacy same-RoadId exclusions hiding old-road collisions.
                var segment = copy.AddSegment(a, b, extending ? Guid.NewGuid().ToString("N") : roadId ?? Guid.NewGuid().ToString("N"));
                if (segment == null) return Fail("道路端口无法连接", out reason);
                segment.Path = SegmentPathResolver.Restore(SegmentPathResolver.Store(path));
                segment.ExplicitPath = SegmentPathResolver.Store(segment.Path);
                transaction.EndProfileId = b.Id;
                if (startIntersection != null && !Grow(copy, (Intersection)copy.GetNode(startIntersection), a,
                    originalBoundaries[startIntersection], settings, out reason)) return false;
                if (endIntersection != null && !Grow(copy, (Intersection)copy.GetNode(endIntersection), b,
                    originalBoundaries[endIntersection], settings, out reason)) return false;

                IntersectionBuilder.DetectAndBuild(copy, segment.Id);
                string finalEnd = transaction.EndProfileId;
                if (!EndpointExtension.MergeAt(copy, startExtension, ref finalEnd, mergedRoadId)
                    || !EndpointExtension.MergeAt(copy, endExtension, ref finalEnd, mergedRoadId))
                    return Fail("端点接缝无法合并为连续道路", out reason);
                transaction.EndProfileId = EndpointExtension.TryGetRoad(copy, finalEnd, out _) ? finalEnd : null;
                if (!RoadDeletePlanner.ValidReferences(copy)) return Fail("合并后的道路引用不完整", out reason);
                if (!Validate(transaction, out reason)) return false;
                plan = transaction;
                return true;
            }
            finally { copy.Changed -= track; }
        }

        static Profile Draft(Profile source, LaneDef[] inherited)
        {
            var result = Profile.Create(source.Position, source.Direction, (LaneDef[])(inherited ?? source.Lanes).Clone());
            result.Id = source.Id; result.Y = source.Y; return result;
        }

        static Profile Endpoint(RoadGraph graph, Profile candidate, bool intersection)
        {
            var live = intersection ? null : graph.GetProfile(candidate.Id);
            if (live != null)
            {
                if (!EndpointExtension.TryGetRoad(graph, live.Id, out _) || !live.Position.ApproxEquals(candidate.Position)
                    || !live.Direction.ApproxEquals(candidate.Direction) || !live.HasSameLayout(candidate)) return null;
                return live;
            }
            var created = graph.CreateProfile(candidate.Position, candidate.Direction, (LaneDef[])candidate.Lanes.Clone());
            created.Y = candidate.Y;
            return created;
        }

        sealed class Arm
        {
            public Profile Port, Proposed;
            public RoadSegment Segment;
            public PathChain Path, Remaining, Removed;
            public bool AtStart;
            public float Y0, Y1;
        }

        static bool Grow(RoadGraph graph, Intersection ix, Profile incoming, List<DVec2> oldPolygon,
            RoadBuildSettings settings, out string reason)
        {
            reason = null;
            var ports = IntersectionBuilder.LivePorts(graph, ix);
            ports.Add(incoming);
            var arms = new List<Arm>();
            foreach (var p in ports)
            {
                var seg = graph.GetNode(p.NodeAId == ix.Id ? p.NodeBId : p.NodeAId) as RoadSegment;
                if (seg == null || seg.PortIds.Count != 2 || arms.Any(a => a.Segment == seg))
                    return Fail("路口相邻道路拓扑不支持扩口", out reason);
                var path = SegmentPathResolver.Resolve(graph, seg);
                if (path == null || path.TotalLength < MinLength) return Fail("相邻道路长度不足", out reason);
                arms.Add(new Arm { Port = p, Segment = seg, Path = path, AtStart = seg.PortIds[0] == p.Id,
                    Y0 = graph.GetProfile(seg.PortIds[0]).Y, Y1 = graph.GetProfile(seg.PortIds[1]).Y });
            }
            double radius = Math.Max(8, ports.Where(p => p != incoming).Max(p => DVec2.Distance(p.Position, ix.Anchor)));
            double tolerance = Math.Max(.015, settings.Sagitta * 2.1);
            // Bounded outward search. Curved arms need not become more feasible at every step.
            for (int attempt = 0; attempt < 64; attempt++, radius += 2)
            {
                bool fits = true;
                foreach (var arm in arms)
                {
                    double retreat = 0, limit = Math.Min(128, arm.Path.TotalLength - MinLength);
                    DVec2 pos = arm.Port.Position, tangent = arm.Port.Direction;
                    while (retreat <= limit + 1e-6)
                    {
                        double s = arm.AtStart ? retreat : arm.Path.TotalLength - retreat;
                        PathChainUtil.PointAt(arm.Path, s, out pos, out tangent);
                        if (DVec2.Distance(pos, ix.Anchor) >= radius - 1e-6) break;
                        retreat += .5;
                    }
                    if (retreat > limit + 1e-6) { fits = false; break; }
                    double cut = arm.AtStart ? retreat : arm.Path.TotalLength - retreat;
                    arm.Proposed = Profile.Create(pos, tangent, arm.Port.Lanes);
                    arm.Proposed.Id = arm.Port.Id;
                    arm.Proposed.Y = (float)(arm.Y0 + (arm.Y1 - arm.Y0) * cut / arm.Path.TotalLength);
                    arm.Remaining = PathChainUtil.SubPath(arm.Path, arm.AtStart ? cut : 0,
                        arm.AtStart ? arm.Path.TotalLength : cut);
                    arm.Removed = retreat < 1e-6 ? null : PathChainUtil.SubPath(arm.Path,
                        arm.AtStart ? 0 : cut, arm.AtStart ? cut : arm.Path.TotalLength);
                }
                if (!fits) continue;
                var loop = IntersectionBuilder.BuildBoundaryLoop(arms.Select(a => a.Proposed).ToList(), ix.Anchor,
                    settings.Sagitta, settings.CornerRadius, settings.Curve);
                var polygon = RoadFootprint.Points(loop);
                if (!RoadFootprint.IsSimple(polygon) || !PolygonTriangulator.TryTriangulate(loop, new List<int>())
                    || !RoadFootprint.Covers(polygon, oldPolygon, tolerance)) continue;
                foreach (var arm in arms)
                {
                    if ((arm.Removed != null && !RoadFootprint.Covers(polygon,
                        RoadFootprint.Sweep(arm.Removed, arm.Port.TotalWidth, settings.Sagitta), tolerance))
                        || RoadFootprint.Overlaps(polygon,
                            RoadFootprint.Sweep(arm.Remaining, arm.Port.TotalWidth, settings.Sagitta), tolerance))
                    { fits = false; break; }
                }
                if (!fits) continue;
                foreach (var other in graph.Nodes.Values.OfType<Intersection>())
                    if (other.Id != ix.Id && RoadFootprint.Overlaps(polygon,
                        RoadFootprint.Points(Boundary(graph, other, settings)), tolerance)) { fits = false; break; }
                if (!fits) continue;
                foreach (var other in graph.Segments())
                {
                    if (arms.Any(a => a.Segment == other)) continue;
                    var otherPath = SegmentPathResolver.Resolve(graph, other);
                    if (otherPath != null && RoadFootprint.Overlaps(polygon, RoadFootprint.Sweep(otherPath,
                        graph.GetProfile(other.PortIds[0]).TotalWidth, settings.Sagitta), tolerance)) { fits = false; break; }
                }
                if (!fits) continue;

                // MoveProfile invalidates exact paths; restore all snapshots after all moves.
                foreach (var arm in arms)
                {
                    graph.MoveProfile(arm.Port, arm.Proposed.Position, arm.Proposed.Direction);
                    arm.Port.Y = arm.Proposed.Y;
                }
                foreach (var arm in arms)
                {
                    arm.Segment.Path = arm.Remaining;
                    arm.Segment.ExplicitPath = SegmentPathResolver.Store(arm.Remaining);
                }
                ix.PortIds.Add(incoming.Id);
                if (string.IsNullOrEmpty(incoming.NodeAId)) incoming.NodeAId = ix.Id;
                else incoming.NodeBId = ix.Id;
                var retained = ix.LaneLinks.ToList();
                IntersectionBuilder.BuildLaneLinks(graph, ix);
                var added = ix.LaneLinks.Where(l => l.FromPortId == incoming.Id || l.ToPortId == incoming.Id);
                ix.LaneLinks = retained.Concat(added).GroupBy(LinkKey).Select(g => g.First()).ToList();
                IntersectionBuilder.RefreshLanePaths(graph, ix);
                if (ix.LaneLinks.Any(l => l.Path == null)) return Fail("扩口后的车道连接无法求解", out reason);
                graph.NotifyChanged(new[] { ix.Id });
                return true;
            }
            return Fail("路口扩建空间不足、夹角过小或与相邻道路冲突", out reason);
        }

        static string LinkKey(LaneConnection l) => $"{l.FromPortId}:{l.FromLane}>{l.ToPortId}:{l.ToLane}";

        internal static bool Validate(RoadBuildPlan plan, out string reason)
        {
            reason = null;
            var graph = plan.Result;
            var settings = plan.Settings;
            double tolerance = Math.Max(.015, settings.Sagitta * 2.1);
            var roads = new Dictionary<string, List<DVec2>>();
            foreach (var seg in graph.Segments())
            {
                var path = SegmentPathResolver.Resolve(graph, seg);
                if (path == null) return Fail("路段路径无法求解", out reason);
                roads[seg.Id] = RoadFootprint.Sweep(path, graph.GetProfile(seg.PortIds[0]).TotalWidth, settings.Sagitta);
                if (plan.ChangedNodes.Contains(seg.Id) && !RoadFootprint.IsSimple(roads[seg.Id]))
                    return Fail("道路边缘自交", out reason);
            }
            foreach (var ix in graph.Nodes.Values.OfType<Intersection>())
            {
                var loop = Boundary(graph, ix, settings);
                plan.Boundaries[ix.Id] = loop;
                if (!plan.ChangedNodes.Contains(ix.Id)) continue;
                var polygon = RoadFootprint.Points(loop);
                if (!RoadFootprint.IsSimple(polygon) || !PolygonTriangulator.TryTriangulate(loop, new List<int>()))
                    return Fail("路口边界自交或无法完整三角化", out reason);
                IntersectionBuilder.RefreshLanePaths(graph, ix);
                if (ix.LaneLinks.Any(l => l.Path == null)) return Fail("路口车道连接无法求解", out reason);
                foreach (var road in roads)
                    if (RoadFootprint.Overlaps(polygon, road.Value, tolerance))
                        return Fail("路口与剩余道路相互覆盖", out reason);
                foreach (var other in graph.Nodes.Values.OfType<Intersection>())
                    if (other.Id != ix.Id && RoadFootprint.Overlaps(polygon,
                        RoadFootprint.Points(Boundary(graph, other, settings)), tolerance))
                        return Fail("路口区域与其他路口重叠", out reason);
            }
            var segments = graph.Segments().ToList();
            for (int i = 0; i < segments.Count; i++)
                for (int j = i + 1; j < segments.Count; j++)
                {
                    var a = segments[i]; var b = segments[j];
                    if (!plan.ChangedNodes.Contains(a.Id) && !plan.ChangedNodes.Contains(b.Id)) continue;
                    if (RoadFootprint.Overlaps(roads[a.Id], roads[b.Id], tolerance))
                        return Fail("道路交汇处无法留出路口空间", out reason);
                }
            return true;
        }

        static bool Fail(string message, out string reason) { reason = message; return false; }
    }
}
