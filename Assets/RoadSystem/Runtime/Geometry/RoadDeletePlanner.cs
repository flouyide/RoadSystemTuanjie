using System;
using System.Collections.Generic;
using System.Linq;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    public sealed class RoadDeletePlan
    {
        public string SegmentId { get; internal set; }
        public RoadBuildPlan Edit { get; internal set; }
        public bool IsCurrent(RoadGraph graph, RoadBuildSettings settings) => Edit.IsCurrent(graph, settings);
    }

    /// <summary>Plans one segment removal, including both junctions, without changing the source graph.</summary>
    public static class RoadDeletePlanner
    {
        public static bool TryPlan(RoadGraph source, string segmentId, RoadBuildSettings settings,
            out RoadDeletePlan plan, out string reason)
        {
            plan = null; reason = null;
            if (source == null || !(source.GetNode(segmentId) is RoadSegment))
                return Fail("请选择仍然存在的路段", out reason);
            var graph = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(source));
            foreach (var n in source.Nodes.Values) graph.GetNode(n.Id).DirtyVersion = n.DirtyVersion;
            foreach (var s in source.Segments())
                if (s.Path != null) ((RoadSegment)graph.GetNode(s.Id)).ExplicitPath = SegmentPathResolver.Store(s.Path);
            var edit = new RoadBuildPlan { Source = source, SourceRevision = source.Revision,
                Result = graph, Settings = settings };
            Action<IReadOnlyList<string>> track = ids => { foreach (var id in ids) edit.ChangedNodes.Add(id); };
            graph.Changed += track;
            try
            {
                var segment = (RoadSegment)graph.GetNode(segmentId);
                var affectedPorts = new HashSet<string>(segment.PortIds);
                var intersections = new HashSet<string>();
                foreach (var pid in segment.PortIds)
                {
                    var p = graph.GetProfile(pid);
                    if (p == null) return Fail("路段端口数据不完整", out reason);
                    foreach (var id in new[] { p.NodeAId, p.NodeBId })
                        if (graph.GetNode(id) is Intersection) intersections.Add(id);
                }
                graph.RemoveSegment(segmentId);
                foreach (var id in intersections)
                {
                    var ix = (Intersection)graph.GetNode(id);
                    foreach (var pid in ix.PortIds.ToArray())
                    {
                        affectedPorts.Add(pid);
                        var p = graph.GetProfile(pid);
                        if (Incident(graph, ix, p) != null) continue;
                        ix.PortIds.Remove(pid);
                        Detach(p, ix.Id);
                    }
                    var ports = ix.PortIds.Select(graph.GetProfile).ToList();
                    if (ports.Count >= 3)
                    {
                        if (!Shrink(graph, ix, settings, out reason)) return false;
                    }
                    else if (ports.Count == 2)
                    {
                        if (!Bridge(graph, ix, ports, settings, out reason)) return false;
                    }
                    else
                    {
                        foreach (var p in ports)
                        {
                            var other = Incident(graph, ix, p);
                            Detach(p, ix.Id);
                            if (other != null) edit.ChangedNodes.Add(other.Id);
                        }
                        graph.Nodes.Remove(ix.Id);
                    }
                    graph.NotifyChanged(new[] { ix.Id });
                }
                foreach (var pid in affectedPorts) graph.RemoveLooseProfile(graph.GetProfile(pid));
                if (!ValidReferences(graph)) return Fail("删除后路网引用不完整", out reason);
                if (!RoadBuildPlanner.Validate(edit, out reason)) return false;
                plan = new RoadDeletePlan { SegmentId = segmentId, Edit = edit };
                return true;
            }
            finally { graph.Changed -= track; }
        }

        static RoadSegment Incident(RoadGraph graph, Intersection ix, Profile p)
        {
            if (p == null || (p.NodeAId != ix.Id && p.NodeBId != ix.Id)) return null;
            var other = graph.GetNode(p.NodeAId == ix.Id ? p.NodeBId : p.NodeAId) as RoadSegment;
            return other != null && other.PortIds.Contains(p.Id) ? other : null;
        }

        static void Detach(Profile p, string id)
        {
            if (p == null) return;
            if (p.NodeAId == id) p.NodeAId = null;
            if (p.NodeBId == id) p.NodeBId = null;
        }

        static bool Shrink(RoadGraph graph, Intersection ix, RoadBuildSettings settings, out string reason)
        {
            reason = null;
            var ports = ix.PortIds.Select(graph.GetProfile).ToList();
            if (!ix.HasAnchor)
            { ix.HasAnchor = true; ix.Anchor = IntersectionBuilder.CenterOf(ports); ix.AnchorY = ports.Average(p => p.Y); }
            bool settled = false;
            for (int attempt = 0; attempt < 4; attempt++)
            {
                var loop = RoadBuildPlanner.Boundary(graph, ix, settings);
                var polygon = RoadFootprint.Points(loop);
                var triangles = new List<int>();
                if (!RoadFootprint.IsSimple(polygon) || !PolygonTriangulator.TryTriangulate(loop, triangles))
                    return Fail("剩余路口边界无法生成", out reason);
                if (RoadFootprint.Contains(polygon, ix.Anchor)) { settled = true; break; }
                // An interior triangle centroid remains valid even for a concave pavement.
                ix.Anchor = (loop[triangles[0]].Pos + loop[triangles[1]].Pos + loop[triangles[2]].Pos) / 3;
                ix.AnchorY = (loop[triangles[0]].Y + loop[triangles[1]].Y + loop[triangles[2]].Y) / 3;
            }
            if (!settled) return Fail("剩余路口无法确定有效中心", out reason);
            var retained = ix.LaneLinks.Where(l => ValidLink(graph, ix, l)).ToList();
            IntersectionBuilder.BuildLaneLinks(graph, ix);
            ix.LaneLinks = retained.Concat(ix.LaneLinks)
                .GroupBy(l => $"{l.FromPortId}:{l.FromLane}>{l.ToPortId}:{l.ToLane}").Select(g => g.First()).ToList();
            IntersectionBuilder.RefreshLanePaths(graph, ix);
            if (ix.LaneLinks.Any(l => l.Path == null)) return Fail("剩余路口车道连接无法求解", out reason);
            return true;
        }

        static bool ValidLink(RoadGraph graph, Intersection ix, LaneConnection link)
            => link.FromPortId != link.ToPortId && LaneFlows(graph, ix, link.FromPortId, link.FromLane, true)
                && LaneFlows(graph, ix, link.ToPortId, link.ToLane, false);

        static bool LaneFlows(RoadGraph graph, Intersection ix, string pid, int lane, bool incoming)
        {
            if (!ix.PortIds.Contains(pid)) return false;
            var p = graph.GetProfile(pid); var seg = Incident(graph, ix, p);
            return seg != null && p.Lanes != null && lane >= 0 && lane < p.Lanes.Length
                && p.Lanes[lane].Type == LaneType.Car
                && ((seg.PortIds[1] == pid) == p.Lanes[lane].Forward) == incoming;
        }

        static bool Bridge(RoadGraph graph, Intersection ix, List<Profile> ports,
            RoadBuildSettings settings, out string reason)
        {
            reason = "剩余道路无法平顺连接";
            var a = ports[0]; var b = ports[1];
            var sa = Incident(graph, ix, a); var sb = Incident(graph, ix, b);
            if (sa == null || sb == null || sa == sb) return false;
            if (sa.PortIds[0] == a.Id && sb.PortIds[1] == b.Id)
            { var swap = a; a = b; b = swap; var ss = sa; sa = sb; sb = ss; }
            // Port order is a storage choice, not a reason to leave two physical roads disconnected.
            if (sa.PortIds[1] != a.Id && !ReverseRoadChain(graph, sa, sb.Id, ix.Id)) return false;
            if (sb.PortIds[0] != b.Id && !ReverseRoadChain(graph, sb, sa.Id, ix.Id)) return false;
            if (!a.HasSameLayout(b)) return Fail("剩余道路的车道断面不一致", out reason);
            for (int i = 0; i < a.Lanes.Length; i++)
                if ((a.Lanes[i].Type == LaneType.Car && a.Lanes[i].Forward != b.Lanes[i].Forward)
                    || Math.Abs(a.Lanes[i].CurbHeight - b.Lanes[i].CurbHeight) > 1e-4)
                    return Fail("剩余道路的车道方向或路缘高度不兼容", out reason);
            var originalA = SegmentPathResolver.Resolve(graph, sa);
            var originalB = SegmentPathResolver.Resolve(graph, sb);
            if (originalA == null || originalB == null) return false;
            // Start at existing ports; only consume approach road when a safe turn needs more room.
            double maxA = Math.Min(128, originalA.TotalLength - 2);
            double maxB = Math.Min(128, originalB.TotalLength - 2);
            for (double retreat = 0; retreat <= Math.Max(maxA, maxB) + 1; retreat += 1)
            {
                double cutA = Math.Min(retreat, Math.Max(0, maxA)), cutB = Math.Min(retreat, Math.Max(0, maxB));
                var remainingA = cutA == 0 ? originalA : PathChainUtil.SubPath(originalA, 0, originalA.TotalLength - cutA);
                var remainingB = cutB == 0 ? originalB : PathChainUtil.SubPath(originalB, cutB, originalB.TotalLength);
                if (remainingA == null || remainingB == null) continue;
                var start = Profile.Create(remainingA.EndPoint, remainingA.EndTangent, a.Lanes);
                var end = Profile.Create(remainingB.StartPoint, remainingB.StartTangent, b.Lanes);
                var path = ProfileConnector.Connect(start, end);
                if (!FitsBridge(graph, ix, sa, sb, start, end, path, remainingA, remainingB, settings)) continue;
                // Replace both approaches and the connector with ONE authoritative segment.
                // Retain the outer profiles so their remote roads/intersections stay connected.
                var outerA = graph.GetProfile(sa.PortIds[0]);
                var outerB = graph.GetProfile(sb.PortIds[1]);
                var mergedPath = new PathChain();
                mergedPath.Append(remainingA); mergedPath.Append(path); mergedPath.Append(remainingB);
                graph.RemoveSegment(sa.Id);
                graph.RemoveSegment(sb.Id);
                Detach(a, ix.Id); Detach(b, ix.Id);
                graph.Nodes.Remove(ix.Id);
                graph.RemoveLooseProfile(a); graph.RemoveLooseProfile(b);
                // No RoadId group is needed: the merged road owns a single mesh GameObject.
                var merged = graph.AddSegment(outerA, outerB);
                if (merged == null) return Fail("剩余道路的远端无法合并", out reason);
                merged.Path = mergedPath;
                merged.ExplicitPath = SegmentPathResolver.Store(mergedPath);
                graph.NotifyChanged(new[] { sa.Id, sb.Id, ix.Id, merged.Id });
                reason = null;
                return true;
            }
            return Fail("剩余道路没有足够空间生成无重叠的弯路", out reason);
        }

        static bool FitsBridge(RoadGraph graph, Intersection ix, RoadSegment sa, RoadSegment sb,
            Profile a, Profile b, PathChain path, PathChain remainingA, PathChain remainingB, RoadBuildSettings settings)
        {
            if (path == null || path.TotalLength < .01 || DVec2.Dot(path.StartTangent, a.Direction) < .99999
                || DVec2.Dot(path.EndTangent, b.Direction) < .99999) return false;
            if (path.Elements.OfType<ArcElement>().Any(arc => arc.Radius <= a.TotalWidth * .5 + .01)) return false;
            var footprint = RoadFootprint.Sweep(path, a.TotalWidth, settings.Sagitta);
            double tolerance = Math.Max(.015, settings.Sagitta * 2.1);
            if (!RoadFootprint.IsSimple(footprint)) return false;
            foreach (var road in graph.Segments())
            {
                var remaining = road == sa ? remainingA : road == sb ? remainingB : SegmentPathResolver.Resolve(graph, road);
                if (remaining == null || RoadFootprint.Overlaps(footprint,
                    RoadFootprint.Sweep(remaining, graph.GetProfile(road.PortIds[0]).TotalWidth, settings.Sagitta), tolerance)) return false;
            }
            foreach (var other in graph.Nodes.Values.OfType<Intersection>())
                if (other != ix && RoadFootprint.Overlaps(footprint,
                    RoadFootprint.Points(RoadBuildPlanner.Boundary(graph, other, settings)), tolerance)) return false;
            return true;
        }

        // Reverse an entire chain up to loose ends/intersections so shared profiles remain coherent.
        // Positions, physical lane directions, IDs and exact circles are preserved.
        internal static bool ReverseRoadChain(RoadGraph graph, RoadSegment first, string forbiddenSegment, string removedIntersection)
        {
            var paths = new Dictionary<RoadSegment, PathChain>();
            var profiles = new HashSet<Profile>();
            var queue = new Queue<RoadSegment>(); queue.Enqueue(first);
            while (queue.Count > 0)
            {
                var segment = queue.Dequeue();
                if (segment.Id == forbiddenSegment) return false;
                if (paths.ContainsKey(segment)) continue;
                var reversed = PathChainUtil.Reverse(SegmentPathResolver.Resolve(graph, segment));
                if (reversed == null) return false;
                paths.Add(segment, reversed);
                foreach (var pid in segment.PortIds)
                {
                    var p = graph.GetProfile(pid);
                    if (p == null || p.Lanes == null) return false;
                    profiles.Add(p);
                    foreach (var id in new[] { p.NodeAId, p.NodeBId })
                        if (id != segment.Id && graph.GetNode(id) is RoadSegment next) queue.Enqueue(next);
                }
            }
            var changed = new HashSet<string>();
            foreach (var p in profiles)
            {
                p.Direction = -p.Direction;
                p.Lanes = p.Lanes.Reverse().Select(l => new LaneDef(l.Type, l.Width,
                    l.Type == LaneType.Car ? !l.Forward : l.Forward, l.CurbHeight)).ToArray();
                foreach (var id in new[] { p.NodeAId, p.NodeBId })
                    if (id != removedIntersection && graph.GetNode(id) is Intersection other)
                    {
                        foreach (var link in other.LaneLinks)
                        {
                            if (link.FromPortId == p.Id) link.FromLane = p.Lanes.Length - 1 - link.FromLane;
                            if (link.ToPortId == p.Id) link.ToLane = p.Lanes.Length - 1 - link.ToLane;
                        }
                        changed.Add(id);
                    }
            }
            foreach (var pair in paths)
            {
                pair.Key.PortIds.Reverse();
                pair.Key.Path = pair.Value;
                pair.Key.ExplicitPath = SegmentPathResolver.Store(pair.Value);
                changed.Add(pair.Key.Id);
            }
            graph.NotifyChanged(changed.ToList());
            return true;
        }

        internal static bool ValidReferences(RoadGraph graph)
        {
            foreach (var node in graph.Nodes.Values)
            {
                if (node is RoadSegment && node.PortIds.Count != 2) return false;
                if (node is Intersection && node.PortIds.Count < 3) return false;
                if (node.PortIds.Distinct().Count() != node.PortIds.Count) return false;
                foreach (var pid in node.PortIds)
                {
                    var p = graph.GetProfile(pid);
                    if (p == null || (p.NodeAId != node.Id && p.NodeBId != node.Id)) return false;
                }
            }
            foreach (var p in graph.Profiles.Values)
                foreach (var id in new[] { p.NodeAId, p.NodeBId })
                    if (!string.IsNullOrEmpty(id) && (graph.GetNode(id) == null || !graph.GetNode(id).PortIds.Contains(p.Id))) return false;
            return true;
        }

        static bool Fail(string text, out string reason) { reason = text; return false; }
    }
}
