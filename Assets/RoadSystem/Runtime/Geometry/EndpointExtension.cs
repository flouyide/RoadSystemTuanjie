using System;
using System.Collections.Generic;
using System.Linq;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>Endpoint identity, oriented cross sections and exact concatenation. No source graph writes.</summary>
    public static class EndpointExtension
    {
        public const double TangentDot = .99999;

        public static bool TryGetRoad(RoadGraph graph, string id, out RoadSegment road)
        {
            road = null;
            var p = graph?.GetProfile(id);
            if (p == null || p.Lanes == null || p.Lanes.Length == 0
                || string.IsNullOrEmpty(p.NodeAId) == string.IsNullOrEmpty(p.NodeBId)) return false;
            road = graph.GetNode(string.IsNullOrEmpty(p.NodeAId) ? p.NodeBId : p.NodeAId) as RoadSegment;
            if (road == null || road.PortIds.Count != 2 || road.PortIds[0] == road.PortIds[1] || !road.PortIds.Contains(id)) return false;
            return graph.Nodes.Values.Count(n => n.PortIds.Contains(id)) == 1;
        }

        public static LaneDef[] ReverseLanes(LaneDef[] lanes) => lanes.Reverse().Select(l =>
            new LaneDef(l.Type, l.Width, l.Type == LaneType.Car ? !l.Forward : l.Forward, l.CurbHeight)).ToArray();

        public static bool Compatible(Profile a, Profile b)
        {
            if (!a.HasSameLayout(b)) return false;
            for (int i = 0; i < a.Lanes.Length; i++)
                if (Math.Abs(a.Lanes[i].CurbHeight - b.Lanes[i].CurbHeight) > 1e-4
                    || (a.Lanes[i].Type == LaneType.Car && a.Lanes[i].Forward != b.Lanes[i].Forward)) return false;
            return true;
        }

        /// <summary>Two-click extension: free endpoint tangents follow the solved circle, not the mouse chord.</summary>
        public static PathChain ConnectAutomatic(Profile start, Profile end, bool fixedStart, bool fixedEnd)
        {
            if (fixedStart && fixedEnd) return ProfileConnector.ConnectExact(start, end);
            if (fixedStart) return ArcToPoint(start.Position, start.Direction, end.Position);
            if (fixedEnd) return PathChainUtil.Reverse(ArcToPoint(end.Position, -end.Direction, start.Position));
            return ProfileConnector.Connect(start, end);
        }

        static PathChain ArcToPoint(DVec2 start, DVec2 tangent, DVec2 end)
        {
            tangent = tangent.Normalized;
            var delta = end - start;
            if (delta.Length < 1e-6) return null;
            double lateral = DVec2.Dot(delta, tangent.PerpLeft);
            var path = new PathChain();
            if (Math.Abs(lateral) < 1e-6)
            {
                if (DVec2.Dot(delta, tangent) <= 0) return null;
                path.Add(new LineElement(start, end)); return path;
            }
            double signedRadius = delta.LengthSquared / (2 * lateral);
            var center = start + tangent.PerpLeft * signedRadius;
            double a = Math.Atan2(start.Y - center.Y, start.X - center.X);
            double b = Math.Atan2(end.Y - center.Y, end.X - center.X);
            double sweep = ArcIntersections.NormalizeAngle(b - a);
            if (signedRadius > 0 && sweep < 0) sweep += 2 * Math.PI;
            if (signedRadius < 0 && sweep > 0) sweep -= 2 * Math.PI;
            path.Add(new ArcElement(center, Math.Abs(signedRadius), a, sweep));
            return path;
        }

        // The view faces out of an old road for a new start, into it for a new end.
        public static Profile View(RoadGraph graph, string id, bool start)
        {
            if (!TryGetRoad(graph, id, out var road)) return null;
            var p = graph.GetProfile(id);
            var path = SegmentPathResolver.Restore(road.ExplicitPath) ?? road.Path
                ?? ProfileConnector.Connect(graph.GetProfile(road.PortIds[0]), graph.GetProfile(road.PortIds[1]));
            if (path == null) return null;
            bool atStart = road.PortIds[0] == id;
            bool reverse = atStart == start;
            var tangent = atStart ? path.StartTangent : path.EndTangent;
            var view = Profile.Create(p.Position, reverse ? -tangent : tangent,
                reverse ? ReverseLanes(p.Lanes) : (LaneDef[])p.Lanes.Clone());
            view.Id = id; view.Y = p.Y;
            return view;
        }

        internal static bool Normalize(RoadGraph graph, string id, bool start, string forbiddenRoad)
        {
            if (id == null) return true;
            if (!TryGetRoad(graph, id, out var road)) return false;
            if ((road.PortIds[0] == id) == start)
                return RoadDeletePlanner.ReverseRoadChain(graph, road, forbiddenRoad, null);
            return true;
        }

        public static bool SameChain(RoadGraph graph, string a, string b)
        {
            if (!TryGetRoad(graph, a, out var first) || !TryGetRoad(graph, b, out var last)) return false;
            var seen = new HashSet<string>(); var queue = new Queue<RoadSegment>(); queue.Enqueue(first);
            while (queue.Count > 0)
            {
                var road = queue.Dequeue();
                if (road == last) return true;
                if (!seen.Add(road.Id)) continue;
                foreach (var pid in road.PortIds)
                {
                    var p = graph.GetProfile(pid);
                    foreach (var nid in new[] { p.NodeAId, p.NodeBId })
                        if (graph.GetNode(nid) is RoadSegment next && !seen.Contains(next.Id)) queue.Enqueue(next);
                }
            }
            return false;
        }

        internal static bool MergeAt(RoadGraph graph, string id, ref string finalEnd, string roadId)
        {
            if (id == null) return true;
            var p = graph.GetProfile(id);
            if (p == null) return false;
            var a = graph.GetNode(p.NodeAId) as RoadSegment;
            var b = graph.GetNode(p.NodeBId) as RoadSegment;
            if (a == null || b == null || a == b) return false;
            if (a.PortIds[0] == id) { var swap = a; a = b; b = swap; }
            if (a.PortIds[1] != id || b.PortIds[0] != id) return false;
            var left = SegmentPathResolver.Resolve(graph, a); var right = SegmentPathResolver.Resolve(graph, b);
            if (left == null || right == null || DVec2.Dot(left.EndTangent, right.StartTangent) < TangentDot) return false;
            var start = graph.GetProfile(a.PortIds[0]); var end = graph.GetProfile(b.PortIds[1]);
            if (start == end || !Compatible(start, end)) return false;
            var path = new PathChain(); path.Append(left); path.Append(right);
            graph.RemoveSegment(a.Id); graph.RemoveSegment(b.Id); graph.RemoveLooseProfile(p);
            var merged = graph.AddSegment(start, end, roadId);
            if (merged == null) return false;
            merged.Path = path; merged.ExplicitPath = SegmentPathResolver.Store(path);
            if (finalEnd == id) finalEnd = end.Id;
            return true;
        }

        internal static bool Continuous(PathChain path)
        {
            for (int i = 1; i < path.Elements.Count; i++)
                if (DVec2.Distance(path.Elements[i - 1].EndPoint, path.Elements[i].StartPoint) > 1e-4
                    || DVec2.Dot(path.Elements[i - 1].EndTangent, path.Elements[i].StartTangent) < TangentDot) return false;
            return true;
        }

        /// <summary>Second-click aid: exact intersection/projection of tangent rays, without locking B.</summary>
        public static bool TryControl(DVec2 a, DVec2? startDirection, Profile end, DVec2 cursor,
            double minimumLeg, double minimumRadius, out DVec2 control)
        {
            var back = -end.Direction;
            if (startDirection.HasValue)
            {
                var d = startDirection.Value;
                double cross = DVec2.Cross(d, back);
                if (Math.Abs(cross) < 1e-8)
                {
                    control = a + d * DVec2.Dot(cursor - a, d);
                    if (DVec2.Dot(d, end.Direction) < TangentDot
                        || Math.Abs(DVec2.Cross(end.Position - a, d)) > 1e-4) return false;
                }
                else control = a + d * (DVec2.Cross(end.Position - a, back) / cross);
            }
            else control = end.Position + back * DVec2.Dot(cursor - end.Position, back);
            if (DVec2.Dot(control - end.Position, back) < minimumLeg
                || (startDirection.HasValue && DVec2.Dot(control - a, startDirection.Value) < minimumLeg)) return false;
            var path = ControlPointCurve.Solve(a, control, end.Position, minimumLeg, minimumRadius, out _);
            return path != null && DVec2.Dot(path.EndTangent, end.Direction) >= TangentDot
                && (!startDirection.HasValue || DVec2.Dot(path.StartTangent, startDirection.Value) >= TangentDot);
        }
    }
}
