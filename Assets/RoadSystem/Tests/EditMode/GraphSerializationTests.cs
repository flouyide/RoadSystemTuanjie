using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;
using System;
using System.Linq;

namespace RoadSystem.Tests
{
    public class GraphSerializationTests
    {
        [Test]
        public void AddSegment_LinksPorts()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(0, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            var seg = g.AddSegment(pa, pb);
            Assert.NotNull(seg);
            Assert.AreEqual(2, seg.PortIds.Count);
            Assert.True(pa.IsLooseEnd, "新段起点 profile 只挂一个节点，应为悬空端");
            Assert.True(pb.IsLooseEnd, "新段终点 profile 只挂一个节点，应为悬空端");
        }

        [Test]
        public void AddSegment_RejectsMismatchedLayout()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(0, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0),
                new[] { new LaneDef(LaneType.Car, 3.5f) });
            Assert.IsNull(g.AddSegment(pa, pb), "断面不一致应拒绝");
        }

        [Test]
        public void AddSegment_RejectsOccupiedIntersectionPort()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(0, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            var pc = g.CreateProfile(new DVec2(60, 0), new DVec2(1, 0));
            var pd = g.CreateProfile(new DVec2(90, 0), new DVec2(1, 0));
            Assert.NotNull(g.AddSegment(pa, pb));
            Assert.NotNull(g.AddSegment(pb, pc));
            Assert.IsNull(g.AddSegment(pb, pd), "端口已连接两个实体时不可再挂第三个");
        }

        [Test]
        public void JsonRoundTrip_PreservesGraph()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(0, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            var pc = g.CreateProfile(new DVec2(40, 10), new DVec2(0.6, 0.8));
            g.AddSegment(pa, pb);
            g.AddSegment(pb, pc);

            string json = RoadGraphJson.ToJson(g);
            var g2 = RoadGraphJson.FromJson(json);

            Assert.AreEqual(3, g2.Profiles.Count);
            Assert.AreEqual(2, g2.Nodes.Count);
            var pa2 = g2.GetProfile(pa.Id);
            Assert.NotNull(pa2);
            Assert.True(pa2.Position.ApproxEquals(pa.Position));
            Assert.AreEqual(4, pa2.Lanes.Length);
            Assert.AreEqual(LaneType.Sidewalk, pa2.Lanes[0].Type);
            // 共享 profile pb 两端各挂一段
            var pb2 = g2.GetProfile(pb.Id);
            Assert.False(pb2.IsLooseEnd);
        }

        [Test]
        public void ChangedEvent_FiresWithAffectedNodes()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(0, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            var seg = g.AddSegment(pa, pb);

            int fired = 0;
            g.Changed += ids => fired++;
            g.MoveProfile(pa, new DVec2(0, 1), new DVec2(1, 0));
            Assert.AreEqual(1, fired);
            Assert.Greater(seg.DirtyVersion, 0);
        }

        [Test]
        public void ControlPointCurve_UsesTangentCircularArc_AndRejectsTightTurns()
        {
            var a = new DVec2(0, 0);
            var c = new DVec2(40, 0);
            var b = new DVec2(40, 40);
            var path = ControlPointCurve.Solve(a, c, b, 1, 6, out var reason);
            Assert.IsNull(reason);
            Assert.AreEqual(1, path.Elements.Count);
            var arc = (ArcElement)path.Elements[0];
            Assert.That(arc.Radius, Is.EqualTo(40).Within(1e-6));
            Assert.That(arc.Sweep, Is.EqualTo(Math.PI / 2).Within(1e-6));
            Assert.True(path.StartPoint.ApproxEquals(a));
            Assert.True(path.EndPoint.ApproxEquals(b));
            Assert.True(path.StartTangent.ApproxEquals(DVec2.UnitX));
            Assert.True(path.EndTangent.ApproxEquals(DVec2.UnitY));
            Assert.IsNull(ControlPointCurve.Solve(a, new DVec2(2, 0),
                new DVec2(2, 2), 1, 6, out _));
        }

        [Test]
        public void ControlledCurve_IntersectionFragmentsSurviveJsonRoundTrip()
        {
            var graph = new RoadGraph();
            var start = graph.CreateProfile(new DVec2(-100, 0), DVec2.UnitX);
            var end = graph.CreateProfile(new DVec2(0, 100), DVec2.UnitY);
            var path = ControlPointCurve.Solve(start.Position, new DVec2(0, 0),
                end.Position, 1, 6, out _);
            var curve = graph.AddSegment(start, end);
            curve.Path = path;
            curve.ExplicitPath = SegmentPathResolver.Store(path);
            var crossStart = graph.CreateProfile(new DVec2(-130, 30), DVec2.UnitX);
            var crossEnd = graph.CreateProfile(new DVec2(30, 30), DVec2.UnitX);
            Assert.True(IntersectionBuilder.CanBuildRoad(graph, crossStart, crossEnd,
                null, out var reason), reason);
            var crossing = graph.AddSegment(crossStart, crossEnd);
            Assert.True(IntersectionBuilder.DetectAndBuild(graph, crossing.Id));
            var arcFragments = graph.Segments().Where(s =>
                SegmentPathResolver.Resolve(graph, s).Elements.OfType<ArcElement>().Any()).ToList();
            Assert.Greater(arcFragments.Count, 0);

            var restored = RoadGraphJson.FromJson(RoadGraphJson.ToJson(graph));
            foreach (var fragment in arcFragments)
            {
                var restoredSegment = (RoadSegment)restored.GetNode(fragment.Id);
                Assert.NotNull(restoredSegment.ExplicitPath);
                var before = SegmentPathResolver.Resolve(graph, fragment);
                var after = SegmentPathResolver.Resolve(restored, restoredSegment);
                Assert.True(after.StartPoint.ApproxEquals(before.StartPoint));
                Assert.True(after.EndPoint.ApproxEquals(before.EndPoint));
                Assert.That(after.TotalLength, Is.EqualTo(before.TotalLength).Within(1e-5));
            }
        }
    }
}
