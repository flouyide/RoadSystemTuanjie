using System;
using System.Linq;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;

namespace RoadSystem.Tests
{
    public class EndpointExtensionTests
    {
        static RoadSegment Road(RoadGraph graph, DVec2 a, DVec2 b, string group = null)
        {
            var d = (b - a).Normalized;
            var road = graph.AddSegment(graph.CreateProfile(a, d), graph.CreateProfile(b, d), group);
            var path = new PathChain(); path.Add(new LineElement(a, b));
            road.Path = path; road.ExplicitPath = SegmentPathResolver.Store(path); return road;
        }

        static string State(RoadGraph graph) => string.Join("|", graph.Profiles.Values.Select(p =>
            $"{p.Id}:{p.Position}:{p.Direction}:{p.NodeAId}:{p.NodeBId}:" + string.Join(",", p.Lanes.Select(l => $"{l.Type}/{l.Width}/{l.Forward}"))))
            + string.Join("|", graph.Nodes.Values.Select(n => n.Id + string.Join(",", n.PortIds)));

        static RoadBuildPlan Plan(RoadGraph graph, Profile a, Profile b, PathChain path = null)
        {
            var before = State(graph); long version = graph.Revision; int changes = 0;
            graph.Changed += _ => changes++;
            Assert.True(RoadBuildPlanner.TryPlan(graph, a, b, path, "extension", null, null,
                RoadBuildSettings.Default, out var plan, out var reason), reason);
            Assert.AreEqual(before, State(graph)); Assert.AreEqual(version, graph.Revision); Assert.AreEqual(0, changes);
            return plan;
        }

        [TestCase(false, false)] [TestCase(false, true)] [TestCase(true, false)] [TestCase(true, true)]
        public void SingleEnd_ExtendsEitherEndInEitherDrawingDirection(bool reversed, bool newStart)
        {
            var graph = new RoadGraph();
            var old = Road(graph, new DVec2(reversed ? 0 : -40, 0), new DVec2(reversed ? -40 : 0, 0), "original");
            var oldEnd = graph.Profiles.Values.Single(p => p.Position.X == 0);
            var free = Profile.Create(new DVec2(40, 0), newStart ? DVec2.UnitX : -DVec2.UnitX, null);
            var path = new PathChain(); path.Add(new LineElement(newStart ? oldEnd.Position : free.Position, newStart ? free.Position : oldEnd.Position));
            var plan = Plan(graph, newStart ? oldEnd : free, newStart ? free : oldEnd, path);
            var merged = plan.Result.Segments().Single();
            Assert.AreEqual("original", merged.RoadId); Assert.IsNull(plan.Result.GetNode(old.Id));
            Assert.IsNull(plan.Result.GetProfile(oldEnd.Id)); Assert.AreEqual(2, plan.Result.Profiles.Count);
            Assert.AreEqual(80, SegmentPathResolver.Resolve(plan.Result, merged).TotalLength, 1e-8);
            Assert.True(EndpointExtension.TryGetRoad(plan.Result, plan.EndProfileId, out _));
            Assert.True(RoadDeletePlanner.TryPlan(plan.Result, merged.Id, RoadBuildSettings.Default, out var deletion, out _));
            Assert.IsEmpty(deletion.Edit.Result.Nodes);
        }

        [TestCase(false, false, false)] [TestCase(false, true, false)]
        [TestCase(true, false, false)] [TestCase(true, true, false)]
        [TestCase(false, false, true)] [TestCase(false, true, true)]
        [TestCase(true, false, true)] [TestCase(true, true, true)]
        public void TwoEnds_AllOrientationsMergeAndRoundTrip(bool reverseA, bool reverseB, bool sameGroup)
        {
            var graph = new RoadGraph();
            var a = Road(graph, new DVec2(reverseA ? -50 : -100, 0), new DVec2(reverseA ? -100 : -50, 0), "groupA");
            var b = Road(graph, new DVec2(reverseB ? 100 : 50, 0), new DVec2(reverseB ? 50 : 100, 0), sameGroup ? "groupA" : "groupB");
            var start = graph.Profiles.Values.Single(p => p.Position.X == -50);
            var end = graph.Profiles.Values.Single(p => p.Position.X == 50);
            var plan = Plan(graph, start, end);
            var merged = plan.Result.Segments().Single();
            Assert.IsNull(plan.Result.GetNode(a.Id)); Assert.IsNull(plan.Result.GetNode(b.Id));
            Assert.AreEqual(2, plan.Result.Profiles.Count);
            var restored = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(plan.Result));
            var path = SegmentPathResolver.Resolve(restored, (RoadSegment)restored.GetNode(merged.Id));
            Assert.AreEqual(200, path.TotalLength, 1e-8);
            Assert.Less(DVec2.Distance(new DVec2(-100, 0), path.StartPoint), 1e-8);
            Assert.Less(DVec2.Distance(new DVec2(100, 0), path.EndPoint), 1e-8);
        }

        [TestCase(false)] [TestCase(true)]
        public void CurvedOldRoad_KeepsExactCircle(bool atStart)
        {
            var graph = new RoadGraph(); var arc = new ArcElement(DVec2.Zero, 40, 0, Math.PI / 2);
            var old = graph.AddSegment(graph.CreateProfile(arc.StartPoint, arc.StartTangent), graph.CreateProfile(arc.EndPoint, arc.EndTangent));
            old.Path = new PathChain(); old.Path.Add(arc); old.ExplicitPath = SegmentPathResolver.Store(old.Path);
            var selected = graph.GetProfile(old.PortIds[atStart ? 0 : 1]);
            var view = EndpointExtension.View(graph, selected.Id, true);
            var free = Profile.Create(view.Position + view.Direction * 40, view.Direction, view.Lanes);
            var merged = Plan(graph, selected, free).Result.Segments().Single();
            var resultArc = SegmentPathResolver.Restore(merged.ExplicitPath).Elements.OfType<ArcElement>().Single();
            Assert.AreEqual(arc.Center, resultArc.Center); Assert.AreEqual(arc.Radius, resultArc.Radius);
            Assert.AreEqual(Math.Abs(arc.Sweep), Math.Abs(resultArc.Sweep), 1e-8);
        }

        [Test]
        public void IntermediateCrossing_IsKeptAndOnlyEndpointSeamsMerge()
        {
            var graph = new RoadGraph(); var a = Road(graph, new DVec2(-160, 0), new DVec2(-100, 0), "same");
            var b = Road(graph, new DVec2(100, 0), new DVec2(160, 0), "same");
            Road(graph, new DVec2(0, -100), new DVec2(0, 100), "same");
            var plan = Plan(graph, graph.GetProfile(a.PortIds[1]), graph.GetProfile(b.PortIds[0]));
            Assert.AreEqual(4, plan.Result.Segments().Count());
            Assert.AreEqual(4, plan.Result.Nodes.Values.OfType<Intersection>().Single().PortIds.Count);
            Assert.True(EndpointExtension.TryGetRoad(plan.Result, plan.EndProfileId, out _));
        }

        [Test]
        public void InvalidEndpointOrClosedLoop_DoesNotMutateSource()
        {
            var graph = new RoadGraph(); var road = Road(graph, DVec2.Zero, new DVec2(50, 0));
            var orphan = graph.CreateProfile(new DVec2(-30, 0), DVec2.UnitX);
            Assert.False(EndpointExtension.TryGetRoad(graph, orphan.Id, out _));
            var before = State(graph);
            Assert.False(RoadBuildPlanner.TryPlan(graph, graph.GetProfile(road.PortIds[0]), graph.GetProfile(road.PortIds[1]),
                null, null, null, null, RoadBuildSettings.Default, out _, out _));
            Assert.False(RoadBuildPlanner.TryPlan(graph, orphan, Profile.Create(new DVec2(-60, 0), -DVec2.UnitX, null),
                null, null, null, null, RoadBuildSettings.Default, out _, out _));
            Assert.AreEqual(before, State(graph));
        }

        [Test]
        public void LooseAngleTolerance_IsRejectedAndSourceUnchanged()
        {
            var graph = new RoadGraph(); var old = Road(graph, new DVec2(-50, 0), DVec2.Zero);
            var end = Profile.Create(new DVec2(50, 5), DVec2.UnitX, null);
            var path = new PathChain(); path.Add(new LineElement(DVec2.Zero, end.Position));
            var before = State(graph);
            Assert.False(RoadBuildPlanner.TryPlan(graph, graph.GetProfile(old.PortIds[1]), end, path,
                null, null, null, RoadBuildSettings.Default, out _, out var reason));
            Assert.That(reason, Does.Contain("切线")); Assert.AreEqual(before, State(graph));
        }

        [TestCase(false)] [TestCase(true)]
        public void ControlAssist_ExactNonGridTangentIntersection(bool fixedStart)
        {
            var a = new DVec2(1.23, 4.56); var d = new DVec2(2, 1).Normalized;
            var c = a + d * 40.37; var outDirection = d.PerpLeft;
            var b = Profile.Create(c + outDirection * 45.91, outDirection, null);
            Assert.True(EndpointExtension.TryControl(a, fixedStart ? (DVec2?)d : null, b, c + d * .1,
                1, 6, out var control));
            var path = ControlPointCurve.Solve(a, control, b.Position, 1, 6, out _);
            Assert.Greater(DVec2.Dot(path.EndTangent, b.Direction), EndpointExtension.TangentDot);
            if (fixedStart) Assert.Less(DVec2.Distance(c, control), 1e-8);
        }

        [Test]
        public void SingleEndedExtension_InheritsAsymmetricSection()
        {
            var graph = new RoadGraph(); var old = Road(graph, DVec2.Zero, new DVec2(50, 0));
            foreach (var id in old.PortIds)
            {
                var p = graph.GetProfile(id);
                p.Lanes[0].Width = 1; p.Lanes[3].Width = 3; p.Lanes[2].Forward = true;
            }
            var plan = Plan(graph, graph.GetProfile(old.PortIds[0]), Profile.Create(new DVec2(-50, 0), -DVec2.UnitX, null));
            var merged = plan.Result.Segments().Single();
            var section = plan.Result.GetProfile(merged.PortIds[0]);
            Assert.AreEqual(3, section.Lanes[0].Width); Assert.AreEqual(1, section.Lanes[3].Width);
            Assert.False(section.Lanes[1].Forward); Assert.False(section.Lanes[2].Forward);
        }

        static DVec2 LaneCenter(Profile p, int index)
        {
            double offset = p.Lanes.Take(index).Sum(l => (double)l.Width) + p.Lanes[index].Width * .5;
            return p.LateralPoint(offset);
        }

        [Test]
        public void ReverseExtension_PreservesRemoteIntersectionLanePositions()
        {
            var graph = new RoadGraph(); Road(graph, new DVec2(-100, 0), new DVec2(100, 0));
            var vertical = Road(graph, new DVec2(0, -100), new DVec2(0, 100));
            Assert.True(IntersectionBuilder.DetectAndBuild(graph, vertical.Id));
            var ix = graph.Nodes.Values.OfType<Intersection>().Single();
            var tip = graph.Profiles.Values.Single(p => p.Position.X == -100);
            var plan = Plan(graph, tip, Profile.Create(new DVec2(-150, 0), -DVec2.UnitX, null));
            var result = (Intersection)plan.Result.GetNode(ix.Id);
            Assert.AreEqual(ix.LaneLinks.Count, result.LaneLinks.Count);
            foreach (var link in ix.LaneLinks)
            {
                var from = LaneCenter(graph.GetProfile(link.FromPortId), link.FromLane);
                var to = LaneCenter(graph.GetProfile(link.ToPortId), link.ToLane);
                Assert.True(result.LaneLinks.Any(l => l.FromPortId == link.FromPortId && l.ToPortId == link.ToPortId
                    && DVec2.Distance(from, LaneCenter(plan.Result.GetProfile(l.FromPortId), l.FromLane)) < 1e-6
                    && DVec2.Distance(to, LaneCenter(plan.Result.GetProfile(l.ToPortId), l.ToLane)) < 1e-6 && l.Path != null));
            }
        }

        [Test]
        public void IncompatibleTwoEndedFlow_RejectsWithoutReversingSource()
        {
            var graph = new RoadGraph(); var a = Road(graph, new DVec2(-50, 0), new DVec2(-100, 0));
            var b = Road(graph, new DVec2(50, 0), new DVec2(100, 0));
            foreach (var id in b.PortIds) graph.GetProfile(id).Lanes[2].Forward = true;
            var before = State(graph); long revision = graph.Revision;
            Assert.False(RoadBuildPlanner.TryPlan(graph, graph.GetProfile(a.PortIds[0]), graph.GetProfile(b.PortIds[0]),
                null, null, null, null, RoadBuildSettings.Default, out _, out _));
            Assert.AreEqual(before, State(graph)); Assert.AreEqual(revision, graph.Revision);
        }
    }
}
