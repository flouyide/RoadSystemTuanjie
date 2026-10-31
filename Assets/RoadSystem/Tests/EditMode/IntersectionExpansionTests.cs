using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;

namespace RoadSystem.Tests
{
    public class IntersectionExpansionTests
    {
        static RoadSegment Road(RoadGraph g, DVec2 a, DVec2 b, PathChain path = null)
        {
            var direction = (b - a).Normalized;
            var s = g.AddSegment(g.CreateProfile(a, path?.StartTangent ?? direction),
                g.CreateProfile(b, path?.EndTangent ?? direction), Guid.NewGuid().ToString("N"));
            if (path != null) { s.Path = path; s.ExplicitPath = SegmentPathResolver.Store(path); }
            return s;
        }

        static Intersection Tee(RoadGraph g, DVec2 center, bool upperLeg = false, bool curved = false)
        {
            Road(g, center + new DVec2(-150, 0), center + new DVec2(150, 0));
            var start = center + (upperLeg ? new DVec2(0, 150) : new DVec2(0, -150));
            PathChain path = null;
            if (curved)
            {
                start = center + new DVec2(-120, -120);
                path = ControlPointCurve.Solve(start, center + new DVec2(0, -120), center, 1, 6, out _);
            }
            var incoming = Road(g, start, center, path);
            Assert.True(IntersectionBuilder.DetectAndBuild(g, incoming.Id));
            return g.Nodes.Values.OfType<Intersection>().Single(ix => DVec2.Distance(ix.Anchor, center) < .01);
        }

        static RoadBuildPlan Connect(RoadGraph graph, DVec2 a, DVec2 b, string startIx, string endIx,
            RoadBuildSettings settings, PathChain path = null, LaneDef[] lanes = null)
        {
            var direction = (b - a).Normalized;
            var start = Profile.Create(a, path?.StartTangent ?? direction, lanes ?? Profile.DefaultLanes());
            var end = Profile.Create(b, path?.EndTangent ?? direction, lanes ?? Profile.DefaultLanes());
            Assert.True(RoadBuildPlanner.TryPlan(graph, start, end, path, null, startIx, endIx,
                settings, out var plan, out var reason), reason);
            return plan;
        }

        static string LinkKey(LaneConnection l) => $"{l.FromPortId}:{l.FromLane}>{l.ToPortId}:{l.ToLane}";

        [TestCase(0.0, false)] [TestCase(.5, false)] [TestCase(1.0, false)]
        [TestCase(0.0, true)] [TestCase(.5, true)] [TestCase(1.0, true)]
        public void TeeToCrossToFive_PreservesIdentityAndLinks(double curve, bool reverse)
        {
            var graph = new RoadGraph();
            var ix = Tee(graph, DVec2.Zero);
            var id = ix.Id;
            var ids = ix.PortIds.ToArray();
            var roads = graph.Segments().ToDictionary(s => s.Id, s => s.RoadId);
            var links = ix.LaneLinks.Select(LinkKey).ToArray();
            var settings = RoadBuildSettings.Default; settings.Curve = curve;
            foreach (var point in new[] { new DVec2(0, 150), new DVec2(150, 150) })
            {
                long revision = graph.Revision;
                var originalPositions = graph.Profiles.ToDictionary(p => p.Key, p => p.Value.Position);
                var plan = Connect(graph, reverse ? point : DVec2.Zero, reverse ? DVec2.Zero : point,
                    reverse ? null : id, reverse ? id : null, settings);
                Assert.AreEqual(revision, graph.Revision);
                foreach (var p in originalPositions) Assert.AreEqual(p.Value, graph.GetProfile(p.Key).Position);
                Assert.True(plan.IsCurrent(graph, settings));
                graph = plan.Result;
                ix = (Intersection)graph.GetNode(id);
                Assert.AreEqual(DVec2.Zero, ix.Anchor);
                foreach (var port in ids) Assert.Contains(port, ix.PortIds);
                foreach (var road in roads) Assert.AreEqual(road.Value, ((RoadSegment)graph.GetNode(road.Key)).RoadId);
                foreach (var link in links) Assert.Contains(link, ix.LaneLinks.Select(LinkKey).ToArray());
                Assert.True(ix.LaneLinks.All(l => l.Path != null));
                Assert.AreEqual(ix.LaneLinks.Count, ix.LaneLinks.Select(LinkKey).Distinct().Count());
            }
            Assert.AreEqual(5, ix.PortIds.Count);
            Assert.AreEqual(5, graph.Segments().Count());
        }

        [TestCase(false)] [TestCase(true)]
        public void CircularRoad_CanStartOrEndAtIntersection(bool reverse)
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero);
            var far = new DVec2(100, 100); var control = new DVec2(0, 100);
            var a = reverse ? far : DVec2.Zero; var b = reverse ? DVec2.Zero : far;
            var path = ControlPointCurve.Solve(a, control, b, 1, 6, out _);
            var plan = Connect(graph, a, b, reverse ? null : ix.Id, reverse ? ix.Id : null,
                RoadBuildSettings.Default, path);
            Assert.AreEqual(4, ((Intersection)plan.Result.GetNode(ix.Id)).PortIds.Count);
            Assert.True(plan.Result.Segments().Any(s => s.ExplicitPath.Any(e => e.Kind == "arc")));
        }

        [Test]
        public void ExistingCircularArm_RetainsCircleAndSurvivesDtoRoundTrip()
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero, curved: true);
            var old = graph.Segments().Single(s => s.ExplicitPath.Any(e => e.Kind == "arc"));
            var circle = old.ExplicitPath.First(e => e.Kind == "arc");
            var plan = Connect(graph, DVec2.Zero, new DVec2(0, 150), ix.Id, null, RoadBuildSettings.Default);
            var copy = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(plan.Result));
            var restored = ((RoadSegment)copy.GetNode(old.Id)).ExplicitPath.First(e => e.Kind == "arc");
            Assert.AreEqual(circle.Center, restored.Center);
            Assert.AreEqual(circle.Radius, restored.Radius);
            Assert.True(((Intersection)copy.GetNode(ix.Id)).HasAnchor);
            Assert.AreEqual(ix.Anchor, ((Intersection)copy.GetNode(ix.Id)).Anchor);
        }

        [Test]
        public void BothEndpoints_ExpandDifferentJunctionsInOnePlan()
        {
            var graph = new RoadGraph(); var a = Tee(graph, DVec2.Zero);
            var b = Tee(graph, new DVec2(0, 300), true);
            var plan = Connect(graph, a.Anchor, b.Anchor, a.Id, b.Id, RoadBuildSettings.Default);
            Assert.AreEqual(4, ((Intersection)plan.Result.GetNode(a.Id)).PortIds.Count);
            Assert.AreEqual(4, ((Intersection)plan.Result.GetNode(b.Id)).PortIds.Count);
            Assert.AreEqual(3, a.PortIds.Count);
            Assert.AreEqual(3, b.PortIds.Count);
        }

        [Test]
        public void SecondEndpointFailure_LeavesFirstJunctionAndOriginalGraphUntouched()
        {
            var graph = new RoadGraph(); var a = Tee(graph, DVec2.Zero);
            var b = Tee(graph, new DVec2(0, 400)); // its south arm occupies the proposed approach
            var before = graph.Profiles.ToDictionary(p => p.Key, p => p.Value.Position);
            long version = graph.Revision;
            var start = Profile.Create(a.Anchor, DVec2.UnitY, null);
            var end = Profile.Create(b.Anchor, DVec2.UnitY, null);
            Assert.False(RoadBuildPlanner.TryPlan(graph, start, end, null, null, a.Id, b.Id,
                RoadBuildSettings.Default, out var plan, out var reason));
            Assert.IsNull(plan); Assert.IsNotEmpty(reason); Assert.AreEqual(version, graph.Revision);
            Assert.AreEqual(3, a.PortIds.Count); Assert.AreEqual(3, b.PortIds.Count);
            foreach (var p in before) Assert.AreEqual(p.Value, graph.GetProfile(p.Key).Position);
        }

        [Test]
        public void Snap_UsesPavementHeightAndStableLegacyAnchor()
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero); ix.HasAnchor = false;
            Assert.True(RoadBuildPlanner.TrySnapIntersection(graph, DVec2.Zero, 0, RoadBuildSettings.Default,
                out var id, out var anchor, out _));
            Assert.AreEqual(ix.Id, id); Assert.False(ix.HasAnchor);
            Assert.False(RoadBuildPlanner.TrySnapIntersection(graph, DVec2.Zero, 10, RoadBuildSettings.Default,
                out _, out _, out _));
            var plan = Connect(graph, anchor, new DVec2(0, 150), ix.Id, null, RoadBuildSettings.Default);
            Assert.AreEqual(anchor, ((Intersection)plan.Result.GetNode(ix.Id)).Anchor);
        }

        [Test]
        public void WideBranch_UsesItsOwnIndependentProfileLayout()
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero);
            var lanes = Profile.DefaultLanes(); lanes[1].Width = lanes[2].Width = 5;
            var plan = Connect(graph, DVec2.Zero, new DVec2(0, 150), ix.Id, null, RoadBuildSettings.Default, lanes: lanes);
            var expanded = (Intersection)plan.Result.GetNode(ix.Id);
            Assert.AreEqual(14, plan.Result.GetProfile(expanded.PortIds.Last()).TotalWidth);
            foreach (var id in ix.PortIds) Assert.AreEqual(11, plan.Result.GetProfile(id).TotalWidth);
        }

        [Test]
        public void Clone_SeparatesLayoutsLinksAndExplicitPathElements()
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero);
            var copy = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(graph));
            var port = copy.GetProfile(ix.PortIds[0]); port.Lanes[1].Width = 99;
            ((Intersection)copy.GetNode(ix.Id)).LaneLinks[0].FromLane = 99;
            var road = copy.Segments().First(); road.ExplicitPath[0].A = new DVec2(999, 999);
            Assert.AreEqual(3.5, graph.GetProfile(port.Id).Lanes[1].Width);
            Assert.AreNotEqual(99, ix.LaneLinks[0].FromLane);
            Assert.AreNotEqual(road.ExplicitPath[0].A, ((RoadSegment)graph.GetNode(road.Id)).ExplicitPath[0].A);
        }

        [Test]
        public void MiddleCrossingAndSameTargetAndShortBranch_AreRejected()
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero);
            var a = Profile.Create(new DVec2(0, -170), DVec2.UnitY, null);
            var b = Profile.Create(new DVec2(0, 170), DVec2.UnitY, null);
            Assert.False(RoadBuildPlanner.TryPlan(graph, a, b, null, null, null, null, RoadBuildSettings.Default, out _, out _));
            a.Position = DVec2.Zero; b.Position = new DVec2(0, 5);
            Assert.False(RoadBuildPlanner.TryPlan(graph, a, b, null, null, ix.Id, null, RoadBuildSettings.Default, out _, out _));
            Assert.False(RoadBuildPlanner.TryPlan(graph, a, b, null, null, ix.Id, ix.Id, RoadBuildSettings.Default, out _, out _));
        }

        [Test]
        public void PlanVersion_ChangesInvalidatePreview()
        {
            var graph = new RoadGraph(); var ix = Tee(graph, DVec2.Zero);
            var plan = Connect(graph, DVec2.Zero, new DVec2(0, 150), ix.Id, null, RoadBuildSettings.Default);
            var changed = RoadBuildSettings.Default; changed.Curve = .5;
            Assert.False(plan.IsCurrent(graph, changed));
            Assert.False(plan.IsCurrent(RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(graph)), RoadBuildSettings.Default));
            graph.NotifyChanged(new[] { ix.Id });
            Assert.False(plan.IsCurrent(graph, RoadBuildSettings.Default));
        }

        [Test]
        public void PolygonCoverage_ChecksConcaveEdgesNotOnlyVertices()
        {
            var u = new List<DVec2> { new DVec2(0,0), new DVec2(6,0), new DVec2(6,6),
                new DVec2(4,6), new DVec2(4,2), new DVec2(2,2), new DVec2(2,6), new DVec2(0,6) };
            var bridge = new List<DVec2> {new DVec2(1,4),new DVec2(5,4),new DVec2(5,5),new DVec2(1,5)};
            Assert.True(bridge.All(p => RoadFootprint.Contains(u, p)));
            Assert.False(RoadFootprint.Covers(u, bridge, .01));
        }
    }
}
