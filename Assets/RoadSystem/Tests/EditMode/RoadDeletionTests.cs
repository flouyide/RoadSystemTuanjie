using System;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;

namespace RoadSystem.Tests
{
    public class RoadDeletionTests
    {
        static RoadSegment Road(RoadGraph g, DVec2 a, DVec2 b, string roadId = null, PathChain path = null)
        {
            var d = (b - a).Normalized;
            var s = g.AddSegment(g.CreateProfile(a, path?.StartTangent ?? d),
                g.CreateProfile(b, path?.EndTangent ?? d), roadId ?? Guid.NewGuid().ToString("N"));
            s.Path = path ?? ProfileConnector.Connect(g.GetProfile(s.PortIds[0]), g.GetProfile(s.PortIds[1]));
            s.ExplicitPath = SegmentPathResolver.Store(s.Path);
            return s;
        }

        static Intersection Junction(RoadGraph graph, DVec2 center, bool cross = true)
        {
            Road(graph, center + new DVec2(-120, 0), center + new DVec2(120, 0));
            var v = Road(graph, center + new DVec2(0, -120), center + new DVec2(0, cross ? 120 : 0));
            Assert.True(IntersectionBuilder.DetectAndBuild(graph, v.Id));
            return graph.Nodes.Values.OfType<Intersection>().Single(n => n.Anchor.ApproxEquals(center));
        }

        static RoadSegment Arm(RoadGraph graph, Intersection ix, Func<DVec2, bool> matches)
        {
            var p = ix.PortIds.Select(graph.GetProfile).Single(p1 => matches(p1.Position - ix.Anchor));
            return graph.GetNode(p.NodeAId == ix.Id ? p.NodeBId : p.NodeAId) as RoadSegment;
        }

        static string Fingerprint(RoadGraph g)
        {
            var result = string.Join("|", g.Profiles.Values.OrderBy(p => p.Id).Select(p =>
                $"{p.Id}:{p.Position.X:R},{p.Position.Y:R}:{p.Direction.X:R},{p.Direction.Y:R}:{p.NodeAId}:{p.NodeBId}:" +
                string.Join(",", p.Lanes.Select(l => $"{l.Type}:{l.Width}:{l.Forward}:{l.CurbHeight}"))));
            foreach (var n in g.Nodes.Values.OrderBy(n => n.Id))
            {
                result += $"|{n.Id}:{string.Join(",", n.PortIds)}";
                if (n is RoadSegment s) result += s.RoadId + PathKey(s.ExplicitPath);
                if (n is Intersection ix) result += $"{ix.Anchor.X:R}:{ix.Anchor.Y:R}:" +
                    string.Join(",", ix.LaneLinks.Select(l => $"{l.FromPortId}:{l.FromLane}>{l.ToPortId}:{l.ToLane}"));
            }
            return result;
        }

        static string PathKey(List<PathElementDto> path) => path == null ? "null" : string.Join("|", path.Select(e =>
            $"{e.Kind}:{e.A.X:R},{e.A.Y:R}:{e.B.X:R},{e.B.Y:R}:{e.Center.X:R},{e.Center.Y:R}:{e.Radius:R}:{e.StartAngle:R}:{e.Sweep:R}"));

        static RoadDeletePlan Delete(RoadGraph graph, string id, RoadBuildSettings settings)
        {
            var before = Fingerprint(graph); var revision = graph.Revision; int changed = 0;
            graph.Changed += _ => changed++;
            Assert.True(RoadDeletePlanner.TryPlan(graph, id, settings, out var plan, out var reason), reason);
            Assert.AreEqual(before, Fingerprint(graph)); Assert.AreEqual(revision, graph.Revision); Assert.AreEqual(0, changed);
            Assert.IsNull(plan.Edit.Result.GetNode(id));
            Assert.True(plan.Edit.ChangedNodes.Contains(id));
            return plan;
        }

        [TestCase(0.0, 0)] [TestCase(0.0, 1)] [TestCase(0.0, 2)] [TestCase(0.0, 3)]
        [TestCase(.5, 0)] [TestCase(.5, 1)] [TestCase(.5, 2)] [TestCase(.5, 3)]
        [TestCase(1.0, 0)] [TestCase(1.0, 1)] [TestCase(1.0, 2)] [TestCase(1.0, 3)]
        public void CrossToTee_CleansPortsAndLaneLinks(double curve, int arm)
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero);
            var p = graph.GetProfile(ix.PortIds[arm]);
            var segment = graph.GetNode(p.NodeAId == ix.Id ? p.NodeBId : p.NodeAId) as RoadSegment;
            var settings = RoadBuildSettings.Default; settings.Curve = curve;
            var plan = Delete(graph, segment.Id, settings);
            var result = (Intersection)plan.Edit.Result.GetNode(ix.Id);
            Assert.AreEqual(3, result.PortIds.Count); Assert.IsNull(plan.Edit.Result.GetProfile(p.Id));
            Assert.True(result.LaneLinks.All(l => l.FromPortId != p.Id && l.ToPortId != p.Id && l.Path != null));
            Assert.AreEqual(3, plan.Edit.Result.Segments().Count());
            var loop = plan.Edit.Boundaries[ix.Id];
            Assert.True(RoadFootprint.IsSimple(RoadFootprint.Points(loop)));
            Assert.True(PolygonTriangulator.TryTriangulate(loop, new List<int>()));
        }

        [TestCase(0.0)] [TestCase(.5)] [TestCase(1.0)]
        public void FiveToFourToTee_UsesSameIntersectionId(double curve)
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero);
            var settings = RoadBuildSettings.Default; settings.Curve = curve;
            var d = new DVec2(1, 1).Normalized;
            Assert.True(RoadBuildPlanner.TryPlan(graph, Profile.Create(DVec2.Zero, d, null),
                Profile.Create(d * 150, d, null), null, null, ix.Id, null, settings, out var addition, out var why), why);
            graph = addition.Result; ix = (Intersection)graph.GetNode(ix.Id);
            var diagonal = Arm(graph, ix, p => p.X > 1 && p.Y > 1);
            graph = Delete(graph, diagonal.Id, settings).Edit.Result;
            ix = (Intersection)graph.GetNode(ix.Id); Assert.AreEqual(4, ix.PortIds.Count);
            var north = Arm(graph, ix, p => p.Y > 1);
            graph = Delete(graph, north.Id, settings).Edit.Result;
            Assert.AreEqual(3, ((Intersection)graph.GetNode(ix.Id)).PortIds.Count);
        }

        [TestCase(0.0)] [TestCase(.5)] [TestCase(1.0)]
        public void TeeToSingleSegment_RemovesOldNodesAndKeepsOuterProfiles(double curve)
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero, false);
            var branch = Arm(graph, ix, p => p.Y < -1);
            var originals = graph.Segments().Where(s => s != branch).ToArray();
            var outerIds = originals.SelectMany(s => s.PortIds).Where(id => !ix.PortIds.Contains(id)).ToArray();
            var settings = RoadBuildSettings.Default; settings.Curve = curve;
            var plan = Delete(graph, branch.Id, settings);
            Assert.IsNull(plan.Edit.Result.GetNode(ix.Id));
            Assert.AreEqual(1, plan.Edit.Result.Segments().Count());
            foreach (var original in originals)
            {
                Assert.IsNull(plan.Edit.Result.GetNode(original.Id));
                Assert.True(plan.Edit.ChangedNodes.Contains(original.Id));
            }
            var bridge = plan.Edit.Result.Segments().Single();
            Assert.IsNull(bridge.RoadId);
            CollectionAssert.AreEquivalent(outerIds, bridge.PortIds);
            Assert.AreEqual(2, plan.Edit.Result.Profiles.Count);
            var roundTrip = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(plan.Edit.Result));
            Assert.AreEqual(PathKey(bridge.ExplicitPath), PathKey(((RoadSegment)roundTrip.GetNode(bridge.Id)).ExplicitPath));
            Assert.True(bridge.PortIds.All(id => plan.Edit.Result.GetProfile(id).IsLooseEnd));
            var deletedAgain = Delete(plan.Edit.Result, bridge.Id, settings).Edit.Result;
            Assert.IsEmpty(deletedAgain.Nodes); Assert.IsEmpty(deletedAgain.Profiles);
        }

        [Test]
        public void Bridge_PreservesCurvedExternalRoadExactly()
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero, false);
            var left = Arm(graph, ix, p => p.X < -1);
            var far = graph.GetProfile(left.PortIds[0]); var near = graph.GetProfile(left.PortIds[1]);
            var path = ControlPointCurve.Solve(new DVec2(-100, -100), new DVec2(-100, 0), near.Position, 1, 6, out _);
            graph.MoveProfile(far, path.StartPoint, path.StartTangent);
            left.Path = path; left.ExplicitPath = SegmentPathResolver.Store(path);
            var expected = PathKey(left.ExplicitPath);
            var branch = Arm(graph, ix, p => p.Y < -1);
            var plan = Delete(graph, branch.Id, RoadBuildSettings.Default);
            var merged = plan.Edit.Result.Segments().Single();
            Assert.AreEqual(expected, PathKey(merged.ExplicitPath.Take(left.ExplicitPath.Count).ToList()));
        }

        [Test]
        public void SharedRoadId_DeletesOnlySelectedSegmentAndKeepsSharedProfile()
        {
            var graph = new RoadGraph(); var a = graph.CreateProfile(DVec2.Zero, DVec2.UnitX);
            var middle = graph.CreateProfile(new DVec2(30, 0), DVec2.UnitX);
            var b = graph.CreateProfile(new DVec2(60, 0), DVec2.UnitX);
            var left = graph.AddSegment(a, middle, "same-group"); var right = graph.AddSegment(middle, b, "same-group");
            var plan = Delete(graph, left.Id, RoadBuildSettings.Default);
            Assert.NotNull(plan.Edit.Result.GetNode(right.Id)); Assert.IsNull(plan.Edit.Result.GetProfile(a.Id));
            Assert.True(plan.Edit.Result.GetProfile(middle.Id).IsLooseEnd);
            Assert.True(plan.Edit.ChangedNodes.Contains(right.Id));
        }

        [Test]
        public void DeleteOnlyRoad_RemovesOrphanProfiles()
        {
            var graph = new RoadGraph(); var road = Road(graph, DVec2.Zero, new DVec2(20, 0));
            var result = Delete(graph, road.Id, RoadBuildSettings.Default).Edit.Result;
            Assert.IsEmpty(result.Nodes); Assert.IsEmpty(result.Profiles);
        }

        static Intersection Stub(RoadGraph graph, int arms)
        {
            var ix = new Intersection { HasAnchor = true }; graph.Nodes[ix.Id] = ix;
            for (int i = 0; i < arms; i++)
            {
                var s = Road(graph, new DVec2(0, i * 30), new DVec2(20, i * 30));
                var p = graph.GetProfile(s.PortIds[0]); p.NodeBId = ix.Id; ix.PortIds.Add(p.Id);
            }
            return ix;
        }

        [TestCase(1)] [TestCase(2)]
        public void ZeroOrOneRemainingArm_CleansIntersection(int initialArms)
        {
            var graph = new RoadGraph(); var ix = Stub(graph, initialArms);
            var s = graph.Segments().First();
            var result = Delete(graph, s.Id, RoadBuildSettings.Default).Edit.Result;
            Assert.IsNull(result.GetNode(ix.Id)); Assert.AreEqual(initialArms - 1, result.Segments().Count());
            Assert.True(result.Profiles.Values.All(p => p.IsLooseEnd && p.NodeAId != ix.Id && p.NodeBId != ix.Id));
        }

        [Test]
        public void SameJunctionAtBothEnds_IsProcessedOnce()
        {
            var graph = new RoadGraph(); var s = Road(graph, DVec2.Zero, new DVec2(20, 0));
            var ix = new Intersection(); graph.Nodes[ix.Id] = ix;
            foreach (var id in s.PortIds) { ix.PortIds.Add(id); graph.GetProfile(id).NodeBId = ix.Id; }
            var result = Delete(graph, s.Id, RoadBuildSettings.Default).Edit.Result;
            Assert.IsEmpty(result.Nodes); Assert.IsEmpty(result.Profiles);
        }

        [Test]
        public void IncompatibleLayout_RejectsEntireDeletion()
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero, false);
            var target = Arm(graph, ix, p => p.Y < -1);
            var left = Arm(graph, ix, p => p.X < -1);
            foreach (var id in left.PortIds) graph.GetProfile(id).Lanes[1].Width += 1;
            var before = Fingerprint(graph); var version = graph.Revision;
            Assert.False(RoadDeletePlanner.TryPlan(graph, target.Id, RoadBuildSettings.Default, out var plan, out var reason));
            Assert.IsNull(plan); Assert.IsNotEmpty(reason);
            Assert.AreEqual(before, Fingerprint(graph)); Assert.AreEqual(version, graph.Revision);
        }

        [TestCase(false)] [TestCase(true)]
        public void RoadBetweenTwoJunctions_UpdatesAtomically(bool secondFails)
        {
            var graph = new RoadGraph();
            var horizontal = Road(graph, new DVec2(-120, 0), new DVec2(420, 0));
            var v1 = Road(graph, new DVec2(0, -120), new DVec2(0, 120));
            Assert.True(IntersectionBuilder.DetectAndBuild(graph, v1.Id));
            var v2 = Road(graph, new DVec2(300, -120), new DVec2(300, secondFails ? 0 : 120));
            Assert.True(IntersectionBuilder.DetectAndBuild(graph, v2.Id));
            if (secondFails)
            {
                var right = graph.Segments().Single(s => s.PortIds.Any(id => graph.GetProfile(id).Position.X > 419));
                foreach (var id in right.PortIds) graph.GetProfile(id).Lanes[1].Width += 1;
            }
            var middle = graph.Segments().Single(s => s.PortIds.All(id =>
                { var p = graph.GetProfile(id); return graph.GetNode(p.NodeAId) is Intersection || graph.GetNode(p.NodeBId) is Intersection; }));
            var before = Fingerprint(graph);
            bool valid = RoadDeletePlanner.TryPlan(graph, middle.Id, RoadBuildSettings.Default, out var plan, out var reason);
            Assert.AreEqual(!secondFails, valid, reason);
            Assert.AreEqual(before, Fingerprint(graph));
            if (valid) Assert.True(plan.Edit.Result.Nodes.Values.OfType<Intersection>().All(ix => ix.PortIds.Count == 3));
            else Assert.IsNull(plan);
        }

        [Test]
        public void VersionAndSettingsChange_InvalidateDeletionPlan()
        {
            var graph = new RoadGraph(); var road = Road(graph, DVec2.Zero, new DVec2(20, 0));
            var plan = Delete(graph, road.Id, RoadBuildSettings.Default);
            Assert.True(plan.IsCurrent(graph, RoadBuildSettings.Default));
            var settings = RoadBuildSettings.Default; settings.Curve = 1;
            Assert.False(plan.IsCurrent(graph, settings));
            graph.NotifyChanged(new[] { road.Id }); Assert.False(plan.IsCurrent(graph, RoadBuildSettings.Default));
        }

        [TestCase(0, 0.0)] [TestCase(1, 0.0)] [TestCase(2, 0.0)]
        [TestCase(0, .5)] [TestCase(1, .5)] [TestCase(2, .5)]
        [TestCase(0, 1.0)] [TestCase(1, 1.0)] [TestCase(2, 1.0)]
        public void TeeDeleteAnyArm_ConnectsRemainingRoadsWithLineOrArc(int arm, double curve)
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero, false);
            var target = Arm(graph, ix, p => arm == 0 ? p.Y < -1 : arm == 1 ? p.X < -1 : p.X > 1);
            var survivors = graph.Segments().Where(s => s != target).Select(s => s.Id).ToArray();
            var settings = RoadBuildSettings.Default; settings.Curve = curve;
            var result = Delete(graph, target.Id, settings).Edit.Result;
            Assert.IsNull(result.GetNode(ix.Id));
            var bridge = result.Segments().Single();
            Assert.True(survivors.All(id => result.GetNode(id) == null));
            var path = SegmentPathResolver.Resolve(result, bridge);
            Assert.AreEqual(arm != 0, path.Elements.OfType<ArcElement>().Any());
            Assert.True(bridge.PortIds.All(id => result.GetProfile(id).IsLooseEnd));
            for (int i = 1; i < path.Elements.Count; i++)
            {
                Assert.Less(DVec2.Distance(path.Elements[i - 1].EndPoint, path.Elements[i].StartPoint), 1e-6);
                Assert.Greater(DVec2.Dot(path.Elements[i - 1].EndTangent, path.Elements[i].StartTangent), .99999);
            }
            var loaded = RoadGraphMapper.FromDto(RoadGraphMapper.ToDto(result));
            Assert.AreEqual(PathKey(bridge.ExplicitPath), PathKey(((RoadSegment)loaded.GetNode(bridge.Id)).ExplicitPath));
        }

        [TestCase(false)] [TestCase(true)]
        public void TightCorner_RetreatsPortsAndBuildsSafeCircularTurn(bool reverseDrawing)
        {
            var graph = new RoadGraph();
            Road(graph, new DVec2(reverseDrawing ? 120 : -120, 0), new DVec2(reverseDrawing ? -120 : 120, 0));
            var branch = Road(graph, new DVec2(0, reverseDrawing ? 0 : -120), new DVec2(0, reverseDrawing ? -120 : 0));
            Assert.True(IntersectionBuilder.DetectAndBuild(graph, branch.Id));
            var ix = graph.Nodes.Values.OfType<Intersection>().Single();
            var target = Arm(graph, ix, p => p.X > 1);
            var remainingPorts = ix.PortIds.Select(graph.GetProfile).Where(p => !target.PortIds.Contains(p.Id)).ToArray();
            foreach (var p in remainingPorts) graph.MoveProfile(p, p.Position.Normalized * 2, p.Direction);
            var result = Delete(graph, target.Id, RoadBuildSettings.Default).Edit.Result;
            var bridge = result.Segments().Single(s => graph.GetNode(s.Id) == null);
            var path = SegmentPathResolver.Resolve(result, bridge);
            Assert.True(path.Elements.OfType<ArcElement>().Any());
            Assert.True(path.Elements.OfType<ArcElement>().All(a => a.Radius > 5.51));
            Assert.True(bridge.PortIds.All(id => result.GetProfile(id).Position.Length > 2));
        }

        [Test]
        public void ReversalAcrossSharedProfiles_PreservesGeometryAndRemoteLaneConnections()
        {
            var graph = new RoadGraph(); var ix = Junction(graph, DVec2.Zero, false);
            var target = Arm(graph, ix, p => p.X > 1);
            // Both surviving arms end at the junction. Either may be reversed depending on port order.
            foreach (var road in graph.Segments().Where(s => s != target).ToArray())
            {
                var far = graph.GetProfile(road.PortIds[0]);
                var extension = graph.AddSegment(graph.CreateProfile(far.Position - far.Direction * 100, far.Direction), far);
                var remote = new Intersection { HasAnchor = true, Anchor = far.Position - far.Direction * 120 };
                graph.Nodes.Add(remote.Id, remote);
                var port = graph.GetProfile(extension.PortIds[0]); port.NodeBId = remote.Id; remote.PortIds.Add(port.Id);
                // Two additional arms point away from the remote junction.
                foreach (int sign in new[] { -1, 1 })
                {
                    var d = far.Direction.PerpLeft * sign;
                    var side = Road(graph, remote.Anchor + d * 20, remote.Anchor + d * 100);
                    var sp = graph.GetProfile(side.PortIds[0]); sp.NodeBId = remote.Id; remote.PortIds.Add(sp.Id);
                }
                IntersectionBuilder.BuildLaneLinks(graph, remote);
            }
            var originals = graph.Segments().Where(s => s != target && !s.PortIds.Any(ix.PortIds.Contains))
                .ToDictionary(s => s.Id, s => SegmentPathResolver.Resolve(graph, s));
            var result = Delete(graph, target.Id, RoadBuildSettings.Default).Edit.Result;
            var merged = result.Segments().Single(s => graph.GetNode(s.Id) == null);
            Assert.True(merged.PortIds.All(id => !result.GetProfile(id).IsLooseEnd));
            foreach (var pair in originals)
            {
                var after = SegmentPathResolver.Resolve(result, (RoadSegment)result.GetNode(pair.Key));
                Assert.AreEqual(pair.Value.TotalLength, after.TotalLength, 1e-6);
                bool reversed = !after.StartPoint.ApproxEquals(pair.Value.StartPoint);
                for (int i = 0; i <= 10; i++)
                {
                    PathChainUtil.PointAt(pair.Value, pair.Value.TotalLength * i / 10, out var expected, out _);
                    PathChainUtil.PointAt(after, after.TotalLength * (reversed ? 10 - i : i) / 10, out var actual, out _);
                    Assert.Less(DVec2.Distance(expected, actual), 1e-6);
                }
            }
            foreach (var remote in result.Nodes.Values.OfType<Intersection>())
            {
                var before = (Intersection)graph.GetNode(remote.Id);
                Assert.AreEqual(before.LaneLinks.Count, remote.LaneLinks.Count);
                foreach (var link in remote.LaneLinks)
                {
                    var from = result.GetProfile(link.FromPortId); var to = result.GetProfile(link.ToPortId);
                    Assert.AreEqual(LaneType.Car, from.Lanes[link.FromLane].Type);
                    Assert.AreEqual(LaneType.Car, to.Lanes[link.ToLane].Type);
                }
            }
        }
    }
}
