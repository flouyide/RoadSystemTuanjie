using System.Linq;
using System.Reflection;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEditor;
using UnityEngine;

namespace RoadSystem.Tests
{
    public class RoadDeletionIntegrationTests
    {
        GameObject root;
        RoadNetworkBehaviour net;

        [SetUp]
        public void SetUp()
        {
            root = new GameObject("RoadDeletionTest");
            net = root.AddComponent<RoadNetworkBehaviour>();
        }

        [TearDown]
        public void TearDown()
        {
            Undo.ClearUndo(net);
            Object.DestroyImmediate(root);
        }

        static void Invoke(object target, string name)
            => target.GetType().GetMethod(name, BindingFlags.NonPublic | BindingFlags.Instance).Invoke(target, null);

        string MakeCross()
        {
            net.AddSegment(net.PlaceProfile(new DVec2(-100, 0), DVec2.UnitX),
                net.PlaceProfile(new DVec2(100, 0), DVec2.UnitX));
            var vertical = net.AddSegment(net.PlaceProfile(new DVec2(0, -100), DVec2.UnitY),
                net.PlaceProfile(new DVec2(0, 100), DVec2.UnitY));
            Assert.True(net.AutoBuildIntersections(vertical.Id));
            Invoke(net, "RebuildAll");
            return net.Graph.Segments().First().Id;
        }

        RoadNodeView View(string id) => root.GetComponentsInChildren<RoadNodeView>(true).Single(v => v.NodeId == id);

        [Test]
        public void PreviewKeepsPickingStable_CommitUpdatesColliders_AndUndoRestoresGraph()
        {
            string target = MakeCross();
            string before = RoadGraphJson.ToJson(net.Graph);
            var oldView = View(target);
            Assert.True(net.TryPlanDelete(target, out var plan, out var reason), reason);
            Assert.True(net.ShowBuildPreview(plan.Edit, net.RoadMaterial));
            Assert.False(oldView.GetComponent<Renderer>().enabled);
            Assert.True(oldView.GetComponent<Collider>().enabled);
            Assert.True(net.TryGetSegment(oldView.GetComponent<Collider>(), out var selected, out _));
            Assert.AreEqual(target, selected);
            Undo.IncrementCurrentGroup();
            Assert.True(net.TryCommitDeletePlan(plan, out reason), reason);
            Assert.True(oldView == null);
            Assert.AreEqual(3, net.Graph.Segments().Count());
            var ix = net.Graph.Nodes.Values.OfType<Intersection>().Single();
            Assert.AreEqual(3, ix.PortIds.Count);
            var intersection = View(ix.Id);
            Assert.AreEqual("Inter_" + ix.Id, intersection.name);
            Assert.AreSame(net.IntersectionMaterial, intersection.GetComponent<Renderer>().sharedMaterial);
            Assert.False(net.TryGetSegment(intersection.GetComponent<Collider>(), out _, out _));
            foreach (var view in root.GetComponentsInChildren<RoadNodeView>())
                Assert.AreSame(view.GetComponent<MeshFilter>().sharedMesh, view.GetComponent<MeshCollider>().sharedMesh);
            Undo.FlushUndoRecordObjects();
            Undo.PerformUndo();
            Invoke(net, "Update");
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
            Assert.True(net.TryGetSegment(View(target).GetComponent<Collider>(), out _, out _));
            Undo.PerformRedo();
            Invoke(net, "Update");
            Assert.IsNull(net.Graph.GetNode(target));
        }

        [Test]
        public void MeshPreparationFailure_RestoresOriginalRenderingAndGraph()
        {
            string target = MakeCross();
            var graph = net.Graph; string before = RoadGraphJson.ToJson(graph);
            var original = View(target);
            Assert.True(net.TryPlanDelete(target, out var plan, out var reason), reason);
            Assert.True(net.ShowBuildPreview(plan.Edit, net.RoadMaterial));
            // Corrupt only the isolated result to exercise the host's mesh preparation failure.
            var surviving = plan.Edit.Result.Segments().First();
            plan.Edit.Result.GetProfile(surviving.PortIds[0]).Lanes = new LaneDef[0];
            Assert.False(net.TryCommitDeletePlan(plan, out reason));
            Assert.IsNotEmpty(reason);
            Assert.AreSame(graph, net.Graph);
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
            Assert.True(original.GetComponent<Renderer>().enabled);
            Assert.True(net.TryGetSegment(original.GetComponent<Collider>(), out _, out _));
        }

        [Test]
        public void StalePlan_DoesNotRemoveRoadOrLeaveHiddenRenderer()
        {
            string target = MakeCross();
            Assert.True(net.TryPlanDelete(target, out var plan, out _));
            Assert.True(net.ShowBuildPreview(plan.Edit, net.RoadMaterial));
            net.Graph.NotifyChanged(new[] { target });
            Assert.False(net.TryCommitDeletePlan(plan, out _));
            Assert.NotNull(net.Graph.GetNode(target));
            Assert.True(View(target).GetComponent<Renderer>().enabled);
        }

        [Test]
        public void DeadEndCapsAndHeight_ProduceClosedSidewalkEnds()
        {
            var path = new PathChain(); path.Add(new LineElement(DVec2.Zero, new DVec2(30, 0)));
            var uncapped = SegmentMeshBuilder.Build(path, Profile.DefaultLanes(), DVec2.Zero);
            var capped = SegmentMeshBuilder.Build(path, Profile.DefaultLanes(), DVec2.Zero,
                endHeightDelta: 2, capStart: true, capEnd: true);
            try
            {
                Assert.AreEqual(uncapped.triangles.Length + 24, capped.triangles.Length);
                Assert.AreEqual(2.15f, capped.vertices.Max(v => v.y), 1e-5);
                Assert.True(capped.normals.Any(n => n.x < -.9f));
                Assert.True(capped.normals.Any(n => n.x > .9f));
            }
            finally { Object.DestroyImmediate(uncapped); Object.DestroyImmediate(capped); }
        }

        [TestCase(0)] [TestCase(1)] [TestCase(2)]
        public void DeleteTeeArm_ProducesOneGameObjectAndUndoRestoresAllArms(int arm)
        {
            string first = MakeCross();
            Assert.True(net.TryPlanDelete(first, out var crossPlan, out var reason), reason);
            Assert.True(net.TryCommitDeletePlan(crossPlan, out reason), reason);
            Undo.FlushUndoRecordObjects();
            Undo.IncrementCurrentGroup();
            var ix = net.Graph.Nodes.Values.OfType<Intersection>().Single();
            var p = net.Graph.GetProfile(ix.PortIds[arm]);
            string target = p.NodeAId == ix.Id ? p.NodeBId : p.NodeAId;
            string before = RoadGraphJson.ToJson(net.Graph);
            var oldViews = root.GetComponentsInChildren<RoadNodeView>().ToArray();
            Assert.True(net.TryPlanDelete(target, out var plan, out reason), reason);
            Assert.True(net.ShowBuildPreview(plan.Edit, net.RoadMaterial));
            Assert.True(net.TryCommitDeletePlan(plan, out reason), reason);
            Assert.True(oldViews.All(v => v == null));
            Assert.AreEqual(1, net.Graph.Nodes.Count);
            Assert.AreEqual(1, root.transform.childCount, "Merged road must not have an extra group GameObject");
            var view = root.GetComponentsInChildren<RoadNodeView>().Single();
            Assert.AreSame(root.transform, view.transform.parent);
            Assert.AreEqual(1, root.GetComponentsInChildren<MeshFilter>().Length);
            Assert.AreEqual(1, root.GetComponentsInChildren<MeshCollider>().Length);
            Assert.True(net.TryGetSegment(view.GetComponent<Collider>(), out var selected, out _));
            Assert.AreEqual(net.Graph.Segments().Single().Id, selected);
            Undo.FlushUndoRecordObjects();
            Undo.PerformUndo(); Invoke(net, "Update");
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
            Assert.AreEqual(4, root.GetComponentsInChildren<RoadNodeView>().Length);
            Undo.PerformRedo(); Invoke(net, "Update");
            Assert.AreEqual(1, root.transform.childCount);
            Assert.AreEqual(1, root.GetComponentsInChildren<RoadNodeView>().Length);
        }
    }
}
