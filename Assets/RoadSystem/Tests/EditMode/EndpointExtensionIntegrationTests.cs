using System.Linq;
using System.Reflection;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;
using UnityEditor;
using UnityEngine;

namespace RoadSystem.Tests
{
    public class EndpointExtensionIntegrationTests
    {
        GameObject root;
        RoadNetworkBehaviour net;
        RoadPlayerBuilder player;
        const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Instance;

        [SetUp]
        public void SetUp()
        {
            root = new GameObject("EndpointExtensionTest"); root.SetActive(false);
            net = root.AddComponent<RoadNetworkBehaviour>(); player = root.AddComponent<RoadPlayerBuilder>();
            Set("net", net);
        }

        [TearDown]
        public void TearDown() { Undo.ClearUndo(net); Object.DestroyImmediate(root); }

        void Set(string field, object value) => typeof(RoadPlayerBuilder).GetField(field, Private).SetValue(player, value);
        T Get<T>(string field) => (T)typeof(RoadPlayerBuilder).GetField(field, Private).GetValue(player);
        static object Call(object obj, string method, params object[] args)
            => obj.GetType().GetMethod(method, Private).Invoke(obj, args);

        RoadSegment Road(DVec2 a, DVec2 b, string group = null)
            => net.AddSegment(net.PlaceProfile(a, (b - a).Normalized), net.PlaceProfile(b, (b - a).Normalized), group);

        [TestCase(0f)] [TestCase(7f)]
        public void PreviousEnd_IsSelectableBeforeGridAndOrphanIsIgnored(float grid)
        {
            var road = Road(new DVec2(-40, .27), new DVec2(10.3, .27));
            var end = net.Graph.GetProfile(road.PortIds[1]);
            net.PlaceProfile(end.Position + new DVec2(.1, .1), DVec2.UnitX);
            Set("gridSnap", grid); Set("lastRoadEndId", end.Id);
            object[] args = { end.Position + new DVec2(.11, .11), 0f, null, null };
            var picked = (Profile)Call(player, "ResolveEndpoint", args);
            Assert.AreEqual(end.Id, picked.Id); Assert.AreEqual(end.Position, (DVec2)args[0]);
            Set("looseEndSnap", 0f);
            Assert.IsNull(Call(player, "FindLooseEnd", end.Position, null));
        }

        [TestCase(0f)] [TestCase(7f)]
        public void ThreeClicks_AssistControlThenMergeIntoOneObject(float grid)
        {
            var left = Road(new DVec2(-40.2, 4.3), new DVec2(.3, 4.3), "left-group");
            Road(new DVec2(50.5, 54.8), new DVec2(50.5, 104.8), "right-group");
            Call(net, "RebuildAll");
            string before = RoadGraphJson.ToJson(net.Graph);
            Set("gridSnap", grid); Set("buildMode", RoadBuildMode.Curve);
            Call(player, "OnCurveClick", new DVec2(.3, 4.3), 0f);
            Call(player, "OnCurveClick", new DVec2(50.7, 4.4), 0f);
            Assert.AreEqual(2, Get<int>("curveStage"));
            Assert.Less(DVec2.Distance(Get<DVec2>("curveControlPos"), new DVec2(50.5, 4.3)), 1e-8);
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
            Undo.IncrementCurrentGroup();
            Call(player, "OnCurveClick", new DVec2(50.6, 54.9), 0f);
            Assert.AreEqual(0, Get<int>("curveStage"));
            Assert.AreEqual(1, net.Graph.Segments().Count());
            Assert.AreEqual(1, root.transform.childCount);
            var view = root.GetComponentsInChildren<RoadNodeView>(true).Single();
            Assert.True(net.TryGetSegment(view.GetComponent<Collider>(), out _, out _));
            Assert.AreSame(view.GetComponent<MeshFilter>().sharedMesh, view.GetComponent<MeshCollider>().sharedMesh);
            Assert.True(EndpointExtension.TryGetRoad(net.Graph, Get<string>("lastRoadEndId"), out _));
            Undo.FlushUndoRecordObjects(); Undo.PerformUndo(); Call(net, "Update");
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
            Undo.PerformRedo(); Call(net, "Update");
            Assert.AreEqual(1, root.transform.childCount);
        }

        [Test]
        public void InvalidFixedControl_RejectsThirdClickWithoutMovingOldRoads()
        {
            Road(new DVec2(-40, 0), DVec2.Zero);
            Road(new DVec2(50, 50), new DVec2(50, 100));
            string before = RoadGraphJson.ToJson(net.Graph);
            Call(player, "OnCurveClick", DVec2.Zero, 0f);
            Call(player, "OnCurveClick", new DVec2(30, 0), 0f);
            object[] args = { new DVec2(50, 50), 0f, null, null, null, null };
            Assert.False((bool)Call(player, "TryCurveCandidate", args));
            Assert.That((string)args[5], Does.Contain("切线"));
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
            Call(player, "CancelPending");
            Assert.AreEqual(0, Get<int>("curveStage")); Assert.IsNull(Get<string>("curveStartId"));
            Assert.AreEqual(before, RoadGraphJson.ToJson(net.Graph));
        }
    }
}
