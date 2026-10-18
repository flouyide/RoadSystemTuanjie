using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;
using RoadSystem.Meshing;
using UnityEditor;
using UnityEngine;

namespace RoadSystem.Tests
{
    /// <summary>
    /// M4 路口验收：十字/T 型自动生成、port 挂接、车道配对、退化合并、边界环合法性。
    /// </summary>
    public class IntersectionTests
    {
        /// <summary>横段 (-30,0)→(30,0) 与竖段 (0,-30)→(0,30) 垂直互穿 → 十字路口。</summary>
        (RoadGraph g, RoadSegment h, RoadSegment v) MakeCross()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(-30, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            var h = g.AddSegment(pa, pb);
            var pc = g.CreateProfile(new DVec2(0, -30), new DVec2(0, 1));
            var pd = g.CreateProfile(new DVec2(0, 30), new DVec2(0, 1));
            var v = g.AddSegment(pc, pd);
            return (g, h, v);
        }

        [Test]
        public void Cross_BuildsFourWayIntersection()
        {
            var (g, h, v) = MakeCross();
            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id), "十字互穿应建路口");

            var ix = g.Nodes.Values.OfType<Intersection>().SingleOrDefault();
            Assert.NotNull(ix, "应存在 1 个 Intersection 节点");
            Assert.AreEqual(4, ix.PortIds.Count, "十字路口应有 4 个 port");

            // 原两段各拆成两半 → 共 4 段
            Assert.AreEqual(4, g.Nodes.Values.OfType<RoadSegment>().Count());
            Assert.IsNull(g.GetNode(h.Id), "被穿越的横段应已删除（拆成两半）");
            Assert.IsNull(g.GetNode(v.Id), "穿越的竖段应已删除（拆成两半）");

            // 每个 port 都挂上路口节点
            foreach (var pid in ix.PortIds)
            {
                var p = g.GetProfile(pid);
                Assert.NotNull(p);
                Assert.True(p.NodeAId == ix.Id || p.NodeBId == ix.Id, $"port {pid} 应挂接 Intersection");
                Assert.False(p.IsLooseEnd, "路口 port 应挂两个节点（半段 + 路口）");
            }

            // 车道配对已生成
            Assert.Greater(ix.LaneLinks.Count, 0, "应生成默认 LaneConnection 配对");
        }

        [Test]
        public void Cross_PortsAreOrderedAndBoundaryLoopIsValid()
        {
            var (g, _, v) = MakeCross();
            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();

            var ports = ix.PortIds.Select(id => g.GetProfile(id)).ToList();
            var center = IntersectionBuilder.CenterOf(ports);
            var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05);
            Assert.NotNull(loop);
            Assert.GreaterOrEqual(loop.Count, 8, "含 fillet 转角采样的边界环应有不少于 8 个顶点");

            // 环闭合（首尾不重合但首点=绕一圈回到起点）：检查无重复顶点
            for (int i = 0; i < loop.Count; i++)
                for (int j = i + 1; j < loop.Count; j++)
                    Assert.False(loop[i].Pos.ApproxEquals(loop[j].Pos, 1e-4),
                        $"环顶点重复: {i} vs {j}");

            // 有向面积为正（数学 CCW，供三角化）
            double area2 = 0;
            for (int i = 0; i < loop.Count; i++)
                area2 += DVec2.Cross(loop[i].Pos, loop[(i + 1) % loop.Count].Pos);
            Assert.Greater(area2, 0, "边界环应为 CCW 方向");
        }

        [Test]
        public void Cross_CornerUsesRequestedCircularRadius()
        {
            var (g, _, v) = MakeCross();
            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();
            var ports = IntersectionBuilder.LivePorts(g, ix);
            var center = IntersectionBuilder.CenterOf(ports);
            var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.02, 5);

            var west = ports.Single(p => p.Position.X < -10 && System.Math.Abs(p.Position.Y) < 1e-5);
            var south = ports.Single(p => p.Position.Y < -10 && System.Math.Abs(p.Position.X) < 1e-5);
            var corner = IntersectionBuilder.BuildCornerPath(west, west.RightEdge,
                south, south.LeftEdge, 5);
            Assert.AreEqual(3, corner.Elements.Count);
            Assert.IsInstanceOf<ArcElement>(corner.Elements[1]);
            Assert.AreEqual(5, ((ArcElement)corner.Elements[1]).Radius, 1e-6);
            var fitted = IntersectionBuilder.BuildCornerPath(west, west.RightEdge,
                south, south.LeftEdge, 20);
            Assert.Less(((ArcElement)fitted.Elements[1]).Radius, 20,
                "可用直边不足时应自动缩小圆角半径");
            for (int i = 0; i + 1 < corner.Elements.Count; i++)
            {
                Assert.True(corner.Elements[i].EndPoint.ApproxEquals(corner.Elements[i + 1].StartPoint, 1e-6));
                Assert.Greater(DVec2.Dot(corner.Elements[i].EndTangent,
                    corner.Elements[i + 1].StartTangent), 0.99999);
            }

            // 西南角两条外缘分别在 y=-5.5、x=-5.5；其 5m fillet 圆心为 (-10.5,-10.5)。
            var arcPoints = loop.Where(p => p.Pos.X > -10.4 && p.Pos.X < -5.6
                && p.Pos.Y > -10.4 && p.Pos.Y < -5.6).ToList();
            Assert.GreaterOrEqual(arcPoints.Count, 2, "路口转角应包含采样后的圆弧");
            foreach (var p in arcPoints)
                Assert.AreEqual(5, DVec2.Distance(p.Pos, new DVec2(-10.5, -10.5)), 1e-5);
        }

        [TestCase(30, false)]
        [TestCase(90, false)]
        [TestCase(150, false)]
        [TestCase(30, true)]
        [TestCase(90, true)]
        [TestCase(150, true)]
        public void IntersectionCurve_MorphsToStraightWithoutBreakingSurface(double degrees, bool tee)
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            double radians = degrees * System.Math.PI / 180;
            var branch = new DVec2(System.Math.Cos(radians), System.Math.Sin(radians));
            g.AddSegment(g.CreateProfile(east * -100, east), g.CreateProfile(east * 100, east));
            var other = g.AddSegment(g.CreateProfile(branch * -100, branch),
                g.CreateProfile(branch * (tee ? 0 : 100), branch));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, other.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();
            var ports = IntersectionBuilder.LivePorts(g, ix);
            var center = IntersectionBuilder.CenterOf(ports);
            var original = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05, 5);
            var zero = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05, 5, 0);
            Assert.AreEqual(original.Count, zero.Count);
            for (int i = 0; i < original.Count; i++)
                Assert.True(original[i].Pos.ApproxEquals(zero[i].Pos, 1e-8));

            foreach (double curve in new[] { 0d, 0.25d, 0.5d, 0.75d, 1d })
            {
                var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05, 5, curve);
                Assert.NotNull(loop, $"curve={curve}");
                if (curve == 1d) Assert.AreEqual(ports.Count * 2, loop.Count,
                    "直边模式每个 port 只需两个截面端点");
                for (int i = 0; i < loop.Count; i++)
                for (int j = i + 2; j < loop.Count; j++)
                {
                    if (i == 0 && j == loop.Count - 1) continue;
                    Assert.False(ArcIntersections.LineLine(loop[i].Pos, loop[(i + 1) % loop.Count].Pos,
                        loop[j].Pos, loop[(j + 1) % loop.Count].Pos, out _),
                        $"curve={curve} 边界自交");
                }
                var triangles = new List<int>();
                Assert.True(PolygonTriangulator.TryTriangulate(loop, triangles),
                    $"curve={curve} 三角化失败");
                Assert.AreEqual((loop.Count - 2) * 3, triangles.Count);
            }
        }

        [Test]
        public void IntersectionMaterial_IsAvailableAndCanBeAssigned()
        {
            var material = AssetDatabase.LoadAssetAtPath<Material>(
                "Assets/RoadSystem/Materials/M_Intersection.mat");
            Assert.NotNull(material);
            var go = new GameObject("IntersectionMaterialTest");
            try
            {
                var network = go.AddComponent<RoadNetworkBehaviour>();
                network.IntersectionMaterial = material;
                Assert.AreSame(material, network.IntersectionMaterial);
            }
            finally { Object.DestroyImmediate(go); }
        }

        [Test]
        public void Tee_NewSegmentEndOnExistingRoad()
        {
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(-30, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            g.AddSegment(pa, pb);
            var pc = g.CreateProfile(new DVec2(0, -30), new DVec2(0, 1));
            var pd = g.CreateProfile(new DVec2(0, 0), new DVec2(0, 1)); // 终点正好落在横段上
            var v = g.AddSegment(pc, pd);

            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id), "终点落在既有路上应建 T 型路口");

            var ix = g.Nodes.Values.OfType<Intersection>().SingleOrDefault();
            Assert.NotNull(ix);
            Assert.AreEqual(3, ix.PortIds.Count, "T 型路口应有 3 个 port");

            // 横段拆成两半 + 竖段截断（未拆分）→ 共 3 段
            Assert.AreEqual(3, g.Nodes.Values.OfType<RoadSegment>().Count());

            // 竖段终点被后退：不再位于 (0,0)
            var pdAfter = g.GetProfile(pd.Id);
            Assert.NotNull(pdAfter);
            Assert.Greater(DVec2.Distance(pdAfter.Position, new DVec2(0, 0)), 1.0,
                "T 型流入 port 应从交点后退");
        }

        [Test]
        public void PlacementPreview_RejectsCrossingTooCloseToExistingJunction()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var north = new DVec2(0, 1);
            g.AddSegment(g.CreateProfile(new DVec2(-100, 0), east),
                g.CreateProfile(new DVec2(100, 0), east));
            var first = g.AddSegment(g.CreateProfile(new DVec2(0, -100), north),
                g.CreateProfile(new DVec2(0, 100), north));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, first.Id));
            int nodesBefore = g.Nodes.Count, profilesBefore = g.Profiles.Count;

            var closeStart = Profile.Create(new DVec2(-25, -100), north, null);
            var closeEnd = Profile.Create(new DVec2(-25, 100), north, null);
            Assert.False(IntersectionBuilder.CanBuildRoad(g, closeStart, closeEnd, null,
                out var reason));
            Assert.IsNotEmpty(reason);
            Assert.AreEqual(nodesBefore, g.Nodes.Count, "预检不得修改活图");
            Assert.AreEqual(profilesBefore, g.Profiles.Count);

            var farStart = Profile.Create(new DVec2(-60, -100), north, null);
            var farEnd = Profile.Create(new DVec2(-60, 100), north, null);
            Assert.True(IntersectionBuilder.CanBuildRoad(g, farStart, farEnd, null, out _));
        }

        [Test]
        public void PlacementPreview_AllowsTeeAndRejectsSameRoadSelfCrossing()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var north = new DVec2(0, 1);
            g.AddSegment(g.CreateProfile(new DVec2(-100, 0), east),
                g.CreateProfile(new DVec2(100, 0), east), "road-a");
            var start = Profile.Create(new DVec2(0, -100), north, null);
            var end = Profile.Create(DVec2.Zero, north, null);
            Assert.True(IntersectionBuilder.CanBuildRoad(g, start, end, "road-b", out _));
            Assert.False(IntersectionBuilder.CanBuildRoad(g, start, end, "road-a", out _));
        }

        [Test]
        public void PlacementPreview_RejectsTwoCrossingsThatCannotBothBeSplit()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var north = new DVec2(0, 1);
            foreach (double y in new[] { 0d, 25d })
                g.AddSegment(g.CreateProfile(new DVec2(-100, y), east),
                    g.CreateProfile(new DVec2(100, y), east));
            var start = Profile.Create(new DVec2(0, -100), north, null);
            var end = Profile.Create(new DVec2(0, 100), north, null);
            Assert.False(IntersectionBuilder.CanBuildRoad(g, start, end, null, out _));
            Assert.AreEqual(2, g.Nodes.Count, "模拟拆分不得进入活图");
        }

        [TestCase(30)]
        [TestCase(90)]
        [TestCase(150)]
        public void Tee_ClickOnRoadSurfaceSnapsToCenterline(double degrees)
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            g.AddSegment(g.CreateProfile(east * -100, east), g.CreateProfile(east * 100, east));
            Assert.True(IntersectionBuilder.TrySnapToRoad(g, new DVec2(0, 3), 6,
                out var snapped));
            Assert.True(snapped.ApproxEquals(DVec2.Zero, 1e-6));
            Assert.False(IntersectionBuilder.TrySnapToRoad(g, new DVec2(0, 7), 6,
                out _), "路面外的终点不应吸附");

            double radians = degrees * System.Math.PI / 180;
            var branch = new DVec2(System.Math.Cos(radians), System.Math.Sin(radians));
            var tee = g.AddSegment(g.CreateProfile(branch * -100, branch),
                g.CreateProfile(snapped, branch));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, tee.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();
            Assert.AreEqual(3, ix.PortIds.Count);
            var ports = IntersectionBuilder.LivePorts(g, ix);
            var center = IntersectionBuilder.CenterOf(ports);
            var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05);
            var triangles = new List<int>();
            Assert.True(PolygonTriangulator.TryTriangulate(loop, triangles));
        }

        [Test]
        public void Tee_StartOnRoadSurfaceSnapsToCenterline()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var north = new DVec2(0, 1);
            g.AddSegment(g.CreateProfile(east * -100, east), g.CreateProfile(east * 100, east));
            Assert.True(IntersectionBuilder.TrySnapToRoad(g, new DVec2(0, 2), 6,
                out var snapped));
            var tee = g.AddSegment(g.CreateProfile(snapped, north),
                g.CreateProfile(north * 100, north));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, tee.Id));
            Assert.AreEqual(3, g.Nodes.Values.OfType<Intersection>().Single().PortIds.Count);
        }

        [Test]
        public void RoadSnap_ProjectsOntoArcCenterline()
        {
            var g = new RoadGraph();
            var start = g.CreateProfile(new DVec2(30, 0), new DVec2(0, 1));
            var end = g.CreateProfile(new DVec2(0, 30), new DVec2(-1, 0));
            var road = g.AddSegment(start, end);
            road.Path = new PathChain();
            road.Path.Add(new ArcElement(DVec2.Zero, 30, 0, System.Math.PI / 2));
            var radial = new DVec2(System.Math.Sqrt(0.5), System.Math.Sqrt(0.5));
            Assert.True(IntersectionBuilder.TrySnapToRoad(g, radial * 33, 6, out var snapped));
            Assert.True(snapped.ApproxEquals(radial * 30, 1e-5));
        }

        [Test]
        public void MergeDegenerate_TwoPortsMergeBackToSegment()
        {
            var (g, _, v) = MakeCross();
            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();

            // 按位置找 port：相交道路总半宽 + 转角余量 = 14m。
            // 剩余 aOut 段（aOut 为其终点）+ bIn 段（bIn 为其起点）→ 恰好可合并回一段
            Profile FindPort(DVec2 pos)
                => IntersectionBuilder.LivePorts(g, ix)
                    .First(p => p.Position.ApproxEquals(pos, 0.5));

            string SegOf(Profile port)
            {
                Assert.True(port.NodeAId == ix.Id || port.NodeBId == ix.Id);
                return port.NodeAId == ix.Id ? port.NodeBId : port.NodeAId;
            }

            g.RemoveSegment(SegOf(FindPort(new DVec2(14, 0))));
            IntersectionBuilder.MergeDegenerate(g, ix.Id);          // 剩 3 port，不合并
            Assert.NotNull(g.GetNode(ix.Id), "3-port 路口不应合并");

            g.RemoveSegment(SegOf(FindPort(new DVec2(0, -14))));
            IntersectionBuilder.MergeDegenerate(g, ix.Id);          // 剩 2 port，合并
            Assert.IsNull(g.GetNode(ix.Id), "2-port 路口应被拆除");

            var segs = g.Nodes.Values.OfType<RoadSegment>().ToList();
            Assert.AreEqual(1, segs.Count, "剩余两半段应合并回 1 段");

            // 合并段的两端 profile 仍可解析
            foreach (var pid in segs[0].PortIds)
                Assert.NotNull(g.GetProfile(pid));
        }

        [Test]
        public void Detect_EndpointConnection_DoesNotBuild()
        {
            // 链式续接（共享 profile 端点）不应误建路口
            var g = new RoadGraph();
            var pa = g.CreateProfile(new DVec2(0, 0), new DVec2(1, 0));
            var pb = g.CreateProfile(new DVec2(30, 0), new DVec2(1, 0));
            g.AddSegment(pa, pb);
            var pc = g.CreateProfile(new DVec2(30, 20), new DVec2(-1, 0));
            var seg2 = g.AddSegment(pb, pc); // 从 pb 续接弯出

            Assert.False(IntersectionBuilder.DetectAndBuild(g, seg2.Id), "端点续接不应建路口");
            Assert.AreEqual(0, g.Nodes.Values.OfType<Intersection>().Count());
        }

        [Test]
        public void Cross_ConnectsBothTrafficDirections()
        {
            var (g, _, v) = MakeCross();
            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();
            var ports = IntersectionBuilder.LivePorts(g, ix);
            foreach (var port in ports)
            {
                Assert.True(ix.LaneLinks.Any(l => l.FromPortId == port.Id));
                Assert.True(ix.LaneLinks.Any(l => l.ToPortId == port.Id));
            }
            foreach (var link in ix.LaneLinks)
            {
                var from = g.GetProfile(link.FromPortId);
                var to = g.GetProfile(link.ToPortId);
                var center = IntersectionBuilder.CenterOf(ports);
                var inDir = from.Lanes[link.FromLane].Forward ? from.Direction : -from.Direction;
                var outDir = to.Lanes[link.ToLane].Forward ? to.Direction : -to.Direction;
                Assert.Greater(DVec2.Dot(inDir, center - from.Position), 0);
                Assert.Less(DVec2.Dot(outDir, center - to.Position), 0);
                Assert.NotNull(link.Path);
            }
        }

        [Test]
        public void ObliqueCross_ProducesTriangulatedSurface()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var diagonal = new DVec2(System.Math.Sqrt(3) * 0.5, 0.5);
            var h = g.AddSegment(g.CreateProfile(east * -100, east),
                g.CreateProfile(east * 100, east));
            var d = g.AddSegment(g.CreateProfile(diagonal * -100, diagonal),
                g.CreateProfile(diagonal * 100, diagonal));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, d.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();
            Assert.AreEqual(4, ix.PortIds.Count);
            Assert.AreEqual(4, g.Segments().Count());
            var ports = IntersectionBuilder.LivePorts(g, ix);
            var center = IntersectionBuilder.CenterOf(ports);
            var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05);
            Assert.NotNull(loop);
            var mesh = IntersectionMeshBuilder.Build(loop, center, 0);
            Assert.NotNull(mesh);
            Assert.Greater(mesh.triangles.Length, 0);
            UnityEngine.Object.DestroyImmediate(mesh);
        }

        [TestCase(15)]
        [TestCase(30)]
        [TestCase(60)]
        [TestCase(120)]
        [TestCase(165)]
        public void AngledTee_HasSimpleTriangulableBoundary(double degrees)
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            double radians = degrees * System.Math.PI / 180;
            var branch = new DVec2(System.Math.Cos(radians), System.Math.Sin(radians));
            g.AddSegment(g.CreateProfile(east * -100, east), g.CreateProfile(east * 100, east));
            var incoming = g.AddSegment(g.CreateProfile(branch * -100, branch),
                g.CreateProfile(DVec2.Zero, branch));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, incoming.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();
            Assert.AreEqual(3, ix.PortIds.Count);
            var ports = IntersectionBuilder.LivePorts(g, ix);
            var center = IntersectionBuilder.CenterOf(ports);
            var loop = IntersectionBuilder.BuildBoundaryLoop(ports, center, 0.05);
            Assert.NotNull(loop);

            foreach (var port in ports)
            {
                var inward = (center - port.Position).Normalized;
                foreach (var vertex in loop)
                {
                    var delta = vertex.Pos - port.Position;
                    if (System.Math.Abs(DVec2.Dot(delta, port.Direction.PerpLeft)) < port.TotalWidth * 0.5 - 1e-5)
                        Assert.GreaterOrEqual(DVec2.Dot(delta, inward), -1e-5,
                            "路口过渡面不应伸入路段端口之外");
                }
            }

            for (int i = 0; i < loop.Count; i++)
            for (int j = i + 2; j < loop.Count; j++)
            {
                if (i == 0 && j == loop.Count - 1) continue;
                Assert.False(ArcIntersections.LineLine(loop[i].Pos, loop[(i + 1) % loop.Count].Pos,
                    loop[j].Pos, loop[(j + 1) % loop.Count].Pos, out _));
            }
            var triangles = new List<int>();
            Assert.True(PolygonTriangulator.TryTriangulate(loop, triangles));
            Assert.AreEqual((loop.Count - 2) * 3, triangles.Count);
            double polygonArea2 = 0, triangleArea2 = 0;
            for (int i = 0; i < loop.Count; i++)
                polygonArea2 += DVec2.Cross(loop[i].Pos, loop[(i + 1) % loop.Count].Pos);
            for (int i = 0; i < triangles.Count; i += 3)
            {
                var a = loop[triangles[i]].Pos;
                var b = loop[triangles[i + 1]].Pos;
                var c = loop[triangles[i + 2]].Pos;
                double area2 = DVec2.Cross(b - a, c - a);
                Assert.Greater(area2, 0);
                triangleArea2 += area2;
            }
            Assert.AreEqual(polygonArea2, triangleArea2, 1e-5);
        }

        [Test]
        public void CurvedSegment_LineArcCrossingBuildsIntersection()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var north = new DVec2(0, 1);
            g.AddSegment(g.CreateProfile(new DVec2(-100, -15), east),
                g.CreateProfile(new DVec2(100, -15), east));
            var curved = g.AddSegment(g.CreateProfile(new DVec2(0, -30), east),
                g.CreateProfile(new DVec2(30, 50), north));
            curved.Path = new PathChain();
            curved.Path.Add(new ArcElement(DVec2.Zero, 30, -System.Math.PI / 2, System.Math.PI / 2));
            curved.Path.Add(new LineElement(new DVec2(30, 0), new DVec2(30, 50)));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, curved.Id));
            Assert.AreEqual(1, g.Nodes.Values.OfType<Intersection>().Count());
        }

        [Test]
        public void SegmentCrossingTwoRoads_BuildsBothIntersections()
        {
            var g = new RoadGraph();
            var east = new DVec2(1, 0);
            var north = new DVec2(0, 1);
            g.AddSegment(g.CreateProfile(new DVec2(-100, 0), east),
                g.CreateProfile(new DVec2(100, 0), east));
            g.AddSegment(g.CreateProfile(new DVec2(-100, 60), east),
                g.CreateProfile(new DVec2(100, 60), east));
            var crossing = g.AddSegment(g.CreateProfile(new DVec2(0, -100), north),
                g.CreateProfile(new DVec2(0, 160), north));
            Assert.True(IntersectionBuilder.DetectAndBuild(g, crossing.Id));
            Assert.AreEqual(2, g.Nodes.Values.OfType<Intersection>().Count());
            foreach (var ix in g.Nodes.Values.OfType<Intersection>())
                Assert.AreEqual(4, IntersectionBuilder.LivePorts(g, ix).Count);
        }
    }
}
