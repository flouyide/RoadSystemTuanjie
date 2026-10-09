using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;

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
        public void MergeDegenerate_TwoPortsMergeBackToSegment()
        {
            var (g, _, v) = MakeCross();
            Assert.True(IntersectionBuilder.DetectAndBuild(g, v.Id));
            var ix = g.Nodes.Values.OfType<Intersection>().Single();

            // 按位置找 port：aIn(8.5,0) 与 bOut(0,-8.5)；删其所属半段
            // 剩余 aOut 段（aOut 为其终点）+ bIn 段（bIn 为其起点）→ 恰好可合并回一段
            Profile FindPort(DVec2 pos)
                => IntersectionBuilder.LivePorts(g, ix)
                    .First(p => p.Position.ApproxEquals(pos, 0.5));

            string SegOf(Profile port)
            {
                Assert.True(port.NodeAId == ix.Id || port.NodeBId == ix.Id);
                return port.NodeAId == ix.Id ? port.NodeBId : port.NodeAId;
            }

            g.RemoveSegment(SegOf(FindPort(new DVec2(8.5, 0))));   // aIn 半段
            IntersectionBuilder.MergeDegenerate(g, ix.Id);          // 剩 3 port，不合并
            Assert.NotNull(g.GetNode(ix.Id), "3-port 路口不应合并");

            g.RemoveSegment(SegOf(FindPort(new DVec2(0, -8.5))));   // bOut 半段
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
    }
}
