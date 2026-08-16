using NUnit.Framework;
using RoadSystem.Core;

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
    }
}
