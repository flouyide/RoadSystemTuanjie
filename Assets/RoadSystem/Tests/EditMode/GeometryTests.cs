using System;
using System.Collections.Generic;
using NUnit.Framework;
using RoadSystem.Core;
using RoadSystem.Geometry;

namespace RoadSystem.Tests
{
    /// <summary>
    /// M1 验收：正交/锐角/钝角/S 形/平行同向/半平面约束 6 类连接全部解出且 G1 连续；
    /// 弧采样弦高误差 ≤ ε。
    /// </summary>
    public class GeometryTests
    {
        const double Eps = 1e-4;

        static void AssertG1(PathChain chain)
        {
            for (int i = 0; i + 1 < chain.Elements.Count; i++)
            {
                var e0 = chain.Elements[i];
                var e1 = chain.Elements[i + 1];
                Assert.True(e0.EndPoint.ApproxEquals(e1.StartPoint, Eps),
                    $"元素{i} 接点位置不连续: {e0.EndPoint} vs {e1.StartPoint}");
                Assert.True(e0.EndTangent.ApproxEquals(e1.StartTangent, Eps),
                    $"元素{i} 接点切向不连续(G1): {e0.EndTangent} vs {e1.StartTangent}");
            }
        }

        static PathChain Connect(DVec2 a, DVec2 dA, DVec2 b, DVec2 dB)
        {
            var chain = ProfileConnector.ConnectCenters(a, dA, b, dB, 0);
            Assert.NotNull(chain, "应解出路径");
            Assert.Greater(chain.Elements.Count, 0);
            Assert.True(chain.StartPoint.ApproxEquals(a, Eps), $"起点位置: {chain.StartPoint} vs {a}");
            Assert.True(chain.EndPoint.ApproxEquals(b, Eps), $"终点位置: {chain.EndPoint} vs {b}");
            Assert.True(chain.StartTangent.ApproxEquals(dA.Normalized, Eps), $"起点切向: {chain.StartTangent} vs {dA}");
            Assert.True(chain.EndTangent.ApproxEquals(dB.Normalized, Eps), $"终点切向: {chain.EndTangent} vs {dB}");
            AssertG1(chain);
            return chain;
        }

        [Test]
        public void Collinear_ProducesSingleLine()
        {
            var c = Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(10, 0), new DVec2(1, 0));
            Assert.AreEqual(1, c.Elements.Count);
            Assert.IsInstanceOf<LineElement>(c.Elements[0]);
        }

        [Test]
        public void Orthogonal90_ProducesSingleArc()
        {
            var c = Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(10, 10), new DVec2(0, 1));
            Assert.AreEqual(1, c.Elements.Count);
            var arc = c.Elements[0] as ArcElement;
            Assert.NotNull(arc);
            Assert.AreEqual(10.0, arc.Radius, 1e-3);
        }

        [Test]
        public void AcuteAngle_Solves()
        {
            Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(12, 3), new DVec2(0.8944, 0.4472));
        }

        [Test]
        public void ObtuseAngle_Solves()
        {
            Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(8, 10), new DVec2(-0.3162, 0.9487));
        }

        [Test]
        public void ParallelShift_SShape_Solves()
        {
            var c = Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(20, 8), new DVec2(1, 0));
            Assert.GreaterOrEqual(c.Elements.Count, 2, "S 形应为多元素路径");
        }

        [Test]
        public void PureLateralShift_Solves()
        {
            Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(0, 8), new DVec2(1, 0));
        }

        [Test]
        public void ParallelOpposite_UTurn_HalfCircle()
        {
            var c = Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(10, 8), new DVec2(-1, 0));
            var last = c.Elements[c.Elements.Count - 1] as ArcElement;
            Assert.NotNull(last, "U 形回转末元素应为半圆");
            Assert.AreEqual(4.0, last.Radius, 1e-3);
        }

        [Test]
        public void HalfPlane_DirectlyBehind_Fails()
        {
            // dst 正后方：强制同向后仍不可解（不允许逆行），返回 null 由编辑器阻止放置
            var chain = ProfileConnector.ConnectCenters(
                new DVec2(0, 0), new DVec2(1, 0), new DVec2(-10, 0), new DVec2(0, 1), 0);
            Assert.IsNull(chain);
        }

        [Test]
        public void SCurve_NonParallel_Solves()
        {
            Connect(new DVec2(0, 0), new DVec2(1, 0), new DVec2(10, 6), new DVec2(0.6, 0.8));
        }

        [Test]
        public void ArcSampling_ChordErrorWithinSagitta()
        {
            double eps = 0.05, r = 10;
            var chain = new PathChain();
            chain.Add(new ArcElement(new DVec2(0, 0), r, 0, Math.PI / 2));
            var pts = PathSampler.Sample(chain, eps);
            Assert.Greater(pts.Count, 2);
            for (int i = 0; i + 1 < pts.Count; i++)
            {
                var mid = (pts[i].Position + pts[i + 1].Position) * 0.5;
                double dev = r - mid.Length; // 弦高
                Assert.LessOrEqual(dev, eps * 1.001, $"弦高误差超限: {dev}");
            }
            Assert.AreEqual(r * Math.PI / 2, pts[pts.Count - 1].ArcLength, 1e-6);
        }

        [Test]
        public void LineArcIntersection_HalfCircle_OneHit()
        {
            var arc = new ArcElement(new DVec2(0, 0), 5, -Math.PI / 2, Math.PI); // 右半圆
            var hits = new List<DVec2>();
            int n = ArcIntersections.LineArc(new DVec2(-10, 0), new DVec2(10, 0), arc, hits);
            Assert.AreEqual(1, n);
        }

        [Test]
        public void ArcArcIntersection_Solves()
        {
            var a1 = new ArcElement(new DVec2(0, 0), 10, 0, Math.PI);
            var a2 = new ArcElement(new DVec2(10, 0), 10, 0, Math.PI);
            var hits = new List<DVec2>();
            Assert.GreaterOrEqual(ArcIntersections.ArcArc(a1, a2, hits), 1);
        }
    }
}
