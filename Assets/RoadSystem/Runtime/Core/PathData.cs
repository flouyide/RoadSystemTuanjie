using System;
using System.Collections.Generic;

namespace RoadSystem.Core
{
    /// <summary>
    /// 路径元素：直线或圆弧。整条 PathChain 首尾相切（G1 连续）。
    /// 纯数据，由 L1 几何层解算生成，作为 RoadSegment 的缓存。
    /// </summary>
    public abstract class PathElement
    {
        public abstract DVec2 StartPoint { get; }
        public abstract DVec2 EndPoint { get; }
        public abstract DVec2 StartTangent { get; }
        public abstract DVec2 EndTangent { get; }
        public abstract double Length { get; }
    }

    public sealed class LineElement : PathElement
    {
        public DVec2 A;
        public DVec2 B;

        public LineElement(DVec2 a, DVec2 b) { A = a; B = b; }

        public override DVec2 StartPoint => A;
        public override DVec2 EndPoint => B;
        public override DVec2 StartTangent => (B - A).Normalized;
        public override DVec2 EndTangent => (B - A).Normalized;
        public override double Length => (B - A).Length;
    }

    /// <summary>
    /// 圆弧元素。Sweep > 0 为逆时针（左转），Sweep < 0 为顺时针（右转）。
    /// StartAngle 为起点相对圆心的极角（弧度）。
    /// </summary>
    public sealed class ArcElement : PathElement
    {
        public DVec2 Center;
        public double Radius;
        public double StartAngle;
        public double Sweep;

        public ArcElement(DVec2 center, double radius, double startAngle, double sweep)
        {
            Center = center;
            Radius = radius;
            StartAngle = startAngle;
            Sweep = sweep;
        }

        public double EndAngle => StartAngle + Sweep;

        public DVec2 PointAtAngle(double theta)
            => Center + new DVec2(Math.Cos(theta), Math.Sin(theta)) * Radius;

        /// <summary>theta 处沿行进方向的单位切向。</summary>
        public DVec2 TangentAtAngle(double theta)
        {
            var t = new DVec2(-Math.Sin(theta), Math.Cos(theta)); // CCW 方向
            return Sweep >= 0 ? t : -t;
        }

        public override DVec2 StartPoint => PointAtAngle(StartAngle);
        public override DVec2 EndPoint => PointAtAngle(EndAngle);
        public override DVec2 StartTangent => TangentAtAngle(StartAngle);
        public override DVec2 EndTangent => TangentAtAngle(EndAngle);
        public override double Length => Math.Abs(Sweep) * Radius;
    }

    public sealed class PathChain
    {
        public readonly List<PathElement> Elements = new List<PathElement>();

        public double TotalLength
        {
            get
            {
                double L = 0;
                foreach (var e in Elements) L += e.Length;
                return L;
            }
        }

        public DVec2 StartPoint => Elements.Count > 0 ? Elements[0].StartPoint : DVec2.Zero;
        public DVec2 EndPoint => Elements.Count > 0 ? Elements[Elements.Count - 1].EndPoint : DVec2.Zero;
        public DVec2 StartTangent => Elements.Count > 0 ? Elements[0].StartTangent : DVec2.UnitX;
        public DVec2 EndTangent => Elements.Count > 0 ? Elements[Elements.Count - 1].EndTangent : DVec2.UnitX;

        public void Add(PathElement e)
        {
            if (e == null || e.Length < 1e-9) return;
            Elements.Add(e);
        }

        public void Append(PathChain other)
        {
            if (other == null) return;
            foreach (var e in other.Elements) Add(e);
        }
    }

    /// <summary>采样点：位置 / 单位切向 / 累计弧长。</summary>
    public struct PathPoint
    {
        public DVec2 Position;
        public DVec2 Tangent;
        public double ArcLength;
    }
}
