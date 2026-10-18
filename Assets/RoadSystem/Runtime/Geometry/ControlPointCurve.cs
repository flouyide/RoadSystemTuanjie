using System;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>Three-click curve: the middle click is the virtual corner of two tangent guides.</summary>
    public static class ControlPointCurve
    {
        public static PathChain Solve(DVec2 start, DVec2 control, DVec2 end,
            double minimumLeg, double minimumRadius, out string reason)
        {
            reason = null;
            var first = control - start;
            var second = end - control;
            double firstLength = first.Length, secondLength = second.Length;
            if (firstLength < minimumLeg || secondLength < minimumLeg
                || DVec2.Distance(start, end) < minimumLeg)
            {
                reason = "起点、控制点或终点距离过近";
                return null;
            }

            var u = first / firstLength;
            var v = second / secondLength;
            double turn = Math.Atan2(DVec2.Cross(u, v), DVec2.Dot(u, v));
            if (Math.Abs(turn) >= Math.PI - 0.05)
            {
                reason = "弯路不能接近掉头";
                return null;
            }
            var path = new PathChain();
            if (Math.Abs(turn) < 0.015)
            {
                path.Add(new LineElement(start, end));
                return path;
            }

            // Use the largest circular fillet fitting both guide legs. Zero-length
            // approach/departure lines are omitted by PathChain.Add.
            double tangentDistance = Math.Min(firstLength, secondLength);
            double radius = tangentDistance / Math.Tan(Math.Abs(turn) * 0.5);
            if (radius < minimumRadius || double.IsNaN(radius) || double.IsInfinity(radius))
            {
                reason = "转弯半径过小";
                return null;
            }
            var entry = control - u * tangentDistance;
            var exit = control + v * tangentDistance;
            double sign = Math.Sign(turn);
            var center = entry + u.PerpLeft * (sign * radius);
            double startAngle = Math.Atan2(entry.Y - center.Y, entry.X - center.X);
            path.Add(new LineElement(start, entry));
            path.Add(new ArcElement(center, radius, startAngle, turn));
            path.Add(new LineElement(exit, end));
            if (DVec2.Distance(path.EndPoint, end) > 1e-5)
            {
                reason = "弯路圆弧无法求解";
                return null;
            }
            return path;
        }
    }
}
