using System;
using System.Collections.Generic;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>
    /// 路径离散化采样。
    /// 弧段：弦高误差（sagitta）控制，步长 Δθ = 2·acos(1 - ε/r)；
    /// 直线段：按最大步长细分。输出位置/切向/累计弧长。
    /// </summary>
    public static class PathSampler
    {
        public static List<PathPoint> Sample(PathChain path, double sagitta = GeomConsts.DefaultSagitta)
        {
            var pts = new List<PathPoint>();
            if (path == null) return pts;
            Sample(path, sagitta, pts);
            return pts;
        }

        public static void Sample(PathChain path, double sagitta, List<PathPoint> outPts)
        {
            outPts.Clear();
            if (path == null || path.Elements.Count == 0) return;

            double arcLen = 0;
            bool first = true;
            foreach (var e in path.Elements)
            {
                if (e is LineElement line)
                {
                    int n = Math.Max(1, (int)Math.Ceiling(line.Length / GeomConsts.MaxLineStep));
                    int start = first ? 0 : 1; // 避免与上一元素末端重复
                    for (int i = start; i <= n; i++)
                    {
                        double t = (double)i / n;
                        var p = DVec2.Lerp(line.A, line.B, t);
                        outPts.Add(new PathPoint
                        {
                            Position = p,
                            Tangent = line.StartTangent,
                            ArcLength = arcLen + line.Length * t
                        });
                    }
                    arcLen += line.Length;
                }
                else if (e is ArcElement arc)
                {
                    // Δθ = 2·acos(1 - ε/r)，保证弦高误差 ≤ ε
                    double dTheta = 2.0 * Math.Acos(GeomConsts.Clamp(1.0 - sagitta / arc.Radius, -1.0, 1.0));
                    dTheta = Math.Max(dTheta, 1e-3);
                    int n = Math.Max(1, (int)Math.Ceiling(Math.Abs(arc.Sweep) / dTheta));
                    int start = first ? 0 : 1;
                    for (int i = start; i <= n; i++)
                    {
                        double t = (double)i / n;
                        double theta = arc.StartAngle + arc.Sweep * t;
                        outPts.Add(new PathPoint
                        {
                            Position = arc.PointAtAngle(theta),
                            Tangent = arc.TangentAtAngle(theta),
                            ArcLength = arcLen + arc.Length * t
                        });
                    }
                    arcLen += arc.Length;
                }
                first = false;
            }
        }
    }
}
