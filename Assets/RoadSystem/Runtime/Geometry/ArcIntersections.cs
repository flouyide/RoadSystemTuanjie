using System;
using System.Collections.Generic;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>
    /// 求交工具：全部为 O(1) 闭式解（文章1 的核心论据），供路口交汇检测使用。
    /// 严禁引入贝塞尔迭代求交。
    /// </summary>
    public static class ArcIntersections
    {
        /// <summary>线段-线段求交。命中返回 true 并输出交点。</summary>
        public static bool LineLine(DVec2 p1, DVec2 p2, DVec2 p3, DVec2 p4, out DVec2 hit)
        {
            hit = DVec2.Zero;
            DVec2 d1 = p2 - p1, d2 = p4 - p3;
            double denom = DVec2.Cross(d1, d2);
            if (Math.Abs(denom) < GeomConsts.ParallelEps) return false;
            double t = DVec2.Cross(p3 - p1, d2) / denom;
            double u = DVec2.Cross(p3 - p1, d1) / denom;
            if (t < -GeomConsts.RayTol || t > 1 + GeomConsts.RayTol) return false;
            if (u < -GeomConsts.RayTol || u > 1 + GeomConsts.RayTol) return false;
            hit = p1 + d1 * t;
            return true;
        }

        /// <summary>线段-圆弧求交，输出全部交点（0~2 个）。</summary>
        public static int LineArc(DVec2 a, DVec2 b, ArcElement arc, List<DVec2> hits)
        {
            int count = 0;
            DVec2 d = b - a;
            double len = d.Length;
            if (len < GeomConsts.DistEps) return 0;
            d = d / len;

            // |a + t·d - c|² = r² 的一元二次方程
            DVec2 f = a - arc.Center;
            double qa = 1.0; // d 已单位化
            double qb = 2.0 * DVec2.Dot(f, d);
            double qc = DVec2.Dot(f, f) - arc.Radius * arc.Radius;
            double disc = qb * qb - 4 * qa * qc;
            if (disc < 0) return 0;

            double sq = Math.Sqrt(disc);
            for (int i = 0; i < 2; i++)
            {
                double t = (-qb + (i == 0 ? -sq : sq)) / (2 * qa);
                if (t < -GeomConsts.RayTol || t > len + GeomConsts.RayTol) continue;
                DVec2 p = a + d * GeomConsts.Clamp(t, 0, len);
                if (AngleOnArc(arc, p))
                {
                    hits.Add(p);
                    count++;
                }
            }
            return count;
        }

        /// <summary>圆弧-圆弧求交，输出全部交点（0~2 个）。</summary>
        public static int ArcArc(ArcElement a1, ArcElement a2, List<DVec2> hits)
        {
            int count = 0;
            DVec2 d = a2.Center - a1.Center;
            double dist = d.Length;
            if (dist < GeomConsts.DistEps) return 0; // 同心：无交点或无穷多（重合），均不处理
            if (dist > a1.Radius + a2.Radius + GeomConsts.DistEps) return 0;
            if (dist < Math.Abs(a1.Radius - a2.Radius) - GeomConsts.DistEps) return 0;

            double aa = (a1.Radius * a1.Radius - a2.Radius * a2.Radius + dist * dist) / (2 * dist);
            double h2 = a1.Radius * a1.Radius - aa * aa;
            if (h2 < 0) h2 = 0;
            double h = Math.Sqrt(h2);

            DVec2 pMid = a1.Center + d * (aa / dist);
            DVec2 perp = new DVec2(-d.Y, d.X) * (h / dist);

            void TryAdd(DVec2 p)
            {
                if (AngleOnArc(a1, p) && AngleOnArc(a2, p)) { hits.Add(p); count++; }
            }

            if (h < GeomConsts.DistEps) TryAdd(pMid);
            else { TryAdd(pMid + perp); TryAdd(pMid - perp); }
            return count;
        }

        /// <summary>判断点是否落在圆弧的扫掠范围内（容差内）。</summary>
        public static bool AngleOnArc(ArcElement arc, DVec2 p, double angleTol = 1e-4)
        {
            double theta = Math.Atan2(p.Y - arc.Center.Y, p.X - arc.Center.X);
            double rel = NormalizeAngle(theta - arc.StartAngle); // (-π, π]
            double sweep = arc.Sweep;
            if (sweep >= 0)
                return rel >= -angleTol && rel <= sweep + angleTol
                       || rel + 2 * Math.PI <= sweep + angleTol; // 跨 2π 情形
            return rel <= angleTol && rel >= sweep - angleTol
                   || rel - 2 * Math.PI >= sweep - angleTol;
        }

        public static double NormalizeAngle(double a)
        {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a <= -Math.PI) a += 2 * Math.PI;
            return a;
        }
    }
}
