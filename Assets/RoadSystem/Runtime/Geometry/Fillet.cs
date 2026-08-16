using System;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>
    /// 文章3 "The Geometrical Solution"：two-line fillet 变体 —— 一个切点固定、一个求解。
    /// 输入：点 A、方向 dA（A 侧切点固定）；点 B、方向 dB。
    /// 输出：Arc(A→M) + Line(M→B)，或对称的 Line(A→M) + Arc(M→B)；不可解返回 null。
    ///
    /// 构造（C 为两延续线交点，CA、CB 为到交点的距离）：
    ///   设 CB ≥ CA，在 CB 上取 M 使 CM = CA；过 M、A 分别作延续线垂线交于圆心 O；
    ///   全等三角形 OCM ≌ OCA 保证 OM = OA，故圆在 A、M 两处同时相切。
    ///
    /// 符号约定：路径从 A 出发沿 +dA 前进、到达 B 时沿 +dB 前进，
    /// 因此 C 必须位于 A 射线前方（tA ≥ 0）且位于 B 射线后方（s ≤ 0，C = B + dB·s）。
    /// </summary>
    public static class Fillet
    {
        public static PathChain Solve(DVec2 a, DVec2 dA, DVec2 b, DVec2 dB)
        {
            double denom = DVec2.Cross(dA, dB);
            if (Math.Abs(denom) < GeomConsts.ParallelEps) return null; // 平行：由调用方特判

            DVec2 w = b - a;
            double tA = DVec2.Cross(w, dB) / denom; // C = a + dA·tA，需 tA ≥ 0
            double s = DVec2.Cross(w, dA) / denom;  // C = b + dB·s，需 s ≤ 0

            if (tA < -GeomConsts.RayTol || s > GeomConsts.RayTol) return null;

            double cA = Math.Max(0, tA);
            double cB = Math.Max(0, -s);

            // 退化：几乎共点/共线 → 直接直线
            if (cA < GeomConsts.DistEps || cB < GeomConsts.DistEps)
                return Line(a, b);

            DVec2 c = a + dA * cA;

            if (cB >= cA)
            {
                // M 在 CB 上，CM = CA → Arc(A→M) + Line(M→B)
                DVec2 m = c + dB * cA;
                var arc = BuildArc(a, dA, m, dB);
                if (arc == null) return Line(a, b); // 半径退化兜底
                var chain = new PathChain();
                chain.Add(arc);
                chain.Add(new LineElement(m, b));
                return chain;
            }
            else
            {
                // M 在 CA 上，CM = CB → Line(A→M) + Arc(M→B)
                DVec2 m = c - dA * cB;
                var arc = BuildArc(m, dA, b, dB);
                if (arc == null) return Line(a, b);
                var chain = new PathChain();
                chain.Add(new LineElement(a, m));
                chain.Add(arc);
                return chain;
            }
        }

        /// <summary>
        /// 构造过 P0（切向 t0）与 P1（切向 t1）的圆弧。
        /// 圆心 O = 过 P0 垂直 t0 的直线 与 过 P1 垂直 t1 的直线 的交点。
        /// </summary>
        static ArcElement BuildArc(DVec2 p0, DVec2 t0, DVec2 p1, DVec2 t1)
        {
            DVec2 n0 = t0.PerpLeft;
            DVec2 n1 = t1.PerpLeft;
            double denom = DVec2.Cross(n0, n1); // == Cross(t0, t1)
            if (Math.Abs(denom) < GeomConsts.ParallelEps) return null;

            double s = DVec2.Cross(p1 - p0, n1) / denom;
            DVec2 o = p0 + n0 * s;
            double r = Math.Abs(s);
            if (r < GeomConsts.MinRadius) return null;

            // 转向符号：圆心在切向左侧 → 逆时针（正 sweep）
            double turnSign = DVec2.Cross(t0, o - p0) >= 0 ? 1.0 : -1.0;

            double startAngle = Math.Atan2(p0.Y - o.Y, p0.X - o.X);

            // |sweep| ∈ (0, π]：用起点/终点半径向量夹角 + 转向符号确定
            DVec2 v0 = p0 - o, v1 = p1 - o;
            double cosSweep = GeomConsts.Clamp(DVec2.Dot(v0, v1) / (r * r), -1.0, 1.0);
            double sweep = turnSign * Math.Acos(cosSweep);
            if (Math.Abs(sweep) < 1e-9) return null;

            var arc = new ArcElement(o, r, startAngle, sweep);
            // 防御：终点切向必须与 t1 一致（不一致说明应取优弧另一侧）
            if (DVec2.Dot(arc.EndTangent, t1) < 0)
                arc = new ArcElement(o, r, startAngle, -Math.Sign(sweep) * (2 * Math.PI - Math.Abs(sweep)));
            return arc;
        }

        static PathChain Line(DVec2 a, DVec2 b)
        {
            var chain = new PathChain();
            chain.Add(new LineElement(a, b));
            return chain;
        }
    }
}
