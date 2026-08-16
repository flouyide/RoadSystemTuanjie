using System;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>
    /// 两个 profile 间的平滑连接总入口（文章3 判定树）：
    /// 等长 profile 只需对中心线解一次 —— 同一圆心旋转出同心弧即得其余车道边线。
    ///
    /// 判定树：
    ///  0. 半平面防御：dst 落在 src 后方半平面 → 强制 dB = dA
    ///  1. 延续线相交且交点在双方前方 → 单 fillet（Arc+Line 或 Line+Arc）
    ///  2. 平行同向：共线 → 直线；横向偏移 → 半圆 + 直线
    ///  3. 平行反向 / 交点在后方（S 形平移等）→ Hermite 中间 profile，t=0.5 求值，递归拆分
    ///  4. 递归深度上限 MaxDepth，仍不可解返回 null（编辑器层应阻止该放置）
    /// </summary>
    public static class ProfileConnector
    {
        public static PathChain Connect(Profile src, Profile dst)
        {
            if (src == null || dst == null) return null;
            return ConnectCenters(src.Position, src.Direction, dst.Position, dst.Direction, 0);
        }

        public static PathChain ConnectCenters(DVec2 a, DVec2 dA, DVec2 b, DVec2 dB, int depth)
        {
            dA = dA.Normalized;
            dB = dB.Normalized;

            // 0. 半平面约束（几何层防御，编辑器层同样前置约束）
            if (DVec2.Dot(b - a, dA) < 0)
                dB = dA;

            double dist = DVec2.Distance(a, b);
            if (dist < GeomConsts.DistEps) return null;

            double denom = DVec2.Cross(dA, dB);

            if (Math.Abs(denom) < GeomConsts.ParallelEps)
            {
                if (DVec2.Dot(dA, dB) > 0)
                {
                    // 平行同向：共线且目标在前方 → 直线；
                    // 横向偏移（S 形平移）→ Hermite 中间 profile 拆成两个 fillet。
                    // 注：单个半圆无法保持行进方向不变（180° 后切向反转），
                    // 半圆+直线仅适用于反向平行的 U 形回转（见下）。
                    var collinear = TryCollinear(a, dA, b);
                    if (collinear != null) return collinear;
                    return HermiteSplit(a, dA, b, dB, depth);
                }
                // 平行反向 → U 形回转：直线 + 半圆（或 半圆 + 直线）
                var uTurn = TryUTurn(a, dA, b);
                if (uTurn != null) return uTurn;
                return HermiteSplit(a, dA, b, dB, depth);
            }

            // 非平行：尝试单 fillet
            var fillet = Fillet.Solve(a, dA, b, dB);
            if (fillet != null) return fillet;

            // 交点在后方等不可单 fillet 情形 → Hermite 拆分
            return HermiteSplit(a, dA, b, dB, depth);
        }

        /// <summary>平行同向且共线、目标在正前方 → 单直线；否则返回 null。</summary>
        static PathChain TryCollinear(DVec2 a, DVec2 dA, DVec2 b)
        {
            DVec2 w = b - a;
            double longitudinal = DVec2.Dot(w, dA);
            if (longitudinal < GeomConsts.DistEps) return null; // 目标在后方：沿线逆行，非法
            DVec2 lateral = w - dA * longitudinal;
            if (lateral.Length > GeomConsts.DistEps) return null;
            var chain = new PathChain();
            chain.Add(new LineElement(a, b));
            return chain;
        }

        /// <summary>
        /// 平行反向（dB = -dA）→ U 形回转：半圆直径 = 横向偏移。
        /// 目标纵向在前：Line + 半圆；纵向在后：半圆 + Line。
        /// </summary>
        static PathChain TryUTurn(DVec2 a, DVec2 dA, DVec2 b)
        {
            DVec2 w = b - a;
            double longitudinal = DVec2.Dot(w, dA);
            DVec2 lateral = w - dA * longitudinal;
            double latLen = lateral.Length;
            if (latLen < GeomConsts.DistEps) return null; // 正对调头无横向空间，交给 Hermite

            double r = latLen * 0.5;
            if (r < GeomConsts.MinRadius) return null;

            DVec2 n = lateral / latLen; // 横向单位向量（指向 b 侧）
            double turnSign = DVec2.Cross(dA, n) >= 0 ? 1.0 : -1.0;

            var chain = new PathChain();
            if (longitudinal >= 0)
            {
                // Line(A→A+dA·lon) + 半圆(终点 = B，切向 -dA = dB)
                DVec2 lineEnd = a + dA * longitudinal;
                DVec2 center = lineEnd + n * r;
                double startAngle = Math.Atan2(lineEnd.Y - center.Y, lineEnd.X - center.X);
                chain.Add(new LineElement(a, lineEnd));
                chain.Add(new ArcElement(center, r, startAngle, turnSign * Math.PI));
            }
            else
            {
                // 半圆(A→A+lateral，切向 -dA) + Line(→B)
                DVec2 center = a + n * r;
                double startAngle = Math.Atan2(a.Y - center.Y, a.X - center.X);
                DVec2 arcEnd = a + lateral;
                chain.Add(new ArcElement(center, r, startAngle, turnSign * Math.PI));
                chain.Add(new LineElement(arcEnd, b));
            }
            return chain;
        }

        /// <summary>
        /// Hermite 中间 profile：以两中心与方向建三次 Hermite 样条，
        /// 切向模长取中心距的 HermiteTangentScale 倍（1.0~1.5 推荐区间），
        /// 在 t=0.5 求 P（中间 profile 中心）与 P′（切向），递归拆成两个子问题。均为 O(1) 闭式。
        /// </summary>
        static PathChain HermiteSplit(DVec2 a, DVec2 dA, DVec2 b, DVec2 dB, int depth)
        {
            if (depth >= GeomConsts.MaxDepth) return null;

            double dist = DVec2.Distance(a, b);
            if (dist < GeomConsts.DistEps) return null;

            double m = GeomConsts.HermiteTangentScale * dist;
            DVec2 m0 = dA * m;
            DVec2 m1 = dB * m;

            // P(0.5)  = 0.5(a+b) + 0.125(m0 - m1)
            // P'(0.5) = 1.5(b-a) - 0.25(m0 + m1)
            DVec2 midPos = (a + b) * 0.5 + (m0 - m1) * 0.125;
            DVec2 midTan = (b - a) * 1.5 - (m0 + m1) * 0.25;
            if (midTan.Length < GeomConsts.DistEps) return null;
            midTan = midTan.Normalized;

            var first = ConnectCenters(a, dA, midPos, midTan, depth + 1);
            if (first == null) return null;
            var second = ConnectCenters(midPos, midTan, b, dB, depth + 1);
            if (second == null) return null;

            first.Append(second);
            return first;
        }
    }
}
