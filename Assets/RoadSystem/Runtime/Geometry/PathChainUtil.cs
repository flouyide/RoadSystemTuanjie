using System;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>
    /// PathChain 弧长工具：按累计弧长定位点/切向、截取子路径。
    /// 供路口交汇检测与段拆分使用（M4）。
    /// </summary>
    public static class PathChainUtil
    {
        /// <summary>
        /// 按累计弧长 s 取点与单位切向。s 被钳制到 [0, TotalLength]。
        /// 空/无效路径返回 false。
        /// </summary>
        public static bool PointAt(PathChain path, double s, out DVec2 pos, out DVec2 tangent)
        {
            pos = DVec2.Zero;
            tangent = DVec2.UnitX;
            if (path == null || path.Elements.Count == 0) return false;

            double acc = 0;
            foreach (var e in path.Elements)
            {
                double len = e.Length;
                if (s <= acc + len || ReferenceEquals(e, path.Elements[path.Elements.Count - 1]))
                {
                    double t = len > 1e-9 ? (s - acc) / len : 0;
                    t = GeomConsts.Clamp(t, 0, 1);
                    if (e is LineElement line)
                    {
                        pos = DVec2.Lerp(line.A, line.B, t);
                        tangent = line.StartTangent;
                    }
                    else if (e is ArcElement arc)
                    {
                        double theta = arc.StartAngle + arc.Sweep * t;
                        pos = arc.PointAtAngle(theta);
                        tangent = arc.TangentAtAngle(theta);
                    }
                    else
                    {
                        return false;
                    }
                    return true;
                }
                acc += len;
            }
            return false; // 不可达（末元素兜底已覆盖）
        }

        /// <summary>
        /// 截取子路径 [s0, s1]（弧长参数，自动钳制并保证 s0 &lt; s1）。
        /// 边界点处元素被精确切断：直线按比例分割，圆弧按角度分割。
        /// 返回新 PathChain；区间退化（长度近 0）返回 null。
        /// </summary>
        public static PathChain SubPath(PathChain path, double s0, double s1)
        {
            if (path == null || path.Elements.Count == 0) return null;
            double total = path.TotalLength;
            s0 = GeomConsts.Clamp(s0, 0, total);
            s1 = GeomConsts.Clamp(s1, 0, total);
            if (s1 - s0 < GeomConsts.DistEps) return null;

            var result = new PathChain();
            double acc = 0;
            for (int i = 0; i < path.Elements.Count && acc < s1 - 1e-12; i++)
            {
                var e = path.Elements[i];
                double len = e.Length;
                double eStart = acc, eEnd = acc + len;

                if (eEnd <= s0 + 1e-12) { acc = eEnd; continue; }   // 完全在区间前
                if (eStart >= s1 - 1e-12) break;                     // 完全在区间后

                if (e is LineElement line)
                {
                    double t0 = len > 1e-9 ? GeomConsts.Clamp((s0 - eStart) / len, 0, 1) : 0;
                    double t1 = len > 1e-9 ? GeomConsts.Clamp((s1 - eStart) / len, 0, 1) : 1;
                    var a = DVec2.Lerp(line.A, line.B, t0);
                    var b = DVec2.Lerp(line.A, line.B, t1);
                    result.Add(new LineElement(a, b));
                }
                else if (e is ArcElement arc)
                {
                    double t0 = len > 1e-9 ? GeomConsts.Clamp((s0 - eStart) / len, 0, 1) : 0;
                    double t1 = len > 1e-9 ? GeomConsts.Clamp((s1 - eStart) / len, 0, 1) : 1;
                    double a0 = arc.StartAngle + arc.Sweep * t0;
                    double a1 = arc.StartAngle + arc.Sweep * t1;
                    var sub = new ArcElement(arc.Center, arc.Radius, a0, a1 - a0);
                    result.Add(sub);
                }
                acc = eEnd;
            }
            return result.Elements.Count > 0 ? result : null;
        }

        /// <summary>路径的包围盒（min/max 角点）；空路径返回 false。</summary>
        public static bool Bounds(PathChain path, out DVec2 min, out DVec2 max)
        {
            // C# 不允许局部函数捕获 out 参数 → 先落到局部变量，结束时写回
            double minX = double.MaxValue, minY = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue;
            min = DVec2.Zero;
            max = DVec2.Zero;
            if (path == null || path.Elements.Count == 0) return false;

            void Expand(DVec2 p)
            {
                if (p.X < minX) minX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.X > maxX) maxX = p.X;
                if (p.Y > maxY) maxY = p.Y;
            }

            foreach (var e in path.Elements)
            {
                if (e is LineElement line) { Expand(line.A); Expand(line.B); }
                else if (e is ArcElement arc)
                {
                    // 端点 + 轴向极值点（0/90/180/270°）中落在弧范围内的都纳入
                    Expand(arc.StartPoint);
                    Expand(arc.EndPoint);
                    for (int k = 0; k < 4; k++)
                    {
                        double theta = k * Math.PI / 2;
                        if (ArcIntersections.AngleOnArc(arc, arc.Center +
                                new DVec2(Math.Cos(theta), Math.Sin(theta)) * arc.Radius, 1e-6))
                            Expand(arc.PointAtAngle(theta));
                    }
                }
            }
            min = new DVec2(minX, minY);
            max = new DVec2(maxX, maxY);
            return true;
        }
    }
}
