using System;
using System.Collections.Generic;
using System.Linq;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>Polygon tests shared by placement, intersection growth and previews (world XZ).</summary>
    public static class RoadFootprint
    {
        public static List<DVec2> Points(List<BoundaryVertex> loop)
            => loop?.Select(v => v.Pos).ToList();

        public static List<DVec2> Sweep(PathChain path, double width, double sagitta)
        {
            var samples = PathSampler.Sample(path, sagitta);
            var result = new List<DVec2>();
            foreach (var s in samples) result.Add(s.Position + s.Tangent.PerpLeft * (width * .5));
            for (int i = samples.Count - 1; i >= 0; i--)
                result.Add(samples[i].Position - samples[i].Tangent.PerpLeft * (width * .5));
            return result;
        }

        public static double DistanceToEdge(DVec2 p, DVec2 a, DVec2 b)
        {
            var d = b - a;
            double t = d.LengthSquared < 1e-16 ? 0
                : GeomConsts.Clamp(DVec2.Dot(p - a, d) / d.LengthSquared, 0, 1);
            return DVec2.Distance(p, a + d * t);
        }

        public static double Distance(List<DVec2> polygon, DVec2 p)
        {
            if (polygon == null || polygon.Count < 3) return double.PositiveInfinity;
            double result = double.PositiveInfinity;
            for (int i = 0; i < polygon.Count; i++)
                result = Math.Min(result, DistanceToEdge(p, polygon[i], polygon[(i + 1) % polygon.Count]));
            return result;
        }

        public static bool Contains(List<DVec2> polygon, DVec2 p, double tolerance = 1e-6)
        {
            if (polygon == null || polygon.Count < 3) return false;
            if (Distance(polygon, p) <= tolerance) return true;
            bool inside = false;
            for (int i = 0, j = polygon.Count - 1; i < polygon.Count; j = i++)
            {
                var a = polygon[i]; var b = polygon[j];
                if ((a.Y > p.Y) != (b.Y > p.Y)
                    && p.X < (b.X - a.X) * (p.Y - a.Y) / (b.Y - a.Y) + a.X)
                    inside = !inside;
            }
            return inside;
        }

        static void Cuts(DVec2 a, DVec2 b, DVec2 c, DVec2 d, List<double> cuts)
        {
            var u = b - a; var v = d - c;
            double cross = DVec2.Cross(u, v);
            if (Math.Abs(cross) > 1e-10)
            {
                double t = DVec2.Cross(c - a, v) / cross;
                double s = DVec2.Cross(c - a, u) / cross;
                if (t >= 0 && t <= 1 && s >= -1e-8 && s <= 1 + 1e-8) cuts.Add(t);
            }
            else if (u.LengthSquared > 1e-12 && Math.Abs(DVec2.Cross(c - a, u)) < 1e-7)
            {
                cuts.Add(GeomConsts.Clamp(DVec2.Dot(c - a, u) / u.LengthSquared, 0, 1));
                cuts.Add(GeomConsts.Clamp(DVec2.Dot(d - a, u) / u.LengthSquared, 0, 1));
            }
        }

        static List<double> EdgeCuts(List<DVec2> polygon, DVec2 a, DVec2 b)
        {
            var cuts = new List<double> { 0, 1 };
            for (int i = 0; i < polygon.Count; i++)
                Cuts(a, b, polygon[i], polygon[(i + 1) % polygon.Count], cuts);
            cuts.Sort();
            return cuts;
        }

        // Split at every boundary crossing: checking vertices alone misses concave notches.
        public static bool Covers(List<DVec2> outer, List<DVec2> inner, double tolerance)
        {
            if (inner == null || inner.Count < 3) return true;
            for (int i = 0; i < inner.Count; i++)
            {
                var a = inner[i]; var b = inner[(i + 1) % inner.Count];
                if (!Contains(outer, a, tolerance)) return false;
                var cuts = EdgeCuts(outer, a, b);
                for (int k = 1; k < cuts.Count; k++)
                    if (!Contains(outer, DVec2.Lerp(a, b, (cuts[k - 1] + cuts[k]) * .5), tolerance))
                        return false;
            }
            return true;
        }

        public static bool Overlaps(List<DVec2> a, List<DVec2> b, double tolerance)
        {
            if (a == null || b == null || a.Count < 3 || b.Count < 3) return false;
            if (a.Max(p => p.X) <= b.Min(p => p.X) + tolerance
                || b.Max(p => p.X) <= a.Min(p => p.X) + tolerance
                || a.Max(p => p.Y) <= b.Min(p => p.Y) + tolerance
                || b.Max(p => p.Y) <= a.Min(p => p.Y) + tolerance) return false;
            return Enters(a, b, tolerance) || Enters(b, a, tolerance);
        }

        static bool Enters(List<DVec2> polygon, List<DVec2> other, double tolerance)
        {
            if (polygon == null || other == null || polygon.Count < 3 || other.Count < 3) return false;
            for (int i = 0; i < other.Count; i++)
            {
                var a = other[i]; var b = other[(i + 1) % other.Count];
                if (Contains(polygon, a, 0) && Distance(polygon, a) > tolerance) return true;
                var cuts = EdgeCuts(polygon, a, b);
                for (int k = 1; k < cuts.Count; k++)
                {
                    var p = DVec2.Lerp(a, b, (cuts[k - 1] + cuts[k]) * .5);
                    if (Contains(polygon, p, 0) && Distance(polygon, p) > tolerance) return true;
                }
            }
            // Identical rectangles may have no strictly interior edge point.
            var mean = other.Aggregate(DVec2.Zero, (sum, p) => sum + p) / other.Count;
            return Contains(other, mean, 0) && Contains(polygon, mean, 0)
                && Distance(polygon, mean) > tolerance && Distance(other, mean) > tolerance;
        }

        public static bool IsSimple(List<DVec2> polygon)
        {
            if (polygon == null || polygon.Count < 3) return false;
            for (int i = 0; i < polygon.Count; i++)
            {
                var a = polygon[i]; var b = polygon[(i + 1) % polygon.Count];
                if (DVec2.Distance(a, b) < 1e-7) return false;
                for (int j = i + 2; j < polygon.Count; j++)
                {
                    if (i == 0 && j == polygon.Count - 1) continue;
                    var c = polygon[j]; var d = polygon[(j + 1) % polygon.Count];
                    var cuts = new List<double>();
                    Cuts(a, b, c, d, cuts);
                    if (cuts.Count > 0 && cuts.Any(t => DistanceToEdge(DVec2.Lerp(a, b, t), c, d) < 1e-7))
                        return false;
                }
            }
            return true;
        }
    }
}
