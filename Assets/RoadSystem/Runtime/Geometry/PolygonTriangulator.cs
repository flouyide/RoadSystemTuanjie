using System.Collections.Generic;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>简单 CCW 边界环的耳切三角化。输出三角形保持数学 CCW 绕序。</summary>
    public static class PolygonTriangulator
    {
        public static bool TryTriangulate(IReadOnlyList<BoundaryVertex> loop, List<int> triangles)
        {
            if (loop == null || loop.Count < 3 || triangles == null) return false;
            int n = loop.Count;
            var remaining = new List<int>(n);
            for (int i = 0; i < n; i++) remaining.Add(i);
            double area = 0;
            for (int i = 0; i < n; i++)
                area += DVec2.Cross(loop[i].Pos, loop[(i + 1) % n].Pos);
            if (area <= 1e-8) return false;

            int initialCount = triangles.Count;
            int guard = n * n;
            while (remaining.Count > 3 && guard-- > 0)
            {
                bool found = false;
                for (int i = 0; i < remaining.Count; i++)
                {
                    int a = remaining[(i + remaining.Count - 1) % remaining.Count];
                    int b = remaining[i];
                    int c = remaining[(i + 1) % remaining.Count];
                    var pa = loop[a].Pos; var pb = loop[b].Pos; var pc = loop[c].Pos;
                    if (DVec2.Cross(pb - pa, pc - pb) <= 1e-9) continue;

                    bool occupied = false;
                    foreach (int k in remaining)
                    {
                        if (k == a || k == b || k == c) continue;
                        var p = loop[k].Pos;
                        if (DVec2.Cross(pb - pa, p - pa) >= -1e-9
                            && DVec2.Cross(pc - pb, p - pb) >= -1e-9
                            && DVec2.Cross(pa - pc, p - pc) >= -1e-9)
                        { occupied = true; break; }
                    }
                    if (occupied) continue;

                    triangles.Add(a); triangles.Add(b); triangles.Add(c);
                    remaining.RemoveAt(i);
                    found = true;
                    break;
                }
                if (!found)
                {
                    triangles.RemoveRange(initialCount, triangles.Count - initialCount);
                    return false;
                }
            }
            if (remaining.Count != 3)
            {
                triangles.RemoveRange(initialCount, triangles.Count - initialCount);
                return false;
            }
            triangles.Add(remaining[0]); triangles.Add(remaining[1]); triangles.Add(remaining[2]);
            return true;
        }
    }
}
