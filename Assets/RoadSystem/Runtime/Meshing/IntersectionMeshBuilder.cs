using System.Collections.Generic;
using RoadSystem.Core;
using RoadSystem.Geometry;
using UnityEngine;
using UnityEngine.Rendering;

namespace RoadSystem.Meshing
{
    /// <summary>
    /// M4 路口网格构建（plan §5.3 过渡面三角化）：
    /// 输入边界环（IntersectionBuilder.BuildBoundaryLoop 的输出，含 fillet 转角采样点），
    /// 用耳切法三角化铺装顶面，支持斜交产生的凹边界环，
    /// 并沿环边生成外缘立面（顶面落到 -curb）。
    ///
    /// 顶点布局：顶面 = 环点 n 个；立面独立顶点 2n 个（硬边光影）。
    /// UV：世界对齐的 XZ / vPeriod 采样，密度与路段贴图周期（6m）一致。
    /// 注意：DVec2 的数学 CCW（XZ 平面 atan2 序）与 Unity 三角正面绕序相反——
    /// 每个 CCW 耳三角反转为 Unity 正面绕序，使正面朝上。
    /// </summary>
    public static class IntersectionMeshBuilder
    {
        /// <summary>
        /// 构建路口铺装 mesh。loop 为 CCW 边界环（2D XZ + Y）；origin 为局部原点（路口中心）；
        /// baseY 为挂点世界 Y（顶点 Y 相对化）；curb 为外缘立面落差；vPeriod 为贴图平铺周期（米）。
        /// </summary>
        public static Mesh Build(List<BoundaryVertex> loop, DVec2 origin, float baseY,
            float curb = 0.15f, float vPeriod = 6f)
        {
            var verts = new List<Vector3>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();
            var tris = new List<int>();

            int n = loop.Count;
            if (n >= 3)
            {
                // ---------- 顶面环点 ----------
                for (int i = 0; i < n; i++)
                {
                    var p = loop[i];
                    verts.Add(new Vector3(
                        (float)(p.Pos.X - origin.X), (float)(p.Y - baseY), (float)(p.Pos.Y - origin.Y)));
                    normals.Add(Vector3.up);
                    uvs.Add(new Vector2(
                        (float)(p.Pos.X - origin.X) / vPeriod, (float)(p.Pos.Y - origin.Y) / vPeriod));
                }

                var ccwTriangles = new List<int>();
                if (!PolygonTriangulator.TryTriangulate(loop, ccwTriangles)) return null;
                for (int i = 0; i < ccwTriangles.Count; i += 3)
                {
                    tris.Add(ccwTriangles[i]);
                    tris.Add(ccwTriangles[i + 2]);
                    tris.Add(ccwTriangles[i + 1]);
                }

                // ---------- 外缘立面（独立顶点：top [topBase, topBase+n)，bottom [botBase, botBase+n)） ----------
                int topBase = verts.Count;
                for (int i = 0; i < n; i++)
                {
                    var p = loop[i];
                    verts.Add(new Vector3(
                        (float)(p.Pos.X - origin.X), (float)(p.Y - baseY), (float)(p.Pos.Y - origin.Y)));
                }
                int botBase = verts.Count;
                for (int i = 0; i < n; i++)
                {
                    var p = loop[i];
                    verts.Add(new Vector3(
                        (float)(p.Pos.X - origin.X), (float)(p.Y - baseY) - curb, (float)(p.Pos.Y - origin.Y)));
                }
                for (int i = 0; i < n; i++)
                {
                    int j = (i + 1) % n;
                    // 外法线 = CCW 边向量的右侧（XZ 平面）
                    var e = loop[j].Pos - loop[i].Pos;
                    var nrm = new Vector3((float)e.Y, 0, -(float)e.X).normalized;
                    normals.Add(nrm);
                    uvs.Add(uvs[i]);
                }
                for (int i = 0; i < n; i++)
                {
                    normals.Add(Vector3.down);
                    uvs.Add(uvs[i]);
                }

                for (int i = 0; i < n; i++)
                {
                    int j = (i + 1) % n;
                    int ti = topBase + i, tj = topBase + j, bi = botBase + i, bj = botBase + j;
                    // (top_i, top_j, bot_i) 与 (top_j, bot_j, bot_i)：正面朝外
                    tris.Add(ti); tris.Add(tj); tris.Add(bi);
                    tris.Add(tj); tris.Add(bj); tris.Add(bi);
                }
            }

            var mesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            mesh.SetVertices(verts);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateBounds(); // 必须：否则视锥剔除误杀程序化网格
            mesh.name = "IntersectionMesh";
            return mesh;
        }
    }
}
