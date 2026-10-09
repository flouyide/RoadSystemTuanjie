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
    /// 以路口中心为扇心 fan 三角化生成铺装顶面（星形域假设；转角内凹已在环构建时退化为直线），
    /// 并沿环边生成外缘立面（顶面落到 -curb）。
    ///
    /// fan 相比耳切的优势：所有三角形共享扇心、绕序绝对一致，不存在"部分三角朝上部分朝下"
    /// 被背面剔除露出地面的风险（截图 bug 的根因）。
    ///
    /// 顶点布局：顶面 = 扇心 1 个 + 环点 n 个；立面独立顶点 2n 个（硬边光影）。
    /// UV：世界对齐的 XZ / vPeriod 采样，密度与路段贴图周期（6m）一致。
    /// 注意：DVec2 的数学 CCW（XZ 平面 atan2 序）与 Unity 三角正面绕序相反——
    /// fan 三角统一用 (center, v_{i+1}, v_i) 使正面朝上。
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
                // ---------- 顶面顶点：index 0 = 扇心（路口中心），index 1..n = 环点 ----------
                verts.Add(new Vector3(0, 0, 0)); // 扇心（局部原点）
                normals.Add(Vector3.up);
                uvs.Add(Vector2.zero);
                for (int i = 0; i < n; i++)
                {
                    var p = loop[i];
                    verts.Add(new Vector3(
                        (float)(p.Pos.X - origin.X), (float)(p.Y - baseY), (float)(p.Pos.Y - origin.Y)));
                    normals.Add(Vector3.up);
                    uvs.Add(new Vector2(
                        (float)(p.Pos.X - origin.X) / vPeriod, (float)(p.Pos.Y - origin.Y) / vPeriod));
                }

                // ---------- 顶面 fan 三角：(center, v_{i+1}, v_i) —— 统一绕序、正面朝上 ----------
                for (int i = 0; i < n; i++)
                {
                    int a = 1 + i;               // v_i
                    int b = 1 + (i + 1) % n;     // v_{i+1}
                    tris.Add(0); tris.Add(b); tris.Add(a);
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
                    uvs.Add(uvs[1 + i]);
                }
                for (int i = 0; i < n; i++)
                {
                    normals.Add(Vector3.down);
                    uvs.Add(uvs[1 + i]);
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
