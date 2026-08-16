using System.Collections.Generic;
using RoadSystem.Core;
using RoadSystem.Geometry;
using UnityEngine;
using UnityEngine.Rendering;

namespace RoadSystem.Meshing
{
    /// <summary>
    /// 路段网格构建：沿 PathChain 采样，每个采样点沿法向按 LaneDef 宽度横向偏移布点。
    /// 圆弧偏移仍是圆弧、直线偏移仍是直线 —— 无 pinch/自交（选圆弧表示的根本原因）。
    ///
    /// 顶点布局：每条车道一条环带；人行道额外生成路缘竖面与外缘立面（独立顶点保证硬边光影）。
    /// UV：u = 横向归一化（0=左缘, 1=右缘，对齐车道标线 atlas），v = 弧长 / 贴图周期。
    /// </summary>
    public static class SegmentMeshBuilder
    {
        public static Mesh Build(PathChain path, LaneDef[] lanes, DVec2 origin,
            float sagitta = 0.05f, float vPeriod = 6f)
        {
            var samples = PathSampler.Sample(path, sagitta);
            return BuildFromSamples(samples, lanes, origin, vPeriod);
        }

        public static Mesh BuildFromSamples(List<PathPoint> samples, LaneDef[] lanes, DVec2 origin, float vPeriod)
        {
            var verts = new List<Vector3>();
            var normals = new List<Vector3>();
            var uvs = new List<Vector2>();
            var tris = new List<int>();

            if (samples == null || samples.Count < 2 || lanes == null || lanes.Length == 0)
                return EmptyMesh(verts, normals, uvs, tris);

            double W = 0;
            foreach (var l in lanes) W += l.Width;

            // 各车道边界横向坐标（0=左缘）
            var b = new double[lanes.Length + 1];
            for (int i = 0; i < lanes.Length; i++) b[i + 1] = b[i] + lanes[i].Width;

            Vector3 ToWorld(DVec2 p, double y)
                => new Vector3((float)(p.X - origin.X), (float)y, (float)(p.Y - origin.Y));

            // 每个采样点的左向单位向量缓存
            var lefts = new DVec2[samples.Count];
            for (int i = 0; i < samples.Count; i++) lefts[i] = samples[i].Tangent.PerpLeft;

            DVec2 Lateral(int i, double t) // 采样 i 处横向坐标 t 的点
                => samples[i].Position + lefts[i] * (W * 0.5 - t);

            // ---------- 1. 车道顶面 ----------
            for (int lane = 0; lane < lanes.Length; lane++)
            {
                double y = lanes[lane].Type == LaneType.Sidewalk ? lanes[lane].CurbHeight : 0.0;
                int baseIdx = verts.Count;
                for (int i = 0; i < samples.Count; i++)
                {
                    float v = (float)(samples[i].ArcLength / vPeriod);
                    verts.Add(ToWorld(Lateral(i, b[lane]), y));
                    verts.Add(ToWorld(Lateral(i, b[lane + 1]), y));
                    normals.Add(Vector3.up);
                    normals.Add(Vector3.up);
                    uvs.Add(new Vector2((float)(b[lane] / W), v));
                    uvs.Add(new Vector2((float)(b[lane + 1] / W), v));
                }
                for (int i = 0; i + 1 < samples.Count; i++)
                {
                    int k = baseIdx + i * 2;
                    // v0=近左 v1=近右 v2=远左 v3=远右（上行排列）
                    tris.Add(k); tris.Add(k + 2); tris.Add(k + 1);
                    tris.Add(k + 1); tris.Add(k + 2); tris.Add(k + 3);
                }
            }

            // ---------- 2. 路缘竖面（人行道与车行道的交界） ----------
            for (int j = 1; j < lanes.Length; j++)
            {
                bool leftWalk = lanes[j - 1].Type == LaneType.Sidewalk;
                bool rightWalk = lanes[j].Type == LaneType.Sidewalk;
                if (leftWalk == rightWalk) continue;

                var walk = leftWalk ? lanes[j - 1] : lanes[j];
                double h = walk.CurbHeight;
                if (h <= 1e-4) continue;

                double t = b[j];
                float u = (float)(t / W);
                for (int i = 0; i + 1 < samples.Count; i++)
                {
                    // 法线朝车行道一侧：人行道在左（t 较小）→ 车行道在右（t 较大）
                    // 右方向 = -left
                    DVec2 nDir = leftWalk ? -lefts[i] : lefts[i];
                    var n = new Vector3((float)nDir.X, 0, (float)nDir.Y);
                    float v0 = (float)(samples[i].ArcLength / vPeriod);
                    float v1 = (float)(samples[i + 1].ArcLength / vPeriod);
                    Vector3 b0 = ToWorld(Lateral(i, t), 0);
                    Vector3 t0v = ToWorld(Lateral(i, t), h);
                    Vector3 b1 = ToWorld(Lateral(i + 1, t), 0);
                    Vector3 t1v = ToWorld(Lateral(i + 1, t), h);
                    AddQuad(verts, normals, uvs, tris, b0, t0v, b1, t1v, n, u, v0, v1);
                }
            }

            // ---------- 3. 外缘立面（道路两侧边缘，从顶面落到 y=0，产生厚度感） ----------
            for (int side = 0; side < 2; side++)
            {
                int laneIdx = side == 0 ? 0 : lanes.Length - 1;
                double topY = lanes[laneIdx].Type == LaneType.Sidewalk ? lanes[laneIdx].CurbHeight : 0.0;
                if (topY <= 1e-4) continue;
                double t = side == 0 ? 0.0 : W;
                float u = (float)(t / W);
                for (int i = 0; i + 1 < samples.Count; i++)
                {
                    // 外侧法线：左侧边缘朝左（+left），右侧边缘朝右（-left）
                    DVec2 nDir = side == 0 ? lefts[i] : -lefts[i];
                    var n = new Vector3((float)nDir.X, 0, (float)nDir.Y);
                    float v0 = (float)(samples[i].ArcLength / vPeriod);
                    float v1 = (float)(samples[i + 1].ArcLength / vPeriod);
                    Vector3 t0v = ToWorld(Lateral(i, t), topY);
                    Vector3 b0 = ToWorld(Lateral(i, t), 0);
                    Vector3 t1v = ToWorld(Lateral(i + 1, t), topY);
                    Vector3 b1 = ToWorld(Lateral(i + 1, t), 0);
                    AddQuad(verts, normals, uvs, tris, t0v, b0, t1v, b1, n, u, v0, v1);
                }
            }

            return EmptyMesh(verts, normals, uvs, tris);
        }

        /// <summary>添加竖直面 quad（p0/p1 为近采样点下上，p2/p3 为远采样点下上），并按目标法线校正绕序。</summary>
        static void AddQuad(List<Vector3> verts, List<Vector3> normals, List<Vector2> uvs, List<int> tris,
            Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Vector3 normal, float u, float v0, float v1)
        {
            int k = verts.Count;
            verts.Add(p0); verts.Add(p1); verts.Add(p2); verts.Add(p3);
            for (int i = 0; i < 4; i++) normals.Add(normal);
            uvs.Add(new Vector2(u, v0)); uvs.Add(new Vector2(u, v0));
            uvs.Add(new Vector2(u, v1)); uvs.Add(new Vector2(u, v1));

            // 默认绕序，若与目标法线反向则翻转
            Vector3 fn = Vector3.Cross(p2 - p0, p1 - p0);
            if (Vector3.Dot(fn, normal) >= 0)
            {
                tris.Add(k); tris.Add(k + 2); tris.Add(k + 1);
                tris.Add(k + 1); tris.Add(k + 2); tris.Add(k + 3);
            }
            else
            {
                tris.Add(k); tris.Add(k + 1); tris.Add(k + 2);
                tris.Add(k + 1); tris.Add(k + 3); tris.Add(k + 2);
            }
        }

        static Mesh EmptyMesh(List<Vector3> verts, List<Vector3> normals, List<Vector2> uvs, List<int> tris)
        {
            var mesh = new Mesh { indexFormat = IndexFormat.UInt32 };
            mesh.SetVertices(verts);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateBounds(); // 必须：否则视锥剔除误杀程序化网格
            return mesh;
        }
    }
}
