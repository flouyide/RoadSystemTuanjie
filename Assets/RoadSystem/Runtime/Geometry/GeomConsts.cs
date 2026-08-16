using System;

namespace RoadSystem.Geometry
{
    public static class GeomConsts
    {
        /// <summary>平行判定：两方向夹角 &lt; 1e-4 rad 视为平行（plan §8.1 容差）。</summary>
        public const double ParallelEps = 1e-4;
        /// <summary>通用距离容差（米）。</summary>
        public const double DistEps = 1e-6;
        /// <summary>fillet 半径小于该值时退化为直线（两延续线近乎共线）。</summary>
        public const double MinRadius = 0.05;
        /// <summary>交点允许在射线起点后方的微小容差。</summary>
        public const double RayTol = 1e-6;
        /// <summary>Hermite 中间 profile 切向模长系数：中心距的 1.0~1.5 倍，取中间值。</summary>
        public const double HermiteTangentScale = 1.25;
        /// <summary>ProfileConnector 递归深度上限（plan §3 步骤 6）。</summary>
        public const int MaxDepth = 2;
        /// <summary>默认采样弦高误差（米）。</summary>
        public const double DefaultSagitta = 0.05;
        /// <summary>直线段最大采样步长（米）。</summary>
        public const double MaxLineStep = 2.0;

        public static double Clamp(double v, double min, double max)
            => v < min ? min : (v > max ? max : v);
    }
}
