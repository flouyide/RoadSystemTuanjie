using System;

namespace RoadSystem.Core
{
    /// <summary>
    /// 双精度 2D 向量，对应世界 XZ 平面（X->X, Y->Z）。
    /// L0/L1 内部统一使用 double，避免城市尺度（km 级）下 float 求交抖动。
    /// </summary>
    [Serializable]
    public struct DVec2 : IEquatable<DVec2>
    {
        public double X;
        public double Y;

        public DVec2(double x, double y) { X = x; Y = y; }

        public static readonly DVec2 Zero = new DVec2(0, 0);
        public static readonly DVec2 UnitX = new DVec2(1, 0);
        public static readonly DVec2 UnitY = new DVec2(0, 1);

        public double Length => Math.Sqrt(X * X + Y * Y);
        public double LengthSquared => X * X + Y * Y;

        public DVec2 Normalized
        {
            get
            {
                double len = Length;
                return len > 1e-12 ? new DVec2(X / len, Y / len) : UnitX;
            }
        }

        /// <summary>逆时针旋转 90°（世界 +Y 视角下的左侧）。</summary>
        public DVec2 PerpLeft => new DVec2(-Y, X);
        /// <summary>顺时针旋转 90°（世界 +Y 视角下的右侧）。</summary>
        public DVec2 PerpRight => new DVec2(Y, -X);

        public static double Dot(DVec2 a, DVec2 b) => a.X * b.X + a.Y * b.Y;
        /// <summary>2D 叉积（z 分量）：a x b = a.X*b.Y - a.Y*b.X。</summary>
        public static double Cross(DVec2 a, DVec2 b) => a.X * b.Y - a.Y * b.X;
        public static double Distance(DVec2 a, DVec2 b) => (a - b).Length;
        public static DVec2 Lerp(DVec2 a, DVec2 b, double t) => a + (b - a) * t;

        /// <summary>把向量按长度限制到 maxLen。</summary>
        public DVec2 ClampMagnitude(double maxLen)
        {
            double len = Length;
            return len > maxLen && len > 1e-12 ? this * (maxLen / len) : this;
        }

        public bool ApproxEquals(DVec2 other, double eps = 1e-6)
            => Math.Abs(X - other.X) < eps && Math.Abs(Y - other.Y) < eps;

        public bool Equals(DVec2 other) => X.Equals(other.X) && Y.Equals(other.Y);
        public override bool Equals(object obj) => obj is DVec2 v && Equals(v);
        public override int GetHashCode() => X.GetHashCode() * 397 ^ Y.GetHashCode();
        public override string ToString() => $"({X:F3}, {Y:F3})";

        public static DVec2 operator +(DVec2 a, DVec2 b) => new DVec2(a.X + b.X, a.Y + b.Y);
        public static DVec2 operator -(DVec2 a, DVec2 b) => new DVec2(a.X - b.X, a.Y - b.Y);
        public static DVec2 operator -(DVec2 a) => new DVec2(-a.X, -a.Y);
        public static DVec2 operator *(DVec2 a, double s) => new DVec2(a.X * s, a.Y * s);
        public static DVec2 operator *(double s, DVec2 a) => new DVec2(a.X * s, a.Y * s);
        public static DVec2 operator /(DVec2 a, double s) => new DVec2(a.X / s, a.Y / s);
        public static bool operator ==(DVec2 a, DVec2 b) => a.Equals(b);
        public static bool operator !=(DVec2 a, DVec2 b) => !a.Equals(b);
    }
}
