using System;

namespace RoadSystem.Core
{
    public enum LaneType { Car, Sidewalk, Bike, Median, Parking }

    /// <summary>Profile 上一个车道的定义（纯数据，可序列化）。</summary>
    [Serializable]
    public struct LaneDef : IEquatable<LaneDef>
    {
        public LaneType Type;
        public float Width;      // 米
        public bool Forward;     // 行车方向是否沿 profile 法向正方向（Car 有效）
        public float CurbHeight; // 人行道抬高量（仅 Sidewalk 用，如 0.15m）

        public LaneDef(LaneType type, float width, bool forward = true, float curbHeight = 0f)
        {
            Type = type;
            Width = width;
            Forward = forward;
            CurbHeight = curbHeight;
        }

        public bool Equals(LaneDef other)
            => Type == other.Type && Width.Equals(other.Width)
               && Forward == other.Forward && CurbHeight.Equals(other.CurbHeight);
        public override bool Equals(object obj) => obj is LaneDef d && Equals(d);
        public override int GetHashCode()
            => ((int)Type * 397) ^ Width.GetHashCode() ^ (Forward ? 1 : 0) ^ CurbHeight.GetHashCode();
    }
}
