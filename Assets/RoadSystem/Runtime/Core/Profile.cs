using System;

namespace RoadSystem.Core
{
    /// <summary>
    /// 图的边：0 宽度横截面线。不渲染，只存数据。
    /// Lanes 从左缘到右缘有序排列（相对 Direction 的左手侧起）。
    /// </summary>
    [Serializable]
    public class Profile
    {
        public string Id;
        public DVec2 Position;  // 中心点（世界 XZ）
        public DVec2 Direction; // 单位向量，指向道路前进方向（垂直于 profile 线）
        public LaneDef[] Lanes;

        /// <summary>两端所属的实体节点 Id（segment / intersection），可为空表示悬空端。</summary>
        public string NodeAId;
        public string NodeBId;

        public double TotalWidth
        {
            get
            {
                double w = 0;
                if (Lanes != null)
                    foreach (var l in Lanes) w += l.Width;
                return w;
            }
        }

        public DVec2 Left => Direction.PerpLeft;
        public DVec2 LeftEdge => Position + Left * (TotalWidth * 0.5);
        public DVec2 RightEdge => Position - Left * (TotalWidth * 0.5);

        /// <summary>横向坐标 t（0=左缘，W=右缘）处的世界点。</summary>
        public DVec2 LateralPoint(double t) => Position + Left * (TotalWidth * 0.5 - t);

        /// <summary>各车道边界的横向坐标（含 0 与 W），长度 = Lanes.Length + 1。</summary>
        public double[] BoundaryOffsets()
        {
            int n = Lanes?.Length ?? 0;
            var b = new double[n + 1];
            for (int i = 0; i < n; i++) b[i + 1] = b[i] + Lanes[i].Width;
            return b;
        }

        /// <summary>该 profile 是否为悬空端（只挂了一个实体节点）。</summary>
        public bool IsLooseEnd => string.IsNullOrEmpty(NodeAId) || string.IsNullOrEmpty(NodeBId);

        public static Profile Create(DVec2 pos, DVec2 dir, LaneDef[] lanes)
        {
            return new Profile
            {
                Id = Guid.NewGuid().ToString("N"),
                Position = pos,
                Direction = dir.Normalized,
                Lanes = lanes ?? DefaultLanes()
            };
        }

        /// <summary>MVP 默认断面：人行道 2m + 车道 3.5m + 车道 3.5m + 人行道 2m。</summary>
        public static LaneDef[] DefaultLanes()
        {
            return new[]
            {
                new LaneDef(LaneType.Sidewalk, 2.0f, true, 0.15f),
                new LaneDef(LaneType.Car, 3.5f, true),
                new LaneDef(LaneType.Car, 3.5f, false),
                new LaneDef(LaneType.Sidewalk, 2.0f, true, 0.15f),
            };
        }

        /// <summary>等长判定：MVP 要求相连 profile 断面完全一致（车道数与宽度序列相同）。</summary>
        public bool HasSameLayout(Profile other)
        {
            if (other == null || Lanes == null || other.Lanes == null) return false;
            if (Lanes.Length != other.Lanes.Length) return false;
            for (int i = 0; i < Lanes.Length; i++)
            {
                if (Lanes[i].Type != other.Lanes[i].Type) return false;
                if (Math.Abs(Lanes[i].Width - other.Lanes[i].Width) > 1e-4) return false;
            }
            return true;
        }
    }
}
