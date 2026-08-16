using System;
using System.Collections.Generic;

namespace RoadSystem.Core
{
    /// <summary>
    /// 路网图的节点：物理实体（路段 / 路口）。边是 0 宽度的 Profile。
    /// </summary>
    [Serializable]
    public abstract class RoadNodeBase
    {
        public string Id;
        /// <summary>入射 profile Id 集合（端口）。</summary>
        public List<string> PortIds = new List<string>();
        /// <summary>脏版本号：预览期帧合并用（同一帧多次变动只重建一次）。</summary>
        [NonSerialized] public int DirtyVersion;

        protected RoadNodeBase()
        {
            Id = Guid.NewGuid().ToString("N");
        }
    }

    /// <summary>路段：恰好 2 个端口（起点/终点 profile）。</summary>
    public sealed class RoadSegment : RoadNodeBase
    {
        /// <summary>由 L1 解算出的 直线+圆弧 序列（缓存，可被标记脏）。不序列化。</summary>
        [NonSerialized] public PathChain Path;

        /// <summary>路径解算失败标记（编辑器层用于阻止非法放置的兜底显示）。</summary>
        [NonSerialized] public bool PathSolveFailed;
    }

    /// <summary>
    /// 路口：N 个端口（N>=3；MVP 支持 3 叉 T 型 / 4 叉十字）。
    /// </summary>
    public sealed class Intersection : RoadNodeBase
    {
        /// <summary>显式车道级连接（意图显式存储，不靠推断）。</summary>
        public List<LaneConnection> LaneLinks = new List<LaneConnection>();
    }

    /// <summary>路口内一条转向车道连接。</summary>
    [Serializable]
    public class LaneConnection
    {
        public string FromPortId;
        public string ToPortId;
        public int FromLane; // Lanes 数组下标
        public int ToLane;
        [NonSerialized] public PathChain Path; // 路口内该转向的 弧+线 路径（缓存）
    }
}
