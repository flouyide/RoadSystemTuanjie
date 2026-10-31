using System;
using System.Collections.Generic;

namespace RoadSystem.Core
{
    /// <summary>
    /// 序列化数据传输对象DTO：不依赖Unity框架的类，由引擎层用 JsonUtility 持久化。
    /// Dictionary 不可被 JsonUtility 序列化，故全部转 List，JSON 才能序列化
    /// </summary>

    /// <summary>单个路段端点（0 宽度端口）的序列化镜像。</summary>
    [Serializable]
    public class ProfileDto
    {
        /// <summary>Profile 唯一标识（Guid 字符串），跨序列化保持稳定以便 PortIds 引用解析。</summary>
        public string Id;
        /// <summary>端点水平位置（XZ 平面），路网几何在 2D 计算。</summary>
        public DVec2 Position;
        /// <summary>端点处的中心线条切向/出流方向，决定路段朝向与车道横截面。</summary>
        public DVec2 Direction;
        /// <summary>车道定义数组（车道数/宽度/偏移），路段两端须等长一致。</summary>
        public LaneDef[] Lanes;
        /// <summary>渲染高度 Y（贴地形高度 + 抬高量），仅用于 mesh 抬升防 Z-fighting。</summary>
        public float Y;
        /// <summary>关联节点 A（RoadSegment/Intersection）的 Id；空串表示尚未接入任何节点（悬空端）。</summary>
        public string NodeAId;
        /// <summary>关联节点 B 的 Id；一个 Profile 最多被两个节点共享（作为连续路段的两端），故用 A/B 两个槽位。</summary>
        public string NodeBId;
    }

    /// <summary>路网物理实体（路段或路口）的序列化镜像。</summary>
    [Serializable]
    public class PathElementDto
    {
        public string Kind; // "line" | "arc"
        public DVec2 A;
        public DVec2 B;
        public DVec2 Center;
        public double Radius;
        public double StartAngle;
        public double Sweep;
    }

    [Serializable]
    public class NodeDto
    {
        public bool HasAnchor;
        public DVec2 Anchor;
        public float AnchorY;
        /// <summary>节点唯一标识（Guid 字符串）。</summary>
        public string Id;
        /// <summary>节点类型标记："segment"（路段）或 "intersection"（路口），反序列化时据此重建具体类型。</summary>
        public string Kind; // "segment" | "intersection"
        /// <summary>端口（Profile）Id 列表：路段为 2 个，路口为 N（N>=3）个。</summary>
        public List<string> PortIds = new List<string>();
        /// <summary>所属道路分组 Id（仅 segment 使用；一次建造流程的若干路段共享同一 Id）。</summary>
        public string RoadId;
        /// <summary>Exact line/arc path for manually controlled or split segments.</summary>
        public List<PathElementDto> ExplicitPath;
        /// <summary>显式车道级转向连接（仅 intersection 使用；路段为空）。</summary>
        public List<LaneConnection> LaneLinks = new List<LaneConnection>(); // 仅 intersection
    }

    /// <summary>整张路网图的序列化根对象：把字典展平为两个 List 以便 JsonUtility 序列化。</summary>
    [Serializable]
    public class RoadGraphDto
    {
        /// <summary>版本 3 增加稳定路口锚点；兼容旧版显式路径及无锚点数据。</summary>
        public int Version = 3;
        /// <summary>所有 Profile（端点）的扁平列表，对应 RoadGraph.Profiles 字典。</summary>
        public List<ProfileDto> Profiles = new List<ProfileDto>();
        /// <summary>所有节点（路段/路口）的扁平列表，对应 RoadGraph.Nodes 字典。</summary>
        public List<NodeDto> Nodes = new List<NodeDto>();
    }

    /// <summary>映射器：在内存活图（RoadGraph，用 Dictionary 存储）与序列化 DTO（用 List 存储）之间互转。</summary>
    public static class RoadGraphMapper
    {
        /// <summary>
        /// 把活图 RoadGraph 转成可序列化的 RoadGraphDto（字典 → 列表）。
        /// 遍历 Profiles 与 Nodes，逐字段拷贝；节点按类型写 Kind 并搬运 LaneLinks。
        /// </summary>
        public static RoadGraphDto ToDto(RoadGraph g)
        {
            var dto = new RoadGraphDto();
            foreach (var p in g.Profiles.Values)
            {
                dto.Profiles.Add(new ProfileDto
                {
                    Id = p.Id,
                    Position = p.Position,
                    Direction = p.Direction,
                    Lanes = p.Lanes == null ? null : (LaneDef[])p.Lanes.Clone(),
                    Y = p.Y,
                    NodeAId = p.NodeAId,
                    NodeBId = p.NodeBId
                });
            }
            foreach (var n in g.Nodes.Values)
            {
                var nd = new NodeDto { Id = n.Id, PortIds = new List<string>(n.PortIds) };
                if (n is Intersection ix)
                {
                    nd.Kind = "intersection";
                    nd.LaneLinks = CopyLinks(ix.LaneLinks);
                    nd.HasAnchor = ix.HasAnchor;
                    nd.Anchor = ix.Anchor;
                    nd.AnchorY = ix.AnchorY;
                }
                else
                {
                    nd.Kind = "segment";
                    var segment = (RoadSegment)n;
                    nd.RoadId = segment.RoadId;
                    nd.ExplicitPath = CopyPath(segment.ExplicitPath);
                }
                dto.Nodes.Add(nd);
            }
            return dto;
        }

        /// <summary>
        /// 把 RoadGraphDto 还原成内存活图 RoadGraph（列表 → 字典）。
        /// 保留原始 Id 以确保 PortIds / NodeAId / NodeBId 引用仍可解析；
        /// 按 Kind 重建 RoadSegment 或 Intersection；[NonSerialized] 的 Path / DirtyVersion 不在此恢复（由宿主重建时重算）。
        /// </summary>
        public static RoadGraph FromDto(RoadGraphDto dto)
        {
            var g = new RoadGraph();
            if (dto == null) return g;
            foreach (var pd in dto.Profiles)
            {
                var p = new Profile
                {
                    Id = pd.Id,
                    Position = pd.Position,
                    Direction = pd.Direction,
                    Lanes = pd.Lanes == null ? null : (LaneDef[])pd.Lanes.Clone(),
                    Y = pd.Y,
                    NodeAId = pd.NodeAId,
                    NodeBId = pd.NodeBId
                };
                g.Profiles[p.Id] = p;
            }
            foreach (var nd in dto.Nodes)
            {
                RoadNodeBase n;
                if (nd.Kind == "intersection")
                {
                    n = new Intersection { LaneLinks = CopyLinks(nd.LaneLinks),
                        HasAnchor = nd.HasAnchor, Anchor = nd.Anchor, AnchorY = nd.AnchorY };
                }
                else
                {
                    var seg = new RoadSegment { RoadId = nd.RoadId, ExplicitPath = CopyPath(nd.ExplicitPath) };
                    n = seg;
                }
                n.Id = nd.Id;
                n.PortIds = nd.PortIds == null ? new List<string>() : new List<string>(nd.PortIds);
                g.Nodes[n.Id] = n;
            }
            return g;
        }

        static List<LaneConnection> CopyLinks(List<LaneConnection> source)
        {
            var result = new List<LaneConnection>();
            if (source != null) foreach (var l in source)
                result.Add(new LaneConnection { FromPortId = l.FromPortId, ToPortId = l.ToPortId,
                    FromLane = l.FromLane, ToLane = l.ToLane });
            return result;
        }

        static List<PathElementDto> CopyPath(List<PathElementDto> source)
        {
            if (source == null) return null;
            var result = new List<PathElementDto>();
            foreach (var e in source) result.Add(new PathElementDto { Kind = e.Kind,
                A = e.A, B = e.B, Center = e.Center, Radius = e.Radius,
                StartAngle = e.StartAngle, Sweep = e.Sweep });
            return result;
        }
    }
}
