using System;
using System.Collections.Generic;

namespace RoadSystem.Core
{
    /// <summary>
    /// 序列化 DTO：纯 POCO，由引擎层用 JsonUtility 持久化。
    /// Dictionary 不可被 JsonUtility 序列化，故全部转 List。
    /// </summary>
    [Serializable]
    public class ProfileDto
    {
        public string Id;
        public DVec2 Position;
        public DVec2 Direction;
        public LaneDef[] Lanes;
        public float Y;
        public string NodeAId;
        public string NodeBId;
    }

    [Serializable]
    public class NodeDto
    {
        public string Id;
        public string Kind; // "segment" | "intersection"
        public List<string> PortIds = new List<string>();
        public List<LaneConnection> LaneLinks = new List<LaneConnection>(); // 仅 intersection
    }

    [Serializable]
    public class RoadGraphDto
    {
        public int Version = 1;
        public List<ProfileDto> Profiles = new List<ProfileDto>();
        public List<NodeDto> Nodes = new List<NodeDto>();
    }

    public static class RoadGraphMapper
    {
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
                    Lanes = p.Lanes,
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
                    nd.LaneLinks = new List<LaneConnection>(ix.LaneLinks);
                }
                else nd.Kind = "segment";
                dto.Nodes.Add(nd);
            }
            return dto;
        }

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
                    Lanes = pd.Lanes,
                    Y = pd.Y,
                    NodeAId = pd.NodeAId,
                    NodeBId = pd.NodeBId
                };
                g.Profiles[p.Id] = p;
            }
            foreach (var nd in dto.Nodes)
            {
                RoadNodeBase n = nd.Kind == "intersection"
                    ? (RoadNodeBase)new Intersection { LaneLinks = nd.LaneLinks ?? new List<LaneConnection>() }
                    : new RoadSegment();
                n.Id = nd.Id;
                n.PortIds = nd.PortIds ?? new List<string>();
                g.Nodes[n.Id] = n;
            }
            return g;
        }
    }
}
