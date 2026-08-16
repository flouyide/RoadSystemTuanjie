using System;
using System.Collections.Generic;

namespace RoadSystem.Core
{
    /// <summary>
    /// 路网存储：邻接表。物理实体（RoadSegment / Intersection）为节点，Profile 为边。
    /// 所有编辑产生受影响集合，通过 Changed 事件通知上层做增量重建。
    /// </summary>
    public sealed class RoadGraph
    {
        public readonly Dictionary<string, RoadNodeBase> Nodes = new Dictionary<string, RoadNodeBase>();
        public readonly Dictionary<string, Profile> Profiles = new Dictionary<string, Profile>();

        /// <summary>受影响节点 Id 集合（路段/路口）。</summary>
        public event Action<IReadOnlyList<string>> Changed;

        // ---------------- Profile ----------------

        public Profile CreateProfile(DVec2 pos, DVec2 dir, LaneDef[] lanes = null)
        {
            var p = Profile.Create(pos, dir, lanes);
            Profiles.Add(p.Id, p);
            return p;
        }

        public void MoveProfile(Profile p, DVec2 newPos, DVec2 newDir)
        {
            p.Position = newPos;
            p.Direction = newDir.Normalized;
            NotifyAffected(AffectedByProfile(p));
        }

        // ---------------- Segment ----------------

        /// <summary>
        /// 用两个 profile 创建路段。MVP 要求两端断面一致（等长 profile）。
        /// 成功返回 RoadSegment；断面不一致返回 null。
        /// </summary>
        public RoadSegment AddSegment(Profile a, Profile b)
        {
            if (a == null || b == null || a == b) return null;
            if (!a.HasSameLayout(b)) return null;

            var seg = new RoadSegment();
            seg.PortIds.Add(a.Id);
            seg.PortIds.Add(b.Id);
            Nodes.Add(seg.Id, seg);
            Attach(a, seg.Id);
            Attach(b, seg.Id);
            NotifyAffected(new[] { seg.Id });
            return seg;
        }

        public void RemoveSegment(string segId)
        {
            if (!Nodes.TryGetValue(segId, out var node)) return;
            var affected = new List<string>();
            foreach (var pid in node.PortIds)
            {
                if (Profiles.TryGetValue(pid, out var p))
                {
                    Detach(p, segId);
                    // 端口另一端实体受影响（共享 profile 的相邻段需重建）
                    var affectedSet = AffectedByProfile(p);
                    foreach (var id in affectedSet)
                        if (id != segId && !affected.Contains(id)) affected.Add(id);
                }
            }
            Nodes.Remove(segId);
            if (affected.Count > 0) NotifyAffected(affected);
        }

        /// <summary>删除一个 profile 及其唯一关联的悬空段（编辑器取消放置用）。</summary>
        public void RemoveLooseProfile(Profile p)
        {
            if (p == null) return;
            if (!string.IsNullOrEmpty(p.NodeAId) && Nodes.ContainsKey(p.NodeAId)) return; // 非悬空
            if (!string.IsNullOrEmpty(p.NodeBId) && Nodes.ContainsKey(p.NodeBId)) return;
            Profiles.Remove(p.Id);
        }

        // ---------------- 查询 ----------------

        public IEnumerable<RoadSegment> Segments()
        {
            foreach (var n in Nodes.Values)
                if (n is RoadSegment s) yield return s;
        }

        public Profile GetProfile(string id)
            => id != null && Profiles.TryGetValue(id, out var p) ? p : null;

        public RoadNodeBase GetNode(string id)
            => id != null && Nodes.TryGetValue(id, out var n) ? n : null;

        /// <summary>所有悬空端 profile（只挂了一个实体），供吸附/续接使用。</summary>
        public List<Profile> LooseEndProfiles()
        {
            var list = new List<Profile>();
            foreach (var p in Profiles.Values)
                if (p.IsLooseEnd) list.Add(p);
            return list;
        }

        // ---------------- 内部 ----------------

        void Attach(Profile p, string nodeId)
        {
            if (string.IsNullOrEmpty(p.NodeAId)) p.NodeAId = nodeId;
            else if (p.NodeAId != nodeId && string.IsNullOrEmpty(p.NodeBId)) p.NodeBId = nodeId;
        }

        void Detach(Profile p, string nodeId)
        {
            if (p.NodeAId == nodeId) p.NodeAId = null;
            else if (p.NodeBId == nodeId) p.NodeBId = null;
        }

        /// <summary>受影响集合：profile 两端节点 + 相邻节点（共享 profile 链）。</summary>
        List<string> AffectedByProfile(Profile p)
        {
            var set = new List<string>();
            void AddNode(string nid)
            {
                if (string.IsNullOrEmpty(nid) || set.Contains(nid) || !Nodes.ContainsKey(nid)) return;
                set.Add(nid);
                // 路口任一 port 变动整口重建：把该节点其余 port 的对端节点也纳入
                var node = Nodes[nid];
                if (node is Intersection)
                {
                    foreach (var otherPid in node.PortIds)
                    {
                        var op = GetProfile(otherPid);
                        if (op == null) continue;
                        if (!string.IsNullOrEmpty(op.NodeAId) && op.NodeAId != nid && !set.Contains(op.NodeAId))
                            set.Add(op.NodeAId);
                        if (!string.IsNullOrEmpty(op.NodeBId) && op.NodeBId != nid && !set.Contains(op.NodeBId))
                            set.Add(op.NodeBId);
                    }
                }
            }
            AddNode(p.NodeAId);
            AddNode(p.NodeBId);
            return set;
        }

        void NotifyAffected(IReadOnlyList<string> ids)
        {
            if (ids == null || ids.Count == 0) return;
            foreach (var id in ids)
            {
                var n = GetNode(id);
                if (n != null) n.DirtyVersion++;
            }
            Changed?.Invoke(ids);
        }

        public void Clear()
        {
            Nodes.Clear();
            Profiles.Clear();
        }
    }
}
