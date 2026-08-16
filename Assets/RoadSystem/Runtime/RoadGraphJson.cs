using RoadSystem.Core;
using UnityEngine;

namespace RoadSystem
{
    /// <summary>RoadGraph &lt;-&gt; JSON（JsonUtility）。DTO 转换在 Core 层，保持 L0 无引擎依赖。</summary>
    public static class RoadGraphJson
    {
        public static string ToJson(RoadGraph g)
        {
            var dto = RoadGraphMapper.ToDto(g);
            return JsonUtility.ToJson(dto);
        }

        public static RoadGraph FromJson(string json)
        {
            if (string.IsNullOrEmpty(json)) return new RoadGraph();
            var dto = JsonUtility.FromJson<RoadGraphDto>(json);
            return RoadGraphMapper.FromDto(dto);
        }
    }
}
