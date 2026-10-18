using System;
using System.Collections.Generic;
using RoadSystem.Core;

namespace RoadSystem.Geometry
{
    /// <summary>Converts persisted line/arc elements and resolves segment geometry.</summary>
    public static class SegmentPathResolver
    {
        public static List<PathElementDto> Store(PathChain path)
        {
            if (path == null) return null;
            var result = new List<PathElementDto>();
            foreach (var element in path.Elements)
            {
                if (element is LineElement line)
                    result.Add(new PathElementDto { Kind = "line", A = line.A, B = line.B });
                else if (element is ArcElement arc)
                    result.Add(new PathElementDto { Kind = "arc", Center = arc.Center,
                        Radius = arc.Radius, StartAngle = arc.StartAngle, Sweep = arc.Sweep });
            }
            return result;
        }

        public static PathChain Restore(List<PathElementDto> elements)
        {
            if (elements == null || elements.Count == 0) return null;
            var path = new PathChain();
            foreach (var element in elements)
            {
                if (element == null) return null;
                if (element.Kind == "line") path.Add(new LineElement(element.A, element.B));
                else if (element.Kind == "arc" && element.Radius > 0
                    && !double.IsNaN(element.Radius) && !double.IsInfinity(element.Radius))
                    path.Add(new ArcElement(element.Center, element.Radius,
                        element.StartAngle, element.Sweep));
                else return null;
            }
            return path.Elements.Count > 0 ? path : null;
        }

        public static PathChain Resolve(RoadGraph graph, RoadSegment segment)
        {
            if (segment == null || segment.PortIds.Count != 2) return null;
            if (segment.Path != null) return segment.Path;
            if (segment.ExplicitPath != null && segment.ExplicitPath.Count > 0)
                return segment.Path = Restore(segment.ExplicitPath);
            var start = graph.GetProfile(segment.PortIds[0]);
            var end = graph.GetProfile(segment.PortIds[1]);
            return segment.Path = start != null && end != null
                ? ProfileConnector.Connect(start, end) : null;
        }
    }
}
