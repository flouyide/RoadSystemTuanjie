using UnityEngine;

namespace RoadSystem
{
    /// <summary>Explicit binding for generated colliders. Never infer graph IDs from object names.</summary>
    [DisallowMultipleComponent]
    public sealed class RoadNodeView : MonoBehaviour
    {
        public RoadNetworkBehaviour Owner { get; private set; }
        public string NodeId { get; private set; }
        public void Bind(RoadNetworkBehaviour owner, string nodeId) { Owner = owner; NodeId = nodeId; }
    }
}
