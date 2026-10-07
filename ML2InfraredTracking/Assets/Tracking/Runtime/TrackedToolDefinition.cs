using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public enum TrackedToolRole { Unspecified, Probe, DynamicReferenceFrame, Instrument }

    public sealed class ToolGeometryBinding
    {
        public ObjectBinding Tracking { get; }
        public string GeometryId { get; }
        public ToolGeometryBinding(ObjectBinding tracking, string geometryId)
        {
            Tracking = tracking ?? throw new ArgumentNullException(nameof(tracking));
            if (string.IsNullOrWhiteSpace(geometryId)) throw new ArgumentException("Geometry ID required.");
            GeometryId = geometryId;
        }
    }

    /// <summary>A UI-selected physical tool instance. Role is application metadata, never inferred from shape.</summary>
    public sealed class TrackedToolDefinition
    {
        public string ObjectId { get; }
        public string DisplayName { get; }
        public TrackedToolRole Role { get; }
        public IReadOnlyList<ToolGeometryBinding> Bindings { get; }

        public TrackedToolDefinition(string objectId, string displayName, TrackedToolRole role,
            params ToolGeometryBinding[] bindings)
        {
            if (string.IsNullOrWhiteSpace(objectId) || string.IsNullOrWhiteSpace(displayName) ||
                !Enum.IsDefined(typeof(TrackedToolRole), role)) throw new ArgumentException("Valid tool identity and role required.");
            if (bindings == null || bindings.Length == 0) throw new ArgumentException("Tool bindings required.");
            var providers = new HashSet<string>();
            foreach (var binding in bindings)
                if (binding == null || !providers.Add(binding.Tracking.ProviderId))
                    throw new ArgumentException("Each tool may bind a provider once.");
            ObjectId = objectId; DisplayName = displayName; Role = role;
            Bindings = Array.AsReadOnly((ToolGeometryBinding[])bindings.Clone());
        }
    }
}
