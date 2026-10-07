using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    /// <summary>Immutable selection snapshot installed while tracking is stopped.</summary>
    public sealed class TrackingConfiguration
    {
        public IReadOnlyList<MarkerGeometryDefinition> Geometries { get; }
        public IReadOnlyList<TrackedToolDefinition> Tools { get; }
        private readonly Dictionary<string, MarkerGeometryDefinition> geometries = new Dictionary<string, MarkerGeometryDefinition>();

        public TrackingConfiguration(IEnumerable<MarkerGeometryDefinition> geometries, IEnumerable<TrackedToolDefinition> tools)
        {
            var gs = new List<MarkerGeometryDefinition>(geometries ?? throw new ArgumentNullException(nameof(geometries)));
            var ts = new List<TrackedToolDefinition>(tools ?? throw new ArgumentNullException(nameof(tools)));
            if (ts.Count == 0) throw new ArgumentException("Select at least one tool.");
            foreach (var geometry in gs)
            {
                if (geometry == null || this.geometries.ContainsKey(geometry.Id)) throw new ArgumentException("Unique geometry IDs required.");
                this.geometries.Add(geometry.Id, geometry);
            }
            var toolIds = new HashSet<string>();
            var providerGeometry = new HashSet<(string, string)>();
            var providerBodies = new HashSet<(string, string)>();
            foreach (var tool in ts)
            {
                if (tool == null || !toolIds.Add(tool.ObjectId)) throw new ArgumentException("Unique tool instance IDs required.");
                foreach (var binding in tool.Bindings)
                {
                    if (!this.geometries.ContainsKey(binding.GeometryId)) throw new ArgumentException("Unknown geometry: " + binding.GeometryId);
                    if (!providerGeometry.Add((binding.Tracking.ProviderId, binding.GeometryId)))
                        throw new ArgumentException("Two tools cannot use the same geometry on one provider without an additional identity mechanism.");
                    if (!providerBodies.Add((binding.Tracking.ProviderId, binding.Tracking.ProviderObjectId)))
                        throw new ArgumentException("Provider body ID assigned to more than one tool.");
                }
            }
            Geometries = gs.AsReadOnly(); Tools = ts.AsReadOnly();
        }

        public MarkerGeometryDefinition GetGeometry(string id) => geometries[id];
        public ProviderGeometryConfiguration ForProvider(string id)
        {
            var entries = new List<ProviderGeometryBinding>();
            foreach (var tool in Tools)
                foreach (var binding in tool.Bindings)
                    if (binding.Tracking.ProviderId == id)
                        entries.Add(new ProviderGeometryBinding(tool, binding, GetGeometry(binding.GeometryId)));
            return new ProviderGeometryConfiguration(id, entries);
        }
    }

    public sealed class ProviderGeometryBinding
    {
        public TrackedToolDefinition Tool { get; }
        public ToolGeometryBinding Binding { get; }
        public MarkerGeometryDefinition Geometry { get; }
        internal ProviderGeometryBinding(TrackedToolDefinition tool, ToolGeometryBinding binding, MarkerGeometryDefinition geometry)
        { Tool = tool; Binding = binding; Geometry = geometry; }
    }

    public sealed class ProviderGeometryConfiguration
    {
        public string ProviderId { get; }
        public IReadOnlyList<ProviderGeometryBinding> Tools { get; }
        public IReadOnlyList<MarkerGeometryDefinition> Geometries { get; }
        internal ProviderGeometryConfiguration(string providerId, List<ProviderGeometryBinding> tools)
        {
            ProviderId = providerId; Tools = tools.AsReadOnly();
            var geometries = new List<MarkerGeometryDefinition>();
            foreach (var tool in tools) geometries.Add(tool.Geometry);
            Geometries = geometries.AsReadOnly();
        }
    }
}
