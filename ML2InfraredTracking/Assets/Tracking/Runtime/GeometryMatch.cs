using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    /// <summary>One algorithm-produced candidate, not yet assigned to a tool. Detection indices
    /// refer to a single shared input frame, allowing overlapping marker assignments to be rejected.</summary>
    public sealed class GeometryMatch
    {
        public string GeometryId { get; }
        public string GeometryRevision { get; }
        public RigidPose ReferenceFromBody { get; }
        public double? Quality { get; }
        public IReadOnlyList<int> DetectionIndices { get; }
        public GeometryMatch(string geometryId, string geometryRevision, RigidPose referenceFromBody,
            IEnumerable<int> detectionIndices, double? quality = null)
        {
            if (string.IsNullOrWhiteSpace(geometryId) || string.IsNullOrWhiteSpace(geometryRevision) || !referenceFromBody.IsValid)
                throw new ArgumentException("Valid geometry identity and pose required.");
            if (quality.HasValue && (!Numeric.Finite(quality.Value) || quality.Value < 0 || quality.Value > 1))
                throw new ArgumentException("Quality must be in [0,1].");
            var indices = new List<int>(detectionIndices ?? throw new ArgumentNullException(nameof(detectionIndices)));
            if (indices.Count < 3) throw new ArgumentException("At least three observed markers required.");
            var seen = new HashSet<int>();
            foreach (int index in indices)
                if (index < 0 || !seen.Add(index)) throw new ArgumentException("Unique nonnegative detection indices required.");
            GeometryId = geometryId; GeometryRevision = geometryRevision; ReferenceFromBody = referenceFromBody.Normalized;
            Quality = quality; DetectionIndices = indices.AsReadOnly();
        }
    }
}
