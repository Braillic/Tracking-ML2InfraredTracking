using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    /// <summary>Immutable marker centres in canonical body-local metres. No instrument tip or UI policy.</summary>
    public sealed class MarkerGeometryDefinition
    {
        public string Id { get; }
        public string Revision { get; }
        public IReadOnlyList<Vector3d> Markers { get; }

        public MarkerGeometryDefinition(string id, string revision, IEnumerable<Vector3d> markers)
        {
            if (string.IsNullOrWhiteSpace(id) || string.IsNullOrWhiteSpace(revision))
                throw new ArgumentException("Geometry ID and revision are required.");
            var points = new List<Vector3d>(markers ?? throw new ArgumentNullException(nameof(markers)));
            if (points.Count < 3) throw new ArgumentException("A rigid marker geometry needs at least three points.");
            for (int i = 0; i < points.Count; i++)
            {
                if (!points[i].IsFinite) throw new ArgumentException("Marker coordinates must be finite.");
                for (int j = 0; j < i; j++)
                    if (points[i].X == points[j].X && points[i].Y == points[j].Y && points[i].Z == points[j].Z)
                        throw new ArgumentException("Duplicate marker centres.");
            }
            bool nonCollinear = false;
            for (int i = 2; i < points.Count; i++)
            {
                var cross = Vector3d.Cross(points[1] - points[0], points[i] - points[0]);
                if (cross.X != 0 || cross.Y != 0 || cross.Z != 0) nonCollinear = true;
            }
            if (!nonCollinear) throw new ArgumentException("Marker centres must not all be collinear.");
            Id = id; Revision = revision; Markers = points.AsReadOnly();
        }
    }
}
