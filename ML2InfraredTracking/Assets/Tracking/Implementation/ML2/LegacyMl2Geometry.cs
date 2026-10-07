using System;
using Braillic.Tracking.Runtime;

namespace Braillic.Tracking.Implementation.ML2
{
    /// <summary>Explicit capability of the unchanged single-model desktop protocol.
    /// Coordinates are the current desktop model reflected into canonical body coordinates.</summary>
    public static class LegacyMl2Geometry
    {
        public const string Id = "ml2-legacy-asymmetric-4";
        public const string Revision = "1";
        public static MarkerGeometryDefinition Create() => new MarkerGeometryDefinition(Id, Revision, new[] {
            new Vector3d(-.0466, 0, -.0206), new Vector3d(.0444, 0, -.0206),
            new Vector3d(.0095, .0623, -.0206), new Vector3d(-.007, -.0368, -.0206) });

        public static bool Matches(MarkerGeometryDefinition geometry)
        {
            var expected = Create();
            if (geometry.Id != Id || geometry.Revision != Revision || geometry.Markers.Count != expected.Markers.Count) return false;
            for (int i = 0; i < expected.Markers.Count; i++)
            {
                var a = geometry.Markers[i]; var b = expected.Markers[i];
                if (Math.Abs(a.X-b.X)>1e-7 || Math.Abs(a.Y-b.Y)>1e-7 || Math.Abs(a.Z-b.Z)>1e-7) return false;
            }
            return true;
        }
    }
}
