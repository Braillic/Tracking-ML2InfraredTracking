using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    /// <summary>Extension point for future identification/pose algorithms. TFrame is the adapter's
    /// calibrated detection input. Return all bodies found in ONE frame, or an empty list for none.
    /// No implementation is supplied. Poses passed to PublishIdentifiedFrame must already be in
    /// the provider session's reference coordinates, with canonical body axes and metres.</summary>
    public interface IGeometryIdentifier<TFrame>
    {
        IReadOnlyList<GeometryMatch> Identify(TFrame frame, IReadOnlyList<MarkerGeometryDefinition> geometries);
    }
}
