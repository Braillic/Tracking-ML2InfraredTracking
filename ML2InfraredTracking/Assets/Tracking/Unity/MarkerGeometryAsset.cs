using Braillic.Tracking.Runtime;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    [CreateAssetMenu(menuName = "Tracking/Marker Geometry")]
    public sealed class MarkerGeometryAsset : ScriptableObject
    {
        [SerializeField] private string geometryId;
        [SerializeField] private string revision = "1";
        [Tooltip("Marker centres in Unity body-local metres. Converted to canonical coordinates at the API boundary.")]
        [SerializeField] private Vector3[] markerCentres;
        public string GeometryId => geometryId;
        public MarkerGeometryDefinition CreateDefinition()
        {
            if (markerCentres == null) throw new System.InvalidOperationException("Set marker centres on the geometry asset.");
            var points = new Vector3d[markerCentres.Length];
            for (int i = 0; i < points.Length; i++)
                points[i] = new Vector3d(markerCentres[i].x, markerCentres[i].y, -markerCentres[i].z);
            return new MarkerGeometryDefinition(geometryId, revision, points);
        }
    }
}
