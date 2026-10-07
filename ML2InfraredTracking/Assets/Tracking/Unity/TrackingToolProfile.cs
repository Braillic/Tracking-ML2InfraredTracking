using Braillic.Tracking.Runtime;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    [CreateAssetMenu(menuName = "Tracking/Tool Profile")]
    public sealed class TrackingToolProfile : ScriptableObject
    {
        [SerializeField] private string displayName;
        [SerializeField] private TrackedToolRole role;
        [SerializeField] private MarkerGeometryAsset geometry;
        public string DisplayName => displayName;
        public TrackedToolRole Role => role;
        public MarkerGeometryAsset Geometry => geometry;
    }
}
