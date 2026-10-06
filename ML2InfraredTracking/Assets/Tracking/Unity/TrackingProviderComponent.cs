using Braillic.Tracking.Runtime;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    /// <summary>Unity composition extension point. Only concrete implementations know device SDKs.</summary>
    public abstract class TrackingProviderComponent : MonoBehaviour
    {
        [SerializeField] private bool required = true;
        public bool Required => required;
        public abstract ITrackingProvider CreateProvider();
    }
}
