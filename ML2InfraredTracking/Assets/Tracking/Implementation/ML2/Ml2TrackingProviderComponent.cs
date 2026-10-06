using Braillic.Tracking.Runtime;
using Braillic.Tracking.Unity;
using UnityEngine;

namespace Braillic.Tracking.Implementation.ML2
{
    public sealed class Ml2TrackingProviderComponent : TrackingProviderComponent
    {
        [SerializeField] private string providerId = "ml2-ir";
        [SerializeField] private string providerObjectId = "marker-body";
        [SerializeField] private string calibrationId = "ir-calibration-v1";
        [SerializeField] private DepthSensorAPI sensor;
        [SerializeField] private ML2DepthRawStream capture;
        [SerializeField] private DepthFrameTcpServer sender;
        [SerializeField] private PoseEstimateTcpServer receiver;
        public override ITrackingProvider CreateProvider() => new Ml2TrackingProvider(providerId,providerObjectId,calibrationId,sensor,capture,sender,receiver);
    }
}
