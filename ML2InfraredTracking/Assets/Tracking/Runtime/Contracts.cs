using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public enum TrackingUpdatePhase { Update, LateUpdate, BeforeRender }
    public enum TrackingState { Stopped, Starting, Running, Stopping, Faulted, Disposed }
    public enum ProviderState { Stopped, Starting, Running, Stopping, Faulted }



    public readonly struct ObservationRequirements
    {
        public readonly double MaximumAgeSeconds, MaximumSkewSeconds, MaximumClockUncertaintySeconds;
        public ObservationRequirements(double maximumAgeSeconds, double maximumSkewSeconds, double maximumClockUncertaintySeconds)
        {
            Check(maximumAgeSeconds); Check(maximumSkewSeconds); Check(maximumClockUncertaintySeconds);
            MaximumAgeSeconds=maximumAgeSeconds; MaximumSkewSeconds=maximumSkewSeconds;
            MaximumClockUncertaintySeconds=maximumClockUncertaintySeconds;
        }
        private static void Check(double value) { if(!Numeric.Finite(value)||value<0)throw new ArgumentOutOfRangeException(nameof(value)); }
    }

    public sealed class ObjectBinding
    {
        public string ProviderId { get; }
        public string ProviderObjectId { get; }
        // Logical object's axes need not be identical to the vendor/marker model axes.
        public RigidPose ProviderBodyFromLogicalBody { get; }
        public ObjectBinding(string providerId,string providerObjectId,RigidPose providerBodyFromLogicalBody)
        {
            if(string.IsNullOrWhiteSpace(providerId)||string.IsNullOrWhiteSpace(providerObjectId))throw new ArgumentException("Object identifiers required.");
            if(!providerBodyFromLogicalBody.IsValid)throw new ArgumentException("Invalid rigid-body calibration.");
            ProviderId=providerId;ProviderObjectId=providerObjectId;ProviderBodyFromLogicalBody=providerBodyFromLogicalBody.Normalized;
        }
    }

    public readonly struct ResolvedTrackingPose
    {
        public readonly string ObjectId, ReferenceObjectId;
        public readonly CoordinateFrame Frame;
        public readonly RigidPose FrameFromObject;
        public readonly PoseObservation Source;
        public readonly PoseObservation? Reference;
        public readonly long Revision;
        public readonly double ClockUncertaintySeconds;
        internal ResolvedTrackingPose(string objectId,string referenceId,CoordinateFrame frame,RigidPose pose,
            PoseObservation source,PoseObservation? reference,long revision,double uncertainty)
        { ObjectId=objectId;ReferenceObjectId=referenceId;Frame=frame;FrameFromObject=pose;Source=source;Reference=reference;Revision=revision;ClockUncertaintySeconds=uncertainty; }
    }

    public readonly struct ProviderStatus
    {
        public readonly string Id, Error;
        public readonly ProviderState State;
        public readonly bool Required;
        public ProviderStatus(string id,ProviderState state,string error,bool required)
        { Id=id;State=state;Error=error;Required=required; }
    }


}
