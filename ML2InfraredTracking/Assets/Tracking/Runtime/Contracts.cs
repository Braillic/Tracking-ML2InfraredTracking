using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public enum TrackingUpdatePhase { Update, LateUpdate, BeforeRender }
    public enum TrackingState { Stopped, Starting, Running, Stopping, Faulted, Disposed }
    public enum ProviderState { Stopped, Starting, Running, Stopping, Faulted }

    public interface ITrackingProvider : IDisposable
    {
        string Id { get; }
        ProviderState State { get; }
        string LastError { get; }
        void Initialize(TrackingProviderContext context);
        void Start();
        void Stop();
        void Pump(TrackingUpdatePhase phase);
        void ResetReferenceFrame();
    }

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

    public interface ITrackingSystem : IDisposable
    {
        TrackingState State { get; }
        long Revision { get; }
        string LastError { get; }
        void Start();
        void Stop();
        void Pump(TrackingUpdatePhase phase);
        bool TryGetProviderFrame(string providerId,out CoordinateFrame frame);
        void ResetProviderReference(string providerId);
        void SetFrameCalibration(string id,CoordinateFrame from,CoordinateFrame to,RigidPose toFrom,double validFrom,double validUntil);
        void RemoveFrameCalibration(string id);
        bool TryGetPose(string objectId,ObservationRequirements requirements,out ResolvedTrackingPose pose,out TrackingIssue issue);
        bool TryGetPoseInFrame(string objectId,CoordinateFrame frame,ObservationRequirements requirements,out ResolvedTrackingPose pose,out TrackingIssue issue);
        bool TryGetRelativePose(string objectId,string referenceObjectId,ObservationRequirements requirements,out ResolvedTrackingPose pose,out TrackingIssue issue);
        IReadOnlyList<ProviderStatus> GetProviderStatus();
    }
}
