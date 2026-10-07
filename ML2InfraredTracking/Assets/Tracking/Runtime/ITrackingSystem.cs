using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public interface ITrackingSystem : IDisposable
    {
        TrackingConfiguration Configuration { get; }
        void ConfigureTools(TrackingConfiguration configuration);
        bool TryGetToolDefinition(string objectId, out TrackedToolDefinition tool);
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
