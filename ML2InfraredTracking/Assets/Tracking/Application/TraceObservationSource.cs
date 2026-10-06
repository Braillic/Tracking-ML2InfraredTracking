using System;
using Braillic.Tracking.Runtime;

namespace Braillic.Tracking.Application
{
    public readonly struct TipObservation
    {
        public readonly ResolvedTrackingPose Tracking;
        public readonly Vector3d TipPosition;
        public readonly ulong CollectionEpoch;
        internal TipObservation(ResolvedTrackingPose tracking,Vector3d tip,ulong epoch)
        {Tracking=tracking;TipPosition=tip;CollectionEpoch=epoch;}
    }
    /// <summary>Application-level raw tip source for tracing algorithms. No Unity, transport,
    /// prefab, anatomy or fixed probe length. Configured body/reference IDs are logical IDs.</summary>
    public sealed class TraceObservationSource
    {
        private readonly ITrackingSystem system;
        private readonly string objectId,referenceObjectId;
        private readonly Vector3d tipInBody;
        private readonly ObservationRequirements requirements;
        private TrackingSession lastSource,lastReference;
        private CoordinateFrame lastFrame;
        private long lastRevision=-1;
        public ulong CollectionEpoch { get; private set; }
        public string InstrumentCalibrationId { get; }
        public TraceObservationSource(ITrackingSystem system,string objectId,string referenceObjectId,
            Vector3d tipInBody,string instrumentCalibrationId,ObservationRequirements requirements)
        {
            this.system=system??throw new ArgumentNullException(nameof(system));
            if(string.IsNullOrWhiteSpace(objectId)||string.IsNullOrWhiteSpace(instrumentCalibrationId)||!tipInBody.IsFinite)throw new ArgumentException("Valid instrument binding/calibration required.");
            this.objectId=objectId;this.referenceObjectId=referenceObjectId;
            this.tipInBody=tipInBody;InstrumentCalibrationId=instrumentCalibrationId;this.requirements=requirements;
        }
        // Null frame requests the provider's native frame. For AR presentation pass the display frame explicitly.
        public bool TryRead(CoordinateFrame frame,out TipObservation tip,out TrackingIssue issue)
        {
            tip=default;
            if(system.Revision!=lastRevision){CollectionEpoch=0;lastSource=null;lastReference=null;lastFrame=null;lastRevision=system.Revision;}
            ResolvedTrackingPose pose;
            bool ok=!string.IsNullOrEmpty(referenceObjectId)
                ? system.TryGetRelativePose(objectId,referenceObjectId,requirements,out pose,out issue)
                : frame!=null ? system.TryGetPoseInFrame(objectId,frame,requirements,out pose,out issue)
                : system.TryGetPose(objectId,requirements,out pose,out issue);
            if(!ok)return false;
            if(!ReferenceEquals(lastSource,pose.Source.Session)||!ReferenceEquals(lastReference,pose.Reference?.Session)||!Equals(lastFrame,pose.Frame))
            {
                CollectionEpoch=BitConverter.ToUInt64(Guid.NewGuid().ToByteArray(),0)|1UL;
                lastSource=pose.Source.Session;lastReference=pose.Reference?.Session;lastFrame=pose.Frame;
            }
            tip=new TipObservation(pose,pose.FrameFromObject.Apply(tipInBody),CollectionEpoch);return true;
        }
    }
}
