using System;

namespace Braillic.Tracking
{
    public interface ITrackingClock { string Id { get; } double NowSeconds { get; } }
    public enum TrackingIssue
    {
        None, UnknownSource, SessionClosed, SessionSuperseded, NoObservation, SourceUnavailable,
        InvalidPose, InvalidTimestamp, InvalidQuality, OutOfOrder, Stale,
        ClockMismatch, ClockRegressed, FrameMismatch, TimeMismatch, CapacityReached
    }

    public sealed class TrackingSession
    {
        public string SourceId { get; }
        public string SessionId { get; }
        public string ClockId { get; }
        public string CalibrationId { get; }
        public CoordinateFrame ReferenceFrame { get; }
        internal TrackingSession(string source,string session,string clock,string calibration,CoordinateFrame frame)
        { SourceId=source; SessionId=session; ClockId=clock; CalibrationId=calibration; ReferenceFrame=frame; }
    }

    public readonly struct PoseObservation
    {
        public readonly TrackingSession Session;
        public readonly string ObjectId;
        public readonly ulong Sequence;
        public readonly double CaptureSeconds, AvailableSeconds;
        public readonly RigidPose ReferenceFromObject;
        public readonly double? Quality;
        internal PoseObservation(TrackingSession session,string objectId,ulong sequence,double capture,
            double available,RigidPose pose,double? quality)
        { Session=session; ObjectId=objectId; Sequence=sequence; CaptureSeconds=capture;
            AvailableSeconds=available; ReferenceFromObject=pose; Quality=quality; }
    }

    public readonly struct RelativeObservation
    {
        // Original observations retain BOTH capture times and identities. No invented fused time.
        public readonly PoseObservation Object, Reference;
        public readonly RigidPose ReferenceFromObject;
        internal RelativeObservation(PoseObservation obj,PoseObservation reference,RigidPose pose)
        { Object=obj; Reference=reference; ReferenceFromObject=pose; }
    }

    public interface ITrackingReader
    {
        bool TryGetLatest(string sourceId,string objectId,double maximumAgeSeconds,
            out PoseObservation observation,out TrackingIssue issue);
        bool TryGetRelative(string sourceId,string objectId,string referenceSourceId,string referenceId,
            double maximumAgeSeconds,double maximumCaptureSkewSeconds,
            out RelativeObservation observation,out TrackingIssue issue);
    }

    /// <summary>Lifecycle of adapters; capture/SDK/transport setup stays in their implementation.</summary>
    public interface ITrackingBackend : IDisposable
    {
        void Start();
        void Poll();
        void Stop();
    }
}
