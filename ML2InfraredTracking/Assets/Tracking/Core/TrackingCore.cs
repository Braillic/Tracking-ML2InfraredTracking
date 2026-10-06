using System;
using System.Collections.Generic;

namespace Braillic.Tracking
{
    /// <summary>Thread-safe latest-measurement store. No hardware, rendering, or acquisition policy.</summary>
    public sealed class TrackingCore : ITrackingReader
    {
        private sealed class ObjectState
        {
            public bool Seen, Available;
            public ulong Sequence;
            public double Capture;
            public PoseObservation Observation;
        }
        private sealed class SourceState
        {
            public TrackingSession Session;
            public bool Open=true;
            public readonly Dictionary<string,ObjectState> Objects=new Dictionary<string,ObjectState>();
        }
        private readonly object gate=new object();
        private readonly Dictionary<string,SourceState> sources=new Dictionary<string,SourceState>();
        private readonly ITrackingClock clock;
        private readonly string clockId;
        private readonly int maximumObjectsPerSource;
        private double lastNow=double.NegativeInfinity;
        private TrackingIssue clockFault;
        public string ClockId => clockId;

        public TrackingCore(ITrackingClock clock,int maximumObjectsPerSource=64)
        {
            this.clock=clock ?? throw new ArgumentNullException(nameof(clock));
            clockId=Numeric.Required(clock.Id,nameof(clock));
            if (maximumObjectsPerSource<1) throw new ArgumentOutOfRangeException(nameof(maximumObjectsPerSource));
            this.maximumObjectsPerSource=maximumObjectsPerSource;
        }

        public TrackingSession OpenSession(string sourceId,string sessionId,string mappedClockId,
            CoordinateFrame referenceFrame,string calibrationId)
        {
            Numeric.Required(sourceId,nameof(sourceId)); Numeric.Required(sessionId,nameof(sessionId));
            Numeric.Required(calibrationId,nameof(calibrationId));
            if (referenceFrame==null) throw new ArgumentNullException(nameof(referenceFrame));
            if (mappedClockId!=clockId) throw new ArgumentException("Map device time into the core clock before publication.",nameof(mappedClockId));
            lock(gate)
            {
                var token=new TrackingSession(sourceId,sessionId,clockId,calibrationId,referenceFrame);
                sources[sourceId]=new SourceState { Session=token };
                return token; // Reopening always invalidates old tokens, even if textual IDs repeat.
            }
        }

        private TrackingIssue Current(TrackingSession token,out SourceState state)
        {
            state=null;
            if (token==null || !sources.TryGetValue(token.SourceId,out state)) return TrackingIssue.UnknownSource;
            if (!ReferenceEquals(token,state.Session)) return TrackingIssue.SessionSuperseded;
            return state.Open ? TrackingIssue.None : TrackingIssue.SessionClosed;
        }
        private TrackingIssue Now(out double now)
        {
            now=clock.NowSeconds;
            if (clock.Id!=clockId) clockFault=TrackingIssue.ClockMismatch;
            if (clockFault!=TrackingIssue.None) return clockFault;
            if (!Numeric.Finite(now) || now<0) return TrackingIssue.InvalidTimestamp;
            if (now<lastNow) { clockFault=TrackingIssue.ClockRegressed; return clockFault; }
            lastNow=now; return TrackingIssue.None;
        }

        public TrackingIssue Publish(TrackingSession session,string objectId,ulong sequence,
            double captureSeconds,double availableSeconds,RigidPose pose,double? quality=null)
        {
            Numeric.Required(objectId,nameof(objectId));
            lock(gate)
            {
                var issue=Current(session,out var state); if(issue!=TrackingIssue.None)return issue;
                issue=Now(out double now); if(issue!=TrackingIssue.None)return issue;
                if (!Numeric.Finite(captureSeconds)||!Numeric.Finite(availableSeconds)||captureSeconds<0
                    ||captureSeconds>availableSeconds||availableSeconds>now) return TrackingIssue.InvalidTimestamp;
                if (!pose.IsValid) return TrackingIssue.InvalidPose;
                if (quality.HasValue && !Numeric.Finite(quality.Value)) return TrackingIssue.InvalidQuality;
                if (!state.Objects.TryGetValue(objectId,out var obj))
                {
                    if(state.Objects.Count>=maximumObjectsPerSource)return TrackingIssue.CapacityReached;
                    obj=new ObjectState(); state.Objects.Add(objectId,obj);
                }
                if(obj.Seen && (sequence<=obj.Sequence || captureSeconds<=obj.Capture))return TrackingIssue.OutOfOrder;
                obj.Seen=true; obj.Available=true; obj.Sequence=sequence; obj.Capture=captureSeconds;
                obj.Observation=new PoseObservation(session,objectId,sequence,captureSeconds,availableSeconds,pose.Normalized,quality);
                return TrackingIssue.None;
            }
        }

        // No frame fabricated for a disconnect/unavailable poll. Retain watermarks to stop replay of old samples.
        public TrackingIssue MarkUnavailable(TrackingSession session,string objectId)
        {
            Numeric.Required(objectId,nameof(objectId));
            lock(gate)
            {
                var issue=Current(session,out var state); if(issue!=TrackingIssue.None)return issue;
                if(state.Objects.TryGetValue(objectId,out var obj))obj.Available=false;
                return TrackingIssue.None;
            }
        }
        public TrackingIssue CloseSession(TrackingSession session)
        {
            lock(gate)
            { var issue=Current(session,out var state); if(issue==TrackingIssue.None)state.Open=false; return issue; }
        }
        private static void CheckLimit(double value,string name)
        { if(!Numeric.Finite(value)||value<0)throw new ArgumentOutOfRangeException(name); }
        private TrackingIssue Read(string sourceId,string objectId,double now,double maxAge,out PoseObservation result)
        {
            result=default;
            if(!sources.TryGetValue(sourceId,out var state))return TrackingIssue.UnknownSource;
            if(!state.Open)return TrackingIssue.SessionClosed;
            if(!state.Objects.TryGetValue(objectId,out var obj)||!obj.Seen)return TrackingIssue.NoObservation;
            if(!obj.Available)return TrackingIssue.SourceUnavailable;
            double age=now-obj.Observation.CaptureSeconds;
            if(age<0)return TrackingIssue.InvalidTimestamp;
            if(age>maxAge)return TrackingIssue.Stale;
            result=obj.Observation;return TrackingIssue.None;
        }
        public bool TryGetLatest(string sourceId,string objectId,double maximumAgeSeconds,
            out PoseObservation observation,out TrackingIssue issue)
        {
            Numeric.Required(sourceId,nameof(sourceId));Numeric.Required(objectId,nameof(objectId));
            CheckLimit(maximumAgeSeconds,nameof(maximumAgeSeconds));
            lock(gate)
            {
                observation=default;issue=Now(out double now);
                if(issue==TrackingIssue.None)issue=Read(sourceId,objectId,now,maximumAgeSeconds,out observation);
                return issue==TrackingIssue.None;
            }
        }
        public bool TryGetRelative(string sourceId,string objectId,string referenceSourceId,string referenceId,
            double maximumAgeSeconds,double maximumCaptureSkewSeconds,
            out RelativeObservation observation,out TrackingIssue issue)
        {
            Numeric.Required(sourceId,nameof(sourceId));Numeric.Required(objectId,nameof(objectId));
            Numeric.Required(referenceSourceId,nameof(referenceSourceId));Numeric.Required(referenceId,nameof(referenceId));
            CheckLimit(maximumAgeSeconds,nameof(maximumAgeSeconds));CheckLimit(maximumCaptureSkewSeconds,nameof(maximumCaptureSkewSeconds));
            lock(gate)
            {
                observation=default;issue=Now(out double now);if(issue!=TrackingIssue.None)return false;
                issue=Read(sourceId,objectId,now,maximumAgeSeconds,out var obj);if(issue!=TrackingIssue.None)return false;
                issue=Read(referenceSourceId,referenceId,now,maximumAgeSeconds,out var reference);if(issue!=TrackingIssue.None)return false;
                if(!obj.Session.ReferenceFrame.Equals(reference.Session.ReferenceFrame)) { issue=TrackingIssue.FrameMismatch;return false; }
                if(Math.Abs(obj.CaptureSeconds-reference.CaptureSeconds)>maximumCaptureSkewSeconds) { issue=TrackingIssue.TimeMismatch;return false; }
                var relative=reference.ReferenceFromObject.Inverse*obj.ReferenceFromObject;
                if(!relative.IsValid) { issue=TrackingIssue.InvalidPose;return false; }
                observation=new RelativeObservation(obj,reference,relative);return true;
            }
        }
    }
}
