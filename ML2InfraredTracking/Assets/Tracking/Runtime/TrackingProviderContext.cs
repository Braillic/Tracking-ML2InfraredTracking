using System;
using System.Threading;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    /// <summary>Provider-scoped publication boundary. Use only on the runtime owner thread.
    /// Worker SDK callbacks must use a bounded mailbox and publish from Pump.</summary>
    public sealed class TrackingProviderContext
    {
        private readonly TrackingCore core;
        private readonly Action<CoordinateFrame> invalidated;
        private readonly Action changed;
        private readonly Action published;
        private bool enabled;
        private readonly int ownerThread=Thread.CurrentThread.ManagedThreadId;
        private void CheckThread()
        { if(Thread.CurrentThread.ManagedThreadId!=ownerThread)throw new InvalidOperationException("Publish from the provider Pump on the tracking owner thread."); }
        public string ProviderId { get; }
        public ITrackingClock Clock { get; }
        public TrackingSession Session { get; private set; }
        public ClockMapping ClockMap { get; private set; }
        public ProviderGeometryConfiguration GeometryConfiguration { get; internal set; }
        public CoordinateFrame Frame => Session?.ReferenceFrame;
        internal TrackingProviderContext(string providerId,TrackingCore core,ITrackingClock clock,Action<CoordinateFrame> invalidated,Action changed,Action published)
        { ProviderId=providerId;this.core=core;Clock=clock;this.invalidated=invalidated;this.changed=changed;this.published=published; }
        internal void Enable() { enabled=true; }
        internal void Disable() { enabled=false;EndSession(); }
        public TrackingSession BeginSession(string sessionId,CoordinateFrame frame,string calibrationId,ClockMapping clockMap)
        {
            CheckThread();
            if(!enabled)throw new InvalidOperationException("Provider is stopped.");
            if(clockMap==null||clockMap.TargetClockId!=Clock.Id)throw new ArgumentException("Map provider time to the runtime clock.");
            EndSession();
            Session=core.OpenSession(ProviderId,sessionId,Clock.Id,frame,calibrationId);ClockMap=clockMap;
            changed();return Session;
        }
        public void EndSession()
        {
            CheckThread();
            if(Session==null)return;
            var old=Session;core.CloseSession(old);Session=null;ClockMap=null;
            invalidated(old.ReferenceFrame);changed();
        }
        public TrackingIssue Publish(TrackingSession token,string objectId,ulong sequence,double captureInSourceClock,
            double availableInRuntimeClock,RigidPose frameFromBody,double? quality=null)
        {
            CheckThread();
            if(!enabled||token==null||!ReferenceEquals(token,Session))return TrackingIssue.SessionSuperseded;
            if(GeometryConfiguration!=null)
            {
                bool configured=false;
                foreach(var tool in GeometryConfiguration.Tools)
                    if(tool.Binding.Tracking.ProviderObjectId==objectId)configured=true;
                if(!configured)return TrackingIssue.UnknownSource;
            }
            if(!ClockMap.TryMap(captureInSourceClock,Clock.NowSeconds,out double capture))return TrackingIssue.InvalidTimestamp;
            var issue=core.Publish(token,objectId,sequence,capture,availableInRuntimeClock,frameFromBody,quality);
            if(issue==TrackingIssue.None)published();
            return issue;
        }
        public void MarkUnavailable(TrackingSession token,string objectId)
        { CheckThread();if(enabled&&ReferenceEquals(token,Session))core.MarkUnavailable(token,objectId); }

        /// <summary>Routes all identified geometries in one captured frame to configured bodies.
        /// Unknown/revised/ambiguous matches invalidate the entire new frame; old frames never erase newer data.</summary>
        public TrackingIssue PublishIdentifiedFrame(TrackingSession token, ulong sequence, double captureInSourceClock,
            double availableInRuntimeClock, IReadOnlyList<GeometryMatch> matches)
        {
            CheckThread();
            if (!enabled || token == null || !ReferenceEquals(token, Session)) return TrackingIssue.SessionSuperseded;
            if (GeometryConfiguration == null) return TrackingIssue.ConfigurationMissing;
            if (matches == null) throw new ArgumentNullException(nameof(matches));
            if (!ClockMap.TryMap(captureInSourceClock, Clock.NowSeconds, out double capture)) return TrackingIssue.InvalidTimestamp;
            var samples = new List<TrackedBodySample>();
            var geometryIds = new HashSet<string>();
            var detections = new HashSet<int>();
            TrackingIssue rejection = TrackingIssue.None;
            foreach (var match in matches)
            {
                if (match == null || !geometryIds.Add(match.GeometryId)) { rejection = TrackingIssue.AmbiguousGeometry; break; }
                ProviderGeometryBinding selected = null;
                foreach (var tool in GeometryConfiguration.Tools)
                    if (tool.Geometry.Id == match.GeometryId) { selected = tool; break; }
                if (selected == null) { rejection = TrackingIssue.UnknownGeometry; break; }
                if (selected.Geometry.Revision != match.GeometryRevision) { rejection = TrackingIssue.GeometryRevisionMismatch; break; }
                if (match.DetectionIndices.Count > selected.Geometry.Markers.Count) { rejection = TrackingIssue.AmbiguousGeometry; break; }
                foreach (int index in match.DetectionIndices)
                    if (!detections.Add(index)) { rejection = TrackingIssue.AmbiguousGeometry; break; }
                if (rejection != TrackingIssue.None) break;
                samples.Add(new TrackedBodySample(selected.Binding.Tracking.ProviderObjectId, match.ReferenceFromBody, match.Quality));
            }
            if (rejection != TrackingIssue.None) samples.Clear();
            var issue = core.PublishFrame(token, sequence, capture, availableInRuntimeClock, samples);
            if (issue != TrackingIssue.None) return issue;
            published(); // All bodies and disappearances are visible before any consumer is notified.
            return rejection;
        }
    }
}
