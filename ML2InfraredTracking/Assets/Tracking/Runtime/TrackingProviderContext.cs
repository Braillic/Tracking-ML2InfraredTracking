using System;
using System.Threading;

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
            if(!ClockMap.TryMap(captureInSourceClock,Clock.NowSeconds,out double capture))return TrackingIssue.InvalidTimestamp;
            var issue=core.Publish(token,objectId,sequence,capture,availableInRuntimeClock,frameFromBody,quality);
            if(issue==TrackingIssue.None)published();
            return issue;
        }
        public void MarkUnavailable(TrackingSession token,string objectId)
        { CheckThread();if(enabled&&ReferenceEquals(token,Session))core.MarkUnavailable(token,objectId); }
    }
}
