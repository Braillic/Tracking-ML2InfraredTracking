using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public readonly struct ReplayPose
    {
        public readonly string ObjectId;
        public readonly ulong Sequence;
        public readonly double CaptureSeconds,AvailableSeconds;
        public readonly RigidPose FrameFromObject;
        public readonly bool Tracked;
        public ReplayPose(string objectId,ulong sequence,double captureSeconds,double availableSeconds,RigidPose pose,bool tracked=true)
        {ObjectId=objectId;Sequence=sequence;CaptureSeconds=captureSeconds;AvailableSeconds=availableSeconds;FrameFromObject=pose;Tracked=tracked;}
    }
    /// <summary>Reusable recorded-pose provider; timings are relative to Start. No camera, SDK or file format dependency.</summary>
    public sealed class ReplayTrackingProvider : ITrackingProvider
    {
        private readonly ReplayPose[] records;
        private readonly string frameId;
        private TrackingProviderContext context;
        private TrackingSession session;
        private double started;
        private int next;
        private bool disposed;
        public string Id { get; }
        public ProviderState State { get; private set; }
        public string LastError { get; private set; }
        public ReplayTrackingProvider(string id,string frameId,IReadOnlyList<ReplayPose> records)
        {
            if(string.IsNullOrWhiteSpace(id)||string.IsNullOrWhiteSpace(frameId)||records==null)throw new ArgumentException("Replay configuration required.");
            Id=id;this.frameId=frameId;this.records=new ReplayPose[records.Count];
            var previous=new Dictionary<string,ReplayPose>();double last=-1;
            for(int i=0;i<records.Count;i++)
            {
                var p=records[i];
                if(string.IsNullOrWhiteSpace(p.ObjectId)||!Numeric.Finite(p.CaptureSeconds)||!Numeric.Finite(p.AvailableSeconds)||
                    p.CaptureSeconds<0||p.AvailableSeconds<p.CaptureSeconds||p.AvailableSeconds<last||(p.Tracked&&!p.FrameFromObject.IsValid))throw new ArgumentException("Invalid recording.");
                if(previous.TryGetValue(p.ObjectId,out var old)&&(p.Sequence<=old.Sequence||p.CaptureSeconds<=old.CaptureSeconds))throw new ArgumentException("Recording must increase per object.");
                previous[p.ObjectId]=p;last=p.AvailableSeconds;this.records[i]=p;
            }
        }
        public void Initialize(TrackingProviderContext value)
        { if(context!=null)throw new InvalidOperationException("Already initialized.");context=value; }
        public void Start()
        {
            if(disposed)throw new ObjectDisposedException(nameof(ReplayTrackingProvider));
            if(State==ProviderState.Running)return;
            started=context.Clock.NowSeconds;next=0;
            session=context.BeginSession(Guid.NewGuid().ToString("N"),new CoordinateFrame(frameId,Guid.NewGuid().ToString("N")),"recorded-calibration",
                new ClockMapping("replay/"+Id,context.Clock.Id,1,started,0,0,double.MaxValue));
            State=ProviderState.Running;
        }
        public void Stop() { context?.EndSession();State=ProviderState.Stopped; }
        public void Pump(TrackingUpdatePhase phase)
        {
            if(State!=ProviderState.Running)return;
            while(next<records.Length&&started+records[next].AvailableSeconds<=context.Clock.NowSeconds)
            {
                var p=records[next++];
                if(p.Tracked)
                {
                    var issue=context.Publish(session,p.ObjectId,p.Sequence,p.CaptureSeconds,started+p.AvailableSeconds,p.FrameFromObject);
                    if(issue!=TrackingIssue.None)throw new InvalidOperationException("Replay publication failed: "+issue);
                }
                else context.MarkUnavailable(session,p.ObjectId);
            }
        }
        public void ResetReferenceFrame() { Stop();Start(); }
        public void Dispose() { if(disposed)return;Stop();disposed=true; }
    }
}
