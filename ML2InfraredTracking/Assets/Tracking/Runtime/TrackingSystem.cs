using System;
using System.Collections.Generic;
using System.Threading;

namespace Braillic.Tracking.Runtime
{
    /// <summary>Device-independent tracking service. All commands, queries and provider publication
    /// run on one owner thread; providers marshal asynchronous SDK output through their mailboxes.</summary>
    public sealed class TrackingSystem : ITrackingSystem
    {
        private sealed class Entry
        {
            public ITrackingProvider Provider;
            public TrackingProviderContext Context;
            public bool Required,Failed;
            public string Error;
        }
        private readonly Dictionary<string,Entry> providers=new Dictionary<string,Entry>();
        private readonly Dictionary<string,ObjectBinding[]> objects=new Dictionary<string,ObjectBinding[]>();
        private readonly TrackingCore core;
        private readonly ITrackingClock clock;
        private readonly FrameCalibrationGraph frames=new FrameCalibrationGraph();
        private readonly int ownerThread=Thread.CurrentThread.ManagedThreadId;
        private bool requested,stopping,faulted,clockBroken;
        private double lastNow=double.NegativeInfinity;
        public TrackingState State { get; private set; }
        public long Revision { get; private set; }
        public string LastError { get; private set; }
        public TrackingConfiguration Configuration { get; private set; }
        public event Action ObservationPublished;
        public event Action ContinuityChanged;

        public TrackingSystem(ITrackingClock clock)
        { this.clock=clock??throw new ArgumentNullException(nameof(clock));core=new TrackingCore(clock); }
        private void CheckThread()
        {
            if(Thread.CurrentThread.ManagedThreadId!=ownerThread)throw new InvalidOperationException("Use the tracking owner thread; marshal worker callbacks in the provider.");
            if(State==TrackingState.Disposed)throw new ObjectDisposedException(nameof(TrackingSystem));
        }
        private void Changed() { Revision++;ContinuityChanged?.Invoke(); }
        public void AddProvider(ITrackingProvider provider,bool required=true)
        {
            CheckThread();if(State!=TrackingState.Stopped)throw new InvalidOperationException("Configure providers before starting.");
            if(provider==null||string.IsNullOrWhiteSpace(provider.Id)||providers.ContainsKey(provider.Id))throw new ArgumentException("Unique provider ID required.");
            var context=new TrackingProviderContext(provider.Id,core,clock,frames.RemoveFrame,Changed,()=>ObservationPublished?.Invoke());
            try { provider.Initialize(context); }
            catch { context.Disable();provider.Stop();provider.Dispose();throw; }
            providers.Add(provider.Id,new Entry{Provider=provider,Context=context,Required=required});
        }
        /// <summary>Priority is explicit list order. No automatic pose averaging or hidden provider selection.</summary>
        public void BindObject(string objectId,params ObjectBinding[] priorityOrder)
        {
            CheckThread();if(State!=TrackingState.Stopped)throw new InvalidOperationException("Change object mappings while stopped.");
            if(Configuration!=null)throw new InvalidOperationException("Use ConfigureTools to replace a geometry-aware configuration atomically.");
            if(string.IsNullOrWhiteSpace(objectId)||priorityOrder==null||priorityOrder.Length==0)throw new ArgumentException("Object bindings required.");
            var seen=new HashSet<string>();
            foreach(var binding in priorityOrder)
                if(binding==null||!providers.ContainsKey(binding.ProviderId)||!seen.Add(binding.ProviderId))throw new ArgumentException("Bind each provider at most once, after adding it.");
            objects[objectId]=(ObjectBinding[])priorityOrder.Clone();Changed();
        }
        public bool TryGetToolDefinition(string objectId, out TrackedToolDefinition tool)
        {
            CheckThread(); tool = null;
            if (Configuration == null) return false;
            foreach (var candidate in Configuration.Tools)
                if (candidate.ObjectId == objectId) { tool = candidate; return true; }
            return false;
        }
        public void ConfigureTools(TrackingConfiguration configuration)
        {
            CheckThread();
            if (State != TrackingState.Stopped || !ShutdownComplete)
                throw new InvalidOperationException("Stop tracking and finish resource release before changing tools.");
            if (configuration == null) throw new ArgumentNullException(nameof(configuration));
            var routes = new Dictionary<string, ObjectBinding[]>();
            foreach (var tool in configuration.Tools)
            {
                var bindings = new List<ObjectBinding>();
                foreach (var binding in tool.Bindings)
                {
                    if (!providers.ContainsKey(binding.Tracking.ProviderId)) throw new ArgumentException("Unknown provider: " + binding.Tracking.ProviderId);
                    bindings.Add(binding.Tracking);
                }
                routes.Add(tool.ObjectId, bindings.ToArray());
            }
            // Validate every provider before replacing any route or context configuration.
            var selections = new Dictionary<string, ProviderGeometryConfiguration>();
            foreach (var pair in providers)
            {
                var selection = configuration.ForProvider(pair.Key);
                if (selection.Tools.Count > 0)
                {
                    if (!(pair.Value.Provider is IGeometryTrackingProvider geometryProvider))
                        throw new NotSupportedException(pair.Key + " does not support geometry configuration.");
                    geometryProvider.ValidateGeometryConfiguration(selection);
                }
                selections.Add(pair.Key, selection);
            }
            foreach (var pair in providers) { pair.Value.Context.EndSession(); pair.Value.Context.GeometryConfiguration = selections[pair.Key]; }
            objects.Clear(); foreach (var pair in routes) objects.Add(pair.Key, pair.Value);
            Configuration = configuration; Changed();
        }
        public void Start()
        {
            CheckThread();if(providers.Count==0||objects.Count==0)throw new InvalidOperationException("Configure providers and object mappings first.");
            if(clockBroken)throw new InvalidOperationException("Create a new tracking system after a runtime clock reset.");
            requested=true;
            if(stopping||State==TrackingState.Running||State==TrackingState.Starting)return;
            StartProviders();
        }
        private void StartProviders()
        {
            faulted=false;LastError=null;State=TrackingState.Starting;
            foreach(var entry in providers.Values)
            {
                entry.Failed=false;entry.Error=null;entry.Context.Enable();
                try { entry.Provider.Start(); }
                catch(Exception error) { Fail(entry,error.Message);if(entry.Required)break; }
            }
            Reconcile();
        }
        public void Stop()
        {
            CheckThread();requested=false;faulted=false;stopping=false; // Explicit Stop also retries a blocked release.
            StopProviders();Reconcile();
        }
        private void StopProviders()
        {
            if(stopping)return;
            stopping=true;State=faulted?TrackingState.Faulted:TrackingState.Stopping;
            foreach(var entry in providers.Values)
            {
                entry.Context.Disable();
                try { entry.Provider.Stop(); }
                catch(Exception error) { entry.Failed=true;entry.Error=error.Message;LastError=error.Message;faulted=true; }
            }
            Changed();
        }
        public void Pump(TrackingUpdatePhase phase)
        {
            CheckThread();
            if(State==TrackingState.Stopped&&!requested)return;
            double now=clock.NowSeconds;
            if(!Numeric.Finite(now)||now<lastNow||clock.Id!=core.ClockId)
            { clockBroken=true;faulted=true;requested=false;LastError="Runtime clock changed or regressed.";StopProviders(); }
            else lastNow=now;
            foreach(var entry in providers.Values)
            {
                try
                {
                    entry.Provider.Pump(phase); // Also advances asynchronous release while stopped/stopping.
                    if(requested&&!stopping&&!entry.Failed&&(entry.Provider.State==ProviderState.Faulted||entry.Provider.State==ProviderState.Stopped||entry.Provider.LastError!=null))
                        Fail(entry,entry.Provider.LastError??"Provider faulted.");
                }
                catch(Exception error) { Fail(entry,error.Message); }
            }
            Reconcile();
        }
        private void Fail(Entry entry,string error)
        {
            entry.Failed=true;entry.Error=error;entry.Context.Disable();
            try { entry.Provider.Stop(); } catch(Exception cleanup) { entry.Error+="; cleanup: "+cleanup.Message; }
            if(entry.Required) { requested=false;faulted=true;LastError=entry.Provider.Id+": "+entry.Error;StopProviders(); }
        }
        private void Reconcile()
        {
            if(stopping)
            {
                foreach(var entry in providers.Values)
                    if(entry.Provider.State!=ProviderState.Stopped&&entry.Provider.State!=ProviderState.Faulted)return;
                stopping=false;State=faulted?TrackingState.Faulted:TrackingState.Stopped;
                if(requested)StartProviders();
                return;
            }
            if(faulted){State=TrackingState.Faulted;return;}
            if(!requested){State=TrackingState.Stopped;return;}
            foreach(var entry in providers.Values)
                if(entry.Required&&(entry.Failed||entry.Provider.State!=ProviderState.Running)) { State=TrackingState.Starting;return; }
            State=TrackingState.Running;
        }
        public bool ShutdownComplete
        {
            get { foreach(var e in providers.Values)if(e.Provider.State!=ProviderState.Stopped&&e.Provider.State!=ProviderState.Faulted)return false;return !requested; }
        }
        public void ResetProviderReference(string providerId)
        {
            CheckThread();if(!providers.TryGetValue(providerId,out var entry))throw new ArgumentException("Unknown provider.");
            if(!requested||stopping)return;
            entry.Context.EndSession();
            try { entry.Provider.ResetReferenceFrame(); } catch(Exception error) { Fail(entry,error.Message); }
        }
        public bool TryGetProviderFrame(string providerId,out CoordinateFrame frame)
        {
            CheckThread();frame=null;
            if(!requested||stopping||!providers.TryGetValue(providerId,out var e)||e.Failed||e.Provider.State!=ProviderState.Running)return false;
            frame=e.Context.Frame;return frame!=null;
        }
        public void SetFrameCalibration(string id,CoordinateFrame from,CoordinateFrame to,RigidPose toFrom,double validFrom,double validUntil)
        {
            CheckThread();
            if(!CurrentFrame(from)||!CurrentFrame(to))throw new ArgumentException("Calibration must name current provider frames and epochs.");
            frames.Set(id,from,to,toFrom,validFrom,validUntil);Changed();
        }
        public void RemoveFrameCalibration(string id)
        { CheckThread();long revision=frames.Revision;frames.Remove(id);if(frames.Revision!=revision)Changed(); }
        private bool CurrentFrame(CoordinateFrame frame)
        {
            if(frame==null||!requested||stopping)return false;
            foreach(var e in providers.Values)if(!e.Failed&&e.Provider.State==ProviderState.Running&&frame.Equals(e.Context.Frame))return true;
            return false;
        }
        private bool Select(string objectId,ObservationRequirements requirements,out PoseObservation observation,
            out RigidPose frameFromLogical,out double uncertainty,out TrackingIssue issue)
        {
            observation=default;frameFromLogical=default;uncertainty=0;issue=TrackingIssue.SourceUnavailable;
            if(!requested||stopping||faulted||State!=TrackingState.Running)return false;
            if(!objects.TryGetValue(objectId,out var bindings)){issue=TrackingIssue.UnknownSource;return false;}
            foreach(var binding in bindings)
            {
                var e=providers[binding.ProviderId];
                if(e.Failed||e.Provider.State!=ProviderState.Running)continue;
                if(!core.TryGetLatest(binding.ProviderId,binding.ProviderObjectId,requirements.MaximumAgeSeconds,out var raw,out issue))continue;
                var map=e.Context.ClockMap;
                if(map==null||!ReferenceEquals(raw.Session,e.Context.Session)||!map.IsCurrent(clock.NowSeconds)) { issue=TrackingIssue.ClockMismatch;continue; }
                double u=map.UncertaintySeconds;
                if(u>requirements.MaximumClockUncertaintySeconds){issue=TrackingIssue.ClockMismatch;continue;}
                if(clock.NowSeconds-raw.CaptureSeconds+u>requirements.MaximumAgeSeconds){issue=TrackingIssue.Stale;continue;}
                observation=raw;uncertainty=u;frameFromLogical=raw.ReferenceFromObject*binding.ProviderBodyFromLogicalBody;
                issue=TrackingIssue.None;return true;
            }
            return false;
        }
        public bool TryGetPose(string objectId,ObservationRequirements requirements,out ResolvedTrackingPose pose,out TrackingIssue issue)
        {
            CheckThread();pose=default;
            if(!Select(objectId,requirements,out var raw,out var transform,out var uncertainty,out issue))return false;
            pose=new ResolvedTrackingPose(objectId,null,raw.Session.ReferenceFrame,transform,raw,null,Revision,uncertainty);return true;
        }
        public bool TryGetPoseInFrame(string objectId,CoordinateFrame frame,ObservationRequirements requirements,out ResolvedTrackingPose pose,out TrackingIssue issue)
        {
            CheckThread();pose=default;
            if(!Select(objectId,requirements,out var raw,out var transform,out var uncertainty,out issue))return false;
            if(!CurrentFrame(frame)||!frames.TryResolve(raw.Session.ReferenceFrame,frame,clock.NowSeconds,out var targetFromSource))
            { issue=TrackingIssue.FrameMismatch;return false; }
            pose=new ResolvedTrackingPose(objectId,null,frame,targetFromSource*transform,raw,null,Revision,uncertainty);return true;
        }
        public bool TryGetRelativePose(string objectId,string referenceObjectId,ObservationRequirements requirements,out ResolvedTrackingPose pose,out TrackingIssue issue)
        {
            CheckThread();pose=default;
            if(!Select(objectId,requirements,out var obj,out var sourceFromObject,out var u1,out issue)||
                !Select(referenceObjectId,requirements,out var reference,out var referenceFrameFromBody,out var u2,out issue))return false;
            if(Math.Abs(obj.CaptureSeconds-reference.CaptureSeconds)+u1+u2>requirements.MaximumSkewSeconds)
            { issue=TrackingIssue.TimeMismatch;return false; }
            if(!frames.TryResolve(obj.Session.ReferenceFrame,reference.Session.ReferenceFrame,clock.NowSeconds,out var referenceFrameFromSource))
            { issue=TrackingIssue.FrameMismatch;return false; }
            var result=referenceFrameFromBody.Inverse*referenceFrameFromSource*sourceFromObject;
            // Frame identifies the physical reference body's axes; both measured times remain in the result.
            var frame=new CoordinateFrame("body/"+referenceObjectId,reference.Session.ReferenceFrame.Epoch+"/"+reference.Session.SessionId);
            pose=new ResolvedTrackingPose(objectId,referenceObjectId,frame,result,obj,reference,Revision,u1+u2);return true;
        }
        public IReadOnlyList<ProviderStatus> GetProviderStatus()
        {
            CheckThread();var result=new List<ProviderStatus>();
            foreach(var e in providers.Values)result.Add(new ProviderStatus(e.Provider.Id,e.Provider.State,e.Error??e.Provider.LastError,e.Required));
            return result.AsReadOnly();
        }
        public void Dispose()
        {
            if(State==TrackingState.Disposed)return;CheckThread();
            if(!ShutdownComplete){Stop();if(!ShutdownComplete)throw new InvalidOperationException("Continue pumping until asynchronous shutdown completes before disposing.");}
            foreach(var entry in providers.Values)entry.Provider.Dispose();
            State=TrackingState.Disposed;
        }
    }
}
