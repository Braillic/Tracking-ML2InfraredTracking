using System;
using System.Globalization;
using Braillic.Tracking.Runtime;
using Braillic.Tracking.Unity;

namespace Braillic.Tracking.Implementation.ML2
{
    /// <summary>Complete ML2 + remote estimator provider. The tested desktop protocol and
    /// capture-time pose calculation remain in the existing implementation components.</summary>
    public sealed class Ml2TrackingProvider : ITrackingProvider
    {
        private readonly string objectId,calibrationId;
        private readonly DepthSensorAPI sensor;
        private readonly ML2DepthRawStream capture;
        private readonly DepthFrameTcpServer sender;
        private readonly PoseEstimateTcpServer receiver;
        private TrackingProviderContext context;
        private TrackingSession session;
        private bool requested,disposed;
        private ulong wireSession;
        public string Id { get; }
        public ProviderState State { get; private set; }
        public string LastError => sensor.Lifecycle.LastError;

        public Ml2TrackingProvider(string id,string objectId,string calibrationId,DepthSensorAPI sensor,
            ML2DepthRawStream capture,DepthFrameTcpServer sender,PoseEstimateTcpServer receiver)
        {
            if(string.IsNullOrWhiteSpace(id)||string.IsNullOrWhiteSpace(objectId)||string.IsNullOrWhiteSpace(calibrationId))throw new ArgumentException("Provider identities required.");
            Id=id;this.objectId=objectId;this.calibrationId=calibrationId;
            this.sensor=sensor??throw new ArgumentNullException(nameof(sensor));
            this.capture=capture??throw new ArgumentNullException(nameof(capture));
            this.sender=sender??throw new ArgumentNullException(nameof(sender));
            this.receiver=receiver??throw new ArgumentNullException(nameof(receiver));
            if(sensor.streamVisualizer!=capture)throw new ArgumentException("Sensor output must reference this provider's capture adapter.");
        }
        public void Initialize(TrackingProviderContext value)
        {
            if(context!=null)throw new InvalidOperationException("Provider already initialized.");
            context=value;
            sender.SetApplicationManaged(true);receiver.SetApplicationManaged(sender);
            capture.BindTrackingSender(sender);capture.TrackingSubmissionEnabled=false;
            sensor.BindTrackingLifecycle(sender);
            receiver.ObservationAccepted+=OnAccepted;
            receiver.TrackingInvalidated+=OnInvalidated;
        }
        public void Start()
        {
            if(disposed)throw new ObjectDisposedException(nameof(Ml2TrackingProvider));
            if(requested)return;
            requested=true;State=ProviderState.Starting;
            try { sender.StartServer();receiver.StartServer();sensor.Lifecycle.Start();EnsureSession(); }
            catch { Stop();throw; }
        }
        public void Stop()
        {
            requested=false;capture.TrackingSubmissionEnabled=false;OnInvalidated();
            sensor.Lifecycle.Stop();
            try { receiver.StopServer(); } finally { sender.StopServer(); }
            State=sensor.Lifecycle.Released?ProviderState.Stopped:ProviderState.Stopping;
        }
        public void Pump(TrackingUpdatePhase phase)
        {
            // No sensor SDK work or teardown waits in before-render.
            if(phase==TrackingUpdatePhase.Update)sensor.Lifecycle.Pump();
            if(!requested)
            { State=sensor.Lifecycle.Released?ProviderState.Stopped:ProviderState.Stopping;return; }
            if(!sensor.isActiveAndEnabled||!capture.isActiveAndEnabled||!sender.isActiveAndEnabled||!receiver.isActiveAndEnabled||
                DepthFrameTcpServer.ActiveServer!=sender||PoseEstimateTcpServer.ActiveServer!=receiver)
                throw new InvalidOperationException("Provider components disabled or a second owner is using the tracking servers.");
            if(!sender.IsRunning||!receiver.IsRunning)throw new InvalidOperationException("Tracking transport failed: "+sender.Status+"; "+receiver.Status);
            if(sensor.Lifecycle.LastError!=null)throw new InvalidOperationException(sensor.Lifecycle.LastError);
            State=sensor.Lifecycle.State==ProviderState.Running?ProviderState.Running:ProviderState.Starting;
            capture.TrackingSubmissionEnabled=State==ProviderState.Running;
            EnsureSession();
            if(phase==TrackingUpdatePhase.Update||State!=ProviderState.Running)return;
            receiver.PumpApplication(phase==TrackingUpdatePhase.BeforeRender);
            if(!receiver.TryGetAcceptedObservation(out _)&&session!=null)context.MarkUnavailable(session,objectId);
        }
        public void ResetReferenceFrame() { sender.BeginCaptureEpoch();EnsureSession(); }
        private void EnsureSession()
        {
            if(!requested||sender.SessionId==0)return;
            if(session!=null&&ReferenceEquals(context.Session,session)&&wireSession==sender.SessionId)return;
            wireSession=sender.SessionId;
            session=context.BeginSession(wireSession.ToString(CultureInfo.InvariantCulture),
                new CoordinateFrame("ar-world/"+Id,Guid.NewGuid().ToString("N")),calibrationId,ClockMapping.Identity(context.Clock.Id));
        }
        private void OnAccepted(PoseEstimateTcpServer.AcceptedObservation pose)
        {
            if(!requested||State!=ProviderState.Running||pose.SessionId!=sender.SessionId)return;
            EnsureSession();
            var issue=context.Publish(session,objectId,pose.FrameId,pose.CaptureTime,pose.AppliedTime,
                TrackingCoordinates.FromUnity(pose.Position,pose.Rotation),pose.Confidence);
            if(issue!=TrackingIssue.None)context.MarkUnavailable(session,objectId);
        }
        private void OnInvalidated() { context?.EndSession();session=null;wireSession=0; }
        public void Dispose()
        {
            if(disposed)return;
            if(!sensor.Lifecycle.Released)throw new InvalidOperationException("Pump Stop to completion before disposing the ML2 provider.");
            Stop();receiver.ObservationAccepted-=OnAccepted;receiver.TrackingInvalidated-=OnInvalidated;disposed=true;
        }
    }
}
