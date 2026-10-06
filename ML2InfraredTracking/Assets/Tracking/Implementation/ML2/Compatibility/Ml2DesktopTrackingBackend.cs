using System;
using System.Globalization;
using Braillic.Tracking;
using Braillic.Tracking.Application;

namespace Braillic.Tracking.Unity
{
    /// <summary>In-process ML2 backend; owns the existing transport and submission gate.
    /// SDK permission/configuration stays with DepthSensorAPI. No second app or XR rig.</summary>
    public sealed class Ml2DesktopTrackingBackend : ITrackingApplicationBackend
    {
        private readonly TrackingCore core;
        private readonly DepthFrameTcpServer depth;
        private readonly PoseEstimateTcpServer poses;
        private readonly ML2DepthRawStream capture;
        private readonly string sourceId, objectId, calibrationId, frameId;
        private TrackingSession session;
        private ulong wireSession;
        private bool running, disposed;
        public TrackingIssue LastIssue { get; private set; }
        public event Action ObservationPublished;
        public event Action ReferenceInvalidated;

        public Ml2DesktopTrackingBackend(TrackingCore core, DepthFrameTcpServer depth,
            PoseEstimateTcpServer poses, ML2DepthRawStream capture,
            string sourceId, string objectId, string calibrationId)
        {
            this.core = core ?? throw new ArgumentNullException(nameof(core));
            this.depth = depth ?? throw new ArgumentNullException(nameof(depth));
            this.poses = poses ?? throw new ArgumentNullException(nameof(poses));
            this.capture = capture ?? throw new ArgumentNullException(nameof(capture));
            if (string.IsNullOrWhiteSpace(sourceId) || string.IsNullOrWhiteSpace(objectId) ||
                string.IsNullOrWhiteSpace(calibrationId)) throw new ArgumentException("Tracking identities are required.");
            this.sourceId = sourceId; this.objectId = objectId; this.calibrationId = calibrationId;
            frameId = "unity-world/" + core.ClockId;
            // Bind before any lifecycle callback may auto-start a competing owner.
            depth.SetApplicationManaged(true);
            poses.SetApplicationManaged(depth);
            capture.BindTrackingSender(depth);
            capture.TrackingSubmissionEnabled = false;
            poses.ObservationAccepted += OnAccepted;
            poses.TrackingInvalidated += OnInvalidated;
        }

        public void Start()
        {
            if (disposed) throw new ObjectDisposedException(nameof(Ml2DesktopTrackingBackend));
            if (running) return;
            running = true;
            try
            {
                depth.StartServer();
                poses.StartServer();
                EnsureSession();
                capture.TrackingSubmissionEnabled = true;
            }
            catch { Stop(); throw; }
        }

        public void Stop()
        {
            running = false;
            capture.TrackingSubmissionEnabled = false;
            CloseSession();
            try { poses.StopServer(); }
            finally { depth.StopServer(); }
        }

        public void Pump(TrackingPhase phase)
        {
            if (!running) return;
            if (!depth.isActiveAndEnabled || !poses.isActiveAndEnabled || !capture.isActiveAndEnabled ||
                DepthFrameTcpServer.ActiveServer != depth || PoseEstimateTcpServer.ActiveServer != poses)
                throw new InvalidOperationException("The configured ML2 tracking components are disabled or another server owns the ports.");
            if (!depth.IsRunning || !poses.IsRunning)
                throw new InvalidOperationException("A tracking transport stopped; restart tracking after checking its status.");
            EnsureSession();
            poses.PumpApplication(phase == TrackingPhase.BeforeRender);
            // Disconnect/loss is not a new measurement. Never refresh a held pose's timestamp.
            if (!poses.TryGetAcceptedObservation(out _))
            {
                if (session != null) core.MarkUnavailable(session, objectId);
                LastIssue = TrackingIssue.SourceUnavailable;
            }
        }

        public void ResetReferenceFrame()
        {
            depth.BeginCaptureEpoch(); // Invalidates pending old-session results in the existing protocol.
            EnsureSession();
        }

        private void EnsureSession()
        {
            if (!running || depth.SessionId == 0) return;
            if (session != null && wireSession == depth.SessionId) return;
            CloseSession();
            wireSession = depth.SessionId;
            session = core.OpenSession(sourceId, wireSession.ToString(CultureInfo.InvariantCulture), core.ClockId,
                new CoordinateFrame(frameId, Guid.NewGuid().ToString("N")), calibrationId);
        }

        private void OnAccepted(PoseEstimateTcpServer.AcceptedObservation observation)
        {
            if (!running || observation.SessionId != depth.SessionId) return;
            EnsureSession();
            if (session == null) return;
            LastIssue = core.Publish(session, objectId, observation.FrameId, observation.CaptureTime,
                observation.AppliedTime, TrackingCoordinates.FromUnity(observation.Position, observation.Rotation),
                observation.Confidence);
            if (LastIssue == TrackingIssue.None) ObservationPublished?.Invoke();
            else core.MarkUnavailable(session, objectId);
        }

        private void OnInvalidated() { CloseSession(); }
        private void CloseSession()
        { if (session != null) core.CloseSession(session); session = null; wireSession = 0; ReferenceInvalidated?.Invoke(); }

        public void Dispose()
        {
            if (disposed) return;
            try { Stop(); }
            finally
            {
                poses.ObservationAccepted -= OnAccepted;
                poses.TrackingInvalidated -= OnInvalidated;
                disposed = true;
            }
        }
    }
}
