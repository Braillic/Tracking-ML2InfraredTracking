using System;
using Braillic.Tracking;

namespace Braillic.Tracking.Application
{
    public enum TrackingPhase { LateUpdate, BeforeRender }
    public enum TrackingApplicationState { Stopped, Running, Paused, Faulted, Disposed }

    // Backend implementations own acquisition/transport; application code owns intent.


    /// <summary>Single-owner application lifecycle. Call on the backend's owning thread.</summary>
    public sealed class TrackingApplication : ITrackingReader, IDisposable
    {
        private readonly ITrackingReader reader;
        private readonly ITrackingApplicationBackend backend;
        private bool requested, suspended;
        public TrackingApplicationState State { get; private set; }
        public string LastError { get; private set; }

        public TrackingApplication(ITrackingReader reader, ITrackingApplicationBackend backend)
        {
            this.reader = reader ?? throw new ArgumentNullException(nameof(reader));
            this.backend = backend ?? throw new ArgumentNullException(nameof(backend));
        }

        public void Start()
        {
            ThrowIfDisposed();
            requested = true;
            if (suspended) { State = TrackingApplicationState.Paused; return; }
            if (State == TrackingApplicationState.Running) return;
            try { backend.Start(); LastError = null; State = TrackingApplicationState.Running; }
            catch (Exception error) { Fail(error); }
        }

        public void Stop()
        {
            if (State == TrackingApplicationState.Disposed) return;
            requested = false;
            StopBackend(TrackingApplicationState.Stopped);
        }

        public void SetSuspended(bool value)
        {
            ThrowIfDisposed();
            if (suspended == value) return;
            suspended = value;
            if (value) StopBackend(requested ? TrackingApplicationState.Paused : TrackingApplicationState.Stopped);
            else if (requested) Start();
        }

        public void Pump(TrackingPhase phase)
        {
            if (State != TrackingApplicationState.Running) return;
            try { backend.Pump(phase); }
            catch (Exception error) { Fail(error); }
        }

        public void ResetReferenceFrame()
        {
            ThrowIfDisposed();
            if (State != TrackingApplicationState.Running) return;
            try { backend.ResetReferenceFrame(); }
            catch (Exception error) { Fail(error); }
        }

        public bool TryGetLatest(string sourceId, string objectId, double maximumAgeSeconds,
            out PoseObservation observation, out TrackingIssue issue)
        {
            observation = default;
            issue = TrackingIssue.SourceUnavailable;
            return State == TrackingApplicationState.Running &&
                reader.TryGetLatest(sourceId, objectId, maximumAgeSeconds, out observation, out issue);
        }

        public bool TryGetRelative(string sourceId, string objectId, string referenceSourceId,
            string referenceId, double maximumAgeSeconds, double maximumCaptureSkewSeconds,
            out RelativeObservation observation, out TrackingIssue issue)
        {
            observation = default;
            issue = TrackingIssue.SourceUnavailable;
            return State == TrackingApplicationState.Running && reader.TryGetRelative(sourceId, objectId,
                referenceSourceId, referenceId, maximumAgeSeconds, maximumCaptureSkewSeconds, out observation, out issue);
        }

        private void StopBackend(TrackingApplicationState next)
        {
            // Withdraw application reads before backend cleanup, including reentrant callbacks.
            State = next;
            try { backend.Stop(); }
            catch (Exception error) { LastError = error.Message; State = TrackingApplicationState.Faulted; requested = false; }
        }

        private void Fail(Exception error)
        {
            requested = false;
            State = TrackingApplicationState.Faulted;
            LastError = error.Message;
            try { backend.Stop(); }
            catch (Exception cleanup) { LastError += "; cleanup: " + cleanup.Message; }
        }

        private void ThrowIfDisposed()
        { if (State == TrackingApplicationState.Disposed) throw new ObjectDisposedException(nameof(TrackingApplication)); }

        public void Dispose()
        {
            if (State == TrackingApplicationState.Disposed) return;
            Stop();
            State = TrackingApplicationState.Disposed;
            backend.Dispose();
        }
    }
}
