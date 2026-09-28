using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using TMPro;
using UnityEngine;

/// Listens for desktop pose packets (PC → Magic Leap) and applies the newest
/// valid estimate to a tool transform. Companion to <see cref="DepthFrameTcpServer"/>
/// (ML → PC depth); uses a separate port so depth send stays write-only.
/// Base wire format (little-endian, 80 bytes); v4 appends uint64 session ID (88 total):
///   magic "ML2P"
///   ushort version (=3 or 4), ushort packet_size (=80 or 88)
///   ulong  frame_id   (matches depth header frame number)
///   uint   ok         (0 = reject, nonzero = apply)
///   float  confidence
///   float  px, py, pz           Unity world metres
///   float  qx, qy, qz, qw       Unity world quaternion
///   float  detect_ms  PC marker detection duration (own clock, not synced to ML2)
///   float  pnp_ms     PC PnP solve duration
///   float  send_ms    PC world-pose preparation duration (serialization/send are in the return-leg estimate)
///   double pc_recv_ml2  PC depth-frame-received time, translated into ML2's clock domain
///   double pc_send_ml2  PC pose-about-to-send time, translated into ML2's clock domain
///
/// Also handles a same-size clock-sync request/reply pair on this socket (magic "ML2S" /
/// "ML2R", see pose_packet.py:sync_clock_offset) so the desktop can translate its own
/// perf_counter() timestamps above into this clock's domain, letting the latency log
/// split the round trip into its two network legs instead of one lumped bucket.
public sealed class PoseEstimateTcpServer : MonoBehaviour
{
    public const int ProtocolVersion = 3;
    public const int PacketSize = 80;
    public static PoseEstimateTcpServer ActiveServer { get; private set; }

    [Header("TCP server (Magic Leap listens; PC connects and sends poses)")]
    [SerializeField] private bool startAutomatically = true;
    [Tooltip("Use 0.0.0.0 for Wi-Fi and adb forwarding. Use 127.0.0.1 for adb-only access.")]
    [SerializeField] private string bindAddress = "0.0.0.0";
    [SerializeField, Range(1, 65535)] private int port = 50778;

    [Header("Tool")]
    [Tooltip("Optional. If empty, a hidden TrackedTool root is prepared on the main thread before tracking.")]
    [SerializeField] private Transform trackedTool;
    [SerializeField, Min(0f)] private float minConfidence;
    [Tooltip("Ignore duplicate or older frame IDs. Keep enabled for live tracking.")]
    [SerializeField] private bool dropStaleFrames = true;
    [Tooltip("Maximum submit-to-apply age on ML2's own clock. Independent of capture cadence and tracking-loss hold.")]
    [SerializeField, Min(0.001f)] private float maxPoseAgeSeconds = 0.15f;

    [Header("Marker visualization (prepared hidden before tracking)")]
    [Tooltip("Spawn one sphere per model point (object-frame metres). The mounting surface is local Z = 0. Defaults match desktop TEST_MARKER_COORDS.")]
    [SerializeField] private bool createMarkerSpheresOnFirstPose = true;
    [SerializeField] private Vector3[] markerLocalPositions =
    {
        new Vector3(-0.0466f, 0f, 0.0206f), // Left
        new Vector3(0.0444f, 0f, 0.0206f),  // Right
        new Vector3(0.0095f, 0.0623f, 0.0206f), // Up
        new Vector3(-0.007f, -0.0368f, 0.0206f), // Bottom
    };
    [SerializeField, Min(0.001f)] private float markerSphereRadius = 0.004f;
    [SerializeField] private Color markerSphereColor = new Color(1f, 0.2f, 0.2f, 0.9f);

    [Header("Status display (optional)")]
    [SerializeField] private TMP_Text statusText;
    [SerializeField] private bool showOnGuiOverlay = false;

    [Header("Apply")]
    [Tooltip("Consume a fresh pose arriving after LateUpdate immediately before rendering. Main-thread only; logging is deferred to Update.")]
    [SerializeField] private bool applyBeforeRender = true;
    [Tooltip("1 = snap. Smaller fractions smooth each new pose but add lag; 0 holds the previous transform.")]
    [SerializeField, Range(0f, 1f)] private float rotationFollow = 1f;
    [SerializeField, Range(0f, 1f)] private float positionFollow = 1f;
    [SerializeField, Min(0f)] private float hideAfterNoPoseSeconds = 0.65f;

    [Header("Latency (Debug.Log)")]
    [Tooltip("Log throttled [ML2LAT] lines on the main thread when a pose is applied. " +
             "Logging every frame can add measurable overhead — keep Every N >= 10.")]
    [SerializeField] private bool logLatency = true;
    [SerializeField, Min(1)] private int latencyLogEveryN = 15;

    private readonly object _poseLock = new object();
    private bool _hasPending;
    private ulong _pendingFrameId, _pendingSessionId;
    private bool _pendingOk;
    private float _pendingConfidence;
    private Vector3 _pendingPosition;
    private Quaternion _pendingRotation;
    private double _pendingReceivedRealtime;
    private float _pendingDetectMs;
    private float _pendingPnpMs;
    private float _pendingSendMs;
    private double _pendingPcRecvMl2;
    private double _pendingPcSendMl2;
    private int _updateTickCounter;
    private double _lastFpsSampleTime;
    private float _measuredUpdateFps;
    private int _latencyLogCounter;

    private Thread _serverThread;
    private TcpListener _listener;
    private TcpClient _client;
    private volatile bool _running;
    private volatile bool _clientConnected;
    private volatile string _remoteEndpoint;
    private volatile string _status = "Pose stream stopped";
    private ulong _lastAppliedFrameId;
    private bool _hasAppliedFrame;
    private long _stalePoseCount;
    private long _rejectedPoseCount, _wrongSessionCount;
    private double _nextTimingSummary;
    private ulong _timingSessionId;
    private double _lastApplyRealtime, _lastCaptureRealtime = double.NaN;
    private bool _awaitingBeforeRender;
    private readonly TimingWindow _submitToApply = new TimingWindow();
    private readonly TimingWindow _captureToApply = new TimingWindow();
    private readonly TimingWindow _captureToSubmit = new TimingWindow();
    private readonly TimingWindow _pollDuration = new TimingWindow();
    private readonly TimingWindow _readyToSubmit = new TimingWindow();
    private readonly TimingWindow _applyToBeforeRender = new TimingWindow();
    private long _receivedCount;
    private long _appliedCount;
    private double _nextStatusUpdate;
    private bool _markerVisualCreated;
    private double _lastAcceptedPoseTime = double.NegativeInfinity;

    private bool _visualPrepared;
    private string _displayedStatus;
    private double _lastUpdateRealtime;
    private long _lateUpdateApplied, _beforeRenderApplied, _diagnosticOverwritten;
    private readonly TimingWindow _applyWait = new TimingWindow();
    private readonly TimingWindow _applyWork = new TimingWindow();
    private readonly TimingWindow _captureToReady = new TimingWindow();
    private readonly TimingWindow _captureToBeforeRender = new TimingWindow();
    private readonly TimingWindow _updateInterval = new TimingWindow();
    private readonly TimingWindow _rawCopy = new TimingWindow();
    private readonly AppliedPoseSample[] _appliedSamples = new AppliedPoseSample[32];
    private int _sampleRead, _sampleCount;

    private struct AppliedPoseSample
    {
        public ulong FrameId, SessionId;
        public double Received, ApplyStarted, Applied;
        public float DetectMs, PnpMs, WorldMs;
        public double PcReceive, PcSend;
        public bool BeforeRender;
        public DepthFrameTcpServer.FrameTimingSnapshot Frame;
    }

    // Main-thread snapshot of the raw validated observation, independent of visual smoothing.
    public struct AcceptedObservation
    {
        public ulong FrameId, SessionId;
        public double CaptureTime, AppliedTime;
        public Vector3 Position;
        public Quaternion Rotation;
        public float Confidence;
    }
    private AcceptedObservation _latestObservation;

    public bool TryGetAcceptedObservation(out AcceptedObservation observation)
    {
        observation = _latestObservation;
        var depth = DepthFrameTcpServer.ActiveServer;
        double age = Time.realtimeSinceStartupAsDouble - observation.CaptureTime;
        return isActiveAndEnabled && _clientConnected && _hasAppliedFrame && depth != null
            && observation.SessionId != 0 && observation.SessionId == depth.SessionId
            && !double.IsNaN(age) && !double.IsInfinity(age) && age >= 0 && age <= maxPoseAgeSeconds;
    }

    public int Port => port;
    public string Status => _status;

    private void OnEnable()
    {
        if (ActiveServer != null && ActiveServer != this)
        {
            Debug.LogError($"[ML2PoseTCP] Duplicate server on '{name}' disabled. " +
                           $"The active server is on '{ActiveServer.name}'.");
            enabled = false;
            return;
        }

        ActiveServer = this;
        Application.onBeforeRender += ApplyAndMeasureBeforeRender;
        if (startAutomatically)
            StartServer();
    }

    private void Update()
    {
        PrepareTrackedToolVisual();
        double updateNow = Time.realtimeSinceStartupAsDouble;
        if (logLatency && _lastUpdateRealtime > 0.0)
            _updateInterval.Add((updateNow - _lastUpdateRealtime) * 1000.0);
        _lastUpdateRealtime = updateNow;
        FlushAppliedSamples();
        _updateTickCounter++;
        double nowUnscaled = Time.unscaledTimeAsDouble;
        if (_lastFpsSampleTime <= 0.0)
            _lastFpsSampleTime = nowUnscaled;
        double fpsWindow = nowUnscaled - _lastFpsSampleTime;
        if (fpsWindow >= 1.0)
        {
            _measuredUpdateFps = (float)(_updateTickCounter / fpsWindow);
            _updateTickCounter = 0;
            _lastFpsSampleTime = nowUnscaled;
        }
        LogTimingSummary();

        if (Time.unscaledTimeAsDouble >= _nextStatusUpdate)
        {
            _nextStatusUpdate = Time.unscaledTimeAsDouble + 0.25;
            long received = Interlocked.Read(ref _receivedCount);
            long applied = Interlocked.Read(ref _appliedCount);
            if (_clientConnected)
            {
                _status = received == 0
                    ? $"PC connected: {_remoteEndpoint}\nWaiting for pose packets..."
                    : $"PC connected: {_remoteEndpoint}\nposes recv {received} | applied {applied} | stale {_stalePoseCount}";
            }
        }

        if (statusText != null && _displayedStatus != _status)
        {
            statusText.text = _status;
            _displayedStatus = _status;
        }
    }

    private void LateUpdate()
    {
        PrepareTrackedToolVisual();
        TryApplyPendingPose();
        HideTrackedToolIfTimedOut();
    }

    private void PrepareTrackedToolVisual()
    {
        if (_visualPrepared && trackedTool != null) return;
        // Instantiate/material setup is never allowed inside onBeforeRender.
        EnsureTrackedToolVisual();
        _visualPrepared = trackedTool != null;
        if (_visualPrepared && !_hasAppliedFrame) trackedTool.gameObject.SetActive(false);
    }

    private void OnGUI()
    {
        if (!showOnGuiOverlay)
            return;

        GUI.Box(new Rect(12, 78, 520, 58), _status);
    }

    private void OnDisable()
    {
        Application.onBeforeRender -= ApplyAndMeasureBeforeRender;
        StopServer();
        if (ActiveServer == this)
            ActiveServer = null;
    }

    private void OnDestroy()
    {
        StopServer();
    }

    public void StartServer()
    {
        if (_running)
            return;

        _running = true;
        _clientConnected = false;
        Interlocked.Exchange(ref _receivedCount, 0);
        Interlocked.Exchange(ref _appliedCount, 0);
        _lastAppliedFrameId = 0;
        _hasAppliedFrame = false;
        _stalePoseCount = 0;
        _lastAcceptedPoseTime = double.NegativeInfinity;
        _lastUpdateRealtime = 0.0;
        _lateUpdateApplied = _beforeRenderApplied = _diagnosticOverwritten = 0;
        ResetTimingWindows();
        lock (_poseLock) _hasPending = false;
        _status = $"Starting pose server on {bindAddress}:{port}...";
        _serverThread = new Thread(ServerLoop)
        {
            IsBackground = true,
            Name = "ML2 Pose TCP Server"
        };
        _serverThread.Start();
    }

    // Called on the main thread when the sensor/XR tracking origin changes.
    public void InvalidateTrackingSession()
    {
        lock (_poseLock) _hasPending = false;
        _hasAppliedFrame = false;
        _lastAppliedFrameId = 0;
        _lastAcceptedPoseTime = double.NegativeInfinity;
        _lastCaptureRealtime = double.NaN;
        _awaitingBeforeRender = false;
        ResetTimingWindows();
        if (trackedTool != null) trackedTool.gameObject.SetActive(false);
    }

    public void StopServer()
    {
        if (!_running && _serverThread == null)
            return;

        _running = false;
        _clientConnected = false;

        try { _client?.Close(); } catch { /* ignore */ }
        try { _listener?.Stop(); } catch { /* ignore */ }

        if (_serverThread != null && _serverThread.IsAlive)
            _serverThread.Join(1000);

        _client = null;
        _listener = null;
        _serverThread = null;
        lock (_poseLock) _hasPending = false;
        _lastAcceptedPoseTime = double.NegativeInfinity;
        if (trackedTool != null) trackedTool.gameObject.SetActive(false);
        _awaitingBeforeRender = false;
        _sampleCount = 0; _sampleRead = 0;
        _status = "Pose stream stopped";
    }

    public void SubmitPose(ulong frameId, bool ok, float confidence, in Vector3 position,
        in Quaternion rotation, float detectMs, float pnpMs, float sendMs,
        double pcRecvMl2, double pcSendMl2, ulong sessionId = 0)
    {
        // realtimeSinceStartup is safe to read off the main thread; avoid Debug.Log here.
        double receivedRealtime = Time.realtimeSinceStartupAsDouble;
        if (sessionId == 0) sessionId = DepthFrameTcpServer.ActiveServer?.SessionId ?? 0;
        lock (_poseLock)
        {
            if (dropStaleFrames && _hasPending && sessionId == _pendingSessionId && frameId <= _pendingFrameId)
                return;
            _pendingFrameId = frameId;
            _pendingSessionId = sessionId;
            _pendingOk = ok;
            _pendingConfidence = confidence;
            _pendingPosition = position;
            _pendingRotation = rotation;
            _pendingReceivedRealtime = receivedRealtime;
            _pendingDetectMs = detectMs;
            _pendingPnpMs = pnpMs;
            _pendingSendMs = sendMs;
            _pendingPcRecvMl2 = pcRecvMl2;
            _pendingPcSendMl2 = pcSendMl2;
            _hasPending = true;
        }
    }

    private void TryApplyPendingPose(bool beforeRender = false)
    {
        if (!_visualPrepared || trackedTool == null) return;
        ulong frameId, sessionId;
        bool ok;
        float confidence;
        Vector3 position;
        Quaternion rotation;
        double receivedRealtime, pcRecvMl2, pcSendMl2;
        float detectMs, pnpMs, sendMs;
        var depthServer = DepthFrameTcpServer.ActiveServer;
        DepthFrameTcpServer.FrameTimingSnapshot frame = default;
        bool knownFrame = false, entered = false;
        try
        {
            if (beforeRender) entered = Monitor.TryEnter(_poseLock);
            else Monitor.Enter(_poseLock, ref entered);
            if (!entered || !_hasPending) return;
            if (depthServer != null)
                knownFrame = depthServer.TryGetFrameTimingSnapshot(_pendingFrameId, out frame, beforeRender);
            // Leave the mailbox intact if the render path cannot get its timing
            // snapshot immediately. LateUpdate remains the validation fallback.
            if (beforeRender && !knownFrame) return;
            frameId = _pendingFrameId; sessionId = _pendingSessionId;
            ok = _pendingOk; confidence = _pendingConfidence;
            position = _pendingPosition; rotation = _pendingRotation;
            receivedRealtime = _pendingReceivedRealtime;
            detectMs = _pendingDetectMs; pnpMs = _pendingPnpMs; sendMs = _pendingSendMs;
            pcRecvMl2 = _pendingPcRecvMl2; pcSendMl2 = _pendingPcSendMl2;
            _hasPending = false;
        }
        finally { if (entered) Monitor.Exit(_poseLock); }

        double applyStarted = Time.realtimeSinceStartupAsDouble;
        // Recheck session at consumption, not only at socket receipt. An origin
        // reset can occur between receipt validation and mailbox publication.
        if (depthServer == null || sessionId == 0 || sessionId != depthServer.SessionId)
        { Interlocked.Increment(ref _wrongSessionCount); return; }
        if (!ok || float.IsNaN(confidence) || float.IsInfinity(confidence) || confidence < minConfidence
            || !IsFinitePose(position, rotation)) return;
        if (!IsFreshPose(frameId, _hasAppliedFrame, _lastAppliedFrameId, dropStaleFrames,
                         knownFrame, frame.Submit, applyStarted, maxPoseAgeSeconds))
        { _stalePoseCount++; return; }
        rotation = rotation.normalized;
        Vector3 appliedPosition = positionFollow >= 0.999f ? position :
            Vector3.Lerp(trackedTool.position, position, positionFollow);
        Quaternion appliedRotation = rotationFollow >= 0.999f ? rotation :
            Quaternion.Slerp(trackedTool.rotation, rotation, rotationFollow);
        // Set the pose before enabling a hidden model: OnEnable sees the new pose.
        trackedTool.SetPositionAndRotation(appliedPosition, appliedRotation);
        if (!trackedTool.gameObject.activeSelf) trackedTool.gameObject.SetActive(true);
        double applied = Time.realtimeSinceStartupAsDouble;
        _latestObservation = new AcceptedObservation { FrameId = frameId, SessionId = sessionId,
            CaptureTime = frame.Capture.CaptureRealtime, AppliedTime = applied,
            Position = position, Rotation = rotation, Confidence = confidence };
        _lastAppliedFrameId = frameId; _hasAppliedFrame = true;
        _lastAcceptedPoseTime = frame.Submit;
        _lastApplyRealtime = applied; _lastCaptureRealtime = frame.Capture.CaptureRealtime;
        Interlocked.Increment(ref _appliedCount);
        if (beforeRender) _beforeRenderApplied++; else _lateUpdateApplied++;
        _awaitingBeforeRender = logLatency;
        if (logLatency) QueueAppliedSample(new AppliedPoseSample
        {
            FrameId = frameId, SessionId = sessionId, Received = receivedRealtime,
            ApplyStarted = applyStarted, Applied = applied, Frame = frame, BeforeRender = beforeRender,
            DetectMs = detectMs, PnpMs = pnpMs, WorldMs = sendMs, PcReceive = pcRecvMl2, PcSend = pcSendMl2
        });
    }

    private void QueueAppliedSample(AppliedPoseSample sample)
    {
        if (_sampleCount == _appliedSamples.Length)
        {
            _sampleRead = (_sampleRead + 1) % _appliedSamples.Length;
            _sampleCount--; _diagnosticOverwritten++;
        }
        _appliedSamples[(_sampleRead + _sampleCount) % _appliedSamples.Length] = sample;
        _sampleCount++;
    }

    private void FlushAppliedSamples()
    {
        // Main-thread Update only. Log formatting is deliberately outside render callbacks.
        var depth = DepthFrameTcpServer.ActiveServer;
        while (_sampleCount > 0)
        {
            var sample = _appliedSamples[_sampleRead];
            _sampleRead = (_sampleRead + 1) % _appliedSamples.Length; _sampleCount--;
            if (depth == null || sample.SessionId != depth.SessionId || !logLatency) continue;
            var timing = sample.Frame.Capture;
            _submitToApply.Add((sample.Applied - sample.Frame.Submit) * 1000.0);
            _captureToApply.Add((sample.Applied - timing.CaptureRealtime) * 1000.0);
            _captureToSubmit.Add((sample.Frame.Submit - timing.CaptureRealtime) * 1000.0);
            _captureToReady.Add((timing.FrameReadyRealtime - timing.CaptureRealtime) * 1000.0);
            _pollDuration.Add((timing.FrameReadyRealtime - timing.PollStartRealtime) * 1000.0);
            _readyToSubmit.Add((sample.Frame.Submit - timing.FrameReadyRealtime) * 1000.0);
            _rawCopy.Add((timing.RawCopyDoneRealtime - timing.RawCopyStartRealtime) * 1000.0);
            _applyWait.Add((sample.ApplyStarted - sample.Received) * 1000.0);
            _applyWork.Add((sample.Applied - sample.ApplyStarted) * 1000.0);
            MaybeLogLatency(sample);
        }
    }

    internal static bool IsFreshPose(ulong frameId, bool hasApplied, ulong lastApplied,
        bool rejectOldIds, bool knownFrame, double submit, double now, double maxAge)
    {
        double age = now - submit;
        return (!rejectOldIds || !hasApplied || frameId > lastApplied)
            && knownFrame && !double.IsNaN(age) && !double.IsInfinity(age)
            && maxAge > 0.0 && age >= 0.0 && age <= maxAge;
    }

    private static bool IsFinitePose(Vector3 position, Quaternion rotation)
    {
        float norm = Quaternion.Dot(rotation, rotation);
        return !float.IsNaN(position.x) && !float.IsInfinity(position.x)
            && !float.IsNaN(position.y) && !float.IsInfinity(position.y)
            && !float.IsNaN(position.z) && !float.IsInfinity(position.z)
            && !float.IsNaN(norm) && !float.IsInfinity(norm) && norm > 1e-8f;
    }

    /// detectMs/pnpMs/sendMs are PC-side stage durations from its own perf_counter
    /// (see pose_packet.py); pure durations need no clock sync. pcRecvMl2/pcSendMl2
    /// are PC perf_counter timestamps already translated into this clock's domain via
    /// the ML2S/ML2R sync handshake (see HandleSyncRequest), so they can be diffed
    /// directly against ML2's own sendDone/receivedRealtime to isolate each network leg.
    /// The split's accuracy is bounded by the sync handshake's assumption of symmetric
    /// Wi-Fi latency (see the "Clock sync: ... sync_rtt=" line the desktop prints).

    private void MaybeLogLatency(in AppliedPoseSample sample)
    {
        _latencyLogCounter++;
        if (_latencyLogCounter % Math.Max(1, latencyLogEveryN) != 0) return;
        var frame = sample.Frame;
        double total = (sample.Applied - frame.Submit) * 1000.0;
        double wait = (sample.ApplyStarted - sample.Received) * 1000.0;
        double work = (sample.Applied - sample.ApplyStarted) * 1000.0;
        string phase = sample.BeforeRender ? "BeforeRender" : "LateUpdate";
        if (!frame.HasSend)
        {
            Debug.Log($"[ML2LAT] frame={sample.FrameId} phase={phase} submit_to_apply={total:F1}ms " +
                $"apply_wait={wait:F1}ms apply_work={work:F2}ms (send time missing)");
            return;
        }
        double queue = (frame.SendDone - frame.Submit) * 1000.0;
        double leg1 = (sample.PcReceive - frame.SendDone) * 1000.0;
        double leg2 = (sample.Received - sample.PcSend) * 1000.0;
        double remaining = total - (queue + leg1 + sample.DetectMs + sample.PnpMs +
            sample.WorldMs + leg2 + wait + work);
        var capture = frame.Capture;
        Debug.Log($"[ML2LAT] frame={sample.FrameId} session={sample.SessionId} " +
            $"pipeline={DepthFrameTcpServer.PipelineLabel(frame.Pipeline)} phase={phase} " +
            $"submit_to_apply={total:F1}ms capture_to_apply={(sample.Applied-capture.CaptureRealtime)*1000.0:F1}ms " +
            $"(ml_prepare+queue+tcp_write={queue:F1}ms | leg1_estimate(ML2->PC)={leg1:F1}ms | " +
            $"detect={sample.DetectMs:F1}ms | pnp={sample.PnpMs:F1}ms | pack={sample.WorldMs:F1}ms | " +
            $"leg2_estimate(PC->ML2)={leg2:F1}ms | apply_wait={wait:F1}ms | apply_work={work:F2}ms | " +
            $"unaccounted={remaining:F1}ms | update_fps={_measuredUpdateFps:F1}) " +
            $"capture_to_ready={(capture.FrameReadyRealtime-capture.CaptureRealtime)*1000.0:F1}ms " +
            $"ready_to_submit={(frame.Submit-capture.FrameReadyRealtime)*1000.0:F1}ms");
    }

    private void ApplyAndMeasureBeforeRender()
    {
        if (!isActiveAndEnabled) return;
        if (applyBeforeRender) TryApplyPendingPose(true);
        HideTrackedToolIfTimedOut();
        if (!logLatency || !_awaitingBeforeRender) return;
        double now = Time.realtimeSinceStartupAsDouble;
        _applyToBeforeRender.Add((now - _lastApplyRealtime) * 1000.0);
        _captureToBeforeRender.Add((now - _lastCaptureRealtime) * 1000.0);
        _awaitingBeforeRender = false;
    }

    private void ResetTimingWindows()
    {
        _sampleRead = _sampleCount = 0;
        _submitToApply.Clear(); _captureToApply.Clear(); _captureToSubmit.Clear();
        _pollDuration.Clear(); _readyToSubmit.Clear(); _applyToBeforeRender.Clear();
        _applyWait.Clear(); _applyWork.Clear(); _captureToReady.Clear();
        _captureToBeforeRender.Clear(); _updateInterval.Clear(); _rawCopy.Clear();
        _timingSessionId = DepthFrameTcpServer.ActiveServer?.SessionId ?? 0;
    }

    private void LogTimingSummary()
    {
        double now = Time.realtimeSinceStartupAsDouble;
        if (!logLatency || now < _nextTimingSummary) return;
        _nextTimingSummary = now + 2.0;
        var depth = DepthFrameTcpServer.ActiveServer;
        if (depth != null && _timingSessionId != depth.SessionId)
        {
            _timingSessionId = depth.SessionId;
            ResetTimingWindows();
        }
        string displayedAge = trackedTool != null && trackedTool.gameObject.activeSelf
            ? ((now - _lastAcceptedPoseTime) * 1000.0).ToString("F1") : "hidden";
        Debug.Log($"[ML2Timing] session={depth?.SessionId} recv={Interlocked.Read(ref _receivedCount)} " +
            $"applied={Interlocked.Read(ref _appliedCount)} rejected={Interlocked.Read(ref _rejectedPoseCount)} " +
            $"stale={_stalePoseCount} wrong_session={Interlocked.Read(ref _wrongSessionCount)} " +
            $"sender_overwritten={depth?.OverwrittenFrames} rate_dropped={depth?.RateDroppedFrames} " +
            $"displayed_submit_age_ms={displayedAge} displayed_capture_age_ms=" +
            (displayedAge == "hidden" ? "hidden" : double.IsNaN(_lastCaptureRealtime)
                ? "unavailable" : ((now - _lastCaptureRealtime) * 1000.0).ToString("F1")) + " " +
            $"submit_to_apply={_submitToApply.Summary()} capture_to_apply={_captureToApply.Summary()} " +
            $"capture_to_submit={_captureToSubmit.Summary()} sdk_poll={_pollDuration.Summary()} " +
            $"capture_to_ready={_captureToReady.Summary()} ready_to_submit={_readyToSubmit.Summary()} raw_copy={_rawCopy.Summary()} " +
            $"apply_wait={_applyWait.Summary()} apply_work={_applyWork.Summary()} " +
            $"apply_to_before_render={_applyToBeforeRender.Summary()} capture_to_before_render={_captureToBeforeRender.Summary()} " +
            $"update_interval={_updateInterval.Summary()} applied_late={_lateUpdateApplied} applied_before_render={_beforeRenderApplied} " +
            $"diagnostic_overwritten={_diagnosticOverwritten} before_render_enabled={applyBeforeRender} " +
            "presentation=unmeasured (rolling ms p50/p95/p99; per-series counts, at most 256)");
    }

    private void HideTrackedToolIfTimedOut()
    {
        if (hideAfterNoPoseSeconds <= 0f || trackedTool == null || !trackedTool.gameObject.activeSelf)
            return;

        if (Time.realtimeSinceStartupAsDouble - _lastAcceptedPoseTime > hideAfterNoPoseSeconds)
            trackedTool.gameObject.SetActive(false);
    }


    /// Creates a TrackedTool root (if needed) and child spheres at each model marker
    /// outside the render callback. Sphere local positions are the 3D constellation
    /// points used by PnP — once the root is posed, they sit on the physical markers.

    private void EnsureTrackedToolVisual()
    {
        if (trackedTool == null)
        {
            var root = new GameObject("TrackedTool");
            trackedTool = root.transform;
            trackedTool.SetParent(transform, false);
            _markerVisualCreated = false;
        }

        if (!createMarkerSpheresOnFirstPose || _markerVisualCreated)
            return;

        if (markerLocalPositions == null || markerLocalPositions.Length == 0)
        {
            Debug.LogWarning("[ML2PoseTCP] No markerLocalPositions configured; skipping sphere visual.");
            _markerVisualCreated = true;
            return;
        }

        Material markerMaterial = CreateMarkerMaterial(markerSphereColor);
        for (int i = 0; i < markerLocalPositions.Length; i++)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.name = $"Marker_{i}";
            sphere.transform.SetParent(trackedTool, false);
            sphere.transform.localPosition = markerLocalPositions[i];
            sphere.transform.localRotation = Quaternion.identity;
            float diameter = markerSphereRadius * 2f;
            sphere.transform.localScale = new Vector3(diameter, diameter, diameter);

            Collider collider = sphere.GetComponent<Collider>();
            if (collider != null)
                Destroy(collider);

            Renderer renderer = sphere.GetComponent<Renderer>();
            if (renderer != null)
                renderer.sharedMaterial = markerMaterial;
        }

        _markerVisualCreated = true;
        // Debug.Log($"[ML2PoseTCP] Created {markerLocalPositions.Length} marker spheres under '{trackedTool.name}'.");
    }

    private static Material CreateMarkerMaterial(Color color)
    {
        // Prefer cheap unlit shaders — Lit + CreatePrimitive defaults are heavier than needed.
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit")
                        ?? Shader.Find("Unlit/Color")
                        ?? Shader.Find("Sprites/Default")
                        ?? Shader.Find("Universal Render Pipeline/Lit")
                        ?? Shader.Find("Standard");
        var material = new Material(shader);
        if (material.HasProperty("_BaseColor"))
            material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color"))
            material.SetColor("_Color", color);
        return material;
    }

    private void ServerLoop()
    {
        try
        {
            IPAddress address = bindAddress == "0.0.0.0"
                ? IPAddress.Any
                : IPAddress.Parse(bindAddress);

            _listener = new TcpListener(address, port);
            _listener.Start(1);
            _status = $"Pose server listening on {bindAddress}:{port}";

            while (_running)
            {
                try
                {
                    TcpClient client = _listener.AcceptTcpClient();
                    if (!_running)
                    {
                        client.Close();
                        break;
                    }

                    _client = client;
                    client.NoDelay = true;
                    client.ReceiveBufferSize = 64 * 1024;
                    string remote = client.Client.RemoteEndPoint?.ToString() ?? "PC";
                    _remoteEndpoint = remote;
                    _clientConnected = true;
                    _status = $"PC connected: {remote}\nWaiting for pose packets...";
                    ReceivePoses(client);
                }
                catch (SocketException) when (!_running)
                {
                    break;
                }
                catch (Exception exception)
                {
                    if (_running)
                        _status = $"Pose client disconnected ({exception.GetType().Name})\n" +
                                  $"Pose server listening on {bindAddress}:{port}";
                }
                finally
                {
                    _clientConnected = false;
                    try { _client?.Close(); } catch { /* ignore */ }
                    _client = null;
                    if (_running)
                        _status = $"Pose server listening on {bindAddress}:{port}";
                }
            }
        }
        catch (Exception exception)
        {
            _status = $"Pose server error: {exception.Message}";
        }
    }

    private void ReceivePoses(TcpClient client)
    {
        using NetworkStream stream = client.GetStream();
        byte[] packet = new byte[PacketSize];
        byte[] sessionBytes = new byte[8];

        while (_running && client.Connected)
        {
            if (!ReadExact(stream, packet, PacketSize))
                return;

            if (IsSyncRequest(packet))
            {
                HandleSyncRequest(stream, packet);
                continue;
            }

            if (!TryParsePacket(packet, out ulong frameId, out bool ok, out float confidence,
                    out Vector3 position, out Quaternion rotation,
                    out float detectMs, out float pnpMs, out float sendMs,
                    out double pcRecvMl2, out double pcSendMl2))
            {
                throw new InvalidDataException("Invalid ML2P pose packet");
            }

            ulong sessionId = 0;
            if (BitConverter.ToUInt16(packet, 4) == 4)
            {
                if (!ReadExact(stream, sessionBytes, 8)) return;
                sessionId = BitConverter.ToUInt64(sessionBytes, 0);
            }
            var depth = DepthFrameTcpServer.ActiveServer;
            if (depth == null || sessionId == 0 || sessionId != depth.SessionId)
            { Interlocked.Increment(ref _wrongSessionCount); continue; }
            Interlocked.Increment(ref _receivedCount);
            if (!ok) Interlocked.Increment(ref _rejectedPoseCount);
            depth.RecordPoseReceived(frameId);
            SubmitPose(frameId, ok, confidence, position, rotation, detectMs, pnpMs, sendMs,
                pcRecvMl2, pcSendMl2, sessionId);
        }
    }

    private static bool IsSyncRequest(byte[] packet)
    {
        return packet.Length >= 4 &&
               packet[0] == (byte)'M' && packet[1] == (byte)'L' &&
               packet[2] == (byte)'2' && packet[3] == (byte)'S';
    }

    /// One-shot NTP-style clock-sync reply (see pose_packet.py:sync_clock_offset).
    /// Echoes the PC's request timestamp back alongside this clock's current reading
    /// so the desktop can estimate the offset between the two clocks. Runs on this
    /// server thread — Time.realtimeSinceStartupAsDouble is safe to read off the main
    /// thread (same assumption already relied on elsewhere in this file).
    private void HandleSyncRequest(NetworkStream stream, byte[] request)
    {
        double pcT0 = BitConverter.ToDouble(request, 8);
        double ml2T1 = Time.realtimeSinceStartupAsDouble;

        byte[] reply = new byte[PacketSize];
        using (MemoryStream memory = new MemoryStream(reply))
        using (BinaryWriter writer = new BinaryWriter(memory))
        {
            writer.Write((byte)'M');
            writer.Write((byte)'L');
            writer.Write((byte)'2');
            writer.Write((byte)'R');
            writer.Write((ushort)ProtocolVersion);
            writer.Write((ushort)PacketSize);
            writer.Write(pcT0);
            writer.Write(ml2T1);
            // Remaining bytes stay zero-padded to PacketSize.
        }

        stream.Write(reply, 0, reply.Length);
        stream.Flush();
    }

    private static bool TryParsePacket(byte[] packet, out ulong frameId, out bool ok,
        out float confidence, out Vector3 position, out Quaternion rotation,
        out float detectMs, out float pnpMs, out float sendMs,
        out double pcRecvMl2, out double pcSendMl2)
    {
        frameId = 0;
        ok = false;
        confidence = 0f;
        position = default;
        rotation = Quaternion.identity;
        detectMs = 0f;
        pnpMs = 0f;
        sendMs = 0f;
        pcRecvMl2 = 0.0;
        pcSendMl2 = 0.0;

        using MemoryStream memory = new MemoryStream(packet, false);
        using BinaryReader reader = new BinaryReader(memory);

        byte[] magic = reader.ReadBytes(4);
        if (magic.Length != 4 ||
            magic[0] != (byte)'M' || magic[1] != (byte)'L' ||
            magic[2] != (byte)'2' || magic[3] != (byte)'P')
            return false;

        ushort version = reader.ReadUInt16();
        ushort size = reader.ReadUInt16();
        if (!((version == 3 && size == PacketSize) || (version == 4 && size == PacketSize + 8)))
            return false;

        frameId = reader.ReadUInt64();
        ok = reader.ReadUInt32() != 0;
        confidence = reader.ReadSingle();
        position = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
        rotation = new Quaternion(
            reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
        detectMs = reader.ReadSingle();
        pnpMs = reader.ReadSingle();
        sendMs = reader.ReadSingle();
        pcRecvMl2 = reader.ReadDouble();
        pcSendMl2 = reader.ReadDouble();
        return true;
    }

    private bool ReadExact(NetworkStream stream, byte[] buffer, int count)
    {
        int offset = 0;
        while (offset < count)
        {
            if (!_running)
                return false;

            int read;
            try
            {
                read = stream.Read(buffer, offset, count - offset);
            }
            catch (IOException)
            {
                return false;
            }

            if (read == 0)
                return false;

            offset += read;
        }

        return true;
    }
}


// Bounded storage; no allocations until the throttled summary is formatted.
internal sealed class TimingWindow
{
    private readonly double[] values = new double[256];
    private readonly double[] sorted = new double[256];
    private int count, next;
    public void Clear() { count = 0; next = 0; }
    public void Add(double milliseconds)
    {
        if (double.IsNaN(milliseconds) || double.IsInfinity(milliseconds) || milliseconds < 0) return;
        values[next] = milliseconds; next = (next + 1) % values.Length;
        count = Math.Min(count + 1, values.Length);
    }
    internal double Percentile(double fraction)
    {
        if (count == 0) return double.NaN;
        Array.Copy(values, sorted, count); Array.Sort(sorted, 0, count);
        return sorted[Math.Max(0, Math.Min(count - 1, (int)Math.Ceiling(fraction * count) - 1))];
    }
    public string Summary()
    {
        if (count == 0) return "unavailable(n=0)";
        Array.Copy(values, sorted, count); Array.Sort(sorted, 0, count);
        return $"{sorted[(int)Math.Ceiling(.50 * count)-1]:F1}/" +
            $"{sorted[(int)Math.Ceiling(.95 * count)-1]:F1}/{sorted[(int)Math.Ceiling(.99 * count)-1]:F1}(n={count})";
    }
}
