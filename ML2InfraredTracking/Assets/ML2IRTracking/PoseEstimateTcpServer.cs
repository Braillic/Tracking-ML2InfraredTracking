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
///   float  send_ms    PC world-space transform + struct pack duration (excludes the sendall syscall)
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
    [Tooltip("Optional. If empty, a TrackedTool root is created on the first accepted pose.")]
    [SerializeField] private Transform trackedTool;
    [SerializeField, Min(0f)] private float minConfidence;
    [Tooltip("Ignore duplicate or older frame IDs. Keep enabled for live tracking.")]
    [SerializeField] private bool dropStaleFrames = true;
    [Tooltip("Maximum submit-to-apply age on ML2's own clock. Independent of capture cadence and tracking-loss hold.")]
    [SerializeField, Min(0.001f)] private float maxPoseAgeSeconds = 0.15f;

    [Header("Marker visualization (created on first accepted pose)")]
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
    [Tooltip("0 = snap. Small values reduce jitter but add a little lag.")]
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
    private ulong _pendingFrameId;
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
        Application.onBeforeRender += MeasureBeforeRender;
        if (startAutomatically)
            StartServer();
    }

    private void Update()
    {
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

        if (statusText != null)
            statusText.text = _status;
    }

    private void LateUpdate()
    {
        TryApplyPendingPose();
        HideTrackedToolIfTimedOut();
    }

    private void OnGUI()
    {
        if (!showOnGuiOverlay)
            return;

        GUI.Box(new Rect(12, 78, 520, 58), _status);
    }

    private void OnDisable()
    {
        Application.onBeforeRender -= MeasureBeforeRender;
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
        _lastAcceptedPoseTime = double.NegativeInfinity;
        _lastCaptureRealtime = double.NaN;
        _awaitingBeforeRender = false;
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
        _status = "Pose stream stopped";
    }

    public void SubmitPose(ulong frameId, bool ok, float confidence, in Vector3 position,
        in Quaternion rotation, float detectMs, float pnpMs, float sendMs,
        double pcRecvMl2, double pcSendMl2)
    {
        // realtimeSinceStartup is safe to read off the main thread; avoid Debug.Log here.
        double receivedRealtime = Time.realtimeSinceStartupAsDouble;
        lock (_poseLock)
        {
            if (dropStaleFrames && _hasPending && frameId <= _pendingFrameId)
                return;
            _pendingFrameId = frameId;
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

    private void TryApplyPendingPose()
    {
        ulong frameId;
        bool ok;
        float confidence;
        Vector3 position;
        Quaternion rotation;
        double receivedRealtime;
        float detectMs, pnpMs, sendMs;
        double pcRecvMl2, pcSendMl2;

        lock (_poseLock)
        {
            if (!_hasPending)
                return;

            frameId = _pendingFrameId;
            ok = _pendingOk;
            confidence = _pendingConfidence;
            position = _pendingPosition;
            rotation = _pendingRotation;
            receivedRealtime = _pendingReceivedRealtime;
            detectMs = _pendingDetectMs;
            pnpMs = _pendingPnpMs;
            sendMs = _pendingSendMs;
            pcRecvMl2 = _pendingPcRecvMl2;
            pcSendMl2 = _pendingPcSendMl2;
            _hasPending = false;
        }

        if (!ok || float.IsNaN(confidence) || float.IsInfinity(confidence) || confidence < minConfidence
            || !IsFinitePose(position, rotation))
            return;

        var depthServer = DepthFrameTcpServer.ActiveServer;
        double now = Time.realtimeSinceStartupAsDouble;
        double submit = 0.0;
        bool knownFrame = depthServer != null && depthServer.TryGetFrameTiming(frameId,
            out submit, out _, out _, out _);
        if (!IsFreshPose(frameId, _hasAppliedFrame, _lastAppliedFrameId, dropStaleFrames,
                         knownFrame, submit, now, maxPoseAgeSeconds))
        {
            _stalePoseCount++;
            return;
        }
        rotation = rotation.normalized;

        EnsureTrackedToolVisual();
        if (trackedTool == null)
            return;

        trackedTool.gameObject.SetActive(true);
        if (positionFollow >= 0.999f)
            trackedTool.position = position;
        else
            trackedTool.position = Vector3.Lerp(trackedTool.position, position, positionFollow);

        if (rotationFollow >= 0.999f)
            trackedTool.rotation = rotation;
        else
            trackedTool.rotation = Quaternion.Slerp(trackedTool.rotation, rotation, rotationFollow);

        _lastAppliedFrameId = frameId;
        _hasAppliedFrame = true;
        // Receipt of a delayed packet must not extend the lifetime of an old pose.
        _lastAcceptedPoseTime = submit;
        Interlocked.Increment(ref _appliedCount);
        MaybeLogLatency(frameId, receivedRealtime, detectMs, pnpMs, sendMs, pcRecvMl2, pcSendMl2);
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

    private void MaybeLogLatency(ulong frameId, double receivedRealtime,
        float detectMs, float pnpMs, float sendMs, double pcRecvMl2, double pcSendMl2)
    {
        if (!logLatency)
            return;
        _lastApplyRealtime = Time.realtimeSinceStartupAsDouble;
        _awaitingBeforeRender = true;
        var source = DepthFrameTcpServer.ActiveServer;
        if (source != null && source.TryGetFrameTiming(frameId, out double submitted, out _, out _, out _))
        {
            _submitToApply.Add((_lastApplyRealtime - submitted) * 1000.0);
            _lastCaptureRealtime = double.NaN;
            if (source.TryGetCaptureTiming(frameId, out CaptureTiming timing))
            {
                _lastCaptureRealtime = timing.CaptureRealtime;
                _captureToApply.Add((_lastApplyRealtime - timing.CaptureRealtime) * 1000.0);
                _captureToSubmit.Add((submitted - timing.CaptureRealtime) * 1000.0);
                _pollDuration.Add((timing.FrameReadyRealtime - timing.PollStartRealtime) * 1000.0);
                _readyToSubmit.Add((submitted - timing.FrameReadyRealtime) * 1000.0);
            }
        }

        _latencyLogCounter++;
        if (_latencyLogCounter % latencyLogEveryN != 0)
            return;

        double now = Time.realtimeSinceStartupAsDouble;
        double applyWaitMs = (now - receivedRealtime) * 1000.0;

        var depthServer = DepthFrameTcpServer.ActiveServer;
        if (depthServer == null ||
            !depthServer.TryGetFrameTiming(frameId, out double submit, out double sendDone,
                out bool hasSend, out DepthFrameTcpServer.PipelineMode framePipeline))
        {
            Debug.Log($"[ML2LAT] frame={frameId} apply_wait={applyWaitMs:F1}ms " +
                      "(no matching depth submit time — is DepthFrameTcpServer active?)");
            return;
        }

        double submitToApplyMs = (now - submit) * 1000.0;
        if (!hasSend)
        {
            Debug.Log($"[ML2LAT] frame={frameId} submit_to_apply={submitToApplyMs:F1}ms apply_wait={applyWaitMs:F1}ms " +
                      "(send time missing)");
            return;
        }

        double queueMs = (sendDone - submit) * 1000.0;
        double leg1Ms = (pcRecvMl2 - sendDone) * 1000.0; // network: ML2 -> PC
        double leg2Ms = (receivedRealtime - pcSendMl2) * 1000.0; // network: PC -> ML2
        double measuredMs = queueMs + leg1Ms + detectMs + pnpMs + sendMs + leg2Ms + applyWaitMs;
        double unaccountedMs = submitToApplyMs - measuredMs; // PC queue/omitted work; a constant clock offset cancels between leg1 and leg2
        Debug.Log(
            $"[ML2LAT] frame={frameId} pipeline={DepthFrameTcpServer.PipelineLabel(framePipeline)} " +
            $"submit_to_apply={submitToApplyMs:F1}ms " +
            $"(ml_prepare+queue+tcp_write={queueMs:F1}ms | leg1_estimate(ML2->PC)={leg1Ms:F1}ms | " +
            $"detect={detectMs:F1}ms | pnp={pnpMs:F1}ms | pack={sendMs:F1}ms | " +
            $"leg2_estimate(PC->ML2)={leg2Ms:F1}ms | apply_wait={applyWaitMs:F1}ms | " +
            $"unaccounted={unaccountedMs:F1}ms | update_fps={_measuredUpdateFps:F1})");
    }

    private void MeasureBeforeRender()
    {
        if (!logLatency || !_awaitingBeforeRender) return;
        _applyToBeforeRender.Add((Time.realtimeSinceStartupAsDouble - _lastApplyRealtime) * 1000.0);
        _awaitingBeforeRender = false;
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
            _submitToApply.Clear(); _captureToApply.Clear(); _captureToSubmit.Clear();
            _pollDuration.Clear(); _readyToSubmit.Clear(); _applyToBeforeRender.Clear();
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
            $"ready_to_submit={_readyToSubmit.Summary()} apply_to_before_render={_applyToBeforeRender.Summary()} " +
            "presentation=unmeasured (rolling last 256 accepted samples, ms p50/p95/p99)");
    }

    private void HideTrackedToolIfTimedOut()
    {
        if (hideAfterNoPoseSeconds <= 0f || trackedTool == null || !trackedTool.gameObject.activeSelf)
            return;

        if (Time.realtimeSinceStartupAsDouble - _lastAcceptedPoseTime > hideAfterNoPoseSeconds)
            trackedTool.gameObject.SetActive(false);
    }


    /// Creates a TrackedTool root (if needed) and child spheres at each model marker
    /// the first time a pose is accepted. Sphere local positions are the 3D constellation
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
                byte[] sessionBytes = new byte[8];
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
                pcRecvMl2, pcSendMl2);
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
        var sorted = new double[count]; Array.Copy(values, sorted, count); Array.Sort(sorted);
        return sorted[Math.Max(0, Math.Min(count - 1, (int)Math.Ceiling(fraction * count) - 1))];
    }
    public string Summary() => count == 0 ? "unavailable(n=0)" :
        $"{Percentile(.50):F1}/{Percentile(.95):F1}/{Percentile(.99):F1}(n={count})";
}
