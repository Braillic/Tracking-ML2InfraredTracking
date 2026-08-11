using System;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using TMPro;
using UnityEngine;

/// <summary>
/// Listens for desktop pose packets (PC → Magic Leap) and applies the newest
/// valid estimate to a tool transform. Companion to <see cref="DepthFrameTcpServer"/>
/// (ML → PC depth); uses a separate port so depth send stays write-only.
///
/// Wire format (little-endian, fixed 52 bytes):
///   magic "ML2P"
///   ushort version (=1), ushort packet_size (=52)
///   ulong  frame_id   (matches depth header frame number)
///   uint   ok         (0 = reject, nonzero = apply)
///   float  confidence
///   float  px, py, pz           Unity world metres
///   float  qx, qy, qz, qw       Unity world quaternion
/// </summary>
public sealed class PoseEstimateTcpServer : MonoBehaviour
{
    public const int ProtocolVersion = 1;
    public const int PacketSize = 52;
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
    [Tooltip("Ignore poses whose frame_id is older than the last applied pose.")]
    [SerializeField] private bool dropStaleFrames = true;

    [Header("Marker visualization (created on first accepted pose)")]
    [Tooltip("Spawn one sphere per model point (object-frame metres). Defaults match desktop TEST_MARKER_COORDS.")]
    [SerializeField] private bool createMarkerSpheresOnFirstPose = true;
    [SerializeField] private Vector3[] markerLocalPositions =
    {
        new Vector3(0f, 0.0501f, 0f),
        new Vector3(-0.0131f, 0.0126f, 0f),
        new Vector3(0f, 0f, 0f),
        new Vector3(0f, -0.0391f, 0f),
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
    [SerializeField, Min(0f)] private float hideAfterNoPoseSeconds = 0.5f;

    private readonly object _poseLock = new object();
    private bool _hasPending;
    private ulong _pendingFrameId;
    private bool _pendingOk;
    private float _pendingConfidence;
    private Vector3 _pendingPosition;
    private Quaternion _pendingRotation;

    private Thread _serverThread;
    private TcpListener _listener;
    private TcpClient _client;
    private volatile bool _running;
    private volatile bool _clientConnected;
    private volatile string _remoteEndpoint;
    private volatile string _status = "Pose stream stopped";
    private ulong _lastAppliedFrameId;
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
        if (startAutomatically)
            StartServer();
    }

    private void Update()
    {
        TryApplyPendingPose();
        HideTrackedToolIfTimedOut();

        if (Time.unscaledTimeAsDouble >= _nextStatusUpdate)
        {
            _nextStatusUpdate = Time.unscaledTimeAsDouble + 0.25;
            long received = Interlocked.Read(ref _receivedCount);
            long applied = Interlocked.Read(ref _appliedCount);
            if (_clientConnected)
            {
                _status = received == 0
                    ? $"PC connected: {_remoteEndpoint}\nWaiting for pose packets..."
                    : $"PC connected: {_remoteEndpoint}\nposes recv {received} | applied {applied}";
            }
        }

        if (statusText != null)
            statusText.text = _status;
    }

    private void OnGUI()
    {
        if (!showOnGuiOverlay)
            return;

        GUI.Box(new Rect(12, 78, 520, 58), _status);
    }

    private void OnDisable()
    {
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
        _status = $"Starting pose server on {bindAddress}:{port}...";
        _serverThread = new Thread(ServerLoop)
        {
            IsBackground = true,
            Name = "ML2 Pose TCP Server"
        };
        _serverThread.Start();
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
        _status = "Pose stream stopped";
    }

    /// <summary>
    /// Thread-safe mailbox used by the receive loop. Main thread consumes via Update.
    /// </summary>
    public void SubmitPose(ulong frameId, bool ok, float confidence, in Vector3 position,
        in Quaternion rotation)
    {
        lock (_poseLock)
        {
            _pendingFrameId = frameId;
            _pendingOk = ok;
            _pendingConfidence = confidence;
            _pendingPosition = position;
            _pendingRotation = rotation;
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

        lock (_poseLock)
        {
            if (!_hasPending)
                return;

            frameId = _pendingFrameId;
            ok = _pendingOk;
            confidence = _pendingConfidence;
            position = _pendingPosition;
            rotation = _pendingRotation;
            _hasPending = false;
        }

        if (!ok || confidence < minConfidence)
            return;

        if (dropStaleFrames && frameId < _lastAppliedFrameId)
        {
            return;
        }

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
        _lastAcceptedPoseTime = Time.unscaledTimeAsDouble;
        Interlocked.Increment(ref _appliedCount);
    }

    private void HideTrackedToolIfTimedOut()
    {
        if (hideAfterNoPoseSeconds <= 0f || trackedTool == null || !trackedTool.gameObject.activeSelf)
            return;

        if (Time.unscaledTimeAsDouble - _lastAcceptedPoseTime > hideAfterNoPoseSeconds)
            trackedTool.gameObject.SetActive(false);
    }

    /// <summary>
    /// Creates a TrackedTool root (if needed) and child spheres at each model marker
    /// the first time a pose is accepted. Sphere local positions are the 3D constellation
    /// points used by PnP — once the root is posed, they sit on the physical markers.
    /// </summary>
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

            if (!TryParsePacket(packet, out ulong frameId, out bool ok, out float confidence,
                    out Vector3 position, out Quaternion rotation))
            {
                throw new InvalidDataException("Invalid ML2P pose packet");
            }

            Interlocked.Increment(ref _receivedCount);
            DepthFrameTcpServer.ActiveServer?.RecordPoseReceived(frameId);
            SubmitPose(frameId, ok, confidence, position, rotation);
        }
    }

    private static bool TryParsePacket(byte[] packet, out ulong frameId, out bool ok,
        out float confidence, out Vector3 position, out Quaternion rotation)
    {
        frameId = 0;
        ok = false;
        confidence = 0f;
        position = default;
        rotation = Quaternion.identity;

        using MemoryStream memory = new MemoryStream(packet, false);
        using BinaryReader reader = new BinaryReader(memory);

        byte[] magic = reader.ReadBytes(4);
        if (magic.Length != 4 ||
            magic[0] != (byte)'M' || magic[1] != (byte)'L' ||
            magic[2] != (byte)'2' || magic[3] != (byte)'P')
            return false;

        ushort version = reader.ReadUInt16();
        ushort size = reader.ReadUInt16();
        if (version != ProtocolVersion || size != PacketSize)
            return false;

        frameId = reader.ReadUInt64();
        ok = reader.ReadUInt32() != 0;
        confidence = reader.ReadSingle();
        position = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
        rotation = new Quaternion(
            reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
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
