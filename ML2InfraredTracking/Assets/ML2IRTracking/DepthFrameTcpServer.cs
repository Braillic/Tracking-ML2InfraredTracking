using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Threading;
using TMPro;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// Listens for a desktop TCP client and sends the newest depth frame as FLOAT32 meters.
/// Network I/O runs on a background thread so a slow client cannot block the sensor loop.
/// </summary>
public sealed class DepthFrameTcpServer : MonoBehaviour
{
    public const int ProtocolVersion = 3;
    public const int HeaderSize = 72;
    public const uint PixelFormatFloat32Metres = 1;
    public static DepthFrameTcpServer ActiveServer { get; private set; }

    [Header("TCP server (Magic Leap listens; PC connects)")]
    [SerializeField] private bool startAutomatically = true;
    [Tooltip("Use 0.0.0.0 for Wi-Fi and adb forwarding. Use 127.0.0.1 for adb-only access.")]
    [SerializeField] private string bindAddress = "0.0.0.0";
    [SerializeField, Range(1, 65535)] private int port = 50777;
    [Tooltip("0 sends every frame. A lower rate is often more reliable over Wi-Fi.")]
    [SerializeField, Min(0)] private float maximumFramesPerSecond = 15f;

    [Header("Status display (optional)")]
    [SerializeField] private TMP_Text statusText;
    [Tooltip("Optional separate TMP target for pose and intrinsics. If empty, they are appended to Status Text.")]
    [SerializeField] private TMP_Text sensorInfoText;
    [Tooltip("A simple fallback overlay. For XR, assigning a world-space TMP Text is preferable.")]
    [SerializeField] private bool showOnGuiOverlay = true;

    private readonly object _frameLock = new object();
    private byte[] _pendingPayload;
    private byte[] _sparePayload;
    private int _pendingWidth;
    private int _pendingHeight;
    private ulong _pendingFrameNumber;
    private double _pendingTimestamp;
    private Pose _pendingSensorPose;
    private DepthCameraIntrinsics? _pendingIntrinsics;

    private Thread _serverThread;
    private TcpListener _listener;
    private TcpClient _client;
    private volatile bool _running;
    private volatile bool _clientConnected;
    private volatile string _remoteEndpoint;
    private volatile string _status = "Depth stream stopped";
    private string _sensorInfo;
    private long _submittedFrameCount;
    private long _sentFrameCount;
    private int _lastWidth;
    private int _lastHeight;
    private ulong _nextFrameNumber;
    private double _nextSubmitTime;
    private double _nextStatusUpdate;

    // Round-trip latency tracking (depth-sent → pose-received, keyed by frame ID).
    // PoseEstimateTcpServer calls RecordPoseReceived when it gets a pose back.
    private readonly ConcurrentDictionary<ulong, long> _frameSendTicks = new();
    private long _latencySumTicks;
    private long _latencyCount;
    private long _latencyMaxTicks;
    private double _nextLatencyLog;

    public int Port => port;
    public string Status => _status;

    private void OnEnable()
    {
        if (ActiveServer != null && ActiveServer != this)
        {
            Debug.LogError($"[ML2DepthTCP] Duplicate server on '{name}' disabled. " +
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
        if (_clientConnected && Time.unscaledTimeAsDouble >= _nextStatusUpdate)
        {
            _nextStatusUpdate = Time.unscaledTimeAsDouble + 0.25;
            long submitted = Interlocked.Read(ref _submittedFrameCount);
            long sent = Interlocked.Read(ref _sentFrameCount);
            _status = submitted == 0
                ? $"PC connected: {_remoteEndpoint}\nWaiting for first DepthRaw frame..."
                : $"PC connected: {_remoteEndpoint}\nDepth {_lastWidth}x{_lastHeight} | queued {submitted} | sent {sent}";
        }

        long pendingLatencySamples = Interlocked.Read(ref _latencyCount);
        if (pendingLatencySamples >= 100)
        {
            var (avgMs, maxMs, count) = ConsumeLatencyStats();
            if (count > 0)
                Debug.Log($"[ML2Latency] last {count} poses: avg {avgMs:F1} ms, max {maxMs:F1} ms");
        }

        if (statusText != null)
            statusText.text = sensorInfoText == null && !string.IsNullOrEmpty(_sensorInfo)
                ? $"{_status}\n{_sensorInfo}"
                : _status;
        if (sensorInfoText != null)
            sensorInfoText.text = _sensorInfo;
    }

    private void OnGUI()
    {
        if (!showOnGuiOverlay)
            return;

        GUI.Box(new Rect(12, 12, 520, 58), _status);
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
        Interlocked.Exchange(ref _submittedFrameCount, 0);
        Interlocked.Exchange(ref _sentFrameCount, 0);
        _status = $"Starting depth server on {bindAddress}:{port}...";
        _serverThread = new Thread(ServerLoop)
        {
            IsBackground = true,
            Name = "ML2 Depth TCP Server"
        };
        _serverThread.Start();
    }

    public void StopServer()
    {
        if (!_running && _serverThread == null)
            return;

        _running = false;
        _clientConnected = false;
        lock (_frameLock)
            Monitor.PulseAll(_frameLock);

        try { _client?.Close(); } catch { }
        try { _listener?.Stop(); } catch { }

        if (_serverThread != null && _serverThread.IsAlive)
            _serverThread.Join(1000);

        _client = null;
        _listener = null;
        _serverThread = null;
        _status = "Depth stream stopped";
    }

    /// <summary>
    /// Called by PoseEstimateTcpServer when a pose packet arrives for a given frame.
    /// Computes round-trip latency (depth-sent → pose-received) and accumulates stats.
    /// </summary>
    public void RecordPoseReceived(ulong frameId)
    {
        if (!_frameSendTicks.TryRemove(frameId, out long sendTick))
            return;

        long dt = Stopwatch.GetTimestamp() - sendTick;
        Interlocked.Add(ref _latencySumTicks, dt);
        Interlocked.Increment(ref _latencyCount);

        long prevMax;
        do { prevMax = Interlocked.Read(ref _latencyMaxTicks); }
        while (dt > prevMax && Interlocked.CompareExchange(ref _latencyMaxTicks, dt, prevMax) != prevMax);
    }

    /// <summary>
    /// Returns (avgMs, maxMs, sampleCount) and resets the accumulators.
    /// </summary>
    public (double avgMs, double maxMs, long count) ConsumeLatencyStats()
    {
        long sum = Interlocked.Exchange(ref _latencySumTicks, 0);
        long count = Interlocked.Exchange(ref _latencyCount, 0);
        long max = Interlocked.Exchange(ref _latencyMaxTicks, 0);
        double freq = Stopwatch.Frequency;
        double avgMs = count > 0 ? (sum / freq * 1000.0 / count) : 0;
        double maxMs = max / freq * 1000.0;
        return (avgMs, maxMs, count);
    }

    /// <summary>
    /// Called on Unity's main thread. The input array may be reused immediately after this returns.
    /// Only the newest unsent frame is retained.
    /// </summary>
    public void SubmitFrame(float[] depthMetres, int width, int height, in Pose sensorPose,
        DepthCameraIntrinsics? intrinsics)
    {
        if (!_running || depthMetres == null || width <= 0 || height <= 0)
            return;

        int elementCount;
        int byteCount;
        try
        {
            elementCount = checked(width * height);
            byteCount = checked(elementCount * sizeof(float));
        }
        catch (OverflowException)
        {
            return;
        }

        if (depthMetres.Length < elementCount)
            return;

        double now = Time.realtimeSinceStartupAsDouble;
        if (maximumFramesPerSecond > 0f && now < _nextSubmitTime)
            return;
        if (maximumFramesPerSecond > 0f)
            _nextSubmitTime = now + 1.0 / maximumFramesPerSecond;

        lock (_frameLock)
        {
            if (_pendingPayload == null || _pendingPayload.Length != byteCount)
            {
                if (_sparePayload != null && _sparePayload.Length == byteCount)
                {
                    _pendingPayload = _sparePayload;
                    _sparePayload = null;
                }
                else
                {
                    _pendingPayload = new byte[byteCount];
                }
            }

            Buffer.BlockCopy(depthMetres, 0, _pendingPayload, 0, byteCount);
            _pendingWidth = width;
            _pendingHeight = height;
            _pendingFrameNumber = _nextFrameNumber++;
            _pendingTimestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
            _pendingSensorPose = sensorPose;
            _pendingIntrinsics = intrinsics;
            _lastWidth = width;
            _lastHeight = height;
            _sensorInfo = BuildSensorInfo(sensorPose, intrinsics);
            long submitted = Interlocked.Increment(ref _submittedFrameCount);
            if (submitted == 1)
                Debug.Log($"[ML2DepthTCP] First depth frame queued: {width}x{height}, {byteCount} bytes.");
            Monitor.Pulse(_frameLock);
        }
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
            _status = BuildListeningStatus();

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
                    client.SendBufferSize = 1024 * 1024;
                    string remote = client.Client.RemoteEndPoint?.ToString() ?? "PC";
                    _remoteEndpoint = remote;
                    _clientConnected = true;
                    _status = $"PC connected: {remote}\nWaiting for first DepthRaw frame...";
                    SendFrames(client);
                }
                catch (SocketException) when (!_running)
                {
                    break;
                }
                catch (Exception exception)
                {
                    if (_running)
                        _status = $"Client disconnected ({exception.GetType().Name})\n{BuildListeningStatus()}";
                }
                finally
                {
                    _clientConnected = false;
                    try { _client?.Close(); } catch { }
                    _client = null;
                    if (_running)
                        _status = BuildListeningStatus();
                }
            }
        }
        catch (Exception exception)
        {
            _status = $"Depth server error: {exception.Message}";
        }
    }

    private void SendFrames(TcpClient client)
    {
        using NetworkStream stream = client.GetStream();
        byte[] header = new byte[HeaderSize];
        bool intrinsicsSent = false;

        while (_running && client.Connected)
        {
            byte[] payload;
            int width;
            int height;
            ulong frameNumber;
            double timestamp;
            Pose sensorPose;
            DepthCameraIntrinsics? intrinsics;

            lock (_frameLock)
            {
                while (_running && _pendingPayload == null)
                    Monitor.Wait(_frameLock, 500);
                if (!_running)
                    return;

                payload = _pendingPayload;
                width = _pendingWidth;
                height = _pendingHeight;
                frameNumber = _pendingFrameNumber;
                timestamp = _pendingTimestamp;
                sensorPose = _pendingSensorPose;
                intrinsics = _pendingIntrinsics;
                _pendingPayload = null;
            }

            byte[] intrinsicsBytes = !intrinsicsSent && intrinsics.HasValue
                ? BuildIntrinsicsBlock(intrinsics.Value)
                : null;
            int intrinsicsByteCount = intrinsicsBytes?.Length ?? 0;

            WriteHeader(header, width, height, payload.Length, frameNumber, timestamp,
                sensorPose, intrinsicsByteCount);
            stream.Write(header, 0, header.Length);
            stream.Write(payload, 0, payload.Length);
            if (intrinsicsByteCount > 0)
            {
                stream.Write(intrinsicsBytes, 0, intrinsicsByteCount);
                intrinsicsSent = true;
            }
            _frameSendTicks[frameNumber] = Stopwatch.GetTimestamp();
            Interlocked.Increment(ref _sentFrameCount);

            lock (_frameLock)
            {
                if (_sparePayload == null || _sparePayload.Length != payload.Length)
                    _sparePayload = payload;
            }

            // Evict stale entries so the dictionary doesn't grow unbounded
            if (_frameSendTicks.Count > 120)
            {
                ulong cutoff = frameNumber > 100 ? frameNumber - 100 : 0;
                foreach (var key in _frameSendTicks.Keys)
                    if (key < cutoff) _frameSendTicks.TryRemove(key, out _);
            }
        }
    }

    private static void WriteHeader(byte[] header, int width, int height, int payloadBytes,
        ulong frameNumber, double timestamp, in Pose sensorPose, int intrinsicsByteCount)
    {
        using MemoryStream memory = new MemoryStream(header, true);
        using BinaryWriter writer = new BinaryWriter(memory);
        writer.Write(new[] { (byte)'M', (byte)'L', (byte)'2', (byte)'D' });
        writer.Write((ushort)ProtocolVersion);
        writer.Write((ushort)HeaderSize);
        writer.Write((uint)width);
        writer.Write((uint)height);
        writer.Write(PixelFormatFloat32Metres);
        writer.Write((uint)payloadBytes);
        writer.Write(frameNumber);
        writer.Write(timestamp);
        writer.Write(sensorPose.position.x);
        writer.Write(sensorPose.position.y);
        writer.Write(sensorPose.position.z);
        writer.Write(sensorPose.rotation.x);
        writer.Write(sensorPose.rotation.y);
        writer.Write(sensorPose.rotation.z);
        writer.Write(sensorPose.rotation.w);
        writer.Write((uint)intrinsicsByteCount);
    }

    private static byte[] BuildIntrinsicsBlock(in DepthCameraIntrinsics intrinsics)
    {
        byte[] block = new byte[DepthCameraIntrinsics.BinarySize];
        using MemoryStream memory = new MemoryStream(block, true);
        using BinaryWriter writer = new BinaryWriter(memory);
        writer.Write(intrinsics.Fx);
        writer.Write(intrinsics.Fy);
        writer.Write(intrinsics.Cx);
        writer.Write(intrinsics.Cy);
        writer.Write(intrinsics.FovX);
        writer.Write(intrinsics.FovY);
        writer.Write(intrinsics.K1);
        writer.Write(intrinsics.K2);
        writer.Write(intrinsics.P1);
        writer.Write(intrinsics.P2);
        writer.Write(intrinsics.K3);
        return block;
    }

    private static string BuildSensorInfo(in Pose sensorPose, DepthCameraIntrinsics? intrinsics)
    {
        Vector3 p = sensorPose.position;
        Quaternion q = sensorPose.rotation;
        string pose = $"\nSensor pose\nP: ({p.x:F3}, {p.y:F3}, {p.z:F3}) m\n" +
                      $"Q: ({q.x:F4}, {q.y:F4}, {q.z:F4}, {q.w:F4})";
        return intrinsics.HasValue ? pose + "\n" + intrinsics.Value : pose + "\nIntrinsics: waiting...";
    }

    private string BuildListeningStatus()
    {
        string localIp = GetLocalIpv4Address() ?? "ML Wi-Fi IP unavailable";
        return $"Depth server: {localIp}:{port}\nADB: 127.0.0.1:{port}";
    }

    private static string GetLocalIpv4Address()
    {
        try
        {
            return NetworkInterface.GetAllNetworkInterfaces()
                .Where(adapter => adapter.OperationalStatus == OperationalStatus.Up &&
                                  adapter.NetworkInterfaceType != NetworkInterfaceType.Loopback)
                .SelectMany(adapter => adapter.GetIPProperties().UnicastAddresses)
                .Select(info => info.Address)
                .FirstOrDefault(address => address.AddressFamily == AddressFamily.InterNetwork &&
                                           !IPAddress.IsLoopback(address))
                ?.ToString();
        }
        catch
        {
            return null;
        }
    }
}
