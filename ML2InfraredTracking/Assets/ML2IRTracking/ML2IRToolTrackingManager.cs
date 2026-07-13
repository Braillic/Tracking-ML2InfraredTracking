using UnityEngine;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using MagicLeap.OpenXR.Features.PixelSensors;

public class ML2ToolTrackingManager : MonoBehaviour
{
    [Header("Tracking Data")]
    public List<GameObject> Markers;        // Scene markers whose local positions form the 3D model points
    public GameObject TrackedTool;          // Scene object to drive with the estimated pose
    public Renderer targetRenderer;         // Renderer that previews the depth texture
    public float maxNoTrackingDuration = 2.0f; // Seconds before forcing Kalman re-init when tracking is lost

    [SerializeField] private PixelSensorMaterialTable materialList = new();

    // --- Native handles ---
    private IntPtr _poseEstimatorPtr = IntPtr.Zero;
    private IntPtr _kalmanFilterPtr  = IntPtr.Zero;
    private float[] _previousRvecNative = new float[3];
    private float[] _previousTvecNative = new float[3];
    private float[] _cameraMatrixNative = new float[9];     // 3x3 fx,fy,cx,cy
    private float[] _distCoeffsNative;                      // k1,k2,p1,p2,k3
    private byte _kfInitialized = 0;                        // state that shows if the kalman filter is initailized or not

    // --- Model points ---
    private readonly List<Vector3> _objectPoints = new();
    private float[] _objectPointsFlat;

    private float minDepth = 0, maxDepth = 5;
    private readonly System.Diagnostics.Stopwatch _trackingStopwatch = System.Diagnostics.Stopwatch.StartNew();
    private long _lastTrackingTicks;
    private int _consecutiveRejectCount = 0;
    private const int MaxConsecutiveRejects = 15;
    private Texture2D targetTexture;
    private Color32[] _targetTextureClearPixels;
    private readonly object _blobPreviewLock = new();
    private readonly Vector2[] _blobPreviewPoints = new Vector2[20];
    private readonly List<Vector2> blobCenters = new();
    private int _blobPreviewCount;
    private int _blobPreviewWidth;
    private int _blobPreviewHeight;
    private bool _blobPreviewDirty;


    // --- WORKER THREAD INFRASTRUCTURE --- 

    // Input mailbox : coroutine writes, worker reads
    private readonly object _submitLock = new();
    private readonly List<byte[]> _availableByteBuffers = new();
    private byte[] _pendingBytes;
    private int _pendingW, _pendingH;
    private Pose _pendingPose;
    private long _pendingSubmitTicks;
    private bool _pendingReady;
    private long _overwrittenPendingFrames;

    // Output mailbox : worker writes, Update() reads (volatile reference swap)
    private readonly PoseResult _resultA = new PoseResult();
    private readonly PoseResult _resultB = new PoseResult();
    private volatile PoseResult _latestResult;
    private bool _useResultA = true;

    private Thread _workerThread;
    private volatile bool _workerRunning;

    private class PoseResult
    {
        public Vector3 position;
        public Quaternion rotation = Quaternion.identity;
        public bool isValid;
    }

    public Vector3 finalPosition { get; private set; }
    public Quaternion finalRotation { get; private set; }
    private Quaternion previousFilteredRot = Quaternion.identity;
    private float lastTrackingTime;

    

    // ----------------- Called by DepthSensorAPI after sensor starts ---------------
    public void StartWorker()
    {
        if (_workerThread != null && _workerThread.IsAlive)
        {
            return;
        }
        _workerRunning = true;
        _workerThread  = new Thread(WorkerLoop)
        {
            Name = "ML2TrackingWorker",
            IsBackground = true,
            Priority =  System.Threading.ThreadPriority.AboveNormal
        };
        _workerThread.Start();
        Debug.Log("[ML2Tracking] Worker thread started.");
    }

    public bool StopWorker()
    {
        if (_workerThread == null) return true;

        _workerRunning = false;
        
        lock (_submitLock) Monitor.PulseAll(_submitLock);

        if (!_workerThread.Join(2000))
        {
            Debug.LogError("[ML2Tracking] Worker thread did not exit in time — native handles may be unsafe to destroy.");
            return false;
        }

        _workerThread = null;
        return true;
    }

    public byte[] AcquireFrameBuffer(int requiredLength)
    {
        lock (_submitLock)
        {
            for (int i = _availableByteBuffers.Count - 1; i >= 0; i--)
            {
                byte[] candidate = _availableByteBuffers[i];
                if (candidate.Length == requiredLength)
                {
                    _availableByteBuffers.RemoveAt(i);
                    return candidate;
                }
            }

            return new byte[requiredLength];
        }
    }

    private void ReleaseFrameBuffer(byte[] buffer)
    {
        if (buffer == null)
        {
            return;
        }

        lock (_submitLock)
        {
            _availableByteBuffers.Add(buffer);
        }
    }

    // ---------- Called by coroutine : returns immediately ---------------
    public void SubmitRawFrame(byte[] frameBytes, int w, int h, Pose sensorPose)
    {
        if (frameBytes == null)
        {
            return;
        }

        lock (_submitLock)
        {
            if (_pendingBytes != null)
            {
                _availableByteBuffers.Add(_pendingBytes);
                _overwrittenPendingFrames++;
            }

            _pendingBytes = frameBytes;
            _pendingW = w;
            _pendingH = h;
            _pendingPose = sensorPose;
            _pendingSubmitTicks = _trackingStopwatch.ElapsedTicks;
            _pendingReady = true;
            Monitor.Pulse(_submitLock);
        }
    }


    //  ----------------- WORKER LOOP : runs entirely off the main thread -------------
    private void WorkerLoop()
    {
        // Local working copies : avoid holding the lock during processing
        byte[] localBytes = null;
        int localW = 0, localH = 0;
        Pose localPose = default;
        float[] localFloat = null;
        float[] blobXY = new float[40];
        int blobCount = 0;
        Quaternion filteredRot = Quaternion.identity;
        var outRes = ML2IRTRackingPluginImports.PoseResult_CS.Create();
        const int timingLogEveryNFrames = 30;
        long processedSamples = 0;
        long validSamples = 0;
        long noPoseSamples = 0;
        long rejectBehindSamples = 0;
        long rejectJumpSamples = 0;
        float totalRejectJumpPosDelta = 0f;
        float totalRejectJumpRotDelta = 0f;
        float maxRejectJumpPosDelta = 0f;
        float maxRejectJumpRotDelta = 0f;
        long totalQueueDelayTicks = 0;
        long totalDecodeTicks = 0;
        long totalNativeTicks = 0;
        long totalPostTicks = 0;
        long totalWorkerTicks = 0;
        long localPendingSubmitTicks = 0;

        while (_workerRunning)
        {
            long workerPassStart = System.Diagnostics.Stopwatch.GetTimestamp();

            // Wait for a submitted frame
            lock (_submitLock)
            {
                while (!_pendingReady && _workerRunning)
                    Monitor.Wait(_submitLock, 50);

                if (!_workerRunning) break;
                if (!_pendingReady)  continue;

                if (_pendingBytes == null)
                {
                    _pendingReady = false;
                    continue;
                }

                localBytes = _pendingBytes;
                _pendingBytes = null;
                localW = _pendingW;
                localH = _pendingH;
                localPose = _pendingPose;
                localPendingSubmitTicks = _pendingSubmitTicks;
                _pendingReady = false;
            }

            // ---- Decode depth to float[] metres (off main thread) ----
            try
            {
                long afterDequeueTicks = _trackingStopwatch.ElapsedTicks;
                totalQueueDelayTicks += afterDequeueTicks - localPendingSubmitTicks;

                // ---- Decode depth to float[] metres (off main thread) ----
                long decodeStart = System.Diagnostics.Stopwatch.GetTimestamp();
                int n = localW * localH;
                if (localFloat == null || localFloat.Length != n)
                    localFloat = new float[n];

                if (localBytes.Length == n * 2)
                {
                    for (int i = 0; i < n; i++)
                        localFloat[i] = (localBytes[2*i] | (localBytes[2*i+1] << 8)) * 0.001f;
                }
                else if (localBytes.Length == n * 4)
                {
                    Buffer.BlockCopy(localBytes, 0, localFloat, 0, localBytes.Length);
                }
                else continue;
                long decodeEnd = System.Diagnostics.Stopwatch.GetTimestamp();
                totalDecodeTicks += decodeEnd - decodeStart;

                // ---- Rebuild model points if needed ----
                if (_objectPointsFlat == null || _objectPointsFlat.Length != _objectPoints.Count * 3)
                    _objectPointsFlat = FlattenObjectPoints(_objectPoints);

                if (_objectPoints.Count < 4) continue;

                // ---- Native call: blob detect + PnP + Kalman ----
                outRes.hasPose = 0;
                blobCount = 0;
                bool nativePoseProduced = false;

                byte ok;
                long nativeStart = System.Diagnostics.Stopwatch.GetTimestamp();
                ok = ML2IRTRackingPluginImports.RunTrackingAndEstimatePose(
                        _poseEstimatorPtr, _kalmanFilterPtr,
                        localFloat, localW, localH,
                        _objectPointsFlat, _objectPoints.Count,
                        _cameraMatrixNative, _distCoeffsNative,
                        _previousRvecNative, _previousTvecNative,
                        ref _kfInitialized,
                        ref outRes,
                        blobXY, ref blobCount);
                long nativeEnd = System.Diagnostics.Stopwatch.GetTimestamp();
                totalNativeTicks += nativeEnd - nativeStart;
                nativePoseProduced = (ok != 0 && outRes.hasPose != 0);
                PublishBlobPreview(blobXY, blobCount, localW, localH);
                 

                // ---- Build result for main thread ----
                long postStart = System.Diagnostics.Stopwatch.GetTimestamp();
                if (nativePoseProduced)
                {
                    Matrix4x4 camTobj = Utils.MatrixFromColumnMajor(outRes.worldTObject);
                    Matrix4x4 worldTsensor = Matrix4x4.TRS(localPose.position, localPose.rotation, Vector3.one);
                    Matrix4x4 worldTobj = worldTsensor * camTobj;

                    Vector3 pos = worldTobj.GetColumn(3);
                    Quaternion rot = Quaternion.LookRotation(worldTobj.GetColumn(2), worldTobj.GetColumn(1));
                    

                    // finalPosition = pos;
                    // finalRotation = Quaternion.Slerp(previousFilteredRot, rot, 0.8f);
                    // previousFilteredRot = finalRotation;

                    // if (TrackedTool) TrackedTool.transform.SetPositionAndRotation(finalPosition, finalRotation);


                    // FIX 1: Behind-camera rejection
                    // One of the two ambiguous solutions always has negative Z in camera space.
                    Vector3 posInCam = camTobj.GetColumn(3);
                    if (posInCam.z < 0f)
                    {
                        rejectBehindSamples++;
                        goto trackingLost;
                    }

                    // // FIX 2: Temporal jump rejection
                    // // If Kalman is running and pose jumps too far, it's the wrong solution.
                    PoseResult prev = _latestResult;
                    if (_kfInitialized != 0 && prev.isValid)
                    {
                        float posDelta = Vector3.Distance(pos, prev.position);
                        float rotDelta = Quaternion.Angle(rot, prev.rotation);

                        if (posDelta > 0.15f || rotDelta > 45f)
                        {
                            rejectJumpSamples++;
                            _consecutiveRejectCount++;
                            totalRejectJumpPosDelta += posDelta;
                            totalRejectJumpRotDelta += rotDelta;
                            if (posDelta > maxRejectJumpPosDelta) maxRejectJumpPosDelta = posDelta;
                            if (rotDelta > maxRejectJumpRotDelta) maxRejectJumpRotDelta = rotDelta;
                            Debug.Log($"[rejectJump] blobs={blobCount} kf={_kfInitialized} posDelta={posDelta:F4} rotDelta={rotDelta:F2}");

                            if (_consecutiveRejectCount >= MaxConsecutiveRejects)
                            {
                                Debug.LogWarning($"[ML2Tracking] Reject streak reached {MaxConsecutiveRejects}. Resetting Kalman/guess for next frame.");
                                _kfInitialized = 0;
                                _consecutiveRejectCount = 0;
                                filteredRot = Quaternion.identity;
                            }

                            PoseResult lost = _useResultA ? _resultA : _resultB;
                            lost.isValid = false;
                            _latestResult = lost;
                            _useResultA = !_useResultA;
                            goto trackingLost;
                        }
                    }

                    // // FIX 3: Rotation consistency via tighter Kalman reset guard
                    // // Only reset Kalman if tracking has been truly lost for maxNoTrackingDuration,
                    // // not on every missed frame : prevents cold restarts that re-introduce ambiguity.
                    bool coldStart = (_kfInitialized == 0);
                    if (coldStart)
                        filteredRot = rot;
                    else
                    {
                        float tRot  = 1f - Mathf.Exp(-60f * 0.033f);
                        filteredRot = Quaternion.Slerp(filteredRot, rot, tRot);
                    }

                    PoseResult next = _useResultA ? _resultA : _resultB;
                    next.position = pos;
                    next.rotation = filteredRot;
                    next.isValid = true;
                    _latestResult = next;
                    _useResultA = !_useResultA;
                    _consecutiveRejectCount = 0;
                    validSamples++;

                    _lastTrackingTicks = _trackingStopwatch.ElapsedTicks;
                    long postEnd = System.Diagnostics.Stopwatch.GetTimestamp();
                    totalPostTicks += postEnd - postStart;
                    processedSamples++;
                    totalWorkerTicks += postEnd - workerPassStart;
                    LogWorkerTimingIfNeeded(
                        ref processedSamples,
                        ref validSamples,
                        ref noPoseSamples,
                        ref rejectBehindSamples,
                        ref rejectJumpSamples,
                        ref totalRejectJumpPosDelta,
                        ref totalRejectJumpRotDelta,
                        ref maxRejectJumpPosDelta,
                        ref maxRejectJumpRotDelta,
                        ref totalQueueDelayTicks,
                        ref totalDecodeTicks,
                        ref totalNativeTicks,
                        ref totalPostTicks,
                        ref totalWorkerTicks,
                        timingLogEveryNFrames);
                    continue;
                }

                trackingLost:
                if (!nativePoseProduced)
                {
                    noPoseSamples++;
                    Debug.Log($"[noPose] blobs={blobCount} kf={_kfInitialized}");
                }
                double elapsed = (_trackingStopwatch.ElapsedTicks - _lastTrackingTicks)
                     / (double)System.Diagnostics.Stopwatch.Frequency;
                if (elapsed > maxNoTrackingDuration)
                {
                    _kfInitialized = 0;
                    filteredRot = Quaternion.identity;
                    PoseResult lost = _useResultA ? _resultA : _resultB;
                    lost.isValid = false;
                    _latestResult = lost;
                    _useResultA = !_useResultA;
                }

                long trackingLostPostEnd = System.Diagnostics.Stopwatch.GetTimestamp();
                totalPostTicks += trackingLostPostEnd - postStart;
                processedSamples++;
                totalWorkerTicks += trackingLostPostEnd - workerPassStart;
                LogWorkerTimingIfNeeded(
                    ref processedSamples,
                    ref validSamples,
                    ref noPoseSamples,
                    ref rejectBehindSamples,
                    ref rejectJumpSamples,
                    ref totalRejectJumpPosDelta,
                    ref totalRejectJumpRotDelta,
                    ref maxRejectJumpPosDelta,
                    ref maxRejectJumpRotDelta,
                    ref totalQueueDelayTicks,
                    ref totalDecodeTicks,
                    ref totalNativeTicks,
                    ref totalPostTicks,
                    ref totalWorkerTicks,
                    timingLogEveryNFrames);
            }
            finally
            {
                ReleaseFrameBuffer(localBytes);
                localBytes = null;
            }
        }
    }

    private void LogWorkerTimingIfNeeded(
        ref long processedSamples,
        ref long validSamples,
        ref long noPoseSamples,
        ref long rejectBehindSamples,
        ref long rejectJumpSamples,
        ref float totalRejectJumpPosDelta,
        ref float totalRejectJumpRotDelta,
        ref float maxRejectJumpPosDelta,
        ref float maxRejectJumpRotDelta,
        ref long totalQueueDelayTicks,
        ref long totalDecodeTicks,
        ref long totalNativeTicks,
        ref long totalPostTicks,
        ref long totalWorkerTicks,
        int timingLogEveryNFrames)
    {
        if (processedSamples < timingLogEveryNFrames)
        {
            return;
        }

        double tickToMs = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        Debug.Log(
            $"[Timing][Worker] Avg over {processedSamples} processed frames: " +
            $"QueueDelay={(totalQueueDelayTicks / (double)processedSamples) * tickToMs:F2} ms, " +
            $"Decode={(totalDecodeTicks / (double)processedSamples) * tickToMs:F2} ms, " +
            $"Native={(totalNativeTicks / (double)processedSamples) * tickToMs:F2} ms, " +
            $"Post={(totalPostTicks / (double)processedSamples) * tickToMs:F2} ms, " +
            $"WorkerTotal={(totalWorkerTicks / (double)processedSamples) * tickToMs:F2} ms, " +
            $"valid={validSamples}, noPose={noPoseSamples}, rejectBehind={rejectBehindSamples}, rejectJump={rejectJumpSamples}, " +
            $"rejectJumpPosAvg={(rejectJumpSamples > 0 ? totalRejectJumpPosDelta / rejectJumpSamples : 0f):F4} m, " +
            $"rejectJumpPosMax={maxRejectJumpPosDelta:F4} m, " +
            $"rejectJumpRotAvg={(rejectJumpSamples > 0 ? totalRejectJumpRotDelta / rejectJumpSamples : 0f):F2} deg, " +
            $"rejectJumpRotMax={maxRejectJumpRotDelta:F2} deg, overwrittenPending={_overwrittenPendingFrames}");

        processedSamples = 0;
        validSamples = 0;
        noPoseSamples = 0;
        rejectBehindSamples = 0;
        rejectJumpSamples = 0;
        totalRejectJumpPosDelta = 0f;
        totalRejectJumpRotDelta = 0f;
        maxRejectJumpPosDelta = 0f;
        maxRejectJumpRotDelta = 0f;
        totalQueueDelayTicks = 0;
        totalDecodeTicks = 0;
        totalNativeTicks = 0;
        totalPostTicks = 0;
        totalWorkerTicks = 0;
    }

    
    // ---- MAIN THREAD : only reads result and applies transform -----
    
    private Vector3 _lastAppliedPos = Vector3.zero;
    private Quaternion _lastAppliedRot = Quaternion.identity;

    private void Update()
    {
        ApplyBlobPreview();

        if (!TrackedTool) return;
        PoseResult r = _latestResult;
        if (!r.isValid) return;

        bool moved = Vector3.Distance(r.position, _lastAppliedPos) > 0.0002f
                  || Quaternion.Angle(r.rotation, _lastAppliedRot) > 0.1f;
        if (!moved) return;

        TrackedTool.transform.SetPositionAndRotation(r.position, r.rotation);
        _lastAppliedPos = r.position;
        _lastAppliedRot = r.rotation;
    }

    private void PublishBlobPreview(float[] blobXY, int blobCount, int width, int height)
    {
        int count = Mathf.Min(blobCount, _blobPreviewPoints.Length);
        lock (_blobPreviewLock)
        {
            _blobPreviewCount = count;
            _blobPreviewWidth = width;
            _blobPreviewHeight = height;

            for (int i = 0; i < count; i++)
            {
                _blobPreviewPoints[i] = new Vector2(blobXY[2 * i], blobXY[2 * i + 1]);
            }

            _blobPreviewDirty = true;
        }
    }

    private void ApplyBlobPreview()
    {
        if (!targetRenderer) return;

        int count;
        int width;
        int height;

        lock (_blobPreviewLock)
        {
            if (!_blobPreviewDirty)
            {
                return;
            }

            count = _blobPreviewCount;
            width = _blobPreviewWidth;
            height = _blobPreviewHeight;

            blobCenters.Clear();
            for (int i = 0; i < count; i++)
            {
                blobCenters.Add(_blobPreviewPoints[i]);
            }

            _blobPreviewDirty = false;
        }

        if (width <= 0 || height <= 0)
        {
            return;
        }

        if (targetTexture == null)
        {
            targetTexture = new Texture2D(width, height, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Point
            };
        }
        else if (targetTexture.width != width || targetTexture.height != height || targetTexture.format != TextureFormat.RGBA32)
        {
            targetTexture.Reinitialize(width, height, TextureFormat.RGBA32, false);
            targetTexture.filterMode = FilterMode.Point;
        }

        int pixelCount = width * height;
        if (_targetTextureClearPixels == null || _targetTextureClearPixels.Length != pixelCount)
        {
            _targetTextureClearPixels = new Color32[pixelCount];
        }

        targetTexture.SetPixels32(_targetTextureClearPixels);

        Utils.DrawBlobsMarkers(targetTexture, blobCenters, 3, 6, Color.green);
        targetRenderer.material.mainTexture = targetTexture;
    }

   
    private void Start()
    {
        _resultA.isValid = false;
        _resultB.isValid = false;
        _latestResult = _resultA;

        _objectPoints.Clear();
        if (Markers != null)
        {
            foreach (var go in Markers) 
            {
                if (go) 
                {
                    _objectPoints.Add(go.transform.localPosition);
                }
            }
        }

        _poseEstimatorPtr = ML2IRTRackingPluginImports.CreatePoseEstimatorNative();
        _kalmanFilterPtr = ML2IRTRackingPluginImports.CreateKalmanFilterNative(1.0f, 1e-4f, 3.0f);

        if (_poseEstimatorPtr == IntPtr.Zero || _kalmanFilterPtr == IntPtr.Zero)
        {
            Debug.LogError("[ML2Tracking] Failed to create native handles."); 
            enabled = false; 
            return;
        }

         _cameraMatrixNative[0] = 363.11f; _cameraMatrixNative[1] = 0.0f; _cameraMatrixNative[2] = 267.86f;
        _cameraMatrixNative[3] = 0.0f; _cameraMatrixNative[4] = 363.11f; _cameraMatrixNative[5] = 237.837f;
        _cameraMatrixNative[6] = 0.0f; _cameraMatrixNative[7] = 0.0f; _cameraMatrixNative[8] = 1.0f;
        _distCoeffsNative = new float[] { -0.0908f, -0.0129f, -0.0001f, 0.0001f, -0.0112f };
    }

    public void Initialize(uint streamId, MagicLeapPixelSensorFeature feature, PixelSensorId sensorType)
    {
        if (!sensorType.SensorName.Contains("depth", StringComparison.CurrentCultureIgnoreCase)) return;
        if (feature.QueryPixelSensorCapability(sensorType, PixelSensorCapabilityType.Depth, streamId, out var range))
        {
            if (range.IntRange.HasValue) { minDepth = range.IntRange.Value.Min; maxDepth = range.IntRange.Value.Max; }
            if (range.FloatRange.HasValue) { minDepth = range.FloatRange.Value.Min; maxDepth = range.FloatRange.Value.Max; }
        }
    }

    private void OnDestroy()
    {
        bool workerStoppedCleanly = StopWorker();

        if (targetTexture != null)
        {
            Destroy(targetTexture);
            targetTexture = null;
        }

        if (!workerStoppedCleanly)
        {
            return;
        }

        if (_kalmanFilterPtr  != IntPtr.Zero) { ML2IRTRackingPluginImports.DestroyKalmanFilterNative(_kalmanFilterPtr);  _kalmanFilterPtr  = IntPtr.Zero; }
        if (_poseEstimatorPtr != IntPtr.Zero) { ML2IRTRackingPluginImports.DestroyPoseEstimatorNative(_poseEstimatorPtr); _poseEstimatorPtr = IntPtr.Zero; }
    }

    private static float[] FlattenObjectPoints(List<Vector3> pts)
    {
        if (pts == null || pts.Count == 0) return Array.Empty<float>();
        
        var flat = new float[pts.Count * 3];
        
        int k = 0;
        
        foreach (var p in pts) 
        { 
            flat[k++] = p.x; 
            flat[k++] = p.y; 
            flat[k++] = p.z; 
        }

        return flat;
    }
}
