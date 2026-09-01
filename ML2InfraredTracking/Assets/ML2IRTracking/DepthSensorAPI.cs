using System.Collections;
using System.Collections.Generic;
using System.Linq;
using MagicLeap.Android;
using Unity.Collections;
using UnityEngine;
using UnityEngine.XR.MagicLeap;
using UnityEngine.XR.OpenXR;
using MagicLeap.OpenXR.Features.PixelSensors;
using Unity.XR.CoreUtils;
using System.Text;
using System;


public class DepthSensorAPI : MonoBehaviour
{

    [Header("General Configuration")] // change between ML2ToolTrackingManager or IRToolTrack if you are using the ML2IRTracking plugin or the scripts in C#
    public ML2DepthRawStream streamVisualizer;
    //public IRToolTrack streamVisualizer;

    [SerializeField] XROrigin xrOrigin;

    [Tooltip("If Tue will return a raw depth image. If False will return depth32")]
    public bool UseRawDepth;

    [Range(0.2f, 5.00f)] public float DepthRange;

    // [Header("ShortRange =< 1m")] public ShortRangeUpdateRate SRUpdateRate;

    // [Header("LongRange > 1m")] public LongRangeUpdateRate LRUpdateRate;

    // public enum LongRangeUpdateRate
    // {
    //     OneFps = 1, FiveFps = 5

    // }
    // public enum ShortRangeUpdateRate
    // {
    //     FiveFps = 5, ThirtyFps = 30, SixtyFps = 60
    // }

    private const string depthCameraSensorPath = "/pixelsensor/depth/center";

    private MagicLeapPixelSensorFeature pixelSensorFeature;
    private PixelSensorId? sensorId;
    private List<uint> configuredStreams = new List<uint>();

    // Recorded every frame at NextPredictedDisplayTime (always a valid XrLocateSpace
    // query) so we can interpolate to a depth frame's CaptureTime - which is often
    // already outside the runtime's pose-history window by the time we see it -
    // instead of querying XrLocateSpace directly with a timestamp it may reject.
    // 2s of retention keeps CaptureTime (typically <1s stale) inside the recorded
    // range in the common case, so most lookups interpolate between closely-spaced
    // samples instead of falling back to the noisier extrapolation path.
    private readonly SensorPoseHistory sensorPoseHistory = new SensorPoseHistory(2.0);

    // Smooths the noisy raw headset-tracked sensor pose before it's recorded, so
    // both interpolation and extrapolation work from a clean signal instead of
    // reproducing per-sample tracking jitter.
    private readonly OneEuroFilterVector3 positionFilter = new OneEuroFilterVector3();
    private readonly OneEuroFilterQuaternion rotationFilter = new OneEuroFilterQuaternion();

    // Temporary diagnostics for tuning the history-based pose lookup. Remove once
    // the flicker/lag under motion is understood.
    private int _historyMissCount;
    private int _liveUpdateOkCount;
    private int _liveUpdateFailCount;
    private float _lastHistoryDiagLog;


    public uint targetStream
    {
        get { return DepthRange > 1.0f ? (uint)0 : (uint)1; }
    }

    void Start()
    {
        pixelSensorFeature = OpenXRSettings.Instance.GetFeature<MagicLeapPixelSensorFeature>();
        if (pixelSensorFeature == null || !pixelSensorFeature.enabled)
        {
            Debug.LogError("Pixel Sensor Feature not found or not enabled!");
            enabled = false;
            return;
        }
        Permissions.RequestPermission(MLPermission.DepthCamera, OnPermissionGranted, OnPermissionDenied,
            OnPermissionDenied);
    }

    private void OnPermissionGranted(string permission)
    {
        if (permission.Contains(MLPermission.DepthCamera))
            FindAndInitializeSensor();

    }

    private void OnPermissionDenied(string permission)
    {
        Debug.LogError($"Permission {permission} not granted. Example script will not work.");
        enabled = false;
    }

    private void FindAndInitializeSensor()
    {
        var sensors = pixelSensorFeature.GetSupportedSensors();

        foreach (var sensor in sensors)
        {
            Debug.Log("Sensor Name Found: " + sensor.XrPathString);
            if (sensor.XrPathString.Contains(depthCameraSensorPath))
            {
                sensorId = sensor;
                break;
            }
        }

        if (!sensorId.HasValue)
        {
            Debug.LogError($"`{depthCameraSensorPath}` sensor not found.");
            return;
        }

        // Subscribe to the Availability changed callback if the sensor becomes available.
        pixelSensorFeature.OnSensorAvailabilityChanged += OnSensorAvailabilityChanged;
        TryInitializeSensor();
    }

    private void OnSensorAvailabilityChanged(PixelSensorId id, bool available)
    {
        if (sensorId.HasValue && id == sensorId && available)
        {
            Debug.Log("Sensor became available.");
            TryInitializeSensor();
        }
    }

    private void TryInitializeSensor()
    {
        if (sensorId.HasValue && pixelSensorFeature.GetSensorStatus(sensorId.Value) ==
            PixelSensorStatus.Undefined && pixelSensorFeature.CreatePixelSensor(sensorId.Value))
        {
            Debug.Log("Sensor created successfully.");
            ConfigureSensorStreams();
        }
        else
        {
            Debug.LogWarning("Failed to create sensor. Will retry when it becomes available.");
        }
    }

    // The capabilities that the script will edit
    private PixelSensorCapabilityType[] targetCapabilityTypes = new[]
    {
        PixelSensorCapabilityType.UpdateRate,
        PixelSensorCapabilityType.Format,
        PixelSensorCapabilityType.Resolution,
        PixelSensorCapabilityType.Depth,
    };


    private void ConfigureSensorStreams()
    {
        if (!sensorId.HasValue)
        {
            Debug.LogError("Sensor ID not set.");
            return;
        }

        uint streamCount = pixelSensorFeature.GetStreamCount(sensorId.Value);
        if (streamCount < 1)
        {
            Debug.LogError("Expected at least one stream from the sensor.");
            return;
        }

        // Only add the target
        configuredStreams.Add(targetStream);


        pixelSensorFeature.GetPixelSensorCapabilities(sensorId.Value, targetStream, out var capabilities);
        foreach (var pixelSensorCapability in capabilities)
        {
            if (!targetCapabilityTypes.Contains(pixelSensorCapability.CapabilityType))
            {
                continue;
            }

            // More details about the capability
            if (pixelSensorFeature.QueryPixelSensorCapability(sensorId.Value, pixelSensorCapability.CapabilityType, targetStream, out PixelSensorCapabilityRange range) && range.IsValid)
            {
                if (range.CapabilityType == PixelSensorCapabilityType.UpdateRate)
                {
                    var configData = new PixelSensorConfigData(range.CapabilityType, targetStream);

                    uint maxUpdateRate = 5;
                    if (range.IntValues != null && range.IntValues.Length > 0)
                    {
                        foreach (uint intValue in range.IntValues)
                        {
                            Debug.Log($"UpdateRate RangeIntValues : {intValue}");
                            if (intValue > maxUpdateRate)
                            {
                                maxUpdateRate = intValue;
                            }
                        }

                        configData.IntValue = maxUpdateRate;
                    }

                    pixelSensorFeature.ApplySensorConfig(sensorId.Value, configData);

                }
                else if (range.CapabilityType == PixelSensorCapabilityType.Format)
                {
                    var configData = new PixelSensorConfigData(range.CapabilityType, targetStream);
                    PixelSensorFrameFormat desiredFormat = UseRawDepth
                        ? PixelSensorFrameFormat.DepthRaw
                        : PixelSensorFrameFormat.Depth32;
                    if (range.FrameFormats == null || !range.FrameFormats.Contains(desiredFormat))
                    {
                        Debug.LogError($"Requested pixel sensor format {desiredFormat} is unavailable. " +
                                       $"Supported: {string.Join(", ", range.FrameFormats ?? Array.Empty<PixelSensorFrameFormat>())}");
                        continue;
                    }

                    Debug.Log($"Configuring pixel sensor format: {desiredFormat}");
                    configData.IntValue = (uint)desiredFormat;
                    pixelSensorFeature.ApplySensorConfig(sensorId.Value, configData);
                }
                else if (range.CapabilityType == PixelSensorCapabilityType.Resolution)
                {
                    var configData = new PixelSensorConfigData(range.CapabilityType, targetStream);
                    configData.VectorValue = range.ExtentValues[0];
                    pixelSensorFeature.ApplySensorConfig(sensorId.Value, configData);
                }
                else if (range.CapabilityType == PixelSensorCapabilityType.Depth)
                {
                    var configData = new PixelSensorConfigData(range.CapabilityType, targetStream);
                    configData.FloatValue = DepthRange;
                    pixelSensorFeature.ApplySensorConfig(sensorId.Value, configData);
                }
            }
        }

        StartCoroutine(ConfigureStreamsAndStartSensor());
    }

    private IEnumerator ConfigureStreamsAndStartSensor()
    {

        var configureOperation = pixelSensorFeature.ConfigureSensor(sensorId.Value, configuredStreams.ToArray());

        yield return configureOperation;

        if (configureOperation.DidOperationSucceed)
        {
            Debug.Log("Sensor configured with defaults successfully.");
        }
        else
        {
            Debug.LogError("Failed to configure sensor.");
            yield break;
        }


        Dictionary<uint, PixelSensorMetaDataType[]> supportedMetadataTypes =
        new Dictionary<uint, PixelSensorMetaDataType[]>();

        foreach (uint stream in configuredStreams)
        {
            if (pixelSensorFeature.EnumeratePixelSensorMetaDataTypes(sensorId.Value, stream, out var metaDataTypes))
            {
                supportedMetadataTypes[stream] = metaDataTypes;

            }
        }

        // Assuming that `configuredStreams` is correctly populated with the intended stream indices
        PixelSensorAsyncOperationResult startOperation = pixelSensorFeature.StartSensor(sensorId.Value, configuredStreams, supportedMetadataTypes);

        yield return startOperation;

        if (startOperation.DidOperationSucceed)
        {
            Debug.Log("Sensor started successfully. Monitoring data...");
            StartCoroutine(MonitorSensorData());
        }
        else
        {
            Debug.LogError("Failed to start sensor.");
        }
    }

    private IEnumerator MonitorSensorData()
    {

        // Initialize Stream ...
        streamVisualizer.Initialize(configuredStreams[0], pixelSensorFeature, sensorId.Value);


        while (pixelSensorFeature.GetSensorStatus(sensorId.Value) ==
               PixelSensorStatus.Started)
        {

            foreach (uint stream in configuredStreams) 
            {

                if (pixelSensorFeature.GetSensorData(sensorId.Value, stream, out var frame, out var metaData,
                        Allocator.Temp, shouldFlipTexture: true))
                {
                    // Process Frames ...
                    if (!sensorPoseHistory.TryGetPose(frame.CaptureTime, out Pose sensorPose))
                    {
                        // History not warmed up yet (first frame or two) - fall back to a
                        // direct query, which may itself hit TimeInvalid this early.
                        _historyMissCount++;
                        sensorPose = pixelSensorFeature.GetSensorPose(sensorId.Value, frame.CaptureTime);
                    }
                    if (Time.realtimeSinceStartup - _lastHistoryDiagLog > 2f)
                    {
                        _lastHistoryDiagLog = Time.realtimeSinceStartup;
                        Debug.Log($"[ML2SensorPoseHistoryDiag] samples={sensorPoseHistory.SampleCount} " +
                                  $"spanMs={sensorPoseHistory.SpanTicks / 1e6:F1} historyMisses={_historyMissCount} " +
                                  $"liveUpdateOk={_liveUpdateOkCount} liveUpdateFail={_liveUpdateFailCount}");
                    }
                    sensorPose = DepthSensorPoseUtil.ToWorldPose(sensorPose, xrOrigin);

                    streamVisualizer.ProcessFrame(frame, metaData, sensorPose);

                    yield return null;
                }
            }
        }
    }

    // Cap the live-pose polling rate. Sampling every rendered frame (60-90Hz) adds
    // enough native XrLocateSpace overhead to stress frame timing, and packs samples
    // so close together that tracking noise gets amplified into large spurious
    // velocity estimates during extrapolation. ~20Hz still comfortably out-resolves
    // the ~5Hz depth stream while cutting both problems down.
    private const float LiveSampleIntervalSeconds = 1f / 20f;
    private float _lastLiveSampleTime = float.NegativeInfinity;

    private void LateUpdate()
    {
        if (!sensorId.HasValue || pixelSensorFeature == null)
            return;

        if (Time.realtimeSinceStartup - _lastLiveSampleTime < LiveSampleIntervalSeconds)
            return;
        _lastLiveSampleTime = Time.realtimeSinceStartup;

        // Display-rate extrinsic sample, always taken at NextPredictedDisplayTime
        // (always a valid XrLocateSpace query). Feeds SensorPoseHistory so depth
        // frames can interpolate to their own CaptureTime instead of querying
        // XrLocateSpace directly with a timestamp that's often already stale.
        bool gotLivePose = DepthSensorPoseUtil.TryGetLiveSensorTrackingPose(
            pixelSensorFeature, sensorId.Value, out Pose trackingPose, out long time);
        if (gotLivePose)
        {
            double timeSeconds = time / 1e9;
            trackingPose = new Pose(
                positionFilter.Filter(trackingPose.position, timeSeconds),
                rotationFilter.Filter(trackingPose.rotation, timeSeconds));
            sensorPoseHistory.Record(time, trackingPose);
            _liveUpdateOkCount++;
        }
        else
        {
            _liveUpdateFailCount++;
        }

        if (streamVisualizer == null)
            return;

        var debugger = streamVisualizer.SensorPoseDebugger;
        if (debugger == null || !debugger.isActiveAndEnabled || !gotLivePose)
            return;

        Pose livePose = DepthSensorPoseUtil.ToWorldPose(trackingPose, xrOrigin);
        debugger.SubmitLive(livePose);
    }

    public void OnDisable()
    {
        //We start the Coroutine on another MonoBehaviour since it can only run while the object is enabled.
        MonoBehaviour camMono = Camera.main.GetComponent<MonoBehaviour>();
        camMono.StartCoroutine(StopSensorCoroutine());
    }

    private IEnumerator StopSensorCoroutine()
    {
        if (sensorId.HasValue)
        {
            PixelSensorAsyncOperationResult stopSensorAsyncResult =
                pixelSensorFeature.StopSensor(sensorId.Value, configuredStreams);

            yield return stopSensorAsyncResult;

            if (stopSensorAsyncResult.DidOperationSucceed)
            {
                Debug.Log("Sensor stopped successfully.");
                pixelSensorFeature.ClearAllAppliedConfigs(sensorId.Value);
                // Free the sensor so it can be marked available and used in other scripts.
                pixelSensorFeature.DestroyPixelSensor(sensorId.Value);
            }
            else
            {
                Debug.LogError("Failed to stop the sensor.");
            }
        }
    }
}
