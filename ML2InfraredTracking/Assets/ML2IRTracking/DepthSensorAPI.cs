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
using Stopwatch = System.Diagnostics.Stopwatch;

public class DepthSensorAPI : MonoBehaviour
{
    private const int TimingLogEveryNFrames = 30;

    [Header("General Configuration")]
    public ML2ToolTrackingManager streamVisualizer;

    [SerializeField] XROrigin xrOrigin;

    [Tooltip("If True will return a raw depth image. If False will return depth32")]
    public bool UseRawDepth;

    [Range(0.2f, 5.00f)] public float DepthRange;

    [Header("ShortRange =< 1m")] public ShortRangeUpdateRate SRUpdateRate;
    [Header("LongRange > 1m")]   public LongRangeUpdateRate  LRUpdateRate;

    public enum LongRangeUpdateRate  { OneFps = 1, FiveFps = 5 }
    public enum ShortRangeUpdateRate { FiveFps = 5, ThirtyFps = 30, SixtyFps = 60 }

    private const string depthCameraSensorPath = "/pixelsensor/depth/center";
    private MagicLeapPixelSensorFeature pixelSensorFeature;
    private PixelSensorId? sensorId;
    private List<uint> configuredStreams = new List<uint>();

    public uint targetStream => DepthRange > 1.0f ? (uint)0 : (uint)1;


    void Start()
    {
        Application.targetFrameRate = 60;
        QualitySettings.vSyncCount  = 0;
        pixelSensorFeature = OpenXRSettings.Instance.GetFeature<MagicLeapPixelSensorFeature>();
        if (pixelSensorFeature == null || !pixelSensorFeature.enabled)
        {
            Debug.LogError("Pixel Sensor Feature not found or not enabled!");
            enabled = false;
            return;
        }
        Permissions.RequestPermission(MLPermission.DepthCamera, OnPermissionGranted, OnPermissionDenied, OnPermissionDenied);
    }

    private void OnPermissionGranted(string permission)
    {
        if (permission.Contains(MLPermission.DepthCamera)) FindAndInitializeSensor();
    }

    private void OnPermissionDenied(string permission)
    {
        Debug.LogError($"Permission {permission} not granted.");
        enabled = false;
    }

    private void FindAndInitializeSensor()
    {
        var sensors = pixelSensorFeature.GetSupportedSensors();
        foreach (var sensor in sensors)
        {
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

        pixelSensorFeature.OnSensorAvailabilityChanged += OnSensorAvailabilityChanged;
        TryInitializeSensor();
    }

    private void OnSensorAvailabilityChanged(PixelSensorId id, bool available)
    {
        if (sensorId.HasValue && id == sensorId && available) TryInitializeSensor();
    }

    private void TryInitializeSensor()
    {
        if (sensorId.HasValue &&
            pixelSensorFeature.GetSensorStatus(sensorId.Value) == PixelSensorStatus.Undefined &&
            pixelSensorFeature.CreatePixelSensor(sensorId.Value))
        {
            ConfigureSensorStreams();
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
        if (!sensorId.HasValue) return;
        configuredStreams.Clear();
        configuredStreams.Add(targetStream);

        pixelSensorFeature.GetPixelSensorCapabilities(sensorId.Value, targetStream, out var capabilities);
        foreach (var cap in capabilities)
        {
            if (!targetCapabilityTypes.Contains(cap.CapabilityType)) continue;
            if (!pixelSensorFeature.QueryPixelSensorCapability(sensorId.Value, cap.CapabilityType, targetStream, out var range) || !range.IsValid) continue;

            var cfg = new PixelSensorConfigData(range.CapabilityType, targetStream);
            if (range.CapabilityType == PixelSensorCapabilityType.UpdateRate)
            {
                cfg.IntValue = DepthRange > 1 ? (uint)LRUpdateRate : (uint)SRUpdateRate;                
            }  
            else if (range.CapabilityType == PixelSensorCapabilityType.Format)      
            {
                cfg.IntValue = (uint)range.FrameFormats[UseRawDepth ? 1 : 0];
            }
            else if (range.CapabilityType == PixelSensorCapabilityType.Resolution)
            {
                cfg.VectorValue = range.ExtentValues[0];
            }  
            else if (range.CapabilityType == PixelSensorCapabilityType.Depth)
            {
                cfg.FloatValue = DepthRange;
            }       
            pixelSensorFeature.ApplySensorConfig(sensorId.Value, cfg);
        }

        StartCoroutine(ConfigureStreamsAndStartSensor());
    }

    private IEnumerator ConfigureStreamsAndStartSensor()
    {
        var configOp = pixelSensorFeature.ConfigureSensor(sensorId.Value, configuredStreams.ToArray());
        
        yield return configOp;
        
        if (!configOp.DidOperationSucceed) 
        { 
            Debug.LogError("Failed to configure sensor."); 
            yield break; 
        }

        var metaTypes = new Dictionary<uint, PixelSensorMetaDataType[]>();

        foreach (uint stream in configuredStreams)
            if (pixelSensorFeature.EnumeratePixelSensorMetaDataTypes(sensorId.Value, stream, out var t))
                metaTypes[stream] = t;

        var startOp = pixelSensorFeature.StartSensor(sensorId.Value, configuredStreams, metaTypes);

        yield return startOp;

        if (startOp.DidOperationSucceed)
        {
            streamVisualizer.Initialize(configuredStreams[0], pixelSensorFeature, sensorId.Value);
            streamVisualizer.StartWorker();   // start background tracking thread
            StartCoroutine(MonitorSensorData());
        }
        else 
        { 
            Debug.LogError("Failed to start sensor."); 
        }
    }

    private IEnumerator MonitorSensorData()
    {
        long submittedSamples = 0;
        long totalGetSensorDataTicks = 0;
        long totalByteCopyTicks = 0;
        long totalGetSensorPoseTicks = 0;
        long totalSubmitTicks = 0;

        while (pixelSensorFeature.GetSensorStatus(sensorId.Value) == PixelSensorStatus.Started)
        {
            foreach (uint stream in configuredStreams)
            {
                long getSensorDataStart = Stopwatch.GetTimestamp();
                bool hasFrame = pixelSensorFeature.GetSensorData(
                    sensorId.Value, stream,
                    out var frame, out var metaData,
                    Allocator.Temp, shouldFlipTexture: true);
                long getSensorDataEnd = Stopwatch.GetTimestamp();

                if (hasFrame && frame.IsValid && frame.Planes.Length > 0)
                {
                    var plane = frame.Planes[0];
                    int byteLen = plane.ByteData.Length;
                    int w = (int)plane.Width;
                    int h = (int)plane.Height;

                    // Copy once from the temporary NativeArray into a worker-owned buffer.
                    byte[] frameBytes = streamVisualizer.AcquireFrameBuffer(byteLen);
                    long byteCopyStart = Stopwatch.GetTimestamp();
                    NativeArray<byte>.Copy(plane.ByteData, frameBytes, byteLen);
                    long byteCopyEnd = Stopwatch.GetTimestamp();

                    // Pose must also be captured here — valid only while frame is alive
                    long getSensorPoseStart = Stopwatch.GetTimestamp();
                    Pose sensorPose = pixelSensorFeature.GetSensorPose(sensorId.Value, frame.CaptureTime);
                    
                    if (xrOrigin)
                    {
                        var baseTf = xrOrigin.CameraFloorOffsetObject.transform;
                        sensorPose.position = baseTf.TransformPoint(sensorPose.position);
                        sensorPose.rotation = xrOrigin.transform.rotation * sensorPose.rotation;
                    }
                    long getSensorPoseEnd = Stopwatch.GetTimestamp();

                    // Hand off to background thread — returns immediately
                    long submitStart = Stopwatch.GetTimestamp();
                    streamVisualizer.SubmitRawFrame(frameBytes, w, h, sensorPose);
                    long submitEnd = Stopwatch.GetTimestamp();

                    totalGetSensorDataTicks += getSensorDataEnd - getSensorDataStart;
                    totalByteCopyTicks += byteCopyEnd - byteCopyStart;
                    totalGetSensorPoseTicks += getSensorPoseEnd - getSensorPoseStart;
                    totalSubmitTicks += submitEnd - submitStart;
                    submittedSamples++;

                    if (submittedSamples % TimingLogEveryNFrames == 0)
                    {
                        double tickToMs = 1000.0 / Stopwatch.Frequency;
                        Debug.Log(
                            $"[Timing][Producer] Avg over {submittedSamples} submitted frames: " +
                            $"GetSensorData={(totalGetSensorDataTicks / (double)submittedSamples) * tickToMs:F2} ms, " +
                            $"ByteCopy={(totalByteCopyTicks / (double)submittedSamples) * tickToMs:F2} ms, " +
                            $"GetSensorPose={(totalGetSensorPoseTicks / (double)submittedSamples) * tickToMs:F2} ms, " +
                            $"SubmitRawFrame={(totalSubmitTicks / (double)submittedSamples) * tickToMs:F2} ms");

                        submittedSamples = 0;
                        totalGetSensorDataTicks = 0;
                        totalByteCopyTicks = 0;
                        totalGetSensorPoseTicks = 0;
                        totalSubmitTicks = 0;
                    }
                }

                yield return null;   // yields AFTER submit, not after ProcessFrame
            }
        }
    }

    public void OnDisable()
    {
        if (pixelSensorFeature != null)
        {
            pixelSensorFeature.OnSensorAvailabilityChanged -= OnSensorAvailabilityChanged;
        }

        streamVisualizer?.StopWorker();

        var runner = Camera.main ? Camera.main.GetComponent<MonoBehaviour>() : null;
       
        if (runner != null) 
        {
            runner.StartCoroutine(StopSensorCoroutine());
        }
    }

    private IEnumerator StopSensorCoroutine()
    {
        if (!sensorId.HasValue) 
        { 
            yield break; 
        }
        
        var stopOp = pixelSensorFeature.StopSensor(sensorId.Value, configuredStreams);
        
        yield return stopOp;
        
        if (stopOp.DidOperationSucceed)
        {
            pixelSensorFeature.ClearAllAppliedConfigs(sensorId.Value);
            pixelSensorFeature.DestroyPixelSensor(sensorId.Value);
        }
    }
}
