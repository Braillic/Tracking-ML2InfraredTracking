using System.Collections;
using System.Collections.Generic;
using System.Linq;
using MagicLeap.Android;
using Unity.Collections;
using UnityEngine;
using UnityEngine.XR.MagicLeap;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR;
using MagicLeap.OpenXR.Features.PixelSensors;
using Unity.XR.CoreUtils;
using System.Text;
using System;
using Braillic.Tracking.Runtime;
using Braillic.Tracking.Unity;


public class DepthSensorAPI : MonoBehaviour, ICaptureDevice
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

    // Keep SDK poses at their original timestamps. Additional smoothing here
    // delays head motion relative to the image used by desktop PnP.
    // History accepts exact samples or interpolation across at most 100 ms.
    private readonly SensorPoseHistory sensorPoseHistory = new SensorPoseHistory(2.0);
    private readonly List<XRInputSubsystem> inputSubsystems = new List<XRInputSubsystem>();
    private int _lastLiveSampleFrame = -1;
    private long _lastLiveSampleXrTime;
    private long _minimumCaptureTime;
    private double _lastLiveSampleRealtime = double.NegativeInfinity;
    private const double MaxLiveSampleAgeSeconds = 0.1;
    private bool _applicationPaused;

    // Temporary diagnostics for tuning the history-based pose lookup. Remove once
    // the flicker/lag under motion is understood.
    private int _historyMissCount;
    private int _liveUpdateOkCount;
    private int _liveUpdateFailCount;
    private int _historyPoseCount;
    private int _directPoseCount;
    private int _droppedPoseCount;
    private SensorPoseHistory.LookupStatus _lastLookupStatus;
    private float _lastHistoryDiagLog;
    private long _lastDeliveredCaptureTime;
    private long _captureIntervalSum;
    private int _deliveredFrames, _captureIntervals;

    private void OnEnable()
    {
        ResetSensorPoseHistory();
        SubscribeToTrackingOriginChanges();
    }

    private void SubscribeToTrackingOriginChanges()
    {
        foreach (var subsystem in inputSubsystems)
            subsystem.trackingOriginUpdated -= OnTrackingOriginUpdated;
        SubsystemManager.GetSubsystems(inputSubsystems);
        foreach (var subsystem in inputSubsystems)
            subsystem.trackingOriginUpdated += OnTrackingOriginUpdated;
    }

    private void OnTrackingOriginUpdated(XRInputSubsystem subsystem) => ResetSensorPoseHistory();

    private void OnApplicationPause(bool paused)
    {
        _applicationPaused = paused;
        ResetSensorPoseHistory();
    }

    private void ResetSensorPoseHistory()
    {
        (managedLifecycle?trackingSender:DepthFrameTcpServer.ActiveServer)?.BeginCaptureEpoch();
        _lastDeliveredCaptureTime = 0;
        _captureIntervalSum = 0; _captureIntervals = 0; _deliveredFrames = 0;
        sensorPoseHistory.Clear();
        _lastLiveSampleFrame = -1;
        _lastLiveSampleXrTime = 0;
        _lastLiveSampleRealtime = double.NegativeInfinity;
        // Prevent an in-flight image from being paired across a tracking-origin
        // change. If XR has not started, the first successful live sample sets it.
        DepthSensorPoseUtil.TryGetPredictedDisplayTime(out _minimumCaptureTime);
    }

    public uint targetStream
    {
        get { return DepthRange > 1.0f ? (uint)0 : (uint)1; }
    }

    private bool managedLifecycle, permissionRequested, ownsSensor, streamInitialized;
    private bool? permissionGranted;
    private DepthFrameTcpServer trackingSender;
    private CaptureDeviceLifecycle lifecycle;
    public CaptureDeviceLifecycle Lifecycle => lifecycle ?? (lifecycle=new CaptureDeviceLifecycle(this));
    public void BindTrackingLifecycle(DepthFrameTcpServer sender)
    {
        managedLifecycle=true;trackingSender=sender;
        Lifecycle.Stop();
    }
    private void Start() { if(!managedLifecycle)Lifecycle.Start(); }
    private void Update() { if(!managedLifecycle)Lifecycle.Pump(); }
    private void OnPermissionGranted(string permission) { permissionGranted=true; }
    private void OnPermissionDenied(string permission) { permissionGranted=false; }

    bool ICaptureDevice.OwnsResource => ownsSensor;
    bool ICaptureDevice.IsStreaming => ownsSensor && sensorId.HasValue && pixelSensorFeature!=null &&
        pixelSensorFeature.GetSensorStatus(sensorId.Value)==PixelSensorStatus.Started;
    bool ICaptureDevice.Prepare()
    {
        if(pixelSensorFeature==null)
        {
            pixelSensorFeature=OpenXRSettings.Instance?.GetFeature<MagicLeapPixelSensorFeature>();
            if(pixelSensorFeature==null||!pixelSensorFeature.enabled)throw new InvalidOperationException("Pixel Sensor Feature is unavailable.");
        }
        if(!permissionRequested)
        {
            permissionRequested=true;
            Permissions.RequestPermission(MLPermission.DepthCamera,OnPermissionGranted,OnPermissionDenied,OnPermissionDenied);
        }
        if(!permissionGranted.HasValue)return false;
        if(!permissionGranted.Value)throw new InvalidOperationException("Depth camera permission denied.");
        if(!sensorId.HasValue)
            foreach(var sensor in pixelSensorFeature.GetSupportedSensors())
                if(sensor.XrPathString.Contains(depthCameraSensorPath)){sensorId=sensor;break;}
        if(!sensorId.HasValue)throw new InvalidOperationException("Depth pixel sensor not found.");
        if(!ownsSensor)
        {
            // Never release or adopt a sensor acquired by another owner.
            if(pixelSensorFeature.GetSensorStatus(sensorId.Value)!=PixelSensorStatus.Undefined)return false;
            if(!pixelSensorFeature.CreatePixelSensor(sensorId.Value))return false;
            ownsSensor=true;streamInitialized=false;
            ResetSensorPoseHistory();SubscribeToTrackingOriginChanges();
        }
        return true;
    }
    ICaptureOperation ICaptureDevice.Configure()
    {
        ConfigureSensorStreams();
        return new SensorOperation(pixelSensorFeature.ConfigureSensor(sensorId.Value,configuredStreams.ToArray()));
    }
    ICaptureOperation ICaptureDevice.Start()
    {
        var metadata=new Dictionary<uint,PixelSensorMetaDataType[]>();
        foreach(uint stream in configuredStreams)
            if(pixelSensorFeature.EnumeratePixelSensorMetaDataTypes(sensorId.Value,stream,out var types))metadata[stream]=types;
        return new SensorOperation(pixelSensorFeature.StartSensor(sensorId.Value,configuredStreams,metadata));
    }
    ICaptureOperation ICaptureDevice.Stop() => new SensorOperation(pixelSensorFeature.StopSensor(sensorId.Value,configuredStreams));
    void ICaptureDevice.Release()
    {
        if(!ownsSensor)return;
        pixelSensorFeature.ClearAllAppliedConfigs(sensorId.Value);
        if(!pixelSensorFeature.DestroyPixelSensor(sensorId.Value))throw new InvalidOperationException("Could not release depth pixel sensor.");
        ownsSensor=false;streamInitialized=false;
        ResetSensorPoseHistory();
    }
    void ICaptureDevice.Read()
    {
        if(_applicationPaused)return;
        if(streamVisualizer==null)throw new InvalidOperationException("Capture output adapter is missing.");
        if(!streamInitialized)
        {
            ResetSensorPoseHistory();
            streamVisualizer.Initialize(configuredStreams[0],pixelSensorFeature,sensorId.Value);
            streamInitialized=true;
        }
        PollSensorData();
    }
    private sealed class SensorOperation : ICaptureOperation
    {
        private readonly PixelSensorAsyncOperationResult operation;
        public SensorOperation(PixelSensorAsyncOperationResult operation) { this.operation=operation??throw new InvalidOperationException("Sensor operation missing."); }
        public bool IsCompleted => operation.DidOperationFinish;
        public bool Succeeded => operation.DidOperationSucceed;
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
            throw new InvalidOperationException("Sensor ID not set.");
        }

        uint streamCount = pixelSensorFeature.GetStreamCount(sensorId.Value);
        if (streamCount < 1)
        {
            throw new InvalidOperationException("Expected at least one stream from the sensor.");
        }

        // Only add the target, including when the sensor is recreated.
        configuredStreams.Clear();
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
                        throw new InvalidOperationException("Requested pixel sensor format unavailable.");
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

        // Configure/start completion is serialized by CaptureDeviceLifecycle.
    }



    private void PollSensorData()
    {
            SampleLiveSensorPose();

            foreach (uint stream in configuredStreams) 
            {

                double pollStart = Time.realtimeSinceStartupAsDouble;
                if (pixelSensorFeature.GetSensorData(sensorId.Value, stream, out var frame, out var metaData,
                        Allocator.Temp, shouldFlipTexture: true))
                {
                    double frameReady = Time.realtimeSinceStartupAsDouble;
                    if (frame.IsValid && frame.CaptureTime > _lastDeliveredCaptureTime)
                    {
                        _deliveredFrames++;
                        if (_lastDeliveredCaptureTime > 0)
                        { _captureIntervalSum += frame.CaptureTime - _lastDeliveredCaptureTime; _captureIntervals++; }
                        _lastDeliveredCaptureTime = frame.CaptureTime;
                    }
                    if (!frame.IsValid || !TryResolveCapturePose(frame.CaptureTime, out Pose sensorPose))
                        continue;
                    sensorPose = DepthSensorPoseUtil.ToWorldPose(sensorPose, xrOrigin);

                    streamVisualizer.ProcessFrame(frame, metaData, sensorPose, pollStart, frameReady);
                }
            }

            LogPoseDiagnostics();
    }

    private bool TryResolveCapturePose(long captureTime, out Pose sensorPose)
    {
        sensorPose = Pose.identity;
        if (captureTime <= 0 || (_minimumCaptureTime > 0 && captureTime < _minimumCaptureTime))
        {
            _droppedPoseCount++;
            return false;
        }

        // Prefer a bounded lookup when XR no longer retains the capture time.
        // An old history cannot remain usable after live tracking stops updating.
        if (Time.realtimeSinceStartupAsDouble - _lastLiveSampleRealtime > MaxLiveSampleAgeSeconds)
            sensorPoseHistory.Clear();
        if (sensorPoseHistory.TryGetPose(captureTime, out sensorPose, out _lastLookupStatus))
        {
            _historyPoseCount++;
            return true;
        }

        _historyMissCount++;
        if (pixelSensorFeature.TryGetSensorPose(sensorId.Value, captureTime, out sensorPose) &&
            SensorPoseHistory.IsValidPose(sensorPose))
        {
            sensorPose.rotation = sensorPose.rotation.normalized;
            _directPoseCount++;
            return true;
        }

        // TryGetSensorPose may write a cached pose on failure. Never use that out value.
        sensorPose = Pose.identity;
        _droppedPoseCount++;
        return false;
    }

    private void LogPoseDiagnostics()
    {
        if (Time.realtimeSinceStartup - _lastHistoryDiagLog <= 2f)
            return;
        float elapsed = Time.realtimeSinceStartup - _lastHistoryDiagLog;
        _lastHistoryDiagLog = Time.realtimeSinceStartup;
        Debug.Log($"[ML2SensorPoseHistoryDiag] samples={sensorPoseHistory.SampleCount} " +
                  $"spanMs={sensorPoseHistory.SpanTicks / 1e6:F1} historyMisses={_historyMissCount} " +
                  $"historyUsed={_historyPoseCount} directUsed={_directPoseCount} dropped={_droppedPoseCount} " +
                  $"lastLookup={_lastLookupStatus} liveUpdateOk={_liveUpdateOkCount} liveUpdateFail={_liveUpdateFailCount} " +
                  $"delivered_hz={_deliveredFrames / elapsed:F1} capture_period_ms=" +
                  (_captureIntervals > 0 ? (_captureIntervalSum / 1e6 / _captureIntervals).ToString("F1") : "unavailable"));
        _deliveredFrames = 0; _captureIntervalSum = 0; _captureIntervals = 0;
    }

    private void LateUpdate() => SampleLiveSensorPose();

    private void SampleLiveSensorPose()
    {
        if (_applicationPaused || !sensorId.HasValue || pixelSensorFeature == null ||
            pixelSensorFeature.GetSensorStatus(sensorId.Value) != PixelSensorStatus.Started ||
            _lastLiveSampleFrame == Time.frameCount)
            return;
        _lastLiveSampleFrame = Time.frameCount;

        // One query per Unity frame, shared by the producer and debug display.
        // Record the SDK pose directly; never smooth or extrapolate sensor motion.
        bool gotLivePose = DepthSensorPoseUtil.TryGetLiveSensorTrackingPose(
            pixelSensorFeature, sensorId.Value, out Pose trackingPose, out long time);
        gotLivePose = gotLivePose && time > 0 && SensorPoseHistory.IsValidPose(trackingPose);
        if (gotLivePose && time != _lastLiveSampleXrTime)
        {
            if (time < _lastLiveSampleXrTime)
            {
                ResetSensorPoseHistory();
                _minimumCaptureTime = time;
            }
            if (_minimumCaptureTime == 0)
                _minimumCaptureTime = time;
            sensorPoseHistory.Record(time, trackingPose);
            _lastLiveSampleXrTime = time;
            _lastLiveSampleRealtime = Time.realtimeSinceStartupAsDouble;
            _liveUpdateOkCount++;
        }
        else if (!gotLivePose)
        {
            sensorPoseHistory.Clear();
            _lastLiveSampleRealtime = double.NegativeInfinity;
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
        ResetSensorPoseHistory();
        Lifecycle.Stop();
        TrackingCleanupRunner.Drain(Lifecycle);
        foreach(var subsystem in inputSubsystems)subsystem.trackingOriginUpdated-=OnTrackingOriginUpdated;
        inputSubsystems.Clear();
    }

    private void OnDestroy()
    {
        Lifecycle.Stop();
        TrackingCleanupRunner.Drain(Lifecycle);
    }


}
