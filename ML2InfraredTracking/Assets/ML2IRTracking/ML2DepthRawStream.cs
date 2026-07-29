using UnityEngine;
using System;
using MagicLeap.OpenXR.Features.PixelSensors;
public readonly struct DepthCameraIntrinsics
{
    public const int BinarySize = 11 * sizeof(double);

    public readonly double Fx, Fy, Cx, Cy, FovX, FovY;
    public readonly double K1, K2, P1, P2, K3;

    public DepthCameraIntrinsics(Vector2 focalLength, Vector2 principalPoint, Vector2 fov,
        double[] distortion)
    {
        Fx = focalLength.x;
        Fy = focalLength.y;
        Cx = principalPoint.x;
        Cy = principalPoint.y;
        FovX = fov.x;
        FovY = fov.y;
        K1 = GetDistortion(distortion, 0);
        K2 = GetDistortion(distortion, 1);
        P1 = GetDistortion(distortion, 2);
        P2 = GetDistortion(distortion, 3);
        K3 = GetDistortion(distortion, 4);
    }

    private static double GetDistortion(double[] values, int index) =>
        values != null && index < values.Length ? values[index] : 0.0;

    public override string ToString() =>
        $"Pinhole intrinsics\nFocal: ({Fx:F3}, {Fy:F3}) px\n" +
        $"Principal: ({Cx:F3}, {Cy:F3}) px\nFOV: ({FovX:F3}, {FovY:F3}) deg\n" +
        $"Distortion: [{K1:F6}, {K2:F6}, {P1:F6}, {P2:F6}, {K3:F6}]";
}

public class ML2DepthRawStream : MonoBehaviour
{
    [Header("Tracking Data")]
    public Renderer targetRenderer;          // Renderer that previews the depth texture
    [SerializeField] private PixelSensorMaterialTable materialList = new();
    [Tooltip("Optional. If omitted, a DepthFrameTcpServer is added at runtime with its default settings.")]
    [SerializeField] private DepthFrameTcpServer depthTcpServer;
    [Tooltip("Optional. Visualizes the same world_T_sensor packed into each depth TCP frame.")]
    [SerializeField] private DepthSensorPoseDebugger sensorPoseDebugger;

    Texture2D targetTexture, filteredTexture;

    private float[] _floatBuffer;                        // CPU-side float[] depth map (meters) 
    private DepthCameraIntrinsics? intrinsics;

    public DepthSensorPoseDebugger SensorPoseDebugger => sensorPoseDebugger;

    // --- Rendering helpers ---
    private float minDepth = 0, maxDepth = 5;
    private bool _loggedFloatFormat;
    private bool _loggedUnexpectedBuffer;

    // ----------------- Helpers -----------------

    // Converts Magic Leap raw plane into a float[] depth map in meters.
    private float[] GetRawDepthData(in PixelSensorFrame frame, ref float[] buffer)
    {
        if (frame.Planes.Length == 0) return null;
        var firstPlane = frame.Planes[0];

        int size = (int)(firstPlane.Width * firstPlane.Height);
        if (size <= 0)
        {
            Debug.LogWarning("[ML2Tracking] Invalid frame size.");
            return null;
        }

        if (firstPlane.ByteData.Length % sizeof(float) == 0)
        {
            var asFloat = firstPlane.ByteData.Reinterpret<float>(sizeof(float));
            if (buffer == null || buffer.Length != size) buffer = new float[size];

            if (asFloat.Length == size)
            {
                asFloat.CopyTo(buffer);
                LogFloatFormat(firstPlane, false);
                return buffer;
            }

            int rowStrideFloats = firstPlane.Stride % sizeof(float) == 0
                ? (int)firstPlane.Stride / sizeof(float)
                : 0;
            if (rowStrideFloats >= (int)firstPlane.Width &&
                asFloat.Length >= rowStrideFloats * (int)firstPlane.Height)
            {
                int width = (int)firstPlane.Width;
                int height = (int)firstPlane.Height;
                for (int y = 0; y < height; y++)
                {
                    int sourceOffset = y * rowStrideFloats;
                    int targetOffset = y * width;
                    for (int x = 0; x < width; x++)
                        buffer[targetOffset + x] = asFloat[sourceOffset + x];
                }

                LogFloatFormat(firstPlane, true);
                return buffer;
            }
        }

        if (!_loggedUnexpectedBuffer)
        {
            Debug.LogError($"[ML2Tracking] Cannot stream DepthRaw plane as FLOAT32: " +
                           $"{firstPlane.Width}x{firstPlane.Height}, bytes={firstPlane.ByteData.Length}, " +
                           $"bytesPerPixel={firstPlane.BytesPerPixel}, stride={firstPlane.Stride}.");
            _loggedUnexpectedBuffer = true;
        }

        return null;
    }

    private void LogFloatFormat(in PixelSensorPlane plane, bool removedRowPadding)
    {
        if (_loggedFloatFormat) return;

        Debug.Log($"[ML2Tracking] Depth interpreted as FLOAT32" +
                  $"{(removedRowPadding ? " (row padding removed)" : string.Empty)}. " +
                  $"{plane.Width}x{plane.Height}, stride={plane.Stride}.");
        _loggedFloatFormat = true;
    }

    private void CaptureIntrinsicsOnce(in PixelSensorMetaData[] metaData)
    {
        if (intrinsics.HasValue) return;

        foreach (var entry in metaData)
        {
            if (entry is not PixelSensorPinholeIntrinsics pinhole) continue;

            intrinsics = new DepthCameraIntrinsics(pinhole.FocalLength, pinhole.PrincipalPoint,
                pinhole.FOV, pinhole.Distortion);
            Debug.Log($"[ML2Tracking] Captured intrinsics for one-time transmission.\n{intrinsics}");
            return;
        }
    }

    private void Start()
    {
        Debug.Log("[ML2Tracking] Start() - initializing preview, native handles and calibration.");

        // OnEnable runs before Start, so this also prevents a stale serialized
        // reference from feeding a duplicate server that could not bind the port.
        if (DepthFrameTcpServer.ActiveServer != null)
            depthTcpServer = DepthFrameTcpServer.ActiveServer;
        else if (depthTcpServer == null)
            depthTcpServer = GetComponent<DepthFrameTcpServer>();
        if (depthTcpServer == null)
            depthTcpServer = gameObject.AddComponent<DepthFrameTcpServer>();

        if (sensorPoseDebugger == null)
            sensorPoseDebugger = GetComponent<DepthSensorPoseDebugger>();
        if (sensorPoseDebugger == null)
            sensorPoseDebugger = gameObject.AddComponent<DepthSensorPoseDebugger>();

        // Assign the material 
        var mat = materialList.GetMaterialForFrameType(PixelSensorFrameType.DepthRaw);
        if (targetRenderer) targetRenderer.sharedMaterial = mat;

        Debug.Log("[ML2Tracking] Initialization completed.");
    }


    public void Initialize(uint streamId, MagicLeapPixelSensorFeature feature, PixelSensorId sensorType)
    {
        if (!sensorType.SensorName.Contains("depth", StringComparison.CurrentCultureIgnoreCase)) return;

        if (feature.QueryPixelSensorCapability(sensorType, PixelSensorCapabilityType.Depth, streamId, out var range))
        {
            if (range.IntRange.HasValue) { minDepth = range.IntRange.Value.Min; maxDepth = range.IntRange.Value.Max; }
            if (range.FloatRange.HasValue) { minDepth = range.FloatRange.Value.Min; maxDepth = range.FloatRange.Value.Max; }
            Debug.Log($"[ML2Tracking] Depth capability range: {minDepth:F3}..{maxDepth:F3} meters.");
        }
        else
        {
            Debug.LogWarning("[ML2Tracking] QueryPixelSensorCapability(Depth) failed.");
        }
    }

    public void Reset()
    {
        if (targetTexture) { Destroy(targetTexture); targetTexture = null; }
        if (filteredTexture) { Destroy(filteredTexture); filteredTexture = null; }
    }

    private void OnDestroy()
    {
        Reset();
        if (targetRenderer && targetRenderer.material) Destroy(targetRenderer.material);

        Debug.Log("[ML2Tracking] Cleanup completed (textures, material, native handles).");
    }

    // ----------------- Main per-frame processing -----------------
    public void ProcessFrame(in PixelSensorFrame frame, in PixelSensorMetaData[] metaData, in Pose sensorPose)
    {
        if (!frame.IsValid || frame.Planes.Length == 0) return;
        if (frame.FrameType != PixelSensorFrameType.DepthRaw) return;

        var frameType = frame.FrameType;
        var firstPlane = frame.Planes[0];
        int w = (int)firstPlane.Width;
        int h = (int)firstPlane.Height;

        // Ensure preview texture and upload raw plane 
        Utils.EnsureTargetTexture(ref targetTexture, frameType, w, h);
        Utils.UploadMainTexture(frameType, ref firstPlane, targetTexture);

        switch (frameType)
        {
            case PixelSensorFrameType.DepthRaw:
                {
                    CaptureIntrinsicsOnce(metaData);

                    // Build a CPU float[] depth map (meters) for the native pipeline
                    var depthData = GetRawDepthData(in frame, ref _floatBuffer);
                    if (depthData == null) return;

                    depthTcpServer.SubmitFrame(depthData, w, h, sensorPose, intrinsics);
                    if (sensorPoseDebugger != null)
                        sensorPoseDebugger.SubmitFrameSynced(sensorPose, intrinsics, w, h);

                    if (targetRenderer != null)
                        targetRenderer.material.mainTexture = targetTexture;

                    //Matrix4x4 worldTsensor = Matrix4x4.TRS(sensorPose.position, sensorPose.rotation, Vector3.one);
                }
                break;
            default:
                Debug.LogWarning($"[ML2Tracking] Unsupported frame type: {frameType}");
                break;
        }
    }

    private void Update()
    {
    }
}
