using UnityEngine;
using System;
using System.Collections.Generic;
using MagicLeap.OpenXR.Features.PixelSensors;

public class ML2DepthRawStream : MonoBehaviour
{
    [Header("Tracking Data")]
    public Renderer targetRenderer;          // Renderer that previews the depth texture
    [SerializeField] private PixelSensorMaterialTable materialList = new();

    Texture2D targetTexture, filteredTexture;

    private float[] _cameraMatrixNative = new float[9];  // 3x3 fx,fy,cx,cy
    private float[] _distCoeffsNative;                   // k1,k2,p1,p2,k3
    private float[] _floatBuffer;                        // CPU-side float[] depth map (meters) 

    // --- Rendering helpers ---
    private MaterialPropertyBlock _mpb;
    private float minDepth = 0, maxDepth = 5;

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

        var asFloat = firstPlane.ByteData.Reinterpret<float>(sizeof(float));
        if (asFloat.Length == size)
        {
            if (buffer == null || buffer.Length != size) buffer = new float[size];
            asFloat.CopyTo(buffer);
            Debug.LogWarning("[ML2Tracking] Depth interpreted as FLOAT32.");
            return buffer;
        }

       
        return null;
    }

    private void Start()
    {
        Debug.Log("[ML2Tracking] Start() - initializing preview, native handles and calibration.");

        _mpb = new MaterialPropertyBlock();

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
                    // Build a CPU float[] depth map (meters) for the native pipeline
                    var depthData = GetRawDepthData(in frame, ref _floatBuffer);
                    if (depthData == null) return;


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
