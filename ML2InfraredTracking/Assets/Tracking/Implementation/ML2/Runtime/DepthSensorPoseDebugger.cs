using System.Reflection;
using MagicLeap.OpenXR.Features;
using MagicLeap.OpenXR.Features.PixelSensors;
using Unity.XR.CoreUtils;
using UnityEngine;

/// <summary>
/// Runtime XR debug view of the depth-camera extrinsic.
///
/// <b>White sphere + RGB axes</b> track a <i>display-rate</i> GetSensorPose poll
/// (should stick to the physical depth camera as you move your head).
///
/// <b>Cyan ghost sphere</b> is the last <i>depth-frame-synced</i> pose packed into TCP
/// (updates only when a depth frame arrives — often much slower, e.g. ~5 Hz).
///
/// If white tracks the headset but cyan lags, extrinsics are fine and the old
/// "slow follow" was just depth FPS. If white itself drifts/fails to pitch, the
/// GetSensorPose / XROrigin path is wrong.
/// </summary>
public sealed class DepthSensorPoseDebugger : MonoBehaviour
{
    [Header("Visibility")]
    [SerializeField] private bool showAxes = true;
    [SerializeField] private bool showFrustum = true;
    [SerializeField] private bool showLiveOrigin = true;
    [SerializeField] private bool showFrameSyncedGhost = true;

    [Header("Sizes (metres)")]
    [SerializeField, Min(0.01f)] private float axisLength = 0.08f;
    [SerializeField, Min(0.05f)] private float frustumDepth = 0.35f;
    [SerializeField, Min(0.001f)] private float lineWidth = 0.003f;
    [SerializeField, Min(0.001f)] private float originSphereRadius = 0.008f;
    [SerializeField, Min(0.001f)] private float ghostSphereRadius = 0.012f;

    [Header("Colors")]
    [SerializeField] private Color axisXColor = Color.red;
    [SerializeField] private Color axisYColor = Color.green;
    [SerializeField] private Color axisZColor = Color.blue;
    [SerializeField] private Color frustumColor = new Color(1f, 0.85f, 0.2f, 1f);
    [SerializeField] private Color liveOriginColor = new Color(1f, 1f, 1f, 0.95f);
    [SerializeField] private Color frameSyncedGhostColor = new Color(0.2f, 0.9f, 1f, 0.7f);

    private Transform _liveRoot;
    private Transform _ghostRoot;
    private LineRenderer _axisX;
    private LineRenderer _axisY;
    private LineRenderer _axisZ;
    private LineRenderer _frustum;
    private Transform _liveSphere;
    private Transform _ghostSphere;
    private bool _built;
    private bool _hasLivePose;
    private bool _hasFramePose;
    private Pose _livePose;
    private Pose _framePose;
    private DepthCameraIntrinsics? _intrinsics;
    private int _imageWidth;
    private int _imageHeight;

    public Pose LatestLivePose => _livePose;
    public Pose LatestFrameSyncedPose => _framePose;
    public bool HasLivePose => _hasLivePose;

    private void OnEnable()
    {
        EnsureBuilt();
        ApplyVisibility();
    }

    private void OnDisable()
    {
        if (_liveRoot != null)
            _liveRoot.gameObject.SetActive(false);
        if (_ghostRoot != null)
            _ghostRoot.gameObject.SetActive(false);
    }

    private void LateUpdate()
    {
        if (!_built)
            return;

        if (_hasLivePose && _liveRoot != null)
        {
            _liveRoot.SetPositionAndRotation(_livePose.position, _livePose.rotation);
            RefreshLiveGeometry();
        }

        if (_hasFramePose && _ghostRoot != null)
            _ghostRoot.SetPositionAndRotation(_framePose.position, _framePose.rotation);

        ApplyVisibility();
    }

    /// <summary>Display-rate extrinsic (should follow the headset smoothly).</summary>
    public void SubmitLive(in Pose sensorPose)
    {
        EnsureBuilt();
        _livePose = sensorPose;
        _hasLivePose = true;
    }

    /// <summary>Same pose packed into the depth TCP header (updates on depth frames only).</summary>
    public void SubmitFrameSynced(in Pose sensorPose, DepthCameraIntrinsics? intrinsics, int width, int height)
    {
        EnsureBuilt();
        _framePose = sensorPose;
        _intrinsics = intrinsics;
        _imageWidth = width;
        _imageHeight = height;
        _hasFramePose = true;
        // Keep live pose fresh even if LateUpdate poll is unavailable.
        if (!_hasLivePose)
            SubmitLive(sensorPose);
    }

    /// <summary>Backward-compatible alias for frame-synced submits.</summary>
    public void Submit(in Pose sensorPose, DepthCameraIntrinsics? intrinsics, int width, int height) =>
        SubmitFrameSynced(sensorPose, intrinsics, width, height);

    private void EnsureBuilt()
    {
        if (_built)
            return;

        _liveRoot = new GameObject("DepthSensorPoseDebug_Live").transform;
        _liveRoot.SetParent(transform, false);

        _ghostRoot = new GameObject("DepthSensorPoseDebug_FrameSynced").transform;
        _ghostRoot.SetParent(transform, false);

        _axisX = CreateLine(_liveRoot, "AxisX", axisXColor);
        _axisY = CreateLine(_liveRoot, "AxisY", axisYColor);
        _axisZ = CreateLine(_liveRoot, "AxisZ", axisZColor);
        _frustum = CreateLine(_liveRoot, "Frustum", frustumColor);

        _liveSphere = CreateSphere(_liveRoot, "LiveOrigin", originSphereRadius, liveOriginColor);
        _ghostSphere = CreateSphere(_ghostRoot, "FrameSyncedOrigin", ghostSphereRadius, frameSyncedGhostColor);

        _built = true;
        RefreshLiveGeometry();
        ApplyVisibility();
    }

    private void RefreshLiveGeometry()
    {
        if (_axisX != null)
        {
            _axisX.enabled = showAxes;
            if (showAxes)
                SetLocalSegment(_axisX, _liveRoot, Vector3.zero, Vector3.right * axisLength);
        }

        if (_axisY != null)
        {
            _axisY.enabled = showAxes;
            if (showAxes)
                SetLocalSegment(_axisY, _liveRoot, Vector3.zero, Vector3.up * axisLength);
        }

        if (_axisZ != null)
        {
            _axisZ.enabled = showAxes;
            if (showAxes)
                SetLocalSegment(_axisZ, _liveRoot, Vector3.zero, Vector3.forward * axisLength);
        }

        if (_frustum != null)
        {
            _frustum.enabled = showFrustum;
            if (showFrustum)
                DrawFrustum(_frustum, _liveRoot);
        }
    }

    private void ApplyVisibility()
    {
        if (_liveRoot != null)
            _liveRoot.gameObject.SetActive(_hasLivePose && (showAxes || showFrustum || showLiveOrigin));
        if (_liveSphere != null)
            _liveSphere.gameObject.SetActive(showLiveOrigin);
        if (_ghostRoot != null)
            _ghostRoot.gameObject.SetActive(_hasFramePose && showFrameSyncedGhost);
    }

    private void DrawFrustum(LineRenderer line, Transform root)
    {
        GetFrustumExtents(out float halfWidth, out float halfHeight);
        float z = frustumDepth;
        Vector3 c0 = new Vector3(-halfWidth, -halfHeight, z);
        Vector3 c1 = new Vector3(halfWidth, -halfHeight, z);
        Vector3 c2 = new Vector3(halfWidth, halfHeight, z);
        Vector3 c3 = new Vector3(-halfWidth, halfHeight, z);
        Vector3 o = Vector3.zero;

        Vector3[] points =
        {
            o, c0, o, c1, o, c2, o, c3,
            c0, c1, c2, c3, c0
        };
        line.positionCount = points.Length;
        for (int i = 0; i < points.Length; i++)
            line.SetPosition(i, root.TransformPoint(points[i]));
    }

    private void GetFrustumExtents(out float halfWidth, out float halfHeight)
    {
        float z = frustumDepth;
        if (_intrinsics.HasValue)
        {
            var k = _intrinsics.Value;
            if (_imageWidth > 0 && _imageHeight > 0 && k.Fx > 1e-6 && k.Fy > 1e-6)
            {
                halfWidth = 0.5f * _imageWidth / (float)k.Fx * z;
                halfHeight = 0.5f * _imageHeight / (float)k.Fy * z;
                return;
            }

            if (k.FovX > 1e-3 && k.FovY > 1e-3)
            {
                halfWidth = z * Mathf.Tan(0.5f * (float)k.FovX * Mathf.Deg2Rad);
                halfHeight = z * Mathf.Tan(0.5f * (float)k.FovY * Mathf.Deg2Rad);
                return;
            }
        }

        halfWidth = z * Mathf.Tan(30f * Mathf.Deg2Rad);
        halfHeight = z * Mathf.Tan(24f * Mathf.Deg2Rad);
    }

    private static void SetLocalSegment(LineRenderer line, Transform root, Vector3 localA, Vector3 localB)
    {
        line.positionCount = 2;
        line.SetPosition(0, root.TransformPoint(localA));
        line.SetPosition(1, root.TransformPoint(localB));
    }

    private LineRenderer CreateLine(Transform parent, string name, Color color)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, false);
        var line = go.AddComponent<LineRenderer>();
        line.sharedMaterial = CreateColorMaterial(color);
        line.startColor = color;
        line.endColor = color;
        line.startWidth = lineWidth;
        line.endWidth = lineWidth;
        line.useWorldSpace = true;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.receiveShadows = false;
        line.loop = false;
        line.positionCount = 0;
        return line;
    }

    private static Transform CreateSphere(Transform parent, string name, float radius, Color color)
    {
        var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.name = name;
        var t = sphere.transform;
        t.SetParent(parent, false);
        t.localPosition = Vector3.zero;
        float d = radius * 2f;
        t.localScale = new Vector3(d, d, d);
        var col = sphere.GetComponent<Collider>();
        if (col != null)
            Destroy(col);
        var renderer = sphere.GetComponent<Renderer>();
        if (renderer != null)
            renderer.sharedMaterial = CreateColorMaterial(color);
        return t;
    }

    private static Material CreateColorMaterial(Color color)
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit")
                        ?? Shader.Find("Unlit/Color")
                        ?? Shader.Find("Sprites/Default")
                        ?? Shader.Find("Standard");
        var material = new Material(shader);
        if (material.HasProperty("_BaseColor"))
            material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color"))
            material.SetColor("_Color", color);
        return material;
    }
}

/// <summary>
/// Helpers shared by depth streaming + pose debug for XROrigin conversion and live pose time.
/// </summary>
public static class DepthSensorPoseUtil
{
    private static FieldInfo _predictedDisplayTimeField;

    public static Pose ToWorldPose(in Pose trackingPose, XROrigin xrOrigin)
    {
        if (xrOrigin == null)
            return trackingPose;

        // Use the same transform for position and rotation. The old split
        // (floor offset for position, xrOrigin for rotation) can drop pitch/roll
        // when those transforms differ.
        Transform originTf = xrOrigin.CameraFloorOffsetObject != null
            ? xrOrigin.CameraFloorOffsetObject.transform
            : xrOrigin.transform;

        return new Pose(
            originTf.TransformPoint(trackingPose.position),
            originTf.rotation * trackingPose.rotation);
    }

    public static bool TryGetPredictedDisplayTime(out long predictedDisplayTime)
    {
        predictedDisplayTime = 0;
        if (_predictedDisplayTimeField == null)
        {
            _predictedDisplayTimeField = typeof(MagicLeapOpenXRFeatureBase).GetField(
                "PredictedDisplayTime",
                BindingFlags.Static | BindingFlags.NonPublic);
        }

        if (_predictedDisplayTimeField == null)
            return false;

        object value = _predictedDisplayTimeField.GetValue(null);
        if (value is long time && time != 0)
        {
            predictedDisplayTime = time;
            return true;
        }

        return false;
    }

    public static bool TryGetLiveSensorPose(
        MagicLeapPixelSensorFeature feature,
        PixelSensorId sensorId,
        XROrigin xrOrigin,
        out Pose worldPose)
    {
        worldPose = default;
        if (!TryGetLiveSensorTrackingPose(feature, sensorId, out Pose trackingPose, out _))
            return false;

        worldPose = ToWorldPose(trackingPose, xrOrigin);
        return true;
    }

    /// <summary>
    /// Same live (NextPredictedDisplayTime) sensor pose as <see cref="TryGetLiveSensorPose"/>,
    /// but in raw tracking space (no xrOrigin applied) plus the XrTime it was sampled at -
    /// what SensorPoseHistory needs to record a consistent, always-valid timeline.
    /// </summary>
    public static bool TryGetLiveSensorTrackingPose(
        MagicLeapPixelSensorFeature feature,
        PixelSensorId sensorId,
        out Pose trackingPose,
        out long time)
    {
        trackingPose = default;
        time = 0;
        if (feature == null)
            return false;

        if (!TryGetPredictedDisplayTime(out time))
            return false;

        // Try*, not GetSensorPose: a failed XrLocateSpace here must not look like a
        // fresh sample to the caller - SensorPoseHistory only records what this
        // returns true for, and recording a held/stale pose under a fresh timestamp
        // is what was causing the flicker-between-stale-poses symptom.
        return feature.TryGetSensorPose(sensorId, time, out trackingPose);
    }
}
