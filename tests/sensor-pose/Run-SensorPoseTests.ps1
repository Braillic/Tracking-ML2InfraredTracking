param(
    [string]$SourceRoot = (Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'ML2InfraredTracking')
)
$ErrorActionPreference = 'Stop'

function Get-MethodSource([string]$Source, [string]$Signature) {
    $start = $Source.IndexOf($Signature, [StringComparison]::Ordinal)
    if ($start -lt 0) { throw "Missing method: $Signature" }
    $brace = $Source.IndexOf('{', $start)
    $depth = 0
    for ($i = $brace; $i -lt $Source.Length; $i++) {
        if ($Source[$i] -eq '{') { $depth++ }
        if ($Source[$i] -eq '}') {
            $depth--
            if ($depth -eq 0) { return $Source.Substring($start, $i - $start + 1) }
        }
    }
    throw "Unbalanced method: $Signature"
}

# Execute production method bodies with controllable XR responses and clocks.
# Math uses System.Numerics in the shim; separately compile the application with Unity references.
$captureSource = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Assets/ML2IRTracking/DepthSensorAPI.cs')
$sdkSource = Get-Content -Raw -LiteralPath (Join-Path $SourceRoot 'Packages/com.magicleap.unitysdk@2.6.0/Runtime/OpenXR/Common/Spaces.cs')
$captureMethod = Get-MethodSource $captureSource 'private bool TryResolveCapturePose('
$sdkMethod = Get-MethodSource $sdkSource 'internal bool TryGetUnityPose('
$harness = @'
using System;
using UnityEngine;
public sealed class FakeSensorFeature {
    public bool Ok;
    public Pose Result = Pose.identity;
    public int Calls;
    public bool TryGetSensorPose(int id,long time,out Pose pose) {Calls++; pose=Result; return Ok;}
}
public sealed class CaptureHarness {
    public SensorPoseHistory sensorPoseHistory = new SensorPoseHistory();
    public FakeSensorFeature pixelSensorFeature = new FakeSensorFeature();
    public int? sensorId = 1;
    public long _minimumCaptureTime = 0;
    public double _lastLiveSampleRealtime = double.NegativeInfinity;
    private const double MaxLiveSampleAgeSeconds = 0.1;
    public int _droppedPoseCount, _historyPoseCount, _historyMissCount, _directPoseCount;
    public SensorPoseHistory.LookupStatus _lastLookupStatus;
    public bool Resolve(long time,out Pose pose) => TryResolveCapturePose(time,out pose);
__CAPTURE__
}
[Flags]
public enum XrSpaceLocationFlagsML : ulong {
    OrientationValid=1, PositionValid=2, OrientationTracked=4, PositionTracked=8
}
public struct XrPose { public Vector3 Position; public Quaternion Rotation; }
public struct XrSpaceLocation {
    public const ulong XrSpaceLocationStructType=42;
    public ulong Type;
    public XrSpaceLocationFlagsML SpaceLocationFlags;
    public XrPose Pose;
}
public static class Utils { public static bool DidXrCallSucceed(bool result,string name) => result; }
public sealed class SdkHarness {
    public bool Ok=true;
    public XrSpaceLocationFlagsML Flags;
    bool XrLocateSpace(ulong space,ulong baseSpace,long time,out XrSpaceLocation result) {
        result=new XrSpaceLocation {SpaceLocationFlags=Flags,Pose=new XrPose {
            Position=new Vector3(3,4,5),Rotation=Quaternion.identity}};
        return Ok;
    }
    public bool Locate(out Pose pose) => TryGetUnityPose(1,2,3,out pose);
__SDK__
}
'@
$harness = $harness.Replace('__CAPTURE__', $captureMethod).Replace('__SDK__', $sdkMethod)
$generated = Join-Path ([IO.Path]::GetTempPath()) ("ML2SensorPoseTests-" + [Guid]::NewGuid().ToString('N') + '.cs')
try {
    [IO.File]::WriteAllText($generated, $harness)
    Add-Type -Path @(
        (Join-Path $SourceRoot 'Assets/ML2IRTracking/SensorPoseHistory.cs'),
        (Join-Path $PSScriptRoot 'UnityMathShim.cs'),
        (Join-Path $PSScriptRoot 'SensorPoseRegressionTests.cs'),
        $generated
    )
    [SensorPoseRegressionTests]::Run()
} finally {
    if (Test-Path -LiteralPath $generated) { Remove-Item -LiteralPath $generated }
}
