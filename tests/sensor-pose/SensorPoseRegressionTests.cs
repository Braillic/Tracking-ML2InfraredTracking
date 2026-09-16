using System;
using UnityEngine;

public static class SensorPoseRegressionTests
{
    static int passed;
    static void Check(bool condition,string message)
    {
        if(!condition) throw new Exception(message);
        passed++;
    }
    static Pose P(float x=0) => new Pose(new Vector3(x,0,0),Quaternion.identity);
    static long Ns(double seconds) => (long)Math.Round(seconds*1e9);
    public static string Run()
    {
        passed=0;
        var h=new SensorPoseHistory();
        Check(!h.TryGetPose(Ns(1),out _,out var status) && status==SensorPoseHistory.LookupStatus.Empty,"empty history");
        Check(!h.TryGetPose(0,out _,out status) && status==SensorPoseHistory.LookupStatus.InvalidTime,"invalid time");
        h.Record(Ns(1),P(2));
        Check(h.TryGetPose(Ns(1),out var p,out status) && p.position.x==2 && status==SensorPoseHistory.LookupStatus.Exact,"single exact sample");
        Check(!h.TryGetPose(Ns(.999),out _),"single sample cannot backfill");
        Check(!h.TryGetPose(Ns(1.001),out _),"single sample cannot extrapolate");
        h.Record(Ns(1.05),P(4));
        Check(h.TryGetPose(Ns(1.025),out p,out status) && Math.Abs(p.position.x-3)<1e-5 && status==SensorPoseHistory.LookupStatus.Interpolated,"interpolate translation");
        Check(!h.TryGetPose(Ns(.5),out _),"old query cannot clamp");
        Check(!h.TryGetPose(Ns(2),out _),"future query cannot extrapolate");
        h.Record(Ns(1.20),P(9));
        Check(!h.TryGetPose(Ns(1.10),out _,out status) && status==SensorPoseHistory.LookupStatus.GapTooLarge,"reject gap across missing tracking");
        Check(h.TryGetPose(Ns(1.20),out p) && p.position.x==9,"exact after gap");
        h.Record(Ns(1.20),P(50)); h.Record(Ns(1.1),P(60));
        Check(h.SampleCount==3 && h.TryGetPose(Ns(1.20),out p) && p.position.x==9,"ignore duplicate and out-of-order samples");
        h.Record(Ns(1.21),new Pose(new Vector3(float.NaN,0,0),Quaternion.identity));
        h.Record(Ns(1.22),new Pose(new Vector3(0,0,0),new Quaternion(0,0,0,0)));
        h.Record(Ns(1.23),new Pose(new Vector3(0,0,0),new Quaternion(0,0,0,float.PositiveInfinity)));
        Check(h.SampleCount==3,"invalid sample data not recorded");
        h.Record(Ns(4),P(7));
        Check(h.SampleCount==1 && !h.TryGetPose(Ns(1.2),out _),"prune retention");
        h.Clear();
        Check(h.SampleCount==0 && !h.TryGetPose(Ns(4),out _),"clear history on discontinuity");
        h.Record(Ns(5),P());
        h.Record(Ns(5.1),new Pose(new Vector3(0,0,0),new Quaternion(0,0,(float)Math.Sin(Math.PI/4),(float)Math.Cos(Math.PI/4))));
        Check(h.TryGetPose(Ns(5.05),out p) && Math.Abs(p.rotation.z-Math.Sin(Math.PI/8))<1e-5,"interpolate rotation");
        Check(h.TryGetPose(Ns(5.1),out _),"inclusive interpolation boundary");
        bool threw=false; try {new SensorPoseHistory(0);} catch(ArgumentOutOfRangeException) {threw=true;}
        Check(threw,"invalid retention rejected");

        Time.realtimeSinceStartupAsDouble=10;
        var capture=new CaptureHarness();
        capture.sensorPoseHistory.Record(Ns(1),P(2));
        capture.sensorPoseHistory.Record(Ns(1.05),P(4));
        capture._lastLiveSampleRealtime=10;
        capture.pixelSensorFeature.Ok=false;
        Check(capture.Resolve(Ns(1.025),out p) && Math.Abs(p.position.x-3)<1e-5 && capture.pixelSensorFeature.Calls==0,"covered image uses raw history");
        capture.pixelSensorFeature.Result=P(99); // SDK writes cached pose even on failure.
        Check(!capture.Resolve(Ns(1.1),out p) && p.position.x==0 && capture._droppedPoseCount==1,"failed SDK cached pose must not be accepted");
        capture.pixelSensorFeature.Ok=true;
        capture.pixelSensorFeature.Result=P(5);
        Check(capture.Resolve(Ns(1.1),out p) && p.position.x==5 && capture._directPoseCount==1,"valid exact-time SDK fallback accepted");
        capture.pixelSensorFeature.Result=new Pose(new Vector3(float.NaN,0,0),Quaternion.identity);
        Check(!capture.Resolve(Ns(1.15),out _),"invalid successful SDK pose rejected");
        capture.pixelSensorFeature.Ok=false;
        Time.realtimeSinceStartupAsDouble=10.2;
        Check(!capture.Resolve(Ns(1.025),out _) && capture.sensorPoseHistory.SampleCount==0,"old live history expires");
        capture._minimumCaptureTime=Ns(2);
        int calls=capture.pixelSensorFeature.Calls;
        Check(!capture.Resolve(Ns(1.99),out _) && capture.pixelSensorFeature.Calls==calls,"pre-reset capture rejected before lookup");
        Check(!capture.Resolve(0,out _),"zero capture rejected");

        var sdk=new SdkHarness();
        sdk.Flags=XrSpaceLocationFlagsML.OrientationValid;
        Check(!sdk.Locate(out p),"SDK rejects orientation-only pose");
        sdk.Flags=XrSpaceLocationFlagsML.PositionValid;
        Check(!sdk.Locate(out p),"SDK rejects position-only pose");
        sdk.Flags=0;
        Check(!sdk.Locate(out p),"SDK rejects no valid components");
        sdk.Flags=XrSpaceLocationFlagsML.PositionValid|XrSpaceLocationFlagsML.OrientationValid;
        Check(sdk.Locate(out p) && p.position.x==3 && p.position.z==-5,"SDK accepts complete valid pose and converts axes");
        sdk.Ok=false;
        Check(!sdk.Locate(out p),"SDK rejects failed locate call");
        return passed+" sensor-pose regression checks passed.";
    }
}

