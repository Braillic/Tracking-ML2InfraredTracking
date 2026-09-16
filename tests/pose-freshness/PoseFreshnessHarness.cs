// Test-only Unity stand-ins. Run-PoseFreshnessTests inserts actual production
// receive/apply/hide methods; Unity reference compilation is checked separately.
using System;
using System.Collections.Generic;
using System.Threading;

public struct Vector3 {
    public float x,y,z;
    public static Vector3 Lerp(Vector3 a,Vector3 b,float t) => b;
}
public struct Quaternion {
    public float x,y,z,w;
    public static float Dot(Quaternion a,Quaternion b) => a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
    public Quaternion normalized { get { var n=(float)Math.Sqrt(Dot(this,this)); return new Quaternion{x=x/n,y=y/n,z=z/n,w=w/n}; } }
    public static Quaternion Slerp(Quaternion a,Quaternion b,float t) => b;
}
public class GameObject { public bool activeSelf; public void SetActive(bool value) {activeSelf=value;} }
public class Transform { public Vector3 position; public Quaternion rotation; public GameObject gameObject=new GameObject(); }
public static class Time { public static double realtimeSinceStartupAsDouble; }
public class DepthFrameTcpServer {
    public static DepthFrameTcpServer ActiveServer;
    public Dictionary<ulong,double> submissions=new Dictionary<ulong,double>();
    public bool TryGetFrameTiming(ulong id,out double submit,out double done,out bool sent,out int pipeline) {
        done=0; sent=true; pipeline=2; return submissions.TryGetValue(id,out submit);
    }
}
public class PoseHarness {
    public Transform trackedTool=new Transform();
    public float minConfidence=0, positionFollow=1, rotationFollow=1, maxPoseAgeSeconds=.15f, hideAfterNoPoseSeconds=.65f;
    public bool dropStaleFrames=true;
    public object _poseLock=new object();
    public bool _hasPending, _pendingOk, _hasAppliedFrame;
    public ulong _pendingFrameId, _lastAppliedFrameId;
    public Vector3 _pendingPosition;
    public Quaternion _pendingRotation;
    public float _pendingConfidence,_pendingDetectMs,_pendingPnpMs,_pendingSendMs;
    public double _pendingReceivedRealtime,_pendingPcRecvMl2,_pendingPcSendMl2;
    public long _stalePoseCount,_appliedCount;
    public double _lastAcceptedPoseTime=double.NegativeInfinity;
    void EnsureTrackedToolVisual() {}
    void MaybeLogLatency(ulong id,double received,float detect,float pnp,float send,double pcRecv,double pcSend) {}
    public void Apply() => TryApplyPendingPose();
    public void Hide() => HideTrackedToolIfTimedOut();
    public void Receive(ulong id,float x=0,float confidence=.9f,float qw=2) {
        SubmitPose(id,true,confidence,new Vector3{x=x},new Quaternion{w=qw},0,0,0,0,0);
    }
__PRODUCTION_METHODS__
}
public static class PoseFreshnessChecks {
    static int checks;
    static void Check(bool ok,string name) { if(!ok) throw new Exception(name); checks++; }
    public static void Run() {
        var depth=new DepthFrameTcpServer(); DepthFrameTcpServer.ActiveServer=depth;
        depth.submissions[0]=1; Time.realtimeSinceStartupAsDouble=1.05;
        var p=new PoseHarness(); p.Receive(0,3); p.Apply();
        Check(p._appliedCount==1 && p.trackedTool.gameObject.activeSelf,"first frame ID zero accepted");
        Check(Math.Abs(p.trackedTool.rotation.w-1)<1e-5,"quaternion normalized");
        Check(p._lastAcceptedPoseTime==1,"lifetime starts at submit time");
        p.Receive(0,7); p.Apply();
        Check(p._appliedCount==1 && p.trackedTool.position.x==3,"duplicate cannot refresh pose");
        depth.submissions[2]=1.02; depth.submissions[1]=1.01;
        p.Receive(2,2); p.Receive(1,1); p.Apply();
        Check(p._lastAppliedFrameId==2 && p.trackedTool.position.x==2,"older pending packet cannot overwrite newer");
        p.Receive(1,5); p.Apply();
        Check(p._appliedCount==2,"out-of-order pose rejected after apply");
        depth.submissions[3]=1.1; Time.realtimeSinceStartupAsDouble=1.4;
        p.Receive(3,4); p.Apply();
        Check(p._appliedCount==2 && p._lastAcceptedPoseTime==1.02,"increasing stale ID does not refresh lifetime");
        p.Receive(500,5); p.Apply();
        Check(p._appliedCount==2,"unknown depth ID rejected");
        depth.submissions[4]=1.5; p.Receive(4); p.Apply();
        Check(p._appliedCount==2,"future submit rejected");
        depth.submissions[5]=1.35;
        p.Receive(5,float.NaN); p.Apply();
        Check(p._appliedCount==2,"nonfinite position rejected");
        p.Receive(5,0,float.NaN); p.Apply();
        Check(p._appliedCount==2,"nonfinite confidence rejected");
        p.Receive(5,0,.9f,0); p.Apply();
        Check(p._appliedCount==2,"zero quaternion rejected");
        Time.realtimeSinceStartupAsDouble=1.60; p.Hide();
        Check(p.trackedTool.gameObject.activeSelf,"short observation gap holds model");
        Time.realtimeSinceStartupAsDouble=1.68; p.Hide();
        Check(!p.trackedTool.gameObject.activeSelf,"model hides by source age");
        p.Receive(5); p.Apply();
        Check(!p.trackedTool.gameObject.activeSelf,"stale backlog cannot resurrect hidden model");
        depth.submissions[6]=1.66; p.Receive(6); p.Apply();
        Check(p.trackedTool.gameObject.activeSelf && p._appliedCount==3,"fresh recovery restores visibility");
        DepthFrameTcpServer.ActiveServer=null; p.Receive(7); p.Apply();
        Check(p._appliedCount==3,"missing depth server fails closed");
        Check(!PoseHarness.IsFreshPose(8,true,6,true,true,double.NaN,2,.15),"invalid clock rejected");
        Console.WriteLine($"Passed {checks} pose freshness checks (production method bodies).");
    }
}
