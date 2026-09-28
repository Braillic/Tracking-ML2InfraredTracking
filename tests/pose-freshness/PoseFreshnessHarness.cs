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
public class Transform {
    public Vector3 position; public Quaternion rotation; public GameObject gameObject=new GameObject();
    public int writes;
    public void SetPositionAndRotation(Vector3 p, Quaternion q){position=p; rotation=q; writes++; Time.realtimeSinceStartupAsDouble+=.0002;}
}
__CAPTURE__
__WINDOW__
public static class Time { public static double realtimeSinceStartupAsDouble; }
public class DepthFrameTcpServer {
    public static DepthFrameTcpServer ActiveServer;
    public Dictionary<ulong,double> submissions=new Dictionary<ulong,double>();
    public ulong SessionId=99;
    public bool snapshotBusy;
    public struct FrameTimingSnapshot {
        public ulong SessionId; public double Submit, SendDone; public bool HasSend;
        public CaptureTiming Capture;
    }
    public bool TryGetFrameTimingSnapshot(ulong id,out FrameTimingSnapshot snapshot,bool nonBlocking=false) {
        snapshot=default;
        if(nonBlocking && snapshotBusy)return false;
        if(!submissions.TryGetValue(id,out double submit))return false;
        snapshot=new FrameTimingSnapshot{Submit=submit,SendDone=submit+.001,HasSend=true,
            SessionId=SessionId,Capture=new CaptureTiming((long)(submit*1e9),submit-.02,submit-.005,submit-.001)};
        return true;
    }

}
public class PoseHarness {
    public Transform trackedTool=new Transform();
    public float minConfidence=0, positionFollow=1, rotationFollow=1, maxPoseAgeSeconds=.15f, hideAfterNoPoseSeconds=.65f;
    public bool dropStaleFrames=true;
    public object _poseLock=new object();
    public bool _clientConnected=true;
    public AcceptedObservation _latestObservation;
    public bool _hasPending, _pendingOk, _hasAppliedFrame;
    public ulong _pendingFrameId, _lastAppliedFrameId, _pendingSessionId;
    public Vector3 _pendingPosition;
    public Quaternion _pendingRotation;
    public float _pendingConfidence,_pendingDetectMs,_pendingPnpMs,_pendingSendMs;
    public double _pendingReceivedRealtime,_pendingPcRecvMl2,_pendingPcSendMl2;
    public long _stalePoseCount,_appliedCount;
    public double _lastAcceptedPoseTime=double.NegativeInfinity;
    public bool _visualPrepared=true, logLatency=true, _awaitingBeforeRender, isActiveAndEnabled=true, applyBeforeRender=true;
    public double _lastApplyRealtime, _lastCaptureRealtime;
    public ulong _timingSessionId;
    public long _wrongSessionCount, _lateUpdateApplied, _beforeRenderApplied, _diagnosticOverwritten;
    public int visualSetups, logs;
    public double loggedApplied;
    private readonly AppliedPoseSample[] _appliedSamples=new AppliedPoseSample[32];
    private int _sampleRead,_sampleCount;
    private TimingWindow _submitToApply=new TimingWindow(),_captureToApply=new TimingWindow(),
        _captureToSubmit=new TimingWindow(),_captureToReady=new TimingWindow(),_pollDuration=new TimingWindow(),
        _readyToSubmit=new TimingWindow(),_rawCopy=new TimingWindow(),_applyWait=new TimingWindow(),
        _applyWork=new TimingWindow(),_captureToBeforeRender=new TimingWindow(),_updateInterval=new TimingWindow(),
        _applyToBeforeRender=new TimingWindow();
    void EnsureTrackedToolVisual(){visualSetups++; if(trackedTool==null)trackedTool=new Transform();}
    void MaybeLogLatency(in AppliedPoseSample sample){logs++; loggedApplied=sample.Applied;}
    public void BeforeRender()=>ApplyAndMeasureBeforeRender();
    public void Flush()=>FlushAppliedSamples();
    public void Prepare()=>PrepareTrackedToolVisual();
    public int Queued=>_sampleCount;
    public double MedianWork=>_applyWork.Percentile(.5);
    public double MedianWait=>_applyWait.Percentile(.5);

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
        DepthFrameTcpServer.ActiveServer=depth;
        Time.realtimeSinceStartupAsDouble=2.01; depth.submissions[10]=2.0;
        var render=new PoseHarness(); render.Receive(10,10);
        Time.realtimeSinceStartupAsDouble=2.015; render.BeforeRender();
        Check(render._appliedCount==1 && render._beforeRenderApplied==1 && render.visualSetups==0,
              "late packet applies before render without visual construction");
        Check(render.logs==0 && render.Queued==1,"before-render defers diagnostic formatting");
        render.BeforeRender();
        Check(render._appliedCount==1 && render.trackedTool.writes==1,"repeated callbacks do not reapply or refilter a pose");
        Time.realtimeSinceStartupAsDouble=2.1;render.Flush();
        Check(render.logs==1 && Math.Abs(render.loggedApplied-2.0152)<1e-8,
              "deferred logging retains application time, not later print time");
        Check(Math.Abs(render.MedianWait-5)<1e-6 && Math.Abs(render.MedianWork-.2)<1e-6,
              "queue wait and validation/transform cost measured separately");
        render.Receive(10,99);render.BeforeRender();
        Check(render._appliedCount==1,"duplicate cannot be consumed again in another phase");
        depth.submissions[11]=2.08;render.Receive(11,11);render.applyBeforeRender=false;render.BeforeRender();
        Check(render._hasPending && render._appliedCount==1,"disabled render option retains LateUpdate fallback");
        render.Apply();Check(render._lateUpdateApplied==1 && render._appliedCount==2,"LateUpdate still applies pending pose");
        depth.submissions[12]=2.09;render.Receive(12,12);render.applyBeforeRender=true;
        depth.snapshotBusy=true;render.BeforeRender();
        Check(render._hasPending && render._appliedCount==2,"busy snapshot leaves newest pending pose intact");
        depth.snapshotBusy=false;render.BeforeRender();
        Check(render._appliedCount==3,"pending pose recovers after snapshot contention");
        depth.submissions[13]=2.09;render.Receive(13,13);
        using(var held=new ManualResetEventSlim()) using(var release=new ManualResetEventSlim()) {
            var thread=new Thread(()=>{lock(render._poseLock){held.Set();release.Wait(3000);}});
            thread.Start();Check(held.Wait(1000),"mailbox contention fixture acquired lock");
            try {render.BeforeRender();Check(render._hasPending && render._appliedCount==3,"render never waits on receiver mailbox");}
            finally {release.Set();thread.Join();}
        }
        render.BeforeRender();Check(render._appliedCount==4,"latest pose survives mailbox contention");
        depth.submissions[14]=2.09;render.Receive(14,14);depth.SessionId=100;render.BeforeRender();
        Check(render._appliedCount==4 && render._wrongSessionCount==1,"origin changed after receipt: consumption rejects old session");
        render.InvalidateTrackingSession();
        Check(!render.trackedTool.gameObject.activeSelf && render.Queued==0 && !render._awaitingBeforeRender,
              "session reset clears display and deferred timing samples");
        depth.submissions[0]=2.09;render.Receive(0,7);render.BeforeRender();
        Check(render._lastAppliedFrameId==0 && render._appliedCount==5,
              "new session can restart frame numbering without inheriting old ID gate");
        render.SubmitPose(999,true,.9f,new Vector3{x=99},new Quaternion{w=1},0,0,0,0,0,99);
        depth.submissions[1]=2.09;render.Receive(1,1);render.BeforeRender();
        Check(render._lastAppliedFrameId==1 && render.trackedTool.position.x==1,
              "old-session high frame ID cannot suppress a new-session pending result");
        var first=new PoseHarness{_visualPrepared=false,trackedTool=null};
        depth.submissions[20]=2.09;first.Receive(20);first.BeforeRender();
        Check(first._hasPending && first.visualSetups==0,"render never creates first visual");
        first.Prepare();first.BeforeRender();
        Check(first._appliedCount==1 && first.visualSetups==1,"prepared visual accepts pending first pose");
        first.isActiveAndEnabled=false;depth.submissions[21]=2.09;first.Receive(21);first.BeforeRender();
        Check(first._appliedCount==1,"disabled component does not apply in render callback");
        Time.realtimeSinceStartupAsDouble=2.11;
        var raw=new PoseHarness{positionFollow=.5f}; depth.submissions[22]=2.09;raw.Receive(22,12);raw.Apply();
        PoseHarness.AcceptedObservation observation;
        Check(raw.TryGetAcceptedObservation(out observation) && observation.Position.x==12,
              "observation contract uses raw validated pose rather than smoothed display");
        Check(Math.Abs(observation.CaptureTime-2.07)<1e-8 && observation.SessionId==depth.SessionId && observation.FrameId==22,
              "observation contract retains capture timestamp and identity");
        raw._clientConnected=false;Check(!raw.TryGetAcceptedObservation(out observation),"disconnected source unavailable");raw._clientConnected=true;
        raw.isActiveAndEnabled=false;Check(!raw.TryGetAcceptedObservation(out observation),"disabled source unavailable");raw.isActiveAndEnabled=true;
        Time.realtimeSinceStartupAsDouble=2.5;Check(!raw.TryGetAcceptedObservation(out observation),"capture-age limit rejects held pose");
        Time.realtimeSinceStartupAsDouble=2.0;Check(!raw.TryGetAcceptedObservation(out observation),"future observation rejected");
        Time.realtimeSinceStartupAsDouble=2.11;raw._latestObservation.CaptureTime=double.NaN;
        Check(!raw.TryGetAcceptedObservation(out observation),"unmapped capture clock rejected");raw._latestObservation.CaptureTime=2.07;
        raw._latestObservation.SessionId=99;Check(!raw.TryGetAcceptedObservation(out observation),"old epoch observation rejected");
        raw.InvalidateTrackingSession();Check(!raw.TryGetAcceptedObservation(out observation),"origin invalidation hides observation");
        var bounded=new PoseHarness();
        for(ulong i=30;i<70;i++){depth.submissions[i]=2.09;bounded.Receive(i);bounded.BeforeRender();}
        Check(bounded.Queued==32 && bounded._diagnosticOverwritten==8,"diagnostic storage is bounded without blocking application");
        Time.realtimeSinceStartupAsDouble=3.0;bounded.BeforeRender();
        Check(!bounded.trackedTool.gameObject.activeSelf,"before-render still honors source-age hiding");
        Console.WriteLine($"Passed {checks} pose freshness checks (production method bodies).");
    }
}
