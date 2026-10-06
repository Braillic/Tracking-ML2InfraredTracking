// Test doubles stand in for hardware/transport. The backend and coordinate adapter are production code.
using System;
using Braillic.Tracking;
using Braillic.Tracking.Application;
using Braillic.Tracking.Unity;

namespace UnityEngine
{
    public struct Vector3 { public float x,y,z; public Vector3(float x,float y,float z){this.x=x;this.y=y;this.z=z;} }
    public struct Quaternion { public float x,y,z,w; public Quaternion(float x,float y,float z,float w){this.x=x;this.y=y;this.z=z;this.w=w;} }
}
public sealed class DepthFrameTcpServer
{
    public static DepthFrameTcpServer ActiveServer;
    public bool isActiveAndEnabled=true, IsRunning, Managed;
    public string Status => IsRunning?"running":"stopped";
    public ulong SessionId;
    public void SetApplicationManaged(bool managed){Managed=managed;StopServer();}
    public void StartServer(){IsRunning=true;BeginCaptureEpoch();}
    public void StopServer(){IsRunning=false;}
    public void BeginCaptureEpoch(){SessionId++;PoseEstimateTcpServer.ActiveServer?.Invalidate();}
}
public sealed class ML2DepthRawStream
{
    public bool isActiveAndEnabled=true, TrackingSubmissionEnabled=true;
    public DepthFrameTcpServer Bound;
    public void BindTrackingSender(DepthFrameTcpServer sender){Bound=sender;}
}
public sealed class PoseEstimateTcpServer
{
    public static PoseEstimateTcpServer ActiveServer;
    public struct AcceptedObservation
    {
        public ulong SessionId,FrameId;
        public double CaptureTime,AppliedTime;
        public UnityEngine.Vector3 Position;
        public UnityEngine.Quaternion Rotation;
        public float Confidence;
    }
    public event Action<AcceptedObservation> ObservationAccepted;
    public event Action TrackingInvalidated;
    public bool isActiveAndEnabled=true,IsRunning,Available,BeforeRender,FailStart;
    public string Status => IsRunning?"running":"stopped";
    public DepthFrameTcpServer Bound;
    public AcceptedObservation? Next;
    public void SetApplicationManaged(DepthFrameTcpServer sender){Bound=sender;StopServer();}
    public void StartServer(){IsRunning=true;if(FailStart)throw new Exception("cannot start pose transport");}
    public void StopServer(){IsRunning=false;Invalidate();}
    public void Invalidate(){Available=false;Next=null;TrackingInvalidated?.Invoke();}
    public bool TryGetAcceptedObservation(out AcceptedObservation o){o=default;return Available;}
    public void PumpApplication(bool beforeRender)
    {BeforeRender=beforeRender;if(Next.HasValue){var value=Next.Value;Next=null;Available=true;ObservationAccepted?.Invoke(value);}}
}
internal static class BackendTests
{
    private sealed class Clock:ITrackingClock {public string Id=>"clock";public double NowSeconds{get;set;}=20;}
    private static int checks;
    private static void Check(bool ok,string label){if(!ok)throw new Exception(label);checks++;}
    private static PoseEstimateTcpServer.AcceptedObservation Sample(ulong session,ulong sequence,double time) =>
        new PoseEstimateTcpServer.AcceptedObservation{SessionId=session,FrameId=sequence,CaptureTime=time-.02,AppliedTime=time,
            Position=new UnityEngine.Vector3(1,2,3),Rotation=new UnityEngine.Quaternion(0,0,0,1),Confidence=.9f};
    public static int Main()
    {
        var clock=new Clock();var core=new TrackingCore(clock);
        var depth=new DepthFrameTcpServer();var poses=new PoseEstimateTcpServer();var capture=new ML2DepthRawStream();
        DepthFrameTcpServer.ActiveServer=depth;PoseEstimateTcpServer.ActiveServer=poses;
        var backend=new Ml2DesktopTrackingBackend(core,depth,poses,capture,"ml2","probe","calib");
        var app=new TrackingApplication(core,backend);
        int published=0,invalidated=0;
        backend.ObservationPublished+=()=>published++;
        backend.ReferenceInvalidated+=()=>invalidated++;
        Check(depth.Managed && poses.Bound==depth && capture.Bound==depth && !capture.TrackingSubmissionEnabled,"exclusive ownership bound before startup");
        app.Start();Check(depth.IsRunning && poses.IsRunning && capture.TrackingSubmissionEnabled,"start enables transports and submission");
        poses.Next=Sample(depth.SessionId,1,clock.NowSeconds);app.Pump(TrackingPhase.BeforeRender);
        Check(poses.BeforeRender && published==1,"accepted pose delivered in same before-render pump");
        Check(app.TryGetLatest("ml2","probe",.1,out var first,out _) && first.ReferenceFromObject.Position.Z==-3,"canonical pose published");
        var roundtrip=TrackingCoordinates.ToUnity(first.ReferenceFromObject.Position);
        Check(roundtrip.x==1 && roundtrip.y==2 && roundtrip.z==3,"Unity geometry preserved at presentation boundary");
        app.Pump(TrackingPhase.LateUpdate);Check(published==1,"idle pump never duplicates sample");
        clock.NowSeconds+=.02;poses.Next=Sample(depth.SessionId,2,clock.NowSeconds);app.Pump(TrackingPhase.LateUpdate);
        Check(published==2 && app.TryGetLatest("ml2","probe",.1,out var newer,out _) && newer.Sequence==2,"subsequent live frame");
        poses.Available=false;app.Pump(TrackingPhase.LateUpdate);
        Check(!app.TryGetLatest("ml2","probe",.1,out _,out _),"disconnect withdraws current observation");
        poses.Available=true;app.Pump(TrackingPhase.LateUpdate);
        Check(!app.TryGetLatest("ml2","probe",.1,out _,out _),"cached pre-disconnect pose cannot revive");
        clock.NowSeconds+=.02;poses.Next=Sample(depth.SessionId,3,clock.NowSeconds);app.Pump(TrackingPhase.LateUpdate);
        Check(app.TryGetLatest("ml2","probe",.1,out _,out _),"fresh recovery restores measurement");
        ulong old=depth.SessionId;app.ResetReferenceFrame();
        Check(depth.SessionId!=old && invalidated>0 && !app.TryGetLatest("ml2","probe",.1,out _,out _),"origin reset invalidates core and protocol epoch");
        poses.Next=Sample(old,999,clock.NowSeconds);app.Pump(TrackingPhase.BeforeRender);
        Check(!app.TryGetLatest("ml2","probe",.1,out _,out _),"late previous-session result rejected");
        clock.NowSeconds+=.02;poses.Next=Sample(depth.SessionId,1,clock.NowSeconds);app.Pump(TrackingPhase.BeforeRender);
        Check(app.TryGetLatest("ml2","probe",.1,out var reset,out _) && !first.Session.ReferenceFrame.Equals(reset.Session.ReferenceFrame),"new frame epoch and reset sequence accepted");
        app.Stop();Check(!capture.TrackingSubmissionEnabled && !depth.IsRunning && !poses.IsRunning && !app.TryGetLatest("ml2","probe",.1,out _,out _),"stop withdraws all outputs");
        app.Start();depth.BeginCaptureEpoch();clock.NowSeconds+=.02;poses.Next=Sample(depth.SessionId,1,clock.NowSeconds);app.Pump(TrackingPhase.LateUpdate);
        Check(app.TryGetLatest("ml2","probe",.1,out _,out _),"sensor-triggered epoch reset handled");
        capture.isActiveAndEnabled=false;app.Pump(TrackingPhase.LateUpdate);
        Check(app.State==TrackingApplicationState.Faulted && !depth.IsRunning && !capture.TrackingSubmissionEnabled,"disabled component faults and shuts down");
        capture.isActiveAndEnabled=true;poses.FailStart=true;app.Start();
        Check(app.State==TrackingApplicationState.Faulted && !depth.IsRunning && !poses.IsRunning && !capture.TrackingSubmissionEnabled,"partial transport startup rolls back");
        poses.FailStart=false;app.Start();depth.IsRunning=false;app.Pump(TrackingPhase.LateUpdate);
        Check(app.State==TrackingApplicationState.Faulted && !poses.IsRunning,"asynchronous server failure observed");
        app.Dispose();
        Console.WriteLine("PASS: "+checks+" production ML2 backend checks");return 0;
    }
}
