// Exercises the production provider + resource controller with asynchronous SDK/transport doubles.
using System;
using Braillic.Tracking;
using Braillic.Tracking.Runtime;
using Braillic.Tracking.Implementation.ML2;

public sealed class DepthSensorAPI : ICaptureDevice
{
    public sealed class Operation : ICaptureOperation
    { public bool Done,Success=true;public bool IsCompleted=>Done;public bool Succeeded=>Success; }
    public ML2DepthRawStream streamVisualizer;
    public bool isActiveAndEnabled=true;
    public bool OwnsResource{get;private set;}
    public bool IsStreaming{get;private set;}
    public int Reads,Releases;
    public Operation Pending;
    public CaptureDeviceLifecycle Lifecycle {get;}
    public DepthSensorAPI(){Lifecycle=new CaptureDeviceLifecycle(this);}
    public void BindTrackingLifecycle(DepthFrameTcpServer sender){Lifecycle.Stop();}
    public bool Prepare(){OwnsResource=true;return true;}
    public ICaptureOperation Configure()=>Pending=new Operation();
    public ICaptureOperation Start()=>Pending=new Operation();
    public ICaptureOperation Stop()=>Pending=new Operation();
    public void Complete(bool streaming){IsStreaming=streaming;Pending.Done=true;}
    public void Release(){if(IsStreaming)throw new Exception("release while streaming");OwnsResource=false;Releases++;}
    public void Read(){Reads++;}
}
internal static class Ml2ProviderTests
{
    private sealed class Clock:ITrackingClock {public string Id=>"runtime";public double NowSeconds{get;set;}=20;}
    private static int checks;
    private static void Check(bool ok,string label){if(!ok)throw new Exception(label);checks++;}
    private static void Tick(TrackingSystem tracking)=>tracking.Pump(TrackingUpdatePhase.Update);
    private static PoseEstimateTcpServer.AcceptedObservation Sample(ulong session,ulong sequence,double time)=>
        new PoseEstimateTcpServer.AcceptedObservation{SessionId=session,FrameId=sequence,CaptureTime=time-.02,AppliedTime=time,
            Position=new UnityEngine.Vector3(1,2,3),Rotation=new UnityEngine.Quaternion(0,0,0,1),Confidence=.9f};
    public static int Main()
    {
        var clock=new Clock();var tracking=new TrackingSystem(clock);
        var capture=new ML2DepthRawStream();var sensor=new DepthSensorAPI{streamVisualizer=capture};
        var sender=new DepthFrameTcpServer();var receiver=new PoseEstimateTcpServer();
        DepthFrameTcpServer.ActiveServer=sender;PoseEstimateTcpServer.ActiveServer=receiver;
        var provider=new Ml2TrackingProvider("ml2","body","calibration",sensor,capture,sender,receiver);
        tracking.AddProvider(provider);tracking.BindObject("probe",new ObjectBinding("ml2","body",RigidPose.Identity));
        var requirements=new ObservationRequirements(.1,.02,.005);
        tracking.Start();Tick(tracking);Tick(tracking);
        Check(sensor.OwnsResource&&tracking.State==TrackingState.Starting&&!capture.TrackingSubmissionEnabled,"configure pending gates submission");
        sensor.Complete(false);Tick(tracking);
        Check(tracking.State==TrackingState.Starting,"SDK start pending");
        sensor.Complete(true);Tick(tracking);
        Check(tracking.State==TrackingState.Running&&capture.TrackingSubmissionEnabled,"sensor completion enables provider");
        int reads=sensor.Reads;
        receiver.Next=Sample(sender.SessionId,1,clock.NowSeconds);
        tracking.Pump(TrackingUpdatePhase.BeforeRender);
        Check(sensor.Reads==reads&&receiver.BeforeRender,"before-render receives poses without polling SDK");
        Check(tracking.TryGetPose("probe",requirements,out var first,out _)&&first.FrameFromObject.Position.Z==-3,"actual provider publishes canonical pose");
        Check(first.Source.CaptureSeconds==clock.NowSeconds-.02,"capture timestamp preserved");
        ulong old=sender.SessionId;tracking.ResetProviderReference("ml2");
        receiver.Next=Sample(old,999,clock.NowSeconds);tracking.Pump(TrackingUpdatePhase.BeforeRender);
        Check(!tracking.TryGetPose("probe",requirements,out _,out _),"previous epoch delayed packet rejected");
        clock.NowSeconds+=.02;receiver.Next=Sample(sender.SessionId,1,clock.NowSeconds);tracking.Pump(TrackingUpdatePhase.LateUpdate);
        Check(tracking.TryGetPose("probe",requirements,out var reset,out _)&&!first.Frame.Equals(reset.Frame),"new session accepted with new frame epoch");
        tracking.Stop();
        Check(tracking.State==TrackingState.Stopping&&!capture.TrackingSubmissionEnabled&&!sender.IsRunning&&!receiver.IsRunning,"stop immediately gates poses and transport");
        Check(!tracking.TryGetPose("probe",requirements,out _,out _)&&!tracking.ShutdownComplete,"stop hides poses while resource remains owned");
        Tick(tracking);tracking.Start();Tick(tracking);
        Check(sensor.OwnsResource&&!sender.IsRunning,"restart waits for SDK stop completion");
        sensor.Complete(false);Tick(tracking);
        Check(sensor.Releases==1&&sender.IsRunning&&tracking.State==TrackingState.Starting,"restart only after old resource release");
        Tick(tracking);tracking.Stop();Tick(tracking);
        Check(sensor.OwnsResource&&!tracking.ShutdownComplete,"stop during configure waits for outstanding operation");
        sensor.Complete(false);Tick(tracking);
        Check(tracking.ShutdownComplete&&!sensor.OwnsResource&&sensor.Releases==2,"cancelled configure releases without starting");
        receiver.FailStart=true;tracking.Start();
        Check(tracking.State==TrackingState.Faulted&&!sender.IsRunning&&!receiver.IsRunning&&!capture.TrackingSubmissionEnabled,"transport startup failure rolls back whole provider");
        receiver.FailStart=false;tracking.Start();Tick(tracking);Tick(tracking);sensor.Complete(false);Tick(tracking);sensor.Complete(true);Tick(tracking);
        capture.isActiveAndEnabled=false;tracking.Pump(TrackingUpdatePhase.LateUpdate);
        Check(tracking.State==TrackingState.Faulted&&!sender.IsRunning,"disabled capture faults unified service");
        Tick(tracking);sensor.Complete(false);Tick(tracking);
        Check(tracking.ShutdownComplete&&!sensor.OwnsResource,"fault cleanup releases owned SDK resource");
        tracking.Dispose();Check(tracking.State==TrackingState.Disposed,"dispose after asynchronous release");
        Console.WriteLine("PASS: "+checks+" complete ML2 provider lifecycle and publication checks");return 0;
    }
}
