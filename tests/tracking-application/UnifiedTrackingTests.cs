using System;
using System.Collections.Generic;
using System.Threading;
using Braillic.Tracking;
using Braillic.Tracking.Runtime;
using Braillic.Tracking.Application;

internal static class UnifiedTrackingTests
{
    private sealed class Clock:ITrackingClock {public string Id{get;set;}="runtime";public double NowSeconds{get;set;}=10;}
    private sealed class Provider:ITrackingProvider
    {
        public string Id{get;}
        public ProviderState State{get;set;}
        public string LastError{get;set;}
        public TrackingProviderContext Context;
        public ClockMapping Mapping;
        public bool FailStart,FailPump,AsyncStop,FinishStop,FailInitialize;
        public int Starts,Stops,Disposes;
        public Provider(string id){Id=id;}
        public void Initialize(TrackingProviderContext context){Context=context;if(FailInitialize)throw new Exception("init failure");}
        public void Start()
        {
            Starts++;State=ProviderState.Starting;
            Context.BeginSession("session-"+Starts,new CoordinateFrame(Id,"epoch-"+Starts),"calibration",Mapping??ClockMapping.Identity(Context.Clock.Id));
            if(FailStart)throw new Exception("start failure");State=ProviderState.Running;
        }
        public void Stop(){Stops++;Context?.EndSession();State=AsyncStop?ProviderState.Stopping:ProviderState.Stopped;}
        public void Pump(TrackingUpdatePhase phase)
        {if(FailPump)throw new Exception("pump failure");if(FinishStop&&State==ProviderState.Stopping)State=ProviderState.Stopped;}
        public void ResetReferenceFrame(){Context.EndSession();Start();}
        public void Dispose(){Disposes++;}
        public TrackingIssue Pose(string body,ulong sequence,double capture,RigidPose pose) => Context.Publish(Context.Session,body,sequence,capture,Context.Clock.NowSeconds,pose,.9);
    }
    private sealed class Operation:ICaptureOperation {public bool IsCompleted{get;set;}public bool Succeeded{get;set;}=true;}
    private sealed class Device:ICaptureDevice
    {
        public bool OwnsResource{get;private set;}public bool IsStreaming{get;set;}
        public bool Permission=true,ReleaseFails;public int Configures,Starts,Stops,Releases,Reads;
        public Operation Op;
        public bool Prepare(){if(!Permission)return false;OwnsResource=true;return true;}
        public ICaptureOperation Configure(){Configures++;return Op=new Operation();}
        public ICaptureOperation Start(){Starts++;return Op=new Operation();}
        public ICaptureOperation Stop(){Stops++;return Op=new Operation();}
        public void Release(){Releases++;if(ReleaseFails)throw new Exception("release failure");OwnsResource=false;}
        public void Read(){Reads++;}
    }
    private static int checks;
    private static void Check(bool ok,string message){if(!ok)throw new Exception(message);checks++;}
    private static void Throws(Action action,string message){bool threw=false;try{action();}catch{threw=true;}Check(threw,message);}
    private static RigidPose Pose(double x,double y=0,double z=0) => new RigidPose(new Vector3d(x,y,z),Quaterniond.Identity);
    private static bool Near(double a,double b)=>Math.Abs(a-b)<1e-8;
    private static ObservationRequirements Req=>new ObservationRequirements(.2,.02,.005);

    public static int Main()
    {
        Combined();Lifecycle();Capture();Replay();
        Console.WriteLine("PASS: "+checks+" unified runtime, OTS+AR, tracing, clock and capture-lifecycle checks");return 0;
    }
    private static void Combined()
    {
        var clock=new Clock();var system=new TrackingSystem(clock);
        var ots=new Provider("ots"){Mapping=new ClockMapping("ots-clock",clock.Id,1,-990,.001,0,100)};
        var ar=new Provider("ar");system.AddProvider(ots,false);system.AddProvider(ar);
        system.BindObject("probe",new ObjectBinding("ots","tool17",Pose(.01)),new ObjectBinding("ar","constellation-A",RigidPose.Identity));
        system.BindObject("patient",new ObjectBinding("ots","reference",RigidPose.Identity));
        system.Start();Check(system.State==TrackingState.Running,"two providers running");
        Check(ots.Pose("tool17",1,999.99,Pose(.4,.2,1))==TrackingIssue.None,"map OTS clock to runtime");
        Check(ots.Pose("reference",1,999.985,Pose(.1,.2,.9))==TrackingIssue.None,"publish patient reference");
        Check(ar.Pose("constellation-A",1,9.99,Pose(9.8,.41,1))==TrackingIssue.None,"publish AR fallback");
        system.TryGetProviderFrame("ar",out var arFrame);system.TryGetProviderFrame("ots",out var otsFrame);
        Check(!system.TryGetPoseInFrame("probe",arFrame,Req,out _,out var issue)&&issue==TrackingIssue.FrameMismatch,"unregistered OTS/AR worlds never mixed");
        var rotation=new Quaterniond(0,0,Math.Sqrt(.5),Math.Sqrt(.5));
        system.SetFrameCalibration("ots-to-ar",otsFrame,arFrame,new RigidPose(new Vector3d(10,0,0),rotation),0,100);
        Check(system.TryGetPoseInFrame("probe",arFrame,Req,out var world,out _)&&Near(world.FrameFromObject.Position.X,9.8)&&Near(world.FrameFromObject.Position.Y,.41),"calibrated rotated OTS pose rendered in AR world");
        Check(world.Source.Session.SourceId=="ots"&&Near(world.Source.CaptureSeconds,9.99)&&world.Frame.Equals(arFrame),"source/time provenance retained after conversion");
        Check(system.TryGetRelativePose("probe","patient",Req,out var relative,out _)&&Near(relative.FrameFromObject.Position.X,.31)&&Near(relative.FrameFromObject.Position.Z,.1),"probe relative to patient with rigid-body axis calibration");
        Check(relative.Reference.HasValue&&Near(relative.Reference.Value.CaptureSeconds,9.985),"reference keeps its own capture time");
        Check(!system.TryGetRelativePose("probe","patient",new ObservationRequirements(.2,.001,.005),out _,out issue)&&issue==TrackingIssue.TimeMismatch,"temporal mismatch rejected including uncertainty");
        Check(!system.TryGetPose("patient",new ObservationRequirements(.2,.02,.0001),out _,out issue)&&issue==TrackingIssue.ClockMismatch,"uncertain provider clock rejected");
        var tip=new TraceObservationSource(system,"probe","patient",new Vector3d(0,0,.15),"pivot-calib-A",Req);
        Check(tip.TryRead(null,out var t1,out _)&&Near(t1.TipPosition.X,.31)&&Near(t1.TipPosition.Z,.25),"tracing receives reference-relative raw tip");
        tip.TryRead(null,out var duplicate,out _);Check(t1.CollectionEpoch==duplicate.CollectionEpoch&&duplicate.Tracking.Source.Sequence==1,"duplicate polls preserve identity");
        clock.NowSeconds=10.01;ots.Pose("tool17",2,1000,Pose(1.4,.2,1));ots.Pose("reference",2,1000,Pose(1.1,.2,.9));
        Check(tip.TryRead(null,out var moving,out _)&&Near(moving.TipPosition.X,.31)&&moving.CollectionEpoch==t1.CollectionEpoch,"patient and probe motion cancels in patient-relative coordinates");
        ots.Context.MarkUnavailable(ots.Context.Session,"tool17");
        Check(system.TryGetPose("probe",Req,out var fallback,out _)&&fallback.Source.Session.SourceId=="ar","explicit fallback policy selects available AR measurement");
        Check(tip.TryRead(null,out var switched,out _)&&switched.CollectionEpoch!=t1.CollectionEpoch,"source switch invalidates trace continuity");
        Check(!tip.TryRead(null,out _,out _) == false,"mixed-provider relative query works with calibration and compatible times");
        var previous=ots.Context.Session;system.ResetProviderReference("ots");
        Check(ots.Context.Publish(previous,"tool17",500,1000,clock.NowSeconds,Pose(99))==TrackingIssue.SessionSuperseded,"late callback from old epoch rejected");
        Check(!system.TryGetPoseInFrame("probe",otsFrame,Req,out _,out issue)&&issue==TrackingIssue.FrameMismatch,"old target frame rejected");
        ots.Pose("tool17",1,1000.005,Pose(.4,.2,1));ots.Pose("reference",1,1000.005,Pose(.1,.2,.9));
        Check(!system.TryGetPoseInFrame("probe",arFrame,Req,out _,out issue)&&issue==TrackingIssue.FrameMismatch,"calibration invalidated on provider epoch change");
        system.TryGetProviderFrame("ots",out var newOtsFrame);
        Throws(()=>system.SetFrameCalibration("bad",otsFrame,arFrame,RigidPose.Identity,0,100),"cannot install calibration for obsolete epoch");
        system.SetFrameCalibration("ots-to-ar",newOtsFrame,arFrame,RigidPose.Identity,0,10.02);
        clock.NowSeconds=10.03;Check(!system.TryGetPoseInFrame("probe",arFrame,Req,out _,out _),"expired spatial calibration rejected");
        system.SetFrameCalibration("ots-to-ar",newOtsFrame,arFrame,RigidPose.Identity,0,100);
        Throws(()=>system.SetFrameCalibration("second-path",arFrame,newOtsFrame,RigidPose.Identity,0,100),"ambiguous calibration cycle rejected atomically");
        Check(system.TryGetPoseInFrame("probe",arFrame,Req,out _,out _),"failed calibration update preserves previous valid mapping");
        Exception workerError=null;var thread=new Thread(()=>{try{ots.Pose("tool17",2,1000.02,Pose(9));}catch(Exception e){workerError=e;}});
        thread.Start();thread.Join();Check(workerError is InvalidOperationException,"worker SDK callbacks must marshal to owner thread");
        clock.NowSeconds=101;Check(!system.TryGetPose("patient",new ObservationRequirements(200,.02,.005),out _,out _),"expired clock mapping rejected even if caller allows old poses");
        system.Stop();Check(!system.TryGetPose("probe",Req,out _,out _),"stop withdraws all providers");system.Dispose();

        var graph=new FrameCalibrationGraph();var a=new CoordinateFrame("a","1");var b=new CoordinateFrame("b","1");var c=new CoordinateFrame("c","1");
        graph.Set("ab",a,b,Pose(2),0,100);graph.Set("bc",b,c,Pose(3),0,100);
        Check(graph.TryResolve(a,c,5,out var ac)&&Near(ac.Position.X,5),"multi-hop frame composition");
        Check(graph.TryResolve(c,a,5,out var ca)&&Near(ca.Position.X,-5),"inverse multi-hop frame composition");
        graph.RemoveFrame(b);Check(!graph.TryResolve(a,c,5,out _),"removing an epoch removes dependent calibration edges");
    }
    private static void Lifecycle()
    {
        var clock=new Clock();var system=new TrackingSystem(clock);var ar=new Provider("ar");var optional=new Provider("ots"){FailStart=true};
        system.AddProvider(ar);system.AddProvider(optional,false);system.BindObject("probe",new ObjectBinding("ar","p",RigidPose.Identity));system.Start();
        Check(system.State==TrackingState.Running&&system.GetProviderStatus()[1].Error!=null,"optional provider failure leaves required provider running");
        ar.Pose("p",1,9.99,Pose(1));Check(system.TryGetPose("probe",Req,out _,out _),"healthy provider readable after optional failure");
        ar.AsyncStop=true;system.Stop();Check(system.State==TrackingState.Stopping&&!system.TryGetPose("probe",Req,out _,out _),"async stop withdraws observations immediately");
        int starts=ar.Starts;system.Start();Check(ar.Starts==starts,"restart waits for pending stop");
        ar.FinishStop=true;system.Pump(TrackingUpdatePhase.Update);Check(ar.Starts==starts+1&&system.State==TrackingState.Running,"restart after resources released");
        ar.FailPump=true;system.Pump(TrackingUpdatePhase.Update);Check(system.State==TrackingState.Faulted&&!system.TryGetPose("probe",Req,out _,out _),"required provider failure fails closed");
        ar.FailPump=false;system.Stop();system.Pump(TrackingUpdatePhase.Update);system.Dispose();Check(ar.Disposes==1,"owned providers disposed once");
        var badSystem=new TrackingSystem(clock);var bad=new Provider("bad"){FailInitialize=true};Throws(()=>badSystem.AddProvider(bad),"initialize failure surfaces");Check(bad.Disposes==1,"partially initialized provider cleaned up");badSystem.Dispose();
        var resetSystem=new TrackingSystem(clock);var provider=new Provider("ar");resetSystem.AddProvider(provider);resetSystem.BindObject("p",new ObjectBinding("ar","p",RigidPose.Identity));resetSystem.Start();resetSystem.Pump(TrackingUpdatePhase.Update);
        clock.NowSeconds=1;resetSystem.Pump(TrackingUpdatePhase.Update);Check(resetSystem.State==TrackingState.Faulted,"runtime clock regression faults service");Throws(()=>resetSystem.Start(),"clock fault requires a new runtime");resetSystem.Stop();resetSystem.Dispose();
    }
    private static void Capture()
    {
        var d=new Device{Permission=false};var c=new CaptureDeviceLifecycle(d);c.Start();c.Pump();Check(c.State==ProviderState.Starting&&!d.OwnsResource,"permission wait does not acquire sensor");
        c.Stop();d.Permission=true;c.Pump();Check(c.Released&&d.Configures==0,"late permission grant cannot restart stopped capture");
        c.Start();c.Pump();Check(d.Configures==1,"configure acquired resource");c.Stop();c.Start();c.Pump();Check(d.Starts==0&&d.Releases==0,"stop waits for pending configure despite queued restart");
        d.Op.IsCompleted=true;c.Pump();Check(d.Starts==0&&d.Releases==1&&c.State==ProviderState.Starting,"cancelled configuration released before restart");
        c.Pump();d.Op.IsCompleted=true;c.Pump();Check(d.Starts==1,"start only follows completed configuration");
        c.Stop();d.IsStreaming=true;d.Op.IsCompleted=true;c.Pump();Check(d.Stops==1&&c.State==ProviderState.Stopping,"stop during SDK start waits then stops running sensor");
        d.IsStreaming=false;d.Op.IsCompleted=true;c.Pump();Check(c.Released&&c.State==ProviderState.Stopped,"async stop releases sensor");
        c.Start();c.Pump();d.Op.IsCompleted=true;c.Pump();d.IsStreaming=true;d.Op.IsCompleted=true;c.Pump();Check(c.State==ProviderState.Running&&d.Reads==1,"normal capture reads after start completion");
        c.Stop();c.Pump();d.Op.IsCompleted=true;d.Op.Succeeded=false;c.Pump();Check(!c.Released&&c.State==ProviderState.Stopping&&c.LastError!=null,"failed stop does not pretend resource released");
        int stops=d.Stops;c.Pump();Check(d.Stops==stops,"failed release does not spin retrying SDK calls");
        c.Stop();c.Pump();d.Op.IsCompleted=true;d.IsStreaming=false;c.Pump();Check(c.Released,"explicit stop retries cleanup");
        var d2=new Device();var c2=new CaptureDeviceLifecycle(d2);c2.Start();c2.Pump();c2.Stop();d2.Op.IsCompleted=true;d2.ReleaseFails=true;c2.Pump();Check(!c2.Released&&c2.State==ProviderState.Stopping,"release exception retains ownership");
        d2.ReleaseFails=false;c2.Stop();c2.Pump();Check(c2.Released,"release retried after explicit request");
    }
    private static void Replay()
    {
        var clock=new Clock();var system=new TrackingSystem(clock);
        var data=new List<ReplayPose>{new ReplayPose("p",1,0,.02,Pose(1)),new ReplayPose("p",2,.03,.04,Pose(0),false),new ReplayPose("p",3,.05,.06,Pose(2))};
        system.AddProvider(new ReplayTrackingProvider("recorded","recorded-world",data));system.BindObject("probe",new ObjectBinding("recorded","p",RigidPose.Identity));system.Start();
        data.Clear();system.Pump(TrackingUpdatePhase.Update);Check(!system.TryGetPose("probe",Req,out _,out _),"replay does not publish before recorded availability");
        clock.NowSeconds=10.025;system.Pump(TrackingUpdatePhase.Update);Check(system.TryGetPose("probe",Req,out var one,out _)&&Near(one.FrameFromObject.Position.X,1),"replay owns immutable input and preserves pose");
        clock.NowSeconds=10.045;system.Pump(TrackingUpdatePhase.Update);Check(!system.TryGetPose("probe",Req,out _,out _),"replayed loss withdraws observation");
        clock.NowSeconds=10.065;system.Pump(TrackingUpdatePhase.Update);Check(system.TryGetPose("probe",Req,out var two,out _)&&two.Source.Sequence==3,"replay reacquisition");
        clock.NowSeconds=11;Check(!system.TryGetPose("probe",Req,out _,out _),"EOF held observation ages normally");system.Stop();system.Dispose();
    }
}
