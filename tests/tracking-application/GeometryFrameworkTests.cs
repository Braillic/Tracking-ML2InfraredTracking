using System;
using System.Collections.Generic;
using Braillic.Tracking;
using Braillic.Tracking.Runtime;

internal static class GeometryFrameworkTests
{
    private sealed class Clock : ITrackingClock { public string Id=>"clock";public double NowSeconds{get;set;}=10; }
    private sealed class Provider : ITrackingProvider, IGeometryTrackingProvider
    {
        public string Id=>"ar"; public ProviderState State{get;private set;} public string LastError=>null;
        public TrackingProviderContext Context; public int MaximumTools=64;
        public void Initialize(TrackingProviderContext context){Context=context;}
        public void ValidateGeometryConfiguration(ProviderGeometryConfiguration config)
        {if(config.Tools.Count>MaximumTools)throw new NotSupportedException("capacity");}
        public void Start(){State=ProviderState.Running;Context.BeginSession(Guid.NewGuid().ToString(),new CoordinateFrame("world",Guid.NewGuid().ToString()),"calib",ClockMapping.Identity(Context.Clock.Id));}
        public void Stop(){Context.EndSession();State=ProviderState.Stopped;}
        public void ResetReferenceFrame(){Start();}
        public void Pump(TrackingUpdatePhase phase){}
        public void Dispose(){}
    }
    // Synthetic results exercise the extension contract; this is deliberately not a detection algorithm.
    private sealed class FixtureIdentifier : IGeometryIdentifier<IReadOnlyList<GeometryMatch>>
    {
        public IReadOnlyList<GeometryMatch> Identify(IReadOnlyList<GeometryMatch> frame,IReadOnlyList<MarkerGeometryDefinition> geometries)=>frame;
    }
    private static int checks;
    private static void Check(bool ok,string label){if(!ok)throw new Exception(label);checks++;}
    private static void Throws(Action action,string label){try{action();}catch(ArgumentException){checks++;return;}catch(InvalidOperationException){checks++;return;}catch(NotSupportedException){checks++;return;}throw new Exception(label);}
    private static MarkerGeometryDefinition Geometry(string id,double size=1)=>new MarkerGeometryDefinition(id,"1",new[]{new Vector3d(0,0,0),new Vector3d(size,0,0),new Vector3d(0,size,0)});
    private static TrackedToolDefinition Tool(string id,string geometry,TrackedToolRole role=TrackedToolRole.Probe)=>
        new TrackedToolDefinition(id,id,role,new ToolGeometryBinding(new ObjectBinding("ar","body/"+id,RigidPose.Identity),geometry));
    private static GeometryMatch Match(string id,int first,double x=0,string revision="1")=>new GeometryMatch(id,revision,
        new RigidPose(new Vector3d(x,0,0),Quaterniond.Identity),new[]{first,first+1,first+2},.9);
    public static int Main()
    {
        var clock=new Clock();var provider=new Provider();var runtime=new TrackingSystem(clock);runtime.AddProvider(provider);
        var g1=Geometry("probe-a");var g2=Geometry("probe-b",2);var gr=Geometry("reference",3);
        var gs=new[]{g1,g2,gr};var ts=new[]{Tool("probe-1",g1.Id),Tool("probe-2",g2.Id),Tool("patient-drf",gr.Id,TrackedToolRole.DynamicReferenceFrame)};
        var config=new TrackingConfiguration(gs,ts);runtime.ConfigureTools(config);
        gs[0]=gr;ts[0]=ts[2];
        Check(config.Geometries[0]==g1&&config.Tools[0].ObjectId=="probe-1","configuration snapshots caller arrays");
        Check(runtime.TryGetToolDefinition("patient-drf",out var drf)&&drf.Role==TrackedToolRole.DynamicReferenceFrame,"UI-selected DRF role preserved");
        Check(runtime.TryGetToolDefinition("probe-2",out var probe)&&probe.Role==TrackedToolRole.Probe,"two probes have distinct instance IDs");
        Check(provider.Context.GeometryConfiguration.Tools.Count==3,"one provider receives three selected geometries");
        Throws(()=>runtime.BindObject("bypass",new ObjectBinding("ar","body",RigidPose.Identity)),"legacy routing cannot bypass registered configuration");
        Throws(()=>new TrackingConfiguration(new[]{g1},new[]{Tool("a",g1.Id),Tool("b",g1.Id)}),"duplicate shape identity is ambiguous");
        Throws(()=>new TrackingConfiguration(new[]{g1},new[]{Tool("a","missing")}),"unknown geometry rejected");
        Throws(()=>new TrackingConfiguration(new[]{g1},new[]{Tool("a",g1.Id),Tool("a",g1.Id)}),"duplicate tool ID rejected");
        Throws(()=>new MarkerGeometryDefinition("bad","1",new[]{new Vector3d(0,0,0),new Vector3d(1,0,0),new Vector3d(2,0,0)}),"collinear model rejected");
        Throws(()=>new MarkerGeometryDefinition("bad","1",new[]{new Vector3d(0,0,0),new Vector3d(1,0,0),new Vector3d(double.NaN,1,0)}),"nonfinite model rejected");
        provider.MaximumTools=1;
        Throws(()=>runtime.ConfigureTools(config),"provider rejects unsupported geometry capacity");
        Check(ReferenceEquals(runtime.Configuration,config)&&provider.Context.GeometryConfiguration.Tools.Count==3,"failed reconfiguration preserves previous selection");
        provider.MaximumTools=64;runtime.Start();
        Throws(()=>runtime.ConfigureTools(config),"cannot change identity while running");
        var limits=new ObservationRequirements(1,.001,0);var token=provider.Context.Session;
        int events=0;bool completeAtEvent=false;
        runtime.ObservationPublished+=()=>{events++;if(events==1)completeAtEvent=runtime.TryGetRelativePose("probe-2","patient-drf",limits,out var relative,out _)&&Math.Abs(relative.FrameFromObject.Position.X-2)<1e-10;};
        var identifier=new FixtureIdentifier();
        var matches=identifier.Identify(new[]{Match(g1.Id,0,2),Match(g2.Id,3,3),Match(gr.Id,6,1)},provider.Context.GeometryConfiguration.Geometries);
        Check(provider.Context.PublishIdentifiedFrame(token,1,9.99,10,matches)==TrackingIssue.None,"simultaneous multi-body frame accepted");
        Check(events==1&&completeAtEvent,"consumer sees both probes and DRF atomically in one event");
        Check(runtime.TryGetRelativePose("probe-1","patient-drf",limits,out var pose,out _)&&Math.Abs(pose.FrameFromObject.Position.X-1)<1e-10,"patient-relative pose uses correct routed bodies");
        Check(pose.Source.Sequence==1&&pose.Reference.Value.Sequence==1,"both observations retain original frame sequence");
        clock.NowSeconds=10.1;
        Check(provider.Context.PublishIdentifiedFrame(token,2,10.09,10.1,new[]{Match(g1.Id,0,2),Match(gr.Id,6,1)})==TrackingIssue.None,"partial visibility accepted");
        Check(!runtime.TryGetPose("probe-2",limits,out _,out _)&&runtime.TryGetPose("probe-1",limits,out _,out _),"missing probe withdrawn independently");
        Check(provider.Context.PublishIdentifiedFrame(token,1,9.99,10,matches)==TrackingIssue.OutOfOrder,"old multi-body packet cannot restore missing probe");
        clock.NowSeconds=10.2;
        Check(provider.Context.PublishIdentifiedFrame(token,3,10.19,10.2,new[]{Match(g1.Id,0),Match(g1.Id,3)})==TrackingIssue.AmbiguousGeometry,"duplicate matches rejected");
        Check(!runtime.TryGetPose("probe-1",limits,out _,out _)&&!runtime.TryGetPose("patient-drf",limits,out _,out _),"ambiguous frame withdraws all candidates");
        clock.NowSeconds=10.3;
        Check(provider.Context.PublishIdentifiedFrame(token,4,10.29,10.3,new[]{Match(g1.Id,0),Match(g2.Id,2)})==TrackingIssue.AmbiguousGeometry,"shared detection cannot belong to two bodies");
        clock.NowSeconds=10.4;
        Check(provider.Context.PublishIdentifiedFrame(token,5,10.39,10.4,new[]{Match("unknown",0)})==TrackingIssue.UnknownGeometry,"unknown geometry never becomes probe by default");
        clock.NowSeconds=10.5;
        Check(provider.Context.PublishIdentifiedFrame(token,6,10.49,10.5,new[]{Match(g1.Id,0,0,"old")})==TrackingIssue.GeometryRevisionMismatch,"wrong model revision rejected");
        clock.NowSeconds=10.6;
        Check(provider.Context.PublishIdentifiedFrame(token,7,10.59,10.6,matches)==TrackingIssue.None,"new valid frame recovers all tools");
        clock.NowSeconds=10.7;
        Check(provider.Context.PublishIdentifiedFrame(token,8,10.69,10.7,Array.Empty<GeometryMatch>())==TrackingIssue.None,"empty completed detection frame is valid loss");
        Check(!runtime.TryGetPose("probe-1",limits,out _,out _)&&!runtime.TryGetPose("probe-2",limits,out _,out _),"empty frame withdraws all tools");
        Check(provider.Context.Publish(token,"body/probe-1",7,10.59,10.6,RigidPose.Identity)==TrackingIssue.OutOfOrder,"single publication cannot revive an older complete frame");
        Check(provider.Context.Publish(token,"unregistered",9,10.69,10.7,RigidPose.Identity)==TrackingIssue.UnknownSource,"unconfigured body publication rejected");
        runtime.Stop();long oldRevision=runtime.Revision;
        runtime.ConfigureTools(new TrackingConfiguration(new[]{g2},new[]{Tool("new-probe",g2.Id)}));runtime.Start();
        Check(runtime.Revision>oldRevision&&!runtime.TryGetToolDefinition("probe-1",out _),"replacement removes old tool metadata and changes revision");
        Check(provider.Context.PublishIdentifiedFrame(token,99,10.69,10.7,matches)==TrackingIssue.SessionSuperseded,"old configuration token cannot publish");
        runtime.Stop();runtime.Dispose();
        Console.WriteLine("PASS: "+checks+" geometry configuration, identity routing and atomic multi-body checks");return 0;
    }
}
