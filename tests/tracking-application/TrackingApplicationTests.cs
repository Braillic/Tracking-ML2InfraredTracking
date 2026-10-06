using System;
using Braillic.Tracking;
using Braillic.Tracking.Application;

internal static class TrackingApplicationTests
{
    private sealed class Clock : ITrackingClock { public string Id => "test-clock"; public double NowSeconds { get; set; } = 10; }
    private sealed class Backend : ITrackingApplicationBackend
    {
        public readonly Clock Clock = new Clock();
        public readonly TrackingCore Core;
        public TrackingSession Session;
        public int Starts, Stops, Pumps, Disposes;
        public bool FailStart, FailPump, FailStop;
        public TrackingPhase LastPhase;
        private int epoch;
        public Backend() { Core = new TrackingCore(Clock); }
        public void Start()
        {
            Starts++;
            Session = Core.OpenSession("source", "session" + (++epoch), Clock.Id,
                new CoordinateFrame("world", epoch.ToString()), "calibration");
            if (FailStart) throw new InvalidOperationException("start failed after opening session");
        }
        public void Stop()
        {
            Stops++;
            if (Session != null) Core.CloseSession(Session);
            if (FailStop) throw new InvalidOperationException("cleanup failed");
        }
        public void Pump(TrackingPhase phase)
        { Pumps++; LastPhase=phase; if (FailPump) throw new InvalidOperationException("pump failed"); }
        public void ResetReferenceFrame() { Stop(); Start(); }
        public void Dispose() { Disposes++; }
        public void Publish(ulong sequence=1)
        { Check(Core.Publish(Session,"probe",sequence,Clock.NowSeconds-.02,Clock.NowSeconds,RigidPose.Identity)==TrackingIssue.None,"publish fixture"); }
    }
    private static int checks;
    private static void Check(bool ok,string label) { if(!ok)throw new Exception(label); checks++; }
    private static bool Read(TrackingApplication app) => app.TryGetLatest("source","probe",.15,out _,out _);
    public static int Main()
    {
        var backend = new Backend();
        var app = new TrackingApplication(backend.Core,backend);
        Check(!Read(app) && app.State==TrackingApplicationState.Stopped,"initial state has no readable measurement");
        app.Pump(TrackingPhase.BeforeRender); Check(backend.Pumps==0,"stopped backend not pumped");
        app.Start();app.Start(); Check(backend.Starts==1,"start idempotent");
        backend.Publish();Check(Read(app),"application reads core observation");
        app.Pump(TrackingPhase.LateUpdate);app.Pump(TrackingPhase.BeforeRender);
        Check(backend.Pumps==2 && backend.LastPhase==TrackingPhase.BeforeRender,"both application phases delegated");
        app.TryGetLatest("source","probe",.15,out var first,out _);
        app.Pump(TrackingPhase.BeforeRender);
        app.TryGetLatest("source","probe",.15,out var repeated,out _);
        Check(first.CaptureSeconds==repeated.CaptureSeconds && first.Sequence==repeated.Sequence,"render polling never invents measurements");
        backend.Clock.NowSeconds+=.2;Check(!Read(app),"application respects capture freshness");
        backend.Publish(2);Check(Read(app),"new capture restores freshness");
        app.SetSuspended(true);Check(app.State==TrackingApplicationState.Paused && !Read(app),"pause withdraws measurements");
        int count=backend.Pumps;app.Pump(TrackingPhase.BeforeRender);Check(backend.Pumps==count,"no work during pause");
        app.SetSuspended(false);Check(app.State==TrackingApplicationState.Running && backend.Starts==2 && !Read(app),"resume creates fresh session");
        backend.Publish();Check(Read(app),"sequence may restart in new session");
        app.ResetReferenceFrame();Check(!Read(app),"origin reset cannot reuse old measurement");
        backend.Publish();app.TryGetLatest("source","probe",.15,out var reset,out _);
        Check(!first.Session.ReferenceFrame.Equals(reset.Session.ReferenceFrame),"origin epoch changes");
        backend.Core.Publish(backend.Session,"reference",1,backend.Clock.NowSeconds-.02,backend.Clock.NowSeconds,RigidPose.Identity);
        Check(app.TryGetRelative("source","probe","source","reference",.15,0,out _,out _),"relative API forwarded");
        app.Stop();Check(!Read(app),"stop withdraws measurement immediately");
        app.SetSuspended(true);app.SetSuspended(false);Check(app.State==TrackingApplicationState.Stopped,"pause cycle cannot undo explicit Stop");
        app.SetSuspended(true);count=backend.Starts;app.Start();Check(backend.Starts==count && app.State==TrackingApplicationState.Paused,"start while suspended is deferred");
        app.Stop();app.SetSuspended(false);Check(backend.Starts==count,"stop cancels deferred start");
        app.Start();backend.FailPump=true;app.Pump(TrackingPhase.LateUpdate);
        Check(app.State==TrackingApplicationState.Faulted && !Read(app) && app.LastError=="pump failed","pump exception fails closed");
        app.SetSuspended(true);app.SetSuspended(false);Check(app.State!=TrackingApplicationState.Running,"fault does not silently restart");
        backend.FailPump=false;app.Start();Check(app.LastError==null,"explicit restart clears error");
        backend.FailStop=true;app.Stop();Check(app.State==TrackingApplicationState.Faulted && !Read(app),"cleanup failure still blocks reads");
        backend.FailStop=false;app.Dispose();app.Dispose();Check(backend.Disposes==1,"dispose idempotent");
        Check(!Read(app),"disposed reader fails closed");
        bool thrown=false;try{app.Start();}catch(ObjectDisposedException){thrown=true;}Check(thrown,"disposed application cannot restart");
        var failing=new Backend{FailStart=true};var failApp=new TrackingApplication(failing.Core,failing);
        failApp.Start();Check(failApp.State==TrackingApplicationState.Faulted && failing.Stops==1 && !Read(failApp),"partial startup rolls back");
        failApp.Dispose();
        Console.WriteLine("PASS: " + checks + " tracking application checks");
        return 0;
    }
}
