using System;
using ProbeTracing;

public static class ProbeTraceTests
{
    private static int count;
    private static void Check(bool value,string label){if(!value)throw new Exception(label);count++;}
    private static bool Add(ProbeTraceBuffer b,ulong id,double t,double x=0,double y=0,ulong session=7,double now=-1)
        =>b.Observe(session,id,t,now<0?t+.02:now,.15,x,y,0,.9f);
    public static string Run()
    {
        var b=new ProbeTraceBuffer();
        Check(!b.Start(0,1),"zero session rejected");
        Check(b.Start(7,1),"start");
        Check(!Add(b,1,1),"do not record already displayed start frame");
        Check(Add(b,2,1.03),"first fresh point");
        Check(!Add(b,2,1.03),"duplicate ignored");
        Check(!Add(b,1,1.04),"old frame ignored");
        Check(!Add(b,3,1.06,.0001),"spatial thinning");
        Check(Add(b,4,1.09,.003),"sufficient movement");
        Check(b.Points[0].stroke==b.Points[1].stroke,"continuous stroke");
        b.Pause();Check(!Add(b,5,1.12,.006),"paused samples ignored");
        Check(b.Start(7,5)&&Add(b,6,1.15,.006),"resume new observation");
        Check(b.Points[2].stroke!=b.Points[1].stroke,"resume breaks line");
        Check(Add(b,7,1.4,.009)&&b.Points[3].stroke!=b.Points[2].stroke,"time gap breaks line");
        Check(Add(b,8,1.43,.10)&&b.Points[4].stroke!=b.Points[3].stroke,"large jump breaks line");
        Check(!Add(b,9,1.46,.11,now:1.7),"old capture rejected");
        Check(!Add(b,10,1.49,double.NaN),"nan coordinate rejected");
        Check(!Add(b,11,1.52,.11,now:1.50),"future capture rejected");
        Check(Add(b,12,1.55,.103)&&b.Points[5].stroke!=b.Points[4].stroke,"bad observations break stroke");
        Check(!Add(b,13,1.50,.106),"capture time regression rejected");
        int before=b.Points.Count;b.UndoStroke();Check(b.Points.Count==before-1&&!b.Recording,"undo whole final stroke and pause");
        var copy=b.Snapshot();copy[0].xMetres=999;Check(b.Points[0].xMetres!=999,"snapshot independent");
        b.CheckSession(8);Check(b.Invalidated&&!b.Start(8,20),"epoch change freezes trace");
        Check(b.Points.Count>0,"old epoch points retained for export");
        var limited=new ProbeTraceBuffer(.002,.12,.03,2);limited.Start(7,0);
        Add(limited,1,2);Add(limited,2,2.03,.003);
        Check(limited.Full&&!limited.Recording&&!limited.Start(7,2),"capacity stops recording");
        limited.UndoStroke();Check(limited.Points.Count==0&&limited.Start(7,2),"undo recovers capacity");
        var stationary=new ProbeTraceBuffer();stationary.Start(7,0);
        for(ulong i=1;i<=30;i++)Add(stationary,i,3+i*.03);
        Check(stationary.Points.Count==1,"stationary tracking does not create artificial time gaps");
        stationary.TrackingGap();Check(Add(stationary,31,3.93),"tracking loss starts a new stroke even at same position");
        var changed=new ProbeTraceBuffer();changed.Start(7,0);Add(changed,1,5);
        Check(!Add(changed,2,5.03,.003,session:8)&&changed.Invalidated,"incoming wrong session freezes trace");
        var ids=new ProbeTraceBuffer();ulong big=ulong.MaxValue-1;ids.Start(big,0);Add(ids,1,6,session:big);
        Check(ids.Points[0].sessionId==big.ToString(),"uint64 IDs preserve export precision");
        bool invalid=false;try{new ProbeTraceBuffer(double.NaN);}catch(ArgumentException){invalid=true;}
        Check(invalid,"invalid sampling configuration rejected");
        return count+" probe tracing checks passed";
    }
}
