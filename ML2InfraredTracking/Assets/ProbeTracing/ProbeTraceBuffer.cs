using System;
using System.Collections.Generic;

namespace ProbeTracing
{
    [Serializable]
    public struct TracePoint
    {
        // IDs are strings so JSON readers do not lose precision on uint64 IDs.
        public string sessionId, frameId;
        public double captureTimeSeconds, xMetres, yMetres, zMetres;
        public float confidence;
        public int stroke;
    }

    /// Pure managed sampling state; no rendering, transport, or Unity dependencies.
    public sealed class ProbeTraceBuffer
    {
        private readonly List<TracePoint> points = new List<TracePoint>();
        private readonly double spacing, maxGap, maxStep;
        private readonly int capacity;
        private ulong lastFrame;
        private double lastCapture = double.NegativeInfinity;
        private bool haveFrame, breakStroke = true;
        private int stroke;
        public ulong SessionId { get; private set; }
        public bool Recording { get; private set; }
        public bool Invalidated { get; private set; }
        public IReadOnlyList<TracePoint> Points => points;
        public bool Full => points.Count >= capacity;
        public ProbeTraceBuffer(double spacingMetres=.002, double maximumGapSeconds=.12,
            double maximumStepMetres=.03, int maximumPoints=3000)
        {
            if(!Finite(spacingMetres)||!Finite(maximumGapSeconds)||!Finite(maximumStepMetres)
                ||spacingMetres<=0||maximumGapSeconds<=0||maximumStepMetres<spacingMetres||maximumPoints<1)
                throw new ArgumentException("Invalid trace sampling settings.");
            spacing=spacingMetres;maxGap=maximumGapSeconds;maxStep=maximumStepMetres;capacity=maximumPoints;
        }
        private static bool Finite(double v)=>!double.IsNaN(v)&&!double.IsInfinity(v);
        public bool Start(ulong sessionId, ulong currentFrame)
        {
            if(sessionId==0||Invalidated||Full)return false;
            if(SessionId!=0&&sessionId!=SessionId){Invalidate();return false;}
            SessionId=sessionId;lastFrame=currentFrame;haveFrame=true;
            Recording=true;breakStroke=true;lastCapture=double.NegativeInfinity;return true;
        }
        public void Pause(){Recording=false;breakStroke=true;}
        public void TrackingGap(){breakStroke=true;}
        public void Invalidate(){Pause();Invalidated=true;}
        public void CheckSession(ulong sessionId){if(SessionId!=0&&SessionId!=sessionId)Invalidate();}
        public bool Observe(ulong sessionId,ulong frameId,double captureTime,double now,double maxAge,
            double x,double y,double z,float confidence)
        {
            CheckSession(sessionId);
            if(!Recording||Invalidated)return false;
            if(haveFrame&&frameId<=lastFrame)return false;
            lastFrame=frameId;haveFrame=true;
            if(!Finite(captureTime)||!Finite(now)||!Finite(maxAge)||maxAge<=0||now<captureTime||now-captureTime>maxAge
                ||!Finite(x)||!Finite(y)||!Finite(z)||!Finite(confidence)||captureTime<=lastCapture)
            {TrackingGap();return false;}
            if(captureTime-lastCapture>maxGap)breakStroke=true;
            lastCapture=captureTime;
            if(points.Count>0)
            {
                var p=points[points.Count-1];double dx=x-p.xMetres,dy=y-p.yMetres,dz=z-p.zMetres,d=dx*dx+dy*dy+dz*dz;
                if(d>maxStep*maxStep)breakStroke=true;
                if(!breakStroke&&d<spacing*spacing)return false;
            }
            if(Full){Pause();return false;}
            if(breakStroke){stroke++;breakStroke=false;}
            points.Add(new TracePoint{sessionId=sessionId.ToString(),frameId=frameId.ToString(),captureTimeSeconds=captureTime,
                xMetres=x,yMetres=y,zMetres=z,confidence=confidence,stroke=stroke});
            if(Full)Pause();return true;
        }
        public void UndoStroke()
        {
            Pause();if(points.Count==0)return;int last=points[points.Count-1].stroke;
            points.RemoveAll(p=>p.stroke==last);
        }
        public TracePoint[] Snapshot()=>points.ToArray();
    }
}
