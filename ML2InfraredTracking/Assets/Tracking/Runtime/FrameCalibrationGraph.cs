using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    /// <summary>A forest of explicit rigid frame calibrations. Cycles/ambiguous paths are rejected.</summary>
    public sealed class FrameCalibrationGraph
    {
        private sealed class Edge
        {
            public string Id;
            public CoordinateFrame From,To;
            public RigidPose ToFrom;
            public double ValidFrom,ValidUntil;
        }
        private readonly List<Edge> edges=new List<Edge>();
        public long Revision { get; private set; }
        public void Set(string id,CoordinateFrame from,CoordinateFrame to,RigidPose toFrom,double validFrom,double validUntil)
        {
            if(string.IsNullOrWhiteSpace(id)||from==null||to==null||from.Equals(to)||!toFrom.IsValid||
                !Numeric.Finite(validFrom)||!Numeric.Finite(validUntil)||validFrom<0||validUntil<validFrom)throw new ArgumentException("Invalid frame calibration.");
            var candidate=new List<Edge>(edges);candidate.RemoveAll(e=>e.Id==id);
            if(Resolve(candidate,from,to,0,false,out _))throw new ArgumentException("Calibration creates an ambiguous cycle. Remove/replace an existing edge first.");
            candidate.Add(new Edge{Id=id,From=from,To=to,ToFrom=toFrom.Normalized,ValidFrom=validFrom,ValidUntil=validUntil});
            edges.Clear();edges.AddRange(candidate);Revision++;
        }
        public void Remove(string id) { if(edges.RemoveAll(e=>e.Id==id)>0)Revision++; }
        public void RemoveFrame(CoordinateFrame frame)
        { if(frame!=null&&edges.RemoveAll(e=>e.From.Equals(frame)||e.To.Equals(frame))>0)Revision++; }
        public bool TryResolve(CoordinateFrame from,CoordinateFrame to,double now,out RigidPose toFrom)
        { return Resolve(edges,from,to,now,true,out toFrom); }
        private static bool Resolve(List<Edge> source,CoordinateFrame from,CoordinateFrame to,double now,bool checkTime,out RigidPose result)
        {
            result=default;if(from==null||to==null||!Numeric.Finite(now))return false;
            if(from.Equals(to)){result=RigidPose.Identity;return true;}
            var visited=new HashSet<CoordinateFrame>{from};
            var queue=new Queue<KeyValuePair<CoordinateFrame,RigidPose>>();
            queue.Enqueue(new KeyValuePair<CoordinateFrame,RigidPose>(from,RigidPose.Identity));
            while(queue.Count>0)
            {
                var current=queue.Dequeue();
                foreach(var e in source)
                {
                    if(checkTime&&(now<e.ValidFrom||now>e.ValidUntil))continue;
                    CoordinateFrame next;RigidPose step;
                    if(e.From.Equals(current.Key)){next=e.To;step=e.ToFrom;}
                    else if(e.To.Equals(current.Key)){next=e.From;step=e.ToFrom.Inverse;}
                    else continue;
                    if(!visited.Add(next))continue;
                    var composed=step*current.Value;
                    if(next.Equals(to)){result=composed;return true;}
                    queue.Enqueue(new KeyValuePair<CoordinateFrame,RigidPose>(next,composed));
                }
            }
            return false;
        }
    }
}
