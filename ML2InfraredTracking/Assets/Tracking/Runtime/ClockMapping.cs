using System;

namespace Braillic.Tracking.Runtime
{
    /// <summary>An explicitly calibrated source -> runtime clock map. No arrival-time substitution.</summary>
    public sealed class ClockMapping
    {
        public string SourceClockId { get; }
        public string TargetClockId { get; }
        public double Scale { get; }
        public double OffsetSeconds { get; }
        public double UncertaintySeconds { get; }
        public double ValidFrom { get; }
        public double ValidUntil { get; }
        public ClockMapping(string sourceClockId,string targetClockId,double scale,double offsetSeconds,
            double uncertaintySeconds,double validFrom,double validUntil)
        {
            if(string.IsNullOrWhiteSpace(sourceClockId)||string.IsNullOrWhiteSpace(targetClockId))throw new ArgumentException("Clock identities required.");
            if(!Numeric.Finite(scale)||scale<=0||!Numeric.Finite(offsetSeconds)||!Numeric.Finite(uncertaintySeconds)||uncertaintySeconds<0||
                !Numeric.Finite(validFrom)||!Numeric.Finite(validUntil)||validFrom<0||validUntil<validFrom)throw new ArgumentException("Invalid clock calibration.");
            SourceClockId=sourceClockId;TargetClockId=targetClockId;Scale=scale;OffsetSeconds=offsetSeconds;
            UncertaintySeconds=uncertaintySeconds;ValidFrom=validFrom;ValidUntil=validUntil;
        }
        public static ClockMapping Identity(string clockId) => new ClockMapping(clockId,clockId,1,0,0,0,double.MaxValue);
        public bool TryMap(double sourceSeconds,double now,out double mapped)
        {
            mapped=Scale*sourceSeconds+OffsetSeconds;
            return Numeric.Finite(sourceSeconds)&&sourceSeconds>=0&&Numeric.Finite(now)&&now>=ValidFrom&&now<=ValidUntil&&
                Numeric.Finite(mapped)&&mapped>=ValidFrom&&mapped<=ValidUntil;
        }
        public bool IsCurrent(double now) => Numeric.Finite(now)&&now>=ValidFrom&&now<=ValidUntil;
    }
}
