using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public interface ITrackingProvider : IDisposable
    {
        string Id { get; }
        ProviderState State { get; }
        string LastError { get; }
        void Initialize(TrackingProviderContext context);
        void Start();
        void Stop();
        void Pump(TrackingUpdatePhase phase);
        void ResetReferenceFrame();
    }
}
