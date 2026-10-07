using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Application
{
    public interface ITrackingApplicationBackend : IDisposable
    {
        void Start();
        void Stop();
        void Pump(TrackingPhase phase);
        void ResetReferenceFrame();
    }
}
