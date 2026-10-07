using System;
using System.Collections.Generic;

namespace Braillic.Tracking
{
    public interface ITrackingBackend : IDisposable
    {
        void Start();
        void Poll();
        void Stop();
    }
}
