using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public interface ICaptureDevice
    {
        bool OwnsResource { get; }
        bool IsStreaming { get; }
        bool Prepare(); // Permission/device discovery may be pending; never blocks.
        ICaptureOperation Configure();
        ICaptureOperation Start();
        ICaptureOperation Stop();
        void Release();
        void Read();
    }
}
