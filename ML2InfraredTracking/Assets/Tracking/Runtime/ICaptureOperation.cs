using System;
using System.Collections.Generic;

namespace Braillic.Tracking.Runtime
{
    public interface ICaptureOperation { bool IsCompleted { get; } bool Succeeded { get; } }
}
