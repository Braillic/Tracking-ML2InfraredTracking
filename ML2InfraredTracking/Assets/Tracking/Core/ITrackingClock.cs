using System;
using System.Collections.Generic;

namespace Braillic.Tracking
{
    public interface ITrackingClock { string Id { get; } double NowSeconds { get; } }
}
