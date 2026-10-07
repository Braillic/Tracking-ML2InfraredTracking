using System;
using System.Collections.Generic;

namespace Braillic.Tracking
{
    public interface ITrackingReader
    {
        bool TryGetLatest(string sourceId,string objectId,double maximumAgeSeconds,
            out PoseObservation observation,out TrackingIssue issue);
        bool TryGetRelative(string sourceId,string objectId,string referenceSourceId,string referenceId,
            double maximumAgeSeconds,double maximumCaptureSkewSeconds,
            out RelativeObservation observation,out TrackingIssue issue);
    }
}
