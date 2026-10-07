using System;

namespace Braillic.Tracking
{
    /// <summary>One body in a complete multi-body frame; frame time and sequence are shared.</summary>
    public readonly struct TrackedBodySample
    {
        public readonly string ObjectId;
        public readonly RigidPose ReferenceFromBody;
        public readonly double? Quality;
        public TrackedBodySample(string objectId, RigidPose pose, double? quality = null)
        {
            if (string.IsNullOrWhiteSpace(objectId)) throw new ArgumentException("Body ID required.");
            ObjectId = objectId; ReferenceFromBody = pose; Quality = quality;
        }
    }
}
