using Braillic.Tracking;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    public static class TrackingCoordinates
    {
        // B = diag(1,1,-1), applied to BOTH world and body bases. Involution.
        public static RigidPose FromUnity(Vector3 p, Quaternion q) => new RigidPose(
            new Vector3d(p.x, p.y, -p.z), new Quaterniond(-q.x, -q.y, q.z, q.w));
        public static Vector3 ToUnity(Vector3d p) => new Vector3((float)p.X, (float)p.Y, (float)-p.Z);
        public static Quaternion ToUnity(Quaterniond q) => new Quaternion((float)-q.X, (float)-q.Y, (float)q.Z, (float)q.W);
    }
}
