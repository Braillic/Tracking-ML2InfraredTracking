using System;

namespace Braillic.Tracking
{
    public static class Numeric
    {
        public static bool Finite(double x) => !double.IsNaN(x) && !double.IsInfinity(x);
        internal static string Required(string value, string name)
        { if (string.IsNullOrWhiteSpace(value)) throw new ArgumentException("Required identifier", name); return value; }
    }

    /// <summary>Metres, canonical right-handed basis. No engine dependency.</summary>
    public readonly struct Vector3d
    {
        public readonly double X, Y, Z;
        public Vector3d(double x, double y, double z) { X=x; Y=y; Z=z; }
        public bool IsFinite => Numeric.Finite(X) && Numeric.Finite(Y) && Numeric.Finite(Z);
        public static Vector3d operator +(Vector3d a, Vector3d b) => new Vector3d(a.X+b.X,a.Y+b.Y,a.Z+b.Z);
        public static Vector3d operator -(Vector3d a, Vector3d b) => new Vector3d(a.X-b.X,a.Y-b.Y,a.Z-b.Z);
        public static Vector3d operator *(double s, Vector3d a) => new Vector3d(s*a.X,s*a.Y,s*a.Z);
        public static Vector3d Cross(Vector3d a, Vector3d b) => new Vector3d(a.Y*b.Z-a.Z*b.Y,a.Z*b.X-a.X*b.Z,a.X*b.Y-a.Y*b.X);
    }

    /// <summary>Hamilton XYZW quaternion. A default zero quaternion is invalid.</summary>
    public readonly struct Quaterniond
    {
        public readonly double X,Y,Z,W;
        public Quaterniond(double x,double y,double z,double w) { X=x; Y=y; Z=z; W=w; }
        public static Quaterniond Identity => new Quaterniond(0,0,0,1);
        public double NormSquared => X*X+Y*Y+Z*Z+W*W;
        // Allow roundoff from float SDKs, but reject malformed scale rather than hiding it.
        public bool IsUnit => Numeric.Finite(NormSquared) && Math.Abs(NormSquared-1) <= 0.001;
        public Quaterniond Normalized
        {
            get { if (!IsUnit) throw new ArgumentException("Rotation must be a finite unit quaternion.");
                double s=1/Math.Sqrt(NormSquared); return new Quaterniond(s*X,s*Y,s*Z,s*W); }
        }
        public Quaterniond Inverse => new Quaterniond(-X,-Y,-Z,W);
        public Vector3d Rotate(Vector3d p)
        {
            var q=new Vector3d(X,Y,Z); var t=2*Vector3d.Cross(q,p);
            return p + W*t + Vector3d.Cross(q,t);
        }
        public static Quaterniond operator *(Quaterniond a,Quaterniond b) => new Quaterniond(
            a.W*b.X+a.X*b.W+a.Y*b.Z-a.Z*b.Y,
            a.W*b.Y-a.X*b.Z+a.Y*b.W+a.Z*b.X,
            a.W*b.Z+a.X*b.Y-a.Y*b.X+a.Z*b.W,
            a.W*b.W-a.X*b.X-a.Y*b.Y-a.Z*b.Z);
    }

    /// <summary>DestinationFromSource. Apply rotates a source point then adds translation.</summary>
    public readonly struct RigidPose
    {
        public readonly Vector3d Position;
        public readonly Quaterniond Rotation;
        public RigidPose(Vector3d position,Quaterniond rotation) { Position=position; Rotation=rotation; }
        public static RigidPose Identity => new RigidPose(new Vector3d(),Quaterniond.Identity);
        public bool IsValid => Position.IsFinite && Rotation.IsUnit;
        public RigidPose Normalized => new RigidPose(Position,Rotation.Normalized);
        public Vector3d Apply(Vector3d point) => Rotation.Rotate(point)+Position;
        public RigidPose Inverse { get { var r=Rotation.Inverse; return new RigidPose(r.Rotate(-1*Position),r); } }
        // AFromB * BFromC = AFromC; operands must already be valid normalized poses.
        public static RigidPose operator *(RigidPose a,RigidPose b) => new RigidPose(a.Apply(b.Position),(a.Rotation*b.Rotation).Normalized);
    }

    public sealed class CoordinateFrame : IEquatable<CoordinateFrame>
    {
        public string Id { get; }
        public string Epoch { get; }
        public CoordinateFrame(string id,string epoch) { Id=Numeric.Required(id,nameof(id)); Epoch=Numeric.Required(epoch,nameof(epoch)); }
        public bool Equals(CoordinateFrame other) => other != null && Id==other.Id && Epoch==other.Epoch;
        public override bool Equals(object obj) => Equals(obj as CoordinateFrame);
        public override int GetHashCode() => unchecked(Id.GetHashCode()*397 ^ Epoch.GetHashCode());
    }
}
