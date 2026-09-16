// Test-only managed math. Application code compiles separately against real Unity assemblies.
namespace UnityEngine
{
    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x=x; this.y=y; this.z=z; }
        public static Vector3 Lerp(Vector3 a, Vector3 b, float t)
            => new Vector3(a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t);
        public Vector3 InvertZ() => new Vector3(x,y,-z);
    }
    public struct Quaternion
    {
        public float x,y,z,w;
        public Quaternion(float x,float y,float z,float w) { this.x=x; this.y=y; this.z=z; this.w=w; }
        public static Quaternion identity => new Quaternion(0,0,0,1);
        public Quaternion normalized
        {
            get { float n=(float)System.Math.Sqrt(x*x+y*y+z*z+w*w); return new Quaternion(x/n,y/n,z/n,w/n); }
        }
        public Quaternion InvertXY() => new Quaternion(-x,-y,z,w);
        public static Quaternion Slerp(Quaternion a,Quaternion b,float t)
        {
            var q=System.Numerics.Quaternion.Slerp(
                new System.Numerics.Quaternion(a.x,a.y,a.z,a.w),
                new System.Numerics.Quaternion(b.x,b.y,b.z,b.w),t);
            return new Quaternion(q.X,q.Y,q.Z,q.W);
        }
    }
    public struct Pose
    {
        public Vector3 position;
        public Quaternion rotation;
        public Pose(Vector3 p,Quaternion r) { position=p; rotation=r; }
        public static Pose identity => new Pose(new Vector3(0,0,0),Quaternion.identity);
    }
    public static class Time { public static double realtimeSinceStartupAsDouble; }
}

