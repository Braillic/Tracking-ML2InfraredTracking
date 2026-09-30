using System;
using System.Collections.Generic;
using ProbeTracing;

// Minimal scene API doubles. The geometry/render orchestration methods below are extracted
// from the actual component; Unity API availability is checked separately with real references.
public struct Vector3
{
    public float x,y,z;
    public Vector3(float a,float b,float c){x=a;y=b;z=c;}
    public static Vector3 one=>new Vector3(1,1,1);
    public static Vector3 up=>new Vector3(0,1,0);
    public float sqrMagnitude=>x*x+y*y+z*z;
    public float magnitude=>(float)Math.Sqrt(sqrMagnitude);
    public Vector3 normalized=>this*(1/magnitude);
    public static Vector3 operator +(Vector3 a,Vector3 b)=>new Vector3(a.x+b.x,a.y+b.y,a.z+b.z);
    public static Vector3 operator -(Vector3 a,Vector3 b)=>new Vector3(a.x-b.x,a.y-b.y,a.z-b.z);
    public static Vector3 operator *(Vector3 a,float b)=>new Vector3(a.x*b,a.y*b,a.z*b);
}
public struct Quaternion
{
    public static Vector3 LastFrom,LastTo;
    public static Quaternion FromToRotation(Vector3 from,Vector3 to){LastFrom=from;LastTo=to;return default;}
}
public class Transform
{
    public Transform parent;
    public Vector3 localPosition,position,localScale=Vector3.one,lossyScale=Vector3.one;
    public Quaternion localRotation,rotation;
}
public class Collider { public bool enabled=true; }
public class GameObject
{
    public string name;
    public Transform transform=new Transform();
    public Collider collider=new Collider();
    public bool active=true,destroyed;
    public void SetActive(bool value){active=value;}
    public T[] GetComponentsInChildren<T>(bool includeInactive)=>new[]{(T)(object)collider};
}
public sealed class SceneHarness
{
    public Transform trackedToolRoot=new Transform(),probeConnection=new Transform(),probeCylinder=new Transform(),transform=new Transform();
    public GameObject tracePointPrefab=new GameObject();
    public Vector3 tipOffsetMetres=new Vector3(0,0,.15f);
    public List<GameObject> pointVisuals=new List<GameObject>();
    public ProbeTraceBuffer buffer=new ProbeTraceBuffer();
    public string status;
    public static GameObject LastOriginal;
    public SceneHarness(){probeConnection.parent=probeCylinder.parent=trackedToolRoot;probeConnection.localPosition=new Vector3(0,0,.03f);probeConnection.localScale=new Vector3(.001f,.001f,.001f);probeCylinder.localScale=new Vector3(.005f,.05f,.005f);}
    public static GameObject Instantiate(GameObject original,Vector3 position,Quaternion rotation,Transform parent)
    {
        LastOriginal=original;var clone=new GameObject();clone.transform.position=position;clone.transform.rotation=rotation;
        clone.transform.parent=parent;clone.transform.localScale=original.transform.localScale;return clone;
    }
    public static void Destroy(GameObject point){point.destroyed=true;}
__METHODS__
}
public static class ProbeSceneReferenceChecks
{
    private static int n;
    private static void Check(bool ok,string label){if(!ok)throw new Exception(label);n++;}
    private static bool Near(float a,float b)=>Math.Abs(a-b)<1e-6;
    public static string Run()
    {
        var h=new SceneHarness();Vector3 offset;
        Check(h.TryGetTipOffset(out offset)&&Near(offset.z,.18f),"connection 30 mm plus extension 150 mm gives 180 mm");
        Check(Near(offset.x,0)&&Near(offset.y,0),"no unintended axis shift");
        h.UpdateCylinderGeometry();
        Check(Near(h.probeCylinder.localPosition.z,.105f),"cylinder midpoint is 105 mm");
        Check(Near(h.probeCylinder.localScale.y,.075f),"Unity cylinder half-height is 75 mm");
        Check(Near(h.probeCylinder.localScale.x,.005f)&&Near(h.probeCylinder.localScale.z,.005f),"diameter retained");
        Check(Near(h.probeCylinder.localPosition.z-h.probeCylinder.localScale.y,.03f)&&Near(h.probeCylinder.localPosition.z+h.probeCylinder.localScale.y,.18f),"both endpoints agree with connection and tip");
        Check(Near(Quaternion.LastFrom.y,1)&&Near(Quaternion.LastTo.z,1),"cylinder Y axis aligned with extension");
        h.tipOffsetMetres=new Vector3(.15f,0,0);h.UpdateCylinderGeometry();h.TryGetTipOffset(out offset);
        Check(Near(offset.x,.15f)&&Near(offset.z,.03f)&&Near(h.probeCylinder.localPosition.x,.075f),"alternate extension direction retains connection offset");
        Check(Near(Quaternion.LastTo.x,1),"alternate direction updates cylinder rotation");
        h.probeConnection.parent=null;Check(!h.TryGetTipOffset(out offset),"wrong connection parent rejected");h.probeConnection.parent=h.trackedToolRoot;
        h.trackedToolRoot.lossyScale=new Vector3(2,2,2);Check(!h.TryGetTipOffset(out offset),"scaled tracker root rejected");h.trackedToolRoot.lossyScale=Vector3.one;
        h.tipOffsetMetres=new Vector3(float.NaN,0,0);Check(!h.TryGetTipOffset(out offset),"nonfinite extension rejected");h.tipOffsetMetres=new Vector3(0,0,.15f);
        var cylinder=h.probeCylinder;h.probeCylinder=h.probeConnection;Check(!h.TryGetTipOffset(out offset),"same connection/cylinder object rejected");h.probeCylinder=cylinder;
        h.tracePointPrefab.transform.position=new Vector3(0,0,.03f);h.tracePointPrefab.transform.localScale=new Vector3(.0001f,.0001f,.0001f);
        h.buffer.Start(7,0);h.buffer.Observe(7,1,1,1.01,.15,1,2,3,.9f);h.AppendVisual(h.buffer.Points[0]);
        var first=h.pointVisuals[0];
        Check(SceneHarness.LastOriginal==h.tracePointPrefab,"uses supplied prefab");
        Check(Near(first.transform.position.x,1)&&Near(first.transform.position.y,2)&&Near(first.transform.position.z,3),"saved world position ignores prefab translation");
        Check(Near(first.transform.localScale.x,.0001f),"prefab scale preserved");
        Check(!first.collider.enabled&&h.tracePointPrefab.collider.enabled,"overlay collider disabled on instance only");
        h.buffer.Pause();h.buffer.Start(7,1);h.buffer.Observe(7,2,1.1,1.11,.15,1.01,2,3,.9f);h.AppendVisual(h.buffer.Points[1]);
        var second=h.pointVisuals[1];h.UndoStroke();
        Check(h.pointVisuals.Count==1&&h.buffer.Points.Count==1&&second.destroyed&&!first.destroyed,"undo removes only last stroke overlay");
        h.ClearVisuals();Check(h.pointVisuals.Count==0&&first.destroyed&&!h.tracePointPrefab.destroyed,"clear removes overlays without touching asset");
        h.buffer.Invalidate();h.AppendVisual(new TracePoint{frameId="9"});Check(!h.pointVisuals[0].active,"invalid session overlay remains hidden");
        return n+" production scene-reference checks passed";
    }
}
