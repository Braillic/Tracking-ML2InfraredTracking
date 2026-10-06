using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using ProbeTracing;
using Braillic.Tracking.Unity;
using Braillic.Tracking.Application;
using Braillic.Tracking.Runtime;

/// Captures actual tip observations; deliberately does not project/snap points to CT data.
/// Coordinates belong to one Unity/XR tracking epoch. No object-motion compensation yet.
public sealed class ProbeSurfaceTrace : MonoBehaviour
{
    [SerializeField] private TrackingApplicationHost trackingApplication;

    [Header("Scene probe references")]
    public Transform trackedToolRoot;
    [Tooltip("Connection point directly under TrackedTool. Its display scale is ignored.")]
    public Transform probeConnection;
    [Tooltip("Existing Unity cylinder directly under TrackedTool; local Y is its length axis.")]
    public Transform probeCylinder;
    [Header("Rough connection-to-tip estimate, in TrackedTool local metres")]
    [Tooltip("Extension FROM Marker_tool_Center; its local position is added to obtain the marker-to-tip offset.")]
    public Vector3 tipOffsetMetres = new Vector3(0,0,.15f);
    public bool tipDirectionChecked;
    [Header("Sampling")]
    [Min(.0001f)] public float spacingMetres = .002f;
    [Min(.01f)] public float maximumGapSeconds = .12f;
    [Min(.001f)] public float maximumStepMetres = .03f;
    [Min(.01f)] public float maximumCaptureAgeSeconds = .15f;
    [Range(20,20000)] public int maximumPoints = 3000;
    [Header("Visuals")]
    [Tooltip("Prefab placed at each saved world point; its scale and material are preserved.")]
    public GameObject tracePointPrefab;

    [Serializable] private sealed class TraceExport
    {
        public string schema="tracking-probe-trace-v4",utc,sessionId;
        public string sessionIdentity="Application-local collection epoch; not desktop wire session";
        public string coordinates="Unity world, metres, left-handed; xyz. No CT/PCD registration applied.";
        public string timestampClock="Tracking runtime monotonic clock; capture time, not UTC";
        public string frameId,frameEpoch,providerId,trackingSessionId;
        public long trackingRevision;
        public string calibration="rough estimated local offset; not pivot calibrated";
        public string reference="Single XR epoch; object motion is not compensated";
        public Vector3 tipOffsetMetres,connectionPointLocalMetres,connectionToTipMetres;
        public bool tipDirectionChecked,invalidated;
        public float spacingMetres,maximumGapSeconds,maximumStepMetres,maximumCaptureAgeSeconds;
        public int maximumPoints;
        public TracePoint[] points;
    }
    private ProbeTraceBuffer buffer;
    private TraceObservationSource traceSource;
    private TipObservation latestTip;
    private Vector3 sourceOffset;
    private float sourceAge;
    private string savedFrameId,savedFrameEpoch,savedProviderId,savedTrackingSession;
    private long savedTrackingRevision;
    private readonly List<GameObject> pointVisuals=new List<GameObject>();
    private Vector3 savedOffset,savedConnection,savedExtension;
    private Transform savedRoot,savedConnectionTransform,savedCylinder;
    private GameObject savedPointPrefab;
    private float savedSpacing,savedGap,savedStep,savedAge;
    private int savedMaximum;
    private bool savedTipChecked;
    private bool fresh;
    private string status="Check the cylinder: its far end is 150 mm from Marker_tool_Center.";
    public bool Arming=>buffer!=null&&buffer.Armed;
    public double CountdownSeconds=>Arming?Math.Max(0,buffer.StartsAt-Time.realtimeSinceStartupAsDouble):0;
    public string Status=>Arming?(CountdownSeconds>0
        ?$"Tracing starts in {Math.Ceiling(CountdownSeconds):0}... Place the tip on the surface."
        :"Waiting for a fresh frame captured after the countdown..."):status;
    public bool Recording=>buffer!=null&&buffer.Recording;
    public bool Fresh=>fresh;
    public bool Invalidated=>buffer!=null&&buffer.Invalidated;
    public int PointCount=>buffer!=null?buffer.Points.Count:0;
    public string LastExportDirectory { get; private set; }

    private void Start()
    {
        UpdateCylinderGeometry();
        ResetTrace();
    }
    private bool TryGetTipOffset(out Vector3 offset)
    {
        offset=default;
        if(trackedToolRoot==null||probeConnection==null||probeCylinder==null||tracePointPrefab==null
            ||probeConnection==probeCylinder||probeConnection.parent!=trackedToolRoot||probeCylinder.parent!=trackedToolRoot)return false;
        if(!Finite(trackedToolRoot.lossyScale)||(trackedToolRoot.lossyScale-Vector3.one).sqrMagnitude>1e-8f)return false;
        if(!Finite(probeConnection.localPosition)||!Finite(tipOffsetMetres)||tipOffsetMetres.sqrMagnitude<1e-8f)return false;
        // The connection marker's sphere scale does not change the physical extension.
        offset=probeConnection.localPosition+tipOffsetMetres;
        return Finite(offset);
    }
    private void UpdateCylinderGeometry()
    {
        if(!TryGetTipOffset(out var offset))return;
        // Unity's cylinder is centred, with unscaled endpoints Y=-1 and Y=+1.
        probeCylinder.localPosition=(probeConnection.localPosition+offset)*.5f;
        probeCylinder.localRotation=Quaternion.FromToRotation(Vector3.up,tipOffsetMetres.normalized);
        Vector3 scale=probeCylinder.localScale;scale.y=tipOffsetMetres.magnitude*.5f;probeCylinder.localScale=scale;
    }
    public Vector3 EffectiveTipOffsetMetres=>TryGetTipOffset(out var offset)?offset:Vector3.zero;
    private void Update()
    {
        if(buffer==null)return;
        TryReadObservation(out _);
        buffer.CheckSession(traceSource!=null?traceSource.CollectionEpoch:0);
        bool referencesReady=TryGetTipOffset(out var effectiveOffset);
        if(buffer.SessionId!=0&&(!referencesReady||ConfigurationChanged()))buffer.Invalidate();
        if(buffer.Invalidated)
        {
            foreach(var point in pointVisuals)point.SetActive(false);
            status="Trace frozen: tracking session or tip/settings changed. Export the old trace, then Clear for a new one.";
        }
        if(!referencesReady)
        {
            fresh=false;buffer.Pause();
            status="Assign TrackedTool, its Center/Cylinder children and Trace-Prefab. TrackedTool must have unit scale.";
            return;
        }
        UpdateCylinderGeometry();
        fresh=TryReadObservation(out var observation);
        Vector3 tipWorld=default;
        if(fresh)
        {
            tipWorld=observation.Position; // Tip was calculated from the raw unified tracking observation.
            double age=Time.realtimeSinceStartupAsDouble-observation.CaptureTime;
            fresh=Finite(tipWorld)&&age>=0&&age<=maximumCaptureAgeSeconds;
            if(fresh)
            {
                bool added=buffer.Observe(observation.SessionId,observation.FrameId,observation.CaptureTime,
                    Time.realtimeSinceStartupAsDouble,maximumCaptureAgeSeconds,tipWorld.x,tipWorld.y,tipWorld.z,observation.Confidence);
                if(added)
                {
                    AppendVisual(buffer.Points[buffer.Points.Count-1]);
                    status="Tracing. Keep contact with the surface; pause before lifting the tip.";
                    if(buffer.Full)status="Point limit reached. Export, undo the last stroke, or clear.";
                }
            }
        }
        if(!fresh)buffer.TrackingGap();
        // Cylinder follows the existing TrackedTool display; recording uses the raw observation.
    }
    // Application snapshot in the existing export convention (Unity world metres).
    private struct TraceObservation
    {
        public ulong SessionId, FrameId;
        public double CaptureTime;
        public Vector3 Position;
        public Quaternion Rotation;
        public float Confidence;
    }
    private bool TryReadObservation(out TraceObservation observation)
    {
        observation=default;
        if(trackingApplication==null||trackingApplication.Tracking==null||!TryGetTipOffset(out var offset))return false;
        if(traceSource==null||(buffer!=null&&buffer.SessionId==0&&(sourceOffset!=offset||sourceAge!=maximumCaptureAgeSeconds)))
        {
            sourceOffset=offset;sourceAge=maximumCaptureAgeSeconds;
            traceSource=new TraceObservationSource(trackingApplication.Tracking,trackingApplication.ObjectId,null,
                new Braillic.Tracking.Vector3d(offset.x,offset.y,-offset.z),"rough-scene-tip",
                new ObservationRequirements(maximumCaptureAgeSeconds,.01,.005));
        }
        // Missing OTS -> AR calibration must never be mistaken for native Unity world coordinates.
        if(!trackingApplication.TryGetDisplayFrame(out var frame))
        {
            // Let the source observe lifecycle/epoch changes even while no display frame exists.
            traceSource.TryRead(null,out _,out _);
            return false;
        }
        if(!traceSource.TryRead(frame,out latestTip,out _))return false;
        if(buffer==null||buffer.SessionId==0)
        {
            savedFrameId=latestTip.Tracking.Frame.Id;savedFrameEpoch=latestTip.Tracking.Frame.Epoch;
            savedProviderId=latestTip.Tracking.Source.Session.SourceId;
            savedTrackingSession=latestTip.Tracking.Source.Session.SessionId;savedTrackingRevision=latestTip.Tracking.Revision;
        }
        observation=new TraceObservation {
            SessionId=latestTip.CollectionEpoch,FrameId=latestTip.Tracking.Source.Sequence,
            CaptureTime=latestTip.Tracking.Source.CaptureSeconds,Confidence=(float)(latestTip.Tracking.Source.Quality??0),
            Position=TrackingCoordinates.ToUnity(latestTip.TipPosition)
        };
        return true;
    }

    private static bool Finite(Vector3 v)=>!(float.IsNaN(v.x)||float.IsNaN(v.y)||float.IsNaN(v.z)||float.IsInfinity(v.x)||float.IsInfinity(v.y)||float.IsInfinity(v.z));
    private bool ConfigurationChanged()=>trackedToolRoot!=savedRoot||probeConnection!=savedConnectionTransform
        ||probeCylinder!=savedCylinder||tracePointPrefab!=savedPointPrefab||!TryGetTipOffset(out var effectiveOffset)
        ||effectiveOffset!=savedOffset||probeConnection.localPosition!=savedConnection||tipOffsetMetres!=savedExtension
        ||spacingMetres!=savedSpacing||maximumGapSeconds!=savedGap||maximumStepMetres!=savedStep
        ||maximumCaptureAgeSeconds!=savedAge||maximumPoints!=savedMaximum||tipDirectionChecked!=savedTipChecked;
    private void SaveConfiguration()
    {
        TryGetTipOffset(out savedOffset);savedConnection=probeConnection!=null?probeConnection.localPosition:Vector3.zero;
        savedExtension=tipOffsetMetres;savedRoot=trackedToolRoot;savedConnectionTransform=probeConnection;
        savedCylinder=probeCylinder;savedPointPrefab=tracePointPrefab;
        savedSpacing=spacingMetres;savedGap=maximumGapSeconds;savedStep=maximumStepMetres;
        savedAge=maximumCaptureAgeSeconds;savedMaximum=maximumPoints;savedTipChecked=tipDirectionChecked;
    }
    public void ToggleTrace()
    {
        if(buffer==null)return;
        if(buffer.Recording||buffer.Armed){PauseTrace();return;}
        if(buffer.SessionId!=0&&ConfigurationChanged())buffer.Invalidate();
        if(buffer.Invalidated){status="Export then clear the frozen trace first.";return;}
        if(!TryGetTipOffset(out var effectiveOffset)){status="Assign the scene probe references and Trace-Prefab first.";return;}
        if(!tipDirectionChecked){status="Check the cylinder end against the physical tip, then press Confirm tip.";return;}
        if(!Finite(tipOffsetMetres)||tipOffsetMetres.sqrMagnitude<1e-8f||float.IsNaN(maximumCaptureAgeSeconds)||float.IsInfinity(maximumCaptureAgeSeconds)||maximumCaptureAgeSeconds<=0)
        {status="Invalid tip offset or freshness limit.";return;}
        if(!TryReadObservation(out var o))
        {status="No fresh probe pose. Keep the markers visible.";return;}
        if(buffer.SessionId==0)
        {
            // Inspector sampling settings may have changed before recording began.
            try{buffer=new ProbeTraceBuffer(spacingMetres,maximumGapSeconds,maximumStepMetres,maximumPoints);SaveConfiguration();}
            catch(Exception e){status=e.Message;return;}
        }
        if(buffer.Arm(o.SessionId,o.FrameId,Time.realtimeSinceStartupAsDouble,3.0))status="Three-second countdown started.";
        else status="Cannot start: point limit reached or trace belongs to another session.";
    }
    public void PauseTrace()
    {
        bool wasArmed=Arming;buffer?.Pause();
        status=wasArmed?"Countdown cancelled. No positioning points were recorded.":"Paused. Resume starts a new stroke after a three-second countdown.";
    }
    public void ConfirmTip()
    {
        if(buffer!=null&&buffer.SessionId!=0){status="Clear the trace before changing the tip configuration.";return;}
        if(!TryGetTipOffset(out var offset)){status="Assign the scene probe references and Trace-Prefab first.";return;}
        UpdateCylinderGeometry();tipDirectionChecked=true;status="Rough cylinder tip confirmed. Start tracing while touching the surface.";
    }
    public void CycleTipAxis()
    {
        if(buffer!=null&&buffer.SessionId!=0){status="Export/clear before changing the tip axis.";return;}
        Vector3[] axes={Vector3.forward,Vector3.back,Vector3.right,Vector3.left,Vector3.up,Vector3.down};
        float distance=tipOffsetMetres.magnitude;if(!Finite(tipOffsetMetres)||distance<.001f)distance=.15f;
        int best=0;float dot=-2;
        for(int i=0;i<axes.Length;i++){float d=Vector3.Dot(tipOffsetMetres.normalized,axes[i]);if(d>dot){dot=d;best=i;}}
        tipOffsetMetres=axes[(best+1)%axes.Length]*distance;tipDirectionChecked=false;
        UpdateCylinderGeometry();status="Tip axis changed. Compare the cylinder end with the physical probe, then confirm.";
    }
    public void UndoStroke()
    {
        if(buffer==null)return;buffer.UndoStroke();
        while(pointVisuals.Count>buffer.Points.Count)
        { int last=pointVisuals.Count-1;pointVisuals[last].SetActive(false);Destroy(pointVisuals[last]);pointVisuals.RemoveAt(last); }
        status="Last stroke removed; tracing paused.";
    }
    public void ResetTrace()
    {
        buffer?.Pause();traceSource=null;
        try{buffer=new ProbeTraceBuffer(spacingMetres,maximumGapSeconds,maximumStepMetres,maximumPoints);}
        catch(Exception e){status=e.Message;buffer=null;return;}
        SaveConfiguration();ClearVisuals();LastExportDirectory=null;status="Ready. Confirm tip direction, then start while touching the surface.";
    }
    private void AppendVisual(TracePoint p)
    {
        Vector3 position=new Vector3((float)p.xMetres,(float)p.yMetres,(float)p.zMetres);
        // Explicit world placement ignores the prefab asset's authored translation (currently Z=0.03).
        GameObject point=Instantiate(tracePointPrefab,position,tracePointPrefab.transform.rotation,transform);
        point.name="Trace point "+p.frameId;
        foreach(var collider in point.GetComponentsInChildren<Collider>(true))collider.enabled=false;
        point.SetActive(!buffer.Invalidated);pointVisuals.Add(point);
    }
    private void ClearVisuals()
    {
        foreach(var point in pointVisuals)if(point!=null){point.SetActive(false);Destroy(point);}
        pointVisuals.Clear();
    }
    public void ExportTrace()
    {
        buffer?.Pause();
        if(buffer==null||buffer.Points.Count==0){status="No trace points to export.";return;}
        TryReadObservation(out _);
        buffer.CheckSession(traceSource!=null?traceSource.CollectionEpoch:0);
        if(ConfigurationChanged())buffer.Invalidate();
        buffer.Pause();
        var data=new TraceExport{utc=DateTime.UtcNow.ToString("o"),sessionId=buffer.SessionId.ToString(),tipOffsetMetres=savedOffset,frameId=savedFrameId,frameEpoch=savedFrameEpoch,providerId=savedProviderId,trackingSessionId=savedTrackingSession,trackingRevision=savedTrackingRevision,
            connectionPointLocalMetres=savedConnection,connectionToTipMetres=savedExtension,
            tipDirectionChecked=savedTipChecked,invalidated=buffer.Invalidated,spacingMetres=savedSpacing,maximumGapSeconds=savedGap,
            maximumStepMetres=savedStep,maximumCaptureAgeSeconds=savedAge,maximumPoints=savedMaximum,points=buffer.Snapshot()};
        try
        {
            string directory=Path.Combine(Application.persistentDataPath,"ProbeTraces",DateTime.UtcNow.ToString("yyyyMMdd-HHmmss-fff")+"-"+Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory,"trace.json"),JsonUtility.ToJson(data,true));
            WritePointFiles(directory,data.points);
            LastExportDirectory=directory;status="Saved JSON, CSV and PCD: "+directory;Debug.Log("[ProbeTrace] Saved "+directory);
        }
        catch(Exception e){status="Export failed (files may be incomplete): "+e.Message;}
    }
    private static void WritePointFiles(string folder,TracePoint[] points)
    {
        var culture=CultureInfo.InvariantCulture;
        using(var csv=new StreamWriter(Path.Combine(folder,"trace.csv"),false,new UTF8Encoding(false)))
        using(var pcd=new StreamWriter(Path.Combine(folder,"trace.pcd"),false,new UTF8Encoding(false)))
        {
            csv.WriteLine("session_id,frame_id,capture_time_seconds,x_metres,y_metres,z_metres,confidence,stroke");
            pcd.WriteLine("# Probe trace: Unity world metres, left-handed. No CT registration. See trace.json.\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "+points.Length+"\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "+points.Length+"\nDATA ascii");
            foreach(var p in points)
            {
                string x=p.xMetres.ToString("R",culture),y=p.yMetres.ToString("R",culture),z=p.zMetres.ToString("R",culture);
                csv.WriteLine(string.Join(",",p.sessionId,p.frameId,p.captureTimeSeconds.ToString("R",culture),x,y,z,p.confidence.ToString("R",culture),p.stroke.ToString(culture)));
                pcd.WriteLine(x+" "+y+" "+z);
            }
        }
    }
    private void OnApplicationPause(bool paused){if(paused)PauseTrace();}
    private void OnDisable(){buffer?.Pause();foreach(var point in pointVisuals)if(point!=null)point.SetActive(false);}
    private void OnEnable(){if(buffer!=null&&!buffer.Invalidated)foreach(var point in pointVisuals)if(point!=null)point.SetActive(true);}
}
