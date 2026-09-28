using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using ProbeTracing;

/// Captures actual tip observations; deliberately does not project/snap points to CT data.
/// Coordinates belong to one Unity/XR tracking epoch. No object-motion compensation yet.
public sealed class ProbeSurfaceTrace : MonoBehaviour
{
    [Header("Rough tip estimate: local marker coordinates, metres")]
    public Vector3 tipOffsetMetres = new Vector3(0,0,.15f);
    public bool tipDirectionChecked;
    [Header("Sampling")]
    [Min(.0001f)] public float spacingMetres = .002f;
    [Min(.01f)] public float maximumGapSeconds = .12f;
    [Min(.001f)] public float maximumStepMetres = .03f;
    [Min(.01f)] public float maximumCaptureAgeSeconds = .15f;
    [Range(20,20000)] public int maximumPoints = 3000;
    [Header("Visuals")]
    [Min(.0001f)] public float lineWidthMetres = .001f;
    public Color traceColor = Color.green;
    [Tooltip("Optional unlit material. Assign one to retain its shader in an Android build.")]
    public Material traceMaterial;

    [Serializable] private sealed class TraceExport
    {
        public string schema="ml2-probe-trace-v1",utc,sessionId;
        public string coordinates="Unity world, metres, left-handed; xyz. No CT/PCD registration applied.";
        public string timestampClock="ML2 Unity realtime mapped from sensor capture time; not UTC";
        public string calibration="rough estimated local offset; not pivot calibrated";
        public string reference="Single XR epoch; object motion is not compensated";
        public Vector3 tipOffsetMetres;
        public bool tipDirectionChecked,invalidated;
        public float spacingMetres,maximumGapSeconds,maximumStepMetres,maximumCaptureAgeSeconds;
        public int maximumPoints;
        public TracePoint[] points;
    }
    private ProbeTraceBuffer buffer;
    private readonly List<LineRenderer> lines=new List<LineRenderer>();
    private Transform tip;
    private LineRenderer probeAxis;
    private Material ownedMaterial;
    private Vector3 savedOffset;
    private float savedSpacing,savedGap,savedStep,savedAge;
    private int savedMaximum;
    private bool savedTipChecked;
    private bool fresh;
    private int renderedStroke=-1;
    private string status="Check the 150 mm tip marker against the probe; select its local axis, then confirm.";
    public string Status=>status;
    public bool Recording=>buffer!=null&&buffer.Recording;
    public bool Fresh=>fresh;
    public bool Invalidated=>buffer!=null&&buffer.Invalidated;
    public int PointCount=>buffer!=null?buffer.Points.Count:0;
    public string LastExportDirectory { get; private set; }

    private void Start()
    {
        if(traceMaterial==null)
        {
            Shader shader=Shader.Find("Universal Render Pipeline/Unlit")??Shader.Find("Sprites/Default");
            if(shader!=null){ownedMaterial=new Material(shader);ownedMaterial.color=traceColor;traceMaterial=ownedMaterial;}
        }
        var go=GameObject.CreatePrimitive(PrimitiveType.Sphere);go.name="Estimated probe tip";go.transform.SetParent(transform,false);
        go.transform.localScale=Vector3.one*.004f;Destroy(go.GetComponent<Collider>());
        if(traceMaterial!=null)go.GetComponent<Renderer>().sharedMaterial=traceMaterial;
        tip=go.transform;go.SetActive(false);
        probeAxis=MakeLine("Probe origin to estimated tip",.0006f);probeAxis.positionCount=2;probeAxis.gameObject.SetActive(false);
        ResetTrace();
    }
    private void Update()
    {
        if(buffer==null)return;
        var depth=DepthFrameTcpServer.ActiveServer;
        buffer.CheckSession(depth!=null?depth.SessionId:0);
        if(buffer.SessionId!=0&&ConfigurationChanged())buffer.Invalidate();
        if(buffer.Invalidated)
        {
            foreach(var line in lines)line.gameObject.SetActive(false);
            status="Trace frozen: tracking session or tip/settings changed. Export the old trace, then Clear for a new one.";
        }
        var source=PoseEstimateTcpServer.ActiveServer;
        fresh=source!=null&&source.TryGetAcceptedObservation(out var ignored);
        Vector3 tipWorld=default;
        if(fresh&&source.TryGetAcceptedObservation(out var observation))
        {
            tipWorld=observation.Position+observation.Rotation*tipOffsetMetres;
            double age=Time.realtimeSinceStartupAsDouble-observation.CaptureTime;
            fresh=Finite(tipWorld)&&age>=0&&age<=maximumCaptureAgeSeconds;
            if(fresh)
            {
                tip.position=tipWorld;probeAxis.SetPosition(0,observation.Position);probeAxis.SetPosition(1,tipWorld);
                bool added=buffer.Observe(observation.SessionId,observation.FrameId,observation.CaptureTime,
                    Time.realtimeSinceStartupAsDouble,maximumCaptureAgeSeconds,tipWorld.x,tipWorld.y,tipWorld.z,observation.Confidence);
                if(added)
                {
                    AppendVisual(buffer.Points[buffer.Points.Count-1]);
                    if(buffer.Full)status="Point limit reached. Export, undo the last stroke, or clear.";
                }
            }
        }
        if(!fresh)buffer.TrackingGap();
        if(tip!=null)tip.gameObject.SetActive(fresh);
        if(probeAxis!=null)probeAxis.gameObject.SetActive(fresh);
    }
    private static bool Finite(Vector3 v)=>!(float.IsNaN(v.x)||float.IsNaN(v.y)||float.IsNaN(v.z)||float.IsInfinity(v.x)||float.IsInfinity(v.y)||float.IsInfinity(v.z));
    private bool ConfigurationChanged()=>tipOffsetMetres!=savedOffset||spacingMetres!=savedSpacing||maximumGapSeconds!=savedGap
        ||maximumStepMetres!=savedStep||maximumCaptureAgeSeconds!=savedAge||maximumPoints!=savedMaximum||tipDirectionChecked!=savedTipChecked;
    private void SaveConfiguration()
    {
        savedOffset=tipOffsetMetres;savedSpacing=spacingMetres;savedGap=maximumGapSeconds;savedStep=maximumStepMetres;
        savedAge=maximumCaptureAgeSeconds;savedMaximum=maximumPoints;savedTipChecked=tipDirectionChecked;
    }
    public void ToggleTrace()
    {
        if(buffer==null)return;
        if(buffer.Recording){buffer.Pause();status="Paused. Resume starts a separate stroke.";return;}
        if(buffer.SessionId!=0&&ConfigurationChanged())buffer.Invalidate();
        if(buffer.Invalidated){status="Export then clear the frozen trace first.";return;}
        if(!tipDirectionChecked){status="Check the visible tip against the physical tip, then press Confirm tip.";return;}
        if(!Finite(tipOffsetMetres)||tipOffsetMetres.sqrMagnitude<1e-8f||float.IsNaN(maximumCaptureAgeSeconds)||float.IsInfinity(maximumCaptureAgeSeconds)||maximumCaptureAgeSeconds<=0)
        {status="Invalid tip offset or freshness limit.";return;}
        var source=PoseEstimateTcpServer.ActiveServer;
        if(source==null||!source.TryGetAcceptedObservation(out var o)||Time.realtimeSinceStartupAsDouble-o.CaptureTime>maximumCaptureAgeSeconds)
        {status="No fresh probe pose. Keep the markers visible.";return;}
        if(buffer.SessionId==0)
        {
            // Inspector sampling settings may have changed before recording began.
            try{buffer=new ProbeTraceBuffer(spacingMetres,maximumGapSeconds,maximumStepMetres,maximumPoints);SaveConfiguration();}
            catch(Exception e){status=e.Message;return;}
        }
        if(buffer.Start(o.SessionId,o.FrameId))status="Tracing. Keep the physical tip touching the surface; pause when lifting it.";
        else status="Cannot start: point limit reached or trace belongs to another session.";
    }
    public void ConfirmTip()
    {
        if(buffer!=null&&buffer.SessionId!=0){status="Clear the trace before changing the tip configuration.";return;}
        tipDirectionChecked=true;status="Rough tip direction confirmed. Start tracing while touching the surface.";
    }
    public void CycleTipAxis()
    {
        if(buffer!=null&&buffer.SessionId!=0){status="Export/clear before changing the tip axis.";return;}
        Vector3[] axes={Vector3.forward,Vector3.back,Vector3.right,Vector3.left,Vector3.up,Vector3.down};
        float distance=tipOffsetMetres.magnitude;if(!Finite(tipOffsetMetres)||distance<.001f)distance=.15f;
        int best=0;float dot=-2;
        for(int i=0;i<axes.Length;i++){float d=Vector3.Dot(tipOffsetMetres.normalized,axes[i]);if(d>dot){dot=d;best=i;}}
        tipOffsetMetres=axes[(best+1)%axes.Length]*distance;tipDirectionChecked=false;
        status="Tip axis changed. Compare the visible tip with the physical probe, then confirm.";
    }
    public void UndoStroke()
    {
        if(buffer==null)return;buffer.UndoStroke();RebuildVisuals();status="Last stroke removed; tracing paused.";
    }
    public void ResetTrace()
    {
        try{buffer=new ProbeTraceBuffer(spacingMetres,maximumGapSeconds,maximumStepMetres,maximumPoints);}
        catch(Exception e){status=e.Message;buffer=null;return;}
        SaveConfiguration();ClearVisuals();LastExportDirectory=null;status="Ready. Confirm tip direction, then start while touching the surface.";
    }
    private LineRenderer MakeLine(string label,float width)
    {
        var go=new GameObject(label);go.transform.SetParent(transform,false);
        var line=go.AddComponent<LineRenderer>();line.useWorldSpace=true;line.widthMultiplier=width;
        line.startColor=line.endColor=traceColor;line.positionCount=0;line.numCapVertices=3;
        if(traceMaterial!=null)line.sharedMaterial=traceMaterial;return line;
    }
    private void AppendVisual(TracePoint p)
    {
        if(p.stroke!=renderedStroke){renderedStroke=p.stroke;lines.Add(MakeLine("Surface stroke "+p.stroke,lineWidthMetres));}
        var line=lines[lines.Count-1];line.positionCount++;line.SetPosition(line.positionCount-1,new Vector3((float)p.xMetres,(float)p.yMetres,(float)p.zMetres));
        if(buffer.Invalidated)line.gameObject.SetActive(false);
    }
    private void ClearVisuals(){foreach(var line in lines)if(line!=null)Destroy(line.gameObject);lines.Clear();renderedStroke=-1;}
    private void RebuildVisuals(){ClearVisuals();foreach(var p in buffer.Points)AppendVisual(p);}
    public void ExportTrace()
    {
        if(buffer==null||buffer.Points.Count==0){status="No trace points to export.";return;}
        buffer.CheckSession(DepthFrameTcpServer.ActiveServer!=null?DepthFrameTcpServer.ActiveServer.SessionId:0);
        if(ConfigurationChanged())buffer.Invalidate();
        buffer.Pause();
        var data=new TraceExport{utc=DateTime.UtcNow.ToString("o"),sessionId=buffer.SessionId.ToString(),tipOffsetMetres=savedOffset,
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
    private void OnDisable(){buffer?.Pause();if(tip!=null)tip.gameObject.SetActive(false);if(probeAxis!=null)probeAxis.gameObject.SetActive(false);foreach(var line in lines)if(line!=null)line.gameObject.SetActive(false);}
    private void OnEnable(){if(buffer!=null&&!buffer.Invalidated)foreach(var line in lines)if(line!=null)line.gameObject.SetActive(true);}
    private void OnDestroy(){if(ownedMaterial!=null)Destroy(ownedMaterial);}
}
