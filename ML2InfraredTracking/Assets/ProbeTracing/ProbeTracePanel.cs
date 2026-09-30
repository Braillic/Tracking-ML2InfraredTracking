using TMPro;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine.XR.Interaction.Toolkit.UI;

/// Small world-space prototype panel, using the scene's existing XR ray input.
[RequireComponent(typeof(ProbeSurfaceTrace))]
public sealed class ProbeTracePanel : MonoBehaviour
{
    public bool showPanel=true;
    private ProbeSurfaceTrace trace;
    private Canvas canvas;
    private TextMeshProUGUI status,startLabel,axisLabel;
    private bool placed;
    private float nextRefresh;
    private void Start()
    {
        trace=GetComponent<ProbeSurfaceTrace>();
        if(EventSystem.current==null)
        {
            var events=new GameObject("Probe Trace UI EventSystem",typeof(EventSystem),typeof(XRUIInputModule));
            events.transform.SetParent(transform,false);
        }
        var panel=new GameObject("Probe Trace Panel",typeof(RectTransform),typeof(Canvas),typeof(CanvasScaler),typeof(TrackedDeviceGraphicRaycaster));
        panel.transform.SetParent(transform,false);canvas=panel.GetComponent<Canvas>();canvas.renderMode=RenderMode.WorldSpace;
        var rect=(RectTransform)panel.transform;rect.sizeDelta=new Vector2(1000,680);rect.localScale=Vector3.one*.0007f;
        var background=panel.AddComponent<Image>();background.color=new Color(.025f,.045f,.065f,.96f);
        Label("Probe surface tracing",new Vector2(25,-15),new Vector2(950,55),38);
        status=Label("Waiting for tracking...",new Vector2(25,-80),new Vector2(950,220),25);
        startLabel=Button("Start / Pause",0,trace.ToggleTrace);
        Button("Undo stroke",1,trace.UndoStroke);
        Button("Export JSON / CSV / PCD",2,trace.ExportTrace);
        Button("Clear trace",3,trace.ResetTrace);
        axisLabel=Button("Change tip axis",4,trace.CycleTipAxis);
        Button("Confirm tip direction",5,trace.ConfirmTip);
        Button("Recenter panel",6,PlacePanel);
        Button("Pause / Cancel",7,trace.PauseTrace);
        Label("World-space observations only. Keep contact while tracing; pause when lifting the tip.",new Vector2(25,-620),new Vector2(950,50),21);
    }
    private TextMeshProUGUI Label(string text,Vector2 position,Vector2 size,float fontSize,Transform parent=null)
    {
        var go=new GameObject(text,typeof(RectTransform));go.transform.SetParent(parent??canvas.transform,false);
        var r=(RectTransform)go.transform;r.anchorMin=r.anchorMax=r.pivot=new Vector2(0,1);r.anchoredPosition=position;r.sizeDelta=size;
        var label=go.AddComponent<TextMeshProUGUI>();label.text=text;label.fontSize=fontSize;label.color=Color.white;label.raycastTarget=false;
        label.textWrappingMode=TextWrappingModes.Normal;label.overflowMode=TextOverflowModes.Ellipsis;return label;
    }
    private TextMeshProUGUI Button(string title,int slot,UnityAction action)
    {
        var go=new GameObject(title,typeof(RectTransform),typeof(Image),typeof(Button));go.transform.SetParent(canvas.transform,false);
        var rect=(RectTransform)go.transform;rect.anchorMin=rect.anchorMax=rect.pivot=new Vector2(0,1);
        rect.anchoredPosition=new Vector2(25+(slot%2)*480,-315-(slot/2)*73);rect.sizeDelta=new Vector2(465,62);
        go.GetComponent<Image>().color=new Color(.09f,.21f,.29f);var button=go.GetComponent<Button>();button.onClick.AddListener(action);
        var text=Label(title,new Vector2(12,-7),new Vector2(441,48),26,go.transform);text.alignment=TextAlignmentOptions.Center;return text;
    }
    public void PlacePanel()
    {
        if(canvas==null||Camera.main==null)return;
        var camera=Camera.main;canvas.worldCamera=camera;
        canvas.transform.position=camera.transform.position+camera.transform.forward*1.1f+camera.transform.right*.45f;
        canvas.transform.rotation=Quaternion.LookRotation(canvas.transform.position-camera.transform.position,Vector3.up);placed=true;
    }
    private void Update()
    {
        if(canvas==null)return;canvas.gameObject.SetActive(showPanel);if(!showPanel)return;
        if(!placed)PlacePanel();
        if(Time.unscaledTime<nextRefresh)return;nextRefresh=Time.unscaledTime+.2f;
        status.text=$"{trace.Status}\nPoints: {trace.PointCount} | Tracking: {(trace.Fresh?"fresh":"unavailable")}\nCenter to tip: {trace.tipOffsetMetres*1000f} mm; total: {trace.EffectiveTipOffsetMetres*1000f} mm | {(trace.tipDirectionChecked?"direction checked":"check direction")}";
        startLabel.text=trace.Arming?"Cancel countdown":trace.Recording?"Pause tracing":"Start tracing (3 seconds)";
        axisLabel.text="Change tip axis (clear first)";
    }
    private void OnDisable(){if(canvas!=null)canvas.gameObject.SetActive(false);}
}
