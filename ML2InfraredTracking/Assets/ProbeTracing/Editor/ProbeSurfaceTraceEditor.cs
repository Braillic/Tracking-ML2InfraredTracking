#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(ProbeSurfaceTrace))]
public sealed class ProbeSurfaceTraceEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();
        var trace=(ProbeSurfaceTrace)target;
        EditorGUILayout.HelpBox("Tip Offset extends FROM Marker_tool_Center, in TrackedTool local metres. The Cylinder uses this extension. Recorded tip = center position + extension. Trace-Prefab controls point appearance.",MessageType.Info);
        if(!Application.isPlaying)return;
        EditorGUILayout.HelpBox(trace.Status,MessageType.None);
        EditorGUILayout.LabelField("Points",trace.PointCount.ToString());
        if(GUILayout.Button("Cycle tip axis"))trace.CycleTipAxis();
        if(GUILayout.Button("Confirm tip direction"))trace.ConfirmTip();
        if(GUILayout.Button(trace.Arming?"Cancel countdown":trace.Recording?"Pause tracing":"Start tracing (3 seconds)"))trace.ToggleTrace();
        if(GUILayout.Button("Undo last stroke"))trace.UndoStroke();
        if(GUILayout.Button("Export JSON / CSV / PCD"))trace.ExportTrace();
        if(GUILayout.Button("Clear trace"))trace.ResetTrace();
    }
}
#endif
