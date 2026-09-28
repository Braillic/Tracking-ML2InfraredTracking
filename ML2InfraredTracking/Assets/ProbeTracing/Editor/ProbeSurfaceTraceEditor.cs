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
        EditorGUILayout.HelpBox("Tip offset is in the tracked marker's LOCAL axes, metres. 0.15 m = 150 mm. This records Unity-world points without aligning to CT/PCD.",MessageType.Info);
        if(!Application.isPlaying)return;
        EditorGUILayout.HelpBox(trace.Status,MessageType.None);
        EditorGUILayout.LabelField("Points",trace.PointCount.ToString());
        if(GUILayout.Button("Cycle tip axis"))trace.CycleTipAxis();
        if(GUILayout.Button("Confirm tip direction"))trace.ConfirmTip();
        if(GUILayout.Button(trace.Recording?"Pause tracing":"Start tracing"))trace.ToggleTrace();
        if(GUILayout.Button("Undo last stroke"))trace.UndoStroke();
        if(GUILayout.Button("Export JSON / CSV / PCD"))trace.ExportTrace();
        if(GUILayout.Button("Clear trace"))trace.ResetTrace();
    }
}
#endif
