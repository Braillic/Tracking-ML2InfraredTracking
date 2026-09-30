# Probe surface tracing prototype

This is the first stage of the CT point-cloud registration workflow: collect an independent surface trace using the tracked probe. The CT source is a live PCD point cloud. This version does not load, align to, or snap measurements onto that cloud. It does not need a CT mesh.

## Open and run

Open `ML2InfraredTracking/Assets/Scenes/ProbeSurfaceTracing_demo.unity` in the **Original IR Tracking Repo** Unity project. This is a copy of the current IR demo with a `Probe Surface Tracing` object added. 

   ```powershell
   python depth_stream_receiver.py --host 192.168.0.161 --port 50777 --pose-port 50778 --send-pose --headless
   ```



## Data flow

```text
IR frame -> PC pose -> ML2 validates session/frame/pose/freshness
    -> immutable value snapshot of raw accepted marker pose
    -> local_tip_offset = connection.localPosition + connection_to_tip_extension
    -> tip_world = marker_position + marker_rotation * local_tip_offset
    -> fresh-observation sampling -> separate strokes -> export
```

The tracing component polls the latest accepted observation in Unity Update. It records at most once per observation ID, not once per display frame. If multiple poses are accepted between Updates, only the newest is sampled. Capture timestamps are preserved. Visual smoothing on the tracked marker transform does not modify recorded positions. 

Default sampling:

| Setting | Default | Meaning |
|---|---:|---|
| Connection to tip | 150 mm, local +Z | Added to Marker_tool_Center local position; current total offset 180 mm |
| Start/resume delay | 3 seconds | Uses realtime, independent of Unity time scale; excludes pre-deadline captures |
| Spacing | 2 mm | Minimum distance from the last recorded point within a stroke |
| Gap | 120 ms | Longer observation gaps begin a separate stroke |
| Step | 30 mm | Larger steps begin a separate stroke rather than drawing a connecting line |
| Capture age | 150 ms | Maximum age for a point; source pose validation can impose an additional limit |
| Capacity | 3,000 points | Stops recording when full; export, undo, or clear |


## Reference frame and interrupted sessions

The trace is in **Unity world metres within one XR tracking epoch**. Headset movement is accounted for by the existing tracker. Object motion is **not** compensated: 

Loss of fresh tracking stops point additions and breaks the stroke. The last displayed/held marker pose is never repeatedly recorded. Duplicate/older IDs and invalid/future/stale capture times are rejected.

A changed XR session/origin, tip offset, or sampling configuration freezes the current trace and hides its old world-space point overlays. 

## Files and PCD interoperability

## Source and checks

- `Assets/ProbeTracing/ProbeTraceBuffer.cs`: pure managed sampling/state logic.
- `Assets/ProbeTracing/ProbeSurfaceTrace.cs`: tip calculation, rendering, controls and exports.
- `Assets/ProbeTracing/ProbeTracePanel.cs`: XR world-space controls.
- `Assets/ProbeTracing/Editor/ProbeSurfaceTraceEditor.cs`: play-mode Inspector controls.
- `PoseEstimateTcpServer.TryGetAcceptedObservation`: fresh raw-pose snapshot API.

Run in separate PowerShell processes:

```powershell
pwsh -NoProfile -File tests/probe-tracing/Run-ProbeTraceTests.ps1
pwsh -NoProfile -File tests/probe-tracing/Run-ProbeSceneReferenceTests.ps1
pwsh -NoProfile -File tests/pose-freshness/Run-PoseFreshnessTests.ps1
```