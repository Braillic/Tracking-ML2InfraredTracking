# Probe surface tracing prototype

This is the first stage of the CT point-cloud registration workflow: collect an independent surface trace using the tracked probe. The CT source is a live PCD point cloud. This version does not load, align to, or snap measurements onto that cloud. It does not need a CT mesh.

## Open and run

1. Open `ML2InfraredTracking/Assets/Scenes/ProbeSurfaceTracing_demo.unity` in the **Original IR Tracking Repo** Unity project. This is a copy of the current IR demo with a `Probe Surface Tracing` object added. The original IR scene is untouched by this feature.
2. Select `Probe Surface Tracing`. `Tip Offset Metres` is expressed in the marker arrangement's local coordinates. The default `(0, 0, 0.15)` means **150 mm along local +Z**, an unconfirmed starting direction. Change the XYZ values in the Inspector. The physical distance alone does not establish the direction.
3. Build/run this scene on ML2. In Build Profiles, include/select this tracing scene as the startup scene. Do not load both IR demos together: each contains tracking servers and an XR rig. This change does not alter the existing build profile.
4. Run the existing desktop detector, for example:

   ```powershell
   python depth_stream_receiver.py --host 192.168.0.161 --port 50777 --pose-port 50778 --send-pose --headless
   ```

5. A small green tip marker and a thin line from the marker origin show the estimated tip. Use **Change tip axis** to cycle +Z, -Z, +X, -X, +Y, -Y while preserving the distance. Use Inspector XYZ fields for arbitrary offsets. Compare the marker to the physical tip, then choose **Confirm tip direction**. This confirmation is only a direction check, not calibration.
6. Touch the surface, choose **Start tracing**, and move the tip while maintaining contact. **Pause** before lifting it. The IR tracker does not detect physical contact automatically.
7. **Undo stroke** removes the last stroke and pauses. **Clear trace** discards the in-memory trace. **Export** pauses and saves JSON, CSV and ASCII PCD together in a uniquely named directory.

The world-space panel uses the existing XR hand/controller UI rays. It appears beside the view and stays in world space; **Recenter panel** moves it back beside the current view. Play-mode Inspector buttons expose the same operations for desktop development. Headset interaction/rendering needs verification on the device; compilation is not an interaction test.

## Data flow

```text
IR frame -> PC pose -> ML2 validates session/frame/pose/freshness
    -> immutable value snapshot of raw accepted marker pose
    -> tip_world = marker_position + marker_rotation * local_tip_offset
    -> fresh-observation sampling -> separate strokes -> export
```

The tracing component polls the latest accepted observation in Unity Update. It records at most once per observation ID, not once per display frame. If multiple poses are accepted between Updates, only the newest is sampled. Capture timestamps are preserved. Visual smoothing on the tracked marker transform does not modify recorded positions. No tracing callbacks, string formatting, mesh updates, or file I/O were added to the before-render pose application path; that path only copies a small observation struct.

Default sampling:

| Setting | Default | Meaning |
|---|---:|---|
| Tip offset | 150 mm, local +Z | Rough estimate; direction must be checked |
| Spacing | 2 mm | Minimum distance from the last recorded point within a stroke |
| Gap | 120 ms | Longer observation gaps begin a separate stroke |
| Step | 30 mm | Larger steps begin a separate stroke rather than drawing a connecting line |
| Capture age | 150 ms | Maximum age for a point; source pose validation can impose an additional limit |
| Capacity | 3,000 points | Stops recording when full; export, undo, or clear |

Spacing is a sampling setting, not an accuracy guarantee. A stroke with only one point is exported, but a line becomes visible once it contains two points. The live tip marker is visible independently.

## Reference frame and interrupted sessions

The trace is in **Unity world metres within one XR tracking epoch**. Headset movement is accounted for by the existing tracker. Object motion is **not** compensated: keep the object stationary when collecting data intended for later registration.

Loss of fresh tracking stops point additions and breaks the stroke. The last displayed/held marker pose is never repeatedly recorded. Duplicate/older IDs and invalid/future/stale capture times are rejected.

A changed XR session/origin, tip offset, or sampling configuration freezes the current trace and hides its old world-space lines. The data remain available for export. Export and **Clear trace** before recording in the new setup. There is no automatic cross-session relocalization or reuse of old world coordinates.

## Files and PCD interoperability

The export directory is logged as `[ProbeTrace] Saved ...` and is under:

```text
Application.persistentDataPath/ProbeTraces/<unique UTC-labelled directory>/
    trace.json
    trace.csv
    trace.pcd
```

Use the actual logged path when retrieving files. The Android package is currently `com.braillic.depthstreaming`.

- JSON: offset, rough-calibration label, sampling settings, reference-frame description, session, invalidation flag, and complete point records.
- CSV: session ID, frame ID, capture time in seconds, XYZ in metres, confidence, stroke number.
- PCD: ASCII XYZ points, in the **same Unity-world coordinates and metres**, suitable for inspecting alongside other clouds. The file header identifies this convention. PCD contains no stroke IDs/timestamps; retain the JSON/CSV sidecars.

Session/frame IDs are strings in JSON to preserve full uint64 precision. Capture time belongs to ML2's monotonic realtime clock mapped from sensor time, not UTC. The directory UTC label is export time.

Do not directly overlay a CT-generated PCD just because both files are PCD. Its units, axes, handedness, origin and CT-to-world transform still need to be established. The next stage will use the independently measured trace and the CT cloud for registration and separate verification observations. The 150 mm estimate should eventually be replaced by a measured or pivot-calibrated tip offset.

## Source and checks

- `Assets/ProbeTracing/ProbeTraceBuffer.cs`: pure managed sampling/state logic.
- `Assets/ProbeTracing/ProbeSurfaceTrace.cs`: tip calculation, rendering, controls and exports.
- `Assets/ProbeTracing/ProbeTracePanel.cs`: XR world-space controls.
- `Assets/ProbeTracing/Editor/ProbeSurfaceTraceEditor.cs`: play-mode Inspector controls.
- `PoseEstimateTcpServer.TryGetAcceptedObservation`: fresh raw-pose snapshot API.

Run in separate PowerShell processes:

```powershell
pwsh -NoProfile -File tests/probe-tracing/Run-ProbeTraceTests.ps1
pwsh -NoProfile -File tests/pose-freshness/Run-PoseFreshnessTests.ps1
```

Validation covers sampling, gaps, stale/duplicate/future data, epoch changes, capacity/undo, snapshot independence, export units/culture/precision, and the production pose snapshot/acceptance methods. Editor and Android-conditional C# compilation uses installed Unity 6000.3.2f1 references. A full APK build, physical tip check and headset interaction test remain device-side steps.
