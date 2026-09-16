# Sensor-pose regression checks

From the repository root, using PowerShell 7:

```powershell
pwsh -NoProfile -File tests/sensor-pose/Run-SensorPoseTests.ps1
```

The checks run the production SensorPoseHistory source and the production capture
lookup / SDK validity method bodies with controlled clocks and XR responses.
Unity math is replaced by a small System.Numerics shim. They do not exercise
the Unity player loop, native XR runtime, or device presentation.

Coverage: exact samples, interpolation, retention, gaps, out-of-range queries,
invalid numbers, duplicate timestamps, resets, aged history, successful fallback,
failed fallback returning a cached pose, and incomplete XR validity flags.

Also compile the project in Unity before deploying. On ML2, test head-only motion,
probe-only motion, simultaneous motion, tracking loss, pause/resume, and recentering.
The producer logs historyUsed, directUsed, dropped, and lastLookup every two seconds.
Brief missing-pose periods now drop frames instead of sending guessed sensor poses.
