# Pose freshness regression checks

From the repository root, run PowerShell 7:

```powershell
pwsh -NoProfile -File tests/pose-freshness/Run-PoseFreshnessTests.ps1
```

The runner extracts the actual `SubmitPose`, `TryApplyPendingPose`,
`IsFreshPose`, `IsFinitePose`, and `HideTrackedToolIfTimedOut` method bodies.
Test-only stand-ins provide the clock, frame submission times, and transform.
Checks cover first frame ID zero, duplicates/out-of-order packets, mailbox
ordering, source-age rejection, missing timestamps, malformed values,
quaternion normalization, timeout, and stale-pose resurrection.

These checks do not exercise Unity rendering or device sockets. Compile the
Unity application and test the rebuilt APK on ML2 as well.
