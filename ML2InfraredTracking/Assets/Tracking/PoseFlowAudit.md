# Stationary-probe jitter: API flow audit

Reported symptom: the model stays visible but trembles/jumps more when the physical probe is stationary.

## Findings

- The pre-API scene and current presenter both set position/rotation follow to 1. No enabled
  smoothing setting was lost during migration. The raw desktop pose therefore remains visible.
- The managed receiver bypasses its legacy transform writes and visibility timeout. The new
  presenter is the owner of model application; repeated API phase queries deduplicate by session,
  sequence and revision.
- Canonical coordinate conversion is an involution. Tests replay 120 stationary non-identity
  rotations, alternate equivalent quaternion signs, and repeat before-render queries without
  changing the returned geometry or sequence.
- The capture lifecycle restructuring did move polling from a coroutine after normal Update
  into the generic host's early (-1000) Update. This also changed when the live sensor pose and
  XR-origin transform are sampled relative to other application updates. This is a plausible
  regression, not a hardware-confirmed diagnosis of the reported jitter.

## Change

The resource controller can now advance with `Pump(readFrame:false)`. The ML2 provider uses that
in Update for permissions, startup/shutdown and status. It allows one capture poll in LateUpdate,
after normal Update components, before consuming pending poses. Repeated LateUpdate calls cannot
poll a second time without a new Update. Before-render never performs sensor acquisition.

Capture-time pose lookup, timestamp mapping, desktop detection/PnP, body geometry, freshness
limits, presentation follow values and raw trace measurements remain unchanged. No new smoothing
or arbitrary stationary deadband has been applied.

This restores the post-Update relationship; it is not an assertion that LateUpdate has identical
timing to the former coroutine. Headset validation is required to determine whether jitter improves.

## Headset diagnostic

Both demo presenters enable `logPoseDiagnostics`; disable it in the Presenter Inspector after
investigation. `[TrackingPose]` prints once every two seconds:

| Field | Interpretation |
|---|---|
| `samples` | New observations presented in this interval |
| `duplicate_calls_ignored` | Repeated queries that did NOT reapply or smooth the same pose; normally nonzero |
| `snaps` | First presentation/reacquisition events, not ordinary per-frame jitter |
| `external_transform_changes` | Transform movement observed outside presenter writes, including parent movement; thresholds 0.01 mm or 0.1 degree |
| `max_raw_step_mm/deg` | Largest consecutive resolved-input pose change in a continuous presentation segment |
| `max_presented_step_mm/deg` | Largest consecutive applied pose change in that segment |

Step metrics are motion between accepted samples, not absolute error or registration accuracy.
Segment starts after hiding/session changes are counted as snaps and excluded from step maxima.
These maxima do not identify which desktop detection caused a jump; they locate the boundary.

With follow values 1 and a stationary probe:

- Similar raw/presented steps suggest the presenter is reflecting incoming measurement variation.
- External transform changes identify a possible second writer or moving parent to investigate.
- Small numeric steps with visible scene jitter suggest the viewing/reference transform needs
  further examination rather than attributing it to the probe pose alone.

Compare the probe resting on the table with the headset relatively still, then with gentle head
movement. Keep the same desktop command and model geometry for comparison. This is a diagnostic
exercise; synthetic tests and compilation do not replace observing the physical headset.
