# ML2 capture and pose application update

Rebuild and deploy the Unity APK to enable these changes. The depth v5 and pose
v4 wire layouts are unchanged; the current Python receiver remains compatible.

## Desktop path retained

For frame N, `depth_transport.py` reads pixels plus frame/session identity,
capture time, calibration and the capture-time sensor pose. The latest pending
complete frame is consumed by `TrackingPipeline.process()`:

1. Validate identity/order/calibration and prepare UINT8 intensity.
2. Detect weighted subpixel marker centers and filter implausible candidates.
3. Associate markers, run PnP/refinement and validate temporal/geometric quality.
4. Compose the sensor-relative tool pose with the capture-time world sensor pose.
5. Return confidence, state, source identity, world pose and processing timings.
6. The TCP adapter rechecks age and returns the 88-byte session-tagged pose packet.
7. Interactive preview and recording run afterward; headless skips these operations.

The desktop core is the Python reference for a future on-device native backend.
Capture-time sensor poses, session/freshness checks, subpixel centers, motion
compensation, acquisition confirmation and bounded recovery remain in place.

## New application opportunities

`LateUpdate` continues to apply the newest pending valid pose. With **Apply Before
Render** enabled, the `Application.onBeforeRender` callback also checks for a
newer pose that arrived after LateUpdate. This option is enabled by default and
explicitly in the demo scene. Turning it off provides the LateUpdate-only A/B path.

The callback does not create visuals, format log messages or sort percentiles.
Visuals are prepared on the ordinary main-thread Update/LateUpdate path and kept
hidden until a valid pose. Both position and rotation are set before enabling a
hidden model. Custom model `OnEnable` callbacks should remain inexpensive because
reacquisition can activate the model from the render callback.

If the receiver mailbox or timing snapshot is busy, the render callback returns
without waiting and leaves the pending pose for the next opportunity. A pose is
consumed once; repeated callbacks do not repeatedly apply or smooth the same
observation. A newer observation can replace an earlier application in the same
rendered frame. Session is rechecked at consumption to cover origin-reset races.
Source-age expiration, confidence/finite-pose checks and stale-ID protection apply
in both paths. A session reset clears the previous session's frame-ID gate.

Actual application timestamps are stored in a bounded 32-entry value-type ring.
Update later collects summaries and prints sampled logs using those stored times.
`diagnostic_overwritten` indicates if diagnostics fell behind; it does not mean a
tracking pose was dropped. Logs may print in the frame after the application.
Do not use the logcat line's wall-clock prefix as the application timestamp.

Unity describes this callback as immediately before rendering and advises minimal
work there: https://docs.unity3d.com/6000.0/Documentation/ScriptReference/Application-onBeforeRender.html

## Capture/submission changes

- Submit a tracking frame before optional preview texture upload. **Enable Preview**
  on ML2DepthRawStream remains available for A/B testing; the feature is not
  automatically disabled. The preview still consumes main-thread time afterward.
- Keep the required ownership-safe native-to-managed and sender-buffer copies.
  No SDK buffer is retained past its lifetime, and sensor polling stays on the
  existing main-thread coroutine. SDK settings/exposure/timeouts are unchanged.
- Preserve the average submission schedule through small polling jitter instead
  of restarting a full period from each late submission. Following a large stall,
  discard missed slots; do not queue catch-up frames. Individual intervals can be
  shorter than 1/rate, but the configured rate remains the pacing target. This
  does not synchronize capture, network, PC processing and rendering.
- Avoid repeated unchanged TMP text assignments and per-summary percentile-array
  allocation. Reuse the eight-byte session receive buffer.

## Logs and exact timing boundaries

### `[ML2LAT]`: one sampled, matching frame

New fields include `session`, `phase=LateUpdate|BeforeRender`, `capture_to_apply`,
`capture_to_ready`, `ready_to_submit`, and `apply_work`.

`apply_wait` now measures pose receipt/publication time through the start of
validation/application, after mailbox/timing lookup. `apply_work` measures the
validation and transform/activation work for an accepted pose. The old apply_wait
combined these and some diagnostics. Compare **wait + work** to the old metric.

```text
capture_to_apply = capture_to_ready + ready_to_submit + submit_to_apply
submit_to_apply = ml_prepare+queue+tcp_write + leg1_estimate
                + detect + pnp + pack + leg2_estimate
                + apply_wait + apply_work + unaccounted
```

This identity holds per frame before decimal rounding. Unknown mapped capture
time prints `NaN`; no guessed capture latency is substituted. The transport legs
retain their clock-sync uncertainty and include some endpoint software work.

### `[ML2Timing]`: rolling accepted-pose distributions

Additional p50/p95/p99 series:

- `capture_to_ready`: mapped SDK capture time to GetSensorData return.
- `raw_copy`: SDK image to reusable managed Float32 array, before submission.
- `apply_wait` and `apply_work`: the split described above.
- `capture_to_before_render`: capture age at the first before-render callback for
  the latest applied pose. If multiple poses are applied before that callback,
  only the latest contributes to this series.
- `update_interval`: actual Update-to-Update wall time, exposing frame-loop stalls.

`applied_late` and `applied_before_render` count applications by phase since server
start; one rendered frame can contain both. Each timing series prints its own
sample count. Do not add percentiles across stages as though they describe the
same observation. Turning timing logs off does not disable pose application.

### `[ML2Capture]`: delivered frames with a resolved sensor pose

These samples include transport-rate-dropped frames, rather than only frames for
which PC tracking succeeded. Samples for missing capture-time sensor poses remain
in the existing sensor-pose diagnostic/drop counters.

| Field | Boundary |
| --- | --- |
| `capture_to_ready` | Mapped sensor capture to SDK return; sensor/SDK delay plus polling schedule |
| `sdk_poll` | GetSensorData entry to return; not an extra term to add to capture_to_ready |
| `pose_lookup` | SDK return to ProcessFrame entry, including capture-pose/world lookup |
| `raw_copy` | Native plane to managed Float32 array, only when performed |
| `preview_upload` | Preview texture preparation/upload/material assignment, only when performed |

Each series is a rolling window of at most 256 eligible samples, printed every two
seconds by Update. Preview sampling is throttled independently and therefore has
fewer samples. `preview_enabled` reports the setting; a missing renderer still
prevents uploads. `Log Capture Timing` disables these extra capture summaries.

## Device validation

Keep PC headless, sensor settings and network setup fixed. Compare two runs with
Apply Before Render off/on; then compare ML2 preview enabled/disabled. Collect
the same session's ML2LAT, ML2Timing, ML2Capture, sensor-pose diagnostics and PCCore.
Check that applied_before_render grows and that capture-to-before-render improves,
along with p95/p99, without visual instability or new rejected/stale poses.

The phase-preserving limiter may increase the actual submitted/accepted rate
toward the existing setting, so also watch sender_overwritten, PC overwritten and
delivered_hz. It does not increase the sensor's configured capture rate.

The software endpoint remains before-render, not physical display presentation.
These changes cannot establish a sub-30 ms capture-to-visible result without
headset measurements. A higher capture_to_ready with a low ready_to_submit points
toward sensor/SDK delivery and polling cadence rather than desktop computation.

## Regression checks

```powershell
./tests/pose-freshness/Run-PoseFreshnessTests.ps1
./tests/pipeline-quality/Run-PipelineTests.ps1
```

Runners compile the production method bodies with test-only Unity stand-ins.
They cover application phases, duplicate protection, stale/session rejection,
contention without blocking, deferred timestamps, bounded diagnostics, rate pacing,
buffer ownership and C#-to-Python framing. Compile the actual Unity assembly for
Editor and Android conditions as well; stand-ins cannot validate device rendering.
