# Shared tracking core and on-device migration

## Current structure

```text
Live ML2 TCP input                        Recorded intensity + capture metadata
       |                                               |
depth_transport.py                              benchmark_tracking.py
       |                                               |
       +---------- TrackingFrame + TrackingConfig -----+
                               |
                      tracking_core.py
              prepare -> detect -> PnP -> world pose
                               |
                   TrackingResult + stage timings
                     /                       \
        pose_packet.py -> Unity          CSV + timing summary
                |
   desktop preview / interactive recording (after pose publication)
```

- `tracking_types.py`: frame identity, calibration, captured pixels, capture-time
  sensor pose and source timestamps. `TrackingFrame` and the old `DepthFrame`
  name refer to the same contract. Calibration must be resolved for every frame.
- `marker_detection.py`: intensity mapping, threshold, connected components,
  weighted subpixel centers and geometric filtering. Masked preview output is
  optional; the live core skips it.
- `marker_pose.py`: existing correspondence, PnP, refinement, motion gates and
  loss/reacquisition behavior. Numerical algorithms and default gates are unchanged.
- `pose_geometry.py`: image/camera conventions and world-pose composition,
  independent of TCP. Old geometry imports from `pose_packet` remain available.
- `tracking_core.py`: synchronous stateful image-to-world-pose API. It has no
  network, GUI, recording or scheduling operations. Existing optional tracker
  debug logging remains disabled by default.
- `depth_transport.py`: unchanged framing and one latest complete-frame slot.
- `depth_stream_receiver.py`: CLI, connection lifecycle, age-at-publication
  checks, diagnostics, display and recording. Existing helper names remain
  importable for notebooks/tests, but new algorithm code should import core modules.

## Calling the core

```python
from tracking_core import TrackingPipeline, TrackingConfig

pipeline = TrackingPipeline(TrackingConfig())
result = pipeline.process(frame)  # no TCP/Unity dependency
if result.ok:
    position = result.solution.position  # xyz, metres, Unity world
    rotation = result.solution.rotation  # xyzw, unit quaternion
```

Use one instance on one processing worker for one ordered stream. Pixels and
sensor-pose arrays are borrowed read-only until `process` returns. Keep them alive
and unchanged. Strided images are accepted. `result.intensity` and
`result.prepared_image` may alias the input; copy them only if retaining them after
the capture buffer is reused. Calibration matrices in results belong to the core;
consumers must not modify them. No mutable array is safe to reuse while processing.

Model points use metres. Intrinsics are for the original sensor image; the core
adjusts the principal point for the flipped stream exactly once. Sensor pose must
describe the same capture as the pixels. It must not be replaced by the headset's
current pose, predicted display pose, or a separately smoothed pose.

Session, image format/size, calibration or camera-axis changes reset temporal
priors. Duplicate/backward frame IDs or observation times are rejected. Empty
detections still reach the tracker so source-time loss/recovery behavior continues.
An accepted result contains confidence, correspondences and reprojection error;
failures contain a reason and tracking state, never a fabricated world pose.

The optional `deadline` is an absolute time in the processing worker's monotonic
clock. Live TCP sets it to PC receipt + `--max-processing-age`; offline replay
omits it. Never compare PC scheduling time to ML2 capture time directly. If a solve
expires, its prior is cleared. The transport adapter checks age again immediately
before publication. Individual native/OpenCV calls cannot be interrupted by this
deadline; this is a result-age bound, not a hard real-time execution guarantee.

## Performance changes in this refactor

1. Both interactive and headless UINT8 tracking use the same grayscale image.
   No BGR expansion is done before sending the pose.
2. Threshold intensity/mask is calculated once, and no thresholded preview image
   is allocated by the tracking core. Float32 weights are retained for centroid
   summation compatibility.
3. FLOAT32 `unity-raw` mapping also stays grayscale. Legacy `turbo` still maps to
   color and uses the maximum channel for detection; its tracking behavior differs
   from `unity-raw` by design.
4. Camera matrices, distortion and detection arguments are cached until changed.
5. Preview image conversion, masked preview, overlays, display and recording run
   after pose publication. They can still delay processing the next frame; use
   `--headless` for latency measurements. The latest-frame reader continues to run.
6. `--max-detected-markers` is now honored (default 8, matching the previous actual
   candidate pool). The old advertised default of 4 was not wired into detection.
   The pose tracker retains its existing eight-candidate cap.

Threshold keys apply to the next frame. Recording metadata keeps the configuration
that actually produced the current frame's centers/pose. Stopping a recording no
longer overwrites the current frame variable while compiling the preview video.

## Timing interpretation

Wire protocol versions and fields are unchanged; the existing APK is compatible.

`[PCCore]` prints rolling p50/p95/p99 in milliseconds:

| Field | Scope |
| --- | --- |
| `receive_queue` | Complete PC receive to entry into the core, including adapter work |
| `prepare` | Input validation/context, calibration cache lookup, intensity preparation |
| `detect` | Threshold/blob/centroid/geometry detection |
| `pnp` | Camera basis, correspondence, solving, refinement and temporal validation |
| `world` | World-pose composition and final validation |
| `core_total` | Full synchronous core call, including small handoffs between stages |

`[PCTracking] pc_work_ms` remains the adapter processing-to-publication measurement;
it includes the core and send work. `[ML2LAT] detect` carries `prepare + detect`,
and its historical `pack` field carries world-pose preparation. Actual packet
serialization and send occur after the PC-send timestamp and are included in the
return-leg estimate. Main-thread Unity application and sensor delivery timing are
unchanged by this refactor. No E2E claim follows from core-only timings.

## Offline benchmark

Record using the existing preview receiver and the `R` key, then run:

```powershell
python benchmark_tracking.py saves\recording_YYYYMMDD_HHMMSS --csv tracking.csv --json tracking-summary.json
```

This reruns detection and PnP on saved UINT8 intensity, with the saved calibration,
capture time, sensor pose, session, detection settings and tracker timeouts. Source
format switches still reset priors; intensity is never mapped a second time.
CSV contains frame identity, source time, centers, acceptance/state/reason, world
pose, confidence, reprojection error and stage durations. Summary contains p50,
p95 and p99, environment versions, model geometry and acceptance counts.

It excludes disk I/O, TCP, capture delivery, Unity application and presentation.
Saved UINT8 frames cannot measure original FLOAT32-to-UINT8 conversion cost.
Acquisition is included; results also depend on CPU load and time-bounded search.
Do not compare runs with different inputs/settings as an algorithm speedup.
Recordings are currently interactive; `--headless` remains free of recording I/O.

Older recordings with capture-time sensor metadata but no detection settings need
the explicit `--allow-legacy-defaults` option. The report counts those frames.
Missing sensor-pose metadata is an error, not an assumption of a stationary head.
The existing `replay_recording.py` remains the specialized centers-only solver
diagnostic; it does not measure detection or the full new core.

## Future native implementation

The new boundary is the reference for a native backend, not a native implementation.
Port and validate the core incrementally; retain this Python path as the oracle.

The future C ABI should have an opaque per-stream tracker handle and operations
to create/configure, reset, process and destroy it. A versioned input descriptor
should carry borrowed pixels, width/height, row stride, explicit pixel format,
session/frame identity, exact capture timestamp, capture-time world sensor pose,
calibration and image-axis conventions. Output should carry status/reason, source
identity, pose in metres/xyzw, confidence, diagnostics and stage timings. Keep
allocator ownership, byte order, structure size and ABI version explicit. Never
pass C++ containers or rely on compiler-specific struct layout across P/Invoke.

On ML2, a capture adapter hands a valid buffer to one native processing worker;
Unity consumes the newest validated result on its main thread. Keep only the
newest waiting frame, without freeing or mutating an in-flight buffer. A Windows
DLL can run the same native core against this dataset before Android ARM64 `.so`
integration. TCP then becomes an optional development adapter rather than part
of tracking.

Validate accepted/rejected sequences, moving-head compensation, partial occlusion,
clutter, loss/recovery, position and rotation deltas and p50/p95/p99. Synthetic
equivalence is useful for regressions; it does not establish real-world accuracy.
Native floating-point and solver differences need explicit, measured tolerances.
Capture delivery, Unity application timing and actual presentation still need
on-device measurement and optimization after the port.

## Verification

```powershell
python -B -m unittest discover -s . -p "test_*.py"
```

The regression suite includes borrowed/strided images, Float32/UINT8 equivalence,
sequence ordering, capture-time motion, calibration/session resets, expiration,
lost tracking/reacquisition, preview-after-publication ordering, replay metadata,
wire framing and rejected-pose behavior.
