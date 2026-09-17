# Magic Leap 2 depth receiver

The Unity app is the TCP server and the PC initiates the connection. Unity exposes
two full-resolution processing pipelines. The server keeps only
the newest waiting frame, so a slow connection does not stall the Unity sensor
loop or build an unbounded queue.

## Unity / Magic Leap setup

The demo scene's `IRToolManager` now has a **Depth Frame Tcp Server** component.
Its defaults are:

- Bind address: `0.0.0.0` (accept Wi-Fi and ADB-forwarded connections)
- Port: `50777`
- Maximum rate: `15` frames/s by default; the demo scene uses `30` (`0` means every sensor frame)
- Pipeline Mode: `Pipeline 2 - UINT8 (ML2 conversion)` by default
- On-screen status: enabled

The fallback overlay displays the Magic Leap Wi-Fi address and port. For a
reliable XR-world display, create a world-space Canvas with a TextMeshPro
`Text - TextMeshPro` component and assign its `TMP_Text` to **Status Text**. You
may assign a second TMP component to **Sensor Info Text** for the live pose and
intrinsics. If that field is empty, sensor information is appended to Status
Text automatically. The fallback overlay can then be disabled.

`ML2DepthRawStream` also adds a server with these defaults at runtime if none is
assigned, which makes the streaming change usable from other scenes without a
scene edit.

### Select Pipeline 1 or Pipeline 2

Select **IRToolManager → Depth Frame Tcp Server → Processing pipeline → Pipeline Mode**.

| Mode | ML2 preparation | Wire payload | PC preparation before shared detection |
| --- | --- | --- | --- |
| Pipeline 1 - Legacy FLOAT32 (PC conversion) | Copy original DepthRaw floats | FLOAT32, 4 bytes/pixel | `colourise_depth()` → UINT8 BGR |
| Pipeline 2 - UINT8 (ML2 conversion) | Normalize, optional sRGB mapping, round to UINT8 | UINT8, 1 byte/pixel | Grayscale-to-BGR |

Pipeline 1 restores the image-processing path from
`e8a320133921185f47d032a4ce7c3941d4206251`, with the correspondence and
explicit tracking-loss/reacquisition fixes. Both modes share those fixes and
the same `render_analysis()` → threshold → blobs → centres → correspondence →
PnP → sensor-world-pose composition → Unity return path. This is a selectable
processing path, not a checkout/revert of the entire repository.

Runtime API (the integers are pipeline numbers, not dropdown indices):

```csharp
server.SetPipelineMode(1); // FLOAT32 to PC
server.SetPipelineMode(2); // UINT8 to PC
// Or: server.SelectedPipeline = DepthFrameTcpServer.PipelineMode.LegacyFloat32;
```

The packet carries its pipeline's pixel format, including during runtime switches.
Python selects the correct path automatically and resets the tracker on a mode
change. Build and deploy the updated Unity app to expose the selector on the
headset; editing source/Inspector values alone does not update an installed APK.
Restart Python after updating this receiver.

For comparable detection input, use Pipeline 1's default `--view unity-raw`
and match `--raw-min 5 --raw-max 3000 --unity-color-space linear` to Unity's
`rawMin`, `rawMax`, and `convertLinearToSrgb`. The Pipeline 1 `--view turbo`
option retains legacy behaviour but is not equivalent to Pipeline 2.
Keep resolution, exposure, connection, and frame-rate limit fixed for A/B tests.

## PC setup

Create a virtual environment and install the receiver dependencies:

```powershell
cd desktop
py -m venv .venv
.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

### Connect over Wi-Fi

Put the PC and Magic Leap on a mutually reachable network. Use the address shown
in the headset/app:

```powershell
python depth_stream_receiver.py --host 192.168.1.123 --port 50777
```

If this times out, verify that client isolation is disabled on the Wi-Fi network
and that the PC firewall permits the Python process. Android does not need a
runtime permission prompt for `INTERNET`; the permission is included in the app
manifest.

### Connect over USB with ADB forwarding

With the headset visible in `adb devices`, forward the PC port to the listening
port inside the Magic Leap:

```powershell
adb forward tcp:50777 tcp:50777
python depth_stream_receiver.py --host 127.0.0.1 --port 50777
```

To remove the forwarding rule later:

```powershell
adb forward --remove tcp:50777
```

If the Unity port is changed, use the same port in both arguments to `adb
forward` and in the Python command. `0.0.0.0` is the recommended Unity bind
address; `127.0.0.1` may be selected to allow only ADB-forwarded access.

Press `Q` or Escape to quit the viewer. Press `S` to save the normalized intensity
matrix as `.npy`; use `--save-dir PATH` to select its directory.
Pipeline 2 applies the intensity mapping on ML2; Pipeline 1 applies it on PC.
At 544x480, Pipeline 1 sends 1,044,480 payload bytes/frame and Pipeline 2 sends
261,120. Recordings from either pipeline contain the resulting UINT8 intensity.

Unity `[ML2LAT]` logs identify `pipeline=1 FLOAT32 raw` or `pipeline=2 UINT8 sRGB`
and report `ml_prepare+queue+tcp_write`,
`leg1_net(ML2->PC)`, PC processing, return transport, and Unity apply wait.
The receiver automatically reconnects if a deployment or app restart leaves a
stale TCP connection; adjust this with `--frame-timeout` if needed.

The receive timestamp is recorded immediately after payload/metadata decoding,
before `colourise_depth()` or grayscale-to-BGR conversion. PC preparation is
included in `detect`, not `leg1_net`. The PC logs the pipeline and actual
payload size on the first frame and whenever the mode changes.
The Unity `Sensor input is FLOAT32 DepthRaw` log describes acquisition before
conversion; use the TCP pipeline log to identify the transmitted representation.

## Correspondence and recovery

The receiver keeps up to eight candidate blobs after area/aspect checks. Its
spatial filter retains plausible groups without selecting one constellation.
The tracker assigns projected model points to candidates one-to-one, maximizing
the number inside the 40-pixel gate and then minimizing total pixel distance.
Detection array indices are not treated as persistent marker identities.

Geometric acquisition ranks candidate subsets before PnP. Surplus candidates
are treated as possible clutter rather than missing model markers. Acquisition
uses at most 32 solver hypotheses, with a 12 ms budget checked between calls.
Candidate ranking and an individual OpenCV call can exceed that budget; it is
not a hard deadline for the whole frame. These limits are configurable on
`MarkerPoseTracker` as `max_search_hypotheses` and `search_budget_ms`.

Tracking expires after 200 ms without an accepted observation. Empty detections
also update this timeout; a gap with no delivered frames is handled on the next
observation. After loss, stale pose priors are cleared and acquisition requires
two consecutive, consistent estimates using at least four markers.

During tracking, the translation gate is
`min(0.20 m, 0.015 m + 1.0 m/s * time_since_last_accepted_observation)`.
The speed is camera-relative, so both headset and probe motion contribute.
Acquisition confirmation also checks rotation consistency (15 degrees plus
360 degrees/s times the observation interval, capped at 60 degrees).
Tune `lost_timeout_s`, `position_margin_m`, `max_relative_speed_m_s`,
`max_translation_jump_m`, and the `acquire_*` fields in `marker_pose.py`.

Timing uses the frame's ML2 source timestamp, not PC processing time. The current
wire timestamp is taken at submission, so it is a capture-time proxy rather
than the exact sensor exposure time. Both offline replay scripts use the saved
timestamps too. `tracker.state` reports `acquiring`, `tracking`, or `lost`.
These tracking fixes apply to both pipelines. Sensor-pose filtering and Unity's
visibility timeout use the same current settings in both.

Run the regression checks from the repository root:

```powershell
python -B -m unittest discover -s desktop -p test_tracking_recovery.py -v
```

For a headset check, acquire with all four markers visible, move the probe,
occlude the markers for more than 200 ms, and reveal them at a new location.
Reacquisition should not require returning to the old location. This update
does not eliminate all planar-pose ambiguity or implement final-pose smoothing.

## Wire protocol

Each frame has a 112-byte little-endian header (base `<4sHHIIIIQd3f4fI>` plus
40-byte timing extension `<Qqddd>`) followed by
`width * height * 4` little-endian FLOAT32 values (Pipeline 1) or
`width * height` UINT8 intensity values (Pipeline 2) and, when present, a fixed
88-byte numeric intrinsics block (`<11d`):

1. Magic `ML2D`
2. Protocol version `5` for both pipelines and header size `112` (Python also reads legacy v3/v4)
3. Width, height, pixel format (`1` = FLOAT32 DepthRaw, `2` = UINT8 sRGB intensity), payload byte count
4. Frame number and UTC Unix timestamp in seconds
5. Sensor position `x,y,z` in metres and rotation quaternion `x,y,z,w`, both in
   the Unity world coordinate system
6. Numeric intrinsics byte count (`0` or `88`)
7. Session ID (`uint64`), exact capture XrTime (`int64` nanoseconds), mapped capture
   time, SDK-poll start, and SDK-frame-ready time (three `float64` Unity-clock seconds).
   Mapped capture time is NaN when clock conversion is unavailable.

The sensor pose is synchronized with every depth frame. Intrinsics are sent once
per TCP connection (on the first frame after metadata becomes available), then
the intrinsics byte count is zero for later frames. In Python, `receive_frame()`
returns a `DepthFrame` containing unmodified `pixels`, `pixel_format`, `sensor_position`,
`sensor_rotation`, and the optional one-time `CameraIntrinsics`. The numeric
intrinsics order is `fx, fy, cx, cy, fov_x, fov_y, k1, k2, p1, p2, k3`.
`CameraIntrinsics.camera_matrix` and `.distortion_coefficients` are immediately
usable with OpenCV.

## Tracking fixes: rotation, backlog, and slow capture

The live tracker now expresses the previous probe pose in the current camera
frame using the synchronized sensor pose in each image packet. This compensates
for headset movement before marker association, PnP seeding, and motion checks.
Both the headset and probe can move. Probe rotation is hard-gated at
`min(60 degrees, 15 degrees + 360 degrees/second * elapsed)`; translation retains
its time-dependent gate. Each solver candidate is checked before ranking.
Rejected jumps do not refresh the accepted observation time.

A dedicated depth reader continuously drains TCP into one replaceable complete
frame slot. Processing takes the newest frame, with its original PC receive time
and connection calibration. Intermediate frames are dropped during processing,
preview, and recording stalls. The FLOAT32/UINT8 pixel encodings remain unchanged; v5 adds timing and session metadata.

### Independent timing settings

| Setting | Default | Purpose |
| --- | --- | --- |
| `--max-processing-age` | 0.10 s | Reject before solving or sending if too old since complete PC receipt |
| Unity `maxPoseAgeSeconds` | 0.15 s | Reject at apply if too old since that frame's ML2 submission |
| `--tracking-lost-timeout` | 0.65 s | Clear the prior after no accepted source observation |
| `--acquisition-timeout` | 0.65 s | Maximum interval between acquisition confirmations |
| Unity `hideAfterNoPoseSeconds` | 0.65 s | Hold until the last accepted frame's source submission expires |

The demo scene enables duplicate/older frame rejection and the above Unity
settings. Restart the desktop receiver and rebuild/deploy the Unity APK from
this checkout. Existing desktop launch commands still work; the new flags are
optional. Other scenes use the new script defaults for pose age; check their
serialized stale-frame and hide settings if they have their own server component.

At 5 Hz, frames arrive about 200 ms apart, but each may still complete its round
trip in less than 150 ms. The 650 ms loss/hold settings tolerate gaps between
fresh observations; they do not add smoothing or wait before applying a pose.
The last accepted model can remain still for up to 650 ms after its source
submission when tracking fails. Genuine probe turns beyond the configured motion
gate are also rejected and require reacquisition after loss.

ML2 checks age using its own monotonic submission-time ring, not PC clock sync or
logcat timestamps. Unknown, duplicate, older, and expired frames cannot refresh
visibility; expired frames cannot bring back a hidden model. The status display
includes the count of stale poses rejected at apply.

### Verification and latency limits

```powershell
.venv/Scripts/python.exe -B -m unittest discover -s desktop -p 'test_*.py' -v
pwsh -NoProfile -File tests/pose-freshness/Run-PoseFreshnessTests.ps1
```

On-device checks: move the headset with a stationary probe, move both independently,
occlude markers for longer than 650 ms, and briefly stall the desktop preview.
Recovery must use fresh frames rather than replaying old motion. Watch the
`[ML2LAT]` submit-to-apply samples and the status display's stale count. If normal
poses exceed 150 ms, they will now be dropped; measure which stage exceeds budget
before tightening the limits further.

These limits discard stale work; they do not establish latency below 30 ms.
Already-buffered TCP bytes still cross the network before the reader can discard
frames. Preview, disk writes, and interactive save/video operations can still
pause solving, although they no longer create a desktop frame FIFO. Benchmark
without recording/save prompts. Version 5 uses exact monotonic capture timestamps
for tracker intervals; the UTC field is retained only for recording names and legacy readers.
Legacy v3/v4 inputs still use the UTC submission proxy.
Actual capture-to-display latency still needs hardware measurement.


## Fixes #6–8: timing, frame preparation, and centroid precision

### Deployment

**Rebuild/deploy the Unity APK and restart this desktop receiver together.**
Depth packets are now v5 (112-byte header); pose replies are v4 (88 bytes) with an
echoed session ID. The new Python decoder can read old depth recordings/streams,
but an old live desktop receiver cannot read the new ML2 header. Clock-sync
messages remain v3 and 80 bytes. A new ML2 pose server requires a matching nonzero
session; old clients cannot apply untagged poses.

The session changes when the server starts or sensor history/XR origin resets.
Pending frames and timing history are invalidated, the old visual is hidden, and
the PC starts acquisition again. A worker retains the session of its original
frame even if the origin changes during conversion or network transmission.

### Reading the measurements

- `[ML2LAT] submit_to_apply` replaces the misleading `e2e` label. It includes
  sender conversion/queue/write, PC work, transport, and Unity apply waiting.
- `[ML2Timing]` reports rolling p50/p95/p99 and sample counts for capture-to-submit,
  capture-to-apply, SDK polling, frame-ready-to-submit, submit-to-apply, and
  apply-to-`onBeforeRender`. Windows contain at most 256 accepted samples and reset
  for a new session. Cumulative counters include received/applied/rejected/stale,
  wrong-session replies, sender overwrites, and rate-limited frame drops.
- Displayed pose age is shown separately, with `hidden` when the model is hidden.
  Both capture age (when available) and submission age are reported.
- `[ML2SensorPoseHistoryDiag]` now also reports delivered frame rate and mean
  interval between distinct source capture timestamps, including frames whose
  pose lookup subsequently fails.
- `[PCTracking]` reports received/overwritten/processed/accepted/failed-or-expired
  counts and rolling PC-work percentiles across processed successes and failures.
  `--diagnostics-interval 0` disables PC summaries; Unity's `logLatency` disables
  detailed apply summaries. Source-pose diagnostics remain independently throttled.

Capture time is converted via the SDK's XR-to-system conversion, sampled against
Android `CLOCK_MONOTONIC`, then mapped to Unity's monotonic clock. This follows
[OpenXR's clock conversion contract](https://registry.khronos.org/OpenXR/specs/1.0/man/html/xrConvertTimeToTimespecTimeKHR.html).
The bridge rejects sampling spans above 2 ms, failed conversions, negative ages,
and implausibly old captures. It reports unavailable rather than inventing an
offset or using predicted display time as the current time. Its sampling error
is bounded by half the accepted sampling span, plus runtime conversion error.

`onBeforeRender` is a CPU callback, **not physical display presentation**.
Presentation remains explicitly `unmeasured`. Sensor exposure details and
motion-to-photon latency still need on-device/high-speed-camera validation.
One-way transport splits remain estimates; constant clock-sync offset cancels
between the two legs, so `unaccounted` mainly represents PC queue/omitted work.
The existing 150 ms rejection threshold still measures submission-to-apply;
the new capture measurements allow that policy to be assessed before tightening it.
Always assess accepted rate, drop counts, and pose error alongside percentiles.

### Faster preparation without changing intensity mapping

ML2 now checks connection/rate eligibility before copying the raw plane. It
uploads the preview only when enabled and a renderer exists, at a configurable
10 Hz default. UINT8 conversion runs on the sender thread with an owned raw
buffer and snapshotted mapping settings. Full-frame copies and conversion run
outside the handoff lock. Conversion preserves the old mapping and rounding;
no approximate lookup table or extra filter has been introduced.

For a clean latency trial from the repository root:

```powershell
.venv/Scripts/python.exe desktop/depth_stream_receiver.py --host 127.0.0.1 --send-pose --headless
```

Use the headset IP for Wi-Fi, or the existing ADB forwarding setup for localhost.
`--headless` skips GUI, overlays, BGR expansion for UINT8 frames, and interactive
save/recording work. Normal preview and recording remain available. Disk/video
operations in interactive mode can still pause solving; the latest-frame reader
prevents them from creating an accumulating frame queue.

Blob centroid scans are restricted to each component's bounding box. Hypothesis
ranking now has its own 12 ms budget, checked per detection subset, in addition
to the existing 12 ms solver budget. These are cooperative checks, not a hard
whole-frame deadline: a running native solver call cannot be interrupted.

### Accuracy and validation

Intensity-weighted centroids remain floats through all geometry filters,
correspondence, and PnP. Recordings store float64 centers plus JSON sidecars with
exact observation time, capture nanoseconds, session, sensor pose, and the PnP
camera matrix. `replay_recording.py` uses these to compensate head motion during
replay; old recordings without sidecars still load with their previous metadata.
Rounding is confined to drawing.

Final refined candidates are checked again for finite values, positive depth,
and depth range. The packet boundary validates finite values and unit rotation.
The unsupported unflipped/no-object-conversion combination is rejected explicitly
because it creates a reflection; the default flipped pipeline is unchanged.

```powershell
.venv/Scripts/python.exe -B -m unittest discover -s desktop -p 'test_*.py' -v
pwsh -NoProfile -File tests/pipeline-quality/Run-PipelineTests.ps1
pwsh -NoProfile -File tests/pose-freshness/Run-PoseFreshnessTests.ps1
pwsh -NoProfile -File tests/sensor-pose/Run-SensorPoseTests.ps1
```

Regression checks cover subpixel motion, refined-pose rejection, headless operation,
clock mapping failures, session reset, buffer ownership during repeated capture,
200,016 pre-change mapping comparisons, and actual C#-written/Python-decoded packets.
Unity Editor and Android conditional code also require a project compile.
These software tests do not establish a measured latency below 30 ms on ML2.
