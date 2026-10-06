# Unified tracking API

The two demo scenes now use `TrackingSystem` through a generic Unity host. A provider owns the
complete device pipeline, including resource startup, acquisition/SDK connection, estimation or
remote transport, and publication. Application code asks for a logical object's pose. It does
not need to know whether that pose came from ML2, another AR device, replay, or an external OTS.

The working ML2 + desktop pipeline is implemented. The desktop Python algorithm and TCP protocol
are unchanged. OTS + AR composition is implemented and tested with simulated providers; a real
OTS/HoloLens SDK provider and measured cross-device calibrations are still required for those devices.
No files in ArNav-ML are changed by this restructuring.

## Modules and dependencies

```text
Application: trace UI, instrument calibration, sampling, export, later registration/navigation
                                |
                        ITrackingSystem
                                |
        TrackingSystem: lifecycle, logical objects, explicit provider priority,
             clock validity, calibrated frame conversion, relative poses
                  /                              \
          ITrackingProvider                  ITrackingProvider
          ML2 implementation                 future OTS implementation
          sensor -> TCP -> desktop            vendor SDK -> solved body poses
                  \                              /
                    TrackingProviderContext
                                |
                 TrackingCore: validated observations
                                |
               raw tracing input / separate Unity presenter
```

| Folder / assembly | Responsibility |
|---|---|
| `Core/` | Engine-free rigid geometry, sessions, timestamps, observation validation and freshness |
| `Runtime/` | Engine-free unified service, provider contract, lifecycle, clock mappings, frame calibrations, logical object routing, replay |
| `Application/` | Engine-free raw tip observations for tracing; instrument offset is supplied by the application |
| `Unity/` | Generic host, provider-component factory, presentation, Unity coordinate conversion, asynchronous cleanup runner |
| `Implementation/ML2/` | Complete ML2 provider and Unity configuration component |
| `Implementation/ML2/Runtime/` | Existing capture, sensor pose history, raw image preparation and TCP implementation |
| `Implementation/ML2/Compatibility/` | Previous single-backend wrapper retained for callers using the older API |
| `Assets/ProbeTracing/` | Probe geometry, countdown, sampling/strokes, visual prefabs, UI and exports |

Core, Runtime and Application have `noEngineReferences: true`. Unity has a separate assembly
with no Magic Leap references. The ML2 implementation currently compiles into Assembly-CSharp
alongside the project's SDK integration. The legacy `Application/TrackingApplication.cs` remains
for compatibility; new integration should use `ITrackingSystem`, not that older wrapper.

Registration will consume landmarks/traces in patient-reference coordinates and return
`imageFromPatient`. Navigation can then combine that registration with current tracking.
Neither registration algorithms nor anatomy semantics belong in TrackingCore.

## Public API

Configure a concrete `TrackingSystem` with `AddProvider` and `BindObject` before starting; pass
its `ITrackingSystem` interface to consumers.

| Call | Meaning |
|---|---|
| `Start()` / `Stop()` | Request startup / shutdown of every configured provider |
| `Pump(phase)` | Advance lifecycle and consume provider mailboxes on the owning thread |
| `TryGetPose(objectId, requirements, ...)` | Pose of a logical body in the selected provider's native frame |
| `TryGetPoseInFrame(objectId, frame, requirements, ...)` | Same body in an explicitly calibrated target frame, e.g. AR world |
| `TryGetRelativePose(objectId, referenceObjectId, requirements, ...)` | Body pose relative to a tracked reference body, e.g. patient reference |
| `TryGetProviderFrame(...)` | Current provider frame including its epoch |
| `SetFrameCalibration(...)` / `RemoveFrameCalibration(...)` | Register/remove a measured rigid relationship between current frames |
| `ResetProviderReference(...)` | Invalidate the old origin/session and its associated calibrations |
| `GetProviderStatus()` | Individual startup/running/stopping/failure status, including optional providers |

`TrackingState` is Stopped, Starting, Running, Stopping, Faulted or Disposed. **Running does not
mean a probe is visible.** Query success determines whether a fresh observation is available.
Start/Stop are requests, not blocking completion calls. Continue `Pump(Update)` until shutdown
finishes; `TrackingSystem.ShutdownComplete` is the disposal condition. Faulted also requires
cleanup pumping while a provider still owns resources. Providers must remain Stopping until
resource release succeeds, rather than report Faulted/Stopped with live resources.

All service commands, queries and publications belong to the creating thread. A threaded OTS SDK
must place results in its provider's mailbox; its Pump publishes them through the supplied
`TrackingProviderContext`. No worker may manipulate Unity objects or publish directly.

`ResolvedTrackingPose` includes the output frame, logical body transform, original source
observation, optional reference observation, service revision and clock uncertainty. Source
session, sequence, capture time, calibration identity and confidence remain available.

## Combined OTS + AR

Three different mappings are required. None is inferred from similar coordinates or IDs:

1. **Object mapping:** e.g. OTS body `tool-17` and ML2 body `marker-body` both represent logical
   `probe`. `ProviderBodyFromLogicalBody` aligns their body coordinate axes; identity is valid
   only if both models actually use the same origin/orientation. Probe tip offset is separate.
2. **Time mapping:** each provider supplies an explicit affine `ClockMapping` from its capture
   clock into the runtime clock, with bounded uncertainty and a validity interval. Receive time
   is not a substitute for capture time. ML2 already maps capture timestamps into Unity time
   inside its existing capture adapter, so the provider uses an identity mapping at this boundary.
3. **Spatial mapping:** measured `arWorldFromOtsWorld` associates the two providers' current
   frame epochs. Camera intrinsics alone cannot supply this transform. Renew it when its
   calibration expires or an origin/session changes.

For an object seen by more than one provider, list bindings in the desired priority order.
Fallback occurs only when a provider lacks a usable measurement under the requested freshness
and clock limits. The system does not average poses. A selected pose with no spatial calibration
is rejected; it does not silently choose another provider to hide that configuration error.

The frame graph supports inverse and multi-hop transformations, with one unambiguous path.
Cycles are rejected. An origin reset, session replacement or provider stop removes affected
calibrations. Calibrations have explicit validity intervals evaluated at query time. There is no
automatic extrinsic calibration, drift estimation or fusion in this implementation.

For two bodies, both observations must be fresh and their capture-time separation plus combined
clock uncertainty must be within the caller's skew budget. The service does not interpolate or
predict asynchronous measurements. An overly strict budget may intentionally reject a query.

### Composition example

```csharp
using Braillic.Tracking;
using Braillic.Tracking.Runtime;
using Braillic.Tracking.Application;

// ar and ots are complete ITrackingProvider instances constructed by device adapters.
var tracking = new TrackingSystem(clock);
tracking.AddProvider(ar, required: true);    // required for AR-world presentation
tracking.AddProvider(ots, required: false);  // an OTS fault permits configured AR fallback
tracking.BindObject("probe",
    new ObjectBinding(ots.Id, "tool-17", otsBodyFromProbe),
    new ObjectBinding(ar.Id, "marker-body", arBodyFromProbe));
tracking.BindObject("patient", new ObjectBinding(ots.Id, "reference-8", RigidPose.Identity));
tracking.Start();
// The owner continues Pump(Update), Pump(LateUpdate), optionally Pump(BeforeRender).

// After both providers are running, install an independently measured calibration.
if (tracking.TryGetProviderFrame(ots.Id, out var otsFrame) &&
    tracking.TryGetProviderFrame(ar.Id, out var arFrame))
    tracking.SetFrameCalibration("ots-to-ar", otsFrame, arFrame,
        measuredArWorldFromOtsWorld, validFrom, validUntil);

// These are example policy values, not achieved accuracy or recommended clinical limits.
var limits = new ObservationRequirements(0.1, 0.02, 0.005);
bool usable = tracking.TryGetRelativePose("probe", "patient", limits,
    out var patientFromProbe, out var reason);

var tips = new TraceObservationSource(tracking, "probe", "patient",
    calibratedTipInProbeBody, instrumentCalibrationId, limits);
if (tips.TryRead(null, out var tip, out reason))
{
    // tip.TipPosition is in patient-reference coordinates.
    // Apply application sampling policy, deduplicate session + sequence,
    // and start a new collection when tip.CollectionEpoch changes.
}
```

An AR provider used solely for visualization can expose its current world frame without
publishing probe measurements. If the patient reference becomes unavailable, relative queries
fail even if a valid AR probe pose remains. A bedside reference only represents patient motion
while the anatomy remains rigidly related to it; the tracking API cannot establish that relationship.

## A single frame through the current ML2 implementation

1. `TrackingApplicationHost.Update` calls `TrackingSystem.Pump(Update)`.
2. `Ml2TrackingProvider` advances sensor permissions/configuration/start, then polls capture once
   per Update after SDK startup succeeds. Capture-time sensor pose and timestamp mapping use the
   existing implementation. Frame submission remains gated until the provider is Running.
3. `ML2DepthRawStream` -> `DepthFrameTcpServer` prepares and submits the image with its original
   frame/session identity, intrinsics and capture-time sensor pose. Existing rate/queue rules apply.
4. The unchanged desktop detects markers, estimates the body pose and sends the existing packet.
5. LateUpdate or before-render consumes a new return packet through `PoseEstimateTcpServer`.
   Existing packet, session, capture-age and pose validation occur before publication.
6. The provider converts Unity geometry once into canonical geometry and publishes through its
   current context token. Core checks ordering, timestamps, session and pose validity. A delayed
   result from an old session cannot revive a pose.
7. The same acceptance callback lets the generic presenter query the logical `probe` in AR world.
   No extra Update is needed merely to copy the accepted pose into the API.
8. Tracing reads raw body measurements through `TraceObservationSource`, applies its configured
   tip calibration, then the existing sampler decides whether to append a point. Display smoothing
   never becomes registration input. Latest-only sampling is not lossless recording of every frame.

Canonical geometry uses metres, right-handed X-right/Y-up/Z-backward and Hamilton XYZW rotations.
`AFromB` maps B-local points into A coordinates. Unity conversion is at the implementation and
presentation boundaries. Image/DICOM coordinate conversion belongs to image/registration adapters.

For a future OTS frame with probe and patient reference in OTS coordinates:

```text
patientFromProbe = inverse(otsWorldFromPatient) * otsWorldFromProbe
patientTip      = patientFromProbe * calibratedTipInProbe
arWorldFromProbe = arWorldFromOtsWorld * otsWorldFromProbe
imageTip         = imageFromPatient * patientTip        [later Registration/Navigation]
```

For bodies measured by different providers, the relative-pose query inserts the calibrated
world-to-world transform and validates both measurement times before returning a result.

## Existing scenes and tracing

Both `IRtoolTracking_demo` and `ProbeSurfaceTracing_demo` are already wired with:

- Generic `TrackingApplicationHost`: providers, logical bindings, presentation frame, freshness.
- `Ml2TrackingProviderComponent`: sensor, raw capture, sender and receiver dependencies.
- `TrackedBodyPresenter`: existing model, smoothing and visibility settings.

The host's UnityEvent commands remain `StartTracking`, `StopTracking`, `ResetTrackingReference`.
Managed capture and TCP components do not independently start a second tracking loop. Stop now
requests SDK stop and release as well as stopping transport; outstanding configure/start/stop
operations are allowed to complete before teardown or restart. A persistent cleanup runner
finishes asynchronous release after host disable/scene unload. Release failure remains visible
and requires an explicit Stop retry; it never reports successful release with an owned sensor.

Moved scripts retain their `.meta` GUIDs. Existing cylinder, marker connection point, actual saved
tip calibration, trace prefab, countdown and sampler settings are preserved. The current demo
still traces in **Unity world**, because the current desktop pipeline supplies a single body.
Patient-relative tracing is available through the application API once a provider supplies a
patient reference. This change does not add patient-reference detection to the desktop.

JSON trace schema is `tracking-probe-trace-v4`, recording collection epoch, provider/session,
frame/epoch and tracking revision. Existing demo CSV/PCD coordinates remain Unity-world metres.
Collection is invalidated across coordinate resets, provider switches or calibration changes.
Transient tracking loss breaks a stroke; retained visuals are not accepted measurements.

## Integrating into ArNav-ML

Copy `Core`, `Runtime`, `Application` and `Unity` (including `.meta` and `.asmdef` files) as one
versioned module into ArNav. Consumers reference their assemblies and `ITrackingSystem`.
For ML2, include `Implementation/ML2` with the same compatible SDK/OpenXR packages and explicitly
wire its capture/transports and scene dependencies. Prefabs/UI can then use the generic host.
Do not load the standalone tracking Unity application or connect two Unity applications together.

For another device implement `ITrackingProvider`, optionally expose its factory as a
`TrackingProviderComponent`, and configure logical object mappings. Intrinsics, vendor marker
definitions, network protocol and native estimation libraries stay inside that implementation.
`ReplayTrackingProvider` is a working SDK-free provider for recorded relative-time poses and loss
events, useful for offline tracing/registration development. It takes records in memory; file
parsing is the application's responsibility.

The current ML2 TCP implementation retains its singleton listeners and supports one ML2 pipeline
per process. Independent OTS providers can coexist with it. Multiple ML2 pipelines would require
further transport-instance work. No on-device detection/PnP port is included here.

## Validation and hardware follow-up

Run from the repository root in separate PowerShell processes for scripts using Add-Type:

```powershell
./tests/tracking-application/Run-TrackingApplicationTests.ps1 -UnityReferenceProject ./ML2InfraredTracking
python ./tests/tracking-application/verify_scenes.py
./tests/sensor-pose/Run-SensorPoseTests.ps1
./tests/pose-freshness/Run-PoseFreshnessTests.ps1
./tests/pipeline-quality/Run-PipelineTests.ps1
./tests/probe-tracing/Run-ProbeTraceTests.ps1
./tests/probe-tracing/Run-ProbeSceneReferenceTests.ps1
```

Tests exercise combined OTS + AR geometry/time validity, explicit fallback, session invalidation,
replay, raw-tip measurements, actual ML2 provider orchestration, asynchronous resource teardown,
and existing capture/pose/trace/wire regressions. Core/Runtime/Application compile without Unity;
the portable Unity assembly compiles without Magic Leap. The reference-project option recompiles
runtime sources against cached Unity/SDK references in a temporary directory, not an APK build.

Device validation remains necessary: launch both demos, move the probe, collect/export a trace,
Stop/Start including during startup, pause/resume, disable/re-enable the host, unload the scene,
and reset the reference. Check one connection per port, sensor reacquisition, stale model hiding,
trace invalidation and latency logs against the previous baseline. No hardware latency improvement
or external OTS interoperability has yet been measured for this restructuring.
