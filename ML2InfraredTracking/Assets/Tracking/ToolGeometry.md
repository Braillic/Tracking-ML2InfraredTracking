# Tool roles, geometry configuration and identification

## Scope

Pipeline 1 is AR tracking. Pipeline 2 reserves the OTS-authoritative tracking / AR-display
architecture. `UINT8 sRGB` and `FLOAT32 raw` describe image transport, not these pipelines.
The current ML2 implementation and desktop startup now label themselves Pipeline 1.
Serialized transport enum values and protocol bytes are preserved; no timing or detector changes
are part of this update.

The framework can represent and publish **two probes plus a dynamic reference frame in the same
capture**. It is not limited to three tools. Geometry identification is an extension contract;
the actual multi-geometry detection/PnP algorithm and multi-body desktop protocol are not supplied.
The current desktop adapter still supports one fixed asymmetric four-marker model and explicitly
rejects unsupported geometry selections rather than pretending to track them.

## Data ownership

| Concept | Example | Meaning |
|---|---|---|
| Geometry ID + revision | `probe-a-layout`, `1` | Immutable marker arrangement/version |
| Marker geometry | Body-local marker centres in metres | Data supplied to an identification implementation |
| Tool instance ID | `probe-1`, `probe-2`, `patient-drf` | Stable application identity for one physical tool |
| Tool role | `Probe`, `DynamicReferenceFrame`, `Instrument` | UI/application-selected meaning; never inferred from geometry |
| Provider body ID | `body/probe-1` | Adapter's ID routed to the logical tool |
| Body calibration | `ProviderBodyFromLogicalBody` | Aligns provider body axes with logical tool axes |

Probe tip offsets/lengths, countdowns, surface contact and tracing policies remain in the application.
Changing a role does not change marker detection. A DRF is an ordinary tracked rigid body whose
role tells consumers how the application intends to use it.

Geometry definitions use canonical right-handed X-right/Y-up/Z-backward coordinates in metres.
Unity geometry assets accept Unity body-local metres and perform the existing Z reflection when
creating a definition. Markers describe the provider body coordinate system. The configured
body transform converts this to the logical tool coordinate system. Image coordinates, camera
intrinsics and distortion belong to the detector/estimator implementation, not tool roles.

## Application/UI API

`ITrackingSystem` now exposes:

```csharp
TrackingConfiguration Configuration { get; }
void ConfigureTools(TrackingConfiguration configuration);
bool TryGetToolDefinition(string objectId, out TrackedToolDefinition tool);
```

Add providers before selecting tools. A UI constructs immutable definitions from its selected
tool library and applies one configuration snapshot. ConfigureTools atomically replaces the
logical routes; it does not append stale mappings. Every provider validates support before any
selection is installed. Failure preserves the previous configuration.

Stop and complete asynchronous resource release before changing the selection. Then configure
and Start again. This creates new tracking sessions and invalidates old observations and tracing
continuity. The legacy BindObject API remains available for older consumers without geometry
configuration; once ConfigureTools is used, BindObject cannot bypass it.

```csharp
// gProbeA, gProbeB and gDrf are MarkerGeometryDefinition objects from the tool library.
// The provider here must implement future multi-geometry tracking. The current ML2 desktop
// adapter will explicitly report that this three-tool configuration is unsupported.
ToolGeometryBinding Binding(string geometryId, string bodyId) =>
    new ToolGeometryBinding(new ObjectBinding(provider.Id, bodyId, RigidPose.Identity), geometryId);

tracking.ConfigureTools(new TrackingConfiguration(
    new[] { gProbeA, gProbeB, gDrf },
    new[] {
        new TrackedToolDefinition("probe-1", "Probe A", TrackedToolRole.Probe,
            Binding(gProbeA.Id, "body/probe-1")),
        new TrackedToolDefinition("probe-2", "Probe B", TrackedToolRole.Probe,
            Binding(gProbeB.Id, "body/probe-2")),
        new TrackedToolDefinition("patient-drf", "Patient reference", TrackedToolRole.DynamicReferenceFrame,
            Binding(gDrf.Id, "body/drf"))
    }));
tracking.Start();

// Later, after fresh observations are published:
tracking.TryGetPose("probe-1", limits, out var firstProbe, out var issue);
tracking.TryGetPose("probe-2", limits, out var secondProbe, out issue);
tracking.TryGetRelativePose("probe-1", "patient-drf", limits, out var relative, out issue);
// Enumerate tracking.Configuration.Tools for the application's tool list and role filters.
```

Bindings for one logical tool can use different geometry definitions on different providers,
with explicit body-axis calibrations and the existing priority order. A single provider may not
assign the same geometry ID to two tool instances: shape alone cannot distinguish those instances.
Different IDs also do not make physically indistinguishable shapes distinguishable; the future
identification algorithm must detect ambiguous candidates or use additional identity information.

## Unity authoring and current scenes

Two independent ScriptableObject assets are available from **Create > Tracking**:

1. **Marker Geometry**: geometry ID, revision, marker centres.
2. **Tool Profile**: display name, role, reference to a geometry asset.

Each binding on TrackingApplicationHost references a tool profile. Multiple bindings for one
logical tool agree on its role/name; each provider may use a different geometry. A future UI can
use the same assets or construct definitions directly through the engine-free API.

The demo scenes select `Profiles/LegacyMl2Probe.asset`, referencing
`Profiles/LegacyMl2Geometry.asset`. This preserves the existing four-marker arrangement and
single logical `probe`. Body geometry changes are rejected by the current adapter, including
reusing the supported geometry ID with different marker coordinates. The current adapter assumes
the desktop's existing default marker model/axis configuration; there is no new geometry negotiation
over TCP. Do not alter that desktop model independently and assume the profile changes it.

The demo presenter remains a single-tool presenter. Multiple visual tool prefabs/UI controls are
application work for ArNav integration; multi-tool API queries are already independent.

## Provider and identification extension points

`IGeometryTrackingProvider.ValidateGeometryConfiguration(...)` is a side-effect-free capability
check. Providers read the accepted immutable selection through
`TrackingProviderContext.GeometryConfiguration` during startup/processing.

`IGeometryIdentifier<TFrame>` is the algorithm plug-in contract:

```csharp
IReadOnlyList<GeometryMatch> Identify(
    TFrame frame, IReadOnlyList<MarkerGeometryDefinition> geometries);
```

TFrame lets an implementation choose calibrated 2D detections, 3D detections or another input
without introducing a Magic Leap SDK dependency. Each match contains:

- Geometry ID and revision.
- Estimated rigid body pose.
- Optional quality in [0,1].
- Indices of the detections used, relative to the shared input frame.

The adapter converts estimated poses into its session's reference coordinates before publication:

```csharp
var matches = identifier.Identify(inputFrame, context.GeometryConfiguration.Geometries);
// If needed, convert each match's pose from camera coordinates to the session reference frame.
TrackingIssue issue = context.PublishIdentifiedFrame(
    sessionToken, frameSequence, captureTimeInDeviceClock, availableTimeInRuntimeClock, matches);
```

The framework resolves geometry IDs to registered provider body IDs. It never guesses a role
from the detected pose, array order or arrival order. Duplicate matches for a geometry, reused
detection indices across bodies, unknown IDs and mismatched revisions reject the new frame and
withdraw its bodies. The return value distinguishes these reasons. Algorithm exceptions must be
handled by the provider's normal failure policy; there is no placeholder algorithm returning fake poses.

## A frame with two probes and a DRF

For frame 120, suppose the future identifier returns:

```text
geometry A -> detections [0,1,2,3]   -> pose A
geometry B -> detections [4,5,6,7]   -> pose B
geometry R -> detections [8,9,10,11] -> pose R
```

1. The publication boundary verifies the current session and maps the shared capture timestamp.
2. Geometry/revision and detection assignments are validated against the selected configuration.
3. IDs are routed to `body/probe-1`, `body/probe-2`, `body/drf`.
4. TrackingCore installs the complete frame atomically under its store lock.
5. One notification fires after all observations are visible. A consumer querying relative poses
   in that callback cannot see a partially installed frame.
6. Logical queries return each pose with the same source sequence/capture timestamp. The original
   source and reference provenance remains available on relative queries.

If frame 121 contains only probe A and the DRF, probe B becomes unavailable immediately; the
other two remain usable. An empty completed frame withdraws every body from that provider.
An old frame 120 arriving later is rejected and cannot restore probe B or erase frame 121.
No new frame (for example, no SDK data) is different from an empty detection frame: normal
freshness limits apply until a provider explicitly reports a new result/loss.

For a DRF-relative query, a missing/stale DRF causes query failure. World-space probe tracking
can remain available independently. This does not assume that the DRF is motionless.

## Standalone interface scripts

Every authored tracking interface is declared in its own named script:

| Assembly | Scripts |
|---|---|
| Core | `ITrackingClock.cs`, `ITrackingReader.cs`, `ITrackingBackend.cs` |
| Runtime | `ITrackingSystem.cs`, `ITrackingProvider.cs`, `ICaptureDevice.cs`, `ICaptureOperation.cs`, `IGeometryTrackingProvider.cs`, `IGeometryIdentifier.cs` |
| Application | `ITrackingApplicationBackend.cs` |

Vendor-generated SDK/input interfaces are outside this module and are not modified.

## Validation

`GeometryFrameworkTests.cs` uses explicitly synthetic matches to test two probes + DRF,
immutable configuration, unsupported selection rollback, atomic publication, independent loss,
ambiguous assignment, old frame/session rejection and configuration replacement. It performs
no marker detection. The production ML2 provider tests validate the supported legacy profile and
reject an unsupported model. Scene checks verify profile GUIDs, separate interface files and
pipeline labels. Existing lifecycle and wire-format regression suites remain applicable.
