using Braillic.Tracking;
using Braillic.Tracking.Runtime;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    /// <summary>Application-owned visualization. Never use its smoothed Transform for measurements.</summary>
    public sealed class TrackedBodyPresenter : MonoBehaviour
    {
        [SerializeField] private Transform trackedTool;
        [SerializeField, Range(0, 1)] private float positionFollow = 1;
        [SerializeField, Range(0, 1)] private float rotationFollow = 1;
        [SerializeField, Min(0)] private float hideAfterNoPoseSeconds = .65f;
        [Tooltip("Every two seconds, compare incoming resolved poses with presented movement. Does not smooth or change measurements.")]
        [SerializeField] private bool logPoseDiagnostics;
        [SerializeField] private bool createMarkerSpheresOnFirstPose;
        [SerializeField] private Vector3[] markerLocalPositions;
        [SerializeField] private float markerSphereRadius = .004f;
        [SerializeField] private Color markerSphereColor = Color.red;
        private TrackingSession lastSession;
        private ulong lastSequence;
        private long lastRevision;
        private double lastCapture = double.NegativeInfinity;
        private bool prepared;
        private Material ownedMaterial;
        private bool diagnosticPrevious;
        private Vector3 diagnosticRawPosition, diagnosticPresentedPosition;
        private Quaternion diagnosticRawRotation, diagnosticPresentedRotation;
        private float maximumRawPositionStep, maximumRawAngleStep, maximumPresentedPositionStep, maximumPresentedAngleStep;
        private int diagnosticSamples, diagnosticDuplicateCalls, diagnosticSnaps, diagnosticExternalChanges;
        private double nextDiagnosticTime;

        public void Prepare()
        {
            if (prepared) return;
            if (trackedTool == null)
            {
                var root = new GameObject("TrackedTool");
                trackedTool = root.transform;
                trackedTool.SetParent(transform, false);
            }
            trackedTool.gameObject.SetActive(false);
            if (createMarkerSpheresOnFirstPose && markerLocalPositions != null)
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
                if (shader != null)
                {
                    ownedMaterial = new Material(shader);
                    if (ownedMaterial.HasProperty("_BaseColor")) ownedMaterial.SetColor("_BaseColor", markerSphereColor);
                    if (ownedMaterial.HasProperty("_Color")) ownedMaterial.SetColor("_Color", markerSphereColor);
                }
                for (int i = 0; i < markerLocalPositions.Length; i++)
                {
                    var marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    marker.name = "Marker_" + i;
                    marker.transform.SetParent(trackedTool, false);
                    marker.transform.localPosition = markerLocalPositions[i];
                    marker.transform.localScale = Vector3.one * (markerSphereRadius * 2);
                    var collider = marker.GetComponent<Collider>();
                    if (collider != null) Destroy(collider);
                    if (ownedMaterial != null) marker.GetComponent<Renderer>().sharedMaterial = ownedMaterial;
                }
            }
            prepared = true;
        }

        public void Present(ResolvedTrackingPose observation)
        {
            if (!prepared || !isActiveAndEnabled || observation.Source.Session == null) return;
            CheckExternalPoseChange();
            if (ReferenceEquals(lastSession, observation.Source.Session) && lastSequence == observation.Source.Sequence && lastRevision == observation.Revision)
            { if(logPoseDiagnostics)diagnosticDuplicateCalls++;return; }
            bool first = !ReferenceEquals(lastSession, observation.Source.Session) || !trackedTool.gameObject.activeSelf;
            Vector3 p = TrackingCoordinates.ToUnity(observation.FrameFromObject.Position);
            Quaternion q = TrackingCoordinates.ToUnity(observation.FrameFromObject.Rotation);
            // Reacquisition snaps; interpolation from an old coordinate epoch would make a ghost.
            trackedTool.SetPositionAndRotation(first ? p : Vector3.Lerp(trackedTool.position, p, positionFollow),
                first ? q : Quaternion.Slerp(trackedTool.rotation, q, rotationFollow));
            trackedTool.gameObject.SetActive(true);
            if(logPoseDiagnostics)
            {
                diagnosticSamples++;
                if(first)diagnosticSnaps++;
                if(diagnosticPrevious&&!first)
                {
                    maximumRawPositionStep=Mathf.Max(maximumRawPositionStep,Vector3.Distance(p,diagnosticRawPosition));
                    maximumRawAngleStep=Mathf.Max(maximumRawAngleStep,Quaternion.Angle(q,diagnosticRawRotation));
                    maximumPresentedPositionStep=Mathf.Max(maximumPresentedPositionStep,Vector3.Distance(trackedTool.position,diagnosticPresentedPosition));
                    maximumPresentedAngleStep=Mathf.Max(maximumPresentedAngleStep,Quaternion.Angle(trackedTool.rotation,diagnosticPresentedRotation));
                }
                diagnosticRawPosition=p;diagnosticRawRotation=q;
                diagnosticPresentedPosition=trackedTool.position;diagnosticPresentedRotation=trackedTool.rotation;
                diagnosticPrevious=true;
            }
            lastSession = observation.Source.Session; lastSequence = observation.Source.Sequence;
            lastCapture = observation.Source.CaptureSeconds; lastRevision = observation.Revision;
        }

        public void CheckTimeout(double now)
        {
            CheckExternalPoseChange();
            if (double.IsNaN(now) || double.IsInfinity(now) || now < lastCapture ||
                now - lastCapture > hideAfterNoPoseSeconds) Hide();
        }

        public void Hide()
        {
            if (trackedTool != null) trackedTool.gameObject.SetActive(false);
            lastSession = null; lastCapture = double.NegativeInfinity;
            diagnosticPrevious=false;
        }
        private void CheckExternalPoseChange()
        {
            if(!logPoseDiagnostics||!diagnosticPrevious||trackedTool==null||!trackedTool.gameObject.activeSelf)return;
            if(Vector3.Distance(trackedTool.position,diagnosticPresentedPosition)>.00001f ||
               Quaternion.Angle(trackedTool.rotation,diagnosticPresentedRotation)>.1f)
            {
                diagnosticExternalChanges++;
                // Count each observed external change once, including parent movement.
                diagnosticPresentedPosition=trackedTool.position;diagnosticPresentedRotation=trackedTool.rotation;
            }
        }
        private void Update()
        {
            double now=Time.realtimeSinceStartupAsDouble;
            if(!logPoseDiagnostics||now<nextDiagnosticTime)return;
            nextDiagnosticTime=now+2;
            Debug.Log($"[TrackingPose] samples={diagnosticSamples} duplicate_calls_ignored={diagnosticDuplicateCalls} " +
                $"snaps={diagnosticSnaps} external_transform_changes={diagnosticExternalChanges} " +
                $"max_raw_step_mm={(maximumRawPositionStep*1000):F3} max_raw_step_deg={maximumRawAngleStep:F3} " +
                $"max_presented_step_mm={(maximumPresentedPositionStep*1000):F3} max_presented_step_deg={maximumPresentedAngleStep:F3} " +
                $"position_follow={positionFollow:F3} rotation_follow={rotationFollow:F3} frame={lastSequence}",this);
            diagnosticSamples=diagnosticDuplicateCalls=diagnosticSnaps=diagnosticExternalChanges=0;
            maximumRawPositionStep=maximumRawAngleStep=maximumPresentedPositionStep=maximumPresentedAngleStep=0;
        }
        private void OnDisable() { Hide(); }
        private void OnDestroy() { if (ownedMaterial != null) Destroy(ownedMaterial); }
    }
}
