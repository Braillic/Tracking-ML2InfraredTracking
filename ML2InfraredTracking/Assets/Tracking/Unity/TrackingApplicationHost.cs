using System;
using System.Collections.Generic;
using Braillic.Tracking.Runtime;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    /// <summary>Generic Unity composition root. Contains no device SDK, capture or transport type.</summary>
    [DefaultExecutionOrder(-1000)]
    public sealed class TrackingApplicationHost : MonoBehaviour,ITrackingClock
    {
        [Serializable] private sealed class Binding
        {
            public string objectId="probe",providerId="ml2-ir",providerObjectId="marker-body";
            public Vector3 providerBodyFromLogicalPosition;
            public Vector3 providerBodyFromLogicalEuler;
        }
        [SerializeField] private TrackingProviderComponent[] providers;
        [Tooltip("Ordered provider mappings for each logical object; fallback is explicit.")]
        [SerializeField] private Binding[] bindings;
        [SerializeField] private string displayProviderId="ml2-ir";
        [SerializeField] private string displayedObjectId="probe";
        [SerializeField] private TrackedBodyPresenter presenter;
        [SerializeField] private bool startAutomatically=true,beforeRender=true;
        [SerializeField,Min(.001f)] private float maximumObservationAgeSeconds=.15f;
        [SerializeField,Min(0)] private float maximumClockUncertaintySeconds=.005f;
        private TrackingSystem system;
        private bool wantsTracking,paused;
        private string reportedError;
        public ITrackingSystem Tracking => system;
        public string ObjectId => displayedObjectId;
        public string Id { get; private set; }
        public double NowSeconds => Time.realtimeSinceStartupAsDouble;
        public TrackingState State => system?.State??TrackingState.Stopped;
        public bool TryGetDisplayFrame(out CoordinateFrame frame)
        { frame=null;return system!=null&&system.TryGetProviderFrame(displayProviderId,out frame); }

        private void Awake()
        {
            wantsTracking=startAutomatically;
            try
            {
                Id="unity-realtime/"+Guid.NewGuid().ToString("N");
                system=new TrackingSystem(this);
                if(providers==null||providers.Length==0||bindings==null)throw new InvalidOperationException("Configure tracking providers and logical object bindings.");
                foreach(var component in providers)
                {
                    if(component==null)throw new InvalidOperationException("Missing provider component.");
                    system.AddProvider(component.CreateProvider(),component.Required);
                }
                var grouped=new Dictionary<string,List<ObjectBinding>>();
                foreach(var binding in bindings)
                {
                    if(!grouped.TryGetValue(binding.objectId,out var list))grouped[binding.objectId]=list=new List<ObjectBinding>();
                    list.Add(new ObjectBinding(binding.providerId,binding.providerObjectId,TrackingCoordinates.FromUnity(
                        binding.providerBodyFromLogicalPosition,Quaternion.Euler(binding.providerBodyFromLogicalEuler))));
                }
                foreach(var pair in grouped)system.BindObject(pair.Key,pair.Value.ToArray());
                system.ObservationPublished+=PresentCurrent;
                system.ContinuityChanged+=HidePresentation;
                presenter?.Prepare();
            }
            catch(Exception error)
            { Debug.LogError("[TrackingSystem] "+error.Message,this);system?.Stop();TrackingCleanupRunner.Drain(system,true);enabled=false; }
        }
        private void OnEnable()
        { UnityEngine.Application.onBeforeRender+=BeforeRender;if(wantsTracking)StartTracking(); }
        private void OnDisable()
        { UnityEngine.Application.onBeforeRender-=BeforeRender;RequestShutdown(false);HidePresentation(); }
        private void OnDestroy()
        { RequestShutdown(true); }
        private void RequestShutdown(bool dispose)
        {
            if(system==null||system.State==TrackingState.Disposed)return;
            system.Stop();TrackingCleanupRunner.Drain(system,dispose);
        }
        private void OnApplicationPause(bool value)
        {
            paused=value;
            if(value){system?.Stop();HidePresentation();TrackingCleanupRunner.Drain(system,false);}
            else if(wantsTracking&&isActiveAndEnabled)system?.Start();
        }
        private void Update()
        {
            Pump(TrackingUpdatePhase.Update);
            if(system?.LastError!=null&&reportedError!=system.LastError)
            {reportedError=system.LastError;Debug.LogError("[TrackingSystem] "+reportedError,this);}
        }
        private void LateUpdate() { Pump(TrackingUpdatePhase.LateUpdate); }
        private void BeforeRender() { if(beforeRender&&isActiveAndEnabled)Pump(TrackingUpdatePhase.BeforeRender); }
        private void Pump(TrackingUpdatePhase phase)
        {
            system?.Pump(phase);
            if(State!=TrackingState.Running)HidePresentation();
            else {PresentCurrent();presenter?.CheckTimeout(NowSeconds);}
        }
        public void StartTracking()
        { wantsTracking=true;if(!paused&&isActiveAndEnabled)system?.Start(); }
        public void StopTracking()
        { wantsTracking=false;system?.Stop();HidePresentation(); }
        public void ResetTrackingReference()
        { system?.ResetProviderReference(displayProviderId); }
        private void PresentCurrent()
        {
            if(system==null||!TryGetDisplayFrame(out var frame))return;
            var requirements=new ObservationRequirements(maximumObservationAgeSeconds,0,maximumClockUncertaintySeconds);
            if(system.TryGetPoseInFrame(displayedObjectId,frame,requirements,out var pose,out var issue))presenter?.Present(pose);
            else if(issue==TrackingIssue.FrameMismatch||issue==TrackingIssue.ClockMismatch)HidePresentation();
        }
        private void HidePresentation() { presenter?.Hide(); }
    }
}
