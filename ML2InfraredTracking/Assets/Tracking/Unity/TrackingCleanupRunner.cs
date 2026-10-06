using System.Collections.Generic;
using Braillic.Tracking.Runtime;
using UnityEngine;

namespace Braillic.Tracking.Unity
{
    /// <summary>Allows asynchronous SDK shutdown to finish after the host is disabled or its scene unloads.</summary>
    public sealed class TrackingCleanupRunner : MonoBehaviour
    {
        private static TrackingCleanupRunner instance;
        private static bool quitting;
        private readonly HashSet<CaptureDeviceLifecycle> captures=new HashSet<CaptureDeviceLifecycle>();
        private readonly Dictionary<TrackingSystem,bool> systems=new Dictionary<TrackingSystem,bool>();
        private readonly List<CaptureDeviceLifecycle> completedCaptures=new List<CaptureDeviceLifecycle>();
        private readonly List<TrackingSystem> completedSystems=new List<TrackingSystem>();
        private static TrackingCleanupRunner Instance
        {
            get
            {
                if(instance==null&&!quitting)
                {var go=new GameObject("Tracking resource cleanup");DontDestroyOnLoad(go);instance=go.AddComponent<TrackingCleanupRunner>();}
                return instance;
            }
        }
        public static void Drain(CaptureDeviceLifecycle lifecycle)
        { if(!quitting&&!lifecycle.Released)Instance.captures.Add(lifecycle); }
        public static void Drain(TrackingSystem system,bool dispose)
        { if(!quitting&&system!=null) { var runner=Instance;runner.systems.TryGetValue(system,out bool old);runner.systems[system]=dispose||old; } }
        private void Update()
        {
            completedCaptures.Clear();completedSystems.Clear();
            foreach(var capture in captures)
            {
                // A restarted provider owns reads again; cleanup must never become a second poller.
                if(capture.State==ProviderState.Starting||capture.State==ProviderState.Running)
                {completedCaptures.Add(capture);continue;}
                capture.Pump();if(capture.Released)completedCaptures.Add(capture);
            }
            foreach(var pair in systems)
            {
                var system=pair.Key;
                if(system.State==TrackingState.Disposed){completedSystems.Add(system);continue;}
                // A re-enabled host owns the update loop again.
                if(!pair.Value&&(system.State==TrackingState.Running||system.State==TrackingState.Starting))
                {completedSystems.Add(system);continue;}
                system.Pump(TrackingUpdatePhase.Update);
                if(system.ShutdownComplete)
                {if(pair.Value)system.Dispose();completedSystems.Add(system);}
            }
            foreach(var capture in completedCaptures)captures.Remove(capture);
            foreach(var system in completedSystems)systems.Remove(system);
        }
        private void OnApplicationQuit() { quitting=true; }
    }
}
