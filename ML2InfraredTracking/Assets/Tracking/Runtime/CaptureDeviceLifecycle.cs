using System;

namespace Braillic.Tracking.Runtime
{
    public interface ICaptureOperation { bool IsCompleted { get; } bool Succeeded { get; } }
    public interface ICaptureDevice
    {
        bool OwnsResource { get; }
        bool IsStreaming { get; }
        bool Prepare(); // Permission/device discovery may be pending; never blocks.
        ICaptureOperation Configure();
        ICaptureOperation Start();
        ICaptureOperation Stop();
        void Release();
        void Read();
    }
    /// <summary>Serializes async configure/start/stop. Stop during startup waits for the outstanding
    /// operation before release; a subsequent Start cannot revive that cancelled acquisition.</summary>
    public sealed class CaptureDeviceLifecycle
    {
        private enum Operation { None, Configure, Start, Stop }
        private readonly ICaptureDevice device;
        private ICaptureOperation pending;
        private Operation operation;
        private bool desired,teardown,blocked;
        public ProviderState State { get; private set; }
        public string LastError { get; private set; }
        public bool Released => !device.OwnsResource&&pending==null;
        public CaptureDeviceLifecycle(ICaptureDevice device) { this.device=device??throw new ArgumentNullException(nameof(device)); }
        public void Start()
        {
            desired=true;
            if(State==ProviderState.Running)return;
            if(!teardown){LastError=null;blocked=false;State=ProviderState.Starting;}
        }
        public void Stop()
        { desired=false;teardown=true;blocked=false;State=Released?ProviderState.Stopped:ProviderState.Stopping; }
        public void Pump()
        {
            try { Advance(); }
            catch(Exception error)
            {
                LastError=error.Message;desired=false;teardown=true;
                // Do not forget a still-owned resource after a failing release.
                State=Released?ProviderState.Faulted:ProviderState.Stopping;
                blocked=true;
            }
        }
        private void Advance()
        {
            if(blocked)return;
            if(pending!=null)
            {
                if(!pending.IsCompleted)return;
                var completed=operation;bool success=pending.Succeeded;pending=null;operation=Operation.None;
                if(!success)
                {
                    LastError="Capture "+completed+" failed.";desired=false;teardown=true;
                    if(completed==Operation.Stop){blocked=true;State=ProviderState.Stopping;return;}
                }
                else if(completed==Operation.Start&&!teardown&&desired)State=ProviderState.Running;
                else if(completed==Operation.Configure&&!teardown&&desired){Begin(Operation.Start,device.Start());return;}
            }
            if(teardown||!desired)
            {
                if(device.OwnsResource)
                {
                    State=ProviderState.Stopping;
                    if(device.IsStreaming){Begin(Operation.Stop,device.Stop());return;}
                    device.Release();
                    if(device.OwnsResource)throw new InvalidOperationException("Capture resource was not released.");
                }
                teardown=false;State=LastError==null?ProviderState.Stopped:ProviderState.Faulted;
                // A queued restart happens only on a later pump, after all old resources are gone.
                if(desired){LastError=null;State=ProviderState.Starting;}
                return;
            }
            if(State==ProviderState.Running)
            {
                if(!device.IsStreaming)throw new InvalidOperationException("Capture device stopped unexpectedly.");
                device.Read();return;
            }
            State=ProviderState.Starting;
            if(!device.Prepare())return;
            Begin(Operation.Configure,device.Configure());
        }
        private void Begin(Operation kind,ICaptureOperation task)
        { pending=task??throw new InvalidOperationException("Capture operation missing.");operation=kind; }
    }
}
