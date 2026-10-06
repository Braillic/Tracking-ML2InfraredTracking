from pathlib import Path
import re
import sys

project=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[2]/'ML2InfraredTracking'
assets=project/'Assets'
def guid(rel): return re.search(r'guid: (\w+)',(assets/(rel+'.meta')).read_text()).group(1)
host=guid('Tracking/Unity/TrackingApplicationHost.cs')
presenter=guid('Tracking/Unity/TrackedBodyPresenter.cs')
receiver=guid('Tracking/Implementation/ML2/Runtime/PoseEstimateTcpServer.cs')
sender=guid('Tracking/Implementation/ML2/Runtime/DepthFrameTcpServer.cs')
capture=guid('Tracking/Implementation/ML2/Runtime/ML2DepthRawStream.cs')
sensor=guid('Tracking/Implementation/ML2/Runtime/DepthSensorAPI.cs')
provider=guid('Tracking/Implementation/ML2/Ml2TrackingProviderComponent.cs')
trace=guid('ProbeTracing/ProbeSurfaceTrace.cs')
checks=0
def check(ok,label):
    global checks
    assert ok,label
    checks+=1
for name in ['IRtoolTracking_demo','ProbeSurfaceTracing_demo']:
    text=(assets/'Scenes'/(name+'.unity')).read_text(encoding='utf-8-sig')
    docs=re.split(r'(?=^--- !u!)',text,flags=re.M)[1:]
    ids=[re.search(r'&([0-9]+)',d).group(1) for d in docs]
    check(len(ids)==len(set(ids)),name+': duplicate file IDs')
    by_id=dict(zip(ids,docs))
    def find(g):
        found=[(i,d) for i,d in by_id.items() if 'guid: '+g+',' in d]
        check(len(found)==1,name+': exactly one '+g)
        return found[0]
    hid,h=find(host);pid,p=find(presenter);rid,r=find(receiver);sid,s=find(sender);cid,c=find(capture)
    did,d=find(sensor);vid,v=find(provider)
    for key,fid in [('sender',sid),('receiver',rid),('capture',cid),('sensor',did)]:
        check(f'  {key}: {{fileID: {fid}}}' in v,name+': provider '+key+' binding')
    check(f'  streamVisualizer: {{fileID: {cid}}}' in d,name+': sensor output binding')
    check(f'  providers:\n  - {{fileID: {vid}}}' in h,name+': generic host owns provider')
    check(f'  presenter: {{fileID: {pid}}}' in h,name+': presentation binding')
    check('  - objectId: probe\n    providerId: ml2-ir\n    providerObjectId: marker-body' in h,name+': logical object binding')
    check('  startAutomatically: 0' in r and '  startAutomatically: 0' in s,name+': one lifecycle owner')
    check('  startAutomatically: 1' in h,name+': host starts demo')
    for fid,d in [(hid,h),(pid,p),(vid,v)]:
        go=re.search(r'm_GameObject: {fileID: (\d+)}',d).group(1)
        check(f'  - component: {{fileID: {fid}}}' in by_id[go],name+': component attached to object')
    target=re.search(r'trackedTool: {fileID: (\d+)}',p).group(1)
    check(target in by_id,name+': presenter uses existing tool')
    check(f'  trackedTool: {{fileID: {target}}}' in r,name+': visual target preserved')
    if name.startswith('Probe'):
        _,t=find(trace)
        check(f'  trackingApplication: {{fileID: {hid}}}' in t,'trace reads application')
        check(f'  trackedToolRoot: {{fileID: {target}}}' in t,'trace retains same local calibration frame')
        check('  tracePointPrefab: {fileID: 7438540521478375765, guid: 8fe3f5a70613a0e45be8d37b7017168b,' in t,'trace prefab retained')
check('PoseEstimateTcpServer' not in (assets/'ProbeTracing/ProbeSurfaceTrace.cs').read_text(),'trace independent of TCP receiver')
check('DepthFrameTcpServer' not in (assets/'ProbeTracing/ProbeSurfaceTrace.cs').read_text(),'trace independent of TCP sender')
for file in (assets/'Tracking/Unity').glob('*.cs'):
    check(not any(word in file.read_text() for word in ['DepthSensorAPI','ML2DepthRawStream','PoseEstimateTcpServer','MagicLeap.OpenXR']),file.name+': Unity host independent of ML2')
print(f'PASS: {checks} scene wiring and application dependency checks')
