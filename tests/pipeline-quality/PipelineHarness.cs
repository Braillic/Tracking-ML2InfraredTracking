// Test-only clock, math, and Unity value types; the runner inserts production methods.
using System;
using System.IO;
using System.Threading;
using System.Linq;
using UnityEngine;
namespace UnityEngine {
    public struct Vector3 { public float x,y,z; public Vector3(float a,float b,float c){x=a;y=b;z=c;} }
    public struct Quaternion { public float x,y,z,w; public Quaternion(float a,float b,float c,float d){x=a;y=b;z=c;w=d;}
        public static Quaternion identity => new Quaternion(0,0,0,1); }
    public struct Pose { public Vector3 position; public Quaternion rotation;
        public static Pose identity => new Pose{rotation=Quaternion.identity}; }
    public static class Time { public static double realtimeSinceStartupAsDouble; }
    public static class Debug { public static void Log(string text) {} }
    public static class Mathf {
        public static float Max(float a,float b)=>Math.Max(a,b);
        public static float Clamp01(float v)=>Math.Max(0,Math.Min(1,v));
        public static float Pow(float v,float p)=>(float)Math.Pow(v,p);
        public static int RoundToInt(float v)=>(int)Math.Round(v,MidpointRounding.ToEven);
        public static int Clamp(int v,int a,int b)=>Math.Max(a,Math.Min(b,v));
    }
}
public struct DepthCameraIntrinsics {}
public class PoseEstimateTcpServer {
    public static PoseEstimateTcpServer ActiveServer=new PoseEstimateTcpServer();
    public int invalidations;
    public void InvalidateTrackingSession(){invalidations++;}
}
__TIMING__
__WINDOW__
public static class ClockHarness { __CLOCK__ }
public class SenderHarness {
    public enum PipelineMode { LegacyFloat32=1, Ml2UInt8=2 }
    public const int ProtocolVersion=5, HeaderSize=112;
    public const uint PixelFormatFloat32Raw=1, PixelFormatUInt8SrgbIntensity=2;
    public object _frameLock=new object();
    public object _latencyLock=new object();
    public const int LatencyRingSize=128;
    public double[] _latencySubmitRealtime=new double[LatencyRingSize];
    public bool _running=true, _clientConnected=true, convertLinearToSrgb=true;
    public float maximumFramesPerSecond=0, rawMin=5, rawMax=3000;
    public double _nextSubmitTime,_pendingTimestamp;
    public float[] _pendingRaw,_spareRaw;
    public float _pendingRawMin,_pendingRawMax;
    public bool _pendingSrgb;
    public CaptureTiming _pendingTiming;
    public ulong _pendingSessionId, SessionId=99;
    public Pose _pendingSensorPose,_statusPose;
    public DepthCameraIntrinsics? _pendingIntrinsics,_statusIntrinsics;
    public int _pendingWidth,_pendingHeight,_lastWidth,_lastHeight;
    public ulong _pendingFrameNumber,_nextFrameNumber;
    public long _overwrittenFrames,_submittedFrameCount,_rateDroppedFrames;
    public PipelineMode pipelineMode=PipelineMode.Ml2UInt8,_pendingPipeline,_lastSubmittedPipeline;
    public static string PipelineLabel(PipelineMode p)=>p.ToString();
    public void RecordLatencySubmit(ulong id,double time,PipelineMode mode,CaptureTiming timing) {}
    public float[] LeasePending() {lock(_frameLock){var result=_pendingRaw; _pendingRaw=null; return result;} }
    public static byte[] Header(PipelineMode mode,int bytes,CaptureTiming timing) {
        var data=new byte[HeaderSize]; WriteHeader(data,3,1,mode,bytes,7,123,Pose.identity,0,99,timing); return data;
    }
__DEPTH__
}
public static class ParserHarness {
    private const int PacketSize=80;
    public static bool Parse(byte[] data,out ulong id) => TryParsePacket(data,out id,out _,out _,out _,out _,out _,out _,out _,out _,out _);
__PARSE__
}
public static class PipelineChecks {
    static int count;
    static void Check(bool ok,string name){if(!ok)throw new Exception(name);count++;}
    // Frozen pre-change mapping; detects any numerical change while moving work to the sender.
    static byte LegacyMap(float raw,float min,float max,bool srgb) {
        if(float.IsNaN(raw)||float.IsInfinity(raw))return 0;
        float v=Mathf.Clamp01((raw-min)/Mathf.Max(max-min,1e-6f));
        if(srgb)v=v<=.0031308f?v*12.92f:1.055f*Mathf.Pow(v,1f/2.4f)-.055f;
        return (byte)Mathf.Clamp(Mathf.RoundToInt(v*255),0,255);
    }
    public static void Run(string output) {
        var random=new Random(42); var raw=new float[100008];
        for(int i=0;i<100000;i++)raw[i]=(float)(random.NextDouble()*5000-1000);
        raw[100000]=float.NaN; raw[100001]=float.PositiveInfinity;raw[100002]=float.NegativeInfinity;
        raw[100003]=5;raw[100004]=3000;raw[100005]=5+2995*.0031308f;
        raw[100006]=0;raw[100007]=float.MaxValue;
        var bytes=new byte[raw.Length];
        foreach(bool srgb in new[]{false,true}) {
            SenderHarness.ConvertRawToUInt8Srgb(raw,bytes,raw.Length,5,3000,srgb);
            Check(raw.Select((v,i)=>bytes[i]==LegacyMap(v,5,3000,srgb)).All(v=>v),"100008-value mapping matches baseline");
        }
        Time.realtimeSinceStartupAsDouble=1;
        var sender=new SenderHarness(); var input=new float[]{5,123,3000};
        var timing=new CaptureTiming(1234567890123456,4.1,4.11,4.12);
        sender.SubmitFrame(input,3,1,Pose.identity,null,timing);
        var lease=sender.LeasePending(); input[1]=999;
        Check(lease[1]==123,"producer input may be reused after submit");
        for(int i=0;i<100;i++){input[1]=i; sender.SubmitFrame(input,3,1,Pose.identity,null,timing);}
        Check(lease[1]==123,"worker lease is never overwritten by capture");
        Check(sender._pendingRaw[1]==99 && sender._overwrittenFrames==99,"one-slot newest pending frame");
        sender.rawMin=100; sender.convertLinearToSrgb=false;
        Check(sender._pendingRawMin==5 && sender._pendingSrgb,"mapping settings snapshot belongs to frame");
        sender._latencySubmitRealtime[0]=1;
        sender.BeginCaptureEpoch();
        Check(sender.SessionId!=99 && sender._pendingRaw==null && sender._latencySubmitRealtime[0]==0,
              "origin reset invalidates old session, pending frames, and timing ring");
        Check(PoseEstimateTcpServer.ActiveServer.invalidations==1,"origin reset hides old display pose");
        sender._clientConnected=false;
        Check(!sender.CanSubmitFrame(),"disconnected frame preparation skipped");
        sender._clientConnected=true;sender.maximumFramesPerSecond=30;sender._nextSubmitTime=2;
        Check(!sender.CanSubmitFrame(),"rate-limited preparation skipped");
        Check(Math.Abs(ClockHarness.Map(9900000000,10000000000,5,5.001)-4.9005)<1e-10,"XR/system/Unity clock bridge");
        Check(double.IsNaN(ClockHarness.Map(0,100,1,1)),"failed XR conversion stays unavailable");
        Check(double.IsNaN(ClockHarness.Map(200,100,1,1)),"future capture cannot produce negative age");
        Check(double.IsNaN(ClockHarness.Map(90,100,1,1.01)),"wide bridge sampling interval rejected");
        var window=new TimingWindow();
        Check(double.IsNaN(window.Percentile(.95)),"empty latency window unavailable");
        for(int i=1;i<=100;i++)window.Add(i);
        window.Add(double.NaN);window.Add(-1);
        Check(window.Percentile(.50)==50 && window.Percentile(.95)==95 && window.Percentile(.99)==99,"nearest-rank percentiles");
        for(int i=101;i<=1000;i++)window.Add(i);
        Check(window.Percentile(0)==745 && window.Percentile(1)==1000,"rolling window bounded to 256 samples");
        var packet=new byte[80];
        using(var writer=new BinaryWriter(new MemoryStream(packet))){writer.Write(new byte[]{77,76,50,80});writer.Write((ushort)4);writer.Write((ushort)88);writer.Write((ulong)7);}
        Check(ParserHarness.Parse(packet,out ulong id)&&id==7,"v4 pose base parses before session suffix");
        packet[6]=80;
        Check(!ParserHarness.Parse(packet,out _),"invalid version/size pair rejected");
        input=new float[]{5,123,3000};
        foreach(var mode in new[]{SenderHarness.PipelineMode.LegacyFloat32,SenderHarness.PipelineMode.Ml2UInt8}){
            var payload=new byte[mode==SenderHarness.PipelineMode.LegacyFloat32?12:3];
            if(payload.Length==12)Buffer.BlockCopy(input,0,payload,0,12);
            else SenderHarness.ConvertRawToUInt8Srgb(input,payload,3,5,3000,true);
            var header=SenderHarness.Header(mode,payload.Length,timing);
            Check(BitConverter.ToInt64(header,80)==1234567890123456,"wire retains exact capture nanoseconds");
            Check(BitConverter.ToUInt64(header,72)==99 && BitConverter.ToUInt16(header,6)==112,"wire session/header size");
            File.WriteAllBytes(Path.Combine(output,payload.Length==12?"float.bin":"uint8.bin"),header.Concat(payload).ToArray());
        }
        Console.WriteLine($"Passed {count} pipeline checks, including 200016 mapping comparisons.");
    }
}
