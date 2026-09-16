using System;
using System.Collections.Generic;
using UnityEngine;

/// Timestamped, unsmoothed sensor poses in XR tracking space.
/// Only exact samples or interpolation between nearby samples are usable.
public sealed class SensorPoseHistory
{
    public enum LookupStatus { Empty, InvalidTime, OutsideHistory, GapTooLarge, Exact, Interpolated }

    private struct Sample
    {
        public long Time;
        public Pose Pose;
    }

    private readonly List<Sample> _samples = new();
    private readonly long _maxAgeTicks; // XrTime is nanoseconds.
    private readonly long _maxInterpolationGapTicks;

    public SensorPoseHistory(double maxAgeSeconds = 2.0, double maxInterpolationGapSeconds = 0.1)
    {
        if (double.IsNaN(maxAgeSeconds) || double.IsInfinity(maxAgeSeconds) || maxAgeSeconds <= 0)
            throw new ArgumentOutOfRangeException(nameof(maxAgeSeconds));
        if (double.IsNaN(maxInterpolationGapSeconds) || double.IsInfinity(maxInterpolationGapSeconds) ||
            maxInterpolationGapSeconds <= 0)
            throw new ArgumentOutOfRangeException(nameof(maxInterpolationGapSeconds));

        _maxAgeTicks = checked((long)(maxAgeSeconds * 1e9));
        _maxInterpolationGapTicks = checked((long)(maxInterpolationGapSeconds * 1e9));
    }

    public void Record(long time, in Pose trackingPose)
    {
        if (time <= 0 || !IsValidPose(trackingPose))
            return;
        if (_samples.Count > 0 && time <= _samples[^1].Time)
            return;

        _samples.Add(new Sample
        {
            Time = time,
            Pose = new Pose(trackingPose.position, trackingPose.rotation.normalized)
        });

        long cutoff = time - _maxAgeTicks;
        int removeCount = 0;
        while (removeCount < _samples.Count && _samples[removeCount].Time < cutoff)
            removeCount++;
        if (removeCount > 0)
            _samples.RemoveRange(0, removeCount);
    }

    public int SampleCount => _samples.Count;
    public long SpanTicks => _samples.Count >= 2 ? _samples[^1].Time - _samples[0].Time : 0;

    public bool TryGetPose(long queryTime, out Pose trackingPose)
        => TryGetPose(queryTime, out trackingPose, out _);

    public bool TryGetPose(long queryTime, out Pose trackingPose, out LookupStatus status)
    {
        trackingPose = Pose.identity;
        if (queryTime <= 0)
        {
            status = LookupStatus.InvalidTime;
            return false;
        }
        if (_samples.Count == 0)
        {
            status = LookupStatus.Empty;
            return false;
        }
        if (queryTime < _samples[0].Time || queryTime > _samples[^1].Time)
        {
            status = LookupStatus.OutsideHistory;
            return false;
        }

        for (int i = 0; i < _samples.Count; i++)
        {
            Sample next = _samples[i];
            if (queryTime == next.Time)
            {
                trackingPose = next.Pose;
                status = LookupStatus.Exact;
                return true;
            }
            if (queryTime > next.Time)
                continue;

            Sample previous = _samples[i - 1];
            long gap = next.Time - previous.Time;
            if (gap > _maxInterpolationGapTicks)
            {
                status = LookupStatus.GapTooLarge;
                return false;
            }

            float t = (float)((double)(queryTime - previous.Time) / gap);
            trackingPose = new Pose(
                Vector3.Lerp(previous.Pose.position, next.Pose.position, t),
                Quaternion.Slerp(previous.Pose.rotation, next.Pose.rotation, t));
            status = LookupStatus.Interpolated;
            return true;
        }

        status = LookupStatus.OutsideHistory;
        return false;
    }

    public static bool IsValidPose(in Pose pose)
    {
        Vector3 p = pose.position;
        Quaternion q = pose.rotation;
        if (!IsFinite(p.x) || !IsFinite(p.y) || !IsFinite(p.z) ||
            !IsFinite(q.x) || !IsFinite(q.y) || !IsFinite(q.z) || !IsFinite(q.w))
            return false;
        float normSquared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        return IsFinite(normSquared) && normSquared > 1e-6f;
    }

    private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

    public void Clear() => _samples.Clear();
}
