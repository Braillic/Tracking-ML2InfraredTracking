using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Rolling history of sensor tracking-space poses, sampled every frame at
/// NextPredictedDisplayTime (which XrLocateSpace always accepts - unlike an
/// already-past depth frame CaptureTime, which is frequently rejected as
/// XR_ERROR_TIME_INVALID once it falls outside the runtime's pose-history
/// retention window). GetPose interpolates within this history instead of
/// re-querying XrLocateSpace for a stale timestamp.
/// </summary>
public sealed class SensorPoseHistory
{
    private struct Sample
    {
        public long Time;
        public Vector3 Position;
        public Quaternion Rotation;
    }

    private readonly List<Sample> _samples = new();
    private readonly long _maxAgeTicks; // XrTime is nanoseconds
    private readonly long _maxExtrapolationTicks;

    /// <summary>
    /// <paramref name="maxAgeSeconds"/> is how much history to retain. Outside the
    /// recorded range, TryGetPose extrapolates at constant velocity using the two
    /// nearest samples, but only up to <paramref name="maxExtrapolationSeconds"/>
    /// past them - beyond that the constant-velocity assumption gets unreliable
    /// (especially for rotation), so the query time is clamped before extrapolating,
    /// bounding how far the result can run away instead of diverging unboundedly.
    /// Samples recorded here are expected to already be smoothed (e.g. via a One
    /// Euro filter) before being passed to Record - this class only interpolates
    /// / extrapolates, it doesn't denoise.
    /// </summary>
    public SensorPoseHistory(double maxAgeSeconds = 1.0, double maxExtrapolationSeconds = 0.75)
    {
        _maxAgeTicks = (long)(maxAgeSeconds * 1e9);
        _maxExtrapolationTicks = (long)(maxExtrapolationSeconds * 1e9);
    }

    public void Record(long time, in Pose trackingPose)
    {
        if (_samples.Count > 0 && time <= _samples[^1].Time)
            return; // stale/duplicate/out-of-order sample, ignore

        _samples.Add(new Sample { Time = time, Position = trackingPose.position, Rotation = trackingPose.rotation });

        long cutoff = time - _maxAgeTicks;
        int removeCount = 0;
        while (removeCount < _samples.Count - 1 && _samples[removeCount].Time < cutoff)
            removeCount++;
        if (removeCount > 0)
            _samples.RemoveRange(0, removeCount);
    }

    public int SampleCount => _samples.Count;

    /// Nanoseconds between the oldest and newest recorded sample - how far back
    /// TryGetPose can actually interpolate right now instead of clamping.
    public long SpanTicks => _samples.Count >= 2 ? _samples[^1].Time - _samples[0].Time : 0;

    /// <summary>
    /// Interpolates within the recorded range, or extrapolates at constant
    /// velocity (clamped to maxExtrapolationSeconds) outside it.
    /// Returns false only if no samples have been recorded yet.
    /// </summary>
    public bool TryGetPose(long queryTime, out Pose trackingPose)
    {
        int n = _samples.Count;
        if (n == 0)
        {
            trackingPose = Pose.identity;
            return false;
        }

        if (n == 1)
        {
            var s = _samples[0];
            trackingPose = new Pose(s.Position, s.Rotation);
            return true;
        }

        if (queryTime <= _samples[0].Time)
        {
            long clampedTime = Math.Max(queryTime, _samples[0].Time - _maxExtrapolationTicks);
            int count = Math.Min(ExtrapolationFitSampleCount, n);
            trackingPose = FitAndExtrapolate(0, count, clampedTime);
            return true;
        }

        var last = _samples[n - 1];
        if (queryTime >= last.Time)
        {
            long clampedTime = Math.Min(queryTime, last.Time + _maxExtrapolationTicks);
            int count = Math.Min(ExtrapolationFitSampleCount, n);
            trackingPose = FitAndExtrapolate(n - count, count, clampedTime);
            return true;
        }

        for (int i = 1; i < n; i++)
        {
            if (_samples[i].Time < queryTime)
                continue;

            trackingPose = Blend(_samples[i - 1], _samples[i], queryTime);
            return true;
        }

        // Unreachable given the bounds checks above.
        trackingPose = new Pose(last.Position, last.Rotation);
        return true;
    }

    // Constant-velocity blend between two samples. Only used for in-range
    // interpolation between adjacent (closely-spaced) samples, where a 2-point
    // estimate is fine because it's bounded, not amplified.
    private static Pose Blend(Sample a, Sample b, long queryTime)
    {
        long span = b.Time - a.Time;
        float t = span > 0 ? (float)((double)(queryTime - a.Time) / span) : 0f;
        return new Pose(
            Vector3.LerpUnclamped(a.Position, b.Position, t),
            Quaternion.SlerpUnclamped(a.Rotation, b.Rotation, t));
    }

    // How many of the boundary samples to fit when extrapolating. A 2-point
    // finite-difference velocity from noisy tracking samples gets amplified over
    // the extrapolation distance; fitting a line through several samples instead
    // averages that noise out.
    private const int ExtrapolationFitSampleCount = 5;

    // Fits a constant-velocity model (position: least-squares line, rotation:
    // averaged angular velocity) over _samples[start, start+count) and evaluates
    // it at queryTime. Used only outside the recorded range.
    private Pose FitAndExtrapolate(int start, int count, long queryTime)
    {
        if (count < 2)
        {
            var s = _samples[start];
            return new Pose(s.Position, s.Rotation);
        }

        long t0 = _samples[start].Time;

        // Least-squares linear fit of position vs. time (seconds since t0, to
        // keep the regression numbers small instead of raw XrTime nanoseconds).
        double sumT = 0, sumT2 = 0;
        Vector3 sumP = Vector3.zero;
        Vector3 sumTP = Vector3.zero;
        for (int i = start; i < start + count; i++)
        {
            double dt = (_samples[i].Time - t0) / 1e9;
            sumT += dt;
            sumT2 += dt * dt;
            sumP += _samples[i].Position;
            sumTP += _samples[i].Position * (float)dt;
        }

        double n = count;
        double denom = n * sumT2 - sumT * sumT;
        Vector3 velocity;
        Vector3 position0;
        if (Math.Abs(denom) < 1e-9)
        {
            velocity = Vector3.zero;
            position0 = sumP / (float)n;
        }
        else
        {
            velocity = (sumTP * (float)n - sumP * (float)sumT) / (float)denom;
            position0 = (sumP - velocity * (float)sumT) / (float)n;
        }

        // Angular velocity: average the per-step rotation deltas across the
        // window (each as an angle-axis rate), rather than a single noisy
        // consecutive-pair delta.
        Vector3 axisSum = Vector3.zero;
        float weightSum = 0f;
        for (int i = start + 1; i < start + count; i++)
        {
            var prev = _samples[i - 1];
            var cur = _samples[i];
            float dt = (float)((cur.Time - prev.Time) / 1e9);
            if (dt <= 0f)
                continue;

            Quaternion delta = cur.Rotation * Quaternion.Inverse(prev.Rotation);
            delta.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (angleDeg > 180f)
                angleDeg -= 360f;
            if (float.IsNaN(axis.x) || axis == Vector3.zero)
                continue;

            axisSum += axis.normalized * (angleDeg / dt);
            weightSum += 1f;
        }

        double queryDt = (queryTime - t0) / 1e9;
        Vector3 extrapolatedPosition = position0 + velocity * (float)queryDt;

        Quaternion baseRotation = _samples[start].Rotation;
        Quaternion extrapolatedRotation = baseRotation;
        if (weightSum > 0f)
        {
            Vector3 angularVelocityDegPerSec = axisSum / weightSum;
            float angle = angularVelocityDegPerSec.magnitude * (float)queryDt;
            Vector3 axis = angularVelocityDegPerSec.normalized;
            extrapolatedRotation = Quaternion.AngleAxis(angle, axis) * baseRotation;
        }

        return new Pose(extrapolatedPosition, extrapolatedRotation);
    }

    public void Clear() => _samples.Clear();
}
