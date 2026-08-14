using System;
using UnityEngine;

/// <summary>
/// One Euro Filter (Casiez, Roussel, Vogel 2012) - an adaptive low-pass filter
/// purpose-built for smoothing noisy real-time human-motion tracking signals
/// (position/orientation from head, hand, or controller tracking). It trades off
/// jitter suppression against lag based on how fast the signal is currently
/// moving: near-stationary, it smooths aggressively (minCutoff); moving fast, it
/// opens up and tracks closely so real motion doesn't feel laggy (beta controls
/// how quickly it opens up as speed increases).
/// </summary>
public sealed class OneEuroFilterVector3
{
    private readonly float _minCutoff;
    private readonly float _beta;
    private readonly float _dCutoff;
    private bool _initialized;
    private Vector3 _xPrev;
    private Vector3 _dxPrev;
    private double _tPrev;

    public OneEuroFilterVector3(float minCutoff = 1.0f, float beta = 0.007f, float dCutoff = 1.0f)
    {
        _minCutoff = minCutoff;
        _beta = beta;
        _dCutoff = dCutoff;
    }

    public Vector3 Filter(Vector3 x, double t)
    {
        if (!_initialized)
        {
            _xPrev = x;
            _dxPrev = Vector3.zero;
            _tPrev = t;
            _initialized = true;
            return x;
        }

        double dt = t - _tPrev;
        if (dt <= 0)
            dt = 1e-6;

        Vector3 dx = (x - _xPrev) / (float)dt;
        float dAlpha = Alpha(dt, _dCutoff);
        Vector3 edx = Vector3.Lerp(_dxPrev, dx, dAlpha);

        float cutoff = _minCutoff + _beta * edx.magnitude;
        float alpha = Alpha(dt, cutoff);
        Vector3 ex = Vector3.Lerp(_xPrev, x, alpha);

        _xPrev = ex;
        _dxPrev = edx;
        _tPrev = t;
        return ex;
    }

    private static float Alpha(double dt, float cutoff)
    {
        double tau = 1.0 / (2.0 * Math.PI * cutoff);
        return (float)(1.0 / (1.0 + tau / dt));
    }

    public void Reset() => _initialized = false;
}

/// <summary>Same idea as <see cref="OneEuroFilterVector3"/>, for rotation.</summary>
public sealed class OneEuroFilterQuaternion
{
    private readonly float _minCutoff;
    private readonly float _beta;
    private readonly float _dCutoff;
    private bool _initialized;
    private Quaternion _xPrev;
    private float _dxPrev; // smoothed angular speed, degrees/sec
    private double _tPrev;

    public OneEuroFilterQuaternion(float minCutoff = 1.0f, float beta = 0.007f, float dCutoff = 1.0f)
    {
        _minCutoff = minCutoff;
        _beta = beta;
        _dCutoff = dCutoff;
    }

    public Quaternion Filter(Quaternion x, double t)
    {
        if (!_initialized)
        {
            _xPrev = x;
            _dxPrev = 0f;
            _tPrev = t;
            _initialized = true;
            return x;
        }

        double dt = t - _tPrev;
        if (dt <= 0)
            dt = 1e-6;

        float dx = Quaternion.Angle(_xPrev, x) / (float)dt; // degrees/sec
        float dAlpha = Alpha(dt, _dCutoff);
        float edx = Mathf.Lerp(_dxPrev, dx, dAlpha);

        float cutoff = _minCutoff + _beta * edx;
        float alpha = Alpha(dt, cutoff);
        Quaternion ex = Quaternion.Slerp(_xPrev, x, alpha);

        _xPrev = ex;
        _dxPrev = edx;
        _tPrev = t;
        return ex;
    }

    private static float Alpha(double dt, float cutoff)
    {
        double tau = 1.0 / (2.0 * Math.PI * cutoff);
        return (float)(1.0 / (1.0 + tau / dt));
    }

    public void Reset() => _initialized = false;
}
