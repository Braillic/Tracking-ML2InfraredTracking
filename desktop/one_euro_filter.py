"""One Euro Filter (Casiez, Roussel, Vogel 2012) for smoothing the noisy
world-space marker pose (world_T_object) before it's sent to the headset.

Port of the C# OneEuroFilterVector3/OneEuroFilterQuaternion used on-device for
the sensor pose (see ML2InfraredTracking/Assets/ML2IRTracking/OneEuroFilter.cs) -
same algorithm, applied here to the marker pose instead, since PnP noise in
cam_T_object is unfiltered otherwise and passes straight through to
world_T_object = world_T_sensor * cam_T_object.
"""

from __future__ import annotations

import math

import numpy as np


def _alpha(dt: float, cutoff: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / dt)


class OneEuroFilterVec3:
    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.007, d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._initialized = False
        self._x_prev = np.zeros(3, dtype=np.float64)
        self._dx_prev = np.zeros(3, dtype=np.float64)
        self._t_prev = 0.0

    def filter(self, x: np.ndarray, t: float) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64).reshape(3)
        if not self._initialized:
            self._x_prev = x
            self._dx_prev = np.zeros(3, dtype=np.float64)
            self._t_prev = t
            self._initialized = True
            return x.copy()

        dt = t - self._t_prev
        if dt <= 0:
            dt = 1e-6

        dx = (x - self._x_prev) / dt
        d_alpha = _alpha(dt, self.d_cutoff)
        edx = self._dx_prev + d_alpha * (dx - self._dx_prev)

        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(edx))
        alpha = _alpha(dt, cutoff)
        ex = self._x_prev + alpha * (x - self._x_prev)

        self._x_prev = ex
        self._dx_prev = edx
        self._t_prev = t
        return ex

    def reset(self) -> None:
        self._initialized = False


class OneEuroFilterQuat:
    """xyzw quaternion, filtered via slerp (angular-speed-adaptive)."""

    def __init__(self, min_cutoff: float = 1.0, beta: float = 0.007, d_cutoff: float = 1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._initialized = False
        self._x_prev = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        self._dx_prev = 0.0
        self._t_prev = 0.0

    def filter(self, x: np.ndarray, t: float) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64).reshape(4)
        x = x / (np.linalg.norm(x) or 1.0)
        if not self._initialized:
            self._x_prev = x
            self._dx_prev = 0.0
            self._t_prev = t
            self._initialized = True
            return x.copy()

        dt = t - self._t_prev
        if dt <= 0:
            dt = 1e-6

        angle_deg = _quat_angle_deg(self._x_prev, x)
        dx = angle_deg / dt
        d_alpha = _alpha(dt, self.d_cutoff)
        edx = self._dx_prev + d_alpha * (dx - self._dx_prev)

        cutoff = self.min_cutoff + self.beta * edx
        alpha = _alpha(dt, cutoff)
        ex = _slerp(self._x_prev, x, alpha)

        self._x_prev = ex
        self._dx_prev = edx
        self._t_prev = t
        return ex

    def reset(self) -> None:
        self._initialized = False


def _quat_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot = float(np.clip(np.abs(np.dot(a, b)), -1.0, 1.0))
    return math.degrees(2.0 * math.acos(dot))


def _slerp(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    dot = min(dot, 1.0)

    if dot > 0.9995:
        result = a + t * (b - a)
        return result / (np.linalg.norm(result) or 1.0)

    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta_0 = math.sin(theta_0)
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return s0 * a + s1 * b
