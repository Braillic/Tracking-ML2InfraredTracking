"""Frame/calibration contract shared by TCP input and offline/native adapters.

Arrays are borrowed read-only for a synchronous process() call. The caller must
keep pixels, capture-time sensor pose and calibration alive and unchanged until
it returns. Strided NumPy images are supported. Capture time and scheduling
deadline belong to different clocks; never subtract them.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

PIXEL_FORMAT_FLOAT32_RAW = 1
PIXEL_FORMAT_UINT8_SRGB_INTENSITY = 2

@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    fov_x: float
    fov_y: float
    k1: float
    k2: float
    p1: float
    p2: float
    k3: float

    @property
    def camera_matrix(self) -> np.ndarray:
        """OpenCV 3x3 camera matrix."""
        return np.array(((self.fx, 0.0, self.cx),
                         (0.0, self.fy, self.cy),
                         (0.0, 0.0, 1.0)), dtype=np.float64)

    @property
    def distortion_coefficients(self) -> np.ndarray:
        """OpenCV distortion vector ordered [k1, k2, p1, p2, k3]."""
        return np.array((self.k1, self.k2, self.p1, self.p2, self.k3),
                        dtype=np.float64)


@dataclass(frozen=True)
class DepthFrame:
    """One synchronized ML frame and its Unity-world sensor pose."""

    pixels: np.ndarray  # unmodified wire pixels: float32 DepthRaw or uint8 intensity
    frame_id: int
    timestamp: float
    sensor_position: np.ndarray  # xyz metres, Unity world coordinates
    sensor_rotation: np.ndarray  # xyzw quaternion, Unity world coordinates
    intrinsics: CameraIntrinsics | None  # resolved calibration for this frame
    pixel_format: int = PIXEL_FORMAT_UINT8_SRGB_INTENSITY  # wire format
    session_id: int = 0
    capture_time_ns: int = 0
    capture_realtime: float = float("nan")  # ML2 clock; NaN means conversion unavailable
    poll_start_realtime: float = 0.0
    frame_ready_realtime: float = 0.0

    @property
    def observation_time(self) -> float:
        return self.capture_time_ns * 1e-9 if self.capture_time_ns > 0 else self.timestamp


# Neutral name for future on-device adapters; old callers retain DepthFrame.
TrackingFrame = DepthFrame
