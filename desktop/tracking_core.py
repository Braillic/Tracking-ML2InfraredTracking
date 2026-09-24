"""Synchronous image-to-world-pose reference core for desktop and native ports.

No sockets, GUI, recordings, sleeps or application scheduling. One instance per
ordered stream, owned by one worker. Input arrays are borrowed read-only until
process() returns; output image storage can alias the input. Source observation
time drives motion gates. An optional deadline uses the caller's local monotonic
clock and limits processing age, not capture age across devices.
"""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from typing import Callable

import cv2
import numpy as np

from marker_detection import detect_marker_centers, raw_to_uint8_srgb
from marker_pose import MarkerPoseTracker, PoseEstimate, TEST_MARKER_COORDS
from pose_geometry import (pose_camera_matrix, unity_trs_matrix,
                           opencv_camera_pose_to_unity_world)
from tracking_types import (DepthFrame, CameraIntrinsics, PIXEL_FORMAT_FLOAT32_RAW,
                            PIXEL_FORMAT_UINT8_SRGB_INTENSITY)


@dataclass(frozen=True)
class DetectionConfig:
    threshold_method: str = 'fixed'
    fixed_threshold: float = 200.
    top_p: tuple[float, float] = (100., 90.)
    filter_geometry: bool = True
    min_area: int = 2
    max_area: int = 2000
    max_aspect: float = 6.
    max_nearest_neighbor_px: float | None = None
    max_cluster_span_px: float | None = None
    min_cluster_size: int = 3
    max_geometry_ratio_error: float = .35
    max_markers: int = 8


@dataclass(frozen=True)
class TrackingConfig:
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    view: str = 'unity-raw'  # legacy turbo changes detection intensity, not just UI
    raw_min: float = 5.
    raw_max: float = 3000.
    unity_color_space: str = 'linear'
    near: float = .2
    far: float = 5.
    depth_vertically_flipped: bool = True
    convert_object_axes: bool = False

    def __post_init__(self):
        if not self.depth_vertically_flipped and not self.convert_object_axes:
            raise ValueError('Unflipped input requires object-axis conversion (proper rotation)')
        if self.view not in ('unity-raw', 'turbo') or self.unity_color_space not in ('linear', 'gamma'):
            raise ValueError('Unsupported image mapping')
        if not np.isfinite([self.raw_min, self.raw_max, self.near, self.far]).all():
            raise ValueError('Image mapping limits must be finite')
        if self.raw_max <= self.raw_min or self.far <= self.near:
            raise ValueError('Image mapping limits must be increasing')
        d = self.detection
        if d.threshold_method not in ('fixed', 'percentile'):
            raise ValueError('Unsupported threshold method')
        if (not np.isfinite([d.fixed_threshold, *d.top_p, d.max_aspect, d.max_geometry_ratio_error]).all()
                or len(d.top_p) != 2 or not 0 <= d.top_p[1] <= 100
                or d.min_area < 1 or d.max_area < d.min_area or d.max_aspect < 1
                or d.max_markers < 3 or d.min_cluster_size < 1 or d.max_geometry_ratio_error < 0):
            raise ValueError('Invalid detection configuration')


@dataclass(frozen=True)
class StageTimings:
    prepare_ms: float = 0.
    detect_ms: float = 0.
    pnp_ms: float = 0.
    world_ms: float = 0.
    total_ms: float = 0.


@dataclass(frozen=True)
class PoseSolution:
    estimate: PoseEstimate
    camera_matrix: np.ndarray | None
    position: tuple[float, float, float] | None = None
    rotation: tuple[float, float, float, float] | None = None
    pnp_ms: float = 0.
    world_ms: float = 0.
    ready_time: float = 0.
    reason: str = 'no_pose'


@dataclass(frozen=True)
class TrackingResult:
    frame_id: int
    session_id: int
    capture_time_ns: int
    observation_time: float
    state: str
    solution: PoseSolution
    centers: tuple[tuple[float, float], ...] = ()
    cutoff: float = 0.
    intensity: np.ndarray | None = None
    prepared_image: np.ndarray | None = None
    timings: StageTimings = field(default_factory=StageTimings)

    @property
    def ok(self):
        return self.solution.estimate.ok

    @property
    def reason(self):
        return self.solution.reason


def solve_observation(tracker, centers, frame, intrinsics, *,
                      depth_vertically_flipped=True, convert_object_axes=False,
                      deadline=None, clock=None, camera_matrix=None, distortion=None):
    """Common centers-to-world-pose stage; also supports existing center callers."""
    clock = clock or time.perf_counter
    K = camera_matrix if camera_matrix is not None else pose_camera_matrix(
        intrinsics, frame.pixels.shape[0], depth_vertically_flipped=depth_vertically_flipped)
    if K is None:
        tracker.reset()
        return PoseSolution(PoseEstimate.failed(), None, reason='missing_calibration')
    start = clock()
    if deadline is not None and start > deadline:
        return PoseSolution(PoseEstimate.failed(), K, reason='expired_before_pose')
    camera_world = unity_trs_matrix(frame.sensor_position, frame.sensor_rotation)
    if not depth_vertically_flipped or convert_object_axes:
        camera_world[:3, 1] *= -1
    estimate = tracker.estimate(centers, K,
        distortion if distortion is not None else intrinsics.distortion_coefficients,
        observation_time=frame.observation_time, camera_world_transform=camera_world)
    end = clock()
    pnp_ms = (end - start) * 1000.
    if deadline is not None and end > deadline:
        tracker.reset()  # an unpublished observation must not become the next prior
        return PoseSolution(PoseEstimate.failed(), K, pnp_ms=pnp_ms,
                            ready_time=end, reason='expired_during_pose')
    if not (estimate.ok and estimate.rvec is not None and estimate.tvec is not None):
        return PoseSolution(estimate, K, pnp_ms=pnp_ms, ready_time=end,
                            reason='acquiring' if tracker.state == 'acquiring' else 'no_pose')
    prep_start = clock()
    try:
        position, rotation = opencv_camera_pose_to_unity_world(
            estimate.rvec, estimate.tvec, frame.sensor_position, frame.sensor_rotation,
            depth_vertically_flipped=depth_vertically_flipped, convert_object_axes=convert_object_axes)
        if (not np.isfinite((*position, *rotation, estimate.confidence)).all()
                or not 0 <= estimate.confidence <= 1):
            raise ValueError('Invalid world pose')
    except (ValueError, cv2.error):
        tracker.reset()
        return PoseSolution(PoseEstimate.failed(), K, pnp_ms=pnp_ms, reason='invalid_world_pose')
    ready = clock()
    if deadline is not None and ready > deadline:
        tracker.reset()
        return PoseSolution(PoseEstimate.failed(), K, pnp_ms=pnp_ms,
                            world_ms=(ready-prep_start)*1000., ready_time=ready,
                            reason='expired_during_world')
    return PoseSolution(estimate, K, position, rotation, pnp_ms,
                        (ready-prep_start)*1000., ready, 'accepted')


class TrackingPipeline:
    def __init__(self, config=None, tracker=None, clock: Callable[[], float] | None = None):
        self.config = config or TrackingConfig()
        self.tracker = tracker if tracker is not None else MarkerPoseTracker(TEST_MARKER_COORDS.copy())
        self.clock = clock or time.perf_counter
        self.reset()

    def reset(self):
        self.tracker.reset()
        self._context = None
        self._last_frame_id = None
        self._last_observation = None
        self._K = self._distortion = None
        self._detection_config = None
        self._detection_kwargs = None

    def process(self, frame: DepthFrame, *, deadline: float | None = None) -> TrackingResult:
        start = self.clock()

        def rejected(reason):
            return TrackingResult(frame.frame_id, frame.session_id, frame.capture_time_ns,
                frame.observation_time, self.tracker.state,
                PoseSolution(PoseEstimate.failed(), self._K, reason=reason),
                timings=StageTimings(total_ms=(self.clock()-start)*1000.))

        if deadline is not None:
            if not np.isfinite(deadline):
                raise ValueError('Deadline must be a finite local monotonic time')
            if start > deadline:
                return rejected('expired_before_processing')
        pixels = frame.pixels
        expected = {PIXEL_FORMAT_FLOAT32_RAW: np.dtype('<f4'),
                    PIXEL_FORMAT_UINT8_SRGB_INTENSITY: np.dtype('uint8')}.get(frame.pixel_format)
        if (expected is None or pixels.ndim != 2 or not pixels.size or pixels.dtype != expected):
            return rejected('invalid_image')
        if (frame.sensor_position.shape != (3,) or frame.sensor_rotation.shape != (4,)
                or not np.isfinite(frame.sensor_position).all() or not np.isfinite(frame.sensor_rotation).all()
                or not np.isclose(np.linalg.norm(frame.sensor_rotation), 1., atol=1e-4)
                or not np.isfinite(frame.observation_time) or frame.capture_time_ns < 0):
            return rejected('invalid_capture_metadata')
        calibration = frame.intrinsics
        if calibration is None:
            self.reset()
            return rejected('missing_calibration')
        c = self.config
        # Changing coordinate/calibration/image conventions invalidates temporal priors.
        context = (frame.session_id, frame.pixel_format, pixels.shape, calibration,
                   c.depth_vertically_flipped, c.convert_object_axes, c.view,
                   c.raw_min, c.raw_max, c.unity_color_space, c.near, c.far)
        if context != self._context:
            self.reset()
            if (not np.isfinite(tuple(asdict(calibration).values())).all()
                    or calibration.fx <= 0 or calibration.fy <= 0):
                return rejected('invalid_calibration')
            self._context = context
            self._K = pose_camera_matrix(calibration, pixels.shape[0],
                                         depth_vertically_flipped=c.depth_vertically_flipped)
            self._distortion = calibration.distortion_coefficients
        if (self._last_frame_id is not None and (frame.frame_id <= self._last_frame_id
                or frame.observation_time <= self._last_observation)):
            return rejected('out_of_order')
        self._last_frame_id, self._last_observation = frame.frame_id, frame.observation_time
        if frame.pixel_format == PIXEL_FORMAT_UINT8_SRGB_INTENSITY:
            prepared = pixels  # no BGR expansion, raw remapping, or frame copy
        elif c.view == 'unity-raw':
            prepared = raw_to_uint8_srgb(pixels, c.raw_min, c.raw_max,
                                         linear_to_srgb=c.unity_color_space == 'linear')
        else:
            valid = np.isfinite(pixels) & (pixels > 0)
            clipped = np.clip(np.where(valid, pixels, c.near), c.near, c.far)
            grey = ((clipped-c.near)*(255./max(c.far-c.near, 1e-6))).astype(np.uint8)
            grey[~valid] = 0
            prepared = cv2.applyColorMap(255-grey, cv2.COLORMAP_TURBO)
            prepared[~valid] = 0
        intensity = prepared.max(axis=2) if prepared.ndim == 3 else prepared
        if c.detection != self._detection_config:
            self._detection_kwargs = asdict(c.detection)
            self._detection_config = c.detection
        prepared_at = self.clock()
        _, cutoff, centers = detect_marker_centers(intensity,
            **self._detection_kwargs, model_points=self.tracker.model_points, include_thresholded=False)
        detected_at = self.clock()
        solution = solve_observation(self.tracker, centers, frame, calibration,
            depth_vertically_flipped=c.depth_vertically_flipped, convert_object_axes=c.convert_object_axes,
            deadline=deadline, clock=self.clock, camera_matrix=self._K, distortion=self._distortion)
        end = self.clock()
        if deadline is not None and end > deadline and solution.estimate.ok:
            self.tracker.reset()
            solution = PoseSolution(PoseEstimate.failed(), self._K, pnp_ms=solution.pnp_ms,
                world_ms=solution.world_ms, ready_time=end, reason='expired_after_processing')
        return TrackingResult(frame.frame_id, frame.session_id, frame.capture_time_ns,
            frame.observation_time, self.tracker.state, solution, tuple(centers), cutoff,
            intensity, prepared, StageTimings((prepared_at-start)*1000.,
                (detected_at-prepared_at)*1000., solution.pnp_ms, solution.world_ms, (end-start)*1000.))
