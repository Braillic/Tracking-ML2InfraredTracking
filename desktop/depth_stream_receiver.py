#!/usr/bin/env python3
"""Receive selectable FLOAT32/PC-conversion and UINT8/ML2-conversion pipelines.

Optionally estimates tool pose with MarkerPoseTracker and streams Unity-world
poses back to PoseEstimateTcpServer on the headset (--send-pose).
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np
from marker_pose import (
    TEST_MARKER_COORDS,
    MarkerPoseTracker,
    PoseEstimate,
    filter_marker_centers,
    video_save_overlay,
)
from pose_packet import (
    DEFAULT_POSE_PORT,
    PosePacket,
    camera_matrix_for_vertically_flipped_image,
    connect_pose_socket,
    opencv_camera_pose_to_unity_world,
    send_pose,
    sync_clock_offset,
    unity_trs_matrix,
)

MAGIC = b"ML2D"
PROTOCOL_VERSION = 5
PIXEL_FORMAT_FLOAT32_RAW = 1
PIXEL_FORMAT_UINT8_SRGB_INTENSITY = 2
HEADER = struct.Struct("<4sHHIIIIQd3f4fI")
TIMING_HEADER = struct.Struct("<Qqddd")  # session, capture XR ns, mapped capture, poll start, frame ready
INTRINSICS = struct.Struct("<11d")
MAX_PAYLOAD_BYTES = 128 * 1024 * 1024
WINDOW_TITLE = "Magic Leap 2 depth transport"
ANALYSIS_WINDOW_TITLE = "Analysis (f=fixed, p=percentile)"
MARKER_MIN_AREA = 2
# Absolute fallback; close-up blobs often exceed a few thousand pixels.
MARKER_MAX_AREA = 2000
# Prefer image-relative cap: ~8% of frame (≈20k px on 544x480).
MARKER_MAX_AREA_FRAC = 0.10
MARKER_MAX_ASPECT = 6.0
MARKER_RING_RADIUS = 10
MAX_DETECTED_MARKERS = 8  # bounded candidate pool; PnP selects the constellation
# Pre-PnP spatial filter defaults (pixels). These also scale up with image size
# so close-up constellations are not rejected as "too spread out".
MARKER_MAX_NEAREST_NEIGHBOR_PX = 90.0
MARKER_MAX_NEAREST_NEIGHBOR_FRAC = 0.35  # of min(h, w)
MARKER_MAX_CLUSTER_SPAN_PX = 220.0
MARKER_MAX_CLUSTER_SPAN_FRAC = 0.85  # of min(h, w)
MARKER_MIN_CLUSTER_SIZE = 3
MARKER_GEOMETRY_RATIO_ERROR = 0.35

@dataclass(frozen=True)
# intrinsics recieved once per TCP connection
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
    intrinsics: CameraIntrinsics | None  # present once per TCP connection
    pixel_format: int = PIXEL_FORMAT_UINT8_SRGB_INTENSITY  # wire format
    session_id: int = 0
    capture_time_ns: int = 0
    capture_realtime: float = float("nan")  # ML2 clock; NaN means conversion unavailable
    poll_start_realtime: float = 0.0
    frame_ready_realtime: float = 0.0

    @property
    def observation_time(self) -> float:
        return self.capture_time_ns * 1e-9 if self.capture_time_ns > 0 else self.timestamp


def receive_exact(connection: socket.socket, count: int) -> bytes:
    data = bytearray(count)
    view = memoryview(data)
    received = 0
    while received < count:
        chunk_size = connection.recv_into(view[received:])
        if chunk_size == 0:
            raise ConnectionError("Magic Leap closed the connection")
        received += chunk_size
    return bytes(data)


def receive_frame(connection: socket.socket) -> DepthFrame:
    """Decode the wire payload only; conversion happens after recv_done."""
    values = HEADER.unpack(receive_exact(connection, HEADER.size))
    (magic, version, header_size, width, height, pixel_format, payload_size,
     frame_id, timestamp, px, py, pz, qx, qy, qz, qw, intrinsics_size) = values

    if magic != MAGIC:
        raise ValueError(f"Unexpected stream magic {magic!r}")
    expected_header = HEADER.size + TIMING_HEADER.size if version == 5 else HEADER.size
    if version not in (3, 4, 5) or header_size != expected_header:
        raise ValueError(f"Unsupported protocol version/header: {version}/{header_size}")
    timing = TIMING_HEADER.unpack(receive_exact(connection, TIMING_HEADER.size)) if version == 5 else None
    if timing is not None and (timing[0] == 0 or timing[1] <= 0
                               or not np.isfinite(timing[3:]).all() or timing[4] < timing[3]):
        raise ValueError("Invalid frame identity/capture timing")
    if pixel_format == PIXEL_FORMAT_UINT8_SRGB_INTENSITY:
        dtype = np.dtype(np.uint8)
    elif pixel_format == PIXEL_FORMAT_FLOAT32_RAW:
        dtype = np.dtype("<f4")
    else:
        raise ValueError(f"Unsupported pixel format {pixel_format}")
    expected_size = width * height * dtype.itemsize
    if width == 0 or height == 0 or payload_size != expected_size or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError(
            f"Invalid payload: {payload_size} bytes for {width}x{height} {dtype.name}"
        )
    if intrinsics_size not in (0, INTRINSICS.size):
        raise ValueError(
            f"Invalid intrinsics block size: {intrinsics_size}; expected 0 or {INTRINSICS.size}"
        )

    payload = receive_exact(connection, payload_size)
    pixels = np.frombuffer(payload, dtype=dtype).reshape(height, width)
    intrinsics = (CameraIntrinsics(*INTRINSICS.unpack(receive_exact(connection, intrinsics_size)))
                  if intrinsics_size else None)
    return DepthFrame(
        pixels=pixels,
        frame_id=frame_id,
        timestamp=timestamp,
        sensor_position=np.array((px, py, pz), dtype=np.float32),
        sensor_rotation=np.array((qx, qy, qz, qw), dtype=np.float32),
        intrinsics=intrinsics,
        pixel_format=pixel_format,
        **(dict(zip(("session_id", "capture_time_ns", "capture_realtime", "poll_start_realtime",
                     "frame_ready_realtime"), timing)) if timing is not None else {}),
    )


class LatestFrameReceiver:
    """One socket reader, one replaceable complete-frame slot per connection.

    GUI/recording/PnP stalls drop intermediate frames instead of building a FIFO.
    Intrinsics belong to the connection and survive dropping their first frame.
    Receive timestamps are taken here, never when processing eventually starts.
    """

    def __init__(self, connection: socket.socket):
        self.connection = connection
        self._condition = threading.Condition()
        self._pending = None
        self._error = None
        self._stopped = False
        self.dropped_frames = 0
        self.received_frames = 0
        self._thread = threading.Thread(target=self._read, name="ML2 latest frame", daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        with self._condition:
            self._stopped = True
            self._pending = None
            self._condition.notify_all()
        # Interrupt recv_into before joining; no old reader survives reconnect.
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._thread.join()

    def _read(self):
        intrinsics = None
        try:
            while True:
                frame = receive_frame(self.connection)
                received = time.perf_counter()
                if frame.intrinsics is not None:
                    intrinsics = frame.intrinsics
                frame = replace(frame, intrinsics=intrinsics)
                with self._condition:
                    if self._stopped:
                        return
                    if self._pending is not None:
                        self.dropped_frames += 1
                    self.received_frames += 1
                    self._pending = (frame, received)
                    self._condition.notify_all()
        except (ConnectionError, OSError, ValueError) as error:
            with self._condition:
                self._error = error
                self._pending = None  # disconnected sessions cannot publish poses
                self._condition.notify_all()

    def take(self):
        with self._condition:
            self._condition.wait_for(lambda: self._pending is not None or self._error or self._stopped)
            if self._error is not None:
                raise self._error
            if self._stopped:
                raise ConnectionError("Depth receiver stopped")
            result, self._pending = self._pending, None
            return result


def raw_to_uint8_srgb(
    depth: np.ndarray,
    raw_min: float,
    raw_max: float,
    *,
    linear_to_srgb: bool,
) -> np.ndarray:
    """Map FLOAT32 DepthRaw values to the same uint8 image produced on ML2."""
    finite = np.isfinite(depth)
    normalized = (
        (np.where(finite, depth, raw_min) - raw_min)
        / max(raw_max - raw_min, 1e-6)
    )
    normalized = np.clip(normalized, 0.0, 1.0)
    if linear_to_srgb:
        normalized = np.where(
            normalized <= 0.0031308,
            normalized * 12.92,
            1.055 * np.power(normalized, 1.0 / 2.4) - 0.055,
        )
    grey = np.rint(normalized * 255.0).astype(np.uint8)
    grey[~finite] = 0
    return grey


def colourise_depth(depth: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """Legacy e8a3201 FLOAT32-to-BGR mapping, also used by offline replay."""
    finite = np.isfinite(depth)

    if args.view == "unity-raw":
        grey = raw_to_uint8_srgb(
            depth,
            args.raw_min,
            args.raw_max,
            linear_to_srgb=args.unity_color_space == "linear",
        )
        return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)

    valid = finite & (depth > 0)
    clipped = np.clip(np.where(valid, depth, args.near), args.near, args.far)
    grey = ((clipped - args.near) *
            (255.0 / max(args.far - args.near, 1e-6))).astype(np.uint8)
    grey[~valid] = 0
    colour = cv2.applyColorMap(255 - grey, cv2.COLORMAP_TURBO)
    colour[~valid] = 0
    return colour


def prepare_detection_image(frame: DepthFrame, args: argparse.Namespace) -> np.ndarray:
    """Both pipelines converge on UINT8 BGR before render_analysis()."""
    if frame.pixel_format == PIXEL_FORMAT_FLOAT32_RAW:
        return colourise_depth(frame.pixels, args)
    if frame.pixel_format == PIXEL_FORMAT_UINT8_SRGB_INTENSITY:
        return cv2.cvtColor(frame.pixels, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported pixel format {frame.pixel_format}")


def apply_threshold(
    image: np.ndarray,
    threshold_method: str,
    fixed_threshold: float = 200.0,
    top_p: tuple[float, float] = (100.0, 90.0),
) -> tuple[np.ndarray, float]:
    """Threshold a colourised uint8 image. top_p is (absolute floor, percentile).

    Intensity is the max channel per pixel (works for grey unity-raw and turbo).
    Pixels below the cutoff are set to 0; others keep their original colour.
    """
    if image.ndim == 3:
        intensity = image.max(axis=2).astype(np.float32)
    else:
        intensity = image.astype(np.float32)

    if threshold_method == "fixed":
        cutoff = float(fixed_threshold)
    elif threshold_method == "percentile":
        floor, percentile = top_p
        if intensity.size == 0:
            cutoff = float(floor)
        else:
            cutoff = max(float(floor), float(np.percentile(intensity, percentile)))
    else:
        raise ValueError(f"Invalid threshold method: {threshold_method}")

    keep = intensity >= cutoff
    if image.ndim == 3:
        thresholded = np.where(keep[..., None], image, 0).astype(np.uint8)
    else:
        thresholded = np.where(keep, image, 0).astype(np.uint8)
    return thresholded, cutoff


def blob_area_limits(
    image_shape: tuple[int, ...],
    *,
    min_area: int | None = None,
    max_area: int | None = None,
    max_area_frac: float = MARKER_MAX_AREA_FRAC,
) -> tuple[int, int]:
    """Area gate that scales with frame size for close-up markers."""
    height, width = int(image_shape[0]), int(image_shape[1])
    lo = MARKER_MIN_AREA if min_area is None else int(min_area)
    frac_cap = max(lo + 1, int(max_area_frac * height * width))
    hi = MARKER_MAX_AREA if max_area is None else int(max_area)
    return lo, max(hi, frac_cap)


def geometry_pixel_limits(
    image_shape: tuple[int, ...],
    *,
    max_nearest_neighbor_px: float | None = None,
    max_cluster_span_px: float | None = None,
) -> tuple[float, float]:
    """Isolation / span gates that grow for close-up (large on-screen) tools."""
    short_side = float(min(int(image_shape[0]), int(image_shape[1])))
    nn = MARKER_MAX_NEAREST_NEIGHBOR_PX if max_nearest_neighbor_px is None else float(
        max_nearest_neighbor_px
    )
    span = MARKER_MAX_CLUSTER_SPAN_PX if max_cluster_span_px is None else float(
        max_cluster_span_px
    )
    nn = max(nn, MARKER_MAX_NEAREST_NEIGHBOR_FRAC * short_side)
    span = max(span, MARKER_MAX_CLUSTER_SPAN_FRAC * short_side)
    return nn, span


def detect_marker_centers(
    colourised: np.ndarray,
    threshold_method: str,
    fixed_threshold: float,
    top_p: tuple[float, float],
    # max_markers: int = 5,
    *,
    filter_geometry: bool = True,
    model_points: np.ndarray | None = None,
    min_area: int | None = None,
    max_area: int | None = None,
    max_aspect: float = MARKER_MAX_ASPECT,
    max_nearest_neighbor_px: float | None = None,
    max_cluster_span_px: float | None = None,
    min_cluster_size: int = MARKER_MIN_CLUSTER_SIZE,
    max_geometry_ratio_error: float = MARKER_GEOMETRY_RATIO_ERROR,
    max_markers: int=MAX_DETECTED_MARKERS,
) -> tuple[np.ndarray, float, list[tuple[float, float]]]:
    """Threshold a colourised frame and locate bright marker blobs."""
    thresholded, cutoff = apply_threshold(
        colourised,
        threshold_method,
        fixed_threshold=fixed_threshold,
        top_p=top_p,
    )

    if colourised.ndim == 3:
        intensity = colourised.max(axis=2).astype(np.float32)
    else:
        intensity = colourised.astype(np.float32)

    min_blob_area, max_blob_area = blob_area_limits(
        colourised.shape, min_area=min_area, max_area=max_area
    )
    max_nn_px, max_span_px = geometry_pixel_limits(
        colourised.shape,
        max_nearest_neighbor_px=max_nearest_neighbor_px,
        max_cluster_span_px=max_cluster_span_px,
    )

    mask = (intensity >= cutoff).astype(np.uint8) * 255
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    candidates: list[tuple[float, tuple[int, int]]] = []
    for label in range(1, n_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        aspect = max(width, height) / max(1, min(width, height))

        if area < min_blob_area or area > max_blob_area or aspect > max_aspect:
            continue

        # Restrict each blob scan to its bounding box rather than repeatedly
        # scanning the full image for every connected component.
        left, top = int(stats[label, cv2.CC_STAT_LEFT]), int(stats[label, cv2.CC_STAT_TOP])
        ys, xs = np.where(labels[top:top + height, left:left + width] == label)
        ys, xs = ys + top, xs + left
        weights = intensity[ys, xs]
        if weights.size == 0 or float(weights.sum()) <= 0:
            continue

        weight_sum = float(weights.sum())
        cx = float((xs * weights).sum() / weight_sum)
        cy = float((ys * weights).sum() / weight_sum)
        mean_intensity = float(weights.mean())
        candidates.append((mean_intensity, (cx, cy)))

    candidates.sort(key=lambda item: item[0], reverse=True)
    centers = [center for _, center in candidates[:max_markers]]
    if filter_geometry:
        centers = filter_marker_centers(
            centers,
            model_points=TEST_MARKER_COORDS if model_points is None else model_points,
            max_nearest_neighbor_px=max_nn_px,
            max_cluster_span_px=max_span_px,
            min_cluster_size=min_cluster_size,
            max_geometry_ratio_error=max_geometry_ratio_error,
            preserve_candidates=True,
        )
    return thresholded, cutoff, centers


def annotate_markers(image: np.ndarray, centers: list[tuple[float, float]]) -> np.ndarray:
    """Draw a ring and index around each detected marker."""
    overlay = image.copy()
    for index, (x, y) in enumerate(centers):
        p = _cv_point(x, y)
        if p is None:
            continue
        cv2.circle(overlay, p, MARKER_RING_RADIUS, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, str(index), (p[0] + 12, p[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return overlay


_CV_INT32_MIN = -(2**31)
_CV_INT32_MAX = 2**31 - 1


def _cv_point(x: object, y: object) -> tuple[int, int] | None:
    """OpenCV drawing APIs need native Python ints in int32 range (not numpy / huge ints)."""
    try:
        xf = float(np.asarray(x).reshape(-1)[0])
        yf = float(np.asarray(y).reshape(-1)[0])
    except (TypeError, ValueError, IndexError):
        return None
    if not (np.isfinite(xf) and np.isfinite(yf)):
        return None
    # Bad / unstable PnP (e.g. a marker occluded) can project to ±1e20; OpenCV 5 then
    # rejects the point with "Can't parse 'position'" because it must fit int32.
    try:
        xi = int(round(xf))
        yi = int(round(yf))
    except (OverflowError, ValueError):
        return None
    if not (_CV_INT32_MIN <= xi <= _CV_INT32_MAX and _CV_INT32_MIN <= yi <= _CV_INT32_MAX):
        return None
    return xi, yi


def flip_ud_points(
    points: list[tuple[float, float]] | np.ndarray | None,
    image_height: int,
) -> list[tuple[float, float]] | np.ndarray | None:
    """Map stream-buffer coordinates → upright display coordinates (vertical flip)."""
    if points is None:
        return None
    if isinstance(points, list):
        out: list[tuple[float, float]] = []
        for x, y in points:
            p = _cv_point(x, y)
            if p is None:
                continue
            out.append((p[0], int(image_height) - 1 - p[1]))
        return out
    arr = np.asarray(points, dtype=np.float64).reshape(-1, 2).copy()
    if arr.size == 0:
        return arr
    arr[:, 1] = float(image_height) - 1.0 - arr[:, 1]
    return arr


def prepare_display_view(
    image: np.ndarray,
    centers: list[tuple[float, float]],
    projected: np.ndarray | None,
    *,
    unflip: bool,
) -> tuple[np.ndarray, list[tuple[float, float]], np.ndarray | None]:
    """Optionally un-flip the streamed image + overlays for upright PC viewing."""
    if not unflip:
        return image, centers, projected
    height = int(image.shape[0])
    return (
        cv2.flip(image, 0),
        flip_ud_points(centers, height) or [],
        flip_ud_points(projected, height),
    )


def pose_camera_matrix(
    intrinsics: CameraIntrinsics | None,
    image_height: int,
    *,
    depth_vertically_flipped: bool,
) -> np.ndarray | None:
    """Camera matrix used for live PnP (includes flip-cy adjustment when needed)."""
    if intrinsics is None:
        return None
    camera_matrix = intrinsics.camera_matrix
    if depth_vertically_flipped:
        camera_matrix = camera_matrix_for_vertically_flipped_image(camera_matrix, image_height)
    return camera_matrix


def project_model_points(
    estimate: PoseEstimate,
    model_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray | None,
) -> np.ndarray | None:
    """Project all model markers with the estimated pose into the image."""
    if not estimate.ok or estimate.rvec is None or estimate.tvec is None:
        return None
    dist = (
        np.zeros(5, dtype=np.float64)
        if dist_coeffs is None
        else np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
    )
    projected, _ = cv2.projectPoints(
        np.asarray(model_points, dtype=np.float64).reshape(-1, 1, 3),
        estimate.rvec,
        estimate.tvec,
        np.asarray(camera_matrix, dtype=np.float64),
        dist,
    )
    return projected.reshape(-1, 2)


def draw_pose_overlay(
    image: np.ndarray,
    centers: list[tuple[float, float]],
    estimate: PoseEstimate,
    projected: np.ndarray | None,
    *,
    draw_detections: bool = True,
    status_at_bottom: bool = True,
) -> np.ndarray:
    """Overlay detections (red), PnP projections (green), and match lines (cyan)."""
    out = image.copy()
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    detections = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
    if draw_detections:
        for i, (x, y) in enumerate(detections):
            p = _cv_point(x, y)
            if p is None:
                continue
            cv2.circle(out, p, 6, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(out, f"d{i}", (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

    if projected is not None:
        for mi, (x, y) in enumerate(np.asarray(projected, dtype=np.float64).reshape(-1, 2)):
            p = _cv_point(x, y)
            if p is None:
                continue
            cv2.drawMarker(out, p, (0, 255, 0), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
            cv2.putText(out, f"m{mi}", (p[0] + 8, p[1] + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

    if estimate.ok and projected is not None:
        proj = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
        for mi, ii in zip(estimate.model_indices, estimate.image_indices):
            if ii >= detections.shape[0] or mi >= proj.shape[0]:
                continue
            a = _cv_point(detections[ii, 0], detections[ii, 1])
            b = _cv_point(proj[mi, 0], proj[mi, 1])
            if a is None or b is None:
                continue
            cv2.line(out, a, b, (255, 200, 0), 1, cv2.LINE_AA)

    if estimate.ok:
        label = (
            f"ok conf={estimate.confidence:.2f} reproj={estimate.mean_reproj_px:.2f}px "
            f"ids={estimate.model_indices} cov={estimate.coverage:.2f}"
        )
        color = (0, 255, 0)
    else:
        label = "ok=False (PnP failed)"
        color = (0, 0, 255)

    if status_at_bottom:
        y0 = int(out.shape[0]) - 28
        y1 = int(out.shape[0]) - 10
    else:
        y0, y1 = 25, 50
    cv2.putText(out, label, (10, y0), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, color, 1, cv2.LINE_AA)
    cv2.putText(out, "red=det  green=proj  cyan=match", (10, y1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return out


def render_analysis(
    colourised: np.ndarray,
    threshold_method: str,
    fixed_threshold: float,
    top_p: tuple[float, float],
    *,
    filter_geometry: bool = True,
    min_area: int | None = None,
    max_area: int | None = None,
    max_aspect: float = MARKER_MAX_ASPECT,
    max_nearest_neighbor_px: float | None = None,
    max_cluster_span_px: float | None = None,
    min_cluster_size: int = MARKER_MIN_CLUSTER_SIZE,
    max_geometry_ratio_error: float = MARKER_GEOMETRY_RATIO_ERROR,
) -> tuple[np.ndarray, np.ndarray, list[tuple[float, float]]]:
    """Build the Analysis window from an already-colourised frame."""
    analysis, cutoff, centers = detect_marker_centers(
        colourised,
        threshold_method,
        fixed_threshold=fixed_threshold,
        top_p=top_p,
        filter_geometry=filter_geometry,
        min_area=min_area,
        max_area=max_area,
        max_aspect=max_aspect,
        max_nearest_neighbor_px=max_nearest_neighbor_px,
        max_cluster_span_px=max_cluster_span_px,
        min_cluster_size=min_cluster_size,
        max_geometry_ratio_error=max_geometry_ratio_error,
    )
    # Do not draw marker rings or HUD here — caller unflips for display first,
    # then draw_pose_overlay / draw_analysis_hud own graphics and text.
    view = analysis.copy()
    if view.ndim == 2:
        view = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)
    return view, analysis, centers, cutoff


def analysis_mode_label(
    threshold_method: str,
    fixed_threshold: float,
    top_p: tuple[float, float],
    cutoff: float,
) -> str:
    if threshold_method == "fixed":
        return f"fixed >= {fixed_threshold:g}  (cutoff {cutoff:.1f})"
    return f"percentile p{top_p[1]:g} floor {top_p[0]:g}  (cutoff {cutoff:.1f})"


def draw_analysis_hud(image: np.ndarray, mode_label: str) -> np.ndarray:
    """Threshold-mode help text at the top (pose status stays at the bottom)."""
    out = image
    cv2.putText(out, mode_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, "keys: f=fixed  p=percentile  q=quit", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return out


def show_waiting_window(host: str, port: int) -> None:
    waiting = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(waiting, "TCP connected", (40, 130), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (80, 220, 80), 2, cv2.LINE_AA)
    cv2.putText(waiting, f"Waiting for first frame from {host}:{port}", (40, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(waiting, "Check the ML status: queued should become > 0", (40, 230),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.namedWindow(ANALYSIS_WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.imshow(WINDOW_TITLE, waiting)
    cv2.imshow(ANALYSIS_WINDOW_TITLE, waiting)
    cv2.waitKey(1)


def _geometry_filter_kwargs(args: argparse.Namespace) -> dict:
    return {
        "filter_geometry": args.filter_geometry,
        "min_area": args.min_marker_area,
        "max_area": args.max_marker_area,
        "max_aspect": args.max_marker_aspect,
        "max_nearest_neighbor_px": args.max_nearest_neighbor_px,
        "max_cluster_span_px": args.max_cluster_span_px,
        "min_cluster_size": args.min_cluster_size,
        "max_geometry_ratio_error": args.max_geometry_ratio_error,
    }

def _new_recording_dir(base_dir: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    # Keep it deterministic and filesystem-friendly.
    path = base_dir / f"recording_{stamp}"
    path.mkdir(parents=True, exist_ok=False)
    # also make subdirs for intensity, display, centers and intrinsics
    intensity_dir = path / "intensity"
    display_dir = path / "display"
    # analysis_dir = path / "analysis"
    centers_dir = path / "centers"
    intrinsics_dir = path / "intrinsics"
    intensity_dir.mkdir(parents=True, exist_ok=True)
    display_dir.mkdir(parents=True, exist_ok=True)
    # analysis_dir.mkdir(parents=True, exist_ok=True)
    centers_dir.mkdir(parents=True, exist_ok=True)
    intrinsics_dir.mkdir(parents=True, exist_ok=True)
    return path


def estimate_and_send_pose(
    tracker: MarkerPoseTracker,
    centers: list[tuple[float, float]],
    frame: DepthFrame,
    intrinsics: CameraIntrinsics | None,
    pose_connection: socket.socket | None,
    *,
    depth_vertically_flipped: bool = True,
    convert_object_axes: bool = False,
    detect_ms: float = 0.0,
    recv_done: float = 0.0,
    clock_offset: float = 0.0,
    max_processing_age_s: float = 0.10,
) -> tuple[PoseEstimate, np.ndarray | None]:
    """Run MarkerPoseTracker and optionally stream Unity-world pose to the headset.

    detect_ms is the marker-detection duration measured by the caller (before this
    function runs); it is only forwarded into the outgoing packet so ML2 can log a
    per-stage breakdown instead of one lumped "PC + network" bucket.

    recv_done (this frame's perf_counter() receive time) and clock_offset (from
    sync_clock_offset) let ML2 translate PC timestamps into its own clock domain
    and split the round trip into its two network legs.

    Returns (estimate, camera_matrix_used_for_pnp).
    """
    camera_matrix = pose_camera_matrix(
        intrinsics,
        frame.pixels.shape[0],
        depth_vertically_flipped=depth_vertically_flipped,
    )
    if camera_matrix is None:
        tracker.reset()
        return PoseEstimate.failed(), camera_matrix

    pnp_start = time.perf_counter()
    if recv_done > 0 and pnp_start - recv_done > max_processing_age_s:
        return PoseEstimate.failed(), camera_matrix
    camera_world = unity_trs_matrix(frame.sensor_position, frame.sensor_rotation)
    if not depth_vertically_flipped or convert_object_axes:
        camera_world[:3, 1] *= -1  # same camera basis as world-pose conversion

    estimate = tracker.estimate(
        centers,
        camera_matrix,
        intrinsics.distortion_coefficients if intrinsics is not None else None,
        observation_time=frame.observation_time,
        camera_world_transform=camera_world,
    )
    pnp_ms = (time.perf_counter() - pnp_start) * 1000.0

    if pose_connection is None:
        return estimate, camera_matrix

    # Explicit rejected observations let ML2 count failures. The bounded hold
    # policy still uses the last accepted source time (one miss does not flicker).
    if not (estimate.ok and estimate.rvec is not None and estimate.tvec is not None):
        if recv_done <= 0 or time.perf_counter() - recv_done <= max_processing_age_s:
            send_pose(pose_connection, PosePacket.rejected(frame.frame_id, frame.session_id))
        return estimate, camera_matrix

    prep_start = time.perf_counter()
    position, rotation = opencv_camera_pose_to_unity_world(
        estimate.rvec,
        estimate.tvec,
        frame.sensor_position,
        frame.sensor_rotation,
        depth_vertically_flipped=depth_vertically_flipped,
        convert_object_axes=convert_object_axes,
    )
    # prep_ms covers world-space transform + struct packing, up to (but not
    # including) the sendall syscall — a packet can't carry its own send
    # duration, so that part is measured separately below and only printed
    # locally; it's negligible for a 64-byte write with TCP_NODELAY.
    send_ready_time = time.perf_counter()
    if recv_done > 0 and send_ready_time - recv_done > max_processing_age_s:
        tracker.reset()  # do not let a pose we cannot publish become the next prior
        return PoseEstimate.failed(), camera_matrix
    packet = PosePacket(
        frame_id=frame.frame_id,
        session_id=frame.session_id,
        ok=True,
        confidence=float(estimate.confidence),
        position=position,
        rotation=rotation,
        detect_ms=detect_ms,
        pnp_ms=pnp_ms,
        send_ms=(send_ready_time - prep_start) * 1000.0,
        pc_recv_ml2=recv_done + clock_offset,
        pc_send_ml2=send_ready_time + clock_offset,
    )
    send_start = time.perf_counter()
    try:
        send_pose(pose_connection, packet)
    except OSError as error:
        raise ConnectionError(f"Pose stream send failed: {error}") from error
    sendall_ms = (time.perf_counter() - send_start) * 1000.0
    if sendall_ms > 1.0:
        print(f"[ML2LAT] slow pose sendall: {sendall_ms:.1f}ms (not reflected in ML2's log)", flush=True)
    return estimate, camera_matrix


def run(args: argparse.Namespace) -> None:
    save_directory = Path(args.save_dir) if args.save_dir else None
    if save_directory:
        save_directory.mkdir(parents=True, exist_ok=True)

    threshold_method = args.threshold_method
    fixed_threshold = args.fixed_threshold
    top_p = (args.percentile_floor, args.percentile)
    tracker = MarkerPoseTracker(TEST_MARKER_COORDS,
                                lost_timeout_s=args.tracking_lost_timeout,
                                acquisition_timeout_s=args.acquisition_timeout)

    recording = False
    recording_dir: Path | None = None
    recording_index = 0

    while True:
        pose_connection: socket.socket | None = None
        clock_offset = 0.0
        tracker.reset()
        try:
            print(f"Connecting to Magic Leap depth at {args.host}:{args.port} ...")
            with socket.create_connection((args.host, args.port), timeout=args.connect_timeout) as connection:
                connection.settimeout(args.frame_timeout)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("Depth TCP connected; waiting for the first depth frame header...", flush=True)

                if args.send_pose:
                    print(f"Connecting to Magic Leap pose at {args.host}:{args.pose_port} ...")
                    pose_connection = connect_pose_socket(
                        args.host, args.pose_port, timeout=args.connect_timeout
                    )
                    pose_connection.settimeout(args.frame_timeout)
                    print("Pose TCP connected.", flush=True)
                    clock_offset, sync_rtt = sync_clock_offset(pose_connection)
                    print(
                        f"Clock sync: offset={clock_offset * 1000.0:+.1f}ms sync_rtt={sync_rtt * 1000.0:.1f}ms "
                        "(assumes symmetric Wi-Fi latency; ML2's leg1/leg2 split is accurate to "
                        "roughly +/-sync_rtt/2)",
                        flush=True,
                    )
                    tracker.reset()

                if not args.headless:
                    show_waiting_window(args.host, args.port)
                last_pixel_format = None
                last_session_id = None
                processed = accepted = stale_before = 0
                pc_work_ms = deque(maxlen=256)
                next_diagnostics = time.perf_counter() + args.diagnostics_interval
                intrinsics = None

                with LatestFrameReceiver(connection) as receiver:
                    while True:
                        frame, recv_done = receiver.take()
                        if args.diagnostics_interval > 0 and time.perf_counter() >= next_diagnostics:
                            percentiles = (np.percentile(pc_work_ms, [50, 95, 99]).round(2).tolist()
                                           if pc_work_ms else [])
                            print(f"[PCTracking] session={frame.session_id} received={receiver.received_frames} "
                                  f"overwritten={receiver.dropped_frames} processed={processed} accepted={accepted} "
                                  f"failed_or_expired={processed - accepted} stale_before={stale_before} "
                                  f"pc_work_ms_p50_p95_p99={percentiles} state={tracker.state}", flush=True)
                            next_diagnostics = time.perf_counter() + args.diagnostics_interval
                        if time.perf_counter() - recv_done > args.max_processing_age:
                            stale_before += 1
                            continue
                        if frame.session_id != last_session_id:
                            tracker.reset()
                            last_session_id = frame.session_id
                        detection_start = time.perf_counter()
                        if frame.pixel_format != last_pixel_format:
                            pipeline = ("1: FLOAT32 -> colourise_depth() on PC"
                                        if frame.pixel_format == PIXEL_FORMAT_FLOAT32_RAW
                                        else "2: UINT8 from ML2 -> grayscale-to-BGR on PC")
                            print(f"Pipeline {pipeline}; {frame.pixels.shape[1]}x{frame.pixels.shape[0]}, "
                                  f"{frame.pixels.nbytes:,} payload bytes/frame", flush=True)
                            tracker.reset()
                            last_pixel_format = frame.pixel_format
                        if frame.intrinsics is not None and frame.intrinsics != intrinsics:
                            intrinsics = frame.intrinsics
                            print("Intrinsics received once:", intrinsics, flush=True)
                            print("OpenCV camera matrix:\n", intrinsics.camera_matrix, flush=True)
                            print("OpenCV distortion coefficients:",
                                  intrinsics.distortion_coefficients, flush=True)

                        display = (frame.pixels if args.headless and frame.pixel_format == PIXEL_FORMAT_UINT8_SRGB_INTENSITY
                                   else prepare_detection_image(frame, args))
                        # Keep mapped intensity recordings/HUD for both transport modes.
                        intensity = (frame.pixels if frame.pixel_format == PIXEL_FORMAT_UINT8_SRGB_INTENSITY
                                     else display.max(axis=2))
                        if args.headless:
                            analysis, cutoff, centers = detect_marker_centers(
                                display, threshold_method, fixed_threshold, top_p,
                                **_geometry_filter_kwargs(args))
                        else:
                            analysis_view, analysis, centers, cutoff = render_analysis(
                                display, threshold_method, fixed_threshold, top_p,
                                **_geometry_filter_kwargs(args))
                        detect_ms = (time.perf_counter() - detection_start) * 1000.0

                        estimate, pose_K = estimate_and_send_pose(
                            tracker,
                            centers,
                            frame,
                            intrinsics,
                            pose_connection,
                            depth_vertically_flipped=args.depth_vertically_flipped,
                            convert_object_axes=args.convert_object_axes,
                            detect_ms=detect_ms,
                            recv_done=recv_done,
                            clock_offset=clock_offset,
                            max_processing_age_s=args.max_processing_age,
                        )
                        processed += 1
                        accepted += int(estimate.ok)
                        pc_work_ms.append((time.perf_counter() - detection_start) * 1000.0)
                        if args.headless:
                            continue
                        projected = None
                        if pose_K is not None:
                            projected = project_model_points(
                                estimate,
                                tracker.model_points,
                                pose_K,
                                intrinsics.distortion_coefficients if intrinsics is not None else None,
                            )

                        # Detection/PnP stay in stream coordinates; only the PC view is un-flipped.
                        display_view, centers_view, projected_view = prepare_display_view(
                            display, centers, projected, unflip=args.display_unflip
                        )
                        analysis_view, _, _ = prepare_display_view(
                            analysis_view, centers, projected, unflip=args.display_unflip
                        )
                        analysis_view = draw_analysis_hud(
                            analysis_view,
                            analysis_mode_label(threshold_method, fixed_threshold, top_p, cutoff),
                        )

                        annotated_display = draw_pose_overlay(
                            display_view, centers_view, estimate, projected_view
                        )
                        annotated_thresholded = draw_pose_overlay(
                            analysis_view, centers_view, estimate, projected_view
                        )

                        valid = intensity > 0
                        if np.any(valid):
                            minimum = int(intensity[valid].min())
                            maximum = int(intensity[valid].max())
                            label = f"frame {frame.frame_id}  range {minimum}-{maximum} uint8 sRGB"
                        else:
                            label = f"frame {frame.frame_id}  no nonzero intensity"
                        cv2.putText(annotated_display, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.55, (255, 255, 255), 1, cv2.LINE_AA)
                        p = frame.sensor_position
                        q = frame.sensor_rotation
                        pose_label = (f"pose P=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})m "
                                      f"Q=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})")
                        cv2.putText(annotated_display, pose_label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4, (255, 255, 255), 1, cv2.LINE_AA)

                        if recording:
                            cv2.putText(annotated_display, f"REC {recording_index:06d}", (10, 75),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

                        cv2.imshow(WINDOW_TITLE, annotated_display)
                        cv2.imshow(ANALYSIS_WINDOW_TITLE, annotated_thresholded)

                        key = cv2.waitKey(1) & 0xFF
                        if key in (27, ord("q")):
                            if pose_connection is not None:
                                try:
                                    pose_connection.close()
                                except OSError:
                                    pass
                                pose_connection = None
                            return
                        if key == ord("r"):
                            if not recording:
                                base = save_directory or (Path.cwd() / "saves")
                                base.mkdir(parents=True, exist_ok=True)
                                recording_dir = _new_recording_dir(base)
                                recording = True
                                recording_index = 0
                                print(f"Recording started: {recording_dir}", flush=True)
                            else:
                                recording = False
                                print(
                                    f"Recording stopped: {recording_dir} ({recording_index} frames)",
                                    flush=True,
                                )
                                # compile all frames into a video
                                frames_to_load = recording_dir.glob("display/*.npy")
                                # make sure the frames are in the correct order
                                frames_to_load = sorted(frames_to_load, key=lambda x: int(x.stem.split("_")[1]))
                                frames = []
                                for frame in frames_to_load:
                                    frames.append(np.load(frame))
                                video_save_overlay(recording_dir, frames)
                                recording_dir = None
                        if key == ord("f"):
                            threshold_method = "fixed"
                            print("Analysis: fixed threshold", flush=True)
                            analysis_view, analysis, centers, cutoff = render_analysis(
                                display,
                                threshold_method, fixed_threshold, top_p,
                                **_geometry_filter_kwargs(args),
                            )
                            analysis_view, _, _ = prepare_display_view(
                                analysis_view, centers, None, unflip=args.display_unflip
                            )
                            analysis_view = draw_analysis_hud(
                                analysis_view,
                                analysis_mode_label(threshold_method, fixed_threshold, top_p, cutoff),
                            )
                            cv2.imshow(ANALYSIS_WINDOW_TITLE, analysis_view)
                        elif key == ord("p"):
                            threshold_method = "percentile"
                            print("Analysis: percentile threshold", flush=True)
                            analysis_view, analysis, centers, cutoff = render_analysis(
                                display,
                                threshold_method, fixed_threshold, top_p,
                                **_geometry_filter_kwargs(args),
                            )
                            analysis_view, _, _ = prepare_display_view(
                                analysis_view, centers, None, unflip=args.display_unflip
                            )
                            analysis_view = draw_analysis_hud(
                                analysis_view,
                                analysis_mode_label(threshold_method, fixed_threshold, top_p, cutoff),
                            )
                            cv2.imshow(ANALYSIS_WINDOW_TITLE, analysis_view)
                        elif key == ord("s"):
                            # prompt for save name
                            save_name = input("Enter save name: ")
                            if not save_name:
                                print("Save name cannot be empty")
                                continue
                            target = ((save_directory or Path.cwd()) /
                                      f"intensity_{save_name}")
                            np.save(target, intensity)
                            display_target = ((save_directory or Path.cwd()) /
                                      f"display_{save_name}.png")
                            cv2.imwrite(display_target, display)
                            analysis_target = ((save_directory or Path.cwd()) /
                                      f"analysis_{save_name}")
                            np.save(f"{analysis_target}.npy", analysis)
                            cv2.imwrite(f"{analysis_target}.png", analysis)
                            print(f"Saved {target}")
                            print(f"Saved {display_target}")
                            print(f"Saved {analysis_target}")

                        if recording:
                            # Save current frame to the active recording directory.
                            # Save intensity + display + centers for offline inspection.
                            assert recording_dir is not None
                            stem = f"{recording_index:06d}_{frame.frame_id}_{frame.timestamp:.3f}"
                            # save intrinsics and distortion coefficients only once
                            if recording_index == 0 and intrinsics is not None:
                                np.save(recording_dir / "intrinsics" / "camera_matrix.npy", intrinsics.camera_matrix)
                                np.save(recording_dir / "intrinsics" / "dist_coeffs.npy", intrinsics.distortion_coefficients)

                            np.save(recording_dir / "intensity" / f"intensity_{stem}.npy", intensity)
                            # np.save(recording_dir / "analysis" / f"analysis_{stem}.npy", analysis)
                            cv2.imwrite(str(recording_dir / "display" / f"display_{stem}.png"), annotated_display)
                            np.save(recording_dir / "display" / f"display_{stem}.npy", annotated_display)
                            # cv2.imwrite(str(recording_dir / "analysis" / f"analysis_{stem}.png"), analysis)
                            np.save(recording_dir / "centers" / f"centers_{stem}.npy", np.array(centers, dtype=np.float64))
                            # Keep exact capture time and camera extrinsics for motion-correct replay.
                            metadata = dict(session_id=frame.session_id, capture_time_ns=frame.capture_time_ns,
                                            observation_time=frame.observation_time,
                                            pnp_camera_matrix=pose_K.tolist() if pose_K is not None else None,
                                            sensor_position=frame.sensor_position.tolist(),
                                            sensor_rotation=frame.sensor_rotation.tolist(),
                                            depth_vertically_flipped=args.depth_vertically_flipped,
                                            convert_object_axes=args.convert_object_axes)
                            (recording_dir / "centers" / f"centers_{stem}.json").write_text(json.dumps(metadata))
                            recording_index += 1

        except (ConnectionError, OSError, ValueError) as error:
            if not args.reconnect:
                raise
            print(f"Stream unavailable: {error}. Retrying in {args.retry_delay:g}s.")
            time.sleep(args.retry_delay)
        finally:
            if pose_connection is not None:
                try:
                    pose_connection.close()
                except OSError:
                    pass
                pose_connection = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="ML Wi-Fi IP, or 127.0.0.1 when using adb forward")
    parser.add_argument("--port", type=int, default=50777,
                        help="DepthFrameTcpServer port (ML → PC)")
    parser.add_argument("--pose-port", type=int, default=DEFAULT_POSE_PORT,
                        help="PoseEstimateTcpServer port (PC → ML)")
    parser.add_argument("--headless", action="store_true",
                        help="Run detection and tracking without preview, overlays, or interactive recording")
    parser.add_argument("--diagnostics-interval", type=float, default=2.0,
                        help="Seconds between PC timing/drop summaries; 0 disables these diagnostics")
    parser.add_argument("--send-pose", action="store_true",
                        help="Estimate pose live and stream ML2P packets to the headset")
    parser.add_argument(
        "--depth-vertically-flipped",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="ML streams flipped depth (shouldFlipTexture=true): adjust cy for PnP and "
             "skip the OpenCV→Unity Y-flip (default: on). Required for correct motion direction.",
    )
    parser.add_argument(
        "--display-unflip",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Vertically flip PC Analysis/depth windows for upright viewing "
             "(default: on). Does not change PnP/stream coordinates.",
    )
    parser.add_argument(
        "--convert-object-axes",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Also conjugate object axes with C (C@R@C). Usually leave off; "
             "with a flipped depth stream this mirrors probe motion.",
    )
    parser.add_argument("--view", choices=("unity-raw", "turbo"), default="unity-raw",
                        help="Display mapping; unity-raw matches DepthRawMat")
    parser.add_argument("--raw-min", type=float, default=5.0,
                        help="Unity raw grayscale minimum (_RawMin)")
    parser.add_argument("--raw-max", type=float, default=3000.0,
                        help="Unity raw grayscale maximum (_RawMax)")
    parser.add_argument("--unity-color-space", choices=("linear", "gamma"), default="linear",
                        help="Match the Unity project's active color space")
    parser.add_argument("--near", type=float, default=0.2,
                        help="Turbo view near plane in metres")
    parser.add_argument("--far", type=float, default=5.0,
                        help="Turbo view far plane in metres")
    parser.add_argument("--save-dir", default="./saves", help="Directory used when S is pressed")
    parser.add_argument("--threshold-method", choices=("fixed", "percentile"),
                        default="fixed",
                        help="Initial Analysis-window threshold mode (toggle with f/p)")
    parser.add_argument("--fixed-threshold", type=float, default=200.0,
                        help="Cutoff on colourised 0-255 intensity when mode is fixed")
    parser.add_argument("--max-detected-markers", type=int, default=4,
                        help="Maximum number of probe markers to detect and save per frame")
    parser.add_argument("--percentile", type=float, default=90.0,
                        help="Percentile of colourised intensity when mode is percentile")
    parser.add_argument("--percentile-floor", type=float, default=100.0,
                        help="Absolute 0-255 floor combined with percentile cutoff")
    parser.add_argument(
        "--filter-geometry",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Drop isolated / spatially implausible candidates before PnP (default: on)",
    )
    parser.add_argument(
        "--min-marker-area",
        type=int,
        default=MARKER_MIN_AREA,
        help="Minimum connected-component area in pixels",
    )
    parser.add_argument(
        "--max-marker-area",
        type=int,
        default=MARKER_MAX_AREA,
        help="Absolute max blob area; also raised to ~8%% of the frame for close-ups",
    )
    parser.add_argument(
        "--max-marker-aspect",
        type=float,
        default=MARKER_MAX_ASPECT,
        help="Max width/height ratio for a blob (looser helps bloomed close markers)",
    )
    parser.add_argument(
        "--max-nearest-neighbor-px",
        type=float,
        default=None,
        help="Isolation gate in pixels (default: max(90, 0.35*min(h,w)))",
    )
    parser.add_argument(
        "--max-cluster-span-px",
        type=float,
        default=None,
        help="Max cluster diameter in pixels (default: max(220, 0.85*min(h,w)))",
    )
    parser.add_argument(
        "--min-cluster-size",
        type=int,
        default=MARKER_MIN_CLUSTER_SIZE,
        help="Minimum detections required after geometric filtering",
    )
    parser.add_argument(
        "--max-geometry-ratio-error",
        type=float,
        default=MARKER_GEOMETRY_RATIO_ERROR,
        help="Max relative error on scale-normalized pairwise distance signature vs model",
    )
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--frame-timeout", type=float, default=10.0,
                        help="Reconnect if no complete frame data arrives for this many seconds")
    parser.add_argument("--tracking-lost-timeout", type=float, default=0.65,
                        help="Seconds since last accepted source observation before clearing the prior")
    parser.add_argument("--acquisition-timeout", type=float, default=0.65,
                        help="Maximum gap between acquisition confirmations (supports 5 Hz capture)")
    parser.add_argument("--max-processing-age", type=float, default=0.10,
                        help="Drop poses older than this many seconds since complete PC receive")
    parser.add_argument("--retry-delay", type=float, default=2.0)
    parser.add_argument("--no-reconnect", action="store_false", dest="reconnect")
    parser.set_defaults(reconnect=True)
    args = parser.parse_args()
    if not np.isfinite(args.diagnostics_interval) or args.diagnostics_interval < 0:
        parser.error("--diagnostics-interval must be nonnegative and finite")
    if not args.depth_vertically_flipped and not args.convert_object_axes:
        parser.error("--no-depth-vertically-flipped requires --convert-object-axes; otherwise the result is a reflection")
    for name in ("tracking_lost_timeout", "acquisition_timeout", "max_processing_age"):
        if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    return args


if __name__ == "__main__":
    try:
        run(parse_args())
    finally:
        cv2.destroyAllWindows()
