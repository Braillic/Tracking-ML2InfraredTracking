#!/usr/bin/env python3
"""Receive Magic Leap 2 FLOAT32 depth frames and display them with OpenCV."""

from __future__ import annotations

import argparse
import socket
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


MAGIC = b"ML2D"
PROTOCOL_VERSION = 3
PIXEL_FORMAT_FLOAT32_METRES = 1
HEADER = struct.Struct("<4sHHIIIIQd3f4fI")
INTRINSICS = struct.Struct("<11d")
MAX_PAYLOAD_BYTES = 128 * 1024 * 1024
WINDOW_TITLE = "Magic Leap 2 depth"
ANALYSIS_WINDOW_TITLE = "Analysis (f=fixed, p=percentile)"
MARKER_MIN_AREA = 2
MARKER_MAX_AREA = 2000
MARKER_MAX_ASPECT = 4.0
MARKER_RING_RADIUS = 10
MARKER_MIN_AREA = 2
MARKER_MAX_AREA = 2000
MARKER_MAX_ASPECT = 4.0
MARKER_RING_RADIUS = 10


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
    """One synchronized ML depth frame and its Unity-world sensor pose."""

    depth_metres: np.ndarray
    frame_id: int
    timestamp: float
    sensor_position: np.ndarray  # xyz metres, Unity world coordinates
    sensor_rotation: np.ndarray  # xyzw quaternion, Unity world coordinates
    intrinsics: CameraIntrinsics | None  # present once per TCP connection


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
    values = HEADER.unpack(receive_exact(connection, HEADER.size))
    (magic, version, header_size, width, height, pixel_format, payload_size,
     frame_id, timestamp, px, py, pz, qx, qy, qz, qw, intrinsics_size) = values

    if magic != MAGIC:
        raise ValueError(f"Unexpected stream magic {magic!r}")
    if version != PROTOCOL_VERSION or header_size != HEADER.size:
        raise ValueError(f"Unsupported protocol version/header: {version}/{header_size}")
    if pixel_format != PIXEL_FORMAT_FLOAT32_METRES:
        raise ValueError(f"Unsupported pixel format {pixel_format}")
    expected_size = width * height * np.dtype("<f4").itemsize
    if payload_size != expected_size or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError(
            f"Invalid payload: {payload_size} bytes for {width}x{height} FLOAT32"
        )
    if intrinsics_size not in (0, INTRINSICS.size):
        raise ValueError(
            f"Invalid intrinsics block size: {intrinsics_size}; expected 0 or {INTRINSICS.size}"
        )

    payload = receive_exact(connection, payload_size)
    depth_metres = np.frombuffer(payload, dtype="<f4").reshape(height, width)
    intrinsics = (CameraIntrinsics(*INTRINSICS.unpack(receive_exact(connection, intrinsics_size)))
                  if intrinsics_size else None)
    return DepthFrame(
        depth_metres=depth_metres,
        frame_id=frame_id,
        timestamp=timestamp,
        sensor_position=np.array((px, py, pz), dtype=np.float32),
        sensor_rotation=np.array((qx, qy, qz, qw), dtype=np.float32),
        intrinsics=intrinsics,
    )


def colourise_depth(depth: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    finite = np.isfinite(depth)

    if args.view == "unity-raw":
        # Exact CPU equivalent of DepthSensorShader.shader for _Buffer == 0:
        # saturate((depth - _RawMin) / (_RawMax - _RawMin)).
        normalized = ((np.where(finite, depth, args.raw_min) - args.raw_min) /
                      max(args.raw_max - args.raw_min, 1e-6))
        normalized = np.clip(normalized, 0.0, 1.0)
        if args.unity_color_space == "linear":
            # The project uses Linear color space. Unity converts the shader's
            # linear output to sRGB, while OpenCV byte images are already sRGB.
            normalized = np.where(
                normalized <= 0.0031308,
                normalized * 12.92,
                1.055 * np.power(normalized, 1.0 / 2.4) - 0.055,
            )
        grey = np.rint(normalized * 255.0).astype(np.uint8)
        grey[~finite] = 0
        return cv2.cvtColor(grey, cv2.COLOR_GRAY2BGR)

    valid = finite & (depth > 0)
    clipped = np.clip(np.where(valid, depth, args.near), args.near, args.far)
    grey = ((clipped - args.near) *
            (255.0 / max(args.far - args.near, 1e-6))).astype(np.uint8)
    grey[~valid] = 0
    colour = cv2.applyColorMap(255 - grey, cv2.COLORMAP_TURBO)
    colour[~valid] = 0
    return colour

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


def detect_marker_centers(
    colourised: np.ndarray,
    threshold_method: str,
    fixed_threshold: float,
    top_p: tuple[float, float],
) -> tuple[np.ndarray, float, list[tuple[int, int]]]:
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

    mask = (intensity >= cutoff).astype(np.uint8) * 255
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    candidates: list[tuple[float, tuple[int, int]]] = []
    for label in range(1, n_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        aspect = max(width, height) / max(1, min(width, height))

        if area < MARKER_MIN_AREA or area > MARKER_MAX_AREA or aspect > MARKER_MAX_ASPECT:
            continue

        ys, xs = np.where(labels == label)
        weights = intensity[ys, xs]
        if weights.size == 0:
            continue

        weight_sum = float(weights.sum())
        cx = float((xs * weights).sum() / weight_sum)
        cy = float((ys * weights).sum() / weight_sum)
        mean_intensity = float(weights.mean())
        candidates.append((mean_intensity, (int(round(cx)), int(round(cy)))))

    candidates.sort(key=lambda item: item[0], reverse=True)
    centers = [center for _, center in candidates[:5]]
    return thresholded, cutoff, centers


def annotate_markers(image: np.ndarray, centers: list[tuple[int, int]]) -> np.ndarray:
    """Draw a ring and index around each detected marker."""
    overlay = image.copy()
    for index, (x, y) in enumerate(centers):
        cv2.circle(overlay, (x, y), MARKER_RING_RADIUS, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, str(index), (x + 12, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return overlay


def render_analysis(
    colourised: np.ndarray,
    threshold_method: str,
    fixed_threshold: float,
    top_p: tuple[float, float],
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """Build the Analysis window from an already-colourised frame."""
    analysis, cutoff, centers = detect_marker_centers(
        colourised,
        threshold_method,
        fixed_threshold=fixed_threshold,
        top_p=top_p,
    )
    analysis = annotate_markers(analysis, centers)
    if threshold_method == "fixed":
        mode_label = f"fixed >= {fixed_threshold:g}  (cutoff {cutoff:.1f})"
    else:
        mode_label = (
            f"percentile p{top_p[1]:g} floor {top_p[0]:g}  (cutoff {cutoff:.1f})"
        )
    cv2.putText(analysis, mode_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(analysis, "keys: f=fixed  p=percentile  q=quit", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return analysis, centers


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


def run(args: argparse.Namespace) -> None:
    save_directory = Path(args.save_dir) if args.save_dir else None
    if save_directory:
        save_directory.mkdir(parents=True, exist_ok=True)

    threshold_method = args.threshold_method
    fixed_threshold = args.fixed_threshold
    top_p = (args.percentile_floor, args.percentile)

    while True:
        try:
            print(f"Connecting to Magic Leap at {args.host}:{args.port} ...")
            with socket.create_connection((args.host, args.port), timeout=args.connect_timeout) as connection:
                connection.settimeout(args.frame_timeout)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print("TCP connected; waiting for the first depth frame header...", flush=True)
                show_waiting_window(args.host, args.port)
                first_frame = True
                intrinsics = None

                while True:
                    frame = receive_frame(connection)
                    depth = frame.depth_metres
                    if first_frame:
                        print(f"First frame received: {depth.shape[1]}x{depth.shape[0]}, float32 metres",
                              flush=True)
                        first_frame = False
                    if frame.intrinsics is not None:
                        intrinsics = frame.intrinsics
                        print("Intrinsics received once:", intrinsics, flush=True)
                        print("OpenCV camera matrix:\n", intrinsics.camera_matrix, flush=True)
                        print("OpenCV distortion coefficients:",
                              intrinsics.distortion_coefficients, flush=True)

                    display = colourise_depth(depth, args)
                    analysis, centers = render_analysis(
                        display, threshold_method, fixed_threshold, top_p
                    )
                    display = annotate_markers(display, centers)

                    valid = np.isfinite(depth) & (depth > 0)
                    if np.any(valid):
                        minimum = float(depth[valid].min())
                        maximum = float(depth[valid].max())
                        unit = "raw" if args.view == "unity-raw" else "m"
                        label = f"frame {frame.frame_id}  range {minimum:.2f}-{maximum:.2f} {unit}"
                    else:
                        label = f"frame {frame.frame_id}  no valid depth"
                    cv2.putText(display, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 255, 255), 1, cv2.LINE_AA)
                    p = frame.sensor_position
                    q = frame.sensor_rotation
                    pose_label = (f"pose P=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})m "
                                  f"Q=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})")
                    cv2.putText(display, pose_label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                0.45, (255, 255, 255), 1, cv2.LINE_AA)

                    cv2.imshow(WINDOW_TITLE, display)
                    cv2.imshow(ANALYSIS_WINDOW_TITLE, analysis)

                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        return
                    if key == ord("f"):
                        threshold_method = "fixed"
                        print("Analysis: fixed threshold", flush=True)
                        analysis, centers = render_analysis(
                            colourise_depth(depth, args),
                            threshold_method, fixed_threshold, top_p,
                        )
                        cv2.imshow(ANALYSIS_WINDOW_TITLE, analysis)
                    elif key == ord("p"):
                        threshold_method = "percentile"
                        print("Analysis: percentile threshold", flush=True)
                        analysis, centers = render_analysis(
                            colourise_depth(depth, args),
                            threshold_method, fixed_threshold, top_p,
                        )
                        cv2.imshow(ANALYSIS_WINDOW_TITLE, analysis)
                    elif key == ord("s"):
                        # prompt for save name
                        save_name = input("Enter save name: ")
                        if not save_name:
                            print("Save name cannot be empty")
                            continue
                        target = ((save_directory or Path.cwd()) /
                                  f"ml2_depth_{save_name}.npy")
                        np.save(target, depth)
                        display_target = ((save_directory or Path.cwd()) /
                                  f"ml2_display_{save_name}.png")
                        cv2.imwrite(display_target, display)
                        analysis_target = ((save_directory or Path.cwd()) /
                                  f"ml2_analysis_{save_name}.npy")
                        np.save(f"{analysis_target}.npy", analysis)
                        cv2.imwrite(f"{analysis_target}.png", analysis)
                        print(f"Saved {target}")
                        print(f"Saved {display_target}")
                        print(f"Saved {analysis_target}")

        except (ConnectionError, OSError, ValueError) as error:
            if not args.reconnect:
                raise
            print(f"Stream unavailable: {error}. Retrying in {args.retry_delay:g}s.")
            time.sleep(args.retry_delay)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="ML Wi-Fi IP, or 127.0.0.1 when using adb forward")
    parser.add_argument("--port", type=int, default=50777)
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
                        default="percentile",
                        help="Initial Analysis-window threshold mode (toggle with f/p)")
    parser.add_argument("--fixed-threshold", type=float, default=200.0,
                        help="Cutoff on colourised 0-255 intensity when mode is fixed")
    parser.add_argument("--percentile", type=float, default=90.0,
                        help="Percentile of colourised intensity when mode is percentile")
    parser.add_argument("--percentile-floor", type=float, default=100.0,
                        help="Absolute 0-255 floor combined with percentile cutoff")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    parser.add_argument("--frame-timeout", type=float, default=10.0,
                        help="Reconnect if no complete frame data arrives for this many seconds")
    parser.add_argument("--retry-delay", type=float, default=2.0)
    parser.add_argument("--no-reconnect", action="store_false", dest="reconnect")
    parser.set_defaults(reconnect=True)
    return parser.parse_args()


if __name__ == "__main__":
    try:
        run(parse_args())
    finally:
        cv2.destroyAllWindows()
