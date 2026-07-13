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


def colourise_depth(depth: np.ndarray, near: float, far: float) -> np.ndarray:
    valid = np.isfinite(depth) & (depth > 0)
    clipped = np.clip(np.where(valid, depth, near), near, far)
    grey = ((clipped - near) * (255.0 / max(far - near, 1e-6))).astype(np.uint8)
    grey[~valid] = 0
    colour = cv2.applyColorMap(255 - grey, cv2.COLORMAP_TURBO)
    colour[~valid] = 0
    return colour


def show_waiting_window(host: str, port: int) -> None:
    waiting = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(waiting, "TCP connected", (40, 130), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (80, 220, 80), 2, cv2.LINE_AA)
    cv2.putText(waiting, f"Waiting for first frame from {host}:{port}", (40, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(waiting, "Check the ML status: queued should become > 0", (40, 230),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1, cv2.LINE_AA)
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
    cv2.imshow(WINDOW_TITLE, waiting)
    cv2.waitKey(1)


def run(args: argparse.Namespace) -> None:
    save_directory = Path(args.save_dir) if args.save_dir else None
    if save_directory:
        save_directory.mkdir(parents=True, exist_ok=True)

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
                    display = colourise_depth(depth, args.near, args.far)
                    valid = np.isfinite(depth) & (depth > 0)
                    if np.any(valid):
                        minimum = float(depth[valid].min())
                        maximum = float(depth[valid].max())
                        label = f"frame {frame.frame_id}  depth {minimum:.2f}-{maximum:.2f} m"
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

                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        return
                    if key == ord("s"):
                        target = ((save_directory or Path.cwd()) /
                                  f"ml2_depth_{frame.frame_id}_{frame.timestamp:.3f}.npy")
                        np.save(target, depth)
                        print(f"Saved {target}")
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
    parser.add_argument("--near", type=float, default=0.2, help="Display near plane in metres")
    parser.add_argument("--far", type=float, default=5.0, help="Display far plane in metres")
    parser.add_argument("--save-dir", help="Directory used when S is pressed")
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
