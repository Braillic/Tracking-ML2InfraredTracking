"""Minimal PC → Magic Leap pose packet (companion to PoseEstimateTcpServer.cs).

Wire format (little-endian, fixed 80 bytes), magic ML2P / version 3.
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass

import cv2
import numpy as np

POSE_MAGIC = b"ML2P"
POSE_PROTOCOL_VERSION = 3
POSE_PACKET_SIZE = 80
POSE_HEADER = struct.Struct("<4sHHQIfffffffffffdd")  # 80 bytes
DEFAULT_POSE_PORT = 50778

# One-off NTP-style clock-sync exchange over the same pose socket, so PC-side
# perf_counter() timestamps can be translated into ML2's Time.realtimeSinceStartupAsDouble
# domain and directly compared against ML2's own send/receive timestamps. This is what
# lets the ML2-side latency log split the round trip into its two network legs instead
# of one lumped bucket — see sync_clock_offset() below.
SYNC_REQUEST_MAGIC = b"ML2S"
SYNC_REPLY_MAGIC = b"ML2R"
SYNC_REQUEST_HEADER = struct.Struct("<4sHHd64x")  # 80 bytes
SYNC_REPLY_HEADER = struct.Struct("<4sHHdd56x")  # 80 bytes
assert SYNC_REQUEST_HEADER.size == POSE_PACKET_SIZE
assert SYNC_REPLY_HEADER.size == POSE_PACKET_SIZE

# OpenCV camera (X right, Y down, Z forward) → Unity camera (X right, Y up, Z forward).
# Only apply this when the depth image is NOT vertically flipped. Magic Leap's
# shouldFlipTexture=true already swaps the image so OpenCV +Y aligns with Unity +Y;
# applying C on top of that mirrors motion.
_OPENCV_TO_UNITY_CAM = np.diag([1.0, -1.0, 1.0]).astype(np.float64)

assert POSE_HEADER.size == POSE_PACKET_SIZE


@dataclass(frozen=True)
class PosePacket:
    """Unity-world tool pose for one depth frame.

    detect_ms/pnp_ms/send_ms are PC-side stage durations (perf_counter deltas,
    not wall-clock-synced with the headset) so ML2 can log a fine-grained
    breakdown instead of lumping all PC + network time into one bucket.

    pc_recv_ml2/pc_send_ml2 are PC-side perf_counter() timestamps translated into
    ML2's Time.realtimeSinceStartupAsDouble domain via sync_clock_offset(), so ML2
    can directly diff them against its own sendDone/receivedRealtime timestamps to
    split the round trip into its two network legs instead of one lumped residual.
    """

    frame_id: int
    ok: bool
    confidence: float
    position: tuple[float, float, float]  # xyz metres
    rotation: tuple[float, float, float, float]  # xyzw quaternion
    detect_ms: float = 0.0  # marker detection (threshold + connected components + geometry filter)
    pnp_ms: float = 0.0  # PnP solve
    send_ms: float = 0.0  # world-space transform + struct pack (excludes the sendall syscall itself)
    pc_recv_ml2: float = 0.0  # depth frame fully received, ML2 clock domain
    pc_send_ml2: float = 0.0  # pose packet about to be sent, ML2 clock domain

    @staticmethod
    def rejected(frame_id: int) -> PosePacket:
        return PosePacket(
            frame_id=frame_id,
            ok=False,
            confidence=0.0,
            position=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )


def pack_pose_packet(pose: PosePacket) -> bytes:
    px, py, pz = pose.position
    qx, qy, qz, qw = pose.rotation
    return POSE_HEADER.pack(
        POSE_MAGIC,
        POSE_PROTOCOL_VERSION,
        POSE_PACKET_SIZE,
        int(pose.frame_id),
        1 if pose.ok else 0,
        float(pose.confidence),
        float(px),
        float(py),
        float(pz),
        float(qx),
        float(qy),
        float(qz),
        float(qw),
        float(pose.detect_ms),
        float(pose.pnp_ms),
        float(pose.send_ms),
        float(pose.pc_recv_ml2),
        float(pose.pc_send_ml2),
    )


def _receive_exact(connection: socket.socket, count: int) -> bytes:
    data = bytearray(count)
    view = memoryview(data)
    received = 0
    while received < count:
        chunk_size = connection.recv_into(view[received:])
        if chunk_size == 0:
            raise ConnectionError("Magic Leap closed the connection during clock sync")
        received += chunk_size
    return bytes(data)


def sync_clock_offset(connection: socket.socket) -> tuple[float, float]:
    """One-shot NTP-style exchange to estimate the PC/ML2 clock offset.

    Returns (offset, rtt) in seconds. A PC time.perf_counter() reading t
    corresponds to ML2's Time.realtimeSinceStartupAsDouble as (t + offset).

    Assumes symmetric network latency (typical for same-LAN Wi-Fi); the
    estimate's error is bounded by roughly rtt/2. Call once per connection —
    both clocks are independent monotonic timers with negligible drift over a
    single session, so re-syncing mid-session is unnecessary for diagnostics.
    """
    t0 = time.perf_counter()
    connection.sendall(SYNC_REQUEST_HEADER.pack(SYNC_REQUEST_MAGIC, POSE_PROTOCOL_VERSION, POSE_PACKET_SIZE, t0))
    reply = _receive_exact(connection, POSE_PACKET_SIZE)
    t2 = time.perf_counter()
    magic, _version, _size, _echoed_t0, ml2_t1 = SYNC_REPLY_HEADER.unpack(reply)
    if magic != SYNC_REPLY_MAGIC:
        raise ValueError(f"Unexpected clock sync reply magic {magic!r}")
    offset = ml2_t1 - (t0 + t2) / 2.0
    rtt = t2 - t0
    return offset, rtt


def connect_pose_socket(host: str, port: int = DEFAULT_POSE_PORT, timeout: float = 5.0) -> socket.socket:
    """PC connects to Magic Leap PoseEstimateTcpServer (same pattern as depth client)."""
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return sock


def send_pose(connection: socket.socket, pose: PosePacket) -> None:
    connection.sendall(pack_pose_packet(pose))


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a Unity xyzw quaternion."""
    m = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    quat = np.array((x, y, z, w), dtype=np.float64)
    quat /= max(np.linalg.norm(quat), 1e-12)
    return float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])


def opencv_rvec_tvec_to_cam_T_object(
    rvec: np.ndarray,
    tvec: np.ndarray,
    *,
    depth_vertically_flipped: bool = True,
    convert_object_axes: bool = False,
) -> np.ndarray:
    """OpenCV object→camera (rvec/tvec) → 4x4 Unity camera←object matrix.

    When the streamed depth is vertically flipped (ML shouldFlipTexture=true) and
    cy has been adjusted to match, OpenCV's +Y already points the same way as
    Unity camera +Y — do **not** apply the Y-flip matrix C or motion mirrors.

    Only for an unflipped image is:
        R_u = C @ R_cv ,  t_u = C @ t_cv ,  C = diag(1, -1, 1)

    convert_object_axes conjugates with C as well (C @ R @ C); keep False unless
    model points were authored with a matching Y flip.
    """
    r_cv, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    t_cv = np.asarray(tvec, dtype=np.float64).reshape(3)

    if depth_vertically_flipped:
        # Flipped image + corrected cy: OpenCV frame already matches Unity cam axes.
        r_u = r_cv
        t_u = t_cv
        if convert_object_axes:
            # Still allow an explicit object-axis conjugate if the caller wants it,
            # but use C only on the object side: R' = C @ R @ C, t' = C @ t.
            c = _OPENCV_TO_UNITY_CAM
            r_u = c @ r_cv @ c
            t_u = c @ t_cv
    else:
        c = _OPENCV_TO_UNITY_CAM
        if convert_object_axes:
            r_u = c @ r_cv @ c
            t_u = c @ t_cv
        else:
            r_u = c @ r_cv
            t_u = c @ t_cv

    cam_t_object = np.eye(4, dtype=np.float64)
    cam_t_object[:3, :3] = r_u
    cam_t_object[:3, 3] = t_u
    return cam_t_object


def camera_matrix_for_vertically_flipped_image(
    camera_matrix: np.ndarray, image_height: int
) -> np.ndarray:
    """Adjust cy when depth was flipped for Unity (shouldFlipTexture=true).

    Magic Leap GetSensorData(..., shouldFlipTexture=true) vertically flips the
    plane bytes before we stream them. Intrinsics stay in the unflipped sensor
    frame, so OpenCV PnP needs cy' = H - 1 - cy to match GetSensorPose.
    """
    k = np.asarray(camera_matrix, dtype=np.float64).copy()
    k[1, 2] = float(image_height) - 1.0 - k[1, 2]
    return k


def unity_trs_matrix(position_xyz: np.ndarray, rotation_xyzw: np.ndarray) -> np.ndarray:
    """Build a Unity TRS matrix from position (xyz) and quaternion (xyzw)."""
    x, y, z, w = np.asarray(rotation_xyzw, dtype=np.float64).reshape(4)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    rotation = np.array(
        (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        ),
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    return matrix


def opencv_camera_pose_to_unity_world(
    rvec: np.ndarray,
    tvec: np.ndarray,
    sensor_position: np.ndarray,
    sensor_rotation_xyzw: np.ndarray,
    *,
    depth_vertically_flipped: bool = True,
    convert_object_axes: bool = False,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Compose OpenCV PnP with the depth frame's Unity sensor pose.

    world_T_object = world_T_sensor * cam_T_object
    """
    world_t_sensor = unity_trs_matrix(sensor_position, sensor_rotation_xyzw)
    cam_t_object = opencv_rvec_tvec_to_cam_T_object(
        rvec,
        tvec,
        depth_vertically_flipped=depth_vertically_flipped,
        convert_object_axes=convert_object_axes,
    )
    world_t_object = world_t_sensor @ cam_t_object
    position = (
        float(world_t_object[0, 3]),
        float(world_t_object[1, 3]),
        float(world_t_object[2, 3]),
    )
    rotation = rotation_matrix_to_quaternion_xyzw(world_t_object[:3, :3])
    return position, rotation
