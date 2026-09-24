"""Minimal PC → Magic Leap pose packet (companion to PoseEstimateTcpServer.cs).

Wire format: ML2P v3 is 80 bytes; v4 appends an 8-byte session ID (88 bytes).
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass

from pose_geometry import (rotation_matrix_to_quaternion_xyzw,
    opencv_rvec_tvec_to_cam_T_object, camera_matrix_for_vertically_flipped_image,
    unity_trs_matrix, opencv_camera_pose_to_unity_world)
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
    detect_ms: float = 0.0  # image preparation + marker detection
    pnp_ms: float = 0.0  # PnP solve
    send_ms: float = 0.0  # world-pose preparation; serialization/send are in the return-leg estimate
    pc_recv_ml2: float = 0.0  # depth frame fully received, ML2 clock domain
    session_id: int = 0  # nonzero uses version 4 with an 8-byte session suffix
    pc_send_ml2: float = 0.0  # pose packet about to be sent, ML2 clock domain

    @staticmethod
    def rejected(frame_id: int, session_id: int = 0) -> PosePacket:
        return PosePacket(
            frame_id=frame_id,
            session_id=session_id,
            ok=False,
            confidence=0.0,
            position=(0.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0, 1.0),
        )


def pack_pose_packet(pose: PosePacket) -> bytes:
    px, py, pz = pose.position
    qx, qy, qz, qw = pose.rotation
    values = np.asarray((*pose.position, *pose.rotation, pose.confidence,
                         pose.detect_ms, pose.pnp_ms, pose.send_ms, pose.pc_recv_ml2, pose.pc_send_ml2))
    if not np.isfinite(values).all():
        raise ValueError("Cannot transmit a nonfinite pose packet")
    if pose.ok and (not 0 <= pose.confidence <= 1 or not np.isclose(np.linalg.norm(pose.rotation), 1., atol=1e-4)):
        raise ValueError("Accepted pose needs valid confidence and unit quaternion")
    payload = POSE_HEADER.pack(
        POSE_MAGIC,
        4 if pose.session_id else POSE_PROTOCOL_VERSION,
        POSE_PACKET_SIZE + (8 if pose.session_id else 0),
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
    return payload + (struct.pack("<Q", pose.session_id) if pose.session_id else b"")


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
