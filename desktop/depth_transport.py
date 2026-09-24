"""TCP framing and bounded latest-frame reception; no tracking or preview."""
from __future__ import annotations
import socket
import struct
import threading
import time
from dataclasses import replace
import numpy as np
from tracking_types import (CameraIntrinsics, DepthFrame, PIXEL_FORMAT_FLOAT32_RAW,
                            PIXEL_FORMAT_UINT8_SRGB_INTENSITY)

MAGIC = b"ML2D"
PROTOCOL_VERSION = 5
HEADER = struct.Struct("<4sHHIIIIQd3f4fI")
TIMING_HEADER = struct.Struct("<Qqddd")  # session, capture XR ns, mapped capture, poll start, frame ready
INTRINSICS = struct.Struct("<11d")
MAX_PAYLOAD_BYTES = 128 * 1024 * 1024
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

