"""Decode the actual C# writer's bytes with the production Python receiver."""
import io
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, sys.argv[1])
from depth_stream_receiver import receive_frame, raw_to_uint8_srgb

class Stream:
    def __init__(self, data): self.data = io.BytesIO(data)
    def recv_into(self, view): return self.data.readinto(view[:7])

for name in ('float.bin', 'uint8.bin'):
    frame = receive_frame(Stream((Path(sys.argv[2]) / name).read_bytes()))
    assert frame.session_id == 99 and frame.frame_id == 7
    assert frame.capture_time_ns == 1234567890123456
    assert frame.capture_realtime == 4.1 and frame.frame_ready_realtime == 4.12
    expected = np.array([[5., 123., 3000.]], dtype=np.float32)
    if name == 'uint8.bin':
        expected = raw_to_uint8_srgb(expected, 5., 3000., linear_to_srgb=True)
    np.testing.assert_array_equal(frame.pixels, expected)
print('C# -> Python FLOAT32/UINT8 framing, capture identity, and mapping checks passed.')
