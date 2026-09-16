"""Wire decoding, pipeline routing, and equivalent shared detection inputs."""
import io
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import depth_stream_receiver as receiver
import numpy as np

ARGS = SimpleNamespace(view="unity-raw", raw_min=5., raw_max=3000.,
                       unity_color_space="linear", near=.2, far=5.)


class ChunkedSocket:
    def __init__(self, data):
        self.data = io.BytesIO(data)

    def recv_into(self, view):
        return self.data.readinto(view[:11])


def packet(pixels, pixel_format, frame_id=1, intrinsics=False):
    metadata = receiver.INTRINSICS.pack(363, 363, 272, 240, 1, 1, 0, 0, 0, 0, 0) if intrinsics else b""
    return receiver.HEADER.pack(
        receiver.MAGIC, 3 if pixel_format == 1 else 4, receiver.HEADER.size,
        pixels.shape[1], pixels.shape[0], pixel_format, pixels.nbytes,
        frame_id, float(frame_id), 0, 0, 0, 0, 0, 0, 1, len(metadata)
    ) + pixels.tobytes() + metadata


class TransportPipelineTests(unittest.TestCase):
    def test_pipeline1_preserves_float_values_until_colourise(self):
        raw = np.array([[5., 123.456, 3000.], [-1., np.nan, np.inf]], dtype="<f4")
        with patch.object(receiver, "raw_to_uint8_srgb", side_effect=AssertionError("conversion during receive")):
            frame = receiver.receive_frame(ChunkedSocket(packet(raw, 1)))
        self.assertEqual(frame.pixels.dtype, np.dtype("<f4"))
        np.testing.assert_array_equal(frame.pixels, raw)
        with patch.object(receiver, "colourise_depth", wraps=receiver.colourise_depth) as colourise:
            display = receiver.prepare_detection_image(frame, ARGS)
            colourise.assert_called_once()
            self.assertIs(colourise.call_args.args[0], frame.pixels)
        self.assertEqual(display.dtype, np.uint8)
        self.assertEqual(display.shape, (2, 3, 3))

    def test_pipeline2_bypasses_raw_mapping(self):
        grey = np.array([[0, 31, 127, 255]], np.uint8)
        frame = receiver.receive_frame(ChunkedSocket(packet(grey, 2)))
        with patch.object(receiver, "colourise_depth", side_effect=AssertionError("double conversion")):
            display = receiver.prepare_detection_image(frame, ARGS)
        np.testing.assert_array_equal(display, np.repeat(grey[..., None], 3, axis=2))

    def test_runtime_switches_keep_headers_pixels_and_intrinsics_aligned(self):
        arrays = [np.ones((2, 3), "<f4") * 42, np.ones((2, 3), np.uint8) * 127,
                  np.ones((2, 3), "<f4") * 1000]
        socket = ChunkedSocket(b"".join(packet(a, fmt, i + 1, i == 0)
                                        for i, (a, fmt) in enumerate(zip(arrays, (1, 2, 1)))))
        for i, (expected, fmt) in enumerate(zip(arrays, (1, 2, 1))):
            frame = receiver.receive_frame(socket)
            self.assertEqual(frame.pixel_format, fmt)
            self.assertEqual(frame.frame_id, i + 1)
            self.assertEqual(frame.intrinsics is not None, i == 0)
            np.testing.assert_array_equal(frame.pixels, expected)

    def test_both_paths_feed_equivalent_images_and_centres_to_analysis(self):
        raw = np.full((160, 200), 5., np.float32)
        for xy in ((55, 70), (120, 70), (90, 35), (85, 110)):
            cv2.circle(raw, xy, 4, 2800., -1)
        grey = receiver.raw_to_uint8_srgb(raw, 5., 3000., linear_to_srgb=True)
        p1 = receiver.prepare_detection_image(receiver.receive_frame(ChunkedSocket(packet(raw, 1))), ARGS)
        p2 = receiver.prepare_detection_image(receiver.receive_frame(ChunkedSocket(packet(grey, 2))), ARGS)
        np.testing.assert_array_equal(p1, p2)
        result1 = receiver.render_analysis(p1, "fixed", 200., (100., 90.), filter_geometry=False)
        result2 = receiver.render_analysis(p2, "fixed", 200., (100., 90.), filter_geometry=False)
        self.assertEqual(result1[2], result2[2])
        self.assertEqual(len(result1[2]), 4)
        self.assertEqual(result1[3], result2[3])

    def test_wrong_payload_size_is_rejected(self):
        data = bytearray(packet(np.zeros((2, 3), np.uint8), 2))
        # Declare FLOAT32 while leaving the 6-byte image payload unchanged.
        data[16:20] = (1).to_bytes(4, "little")
        with self.assertRaisesRegex(ValueError, "Invalid payload"):
            receiver.receive_frame(ChunkedSocket(data))


if __name__ == "__main__":
    unittest.main()
