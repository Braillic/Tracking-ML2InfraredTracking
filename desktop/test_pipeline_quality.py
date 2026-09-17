import io
import json
import struct
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock, MagicMock, patch

import cv2
import numpy as np
import depth_stream_receiver as receiver
import pose_packet as wire
from marker_pose import MarkerPoseTracker, TEST_MARKER_COORDS, filter_marker_centers
from replay_recording import load_frames
from test_tracking_recovery import K, points, pose
from test_transport_pipelines import ChunkedSocket, packet


def timing_packet(frame_id=1, session=99, capture_ns=1234567890123456, fmt=2, intrinsics=True):
    pixels = np.arange(6, dtype=np.uint8 if fmt == 2 else '<f4').reshape(2, 3)
    legacy = packet(pixels, fmt, frame_id, intrinsics)
    header = list(receiver.HEADER.unpack(legacy[:receiver.HEADER.size]))
    header[1:3] = [5, receiver.HEADER.size + receiver.TIMING_HEADER.size]
    return receiver.HEADER.pack(*header) + receiver.TIMING_HEADER.pack(
        session, capture_ns, 4.1, 4.11, 4.12) + legacy[receiver.HEADER.size:]


class TimingProtocolTests(unittest.TestCase):
    def test_v5_both_pixel_formats_preserve_exact_capture_and_calibration(self):
        for fmt in (1, 2):
            with self.subTest(fmt=fmt):
                data = timing_packet(fmt=fmt)
                frame = receiver.receive_frame(ChunkedSocket(data))
                self.assertEqual(frame.session_id, 99)
                self.assertEqual(frame.capture_time_ns, 1234567890123456)
                self.assertEqual(frame.observation_time, 1234567890123456 * 1e-9)
                self.assertEqual(frame.capture_realtime, 4.1)
                self.assertEqual(frame.frame_ready_realtime, 4.12)
                self.assertEqual(frame.intrinsics.fx, 363)
                np.testing.assert_array_equal(frame.pixels, np.arange(6).reshape(2, 3))

    def test_mixed_protocol_frames_do_not_lose_framing(self):
        data = timing_packet() + packet(np.ones((2, 3), '<f4'), 1, 2) + timing_packet(3, 100)
        stream = ChunkedSocket(data)
        frames = [receiver.receive_frame(stream) for _ in range(3)]
        self.assertEqual([f.frame_id for f in frames], [1, 2, 3])
        self.assertEqual([f.session_id for f in frames], [99, 0, 100])
        self.assertEqual(frames[1].observation_time, frames[1].timestamp)

    def test_invalid_identity_or_capture_time_is_rejected(self):
        for data in (timing_packet(session=0), timing_packet(capture_ns=0)):
            with self.assertRaises(ValueError):
                receiver.receive_frame(ChunkedSocket(data))

    def test_session_is_echoed_for_accepted_and_rejected_poses(self):
        for ok in (True, False):
            value = replace(wire.PosePacket.rejected(7, 99), ok=ok, confidence=.9 if ok else 0.)
            packed = wire.pack_pose_packet(value)
            self.assertEqual(len(packed), 88)
            self.assertEqual(struct.unpack_from('<HH', packed, 4), (4, 88))
            self.assertEqual(struct.unpack_from('<Q', packed, 80)[0], 99)
        self.assertEqual(len(wire.pack_pose_packet(wire.PosePacket.rejected(7))), 80)

    def test_receiver_uses_capture_interval_not_wall_clock(self):
        frame = receiver.receive_frame(ChunkedSocket(timing_packet()))
        tracker = Mock()
        tracker.estimate.return_value = pose()
        receiver.estimate_and_send_pose(tracker, [], frame, frame.intrinsics, None)
        self.assertEqual(tracker.estimate.call_args.kwargs['observation_time'], frame.observation_time)
        self.assertNotEqual(frame.observation_time, frame.timestamp)

    def test_rejected_observation_sends_failure_without_fake_pose(self):
        frame = receiver.receive_frame(ChunkedSocket(timing_packet()))
        tracker = Mock()
        tracker.estimate.return_value = receiver.PoseEstimate.failed()
        connection = Mock()
        receiver.estimate_and_send_pose(tracker, [], frame, frame.intrinsics, connection)
        packed = connection.sendall.call_args.args[0]
        self.assertEqual(struct.unpack_from('<I', packed, 16)[0], 0)
        self.assertEqual(struct.unpack_from('<Q', packed, 80)[0], 99)


class SubpixelTests(unittest.TestCase):
    def test_weighted_centroid_retains_fractional_pixel(self):
        image = np.zeros((30, 30), np.uint8)
        image[10:12, 10:12] = [[10, 30], [10, 30]]
        _, _, centers = receiver.detect_marker_centers(image, 'fixed', 1., (100., 90.), filter_geometry=False)
        self.assertEqual(centers, [(10.75, 10.5)])
        # Same result for grayscale headless input and BGR preview input.
        _, _, bgr = receiver.detect_marker_centers(np.repeat(image[..., None], 3, 2),
                                                  'fixed', 1., (100., 90.), filter_geometry=False)
        self.assertEqual(bgr, centers)

    def test_geometry_preserves_fractional_points_in_both_modes(self):
        centers = points() + np.array([.125, .375])
        expected = {tuple(p) for p in centers}
        for preserve in (True, False):
            actual = filter_marker_centers(centers, preserve_candidates=preserve,
                                           max_nearest_neighbor_px=200., max_cluster_span_px=300.,
                                           min_cluster_size=3, max_geometry_ratio_error=.35)
            self.assertEqual(set(actual), expected)

    def test_real_pnp_follows_subpixel_motion_without_a_one_pixel_step(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS, search_budget_ms=1000., ranking_budget_ms=1000.)
        tracker.estimate(points(), K, observation_time=1.)
        self.assertTrue(tracker.estimate(points(), K, observation_time=1.033).ok)
        moved = tracker.estimate(points(.0001), K, observation_time=1.066)
        self.assertTrue(moved.ok)
        self.assertAlmostEqual(float(moved.tvec[0, 0]), .0001, places=6)

    def test_recording_metadata_uses_exact_observation_time(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp) / 'centers'
            folder.mkdir()
            path = folder / 'centers_000000_7_100.000.npy'
            expected = np.array([[10.75, 10.5], [20.25, 20.125]])
            np.save(path, expected)
            path.with_suffix('.json').write_text(json.dumps(dict(observation_time=4.123456789,
                                                                  session_id=99)))
            frames = load_frames(Path(temp), with_metadata=True)
            self.assertEqual(frames[0][2], 4.123456789)
            np.testing.assert_array_equal(frames[0][3], expected)
            self.assertEqual(frames[0][4]['session_id'], 99)


class FinalPoseValidationTests(unittest.TestCase):
    def test_invalid_refined_pose_is_rejected(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS)
        for t in (np.array([0., 0., -.5]), np.array([0., 0., 2.]), np.array([np.nan, 0., .5])):
            with self.subTest(t=t), patch('marker_pose._refine_pose_and_reassign',
                    return_value=(pose().rvec, t.reshape(3, 1), tuple(range(4)), tuple(range(4)))):
                estimate = tracker._solve_and_score(points(), K, np.zeros(5), tuple(range(4)),
                                                     tuple(range(4)), 4, use_extrinsic_guess=False)
                self.assertFalse(estimate.ok)

    def test_ranking_has_its_own_deadline(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS)
        with patch('marker_pose.time.perf_counter', side_effect=[0., .013]), \
             patch.object(tracker, '_solve_and_score') as solve:
            self.assertFalse(tracker.estimate(points(), K, observation_time=1.).ok)
        solve.assert_not_called()

    def test_reflection_and_invalid_packet_rejected(self):
        with self.assertRaises(ValueError):
            wire.rotation_matrix_to_quaternion_xyzw(np.diag([1., -1., 1.]))
        with self.assertRaises(ValueError):
            wire.opencv_rvec_tvec_to_cam_T_object(pose().rvec, pose().tvec, depth_vertically_flipped=False)
        for value in (replace(wire.PosePacket.rejected(1), position=(float('nan'), 0., 0.)),
                      replace(wire.PosePacket.rejected(1), ok=True, rotation=(0., 0., 0., 0.))):
            with self.assertRaises(ValueError):
                wire.pack_pose_packet(value)

    def test_supported_axis_mappings_preserve_projected_points(self):
        estimate = pose(.02)
        r = cv2.Rodrigues(estimate.rvec)[0]
        c = np.diag([1., -1., 1.])
        for flipped, convert in ((True, False), (True, True), (False, True)):
            actual = wire.opencv_rvec_tvec_to_cam_T_object(estimate.rvec, estimate.tvec,
                      depth_vertically_flipped=flipped, convert_object_axes=convert)
            self.assertAlmostEqual(np.linalg.det(actual[:3, :3]), 1.)
            source = TEST_MARKER_COORDS[0]
            expected = r @ source + estimate.tvec.ravel() if not convert else c @ (r @ (c @ source) + estimate.tvec.ravel())
            np.testing.assert_allclose(actual[:3, :3] @ source + actual[:3, 3], expected)

    def test_headless_loop_never_calls_gui_or_bgr_preview(self):
        with patch('sys.argv', ['receiver', '--headless', '--diagnostics-interval', '0']):
            args = receiver.parse_args()
        frame = receiver.receive_frame(ChunkedSocket(timing_packet()))
        frames = MagicMock()
        frames.__enter__.return_value = frames
        frames.take.side_effect = [(frame, receiver.time.perf_counter()), KeyboardInterrupt()]
        with patch('depth_stream_receiver.socket.create_connection', return_value=MagicMock()), \
             patch('depth_stream_receiver.LatestFrameReceiver', return_value=frames), \
             patch('depth_stream_receiver.show_waiting_window', side_effect=AssertionError('GUI')), \
             patch('depth_stream_receiver.cv2.imshow', side_effect=AssertionError('GUI')), \
             patch('depth_stream_receiver.prepare_detection_image', side_effect=AssertionError('BGR preview')):
            with self.assertRaises(KeyboardInterrupt):
                receiver.run(args)


if __name__ == '__main__':
    unittest.main()
