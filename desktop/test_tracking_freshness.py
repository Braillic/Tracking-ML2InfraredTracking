"""Motion compensation, angular rejection, slow capture, and bounded reception."""
import socket
import time
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

import cv2
import depth_stream_receiver as receiver
import numpy as np
from marker_pose import TEST_MARKER_COORDS, MarkerPoseTracker, PoseEstimate
from pose_packet import unity_trs_matrix
from test_tracking_recovery import RVEC, K, points, pose
from test_transport_pipelines import packet


def camera(angle=0., x=0.):
    result = np.eye(4)
    result[:3, :3] = cv2.Rodrigues(np.array([0., 0., np.radians(angle)]))[0]
    result[0, 3] = x
    return result


def relative_pose(world_pose, world_camera):
    world_tool = np.eye(4)
    world_tool[:3, :3] = cv2.Rodrigues(world_pose.rvec)[0]
    world_tool[:3, 3] = world_pose.tvec.ravel()
    relative = np.linalg.inv(world_camera) @ world_tool
    return replace(world_pose, rvec=cv2.Rodrigues(relative[:3, :3])[0],
                   tvec=relative[:3, 3:4])


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.tracker = MarkerPoseTracker(TEST_MARKER_COORDS)
        self.search = patch.object(self.tracker, '_estimate_combinatorial', return_value=pose()).start()
        self.temporal = patch.object(self.tracker, '_estimate_temporal', return_value=PoseEstimate.failed()).start()
        self.addCleanup(patch.stopall)

    def observe(self, t, cam=None, pts=None):
        return self.tracker.estimate(points() if pts is None else pts, K,
                                     observation_time=t, camera_world_transform=cam)

    def acquire(self, cam=None):
        self.assertFalse(self.observe(1., cam).ok)
        self.assertTrue(self.observe(1.033, cam).ok)

    def test_rotation_only_flip_rejected_in_both_paths(self):
        self.acquire()
        flipped = replace(pose(), rvec=cv2.Rodrigues(
            cv2.Rodrigues(np.array([0., 0., np.pi]))[0] @ cv2.Rodrigues(RVEC)[0])[0])
        self.search.return_value = self.temporal.return_value = flipped
        self.assertFalse(self.observe(1.066).ok)
        np.testing.assert_allclose(self.tracker._prev_rvec.ravel(), RVEC)
        self.assertEqual(self.tracker._last_accepted_time, 1.033)

    def test_angular_limit_grows_with_time_but_stays_capped(self):
        self.acquire()
        changed = replace(pose(), rvec=cv2.Rodrigues(
            camera(40)[:3, :3] @ cv2.Rodrigues(RVEC)[0])[0])
        self.search.return_value = changed
        self.assertFalse(self.observe(1.05).ok)
        self.assertTrue(self.observe(1.15).ok)
        self.search.return_value = replace(pose(), rvec=cv2.Rodrigues(
            camera(140)[:3, :3] @ cv2.Rodrigues(RVEC)[0])[0])
        self.assertFalse(self.observe(1.75).ok)  # 100 degrees still exceeds the cap

    def test_solver_checks_all_branches_before_ranking(self):
        self.acquire()
        self.tracker._last_observation_time = 1.066
        bad = cv2.Rodrigues(camera(180)[:3, :3] @ cv2.Rodrigues(RVEC)[0])[0]
        good = pose()
        with patch('marker_pose.cv2.solvePnPGeneric', return_value=(True, [bad, good.rvec],
                                                                   [good.tvec, good.tvec], None)), \
             patch('marker_pose._refine_pose_and_reassign',
                   side_effect=lambda model, image, r, t, mi, ii, *a, **kw: (r, t, mi, ii)), \
             patch('marker_pose._mean_reprojection_error', return_value=0.), \
             patch('marker_pose._model_support', return_value=4), \
             patch('marker_pose._ranking_cost', return_value=0.):
            result = self.tracker._solve_and_score(points(), K, np.zeros(5), tuple(range(4)),
                                                   tuple(range(4)), 4, use_extrinsic_guess=False)
        self.assertTrue(result.ok)
        np.testing.assert_allclose(result.rvec, good.rvec)

    def test_large_head_motion_during_acquisition_and_tracking(self):
        self.assertFalse(self.observe(1., camera()).ok)
        self.search.return_value = relative_pose(pose(), camera(100, .25))
        self.assertTrue(self.observe(1.21, camera(100, .25)).ok)
        # Move both headset and probe. Only the probe's 1 cm and 8 degree
        # world change should consume the jump gate, not the 100 degree head turn.
        moving = replace(pose(.01), rvec=cv2.Rodrigues(
            camera(8)[:3, :3] @ cv2.Rodrigues(RVEC)[0])[0])
        self.search.return_value = relative_pose(moving, camera(200, -.1))
        self.assertTrue(self.observe(1.243, camera(200, -.1)).ok)

    def test_empty_frame_rebases_prior_without_refreshing_acceptance(self):
        self.acquire(camera())
        self.assertFalse(self.observe(1.1, camera(80, .3), []).ok)
        self.assertEqual(self.tracker._last_accepted_time, 1.033)
        self.search.return_value = relative_pose(pose(), camera(160, -.2))
        self.assertTrue(self.observe(1.2, camera(160, -.2)).ok)

    def test_defaults_acquire_and_continue_at_five_hz_with_jitter(self):
        for period in (.20, .21, .25):
            with self.subTest(period=period):
                self.tracker.reset()
                results = [self.observe(1. + i * period).ok for i in range(20)]
                self.assertEqual(results, [False] + [True] * 19)

    def test_loss_still_expires_and_requires_new_confirmation(self):
        self.acquire()
        self.assertFalse(self.observe(1.69, pts=[]).ok)
        self.assertIsNone(self.tracker._prev_rvec)
        self.search.return_value = pose(.3)
        self.assertFalse(self.observe(1.9).ok)
        self.assertTrue(self.observe(2.11).ok)

    def test_acquisition_has_its_own_timeout(self):
        self.tracker.lost_timeout_s = .1
        self.assertFalse(self.observe(1.).ok)
        self.assertTrue(self.observe(1.21).ok)
        self.tracker.reset()
        self.assertFalse(self.observe(2.).ok)
        self.assertFalse(self.observe(2.7).ok)

    def test_rebased_prior_drives_real_temporal_pnp(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS, search_budget_ms=1000.)
        for index, cam in enumerate((camera(), camera(), camera(90, .12))):
            expected = relative_pose(pose(), cam)
            detections = cv2.projectPoints(TEST_MARKER_COORDS, expected.rvec, expected.tvec, K, None)[0]
            actual = tracker.estimate(detections.reshape(-1, 2), K, observation_time=1 + index * .21,
                                      camera_world_transform=cam)
            if index:
                self.assertTrue(actual.ok)
                np.testing.assert_allclose(actual.tvec, expected.tvec, atol=1e-5)
                np.testing.assert_allclose(cv2.Rodrigues(actual.rvec)[0],
                                           cv2.Rodrigues(expected.rvec)[0], atol=1e-4)


class LatestReceiverTests(unittest.TestCase):
    def connect(self):
        reader, writer = socket.socketpair()
        reader.settimeout(3)
        writer.settimeout(3)
        self.addCleanup(reader.close)
        self.addCleanup(writer.close)
        return reader, writer

    def wait_for_frame(self, source, frame_id):
        with source._condition:
            self.assertTrue(source._condition.wait_for(
                lambda: source._pending and source._pending[0].frame_id == frame_id, timeout=3))

    def test_processing_stall_keeps_only_latest_and_preserves_intrinsics(self):
        reader, writer = self.connect()
        with receiver.LatestFrameReceiver(reader) as source:
            data = b''.join(packet(np.full((2, 3), i, np.uint8), 2, i, i == 1) for i in range(1, 101))
            writer.sendall(data)
            self.wait_for_frame(source, 100)
            frame, received = source.take()
            self.assertEqual(frame.frame_id, 100)
            self.assertEqual(frame.intrinsics.fx, 363)
            self.assertEqual(source.dropped_frames, 99)
            self.assertLessEqual(received, time.perf_counter())
            self.assertIsNone(source._pending)
        self.assertFalse(source._thread.is_alive())

    def test_switch_and_fragmented_frame_stay_aligned(self):
        reader, writer = self.connect()
        with receiver.LatestFrameReceiver(reader) as source:
            writer.sendall(packet(np.ones((2, 3), '<f4'), 1, 1, True))
            self.wait_for_frame(source, 1)
            data = packet(np.full((2, 3), 42, np.uint8), 2, 2)
            writer.sendall(data[:20])
            first, _ = source.take()
            self.assertEqual(first.frame_id, 1)
            writer.sendall(data[20:])
            second, _ = source.take()
            self.assertEqual(second.frame_id, 2)
            self.assertEqual(second.pixel_format, 2)
            np.testing.assert_array_equal(second.pixels, np.full((2, 3), 42, np.uint8))
            self.assertEqual(second.intrinsics, first.intrinsics)

    def test_disconnect_propagates_and_reader_stops(self):
        reader, writer = self.connect()
        with receiver.LatestFrameReceiver(reader) as source:
            writer.close()
            with self.assertRaises(ConnectionError):
                source.take()
        self.assertFalse(source._thread.is_alive())

    def test_shutdown_interrupts_partial_frame(self):
        reader, writer = self.connect()
        with receiver.LatestFrameReceiver(reader) as source:
            writer.sendall(b'ML2D')
        self.assertFalse(source._thread.is_alive())

    def test_malformed_header_propagates(self):
        reader, writer = self.connect()
        with receiver.LatestFrameReceiver(reader) as source:
            writer.sendall(b'BAD!' + bytes(receiver.HEADER.size - 4))
            with self.assertRaises(ValueError):
                source.take()


class ProcessingAgeTests(unittest.TestCase):
    def setUp(self):
        self.tracker = Mock()
        self.tracker.estimate.return_value = pose()
        self.intrinsics = receiver.CameraIntrinsics(363, 363, 272, 240, 1, 1, 0, 0, 0, 0, 0)
        self.frame = receiver.DepthFrame(np.zeros((480, 544), np.uint8), 7, 12.5,
                                         np.zeros(3), np.array([0, 0, 0, 1]), self.intrinsics)
        self.connection = Mock()

    def estimate(self):
        return receiver.estimate_and_send_pose(self.tracker, [], self.frame, self.intrinsics,
                                               self.connection, recv_done=1., max_processing_age_s=.1)

    def test_stale_before_solve_does_not_touch_prior_or_send(self):
        with patch('depth_stream_receiver.time.perf_counter', return_value=1.2):
            estimate, _ = self.estimate()
        self.assertFalse(estimate.ok)
        self.tracker.estimate.assert_not_called()
        self.connection.sendall.assert_not_called()

    def test_expiring_during_solve_drops_result_and_clears_prior(self):
        with patch('depth_stream_receiver.time.perf_counter', side_effect=[1.01, 1.2, 1.2, 1.2]):
            estimate, _ = self.estimate()
        self.assertFalse(estimate.ok)
        self.tracker.reset.assert_called_once()
        self.connection.sendall.assert_not_called()

    def test_fresh_pose_is_sent_and_camera_basis_matches_world_conversion(self):
        for flipped, convert in ((True, False), (True, True), (False, True)):
            with self.subTest(flipped=flipped, convert=convert):
                self.connection.reset_mock()
                with patch('depth_stream_receiver.time.perf_counter', return_value=1.02):
                    estimate, _ = receiver.estimate_and_send_pose(
                        self.tracker, [], self.frame, self.intrinsics, self.connection,
                        recv_done=1., depth_vertically_flipped=flipped, convert_object_axes=convert)
                self.assertTrue(estimate.ok)
                self.connection.sendall.assert_called_once()
                expected = unity_trs_matrix(self.frame.sensor_position, self.frame.sensor_rotation)
                if not flipped or convert:
                    expected[:3, 1] *= -1
                np.testing.assert_allclose(self.tracker.estimate.call_args.kwargs['camera_world_transform'], expected)


if __name__ == '__main__':
    unittest.main()
