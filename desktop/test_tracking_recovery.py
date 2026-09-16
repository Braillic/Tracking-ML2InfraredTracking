"""Regression tests: python -m unittest discover -s desktop -p test_tracking_recovery.py"""
import itertools
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from depth_stream_receiver import (
    CameraIntrinsics,
    DepthFrame,
    detect_marker_centers,
    estimate_and_send_pose,
)
from marker_pose import (
    TEST_MARKER_COORDS,
    MarkerPoseTracker,
    PoseEstimate,
    _assign_mutual_nearest,
)

K = np.array([[363., 0., 272.], [0., 363., 240.], [0., 0., 1.]])
RVEC = np.array([.1, .15, .05])


def points(x=0.0):
    return cv2.projectPoints(TEST_MARKER_COORDS, RVEC, np.array([x, 0., .5]), K, None)[0].reshape(-1, 2)


def pose(x=0.0, n=4):
    return PoseEstimate(True, RVEC.reshape(3, 1), np.array([[x], [0.], [.5]]),
                        .95, .1, tuple(range(n)), tuple(range(n)), n, n, n, 0, 1.)


class AssignmentTests(unittest.TestCase):
    def test_all_detection_orders_preserve_identity(self):
        projected = points()
        for order in itertools.permutations(range(4)):
            detections = projected[list(order)]
            mi, ii = _assign_mutual_nearest(projected, detections, 40.)
            self.assertEqual(mi, (0, 1, 2, 3))
            np.testing.assert_allclose(detections[list(ii)], projected)

    def test_maximum_cardinality_before_distance_and_allows_missing(self):
        # Greedy picks 0 -> 1 and loses the second match. Global assignment gets both.
        mi, ii = _assign_mutual_nearest(
            np.array([[0., 0.], [2., 0.], [100., 0.]]),
            np.array([[1., 0.], [-1., 0.], [300., 0.]]), 1.1)
        self.assertEqual(mi, (0, 1))
        self.assertEqual(ii, (1, 0))

    def test_detection_keeps_dim_markers_beside_bright_clutter(self):
        image = np.zeros((480, 544, 3), dtype=np.uint8)
        locations = [(150, 150), (200, 150), (250, 150), (300, 150),
                     (150, 230), (200, 230), (250, 230), (300, 230)]
        for i, xy in enumerate(locations):
            cv2.circle(image, xy, 4, (250 if i < 4 else 210,) * 3, -1)
        _, _, centres = detect_marker_centers(image, "fixed", 200., (100., 90.))
        self.assertEqual(set(centres), set(locations))


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.tracker = MarkerPoseTracker(TEST_MARKER_COORDS, lost_timeout_s=.20)
        self.search = patch.object(self.tracker, "_estimate_combinatorial", return_value=pose()).start()
        self.temporal = patch.object(self.tracker, "_estimate_temporal", return_value=PoseEstimate.failed()).start()
        self.addCleanup(patch.stopall)

    def observe(self, timestamp, detections=None):
        return self.tracker.estimate(points() if detections is None else detections,
                                     K, observation_time=timestamp)

    def acquire(self):
        self.assertFalse(self.observe(1.).ok)
        self.assertTrue(self.observe(1.033).ok)
        self.assertEqual(self.tracker.state, "tracking")

    def test_empty_frames_expire_prior_and_reacquire_at_new_position(self):
        self.acquire()
        self.observe(1.1, [])
        self.observe(1.25, [])
        self.assertEqual(self.tracker.state, "lost")
        self.assertIsNone(self.tracker._prev_tvec)
        self.search.return_value = pose(.20)  # well beyond the former 6 cm gate
        self.assertFalse(self.observe(1.28).ok)
        recovered = self.observe(1.313)
        self.assertTrue(recovered.ok)
        self.assertAlmostEqual(recovered.tvec[0, 0], .20)

    def test_gap_without_empty_frames_also_expires_prior(self):
        self.acquire()
        self.search.return_value = pose(.20)
        self.assertFalse(self.observe(1.4).ok)
        self.assertIsNone(self.tracker._prev_tvec)
        self.assertTrue(self.observe(1.433).ok)

    def test_rejections_age_out_instead_of_requiring_original_alignment(self):
        self.acquire()
        self.search.return_value = pose(.30)
        for stamp in (1.06, 1.10, 1.15, 1.20):
            self.assertFalse(self.observe(stamp).ok)
        self.assertFalse(self.observe(1.25).ok)
        self.assertTrue(self.observe(1.283).ok)

    def test_gate_depends_on_time_since_acceptance(self):
        self.acquire()
        self.search.return_value = pose(.08)
        self.assertFalse(self.observe(1.05).ok)  # .032 m gate
        self.assertTrue(self.observe(1.15).ok)  # .132 m gate

    def test_acquisition_requires_consecutive_four_point_confirmation(self):
        self.observe(1.)
        self.observe(1.03, [])
        self.assertFalse(self.observe(1.06).ok)
        self.search.return_value = pose(.25)  # inconsistent second observation
        self.assertFalse(self.observe(1.09).ok)
        self.assertTrue(self.observe(1.12).ok)
        self.tracker.reset()
        self.search.return_value = pose(n=3)
        self.assertFalse(self.observe(2.).ok)
        self.assertFalse(self.observe(2.03).ok)

    def test_duplicate_and_out_of_order_observations_cannot_confirm(self):
        self.observe(1.)
        self.assertFalse(self.observe(1.).ok)
        self.assertFalse(self.observe(.9).ok)
        self.assertTrue(self.observe(1.03).ok)
        self.assertEqual(self.search.call_count, 2)

    def test_receiver_passes_empty_observations_and_source_time(self):
        frame = DepthFrame(np.zeros((480, 544), np.uint8), 1, 12.5,
                           np.zeros(3), np.array([0, 0, 0, 1]), None)
        intrinsics = CameraIntrinsics(363, 363, 272, 240, 1, 1, 0, 0, 0, 0, 0)
        with patch.object(self.tracker, "estimate", return_value=PoseEstimate.failed()) as estimate:
            estimate_and_send_pose(self.tracker, [], frame, intrinsics, None)
            self.assertEqual(estimate.call_args.args[0], [])
            self.assertEqual(estimate.call_args.kwargs["observation_time"], 12.5)


class SolverIntegrationTests(unittest.TestCase):
    def test_real_pnp_with_shuffling_clutter_and_reacquisition(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS, search_budget_ms=1000., lost_timeout_s=.20)
        clutter = np.array([[100., 100.], [130., 150.], [400., 350.], [430., 300.]])
        detections = np.vstack((clutter, points()))
        self.assertFalse(tracker.estimate(detections, K, observation_time=1.).ok)
        acquired = tracker.estimate(detections[::-1], K, observation_time=1.033)
        self.assertTrue(acquired.ok)
        np.testing.assert_allclose(acquired.tvec.ravel(), [0., 0., .5], atol=.002)
        tracked = tracker.estimate(detections[[4, 2, 6, 0, 7, 1, 5, 3]], K, observation_time=1.066)
        self.assertTrue(tracked.ok)
        tracker.estimate([], K, observation_time=1.4)
        self.assertFalse(tracker.estimate(points(.15), K, observation_time=1.433).ok)
        recovered = tracker.estimate(points(.15)[::-1], K, observation_time=1.466)
        self.assertTrue(recovered.ok)
        np.testing.assert_allclose(recovered.tvec.ravel(), [.15, 0., .5], atol=.002)

    def test_search_hypothesis_cap(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS, max_search_hypotheses=3,
                                    search_budget_ms=1000.)
        with patch.object(tracker, "_solve_and_score", return_value=PoseEstimate.failed()) as solve:
            tracker.estimate(points(), K, observation_time=1.)
            self.assertEqual(solve.call_count, 3)

    def test_search_time_budget_stops_between_solver_calls(self):
        tracker = MarkerPoseTracker(TEST_MARKER_COORDS)
        with patch.object(tracker, "_solve_and_score", return_value=PoseEstimate.failed()) as solve:
            with patch("marker_pose.time.perf_counter", side_effect=[0., .013]):
                tracker.estimate(points(), K, observation_time=1.)
            self.assertEqual(solve.call_count, 1)


if __name__ == "__main__":
    unittest.main()
