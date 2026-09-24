"""Core contract, sequence integrity, live adapter ordering and recorded replay."""
import csv
import json
import tempfile
import unittest
from dataclasses import asdict, replace
from pathlib import Path
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

import depth_stream_receiver as receiver
from benchmark_tracking import benchmark, load_observations
from marker_detection import detect_marker_centers, raw_to_uint8_srgb
from marker_pose import MarkerPoseTracker, TEST_MARKER_COORDS
from pose_geometry import opencv_camera_pose_to_unity_world, pose_camera_matrix, unity_trs_matrix
from tracking_core import TrackingPipeline, TrackingConfig, DetectionConfig
from tracking_types import TrackingFrame, CameraIntrinsics
from test_tracking_recovery import points


def frame(index=1, *, empty=False):
    pixels = np.zeros((480, 544), np.uint8)
    if not empty:
        for x, y in points():
            cv2.circle(pixels, (round(float(x)), round(float(y))), 3, 240, -1)
    return TrackingFrame(pixels, index, index*.05, np.zeros(3), np.array([0.,0.,0.,1.]),
        CameraIntrinsics(363.,363.,272.,239.,1.,1.,0.,0.,0.,0.,0.),
        session_id=17, capture_time_ns=index*50_000_000)


def core(**kwargs):
    # Remove machine-speed-dependent acquisition budgets from numerical assertions.
    tracker = MarkerPoseTracker(TEST_MARKER_COORDS.copy(), search_budget_ms=1000., ranking_budget_ms=1000.)
    return TrackingPipeline(tracker=tracker, **kwargs)


class CoreTests(unittest.TestCase):
    def test_direct_solver_and_core_agree_over_sequence(self):
        pipeline = core()
        direct = MarkerPoseTracker(TEST_MARKER_COORDS.copy(), search_budget_ms=1000., ranking_budget_ms=1000.)
        accepted = 0
        for i in range(1, 8):
            f = frame(i, empty=i == 5)
            result = pipeline.process(f)
            _, _, centers = detect_marker_centers(f.pixels, 'fixed', 200., (100.,90.))
            K = pose_camera_matrix(f.intrinsics, 480, depth_vertically_flipped=True)
            expected = direct.estimate(centers, K, f.intrinsics.distortion_coefficients,
                observation_time=f.observation_time,
                camera_world_transform=unity_trs_matrix(f.sensor_position, f.sensor_rotation))
            self.assertEqual(result.ok, expected.ok)
            self.assertEqual(result.state, direct.state)
            self.assertEqual(list(result.centers), centers)
            if expected.ok:
                accepted += 1
                position, rotation = opencv_camera_pose_to_unity_world(expected.rvec, expected.tvec,
                    f.sensor_position, f.sensor_rotation, depth_vertically_flipped=True)
                np.testing.assert_allclose(result.solution.position, position, atol=1e-12)
                np.testing.assert_allclose(result.solution.rotation, rotation, atol=1e-12)
        self.assertGreater(accepted, 3)

    def test_uint8_input_is_borrowed_and_strided_input_is_supported(self):
        f = frame()
        storage = np.zeros((480,1088),np.uint8)
        storage[:,::2] = f.pixels
        strided = storage[:,::2]
        before = storage.copy()
        result = core().process(replace(f,pixels=strided))
        self.assertIs(result.intensity,strided)
        self.assertIs(result.prepared_image,strided)
        self.assertEqual(len(result.centers),4)
        np.testing.assert_array_equal(storage,before)

    def test_float_and_uint8_mapping_produce_identical_detection_and_pose(self):
        a,b = core(),core()
        for i in range(1,4):
            raw = frame(i).pixels.astype(np.float32)*10+5
            grey = raw_to_uint8_srgb(raw,5.,3000.,linear_to_srgb=True)
            r1 = a.process(replace(frame(i),pixels=raw,pixel_format=1))
            r2 = b.process(replace(frame(i),pixels=grey))
            self.assertEqual(r1.centers,r2.centers)
            self.assertEqual(r1.ok,r2.ok)
            if r1.ok:
                np.testing.assert_allclose(r1.solution.position,r2.solution.position,atol=1e-12)

    def test_turbo_intensity_matches_legacy_mapping(self):
        with patch('sys.argv',['receiver','--view','turbo']):
            args=receiver.parse_args()
        f=replace(frame(),pixels=np.linspace(0.,6.,480*544,dtype=np.float32).reshape(480,544),pixel_format=1)
        expected=receiver.prepare_detection_image(f,args)
        result=core(config=receiver.tracking_config_from_args(args)).process(f)
        np.testing.assert_array_equal(result.prepared_image,expected)
        np.testing.assert_array_equal(result.intensity,expected.max(axis=2))

    def test_duplicate_and_backward_observations_do_not_change_prior(self):
        p=core()
        p.process(frame(1)); self.assertTrue(p.process(frame(2)).ok)
        previous=p.tracker._prev_tvec.copy()
        for f in (frame(2),frame(1),replace(frame(3),capture_time_ns=1)):
            self.assertEqual(p.process(f).reason,'out_of_order')
            np.testing.assert_array_equal(p.tracker._prev_tvec,previous)
        self.assertTrue(p.process(frame(3)).ok)

    def test_session_calibration_and_format_changes_reacquire(self):
        for change in (lambda f:replace(f,session_id=99),
                       lambda f:replace(f,intrinsics=replace(f.intrinsics,fx=364.)),
                       lambda f:replace(f,pixel_format=1,pixels=f.pixels.astype(np.float32)*10+5)):
            with self.subTest(change=change):
                p=core(); p.process(frame(1)); self.assertTrue(p.process(frame(2)).ok)
                self.assertFalse(p.process(change(frame(3))).ok)
                self.assertTrue(p.process(change(frame(4))).ok)

    def test_empty_sequence_loses_tracking_and_reacquires(self):
        p=core(); p.process(frame(1)); p.process(frame(2))
        self.assertFalse(p.process(frame(20,empty=True)).ok)
        self.assertIsNone(p.tracker._prev_tvec)
        self.assertFalse(p.process(frame(21)).ok)
        self.assertTrue(p.process(frame(22)).ok)

    def test_invalid_input_and_missing_calibration_are_explicit(self):
        for f,reason in ((replace(frame(),pixels=np.zeros((2,3),np.float64)),'invalid_image'),
                         (replace(frame(),sensor_rotation=np.zeros(4)),'invalid_capture_metadata'),
                         (replace(frame(),intrinsics=None),'missing_calibration'),
                         (replace(frame(),intrinsics=replace(frame().intrinsics,fx=0.)),'invalid_calibration')):
            with self.subTest(reason=reason):
                self.assertEqual(core().process(f).reason,reason)

    def test_expiration_before_processing_does_not_run_detector(self):
        p=core(clock=lambda:2.)
        with patch('tracking_core.detect_marker_centers',side_effect=AssertionError('detector')):
            self.assertEqual(p.process(frame(),deadline=1.).reason,'expired_before_processing')

    def test_expiration_during_solver_clears_unpublished_prior(self):
        now=[1.]
        p=core(clock=lambda:now[0]); p.process(frame(1)); p.process(frame(2))
        original=p.tracker.estimate
        def slow(*args,**kwargs):
            estimate=original(*args,**kwargs)
            now[0]=2.
            return estimate
        with patch.object(p.tracker,'estimate',side_effect=slow):
            result=p.process(frame(3),deadline=1.1)
        self.assertEqual(result.reason,'expired_during_pose')
        self.assertIsNone(p.tracker._prev_tvec)
        self.assertFalse(result.ok)

    def test_late_publication_rechecks_age_and_clears_prior(self):
        p=core(); p.process(frame(1)); r=p.process(frame(2))
        connection=MagicMock()
        with patch('depth_stream_receiver.time.perf_counter',return_value=1.2):
            estimate=receiver.publish_pose_solution(r.solution,frame(2),connection,p.tracker,
                                                    recv_done=1.,max_processing_age_s=.1)
        self.assertFalse(estimate.ok)
        self.assertIsNone(p.tracker._prev_tvec)
        connection.sendall.assert_not_called()

    def test_preview_is_constructed_after_publication(self):
        p=core(); p.process(frame(1)); r=p.process(frame(2))
        with patch('sys.argv',['receiver','--send-pose','--diagnostics-interval','0']):
            args=receiver.parse_args()
        source=MagicMock(); source.__enter__.return_value=source
        source.take.return_value=(frame(2),receiver.time.perf_counter())
        events=[]
        cvt=cv2.cvtColor
        def convert(*a,**kw):
            self.assertIn('publish',events)
            events.append('preview')
            return cvt(*a,**kw)
        def publish(*a,**kw):
            events.append('publish'); return r.solution.estimate
        with patch('depth_stream_receiver.socket.create_connection',return_value=MagicMock()), \
             patch('depth_stream_receiver.connect_pose_socket',return_value=MagicMock()), \
             patch('depth_stream_receiver.sync_clock_offset',return_value=(0.,0.)), \
             patch('depth_stream_receiver.LatestFrameReceiver',return_value=source), \
             patch('depth_stream_receiver.TrackingPipeline.process',return_value=r), \
             patch('depth_stream_receiver.publish_pose_solution',side_effect=publish), \
             patch('depth_stream_receiver.show_waiting_window'), \
             patch('depth_stream_receiver.cv2.cvtColor',side_effect=convert), \
             patch('depth_stream_receiver.cv2.imshow'), \
             patch('depth_stream_receiver.cv2.waitKey',return_value=ord('q')):
            receiver.run(args)
        self.assertEqual(events[0],'publish')
        self.assertIn('preview',events)


class RecordingTests(unittest.TestCase):
    def record(self,folder,legacy=False):
        (folder/'intensity').mkdir(); (folder/'centers').mkdir()
        for i in range(1,5):
            f=frame(i)
            stem=f'{i:06d}_{i}_{f.timestamp:.3f}'
            np.save(folder/'intensity'/f'intensity_{stem}.npy',f.pixels)
            meta=dict(session_id=f.session_id,capture_time_ns=f.capture_time_ns,
                observation_time=f.observation_time,sensor_position=f.sensor_position.tolist(),
                sensor_rotation=f.sensor_rotation.tolist(),camera_intrinsics=asdict(f.intrinsics))
            if not legacy:
                meta['tracking_config']=asdict(TrackingConfig())
            (folder/'centers'/f'centers_{stem}.json').write_text(json.dumps(meta),encoding='utf-8')

    def test_image_replay_uses_same_identity_configuration_and_pose(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp); self.record(folder)
            loaded=list(load_observations(folder))
            self.assertEqual([f.capture_time_ns for _,f,_,_ in loaded],[50_000_000*i for i in range(1,5)])
            report=benchmark(folder,csv_path=folder/'result.csv')
            self.assertEqual(report['frames'],4)
            self.assertGreaterEqual(report['accepted'],2)
            with (folder/'result.csv').open(encoding='utf-8') as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual([int(row['frame_id']) for row in rows],[1,2,3,4])
            self.assertEqual(rows[-1]['reason'],'accepted')
            self.assertTrue(all(value>=0 for value in report['timing_ms_p50_p95_p99']['total_ms']))

    def test_legacy_settings_are_never_silently_guessed(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp); self.record(folder,legacy=True)
            with self.assertRaisesRegex(ValueError,'allow-legacy-defaults'):
                list(load_observations(folder))
            self.assertEqual(len(list(load_observations(folder,allow_legacy_defaults=True))),4)

    def test_missing_sensor_pose_metadata_fails_instead_of_assuming_stationary_headset(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp); self.record(folder)
            path=next((folder/'centers').glob('*.json'))
            meta=json.loads(path.read_text(encoding='utf-8')); del meta['sensor_position']
            path.write_text(json.dumps(meta),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'Incomplete capture metadata'):
                list(load_observations(folder))


if __name__=='__main__':
    unittest.main()
