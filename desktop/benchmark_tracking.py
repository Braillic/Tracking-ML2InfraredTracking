#!/usr/bin/env python3
"""Replay recorded intensity images through the live tracking core without TCP/GUI.

Measures processing only, not capture/network/display E2E. Inputs are already
mapped UINT8 intensity, so FLOAT32 mapping cost is not part of this benchmark.
Disk I/O and CSV export are outside core timings. Acquisition frames are included.
"""
from __future__ import annotations

import argparse
import csv
import json
import platform
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from marker_pose import MarkerPoseTracker, TEST_MARKER_COORDS
from tracking_core import DetectionConfig, TrackingConfig, TrackingPipeline
from tracking_types import CameraIntrinsics, TrackingFrame


def load_observations(recording: Path, *, allow_legacy_defaults=False):
    paths = sorted((recording / 'intensity').glob('intensity_*.npy'))
    if not paths:
        raise ValueError(f'No intensity frames in {recording / "intensity"}')
    for path in paths:
        stem = path.stem[len('intensity_'):]
        index, frame_id, timestamp = stem.split('_', 2)
        metadata_path = recording / 'centers' / f'centers_{stem}.json'
        if not metadata_path.exists():
            raise ValueError(f'Missing capture-time sensor metadata: {metadata_path}')
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        required = ('sensor_position', 'sensor_rotation', 'observation_time', 'session_id')
        if any(key not in metadata for key in required):
            raise ValueError(f'Incomplete capture metadata: {metadata_path}')
        config_data = metadata.get('tracking_config')
        if config_data is None:
            if not allow_legacy_defaults:
                raise ValueError('Older recording has no detection settings; explicitly use '
                                 '--allow-legacy-defaults to replay with current defaults')
            config_data = dict(depth_vertically_flipped=metadata.get('depth_vertically_flipped', True),
                               convert_object_axes=metadata.get('convert_object_axes', False))
        else:
            config_data = dict(config_data)
        detection_data = config_data.pop('detection', {})
        if 'top_p' in detection_data:
            detection_data = dict(detection_data, top_p=tuple(detection_data['top_p']))
        config = TrackingConfig(detection=DetectionConfig(**detection_data), **config_data)
        if metadata.get('camera_intrinsics'):
            intrinsics = CameraIntrinsics(**metadata['camera_intrinsics'])
        else:
            K = np.load(recording / 'intrinsics' / 'camera_matrix.npy', allow_pickle=False)
            dist = np.load(recording / 'intrinsics' / 'dist_coeffs.npy', allow_pickle=False).reshape(-1)
            if K.shape != (3, 3) or dist.size != 5:
                raise ValueError('Recording calibration must be a 3x3 matrix and five distortion terms')
            intrinsics = CameraIntrinsics(K[0, 0], K[1, 1], K[0, 2], K[1, 2], 0., 0., *dist)
        pixels = np.load(path, allow_pickle=False)
        if pixels.ndim != 2 or pixels.dtype != np.uint8:
            raise ValueError(f'Expected mapped UINT8 intensity: {path}')
        frame = TrackingFrame(pixels, int(frame_id), float(metadata['observation_time']),
            np.asarray(metadata['sensor_position'], dtype=np.float64),
            np.asarray(metadata['sensor_rotation'], dtype=np.float64), intrinsics,
            session_id=int(metadata['session_id']), capture_time_ns=int(metadata.get('capture_time_ns', 0)))
        yield int(index), frame, config, metadata


def benchmark(recording: Path, *, csv_path: Path | None = None, allow_legacy_defaults=False):
    core = TrackingPipeline()
    rows, reasons, states = [], Counter(), Counter()
    last_source_format = None
    legacy = 0
    for index, frame, config, metadata in load_observations(
            recording, allow_legacy_defaults=allow_legacy_defaults):
        # Recordings contain mapped intensity even when live transport was FLOAT32.
        # Preserve the live reset on source format switches without mapping twice.
        source_format = metadata.get('source_pixel_format')
        if source_format != last_source_format:
            core.reset()
            last_source_format = source_format
        core.config = config
        core.tracker.lost_timeout_s = float(metadata.get('tracking_lost_timeout', .65))
        core.tracker.acquisition_timeout_s = float(metadata.get('acquisition_timeout', .65))
        result = core.process(frame)
        reasons[result.reason] += 1
        states[result.state] += 1
        legacy += int('tracking_config' not in metadata)
        solution = result.solution
        row = dict(index=index, session_id=result.session_id, frame_id=result.frame_id,
            capture_time_ns=result.capture_time_ns, observation_time=result.observation_time,
            accepted=result.ok, state=result.state, reason=result.reason,
            centers=json.dumps(result.centers), confidence=solution.estimate.confidence,
            reprojection_px=solution.estimate.mean_reproj_px if result.ok else None,
            **asdict(result.timings))
        row.update(zip(('x_m', 'y_m', 'z_m'), solution.position or (None,)*3))
        row.update(zip(('qx', 'qy', 'qz', 'qw'), solution.rotation or (None,)*4))
        rows.append(row)
    timing_names = ('prepare_ms', 'detect_ms', 'pnp_ms', 'world_ms', 'total_ms')
    summary = dict(frames=len(rows), accepted=sum(row['accepted'] for row in rows),
        reasons=dict(reasons), states=dict(states), legacy_default_frames=legacy,
        timing_scope='mapped UINT8 image-to-world-pose core; no I/O, transport or presentation',
        includes_acquisition=True, model_points_m=TEST_MARKER_COORDS.tolist(),
        environment=dict(python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__),
        timing_ms_p50_p95_p99={name: np.percentile([row[name] for row in rows], [50,95,99]).tolist()
                              for name in timing_names})
    if csv_path is not None:
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--csv', type=Path, help='Per-frame identity, pose, state and stage timings')
    parser.add_argument('--json', type=Path, help='Summary output path')
    parser.add_argument('--allow-legacy-defaults', action='store_true',
                        help='Explicitly allow missing detection configuration in older recordings')
    args = parser.parse_args()
    try:
        summary = benchmark(args.recording, csv_path=args.csv, allow_legacy_defaults=args.allow_legacy_defaults)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    report = json.dumps(summary, indent=2, allow_nan=False)
    print(report)
    if args.json:
        args.json.write_text(report+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
