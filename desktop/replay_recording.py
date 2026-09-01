#!/usr/bin/env python3
"""Replay saved marker centers through MarkerPoseTracker (no headset needed)."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

from marker_pose import TEST_MARKER_COORDS, MarkerPoseTracker, centers_to_float


def load_frames(recording: Path):
    """Return [(index, frame_id, centers_array), ...] in capture order."""
    files = sorted((recording / "centers").glob("centers_*.npy"))
    frames = []
    for path in files:
        m = re.match(r"centers_(\d+)_(\d+)_", path.name)
        if m is None:
            continue
        frames.append((int(m.group(1)), int(m.group(2)), np.load(path)))
    frames.sort(key=lambda item: item[0])
    return frames


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording", type=Path)
    parser.add_argument("--min-z", type=float, default=0.15)
    parser.add_argument("--max-z", type=float, default=1.50)
    parser.add_argument("--no-gate", action="store_true",
                        help="Disable the depth gate (baseline comparison)")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    K = np.load(args.recording / "intrinsics" / "camera_matrix.npy")
    dist = np.load(args.recording / "intrinsics" / "dist_coeffs.npy")

    if args.no_gate:
        min_z, max_z = 0.0, 1e9
    else:
        min_z, max_z = args.min_z, args.max_z

    tracker = MarkerPoseTracker(
        model_points=TEST_MARKER_COORDS,
        min_probe_z_m=min_z,
        max_probe_z_m=max_z,
    )

    frames = load_frames(args.recording)
    print(f"Loaded {len(frames)} frames from {args.recording}")
    print(f"Depth gate: {'OFF' if args.no_gate else f'{min_z}..{max_z} m'}\n")

    z_values, n_ok, n_fail = [], 0, 0
    by_marker_count = {}

    for index, frame_id, centers in frames:
        centers = np.asarray(centers).reshape(-1, 2)
        n_det = centers.shape[0]
        est = tracker.estimate(centers_to_float(centers), K, dist) if n_det >= 3 else None

        slot = by_marker_count.setdefault(n_det, [0, 0])
        if est is not None and est.ok:
            z = float(est.tvec.reshape(3)[2])
            z_values.append(z)
            n_ok += 1
            slot[0] += 1
            tag = "NEAR" if z < 0.30 else "FAR "
            print(f"{index:5d} n_det={n_det} n_used={est.n_used} {tag} "
                  f"z={z:+.3f} reproj={est.mean_reproj_px:.2f} "
                  f"conf={est.confidence:.2f} model_idx={est.model_indices}")
        else:
            n_fail += 1
            slot[1] += 1
            if not args.quiet:
                print(f"{index:5d} n={n_det} FAIL")

    total = n_ok + n_fail
    print(f"\n=== SUMMARY ({total} frames) ===")
    print(f"tracked: {n_ok} ({100.0 * n_ok / max(total, 1):.1f}%)   failed: {n_fail}")

    for n_det in sorted(by_marker_count):
        ok, fail = by_marker_count[n_det]
        print(f"  {n_det} markers detected: {ok} tracked / {ok + fail} frames")
        print(f"\nrejection reasons: {tracker.debug_counts}")

    if z_values:
        z = np.array(z_values)
        near = int(np.sum(z < 0.15))
        far = int(np.sum(z >= 0.15))
        print(f"\nz: min={z.min():.3f} median={np.median(z):.3f} "
              f"max={z.max():.3f} std={z.std():.3f}")
        print(f"near cluster (<0.15m): {near} ({100.0 * near / len(z):.1f}%)")
        print(f"far  cluster (>=0.15m): {far} ({100.0 * far / len(z):.1f}%)")


if __name__ == "__main__":
    main()