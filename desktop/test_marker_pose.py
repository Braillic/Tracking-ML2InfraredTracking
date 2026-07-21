#!/usr/bin/env python3
"""Offline marker correspondence / pose test on a saved recording.

Expects a directory from depth_stream_receiver (key R), e.g.:
  saves/recording_YYYYMMDD_HHMMSS/
    centers/centers_*.npy
    analysis/analysis_*.png   (optional, for overlay)

Example:
  python test_marker_pose.py saves/recording_20260720_114921
  python test_marker_pose.py saves/recording_20260720_114921 --save-overlay
"""

from __future__ import annotations

import argparse
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from marker_pose import TEST_MARKER_COORDS, MarkerPoseTracker
from depth_stream_receiver import colourise_depth #, detect_marker_centers, annotate_markers
# camera matrix and distortion coefficients fom running depth_stream_receiver
DEFAULT_CAMERA_MATRIX = np.array([[363.10574341,   0.          , 267.85662842],
                                   [  0.         , 363.10574341, 237.83192444],
                                   [  0.         ,  0.         ,  1.        ]], dtype=np.float64)
DEFAULT_DIST_COEFFS = np.array(
    [-0.09079792,-0.01289384, -0.00013479,  0.00011443, -0.01124168], dtype=np.float64)

def _stem_key(path: Path) -> str:
    """centers_000000_7280_....npy → 000000_7280_...."""
    name = path.stem
    prefix = "centers_"
    return name[len(prefix):] if name.startswith(prefix) else name


def _draw_overlay(
    depth_bgr: np.ndarray,
    detections: np.ndarray,
    projected: np.ndarray | None,
    est_ok: bool,
    model_indices: tuple[int, ...],
    image_indices: tuple[int, ...],
    reproj: float,
    conf: float,
    coverage: float = 1.0,
    unmatched_det: int = 0,
) -> np.ndarray:
    out = depth_bgr.copy()
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    for i, (x, y) in enumerate(detections.reshape(-1, 2)):
        p = (int(round(x)), int(round(y)))
        cv2.circle(out, p, 6, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.putText(out, f"d{i}", (p[0] + 8, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

    if projected is not None:
        for mi, (x, y) in enumerate(projected.reshape(-1, 2)):
            p = (int(round(x)), int(round(y)))
            cv2.drawMarker(out, p, (0, 255, 0), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
            cv2.putText(out, f"m{mi}", (p[0] + 8, p[1] + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

    if est_ok:
        for mi, ii in zip(model_indices, image_indices):
            a = detections[ii]
            b = projected[mi]
            cv2.line(
                out,
                (int(round(a[0])), int(round(a[1]))),
                (int(round(b[0])), int(round(b[1]))),
                (255, 200, 0),
                1,
                cv2.LINE_AA,
            )

    label = (
        f"ok={est_ok} ids={model_indices} reproj={reproj:.2f}px cov={coverage:.2f} "
        f"unmatched={unmatched_det} conf={conf:.2f}"
        if est_ok
        else "ok=False"
    )
    cv2.putText(out, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, "red=det  green=proj  cyan=match", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return out


def run_recording(
    recording_path: Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    model_points: np.ndarray,
    save_overlay: bool = False,
    max_frames: int | None = None,
) -> None:
    centers_dir = recording_path / "centers"
    depth_dir = recording_path / "depth"
    overlay_dir = recording_path / "pose_overlay"
    if not centers_dir.is_dir():
        raise FileNotFoundError(f"missing centers/: {centers_dir}")

    center_files = sorted(centers_dir.glob("centers_*.npy"))
    if not center_files:
        raise FileNotFoundError(f"no centers_*.npy in {centers_dir}")
    if max_frames is not None:
        center_files = center_files[:max_frames]

    tracker = MarkerPoseTracker(model_points=model_points)
    print(
        f"recording={recording_path}\n"
        f"frames={len(center_files)}"
        f"min_markers={tracker.min_markers} model_n={len(model_points)}"
    )

    if save_overlay:
        overlay_dir.mkdir(parents=True, exist_ok=True)

    n_ok = 0
    overlays: list[np.ndarray] = []
    reprojs: list[float] = []
    for path in center_files:
        pts = np.load(path).astype(np.float64).reshape(-1, 2)
        est = tracker.estimate(pts, camera_matrix, dist_coeffs)

        projected = None
        if est.ok and est.rvec is not None and est.tvec is not None:
            n_ok += 1
            reprojs.append(est.mean_reproj_px)
            projected, _ = cv2.projectPoints(
                model_points.reshape(-1, 1, 3),
                est.rvec,
                est.tvec,
                camera_matrix,
                dist_coeffs,
            )
            projected = projected.reshape(-1, 2)
            print(
                f"{path.name}  n={len(pts)}  ok  ids={est.model_indices}  "
                f"reproj={est.mean_reproj_px:.2f}px  cov={est.coverage:.2f}  "
                f"unmatched={est.n_unmatched_detections}  conf={est.confidence:.2f}  "
                f"t=({est.tvec[0,0]:.3f},{est.tvec[1,0]:.3f},{est.tvec[2,0]:.3f})"
            )
        else:
            print(f"{path.name}  n={len(pts)}  FAIL")

        if save_overlay:
            stem = _stem_key(path)
            depth_png = depth_dir / f"depth_{stem}.png"
            depth_npy = depth_dir / f"depth_{stem}.npy"
            raw = np.load(depth_npy)
            process_args = SimpleNamespace(raw_min=5, raw_max=3000, view="unity-raw", unity_color_space="linear", threshold_method="fixed", fixed_threshold=200, top_p=(100, 90))
            gray =colourise_depth(raw, process_args)
            canvas = gray
            overlay = _draw_overlay(
                canvas,
                pts,
                projected,
                est.ok,
                est.model_indices,
                est.image_indices,
                est.mean_reproj_px if est.ok else float("inf"),
                est.confidence if est.ok else 0.0,
                coverage=est.coverage if est.ok else 0.0,
                unmatched_det=est.n_unmatched_detections if est.ok else 0,
            )
            # add to list of overlays
            overlays.append(overlay)
            #cv2.imwrite(str(overlay_dir / f"overlay_{stem}.png"), overlay)

    print(
        f"\nsummary: {n_ok}/{len(center_files)} ok"
        + (
            f"  mean_reproj={np.mean(reprojs):.2f}px  "
            f"p95={np.percentile(reprojs, 95):.2f}px"
            if reprojs
            else ""
        )
    )
    if save_overlay:
        print(f"overlays → {overlay_dir}")
    return overlays


def video_save_overlay(recording_path: Path, overlays: list[np.ndarray]) -> None:
    if not overlays:
        return
    h, w = overlays[0].shape[:2]
    if overlays[0].ndim == 2:
        h, w = overlays[0].shape
    writer = cv2.VideoWriter(
        str(recording_path / "pose_overlay.mp4"),
        cv2.VideoWriter_fourcc(*"mp4v"),
        15,
        (w, h),
    )
    for frame in overlays:
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        writer.write(np.ascontiguousarray(frame, dtype=np.uint8))
    writer.release()
    print(f"video saved to {recording_path / 'pose_overlay.mp4'}")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "recording",
        type=Path,
        nargs="?",
        default=Path("saves/recording_20260720_114921"),
        help="Path to recording_YYYYMMDD_HHMMSS directory",
    )
    p.add_argument("--save-overlay", action="store_true",
                   help="Write pose_overlay/*.png with det vs projected markers")
    p.add_argument("--max-frames", type=int, default=None)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    overlays = run_recording(
        recording_path=args.recording,
        camera_matrix=DEFAULT_CAMERA_MATRIX,
        dist_coeffs=DEFAULT_DIST_COEFFS,
        model_points=TEST_MARKER_COORDS,
        save_overlay=args.save_overlay,
        max_frames=args.max_frames,
    )
    if args.save_overlay and overlays:
        video_save_overlay(args.recording, overlays)