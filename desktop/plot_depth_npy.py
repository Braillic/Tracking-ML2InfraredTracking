#!/usr/bin/env python3
"""Load saved ML2 .npy depth frames and write frame + histogram plots.

By default processes every *.npy in ./processing and writes plots to
./processing/plots. Pass explicit .npy paths to override.

Optional fixed / percentile thresholding matches depth_analysis.ipynb
(and depth_stream_receiver): applied on the colourised 0–255 image.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np

from depth_stream_receiver import apply_threshold, colourise_depth


def colourise(depth: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    """Return an RGB image suitable for matplotlib."""
    if depth.dtype == np.uint8:
        if depth.ndim == 2:
            return np.stack([depth, depth, depth], axis=-1)
        # OpenCV BGR -> RGB
        return depth[..., ::-1].copy()

    view_args = SimpleNamespace(
        view=args.view,
        raw_min=args.raw_min,
        raw_max=args.raw_max,
        unity_color_space=args.unity_color_space,
    )
    bgr = colourise_depth(depth, view_args)
    return bgr[..., ::-1]


def intensity_for_hist(colour: np.ndarray) -> np.ndarray:
    if colour.ndim == 3:
        return colour.max(axis=2).ravel()
    return colour.ravel()


def resolve_inputs(args: argparse.Namespace) -> list[Path]:
    if args.npy_files:
        paths = list(args.npy_files)
    else:
        input_dir = args.input_dir
        if not input_dir.is_dir():
            raise FileNotFoundError(
                f"Input directory not found: {input_dir.resolve()}"
            )
        paths = sorted(input_dir.glob("*.npy"))
        if not paths:
            raise FileNotFoundError(f"No .npy files in {input_dir.resolve()}")
    for path in paths:
        if not path.exists():
            raise FileNotFoundError(path)
        if path.suffix.lower() != ".npy":
            raise ValueError(f"Expected a .npy file, got {path}")
    return paths


def hist_range(args: argparse.Namespace) -> tuple[float, float]:
    return (
        args.hist_min if args.hist_min is not None else 0.0,
        args.hist_max if args.hist_max is not None else 255.0,
    )


def plot_one(path: Path, args: argparse.Namespace, out_dir: Path) -> Path:
    frame = np.load(path)
    colour = colourise(frame, args)
    values = intensity_for_hist(colour)
    hmin, hmax = hist_range(args)

    out_dir.mkdir(parents=True, exist_ok=True)
    suffix = "" if args.threshold == "none" else f"_{args.threshold}"
    out_path = out_dir / f"{path.stem}{suffix}_plot{args.suffix}.png"

    if args.threshold == "none":
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].imshow(colour)
        axes[0].set_title(path.name)
        axes[0].axis("off")

        axes[1].hist(values, bins=args.bins, range=(hmin, hmax), color="steelblue")
        axes[1].set_title("histogram")
        axes[1].set_xlabel("intensity (0–255)")
        axes[1].set_ylabel("count")
    else:
        # apply_threshold uses max-channel intensity; RGB works the same as BGR.
        thresholded, cutoff = apply_threshold(
            colour,
            args.threshold,
            fixed_threshold=args.fixed_threshold,
            top_p=(args.percentile_floor, args.percentile),
        )
        thresh_values = intensity_for_hist(thresholded)
        if args.threshold == "fixed":
            thresh_title = f"thresholded (≥ {args.fixed_threshold:g}, cutoff {cutoff:.1f})"
        else:
            thresh_title = (
                f"thresholded (p{args.percentile:g}, floor {args.percentile_floor:g}, "
                f"cutoff {cutoff:.1f})"
            )

        fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
        axes[0].imshow(colour)
        axes[0].set_title(path.name)
        axes[0].axis("off")

        axes[1].hist(values, bins=args.bins, range=(hmin, hmax), color="steelblue")
        axes[1].set_title("histogram")
        axes[1].set_xlabel("intensity (0–255)")

        axes[2].imshow(thresholded)
        axes[2].set_title(thresh_title)
        axes[2].axis("off")

        # Match notebook: exclude zeros smashed by the threshold.
        axes[3].hist(
            thresh_values[thresh_values > 0],
            bins=args.bins,
            range=(max(hmin, 1.0), hmax),
            color="darkorange",
        )
        axes[3].set_title("histogram (thresholded, exclude 0)")
        axes[3].set_xlabel("intensity")

    fig.tight_layout()
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    return out_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "npy_files",
        nargs="*",
        type=Path,
        help="Optional explicit .npy files (default: all *.npy in --input-dir)",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path("processing"),
        help="Folder whose *.npy files are processed when no files are listed "
             "(default: ./processing)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Directory for plot PNGs (default: <input-dir>/plots, or "
             "./plots when listing files)",
    )
    parser.add_argument(
        "--threshold",
        choices=("none", "fixed", "percentile"),
        default="none",
        help="Threshold colourised intensity like the notebook "
             "(default: none → 1x2 plot; fixed/percentile → 1x4 plot)",
    )
    parser.add_argument(
        "--fixed-threshold",
        type=float,
        default=100.0,
        help="Cutoff for --threshold fixed on colourised 0–255 "
             "(notebook used 100 for colourised)",
    )
    parser.add_argument(
        "--percentile",
        type=float,
        default=90.0,
        help="Percentile for --threshold percentile",
    )
    parser.add_argument(
        "--percentile-floor",
        type=float,
        default=100.0,
        help="Absolute floor combined with percentile cutoff",
    )
    parser.add_argument("--bins", type=int, default=256)
    parser.add_argument("--hist-min", type=float, default=None)
    parser.add_argument("--hist-max", type=float, default=None)
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--view", choices=("unity-raw", "turbo"), default="unity-raw")
    parser.add_argument("--raw-min", type=float, default=5.0)
    parser.add_argument("--raw-max", type=float, default=3000.0)
    parser.add_argument("--unity-color-space", choices=("linear", "gamma"),
                        default="linear")
    parser.add_argument("--suffix", type=str, default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    paths = resolve_inputs(args)
    if args.out_dir is not None:
        out_dir = args.out_dir
    elif args.npy_files:
        out_dir = Path("plots")
    else:
        out_dir = args.input_dir / "plots"

    for path in paths:
        out_path = plot_one(path, args, out_dir)
        print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
