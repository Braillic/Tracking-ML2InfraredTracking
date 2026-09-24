"""Image mapping and marker detection. No sockets, windows, or file I/O."""
from __future__ import annotations
import cv2
import numpy as np
from marker_pose import TEST_MARKER_COORDS, filter_marker_centers

MARKER_MIN_AREA = 2
# Absolute fallback; close-up blobs often exceed a few thousand pixels.
MARKER_MAX_AREA = 2000
# Prefer image-relative cap: ~8% of frame (≈20k px on 544x480).
MARKER_MAX_AREA_FRAC = 0.10
MARKER_MAX_ASPECT = 6.0
MARKER_RING_RADIUS = 10
MAX_DETECTED_MARKERS = 8  # bounded candidate pool; PnP selects the constellation
# Pre-PnP spatial filter defaults (pixels). These also scale up with image size
# so close-up constellations are not rejected as "too spread out".
MARKER_MAX_NEAREST_NEIGHBOR_PX = 90.0
MARKER_MAX_NEAREST_NEIGHBOR_FRAC = 0.35  # of min(h, w)
MARKER_MAX_CLUSTER_SPAN_PX = 220.0
MARKER_MAX_CLUSTER_SPAN_FRAC = 0.85  # of min(h, w)
MARKER_MIN_CLUSTER_SIZE = 3
MARKER_GEOMETRY_RATIO_ERROR = 0.35


def raw_to_uint8_srgb(
    depth: np.ndarray,
    raw_min: float,
    raw_max: float,
    *,
    linear_to_srgb: bool,
) -> np.ndarray:
    """Map FLOAT32 DepthRaw values to the same uint8 image produced on ML2."""
    finite = np.isfinite(depth)
    normalized = (
        (np.where(finite, depth, raw_min) - raw_min)
        / max(raw_max - raw_min, 1e-6)
    )
    normalized = np.clip(normalized, 0.0, 1.0)
    if linear_to_srgb:
        normalized = np.where(
            normalized <= 0.0031308,
            normalized * 12.92,
            1.055 * np.power(normalized, 1.0 / 2.4) - 0.055,
        )
    grey = np.rint(normalized * 255.0).astype(np.uint8)
    grey[~finite] = 0
    return grey



def threshold_cutoff(intensity, threshold_method, fixed_threshold, top_p):
    if threshold_method == "fixed":
        cutoff = float(fixed_threshold)
    elif threshold_method == "percentile":
        floor, percentile = top_p
        if intensity.size == 0:
            cutoff = float(floor)
        else:
            cutoff = max(float(floor), float(np.percentile(intensity, percentile)))
    else:
        raise ValueError(f"Invalid threshold method: {threshold_method}")
    return cutoff


def apply_threshold(
    image: np.ndarray,
    threshold_method: str,
    fixed_threshold: float = 200.0,
    top_p: tuple[float, float] = (100.0, 90.0),
) -> tuple[np.ndarray, float]:
    """Threshold a colourised uint8 image. top_p is (absolute floor, percentile).

    Intensity is the max channel per pixel (works for grey unity-raw and turbo).
    Pixels below the cutoff are set to 0; others keep their original colour.
    """
    intensity = image.max(axis=2) if image.ndim == 3 else image
    cutoff = threshold_cutoff(intensity, threshold_method, fixed_threshold, top_p)

    keep = intensity >= cutoff
    if image.ndim == 3:
        thresholded = np.where(keep[..., None], image, 0).astype(np.uint8)
    else:
        thresholded = np.where(keep, image, 0).astype(np.uint8)
    return thresholded, cutoff


def blob_area_limits(
    image_shape: tuple[int, ...],
    *,
    min_area: int | None = None,
    max_area: int | None = None,
    max_area_frac: float = MARKER_MAX_AREA_FRAC,
) -> tuple[int, int]:
    """Area gate that scales with frame size for close-up markers."""
    height, width = int(image_shape[0]), int(image_shape[1])
    lo = MARKER_MIN_AREA if min_area is None else int(min_area)
    frac_cap = max(lo + 1, int(max_area_frac * height * width))
    hi = MARKER_MAX_AREA if max_area is None else int(max_area)
    return lo, max(hi, frac_cap)


def geometry_pixel_limits(
    image_shape: tuple[int, ...],
    *,
    max_nearest_neighbor_px: float | None = None,
    max_cluster_span_px: float | None = None,
) -> tuple[float, float]:
    """Isolation / span gates that grow for close-up (large on-screen) tools."""
    short_side = float(min(int(image_shape[0]), int(image_shape[1])))
    nn = MARKER_MAX_NEAREST_NEIGHBOR_PX if max_nearest_neighbor_px is None else float(
        max_nearest_neighbor_px
    )
    span = MARKER_MAX_CLUSTER_SPAN_PX if max_cluster_span_px is None else float(
        max_cluster_span_px
    )
    nn = max(nn, MARKER_MAX_NEAREST_NEIGHBOR_FRAC * short_side)
    span = max(span, MARKER_MAX_CLUSTER_SPAN_FRAC * short_side)
    return nn, span


def detect_marker_centers(
    colourised: np.ndarray,
    threshold_method: str,
    fixed_threshold: float,
    top_p: tuple[float, float],
    # max_markers: int = 5,
    *,
    filter_geometry: bool = True,
    model_points: np.ndarray | None = None,
    min_area: int | None = None,
    max_area: int | None = None,
    max_aspect: float = MARKER_MAX_ASPECT,
    max_nearest_neighbor_px: float | None = None,
    max_cluster_span_px: float | None = None,
    min_cluster_size: int = MARKER_MIN_CLUSTER_SIZE,
    max_geometry_ratio_error: float = MARKER_GEOMETRY_RATIO_ERROR,
    max_markers: int=MAX_DETECTED_MARKERS,
    include_thresholded: bool = True,
) -> tuple[np.ndarray | None, float, list[tuple[float, float]]]:
    """Threshold a colourised frame and locate bright marker blobs."""
    # Keep Float32 weights to preserve the existing summation/centroid behaviour.
    intensity = (colourised.max(axis=2) if colourised.ndim == 3 else colourised).astype(np.float32)
    cutoff = threshold_cutoff(intensity, threshold_method, fixed_threshold, top_p)
    keep = intensity >= cutoff
    thresholded = None
    if include_thresholded:
        thresholded = np.where(keep[..., None] if colourised.ndim == 3 else keep,
                               colourised, 0).astype(np.uint8)

    min_blob_area, max_blob_area = blob_area_limits(
        colourised.shape, min_area=min_area, max_area=max_area
    )
    max_nn_px, max_span_px = geometry_pixel_limits(
        colourised.shape,
        max_nearest_neighbor_px=max_nearest_neighbor_px,
        max_cluster_span_px=max_cluster_span_px,
    )

    mask = keep.astype(np.uint8)
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    candidates: list[tuple[float, tuple[int, int]]] = []
    for label in range(1, n_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        aspect = max(width, height) / max(1, min(width, height))

        if area < min_blob_area or area > max_blob_area or aspect > max_aspect:
            continue

        # Restrict each blob scan to its bounding box rather than repeatedly
        # scanning the full image for every connected component.
        left, top = int(stats[label, cv2.CC_STAT_LEFT]), int(stats[label, cv2.CC_STAT_TOP])
        ys, xs = np.where(labels[top:top + height, left:left + width] == label)
        ys, xs = ys + top, xs + left
        weights = intensity[ys, xs]
        if weights.size == 0 or float(weights.sum()) <= 0:
            continue

        weight_sum = float(weights.sum())
        cx = float((xs * weights).sum() / weight_sum)
        cy = float((ys * weights).sum() / weight_sum)
        mean_intensity = float(weights.mean())
        candidates.append((mean_intensity, (cx, cy)))

    candidates.sort(key=lambda item: item[0], reverse=True)
    centers = [center for _, center in candidates[:max_markers]]
    if filter_geometry:
        centers = filter_marker_centers(
            centers,
            model_points=TEST_MARKER_COORDS if model_points is None else model_points,
            max_nearest_neighbor_px=max_nn_px,
            max_cluster_span_px=max_span_px,
            min_cluster_size=min_cluster_size,
            max_geometry_ratio_error=max_geometry_ratio_error,
            preserve_candidates=True,
        )
    return thresholded, cutoff, centers

