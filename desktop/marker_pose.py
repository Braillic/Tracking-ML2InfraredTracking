"""2D↔3D marker correspondence and pose.

Per frame:
  1. Acquire image centers (e.g. from detect_marker_centers).
  2. Build candidates: treat detections as a subset of the known 3D model,
     can rule out using pairwise distance-ratio signatures.
  3. Rule out more using: geometric gate → solvePnP → reprojection + temporal score.
  4. Output pose and confidence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations, permutations
from typing import Sequence

import cv2
import numpy as np
# in meters, planar probe first
TEST_MARKER_COORDS = np.array(
    (
        (0, 0.0501, 0), # top
        (-0.0131, 0.0126, 0), # left
        (0, 0, 0), # center 
        (0, -0.0391 , 0) # bottom
    ),
    dtype=np.float64,
)


@dataclass(frozen=True)
class PoseEstimate:
    ok: bool
    rvec: np.ndarray | None  # 3x1
    tvec: np.ndarray | None  # 3x1
    confidence: float
    mean_reproj_px: float
    model_indices: tuple[int, ...] = ()
    image_indices: tuple[int, ...] = ()
    n_used: int = 0
    support: int = 0

    @staticmethod
    def failed() -> PoseEstimate:
        return PoseEstimate(
            ok=False,
            rvec=None,
            tvec=None,
            confidence=0.0,
            mean_reproj_px=float("inf"),
        )


@dataclass
class MarkerPoseTracker:
    """correspondence + PnP.
    """

    model_points: np.ndarray  # (N, 3) metres, object frame
    min_markers: int | None = None
    # Distance-signature gate: max relative error on sorted pairwise ratios.
    # Comparing 2D image distances to 3D model distances is approximate under
    # perspective; non-coplanar layouts need a looser gate.
    max_distance_ratio_error: float | None = None
    max_reproj_px: float = 2.0 # error in pixels between projected and detected points
    # Temporal nearest-neighbour association gate (pixels).
    max_assoc_px: float = 40.0
    # Soft weights for ranking candidates (lower is better before → confidence).
    temporal_translation_weight: float = 50.0  # px-equivalent per metre
    temporal_rotation_weight: float = 30.0  # px-equivalent per radian
    # Reject if confidence below this after ranking.
    min_confidence: float = 0.15

    _prev_rvec: np.ndarray | None = field(default=None, init=False, repr=False)
    _prev_tvec: np.ndarray | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self.model_points = np.asarray(self.model_points, dtype=np.float64).reshape(-1, 3)
        if self.min_markers is None:
            self.min_markers = 3
        if self.max_distance_ratio_error is None:
            self.max_distance_ratio_error = 0.18
        if self.model_points.shape[0] < self.min_markers:
            raise ValueError("model_points shorter than min_markers")
        if self.min_markers < 3:
            raise ValueError("PnP needs at least 3 markers")    

    def reset(self) -> None:
        self._prev_rvec = None
        self._prev_tvec = None

    def estimate(
        self,
        image_points: Sequence[tuple[float, float]] | np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray | None = None,
    ) -> PoseEstimate:
        pts = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
        if pts.shape[0] < self.min_markers and self._prev_rvec is None:
            return PoseEstimate.failed()

        # default to temporal association if already tracking
        if self._prev_rvec is not None and self._prev_tvec is not None:
            temporal = self._estimate_temporal(pts, camera_matrix, dist_coeffs)
            if temporal.ok and temporal.confidence >= self.min_confidence:
                self._prev_rvec = temporal.rvec
                self._prev_tvec = temporal.tvec
                return temporal

        combinatorial = self._estimate_combinatorial(pts, camera_matrix, dist_coeffs)
        if combinatorial.ok:
            self._prev_rvec = combinatorial.rvec
            self._prev_tvec = combinatorial.tvec
        else:
            # keep last pose only if we never found anything this frame
            pass
        return combinatorial

    # ------------------------------------------------------------------
    # Temporal path: project previous pose, keep nearest detection
    # ------------------------------------------------------------------

    def _estimate_temporal(
        self,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> PoseEstimate:
        projected, _ = cv2.projectPoints(
            self.model_points,
            self._prev_rvec,
            self._prev_tvec,
            camera_matrix,
            dist_coeffs,
        )
        projected = projected.reshape(-1, 2)

        # Distance matrix model i × image j; mutual nearest within gate.
        dmat = np.linalg.norm(
            projected[:, None, :] - image_points[None, :, :], axis=2
        )
        model_idx: list[int] = []
        image_idx: list[int] = []
        used_model: set[int] = set()
        used_image: set[int] = set()
        # Greedy on ascending distance, accepting mutual nearest pairs only.
        flat = [(float(dmat[i, j]), i, j)
                for i in range(dmat.shape[0])
                for j in range(dmat.shape[1])]
        flat.sort()
        for dist, i, j in flat:
            if dist > self.max_assoc_px:
                break
            if i in used_model or j in used_image:
                continue
            if int(np.argmin(dmat[i])) != j:
                continue
            if int(np.argmin(dmat[:, j])) != i:
                continue
            # used_model and used_image are sets of model and image indices that have been used to avoid duplicates
            used_model.add(i)
            used_image.add(j)
            model_idx.append(i)
            image_idx.append(j)

        if len(model_idx) < self.min_markers:
            return PoseEstimate.failed()

        return self._solve_and_score(
            image_points,
            camera_matrix,
            dist_coeffs,
            tuple(model_idx),
            tuple(image_idx),
            use_extrinsic_guess=True,
        )

    # ------------------------------------------------------------------
    # Combinatorial path: subsets + distance signature + permutations
    # ------------------------------------------------------------------

    def _estimate_combinatorial(
        self,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> PoseEstimate:
        n_img = image_points.shape[0] # number of detected marker centers
        n_model = self.model_points.shape[0] # number of model points
        k_values = range(self.min_markers, min(n_img, n_model) + 1) # number of points to use for PnP

        best = PoseEstimate.failed()
        best_cost = float("inf")

        model_sig_cache: dict[tuple[int, ...], np.ndarray] = {}
        # hard gate: max relative error on sorted pairwise ratios
        hard_gate = float(self.max_distance_ratio_error)

        for k in reversed(list(k_values)):  # big loop over all possible combinations of model and image points
            img_sig_cache: dict[tuple[int, ...], np.ndarray] = {}
            for img_combo in combinations(range(n_img), k):  # detected marker subsets
                img_pts = image_points[list[int](img_combo)] # image points corresponding to the combination
                img_sig = img_sig_cache.setdefault(
                    img_combo, _pairwise_distance_signature(img_pts) # normalized pairwise distances between the image points
                )
                for model_combo in combinations(range(n_model), k):  # model subsets
                    model_pts = self.model_points[list(model_combo)]
                    model_sig = model_sig_cache.setdefault(
                        model_combo, _pairwise_distance_signature(model_pts)
                    )
                    sig_err = _signature_error(img_sig, model_sig) # distance between image and model pairwise distances
                    if sig_err > hard_gate:
                        continue

                    assign_tol = float(self.max_distance_ratio_error)
                    for image_idx in _candidate_assignments( # correspondences between image and model points
                        img_pts,
                        model_pts,
                        img_combo,
                        assign_tol,
                        exhaustive=(k <= 5),
                    ):
                        paired_img = image_points[list(image_idx)]
                        if not _orientation_consistent(paired_img, model_pts):
                            continue
                        estimate = self._solve_and_score(
                            image_points,
                            camera_matrix,
                            dist_coeffs,
                            model_combo,
                            image_idx,
                            use_extrinsic_guess=False,
                        )
                        if not estimate.ok:
                            continue
                        # Soft signature term only helps when coplanar-ish.
                        cost = _ranking_cost(estimate, self, signature_error=0.0)
                        if cost < best_cost:
                            best_cost = cost
                            best = estimate

            if (
                best.ok
                and best.n_used == k
                and best.support >= k
                and best.confidence >= 0.7
            ):
                break

        return best if best.confidence >= self.min_confidence else PoseEstimate.failed()

    # ------------------------------------------------------------------
    # Shared PnP + scoring
    # ------------------------------------------------------------------

    def _solve_and_score(
        self,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        model_indices: Sequence[int],
        image_indices: Sequence[int],
        use_extrinsic_guess: bool,
    ) -> PoseEstimate:
        obj = self.model_points[list(model_indices)].astype(np.float64)
        img = image_points[list(image_indices)].astype(np.float64)
        n = obj.shape[0]
        if n < self.min_markers:
            return PoseEstimate.failed()

        rvec0 = (
            self._prev_rvec.copy()
            if use_extrinsic_guess and self._prev_rvec is not None
            else np.zeros((3, 1), dtype=np.float64)
        )
        tvec0 = (
            self._prev_tvec.copy()
            if use_extrinsic_guess and self._prev_tvec is not None
            else np.zeros((3, 1), dtype=np.float64)
        )

        flags_to_try = [cv2.SOLVEPNP_SQPNP]
        rvecs: list[np.ndarray] = []
        tvecs: list[np.ndarray] = []
        for flags in flags_to_try:
            try:
                ok, rs, ts, _ = cv2.solvePnPGeneric(
                    obj.reshape(-1, 1, 3),
                    img.reshape(-1, 1, 2),
                    camera_matrix,
                    dist_coeffs,
                    flags=flags,
                    rvec=rvec0,
                    tvec=tvec0,
                    useExtrinsicGuess=use_extrinsic_guess,
                )
            except cv2.error:
                continue
            if ok and rs:
                rvecs.extend(rs)
                tvecs.extend(ts)

        if not rvecs:
            return PoseEstimate.failed()

        best_est = PoseEstimate.failed()
        best_cost = float("inf")
        for rvec, tvec in zip(rvecs, tvecs):
            mean_reproj = _mean_reprojection_error(
                obj, img, rvec, tvec, camera_matrix, dist_coeffs
            )
            if mean_reproj > self.max_reproj_px:
                continue
            if not _all_points_in_front(obj, rvec, tvec):
                continue
            support = _model_support(
                self.model_points,
                image_points,
                rvec,
                tvec,
                camera_matrix,
                dist_coeffs,
                self.max_assoc_px,
            )
            # if not enough model points are supported, reject the pose
            if support < self.min_markers:
                continue
            confidence = _confidence(
                mean_reproj_px=mean_reproj,
                n_used=n,
                n_model=self.model_points.shape[0],
                support=support,
                max_reproj_px=self.max_reproj_px,
                rvec=rvec,
                tvec=tvec,
                prev_rvec=self._prev_rvec,
                prev_tvec=self._prev_tvec,
            )
            est = PoseEstimate(
                ok=True,
                rvec=rvec.reshape(3, 1),
                tvec=tvec.reshape(3, 1),
                confidence=confidence,
                mean_reproj_px=mean_reproj,
                model_indices=tuple(model_indices),
                image_indices=tuple(image_indices),
                n_used=n,
                support=support,
            )
            cost = _ranking_cost(est, self)
            if cost < best_cost:
                best_cost = cost
                best_est = est
        return best_est


def _pairwise_distance_signature(points: np.ndarray) -> np.ndarray:
    """Sorted pairwise distances, normalized by the largest (scale-invariant)."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, points.shape[-1])
    n = pts.shape[0]
    if n < 2:
        return np.zeros(0, dtype=np.float64)
    dists: list[float] = []
    for i in range(n):
        for j in range(i + 1, n):
            dists.append(float(np.linalg.norm(pts[i] - pts[j])))
    sig = np.sort(np.asarray(dists, dtype=np.float64))
    scale = sig[-1]
    if scale < 1e-12:
        return np.zeros_like(sig)
    return sig / scale


def _signature_error(a: np.ndarray, b: np.ndarray) -> float:
    if a.shape != b.shape or a.size == 0:
        return float("inf")
    return float(np.max(np.abs(a - b)))


def _orientation_consistent(image_pts: np.ndarray, model_pts: np.ndarray) -> bool:
    """Reject assignments that flip triangle winding (common P3P swap failure)."""
    img = np.asarray(image_pts, dtype=np.float64).reshape(-1, 2)
    mdl = np.asarray(model_pts, dtype=np.float64).reshape(-1, 3)
    if img.shape[0] < 3:
        return True
    # Image signed area of first triangle.
    a = img[1] - img[0]
    b = img[2] - img[0]
    img_orient = a[0] * b[1] - a[1] * b[0]
    # Model signed area in its best-fit plane (use XY cross via normal).
    ab = mdl[1] - mdl[0]
    ac = mdl[2] - mdl[0]
    normal = np.cross(ab, ac)
    # Project into a stable 2D basis of the plane.
    if np.linalg.norm(normal) < 1e-12:
        return True
    n = normal / np.linalg.norm(normal)
    # Choose axis least aligned with normal as u.
    axis = np.array((0.0, 0.0, 1.0)) if abs(n[2]) < 0.9 else np.array((1.0, 0.0, 0.0))
    u = np.cross(n, axis)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    mdl2 = np.column_stack(((mdl - mdl[0]) @ u, (mdl - mdl[0]) @ v))
    a2 = mdl2[1] - mdl2[0]
    b2 = mdl2[2] - mdl2[0]
    mdl_orient = a2[0] * b2[1] - a2[1] * b2[0]
    return img_orient * mdl_orient > 0.0


def _point_fingerprints(pts: np.ndarray) -> np.ndarray:
    """(n, n-1) sorted neighbour distances per point, row-normalized."""
    pts = np.asarray(pts, dtype=np.float64)
    n = pts.shape[0]
    out = np.zeros((n, n - 1), dtype=np.float64)
    for i in range(n):
        d = np.sort(np.linalg.norm(pts - pts[i], axis=1))[1:]
        scale = d[-1] if d[-1] > 1e-12 else 1.0
        out[i] = d / scale
    return out


def _candidate_assignments(
    image_pts: np.ndarray,
    model_pts: np.ndarray,
    img_combo: tuple[int, ...],
    max_error: float,
    exhaustive: bool = False,
) -> list[tuple[int, ...]]:
    """Map model indices → image indices.
    """
    img = np.asarray(image_pts, dtype=np.float64)
    mdl = np.asarray(model_pts, dtype=np.float64)
    k = mdl.shape[0]
    fi = _point_fingerprints(img)
    fm = _point_fingerprints(mdl)

    cost = np.linalg.norm(fm[:, None, :] - fi[None, :, :], axis=2) # distance between model and image fingerprints
    used: set[int] = set()
    pairing: list[int] = [-1] * k # mapping of model indices to image indices
    for mi in np.argsort(cost.min(axis=1)):
        order = np.argsort(cost[mi]) # sort the image indices by the distance to the model point
        for ji in order:
            ji = int(ji)
            if ji in used:
                continue
            if cost[mi, ji] > max_error * np.sqrt(max(k - 1, 1)):
                break
            used.add(ji)
            pairing[int(mi)] = ji
            break
    assignments: list[tuple[int, ...]] = []
    if len(used) == k and all(p >= 0 for p in pairing):
        assignments.append(tuple(img_combo[p] for p in pairing)) # these are the possible correspondences between image and model points

    if exhaustive or (not assignments and k <= 4): # if less than the total number of markers seen, try all permutations
        scored: list[tuple[float, tuple[int, ...]]] = []
        for perm in permutations(range(k)):
            err = max(float(cost[mi, perm[mi]]) for mi in range(k))
            scored.append((err, tuple(img_combo[perm[mi]] for mi in range(k))))
        scored.sort()
        limit = min(len(scored), 120 if exhaustive else 6)
        for err, assignment in scored[:limit]:
            if assignment not in assignments:
                assignments.append(assignment)
    return assignments


def _all_points_in_front(
    object_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    min_depth: float = 1e-3,
) -> bool:
    """every used model point must sit in front of the camera"""
    R, _ = cv2.Rodrigues(rvec)
    pts = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    cam = (R @ pts.T).T + tvec.reshape(1, 3)
    return bool(np.all(cam[:, 2] > min_depth))


def _mean_reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(
        object_points.reshape(-1, 1, 3),
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    err = np.linalg.norm(projected.reshape(-1, 2) - image_points.reshape(-1, 2), axis=1)
    return float(err.mean())


def _model_support(
    model_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    max_assoc_px: float,
) -> int:
    """How many model points land near some detection under this pose."""
    projected, _ = cv2.projectPoints(
        model_points.reshape(-1, 1, 3),
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    projected = projected.reshape(-1, 2)
    dmat = np.linalg.norm(projected[:, None, :] - image_points[None, :, :], axis=2)
    used_image: set[int] = set()
    support = 0
    flat = sorted(
        (float(dmat[i, j]), i, j)
        for i in range(dmat.shape[0])
        for j in range(dmat.shape[1])
    )
    used_model: set[int] = set()
    for dist, i, j in flat:
        if dist > max_assoc_px:
            break
        if i in used_model or j in used_image:
            continue
        used_model.add(i)
        used_image.add(j)
        support += 1
    return support


def _rotation_angle(rvec_a: np.ndarray, rvec_b: np.ndarray) -> float:
    Ra, _ = cv2.Rodrigues(rvec_a)
    Rb, _ = cv2.Rodrigues(rvec_b)
    R = Ra @ Rb.T
    cos_theta = (np.trace(R) - 1.0) * 0.5
    return float(np.arccos(np.clip(cos_theta, -1.0, 1.0)))


def _confidence(
    mean_reproj_px: float,
    n_used: int,
    n_model: int,
    support: int,
    max_reproj_px: float,
    rvec: np.ndarray,
    tvec: np.ndarray,
    prev_rvec: np.ndarray | None,
    prev_tvec: np.ndarray | None,
) -> float:
    reproj_term = float(np.exp(-mean_reproj_px / max(max_reproj_px * 0.5, 1e-6)))
    coverage = support / max(n_model, 1)
    used_term = n_used / max(n_model, 1)
    temporal = 1.0
    if prev_rvec is not None and prev_tvec is not None:
        dt = float(np.linalg.norm(tvec.reshape(3) - prev_tvec.reshape(3)))
        dtheta = _rotation_angle(rvec, prev_rvec)
        temporal = float(np.exp(-dt / 0.05) * np.exp(-dtheta / 0.35))
    return float(
        np.clip(
            reproj_term * (0.35 + 0.35 * coverage + 0.30 * used_term) * (0.5 + 0.5 * temporal),
            0.0,
            1.0,
        )
    )

def _ranking_cost(
    estimate: PoseEstimate,
    tracker: MarkerPoseTracker,
    signature_error: float = 0.0,
) -> float:
    """Lower is better. Mixes reprojection, support, and temporal delta."""
    cost = estimate.mean_reproj_px
    cost += 3.0 * (tracker.model_points.shape[0] - estimate.support)
    cost += 1.5 * (tracker.model_points.shape[0] - estimate.n_used)
    # Soft distance-signature penalty (important for non-coplanar 2D↔3D mismatch).
    cost += 8.0 * signature_error
    if (
        tracker._prev_rvec is not None
        and tracker._prev_tvec is not None
        and estimate.rvec is not None
        and estimate.tvec is not None
    ):
        dt = float(np.linalg.norm(estimate.tvec.reshape(3) - tracker._prev_tvec.reshape(3)))
        dtheta = _rotation_angle(estimate.rvec, tracker._prev_rvec)
        cost += tracker.temporal_translation_weight * dt
        cost += tracker.temporal_rotation_weight * dtheta
    return cost


def centers_to_float(centers: Sequence[tuple[int, int]] | np.ndarray) -> np.ndarray:
    return np.asarray(centers, dtype=np.float64).reshape(-1, 2)