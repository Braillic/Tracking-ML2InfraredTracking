"""2D↔3D probe-marker correspondence and pose.

Per frame:
  1. Acquire image centers (e.g. from detect_marker_centers).
  2. Build candidates: treat detections as a subset of the known 3D probe model,
     can rule out using pairwise distance-ratio signatures.
  3. Rule out more using: geometric gate → solvePnP → reprojection + temporal score.
  4. Output pose and confidence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import combinations, permutations
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

# in meters, planar probe first
TEST_MARKER_COORDS = np.array(
    (
        (-0.0466, 0.0000, 0.0206),  # left
        (0.0444, 0.0000, 0.0206),   # right
        (0.0095, 0.0623, 0.0206),   # up
        (-0.0070, -0.0368, 0.0206), # bottom
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
    n_detected: int = 0
    n_unmatched_detections: int = 0
    coverage: float = 0.0  # fraction of min(n_detected, n_model) matched

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
    # Reject a solved pose that jumps more than this from the previous frame's
    # committed translation (metres). Filters minimal-config (3-marker) depth flips.
    max_translation_jump_m: float = 0.06
    # After this many consecutive jump-gate rejects, the reference pose is
    # assumed stale; drop it and re-acquire from scratch.
    max_consecutive_rejects: int = 3
    # Soft weights for ranking candidates (lower is better before → confidence).
    temporal_translation_weight: float = 50.0  # px-equivalent per metre
    temporal_rotation_weight: float = 30.0  # px-equivalent per radian
    # Reject if confidence below this after ranking.
    min_confidence: float = 0.15
    # Cold-start needs 4 markers: 3 coplanar points admit two equally-good
    # poses, so acquiring on 3 (with no prior to disambiguate) can lock onto
    # the collapsed branch. An already-running track still continues on 3.
    min_acquire_markers: int = 4
    # LM polish after PnP + one reassignment pass from projected model points.
    refine_pose: bool = True
    refine_iterations: int = 2
    # Added to ranking error for each visible detection left unmatched.
    unmatched_det_penalty_px: float = 12.0
    min_probe_z_m: float = 0.15
    max_probe_z_m: float = 1.50

    _reject_streak: int = field(default=0, init=False, repr=False)
    _prev_rvec: np.ndarray | None = field(default=None, init=False, repr=False)
    _prev_tvec: np.ndarray | None = field(default=None, init=False, repr=False)
    _prev_model_indices: tuple[int, ...] | None = field(default=None, init=False, repr=False)
    _prev_image_indices: tuple[int, ...] | None = field(default=None, init=False, repr=False)

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
        self.debug_counts = {"depth": 0, "min_markers": 0, "coverage": 0,
                             "reproj": 0, "support": 0, "in_front": 0}

    def reset(self) -> None:
        self._prev_rvec = None
        self._prev_tvec = None
        self._prev_model_indices = None
        self._prev_image_indices = None
        self._reject_streak = 0

    def _expected_matches(self, n_detected: int) -> int:
        return min(n_detected, self.model_points.shape[0])

    def _commit(self, estimate: PoseEstimate) -> PoseEstimate:
        self._prev_rvec = estimate.rvec
        self._prev_tvec = estimate.tvec
        self._prev_model_indices = estimate.model_indices
        self._prev_image_indices = estimate.image_indices
        return estimate

    def estimate(
        self,
        image_points: Sequence[tuple[float, float]] | np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray | None = None,
    ) -> PoseEstimate:
        pts = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
        n_detected = pts.shape[0]
        if n_detected < self.min_markers and self._prev_rvec is None:
            return PoseEstimate.failed()

        dist = np.zeros(5, dtype=np.float64) if dist_coeffs is None else np.asarray(
            dist_coeffs, dtype=np.float64
        ).reshape(-1)
        K = np.asarray(camera_matrix, dtype=np.float64)

        # A long reject streak means our reference pose is stale. Drop it and
        # re-acquire this frame instead of comparing against a frozen pose.
        if self._reject_streak >= self.max_consecutive_rejects:
            print(f"[JUMP GATE] lost tracking after {self._reject_streak} rejects — re-acquiring")
            self.reset()

        if self._prev_rvec is not None and self._prev_tvec is not None:
            temporal = self._estimate_temporal(pts, K, dist, n_detected)
            if (temporal.ok and temporal.confidence >= self.min_confidence
                    and self._jump_plausible(temporal)):
                self._reject_streak = 0
                return self._commit(temporal)
            
        if self._prev_rvec is None and n_detected < self.min_acquire_markers:
            self._reject_streak = 0
            return PoseEstimate.failed()

        combinatorial = self._estimate_combinatorial(pts, K, dist, n_detected)
        # _jump_plausible returns True when there's no prev (fresh acquisition).
        if combinatorial.ok and self._jump_plausible(combinatorial):
            self._reject_streak = 0
            return self._commit(combinatorial)

        self._reject_streak += 1
        return PoseEstimate.failed()
    
    def _jump_plausible(self, est: PoseEstimate) -> bool:
        if self._prev_tvec is None or est.tvec is None:
            return True  # no reference yet (startup) — nothing to compare against
        dt = float(np.linalg.norm(est.tvec.reshape(3) - self._prev_tvec.reshape(3)))
        if dt > self.max_translation_jump_m:
            print(f"[JUMP GATE] reject dt={dt*1000:.1f}mm  n_used={est.n_used}  "
                  f"reproj={est.mean_reproj_px:.2f}px  "
                  f"z={float(est.tvec[2,0]):.3f}  prev_z={float(self._prev_tvec.reshape(3)[2]):.3f}")
            return False
        return True

    # ------------------------------------------------------------------
    # Temporal path: project previous pose, keep nearest detection
    # ------------------------------------------------------------------

    def _estimate_temporal(
        self,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        n_detected: int,
    ) -> PoseEstimate:
        # use previous pose to project model points and find correspondences
        # using mutual nearest assignment to find correspondences
        # (prefer previous correspondences when still close)
        projected, _ = cv2.projectPoints(
            self.model_points.reshape(-1, 1, 3),
            self._prev_rvec,
            self._prev_tvec,
            camera_matrix,
            dist_coeffs,
        )
        model_idx, image_idx = _assign_mutual_nearest(
            projected.reshape(-1, 2),
            image_points,
            self.max_assoc_px,
            preferred_model=self._prev_model_indices,
            preferred_image=self._prev_image_indices,
        )
        if len(model_idx) < self.min_markers:
            if self.debug:
                proj = projected.reshape(-1, 2)
                img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
                dists = np.linalg.norm(proj[:, None, :] - img[None, :, :], axis=2)
                nearest = dists.min(axis=1) if dists.size else np.array([])
                print(
                    f"[MarkerPoseDiag] temporal gate failed: matched={len(model_idx)} "
                    f"n_detected={n_detected} n_model={proj.shape[0]} max_assoc_px={self.max_assoc_px} "
                    f"nearest_dist_per_model_pt={np.round(nearest, 1).tolist()}"
                )
            return PoseEstimate.failed()

        return self._solve_and_score(
            image_points,
            camera_matrix,
            dist_coeffs,
            model_idx,
            image_idx,
            n_detected,
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
        n_detected: int,
    ) -> PoseEstimate:
        n_img = image_points.shape[0]  # number of detected marker centers
        n_model = self.model_points.shape[0]  # number of model points
        k_values = range(self.min_markers, min(n_img, n_model) + 1)  # number of points to use for PnP

        best = PoseEstimate.failed()
        best_cost = float("inf")

        model_sig_cache: dict[tuple[int, ...], np.ndarray] = {}
        # hard gate: max relative error on sorted pairwise ratios
        hard_gate = float(self.max_distance_ratio_error)

        for k in reversed(list(k_values)):  # big loop over all possible combinations of model and image points
            img_sig_cache: dict[tuple[int, ...], np.ndarray] = {}
            for img_combo in combinations(range(n_img), k):  # detected marker subsets
                img_pts = image_points[list(img_combo)]
                img_sig = img_sig_cache.setdefault(
                    img_combo, _pairwise_distance_signature(img_pts)  # normalized pairwise distances between the image points
                )
                for model_combo in combinations(range(n_model), k):  # model subsets
                    model_pts = self.model_points[list(model_combo)]
                    model_sig = model_sig_cache.setdefault(
                        model_combo, _pairwise_distance_signature(model_pts)
                    )
                    sig_err = _signature_error(img_sig, model_sig)  # distance between image and model pairwise distances
                    if sig_err > hard_gate:
                        continue

                    for image_idx in _candidate_assignments(  # correspondences between image and model points
                        img_pts,
                        model_pts,
                        img_combo,
                        hard_gate,
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
                            n_detected,
                            use_extrinsic_guess=False,
                        )
                        if not estimate.ok:
                            continue
                        cost = _ranking_cost(estimate, self)
                        if cost < best_cost:
                            best_cost = cost
                            best = estimate

            # good enough at this subset size — no need to try smaller k
            if (
                best.ok
                and best.n_used == k
                and best.confidence >= 0.7
                and (best.support >= k or best.coverage >= 1.0)
            ):
                break

        return best if best.confidence >= self.min_confidence else PoseEstimate.failed()

    def _solve_and_score(
        self,
        image_points: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        model_indices: Sequence[int],
        image_indices: Sequence[int],
        n_detected: int,
        use_extrinsic_guess: bool,
    ) -> PoseEstimate:
        obj = self.model_points[list(model_indices)].astype(np.float64)
        img = image_points[list(image_indices)].astype(np.float64)
        if obj.shape[0] < self.min_markers:
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

        # Tried SOLVEPNP_IPPE here for the coplanar (z=0) model_points, expecting
        # it to handle the planar-pose ambiguity better than SQPNP. In practice it
        # returned ok=0/no candidates for essentially every input in this OpenCV
        # build - including the identity correspondence - so it's not usable here.
        # Back to SQPNP, which at least reliably returns candidates (the ambiguity
        # is instead handled downstream by picking whichever valid candidate is
        # closest to the previous pose).
        flags = cv2.SOLVEPNP_SQPNP
        pass_guess = use_extrinsic_guess
        diag = use_extrinsic_guess and self.debug  # only trace the temporal-path calls
        try:
            ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                obj.reshape(-1, 1, 3),
                img.reshape(-1, 1, 2),
                camera_matrix,
                dist_coeffs,
                flags=flags,
                rvec=rvec0,
                tvec=tvec0,
                useExtrinsicGuess=pass_guess,
            )
        except cv2.error as e:
            if diag:
                print(f"[MarkerPoseDiag]   solvePnPGeneric raised: {e} "
                      f"(model_idx={list(model_indices)} image_idx={list(image_indices)} "
                      f"obj={obj.tolist()} img={img.tolist()})")
            return PoseEstimate.failed()
        if diag and (not ok or not rvecs):
            print(f"[MarkerPoseDiag]   solvePnPGeneric returned ok={ok} rvecs={rvecs} "
                  f"(model_idx={list(model_indices)} image_idx={list(image_indices)} "
                  f"obj={obj.tolist()} img={img.tolist()})")
        rvecs = list(rvecs) if rvecs else []
        tvecs = list(tvecs) if tvecs else []

        # Rescue for the temporal path: SQPNP is a global solver and can return
        # only degenerate (behind-camera) candidates for a coplanar target near
        # the planar-pose ambiguity, or fail outright. SOLVEPNP_ITERATIVE seeded
        # with the previous pose is a local optimizer instead - anchored at
        # continuity with where we already know the object was, so it tends to
        # converge back to the same physically-valid solution rather than landing
        # on the ambiguous flip. Fed into the same scoring loop below so it only
        # wins if it's actually a good fit, not blindly trusted.
        if pass_guess:
            try:
                ok_iter, rvec_iter, tvec_iter = cv2.solvePnP(
                    obj.reshape(-1, 1, 3),
                    img.reshape(-1, 1, 2),
                    camera_matrix,
                    dist_coeffs,
                    rvec=rvec0.copy(),
                    tvec=tvec0.copy(),
                    useExtrinsicGuess=True,
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )
                if ok_iter:
                    rvecs.append(rvec_iter)
                    tvecs.append(tvec_iter)
                    if diag:
                        print("[MarkerPoseDiag]   +iterative-guess candidate added")
            except cv2.error as e:
                if diag:
                    print(f"[MarkerPoseDiag]   iterative-guess rescue raised: {e}")

        if not rvecs:
            return PoseEstimate.failed()

        best_est = PoseEstimate.failed()
        best_cost = float("inf")
        expected = self._expected_matches(n_detected)
        if diag:
            print(f"[MarkerPoseDiag]   solve_and_score: {len(rvecs)} candidate(s) total, "
                  f"input model_idx={list(model_indices)} image_idx={list(image_indices)}")
        for cand_i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            if not _all_points_in_front(obj, rvec, tvec):
                self.debug_counts["in_front"] += 1
                continue

            candidate_z = float(np.asarray(tvec, dtype=np.float64).reshape(3)[2])
            if not (self.min_probe_z_m <= candidate_z <= self.max_probe_z_m):
                self.debug_counts["depth"] += 1
                continue

            rvec_f, tvec_f, mi_f, ii_f = _refine_pose_and_reassign(
                self.model_points,
                image_points,
                rvec,
                tvec,
                model_indices,
                image_indices,
                camera_matrix,
                dist_coeffs,
                self.max_assoc_px,
                refine=self.refine_pose,
                iterations=self.refine_iterations,
            )
            # keeping track of # of markers pose predicts as a ratio to how many are expected based on detections
            n_used = len(mi_f)
            coverage = n_used / max(expected, 1)
            unmatched_det = max(0, n_detected - len(set(ii_f)))

            if n_used < self.min_markers:
                self.debug_counts["min_markers"] += 1
                continue
            # When enough markers are visible, require full min-side coverage.
            if n_detected >= self.min_markers and n_used < min(expected, n_detected - 1):
                self.debug_counts["coverage"] += 1
                continue

            mean_reproj = _mean_reprojection_error(
                self.model_points[list(mi_f)],
                image_points[list(ii_f)],
                rvec_f,
                tvec_f,
                camera_matrix,
                dist_coeffs,
            )
            if mean_reproj > self.max_reproj_px:
                self.debug_counts["reproj"] += 1
                continue

            support = _model_support(
                self.model_points,
                image_points,
                rvec_f,
                tvec_f,
                camera_matrix,
                dist_coeffs,
                self.max_assoc_px,
            )
            if support < self.min_markers:
                self.debug_counts["support"] += 1
                continue
            if diag:
                print(f"[MarkerPoseDiag]   cand {cand_i}: passed all filters, reproj={mean_reproj:.2f}px "
                      f"support={support} n_used={n_used}")

            confidence = _confidence(
                mean_reproj_px=mean_reproj,
                n_model=self.model_points.shape[0],
                n_detected=n_detected,
                support=support,
                coverage=coverage,
                unmatched_det=unmatched_det,
                max_reproj_px=self.max_reproj_px,
                rvec=rvec_f,
                tvec=tvec_f,
                prev_rvec=self._prev_rvec,
                prev_tvec=self._prev_tvec,
            )
            est = PoseEstimate(
                ok=True,
                rvec=rvec_f.reshape(3, 1),
                tvec=tvec_f.reshape(3, 1),
                confidence=confidence,
                mean_reproj_px=mean_reproj,
                model_indices=tuple(mi_f),
                image_indices=tuple(ii_f),
                n_used=n_used,
                support=support,
                n_detected=n_detected,
                n_unmatched_detections=unmatched_det,
                coverage=coverage,
            )
            cost = _ranking_cost(est, self)
            if cost < best_cost:
                best_cost = cost
                best_est = est
        return best_est


def _assign_mutual_nearest(
    projected: np.ndarray,
    image_points: np.ndarray,
    max_gate_px: float,
    preferred_model: Sequence[int] | None = None,
    preferred_image: Sequence[int] | None = None,
) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Greedy mutual-nearest assignment between projected model points and detections."""
    proj = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    dmat = np.linalg.norm(proj[:, None, :] - img[None, :, :], axis=2)

    model_idx: list[int] = []
    image_idx: list[int] = []
    used_model: set[int] = set()
    used_image: set[int] = set()

    # Prefer keeping prior correspondences when still close.
    if preferred_model and preferred_image:
        for mi, ii in zip(preferred_model, preferred_image):
            if mi in used_model or ii in used_image:
                continue
            if mi >= dmat.shape[0] or ii >= dmat.shape[1]:
                continue
            if float(dmat[mi, ii]) <= max_gate_px:
                used_model.add(int(mi))
                used_image.add(int(ii))
                model_idx.append(int(mi))
                image_idx.append(int(ii))

    flat = sorted(
        (float(dmat[i, j]), i, j)
        for i in range(dmat.shape[0])
        for j in range(dmat.shape[1])
    )
    for dist, i, j in flat:
        if dist > max_gate_px:
            break
        if i in used_model or j in used_image:
            continue
        if int(np.argmin(dmat[i])) != j:
            continue
        if int(np.argmin(dmat[:, j])) != i:
            continue
        used_model.add(i)
        used_image.add(j)
        model_idx.append(i)
        image_idx.append(j)
    return tuple(model_idx), tuple(image_idx)


def _refine_pose_lm(
    object_points: np.ndarray,
    image_points: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    obj = np.asarray(object_points, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    r = np.asarray(rvec, dtype=np.float64).reshape(3, 1).copy()
    t = np.asarray(tvec, dtype=np.float64).reshape(3, 1).copy()
    try:
        r, t = cv2.solvePnPRefineLM(
            obj.reshape(-1, 1, 3),
            img.reshape(-1, 1, 2),
            camera_matrix,
            dist_coeffs,
            r,
            t,
        )
    except cv2.error:
        pass
    return r, t


def _refine_pose_and_reassign(
    model_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    model_indices: Sequence[int],
    image_indices: Sequence[int],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    max_assoc_px: float,
    *,
    refine: bool,
    iterations: int,
) -> tuple[np.ndarray, np.ndarray, tuple[int, ...], tuple[int, ...]]:
    """LM polish, then re-project all model points and refresh correspondences."""
    mi = tuple(int(i) for i in model_indices)
    ii = tuple(int(i) for i in image_indices)
    r = np.asarray(rvec, dtype=np.float64).reshape(3, 1).copy()
    t = np.asarray(tvec, dtype=np.float64).reshape(3, 1).copy()

    for _ in range(max(1, iterations)):
        if refine and len(mi) >= 3:
            r, t = _refine_pose_lm(
                model_points[list(mi)],
                image_points[list(ii)],
                camera_matrix,
                dist_coeffs,
                r,
                t,
            )

        projected, _ = cv2.projectPoints(
            model_points.reshape(-1, 1, 3),
            r,
            t,
            camera_matrix,
            dist_coeffs,
        )
        mi, ii = _assign_mutual_nearest(
            projected.reshape(-1, 2),
            image_points,
            max_assoc_px,
            preferred_model=mi,
            preferred_image=ii,
        )
        if len(mi) < 3:
            break

    return r, t, mi, ii


def pairwise_distances(points: np.ndarray) -> np.ndarray:
    """Upper-triangle pairwise distances for NxD points → length N*(N-1)/2."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(-1, pts.size)
    elif pts.ndim > 2:
        pts = pts.reshape(-1, pts.shape[-1])
    n = pts.shape[0]
    if n < 2:
        return np.zeros(0, dtype=np.float64)
    dists: list[float] = []
    for i in range(n):
        for j in range(i + 1, n):
            dists.append(float(np.linalg.norm(pts[i] - pts[j])))
    return np.asarray(dists, dtype=np.float64)


def pairwise_distance_signature(points: np.ndarray) -> np.ndarray:
    """Sorted pairwise distances, normalized by the largest (scale-invariant)."""
    sig = np.sort(pairwise_distances(points))
    if sig.size == 0:
        return sig
    scale = float(sig[-1])
    if scale < 1e-12:
        return np.zeros_like(sig)
    return sig / scale


def signature_error(a: np.ndarray, b: np.ndarray) -> float:
    """Max absolute difference between two distance signatures."""
    if a.shape != b.shape or a.size == 0:
        return float("inf")
    return float(np.max(np.abs(a - b)))


# Back-compat private aliases (existing MarkerPoseTracker call sites).
_pairwise_distance_signature = pairwise_distance_signature
_signature_error = signature_error


def nearest_neighbor_distances(points: np.ndarray) -> np.ndarray:
    """Per-point distance to the nearest other point (inf if only one point)."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(-1, 1)
    n = pts.shape[0]
    if n == 0:
        return np.zeros(0, dtype=np.float64)
    if n == 1:
        return np.array([np.inf], dtype=np.float64)
    nn = np.full(n, np.inf, dtype=np.float64)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            d = float(np.linalg.norm(pts[i] - pts[j]))
            nn[i] = min(nn[i], d)
    return nn


def filter_isolated_centers(
    centers: Sequence[tuple[float, float]] | np.ndarray,
    *,
    max_nearest_neighbor_px: float,
) -> list[tuple[int, int]]:
    """Drop detections whose nearest neighbor is farther than max_nearest_neighbor_px."""
    pts = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 2:
        return []
    nn = nearest_neighbor_distances(pts)
    return [
        (round(pts[i, 0]), round(pts[i, 1]))
        for i, d in enumerate(nn)
        if d <= max_nearest_neighbor_px
    ]


def filter_centers_by_span(
    centers: Sequence[tuple[float, float]] | np.ndarray,
    *,
    max_cluster_span_px: float,
) -> list[tuple[int, int]]:
    """Keep the densest subset whose diameter (max pairwise distance) is ≤ max span."""
    pts = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] <= 1:
        return []
    n = pts.shape[0]
    best: list[int] = []
    best_span = float("inf")

    for seed in range(n):
        order = np.argsort(np.linalg.norm(pts - pts[seed], axis=1))
        chosen: list[int] = []
        for idx in order.tolist():
            trial = chosen + [idx]
            span = (
                float(np.max(pairwise_distances(pts[trial]))) if len(trial) >= 2 else 0.0
            )
            if span <= max_cluster_span_px:
                chosen = trial
            elif chosen:
                break
        trial_span = (
            float(np.max(pairwise_distances(pts[chosen]))) if len(chosen) >= 2 else 0.0
        )
        if len(chosen) > len(best) or (
            len(chosen) == len(best) and len(chosen) >= 2 and trial_span < best_span
        ):
            best = chosen
            best_span = trial_span

    return [(round(pts[i, 0]), round(pts[i, 1])) for i in best]


def filter_centers_by_model_geometry(
    centers: Sequence[tuple[float, float]] | np.ndarray,
    model_points: np.ndarray,
    *,
    min_cluster_size: int,
    max_geometry_ratio_error: float,
) -> list[tuple[int, int]]:
    """Keep the largest detection subset whose pairwise distance ratios match the model."""
    pts = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
    model = np.asarray(model_points, dtype=np.float64).reshape(-1, 3)
    n = pts.shape[0]
    m = model.shape[0]
    if n < min_cluster_size or m < min_cluster_size:
        return []

    best_idx: tuple[int, ...] = ()
    best_err = float("inf")

    for k in range(min(n, m), min_cluster_size - 1, -1):
        model_sigs = [
            (pairwise_distance_signature(model[list(mi)]), mi)
            for mi in combinations(range(m), k)
        ]
        for img_i in combinations(range(n), k):
            img_sig = pairwise_distance_signature(pts[list(img_i)])
            for model_sig, _ in model_sigs:
                err = signature_error(img_sig, model_sig)
                if err <= max_geometry_ratio_error and (
                    len(img_i) > len(best_idx)
                    or (len(img_i) == len(best_idx) and err < best_err)
                ):
                    best_idx = img_i
                    best_err = err
        if best_idx:
            break

    return [(round(pts[i, 0]), round(pts[i, 1])) for i in best_idx]


def filter_marker_centers(
    centers: Sequence[tuple[float, float]] | np.ndarray,
    *,
    model_points: np.ndarray | None = None,
    max_nearest_neighbor_px: float = 90.0,
    max_cluster_span_px: float = 220.0,
    min_cluster_size: int = 3,
    max_geometry_ratio_error: float = 0.35,
) -> list[tuple[int, int]]:
    """Remove isolated / geometrically implausible marker detections.

    Pipeline:
      1. Drop points with no neighbor within max_nearest_neighbor_px.
      2. Keep a compact cluster (diameter ≤ max_cluster_span_px).
      3. Keep the largest subset whose pairwise distance ratios match the 3D model.
    """
    pts = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < min_cluster_size:
        return []

    model = TEST_MARKER_COORDS if model_points is None else model_points
    as_tuples = [
        (int(round(p[0])), int(round(p[1]))) for p in pts
    ]
    nearby = filter_isolated_centers(
        as_tuples, max_nearest_neighbor_px=max_nearest_neighbor_px
    )
    if len(nearby) < min_cluster_size:
        return []

    compact = filter_centers_by_span(
        nearby, max_cluster_span_px=max_cluster_span_px
    )
    if len(compact) < min_cluster_size:
        return []

    geometric = filter_centers_by_model_geometry(
        compact,
        model,
        min_cluster_size=min_cluster_size,
        max_geometry_ratio_error=max_geometry_ratio_error,
    )
    return geometric if geometric else compact


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
    """how the points are arranged relative to each other, in terms of normalized distance to every other point"""
    pts = np.asarray(pts, dtype=np.float64)
    n = pts.shape[0]
    out = np.zeros((n, n - 1), dtype=np.float64)
    for i in range(n):
        d = np.sort(np.linalg.norm(pts - pts[i], axis=1))[1:]
        scale = d[-1] if d[-1] > 1e-12 else 1.0
        # per row normalization
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
        for _, assignment in scored[:limit]:
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
    mi, _ = _assign_mutual_nearest(
        projected.reshape(-1, 2),
        image_points,
        max_assoc_px,
    )
    return len(mi)


def _rotation_angle(rvec_a: np.ndarray, rvec_b: np.ndarray) -> float:
    Ra, _ = cv2.Rodrigues(rvec_a)
    Rb, _ = cv2.Rodrigues(rvec_b)
    R = Ra @ Rb.T
    cos_theta = (np.trace(R) - 1.0) * 0.5
    return float(np.arccos(np.clip(cos_theta, -1.0, 1.0)))


def _pose_delta(
    rvec: np.ndarray,
    tvec: np.ndarray,
    prev_rvec: np.ndarray,
    prev_tvec: np.ndarray,
) -> tuple[float, float]:
    # how much the pose has changed since last frame
    dt = float(np.linalg.norm(tvec.reshape(3) - prev_tvec.reshape(3)))
    dtheta = _rotation_angle(rvec, prev_rvec)
    return dt, dtheta


def _confidence(
    mean_reproj_px: float,
    n_model: int,
    n_detected: int,
    support: int,
    coverage: float,
    unmatched_det: int,
    max_reproj_px: float,
    rvec: np.ndarray,
    tvec: np.ndarray,
    prev_rvec: np.ndarray | None,
    prev_tvec: np.ndarray | None,
) -> float:
    reproj_term = float(np.exp(-mean_reproj_px / max(max_reproj_px * 0.5, 1e-6)))
    expected = min(n_detected, n_model)
    coverage_term = coverage if expected > 0 else 0.0
    model_term = support / max(n_model, 1)
    # if detection not paired to a model point, penalize
    orphan_penalty = float(np.exp(-0.75 * unmatched_det))
    temporal = 1.0
    if prev_rvec is not None and prev_tvec is not None:
        dt, dtheta = _pose_delta(rvec, tvec, prev_rvec, prev_tvec)
        temporal = float(np.exp(-dt / 0.05) * np.exp(-dtheta / 0.35))
    return float(
        np.clip(
            reproj_term
            * (0.20 + 0.45 * coverage_term + 0.35 * model_term)
            * orphan_penalty
            * (0.5 + 0.5 * temporal),
            0.0,
            1.0,
        )
    )


def _ranking_cost(
    estimate: PoseEstimate,
    tracker: MarkerPoseTracker,
) -> float:
    """Lower is better. Mixes reprojection, coverage, and temporal delta."""
    expected = tracker._expected_matches(estimate.n_detected)
    cost = estimate.mean_reproj_px
    cost += tracker.unmatched_det_penalty_px * estimate.n_unmatched_detections
    cost += 8.0 * max(0, expected - estimate.n_used)
    cost += 3.0 * (tracker.model_points.shape[0] - estimate.support)
    if estimate.coverage < 1.0 and estimate.n_detected >= tracker.min_markers:
        cost += 20.0 * (1.0 - estimate.coverage)
    if (
        tracker._prev_rvec is not None
        and tracker._prev_tvec is not None
        and estimate.rvec is not None
        and estimate.tvec is not None
    ):
        dt, dtheta = _pose_delta(
            estimate.rvec, estimate.tvec, tracker._prev_rvec, tracker._prev_tvec
        )
        cost += tracker.temporal_translation_weight * dt
        cost += tracker.temporal_rotation_weight * dtheta
    return cost


def centers_to_float(centers: Sequence[tuple[int, int]] | np.ndarray) -> np.ndarray:
    return np.asarray(centers, dtype=np.float64).reshape(-1, 2)

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