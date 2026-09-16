"""2D↔3D probe-marker correspondence and pose.

Per frame:
  1. Acquire image centers (e.g. from detect_marker_centers).
  2. Build candidates: treat detections as a subset of the known 3D probe model,
     can rule out using pairwise distance-ratio signatures.
  3. Rule out more using: geometric gate → solvePnP → reprojection + temporal score.
  4. Output pose and confidence.
"""

from __future__ import annotations

import time
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from itertools import combinations, permutations
from pathlib import Path

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
    n_unmatched_detections: int = 0  # unmatched expected slots; excludes surplus clutter
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
    lost_timeout_s: float = 0.65  # allows two missed observations at 5 Hz
    acquisition_timeout_s: float = 0.65
    rotation_margin_deg: float = 15.0
    max_angular_speed_deg_s: float = 360.0
    max_rotation_jump_deg: float = 60.0
    position_margin_m: float = 0.015
    max_relative_speed_m_s: float = 1.0
    max_translation_jump_m: float = 0.20  # upper bound on the time-dependent gate
    acquire_confirmation_frames: int = 2
    acquire_rotation_margin_deg: float = 15.0
    acquire_max_angular_speed_deg_s: float = 360.0
    max_candidates: int = 8
    max_search_hypotheses: int = 32
    search_budget_ms: float = 12.0  # checked between solver calls
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
    # Added to ranking error for each expected model match left unmatched.
    unmatched_det_penalty_px: float = 12.0
    min_probe_z_m: float = 0.15
    max_probe_z_m: float = 1.50
    # Emit detailed diagnostics for temporal association and PnP failures.
    debug: bool = False

    state: str = field(default="acquiring", init=False)
    _last_observation_time: float | None = field(default=None, init=False, repr=False)
    _last_accepted_time: float | None = field(default=None, init=False, repr=False)
    _acquire_candidate: PoseEstimate | None = field(default=None, init=False, repr=False)
    _acquire_time: float | None = field(default=None, init=False, repr=False)
    _acquire_count: int = field(default=0, init=False, repr=False)
    _camera_world_transform: np.ndarray | None = field(default=None, init=False, repr=False)
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
        self.debug_counts = {"depth": 0, "min_markers": 0, "coverage": 0,
                             "reproj": 0, "support": 0, "in_front": 0}

    def reset(self) -> None:
        self._camera_world_transform = None
        self._prev_rvec = None
        self._prev_tvec = None
        self.state = "acquiring"
        self._last_observation_time = None
        self._last_accepted_time = None
        self._clear_acquisition()

    def _clear_acquisition(self) -> None:
        self._acquire_candidate = None
        self._acquire_time = None
        self._acquire_count = 0

    def _lose_tracking(self) -> None:
        self.reset()
        self.state = "lost"
        if self.debug:
            print("[MarkerPoseDiag] tracking lost; cleared pose prior")

    def _expected_matches(self, n_detected: int) -> int:
        return min(n_detected, self.model_points.shape[0])

    def _commit(self, estimate: PoseEstimate) -> PoseEstimate:
        self._prev_rvec = estimate.rvec
        self._prev_tvec = estimate.tvec
        self._last_accepted_time = self._last_observation_time
        self.state = "tracking"
        self._clear_acquisition()
        return estimate

    def estimate(
        self,
        image_points: Sequence[tuple[float, float]] | np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray | None = None,
        *,
        observation_time: float | None = None,
        camera_world_transform: np.ndarray | None = None,
    ) -> PoseEstimate:
        """Process every observation, including empty ones, in timestamp order.

        Live callers supply the frame's source timestamp in seconds. The current
        wire timestamp is ML2 submit time (a capture-time proxy). Offline callers
        should use recording timestamps, not the speed of the replay loop.
        """
        now = time.monotonic() if observation_time is None else float(observation_time)
        if not np.isfinite(now):
            raise ValueError("observation_time must be finite")
        if self._last_observation_time is not None and now <= self._last_observation_time:
            return PoseEstimate.failed()  # duplicate/older frames cannot refresh tracking
        if self._last_accepted_time is not None and now - self._last_accepted_time >= self.lost_timeout_s:
            self._lose_tracking()
        # Express every prior in the CURRENT camera frame. Differences then
        # measure probe world motion; headset motion also updates projected
        # marker association, the iterative PnP seed, and acquisition checks.
        self._rebase_camera(camera_world_transform)
        self._last_observation_time = now
        pts = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
        pts = pts[np.isfinite(pts).all(axis=1)][:self.max_candidates]
        n_detected = pts.shape[0]
        if n_detected < self.min_markers:
            self._clear_acquisition()
            return PoseEstimate.failed()

        dist = np.zeros(5, dtype=np.float64) if dist_coeffs is None else np.asarray(
            dist_coeffs, dtype=np.float64
        ).reshape(-1)
        K = np.asarray(camera_matrix, dtype=np.float64)

        if self._prev_rvec is not None and self._prev_tvec is not None:
            temporal = self._estimate_temporal(pts, K, dist, n_detected)
            if (temporal.ok and temporal.confidence >= self.min_confidence
                    and self._jump_plausible(temporal)):
                return self._commit(temporal)
            
        if self._prev_rvec is None and n_detected < self.min_acquire_markers:
            self._clear_acquisition()
            return PoseEstimate.failed()

        combinatorial = self._estimate_combinatorial(pts, K, dist, n_detected)
        if self._prev_rvec is None:
            return self._confirm_acquisition(combinatorial, now)
        if combinatorial.ok and self._jump_plausible(combinatorial):
            return self._commit(combinatorial)

        return PoseEstimate.failed()

    def _rebase_camera(self, camera_world_transform: np.ndarray | None) -> None:
        current = None
        if camera_world_transform is not None:
            current = np.asarray(camera_world_transform, dtype=np.float64).reshape(4, 4)
            if (not np.isfinite(current).all()
                    or not np.allclose(current[3], [0, 0, 0, 1])
                    or not np.allclose(current[:3, :3].T @ current[:3, :3], np.eye(3), atol=1e-5)):
                raise ValueError("camera_world_transform must be a finite rigid transform")
        previous = self._camera_world_transform
        if (current is None) != (previous is None):
            # Switching coordinate conventions cannot reuse a camera-only prior.
            self.reset()
        elif current is not None:
            delta_r = current[:3, :3].T @ previous[:3, :3]
            delta_t = current[:3, :3].T @ (previous[:3, 3] - current[:3, 3])
            if np.linalg.det(delta_r) < 0:
                self.reset()
            else:
                def rebase(rvec, tvec):
                    r, _ = cv2.Rodrigues(rvec)
                    return (cv2.Rodrigues(delta_r @ r)[0],
                            (delta_r @ tvec.reshape(3) + delta_t).reshape(3, 1))
                if self._prev_rvec is not None:
                    self._prev_rvec, self._prev_tvec = rebase(self._prev_rvec, self._prev_tvec)
                if self._acquire_candidate is not None:
                    rvec, tvec = rebase(self._acquire_candidate.rvec, self._acquire_candidate.tvec)
                    self._acquire_candidate = replace(self._acquire_candidate, rvec=rvec, tvec=tvec)
        self._camera_world_transform = None if current is None else current.copy()

    def _displacement_limit(self, elapsed_s: float) -> float:
        return min(self.max_translation_jump_m,
                   self.position_margin_m + self.max_relative_speed_m_s * max(0.0, elapsed_s))

    def _confirm_acquisition(self, candidate: PoseEstimate, now: float) -> PoseEstimate:
        if not candidate.ok or candidate.n_used < self.min_acquire_markers:
            self._clear_acquisition()
            return PoseEstimate.failed()
        self.state = "acquiring"
        consistent = False
        if self._acquire_candidate is not None and self._acquire_time is not None:
            elapsed = now - self._acquire_time
            displacement, angle = _pose_delta(
                candidate.rvec, candidate.tvec,
                self._acquire_candidate.rvec, self._acquire_candidate.tvec)
            angular_limit = min(60.0, self.acquire_rotation_margin_deg +
                                self.acquire_max_angular_speed_deg_s * elapsed)
            consistent = (0 < elapsed < self.acquisition_timeout_s
                          and displacement <= self._displacement_limit(elapsed)
                          and np.degrees(angle) <= angular_limit)
        self._acquire_count = self._acquire_count + 1 if consistent else 1
        self._acquire_candidate = candidate
        self._acquire_time = now
        if self._acquire_count >= self.acquire_confirmation_frames:
            return self._commit(candidate)
        return PoseEstimate.failed()
    
    def _jump_plausible(self, est: PoseEstimate) -> bool:
        if self._prev_tvec is None or est.tvec is None:
            return True  # no reference yet (startup) — nothing to compare against
        displacement, angle = _pose_delta(est.rvec, est.tvec, self._prev_rvec, self._prev_tvec)
        elapsed = self._last_observation_time - self._last_accepted_time
        limit = self._displacement_limit(elapsed)
        angular_limit = min(self.max_rotation_jump_deg,
                            self.rotation_margin_deg + self.max_angular_speed_deg_s * max(0.0, elapsed))
        if displacement > limit or np.degrees(angle) > angular_limit:
            if self.debug:
                print(f"[JUMP GATE] reject displacement={displacement*1000:.1f}mm "
                      f"limit={limit*1000:.1f}mm angle={np.degrees(angle):.1f}deg "
                      f"angular_limit={angular_limit:.1f}deg elapsed={elapsed:.3f}s")
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
        # using a global one-to-one assignment independent of detection order.
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
        # Rank cheap geometric hypotheses first, then bound expensive PnP work.
        n_img = len(image_points)
        n_model = len(self.model_points)
        minimum = self.min_acquire_markers if self._prev_rvec is None else self.min_markers
        hypotheses = []
        gate = float(self.max_distance_ratio_error)
        for k in range(min(n_img, n_model), minimum - 1, -1):
            models = []
            for mi in combinations(range(n_model), k):
                obj = self.model_points[list(mi)]
                models.append((mi, obj, _pairwise_distance_signature(obj),
                               _point_fingerprints(obj)))
            for ii in combinations(range(n_img), k):
                img = image_points[list(ii)]
                sig = _pairwise_distance_signature(img)
                fingerprint = _point_fingerprints(img)
                for mi, obj, model_sig, model_fp in models:
                    signature_error = _signature_error(sig, model_sig)
                    if signature_error > gate:
                        continue
                    costs = np.linalg.norm(model_fp[:, None, :] - fingerprint[None, :, :], axis=2)
                    for perm in permutations(range(k)):
                        assignment = tuple(ii[j] for j in perm)
                        error = float(sum(costs[i, j] for i, j in enumerate(perm)))
                        # Tie-break by geometry, never by incoming blob indices.
                        coordinates = tuple(image_points[list(assignment)].ravel())
                        hypotheses.append((-k, error + signature_error, coordinates, mi, assignment))
        hypotheses.sort()
        best = PoseEstimate.failed()
        best_cost = float("inf")
        solve_start = time.perf_counter()
        solve_count = 0
        for _, _, _, mi, ii in hypotheses:
            if solve_count >= self.max_search_hypotheses:
                break
            if solve_count and (time.perf_counter() - solve_start) * 1000 >= self.search_budget_ms:
                break
            if not _orientation_consistent(image_points[list(ii)], self.model_points[list(mi)]):
                continue
            solve_count += 1
            estimate = self._solve_and_score(
                image_points, camera_matrix, dist_coeffs, mi, ii,
                n_detected, use_extrinsic_guess=False)
            if not estimate.ok or estimate.n_used < minimum:
                continue
            cost = _ranking_cost(estimate, self)
            if cost < best_cost:
                best, best_cost = estimate, cost
            if best.n_used == n_model and best.confidence >= 0.7:
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
            # Extra candidate blobs are possible clutter, not missing model markers.
            unmatched_det = max(0, expected - len(set(ii_f)))

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
            # Reject each solution before ranking so an implausible branch
            # cannot conceal a valid alternative returned by the same solver.
            if not self._jump_plausible(est):
                continue
            cost = _ranking_cost(est, self)
            if cost < best_cost:
                best_cost = cost
                best_est = est
        return best_est


def _assign_mutual_nearest(
    projected: np.ndarray,
    image_points: np.ndarray,
    max_gate_px: float,
) -> tuple[tuple[int, ...], tuple[int, ...]]:
    """Maximum-cardinality, minimum-distance gated one-to-one assignment.

    Dynamic programming over model-point bitmasks: O(N * M * 2**M),
    only 16 masks for the four-marker model. Detections may remain unmatched.
    Canonical coordinate order makes equal-cost ties independent of brightness order.
    """
    proj = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    img = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    distances = np.linalg.norm(proj[:, None, :] - img[None, :, :], axis=2)
    states = {0: (0.0, ())}
    for j in sorted(range(len(img)), key=lambda j: (img[j, 0], img[j, 1])):
        updated = dict(states)  # skipping this detection is always allowed
        for mask, (cost, pairs) in states.items():
            for i in range(len(proj)):
                distance = float(distances[i, j])
                if mask & (1 << i) or not np.isfinite(distance) or distance > max_gate_px:
                    continue
                next_mask = mask | (1 << i)
                candidate = (cost + distance, pairs + ((i, j),))
                if next_mask not in updated or candidate[0] < updated[next_mask][0]:
                    updated[next_mask] = candidate
        states = updated
    _, pairs = min(states.values(), key=lambda entry: (-len(entry[1]), entry[0]))
    pairs = sorted(pairs)
    return tuple(i for i, _ in pairs), tuple(j for _, j in pairs)


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
    preserve_candidates: bool = False,
) -> list[tuple[int, int]]:
    """Remove isolated / geometrically implausible marker detections.

    Pipeline:
      1. Drop points with no neighbor within max_nearest_neighbor_px.
      2. Keep a compact cluster (diameter ≤ max_cluster_span_px).
      3. Keep the largest subset whose pairwise distance ratios match the 3D model.

    With preserve_candidates=True (live receiver), only spatial membership is
    checked here; constellation selection and geometry checks are deferred to PnP.
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

    if preserve_candidates:
        # Keep every point participating in a plausible local group. Do not
        # select a single constellation before temporal association/PnP.
        points = np.asarray(nearby, dtype=np.float64)
        distances = np.linalg.norm(points[:, None, :] - points[None, :, :], axis=2)
        keep = (distances <= max_cluster_span_px).sum(axis=1) >= min_cluster_size
        return [point for point, valid in zip(nearby, keep) if valid]

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
