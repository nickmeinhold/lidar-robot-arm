"""Motion gate — spine step 2 (crucible DESIGN §2.4 + amendments).

The preventive layer: given commanded poses/sequences in ``urdf_calib_frame``
radians, answer CLEAR / SOFT / HARD with machine-readable reasons. This
library has NO hardware access and NO opinion about execution — step 4 wires
it at the single mutator door (shadow mode first).

Soundness stance
----------------
- The sphere union provably contains each link's surface (generator +
  containment tests), so a real mesh self-collision implies a sphere
  contact: the geometric check cannot false-negative. Errors are in the
  conservative direction (false rejects), by construction.
- The hard margin is a SUM of named error-budget terms
  (``so101_gate_config.json``, provenance per term); shadow mode's job is to
  replace estimates with measurements.
- Sweep between samples is bounded by Σⱼ |Δθⱼ|·leverⱼ (amendment 10.1.3 —
  multi-joint composition), levers derived from the committed artifact at
  load (amendment 9.1.8), never from prose.
- The table half-space (amendment 9.1.1) is a static workspace model v0, not
  an environment model; its height carries a metrology note and must be
  re-measured whenever the base moves (amendment 10.1.8).

The gate certifies COMMANDED trajectories; physics can lag. The margin
budget carries the tracking term; step 4's 10 Hz feedback monitor is a
damage-limiter, never a safety invariant (amendment 9.1.2).
"""
from __future__ import annotations

import enum
import hashlib
import itertools
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .so101_kinematics import JOINT_ORDER, SO101Kinematics, default_kinematics

# Closed-interval joint limits are compared with this slack so that a pose
# sitting EXACTLY on a limit (a legal boundary value that may have ridden
# through float round-trips) is not rejected by representation noise. It is
# ~7 orders below encoder quantization (0.088 deg ≈ 1.5e-3 rad) — a float
# guard, never a policy widening.
LIMIT_EPSILON_RAD = 1e-9

# THE near-adjacent exclusion policy, owned by the GATE (single door — the
# sampler imports it from here): the only non-parent-child pair whose
# exclusion is mechanically justified. The gate RE-IMPOSES this at load, so
# an edited ACM that quietly moves a checked pair into excluded_pairs is
# refused — partition alone is necessary, not sufficient (cage-match r4).
NEAR_ADJACENT_ALLOWLIST = {("moving_jaw_so101_v1_link", "wrist_link")}

# Table-exemption policy, gate-owned for the same reason as the ACM
# allowlist (cage-match r5: config alone was an unguarded twin door — adding
# a link to exempt_links would silently darken the half-space for it).
# base_link: bolted to the table. shoulder_link: its only DOF is pan about
# the vertical axis, so its clearance is pose-invariant (tested, ~16 nm
# spread) — but NOTE it sits INSIDE the hard margin by construction (~+6 mm
# static), so this is a structural-geometry acceptance, not a reachability
# proof; re-review if step 3's measured table height moves the plane.
TABLE_EXEMPT_ALLOWLIST = {"base_link", "shoulder_link"}

MODEL_DIR = Path(__file__).parent / "static" / "models" / "SO101"
SPHERES_PATH = MODEL_DIR / "so101_spheres.json"
ACM_PATH = MODEL_DIR / "so101_acm.json"
GATE_CONFIG_PATH = Path(__file__).parent / "so101_gate_config.json"


class GateStatus(enum.Enum):
    CLEAR = "CLEAR"
    SOFT = "SOFT"
    HARD = "HARD"


class Reason(enum.Enum):
    """Closed taxonomy (DESIGN §2.4) — the contract for chat replies, the
    LLM repair loop, and the viewer UI alike."""
    SELF_COLLISION_PAIR = "SELF_COLLISION_PAIR"
    TABLE_PLANE = "TABLE_PLANE"
    JOINT_LIMIT = "JOINT_LIMIT"
    VELOCITY = "VELOCITY"
    WRAP_SEAM = "WRAP_SEAM"
    STALE_START = "STALE_START"
    PREEMPTED_BY_TELEOP = "PREEMPTED_BY_TELEOP"
    NOT_CALIBRATED = "NOT_CALIBRATED"
    EMPTY_SEQUENCE = "EMPTY_SEQUENCE"


@dataclass(frozen=True)
class GateReason:
    code: Reason
    detail: dict

    def __str__(self) -> str:
        return f"{self.code.value}{self.detail}"


@dataclass(frozen=True)
class PoseVerdict:
    status: GateStatus
    min_distance_mm: float          # the GOVERNING clearance (self or table)
    nearest_pair: tuple[str, str] | None  # nearest SELF-collision pair, always
    velocity_scale: float
    reasons: tuple[GateReason, ...]
    governing: str = "self_collision"     # "self_collision" | "table"


@dataclass(frozen=True)
class SequenceVerdict:
    admitted: bool
    reasons: tuple[GateReason, ...]
    violating_step: int | None = None
    n_samples: int = 0
    min_distance_mm: float = float("inf")


class MotionGate:
    """Construct once over the committed artifacts; ask it about poses and
    sequences. Thread-safe for reads (all state is immutable after init)."""

    def __init__(
        self,
        kin: SO101Kinematics | None = None,
        spheres_path: Path = SPHERES_PATH,
        acm_path: Path = ACM_PATH,
        config_path: Path = GATE_CONFIG_PATH,
    ) -> None:
        self.kin = kin or default_kinematics()
        spheres_bytes = Path(spheres_path).read_bytes()
        spheres = json.loads(spheres_bytes)
        acm = json.loads(Path(acm_path).read_text())
        cfg = json.loads(Path(config_path).read_text())

        # artifact chain integrity (amendment 9.1.8): the gate refuses to run
        # over a bake that is not the one the spheres were derived from.
        bake_sha = hashlib.sha256(
            Path(self.kin.bake_path).read_bytes()).hexdigest()
        if spheres["provenance"]["bake_sha256"] != bake_sha:
            raise RuntimeError(
                "so101_spheres.json was generated from a DIFFERENT bake — "
                "regenerate the sphere artifact (python -m "
                "server.scripts.generate_spheres) and re-review it")
        if acm["provenance"]["bake_sha256"] != bake_sha:
            raise RuntimeError(
                "so101_acm.json was measured against a DIFFERENT bake — "
                "regenerate (python -m server.scripts.sample_acm)")
        if acm["provenance"]["spheres_sha256"] != hashlib.sha256(
                spheres_bytes).hexdigest():
            raise RuntimeError(
                "so101_acm.json was measured against a DIFFERENT sphere set — "
                "regenerate the ACM after any sphere regeneration (the chain "
                "bake → spheres → ACM must be unbroken)")

        self.links: list[str] = list(spheres["links"])
        self._centers = {n: np.array([s["center_m"] for s in e["spheres"]])
                         for n, e in spheres["links"].items()}
        self._radii = {n: np.array([s["radius_m"] for s in e["spheres"]])
                       for n, e in spheres["links"].items()}
        self.levers_mm: dict[str, float] = dict(spheres["levers_mm"])

        excluded = {tuple(sorted(e["pair"])) for e in acm["excluded_pairs"]}
        self.checked_pairs: list[tuple[str, str]] = [
            tuple(sorted(e["pair"])) for e in acm["checked_pairs"]]
        # The ACM must PARTITION the pair set: disjoint AND complete. A pair
        # absent from both lists would silently never be checked — a false
        # negative by omission. RuntimeError, not assert: this must survive
        # python -O.
        all_pairs = {tuple(sorted(p))
                     for p in itertools.combinations(self.links, 2)}
        listed = excluded | set(self.checked_pairs)
        if excluded & set(self.checked_pairs) or listed != all_pairs:
            raise RuntimeError(
                "so101_acm.json does not partition the link-pair set "
                f"(missing: {sorted(all_pairs - listed)}, "
                f"double-listed: {sorted(excluded & set(self.checked_pairs))})")
        # POLICY re-imposed at load (not just at ACM build): an excluded pair
        # must be structurally adjacent per THIS bake, or on the allowlist.
        bake = json.loads(Path(self.kin.bake_path).read_text())
        adjacent = set()
        for j in bake["joints"]:
            a, b = j["parent"], j["child"]
            if a in self.links and b in self.links:
                adjacent.add(tuple(sorted((a, b))))
        for p in excluded:
            if p not in adjacent and p not in NEAR_ADJACENT_ALLOWLIST:
                raise RuntimeError(
                    f"excluded pair {p} is neither parent-child in the bake "
                    "nor on the near-adjacent allowlist — refusing an ACM "
                    "that un-checks a non-structural pair")

        # flatten sphere set for vectorized checks: world spheres are built
        # per pose; pair index arrays are precomputed once.
        self._pair_idx: list[tuple[str, str]] = []
        ia, ib = [], []
        offsets: dict[str, int] = {}
        off = 0
        for n in self.links:
            offsets[n] = off
            off += len(self._radii[n])
        self._n_spheres = off
        self._offsets = offsets
        rr = []
        for a, b in self.checked_pairs:
            for i in range(len(self._radii[a])):
                for j in range(len(self._radii[b])):
                    ia.append(offsets[a] + i)
                    ib.append(offsets[b] + j)
                    rr.append(self._radii[a][i] + self._radii[b][j])
                    self._pair_idx.append((a, b))
        self._ia = np.array(ia)
        self._ib = np.array(ib)
        self._rsum = np.array(rr)
        if not self._ia.size:
            raise RuntimeError(
                "ACM leaves ZERO sphere-pair terms to check — refusing to "
                "construct a gate that can never see a collision")
        for n in self.links:
            if not (np.isfinite(self._centers[n]).all()
                    and np.isfinite(self._radii[n]).all()):
                raise RuntimeError(f"non-finite sphere data for {n}")
        if not all(np.isfinite(v) and v > 0 for v in self.levers_mm.values()):
            raise RuntimeError("non-finite/non-positive lever in artifact")

        budget = cfg["margin_budget_mm"]
        required_terms = {"encoder_quantization", "backlash",
                          "tracking_overshoot", "calibration_homing",
                          "sphere_fit_residual", "sampling"}
        if not required_terms <= set(budget):
            raise RuntimeError(
                f"gate config missing margin terms: {required_terms - set(budget)}")
        if any(float(t["value"]) < 0 for t in budget.values()):
            raise RuntimeError("negative margin-budget term — refusing")
        if float(cfg["soft_band_extra_mm"]) <= 0 \
                or float(budget["sampling"]["value"]) <= 0:
            raise RuntimeError("non-positive soft band / sampling term")
        # amendment 9.1.8 as a DOOR, not just a test (cage-match r6): the
        # lever-coupled terms must cover their artifact-derived minimums —
        # a regen that lifts the lever, or a config edit that shrinks a
        # term, is refused here rather than discovered in review
        lever_pan = float(spheres["levers_mm"]["shoulder_pan"])
        enc_min = math.radians(360.0 / 4096.0) * lever_pan
        backlash_min = math.radians(0.5) * lever_pan
        if float(budget["encoder_quantization"]["value"]) < enc_min - 1e-9:
            raise RuntimeError(
                f"encoder_quantization term below artifact-derived minimum "
                f"{enc_min:.3f} mm (lever {lever_pan} mm/rad)")
        if float(budget["backlash"]["value"]) < backlash_min - 1e-9:
            raise RuntimeError(
                f"backlash term below artifact-derived minimum "
                f"{backlash_min:.3f} mm (±0.5° × lever {lever_pan} mm/rad)")
        self.hard_margin_mm: float = float(
            sum(t["value"] for t in budget.values()))
        self.soft_band_mm: float = self.hard_margin_mm + float(
            cfg["soft_band_extra_mm"])
        self.sampling_allowance_mm: float = 2.0 * float(
            budget["sampling"]["value"])
        self._soft_exp_k: float = float(cfg["soft_velocity_exponent"])

        table = cfg["table"]
        self.table_enabled: bool = bool(table["enabled"])
        # height null → derive v0 from the artifact (base mesh bottom = the
        # surface the base stands on); step 3's touch-test overrides in config
        self.table_z_m: float = (
            float(table["height_m"]) if table["height_m"] is not None
            else float(spheres["base_min_z_m"]))
        self.table_exempt: set[str] = set(table["exempt_links"])
        if not self.table_exempt <= TABLE_EXEMPT_ALLOWLIST:
            raise RuntimeError(
                f"table exempt_links {self.table_exempt - TABLE_EXEMPT_ALLOWLIST} "
                "not on the gate's exemption allowlist — a config edit must "
                "not silently darken the half-space for a link")

        self._lever_mm_vec = np.array(
            [self.levers_mm[n] for n in JOINT_ORDER])
        self._lo = np.array([self.kin.joints[n].lower for n in JOINT_ORDER])
        self._hi = np.array([self.kin.joints[n].upper for n in JOINT_ORDER])

    # --- geometry ---------------------------------------------------------

    def _world_spheres(self, q: np.ndarray) -> np.ndarray:
        fr = self.kin.frames(q)
        out = np.empty((self._n_spheres, 3))
        for n in self.links:
            m = fr[n]
            o = self._offsets[n]
            c = self._centers[n]
            out[o:o + len(c)] = (m[:3, :3] @ c.T).T + m[:3, 3]
        return out

    def _min_gaps(self, world: np.ndarray) -> np.ndarray:
        """(n_pairterms,) signed clearance per sphere-pair term (m)."""
        d = np.linalg.norm(world[self._ia] - world[self._ib], axis=1)
        return d - self._rsum

    # --- sweep bound (amendment 10.1.3) -----------------------------------

    def sweep_upper_bound_mm(self, dq: np.ndarray) -> float:
        """Upper bound on the RELATIVE Cartesian sweep for joint delta dq
        (radians): Σⱼ |Δθⱼ| · leverⱼ (multi-joint motion composes —
        amendment 10.1.3).

        Why this bounds the PAIR-GAP change, not just one body's motion
        (the cage-match's two-body attack, refuted): for links A and B on a
        serial chain, rotation at a COMMON-ANCESTOR joint moves both
        rigidly together — their gap is unchanged. Only joints on the chain
        path BETWEEN A and B alter their relative pose, and each such
        joint's contribution is ≤ |Δθⱼ| × (distance from its axis to the
        farther body's spheres) ≤ leverⱼ. Summing over ALL joints
        (a superset of the between-set) therefore bounds the gap change.
        The table plane is static, so a single body's sweep bounds that
        gap trivially.

        Why the margin banks HALF the allowance (the monotone-dive attack,
        refuted): both segment endpoints are checked ≥ hard margin. Along
        the segment with total sweep S, |g(t)−g(0)| ≤ tS and
        |g(t)−g(1)| ≤ (1−t)S, so g(t) ≥ (g0+g1−S)/2 ≥ min(g0,g1) − S/2 —
        the interior can dip at most S/2 below the checked endpoints. A
        "monotone approach spending the whole sweep" would leave the NEXT
        sample below the margin, where it is rejected. Hence sampling
        allowance = 2 × the budget's sampling term."""
        return float(np.abs(dq) @ self._lever_mm_vec)

    def samples_for_delta(self, dq: np.ndarray) -> int:
        """Samples needed so inter-sample sweep ≤ the sampling allowance
        (2 × the budget's sampling term — the term is the half-sweep)."""
        sweep = self.sweep_upper_bound_mm(dq)
        return max(2, int(math.ceil(sweep / self.sampling_allowance_mm)) + 1)

    # --- verdicts ---------------------------------------------------------

    def check_pose(self, q: np.ndarray) -> PoseVerdict:
        q = np.asarray(q, dtype=float)
        if q.shape != (len(JOINT_ORDER),):
            raise ValueError(
                f"pose must be shape ({len(JOINT_ORDER)},), got {q.shape}")
        if not np.isfinite(q).all():
            raise ValueError(f"pose contains non-finite values: {q}")
        reasons: list[GateReason] = []

        for i, name in enumerate(JOINT_ORDER):
            if not (self._lo[i] - LIMIT_EPSILON_RAD <= q[i]
                    <= self._hi[i] + LIMIT_EPSILON_RAD):
                reasons.append(GateReason(Reason.JOINT_LIMIT, {
                    "joint": name, "value_rad": float(q[i]),
                    "limits_rad": [float(self._lo[i]), float(self._hi[i])]}))

        # FK is total (pure trig over all reals), so geometry is evaluated
        # even for out-of-limit poses — the verdict then carries BOTH the
        # JOINT_LIMIT reason and the geometric picture at the illegal pose,
        # which the repair loop wants; there is no numeric hazard to dodge.
        world = self._world_spheres(q)
        gaps = self._min_gaps(world)
        k = int(gaps.argmin())
        min_mm = float(gaps[k] * 1000.0)
        nearest = self._pair_idx[k]

        if min_mm < self.hard_margin_mm:
            reasons.append(GateReason(Reason.SELF_COLLISION_PAIR, {
                "pair": list(nearest), "distance_mm": round(min_mm, 1),
                "hard_margin_mm": self.hard_margin_mm}))

        table_mm = float("inf")
        if self.table_enabled:
            for n in self.links:
                if n in self.table_exempt:
                    continue
                o = self._offsets[n]
                zs = world[o:o + len(self._radii[n]), 2] - self._radii[n]
                clear = float((zs - self.table_z_m).min() * 1000.0)
                if clear < table_mm:
                    table_mm = clear
                if clear < self.hard_margin_mm:
                    reasons.append(GateReason(Reason.TABLE_PLANE, {
                        "link": n, "clearance_mm": round(clear, 1)}))

        governing_mm = min(min_mm, table_mm)
        if reasons and any(r.code in (Reason.JOINT_LIMIT,
                                      Reason.SELF_COLLISION_PAIR,
                                      Reason.TABLE_PLANE) for r in reasons):
            status = GateStatus.HARD
            scale = 0.0
        elif governing_mm < self.soft_band_mm:
            status = GateStatus.SOFT
            # MoveIt Servo's law: exponential scale inside the soft band
            depth = (self.soft_band_mm - governing_mm) / (
                self.soft_band_mm - self.hard_margin_mm)
            scale = float(math.exp(-self._soft_exp_k * depth))
            if table_mm < min_mm:
                reasons.append(GateReason(Reason.TABLE_PLANE, {
                    "clearance_mm": round(table_mm, 1), "band": "soft"}))
            else:
                reasons.append(GateReason(Reason.SELF_COLLISION_PAIR, {
                    "pair": list(nearest),
                    "distance_mm": round(governing_mm, 1), "band": "soft"}))
        else:
            status = GateStatus.CLEAR
            scale = 1.0

        # the scalar a reader reconciles with the status must be the
        # GOVERNING clearance — a table-grazing pose must not report a fat
        # self-gap (cage-match r4)
        return PoseVerdict(status=status,
                           min_distance_mm=round(governing_mm, 2),
                           nearest_pair=nearest, velocity_scale=scale,
                           reasons=tuple(reasons),
                           governing=("table" if table_mm < min_mm
                                      else "self_collision"))

    def check_sequence(self, keyframes: list[np.ndarray]) -> SequenceVerdict:
        """All-or-nothing admission over the fully-interpolated sequence at
        the derived step (DESIGN §2.4). Rejection carries the offending step
        index + reasons. v1 semantics: static environment; certificates
        expire on interference (amendment 9.1.6 — enforcement is step 4's)."""
        if not keyframes:
            return SequenceVerdict(admitted=False, reasons=(
                GateReason(Reason.EMPTY_SEQUENCE, {}),))

        frames = [np.asarray(k, dtype=float) for k in keyframes]
        reasons: list[GateReason] = []
        wrist_i = JOINT_ORDER.index("wrist_roll")
        min_mm = float("inf")
        n_total = 0

        v0 = self.check_pose(frames[0])
        if v0.status == GateStatus.HARD:
            return SequenceVerdict(admitted=False, reasons=v0.reasons,
                                   violating_step=0,
                                   min_distance_mm=v0.min_distance_mm)
        if len(frames) == 1:
            # a one-frame certificate still reports the real pose verdict
            # (distance + any SOFT reasons), not inf/empty
            return SequenceVerdict(admitted=True, reasons=v0.reasons,
                                   n_samples=1,
                                   min_distance_mm=v0.min_distance_mm)

        worst_soft: tuple[GateReason, ...] = ()
        worst_step = -1
        for step in range(len(frames) - 1):
            a, b = frames[step], frames[step + 1]
            dq = b - a
            # wrist_roll rides a LIMITED interval; a >180° linear excursion
            # is almost always an author who wanted the forbidden short arc
            # (amendment 10.1.4). FAIL CLOSED on a safety surface (cage-match
            # consensus): reject with WRAP_SEAM so the IR repair loop rewrites
            # the keyframes — never silently drive the long way around.
            if abs(dq[wrist_i]) > math.pi:
                return SequenceVerdict(
                    admitted=False,
                    reasons=tuple(list(reasons) + [GateReason(
                        Reason.WRAP_SEAM, {
                            "joint": "wrist_roll", "step": step,
                            "delta_deg": round(
                                math.degrees(dq[wrist_i]), 1)})]),
                    violating_step=step, n_samples=n_total,
                    min_distance_mm=min_mm)
            n = self.samples_for_delta(dq)
            qs = self.kin.interpolate(a, b, n)
            # the admission proof requires ENDPOINT-INCLUSIVE samples with
            # n−1 intervals; verify the helper's contract rather than
            # assuming it (segment boundaries are deliberately re-checked —
            # cheap, and each keyframe is provably examined)
            # the half-sweep proof needs endpoint-inclusive samples AND
            # UNIFORM intervals of (b−a)/(n−1) — a monotone but non-uniform
            # partition could spend the whole sweep in one gap (cage-match
            # round 3); verify the winding, not just the terminals
            diffs = np.diff(qs, axis=0)
            if not (len(qs) == n and np.allclose(qs[0], a)
                    and np.allclose(qs[-1], b)
                    and np.allclose(diffs, dq / (n - 1))):
                raise RuntimeError(
                    "interpolate() broke its uniform endpoint-inclusive "
                    "contract — the sampling proof is void")
            for i_q, q in enumerate(qs):
                if step > 0 and i_q == 0:
                    continue  # shared endpoint: checked as previous segment's end
                n_total += 1  # counts EXECUTED checks — honest on rejection too
                v = self.check_pose(q)
                if v.min_distance_mm < min_mm:
                    min_mm = v.min_distance_mm
                    if v.status == GateStatus.SOFT:
                        worst_soft = v.reasons
                        worst_step = step
                if v.status == GateStatus.HARD:
                    tail = [GateReason(r.code,
                                       {**r.detail, "worst_step": worst_step})
                            for r in worst_soft]
                    return SequenceVerdict(
                        admitted=False,
                        reasons=tuple(list(v.reasons) + reasons + tail),
                        violating_step=step,
                        n_samples=n_total, min_distance_mm=min_mm)

        if worst_soft:
            # telemetry fidelity (cage-match round 3): an admitted-but-SOFT
            # path reports its worst sample's reasons + step for shadow mode
            reasons.extend(GateReason(r.code, {**r.detail,
                                               "worst_step": worst_step})
                           for r in worst_soft)
        return SequenceVerdict(admitted=True, reasons=tuple(reasons),
                               n_samples=n_total, min_distance_mm=min_mm)
