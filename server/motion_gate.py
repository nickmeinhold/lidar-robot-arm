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
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .so101_kinematics import JOINT_ORDER, SO101Kinematics, default_kinematics

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
    min_distance_mm: float
    nearest_pair: tuple[str, str] | None
    velocity_scale: float
    reasons: tuple[GateReason, ...]


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
        spheres = json.loads(Path(spheres_path).read_text())
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

        self.links: list[str] = list(spheres["links"])
        self._centers = {n: np.array([s["center_m"] for s in e["spheres"]])
                         for n, e in spheres["links"].items()}
        self._radii = {n: np.array([s["radius_m"] for s in e["spheres"]])
                       for n, e in spheres["links"].items()}
        self.levers_mm: dict[str, float] = dict(spheres["levers_mm"])

        excluded = {tuple(sorted(e["pair"])) for e in acm["excluded_pairs"]}
        self.checked_pairs: list[tuple[str, str]] = [
            tuple(sorted(e["pair"])) for e in acm["checked_pairs"]]
        assert not excluded & set(self.checked_pairs)

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

        budget = cfg["margin_budget_mm"]
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
        """Upper bound on Cartesian sweep for joint delta dq (radians):
        Σⱼ |Δθⱼ| · leverⱼ — multi-joint motion COMPOSES."""
        return float(np.abs(dq) @ self._lever_mm_vec)

    def samples_for_delta(self, dq: np.ndarray) -> int:
        """Samples needed so inter-sample sweep ≤ the sampling allowance
        (2 × the budget's sampling term — the term is the half-sweep)."""
        sweep = self.sweep_upper_bound_mm(dq)
        return max(2, int(math.ceil(sweep / self.sampling_allowance_mm)) + 1)

    # --- verdicts ---------------------------------------------------------

    def check_pose(self, q: np.ndarray) -> PoseVerdict:
        q = np.asarray(q, dtype=float)
        reasons: list[GateReason] = []

        for i, name in enumerate(JOINT_ORDER):
            if not (self._lo[i] - 1e-9 <= q[i] <= self._hi[i] + 1e-9):
                reasons.append(GateReason(Reason.JOINT_LIMIT, {
                    "joint": name, "value_rad": float(q[i]),
                    "limits_rad": [float(self._lo[i]), float(self._hi[i])]}))

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
            reasons.append(GateReason(Reason.SELF_COLLISION_PAIR, {
                "pair": list(nearest), "distance_mm": round(governing_mm, 1),
                "band": "soft"}))
        else:
            status = GateStatus.CLEAR
            scale = 1.0

        return PoseVerdict(status=status, min_distance_mm=round(min_mm, 2),
                           nearest_pair=nearest, velocity_scale=scale,
                           reasons=tuple(reasons))

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

        for step in range(len(frames) - 1):
            a, b = frames[step], frames[step + 1]
            dq = b - a
            # wrist_roll rides a LIMITED interval; a >180° linear excursion
            # is almost always an author who wanted the forbidden short arc
            # (amendment 10.1.4) — flag for the IR repair loop.
            if abs(dq[wrist_i]) > math.pi:
                reasons.append(GateReason(Reason.WRAP_SEAM, {
                    "joint": "wrist_roll", "step": step,
                    "delta_deg": round(math.degrees(dq[wrist_i]), 1)}))
            n = self.samples_for_delta(dq)
            n_total += n
            qs = self.kin.interpolate(a, b, n)
            for q in qs:
                v = self.check_pose(q)
                if v.min_distance_mm < min_mm:
                    min_mm = v.min_distance_mm
                if v.status == GateStatus.HARD:
                    return SequenceVerdict(
                        admitted=False,
                        reasons=tuple(list(v.reasons) + reasons),
                        violating_step=step,
                        n_samples=n_total, min_distance_mm=min_mm)

        return SequenceVerdict(admitted=True, reasons=tuple(reasons),
                               n_samples=n_total, min_distance_mm=min_mm)
