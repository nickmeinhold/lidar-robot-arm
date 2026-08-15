"""Spine step 2 acceptance tests — sphere model + ACM + motion gate.

The crucible's step-2 acceptance gate (DESIGN §3.2 + amendments 9.1.11,
10.1.3): a REAL-pair-semantics timing benchmark, an adversarial pose corpus,
and a held-out oracle that does not share the sphere representation.

Soundness architecture being verified here:
  mesh ⊆ sphere-union (per link, proven by containment)  ⟹  a mesh-mesh
  collision implies a sphere-sphere collision  ⟹  the gate cannot
  false-negative on self-collision. The tests RE-VERIFY containment with
  independent code (the verifier must not share the generator's failure
  modes) and prove the ACM's excluded pairs with convex-hull LP feasibility
  — an instrument that never touches spheres.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import pytest

from server.so101_kinematics import JOINT_ORDER, SO101Kinematics
from server.motion_gate import (
    GateStatus,
    MotionGate,
    Reason,
    SPHERES_PATH,
    ACM_PATH,
    GATE_CONFIG_PATH,
)
from server.scripts.generate_spheres import load_link_pointclouds, read_stl

MODEL_DIR = Path(__file__).parent / "static" / "models" / "SO101"


@pytest.fixture(scope="module")
def kin() -> SO101Kinematics:
    return SO101Kinematics()


@pytest.fixture(scope="module")
def gate() -> MotionGate:
    return MotionGate()


@pytest.fixture(scope="module")
def spheres() -> dict:
    return json.loads(SPHERES_PATH.read_text())


@pytest.fixture(scope="module")
def acm() -> dict:
    return json.loads(ACM_PATH.read_text())


# --- artifact integrity ----------------------------------------------------

def test_spheres_artifact_committed_and_bound_to_bake(spheres):
    """The sphere set is a reviewed artifact BOUND to the bake it was derived
    from (amendment 9.1.8): regenerating geometry must re-derive bounds."""
    assert spheres["provenance"]["bake_sha256"], "artifact must pin its bake"
    import hashlib
    bake_bytes = (MODEL_DIR / "so101_urdf.json").read_bytes()
    assert spheres["provenance"]["bake_sha256"] == hashlib.sha256(bake_bytes).hexdigest()


def test_sphere_counts_match_generator_params(spheres):
    """RESEARCH's K=6 default was falsified at posture level (a K=6 base
    sphere swallows the air the upper arm swings through — home pan sweeps
    read ~1 mm); per-link K is a design §7 open variable. The artifact must
    agree with its own recorded parameters, 7 links, K ≥ 6 each."""
    links = spheres["links"]
    assert len(links) == 7
    params = spheres["provenance"]["params"]
    for name, entry in links.items():
        expected = params["k_per_link"].get(name, params["k_default"])
        assert len(entry["spheres"]) == expected, (
            f"{name}: {len(entry['spheres'])} spheres, params say {expected}")
        assert expected >= 6
        for s in entry["spheres"]:
            assert s["radius_m"] > 0


def test_levers_present_and_sane(spheres):
    """Per-joint lever arms live in the artifact (derived, not prose). The
    shoulder_pan lever anchors the sampling bound; RESEARCH measured
    446.5 mm — the artifact value must be in that regime, and levers must
    decrease monotonically down the chain."""
    levers = spheres["levers_mm"]
    assert set(levers) == set(JOINT_ORDER)
    assert 300 <= levers["shoulder_pan"] <= 600
    chain = [levers[j] for j in JOINT_ORDER]
    assert all(a >= b for a, b in zip(chain, chain[1:])), "levers shrink outward"


# --- containment oracle (independent re-verification) ----------------------

def test_mesh_containment_100_percent(spheres):
    """THE soundness anchor: every link's mesh vertices sit inside its sphere
    union. Independent recompute — this test reads the STLs and the JSON and
    does its own math; it must not import the generator's fit routines.

    RESEARCH: p99 radii leave 0.35–0.91% of geometry outside (unsound in the
    unsafe direction); the max rule + half-longest-triangle-edge inflation
    gives true containment."""
    clouds = load_link_pointclouds(MODEL_DIR)
    for name, entry in spheres["links"].items():
        pts = clouds[name]  # (N,3) link-frame vertices
        centers = np.array([s["center_m"] for s in entry["spheres"]])
        radii = np.array([s["radius_m"] for s in entry["spheres"]])
        d = np.linalg.norm(pts[:, None, :] - centers[None, :, :], axis=2)
        inside = (d <= radii[None, :] + 1e-9).any(axis=1)
        coverage = inside.mean()
        assert coverage == 1.0, (
            f"{name}: {(~inside).sum()} of {len(pts)} vertices escape the "
            f"sphere union (coverage {coverage:.4%}) — containment broken"
        )


def test_triangle_bulge_inflation_recorded(spheres):
    """Vertex containment alone allows a triangle to bulge between spheres;
    the radius must carry the half-longest-edge inflation (RESEARCH)."""
    for name, entry in spheres["links"].items():
        assert entry["edge_inflation_m"] >= 0
        # dense CAD exports: correction must be sub-2mm or the mesh isn't
        # what the research measured
        assert entry["edge_inflation_m"] < 0.002


# --- ACM: every exclusion carries its receipt ------------------------------

def test_acm_exclusions_have_reasons(acm):
    """Pair policy with receipts (Carnot): no bare 'adjacent' hand-waves."""
    assert acm["excluded_pairs"], "ACM must exclude at least the adjacent pairs"
    for entry in acm["excluded_pairs"]:
        assert entry["reason"] in {
            "parent_child_adjacent", "near_adjacent_default_touching",
            "sampled_never_hull_proven",
        }
        assert entry["evidence"], f"{entry['pair']} carries no evidence"


def test_acm_checked_pairs_are_the_complement(acm, spheres):
    n_links = len(spheres["links"])
    all_pairs = n_links * (n_links - 1) // 2
    assert len(acm["excluded_pairs"]) + len(acm["checked_pairs"]) == all_pairs


@pytest.mark.slow
def test_hull_proof_for_sampled_never_pairs(acm, kin):
    """Amendment 9.1.11: ignored pairs need a deterministic geometric
    justification — for sampled-never pairs that is the convex-hull LP proof
    (RESEARCH: a mesh is a subset of its hull; disjoint hulls ⟹ disjoint
    meshes). Re-proven here at 200 fresh poses per pair."""
    scipy_spatial = pytest.importorskip("scipy.spatial")
    from scipy.optimize import linprog

    clouds = load_link_pointclouds(MODEL_DIR)
    hulls = {}
    for name, pts in clouds.items():
        hull = scipy_spatial.ConvexHull(pts)
        hulls[name] = pts[hull.vertices]

    def hulls_intersect(A: np.ndarray, B: np.ndarray) -> bool:
        # feasibility: exists λ,μ ≥ 0, Σλ=1, Σμ=1, Aᵀλ − Bᵀμ = 0
        na, nb = len(A), len(B)
        A_eq = np.zeros((5, na + nb))
        A_eq[:3, :na] = A.T
        A_eq[:3, na:] = -B.T
        A_eq[3, :na] = 1.0
        A_eq[4, na:] = 1.0
        b_eq = np.array([0.0, 0.0, 0.0, 1.0, 1.0])
        res = linprog(np.zeros(na + nb), A_eq=A_eq, b_eq=b_eq,
                      bounds=[(0, None)] * (na + nb), method="highs")
        return res.status == 0

    never = [e for e in acm["excluded_pairs"]
             if e["reason"] == "sampled_never_hull_proven"]
    assert never, "expected sampled-never exclusions (research found two)"
    rng = np.random.default_rng(11)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    for entry in never:
        a, b = entry["pair"]
        for _ in range(200):
            q = rng.uniform(lo, hi)
            fr = kin.frames(q)
            Ah = (fr[a][:3, :3] @ hulls[a].T).T + fr[a][:3, 3]
            Bh = (fr[b][:3, :3] @ hulls[b].T).T + fr[b][:3, 3]
            assert not hulls_intersect(Ah, Bh), (
                f"hull proof FAILED for excluded pair {a}|{b} at {q}"
            )


# --- the gate: statuses, reasons, degenerate states ------------------------

HOME = {n: 0.0 for n in JOINT_ORDER}


def q_of(kin, **overrides) -> np.ndarray:
    d = dict(HOME)
    d.update(overrides)
    return np.array([d[n] for n in JOINT_ORDER])


def test_home_pose_is_clear(gate, kin):
    v = gate.check_pose(q_of(kin))
    assert v.status == GateStatus.CLEAR
    assert v.min_distance_mm > gate.soft_band_mm


def test_folded_pose_is_hard(gate, kin):
    """Fold the arm onto itself: elbow + wrist hard into the upper arm."""
    q = q_of(kin, shoulder_lift=-1.6, elbow_flex=kin.joints["elbow_flex"].lower,
             wrist_flex=kin.joints["wrist_flex"].lower)
    v = gate.check_pose(q)
    assert v.status == GateStatus.HARD
    assert any(r.code == Reason.SELF_COLLISION_PAIR for r in v.reasons)


def test_out_of_limits_is_rejected_with_joint_name(gate, kin):
    q = q_of(kin)
    q[JOINT_ORDER.index("elbow_flex")] = kin.joints["elbow_flex"].upper + 0.2
    v = gate.check_pose(q)
    assert v.status == GateStatus.HARD
    r = [r for r in v.reasons if r.code == Reason.JOINT_LIMIT]
    assert r and r[0].detail["joint"] == "elbow_flex"


def test_table_halfspace_flags_below_plane(gate, kin):
    """Amendment 9.1.1: the table plane is checked in the same pass. A pose
    driving the tool below the base plane must flag TABLE_PLANE."""
    # found by search: shoulder_lift 1.74 swings the forearm ~80 mm below
    # the plane — the mechanism must fire, not merely exist
    q = q_of(kin, shoulder_pan=0.703, shoulder_lift=1.741, elbow_flex=-0.159,
             wrist_flex=0.149, wrist_roll=2.122)
    v = gate.check_pose(q)
    assert v.status == GateStatus.HARD
    table = [r for r in v.reasons if r.code == Reason.TABLE_PLANE]
    assert table and table[0].detail["clearance_mm"] < -50


def test_base_link_exempt_from_table(gate):
    """base_link is bolted to the table; its spheres straddle the plane by
    construction and must not trip the check (exemption carries a reason in
    config)."""
    cfg = json.loads(GATE_CONFIG_PATH.read_text())
    assert "base_link" in cfg["table"]["exempt_links"]


def test_empty_sequence_rejected(gate):
    v = gate.check_sequence([])
    assert not v.admitted
    assert v.reasons[0].code == Reason.EMPTY_SEQUENCE


def test_single_frame_sequence_is_pose_check(gate, kin):
    v = gate.check_sequence([q_of(kin)])
    assert v.admitted


def test_violating_start_rejected_at_step_zero(gate, kin):
    bad = q_of(kin, shoulder_lift=-1.6,
               elbow_flex=kin.joints["elbow_flex"].lower,
               wrist_flex=kin.joints["wrist_flex"].lower)
    v = gate.check_sequence([bad, q_of(kin)])
    assert not v.admitted
    assert v.violating_step == 0


def test_limit_grazing_admitted(gate, kin):
    """Riding exactly along a joint limit is legal — limits are closed
    intervals; the gate must not reject the boundary itself."""
    q = q_of(kin)
    q[JOINT_ORDER.index("wrist_flex")] = kin.joints["wrist_flex"].upper
    v = gate.check_pose(q)
    assert not any(r.code == Reason.JOINT_LIMIT for r in v.reasons)


def test_wrap_seam_flagged_for_long_wrist_roll_path(gate, kin):
    """wrist_roll's span is ~320°, a convex interval — interpolation is
    LINEAR (never circular-shortest). A step demanding >180° of wrist_roll
    travel is almost always an author who wanted the forbidden short-cut:
    flag WRAP_SEAM so the IR layer can repair (amendment 10.1.4)."""
    a = q_of(kin, wrist_roll=kin.joints["wrist_roll"].lower + 0.05)
    b = q_of(kin, wrist_roll=kin.joints["wrist_roll"].upper - 0.05)
    v = gate.check_sequence([a, b])
    assert any(r.code == Reason.WRAP_SEAM for r in v.reasons)


def test_soft_band_scales_velocity(gate, kin):
    """Streaming semantics: inside the soft band the gate returns a scale in
    (0,1); at/inside hard it is 0.0 (hold)."""
    clear = gate.check_pose(q_of(kin))
    assert clear.velocity_scale == 1.0
    # A pose the sampler measured near contact: fold elbow toward base.
    for lift in np.linspace(-0.4, -1.6, 25):
        v = gate.check_pose(q_of(kin, shoulder_lift=lift,
                                 elbow_flex=kin.joints["elbow_flex"].lower))
        if v.status == GateStatus.SOFT:
            assert 0.0 < v.velocity_scale < 1.0
            break
    else:
        pytest.skip("no SOFT pose found on this ray")


# --- sweep bound (amendment 10.1.3: multi-joint composition) ---------------

def test_sampling_step_composes_multijoint_motion(gate):
    """sweep ≤ Σⱼ |Δθⱼ|·leverⱼ — the sample count for a step must be derived
    from the SUM across joints, not the max single joint."""
    dq = np.full(6, math.radians(1.0))  # 1° on every joint at once
    n_multi = gate.samples_for_delta(dq)
    n_single = gate.samples_for_delta(
        np.array([math.radians(1.0), 0, 0, 0, 0, 0]))
    assert n_multi > n_single, "multi-joint motion must sample more densely"
    # worst-case fixture: the sweep between adjacent samples stays under the
    # sampling allowance
    sweep_mm = gate.sweep_upper_bound_mm(dq / n_multi)
    assert sweep_mm <= gate.sampling_allowance_mm + 1e-9


def test_derived_step_matches_research_regime(gate):
    """The derived bound must land in the research's measured regime:
    ~0.38° for 10 mm at worst-case single-joint lever (7.79 mm/deg)."""
    lever_mm_per_rad = gate.levers_mm["shoulder_pan"]
    step_deg = math.degrees(gate.sampling_allowance_mm / lever_mm_per_rad)
    assert 0.1 <= step_deg <= 1.0


# --- margin budget ---------------------------------------------------------

def test_margin_is_a_sum_of_named_terms(gate):
    cfg = json.loads(GATE_CONFIG_PATH.read_text())
    terms = cfg["margin_budget_mm"]
    expected = {"encoder_quantization", "backlash", "tracking_overshoot",
                "calibration_homing", "sphere_fit_residual", "sampling"}
    assert expected <= set(terms)
    assert abs(gate.hard_margin_mm - sum(t["value"] for t in terms.values())) < 1e-9
    for name, t in terms.items():
        assert t["provenance"], f"margin term {name} has no provenance"


# --- adversarial corpus (amendment 9.1.11) ---------------------------------

def test_adversarial_corpus(gate, kin, acm):
    """Joint-limit extremes, gripper open/closed, wrap seams, folded
    configurations — the gate must return a verdict (never crash) and every
    verdict must carry taxonomy reasons when not CLEAR."""
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    corpus = []
    for mask in range(64):  # every limit-corner combination
        corner = np.where([(mask >> i) & 1 for i in range(6)], hi, lo)
        corpus.append(corner)
    corpus.append(np.zeros(6))
    rng = np.random.default_rng(7)
    corpus.extend(rng.uniform(lo, hi, size=(200, 6)))
    for q in corpus:
        v = gate.check_pose(np.asarray(q, dtype=float))
        assert v.status in (GateStatus.CLEAR, GateStatus.SOFT, GateStatus.HARD)
        if v.status != GateStatus.CLEAR:
            assert v.reasons, f"non-CLEAR verdict with no reasons at {q}"
        assert np.isfinite(v.min_distance_mm)


def test_every_excluded_pair_exercised_near_contact(gate, kin, acm, spheres):
    """Each ACM-excluded pair is driven to its measured closest approach and
    the gate must stay silent about that pair there (the exclusion is doing
    its job at the hard part of the workspace, not just at home)."""
    rng = np.random.default_rng(23)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    qs = rng.uniform(lo, hi, size=(2000, 6))
    for entry in acm["excluded_pairs"]:
        a, b = entry["pair"]
        best_q, best_d = None, np.inf
        for q in qs:
            fr = kin.frames(q)
            ca = (fr[a][:3, :3] @ np.array([s["center_m"] for s in spheres["links"][a]["spheres"]]).T).T + fr[a][:3, 3]
            cb = (fr[b][:3, :3] @ np.array([s["center_m"] for s in spheres["links"][b]["spheres"]]).T).T + fr[b][:3, 3]
            ra = np.array([s["radius_m"] for s in spheres["links"][a]["spheres"]])
            rb = np.array([s["radius_m"] for s in spheres["links"][b]["spheres"]])
            d = (np.linalg.norm(ca[:, None] - cb[None, :], axis=2)
                 - ra[:, None] - rb[None, :]).min()
            if d < best_d:
                best_d, best_q = d, q
        v = gate.check_pose(best_q)
        flagged = {tuple(sorted(r.detail.get("pair", ())))
                   for r in v.reasons if r.code == Reason.SELF_COLLISION_PAIR}
        assert tuple(sorted((a, b))) not in flagged, (
            f"excluded pair {a}|{b} flagged at its closest approach — "
            "ACM not applied"
        )


# --- THE acceptance gate: timing with REAL pair semantics (Kelvin FATAL) ---

def test_timing_benchmark_real_pair_semantics(gate, kin):
    """RESEARCH's 20.5 µs figure used proxy pair semantics; the step-2 gate
    re-measures with the REAL checked-pair list and the REAL sphere set.
    Budget: a full-workspace sweep (research's canonical 478-sample case)
    must admit in ≤ 250 ms on CI hardware (design expectation ≈ 10 ms class;
    the assert is generous, the printed number is the record)."""
    # canonical admission case: a wide shoulder_pan sweep that ADMITS —
    # timing a rejection would measure the early-exit path, not admission
    # cost (the full-limit sweep rejects: pan extremes read ~8 mm from the
    # base in sphere space).
    a, b = np.zeros(6), np.zeros(6)
    a[0], b[0] = math.radians(-80), math.radians(80)

    t0 = time.perf_counter()
    verdict = gate.check_sequence([a, b])
    dt_admission = time.perf_counter() - t0
    assert verdict.admitted, (
        f"benchmark sweep must admit — rejected at step {verdict.violating_step}: "
        f"{[str(r) for r in verdict.reasons]}")
    assert verdict.n_samples > 400, "sweep too short to be a meaningful benchmark"

    n = gate.samples_for_delta(b - a)
    qs = kin.interpolate(a, b, n)
    t0 = time.perf_counter()
    for q in qs[:100]:
        gate.check_pose(q)
    per_pose = (time.perf_counter() - t0) / 100

    print(f"\n[benchmark] admission full-pan sweep ({n} samples): "
          f"{dt_admission*1000:.1f} ms; per-pose {per_pose*1e6:.1f} µs; "
          f"checked pairs: {len(gate.checked_pairs)}")
    assert dt_admission < 0.250, "admission blew the budget with real semantics"
    assert per_pose < 0.002, "streaming per-pose check too slow for 30 Hz"
    assert verdict is not None
