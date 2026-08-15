"""Spine step 2 acceptance tests — sphere model + ACM + motion gate.

The crucible's step-2 acceptance gate (DESIGN §3.2 + amendments 9.1.11,
10.1.3): a REAL-pair-semantics timing benchmark, an adversarial pose corpus,
and a held-out oracle that does not share the sphere representation.

Soundness architecture being verified here:
  mesh ⊆ sphere-union (per link — constructive: every surface point is
  within the densify grid's covering radius h/√3 of a sample, every sample
  inside its cluster's max radius)  ⟹  a mesh self-collision implies a
  sphere contact  ⟹  the gate cannot false-negative. The constructive
  proof lives in the generator; the tests here CHECK it independently by
  random surface sampling (a different method, so verifier and verified do
  not share a blind spot) — a probabilistic check of a constructive claim,
  not itself a proof. ACM exclusions are STRUCTURAL only (graph distance),
  asserted directly against the artifact.
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
    NEAR_ADJACENT_ALLOWLIST,
    Reason,
    SPHERES_PATH,
    ACM_PATH,
    GATE_CONFIG_PATH,
)

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

def test_mesh_containment_independent_sampling(spheres):
    """THE soundness anchor, verified by a DIFFERENT method than the
    generator used (cage-match: a verifier sharing the generator's grid
    would share its blind spot). The generator fits a deterministic
    barycentric grid; this test throws RANDOM barycentric samples at every
    triangle — vertices, edges, and INTERIORS — and requires each inside the
    sphere union. Interiors are exactly where the old half-edge inflation
    had its 0.23 mm hole (covering radius is h/√3, not h/2)."""
    from server.scripts.generate_spheres import load_link_meshes

    rng = np.random.default_rng(97)
    for name, entry in spheres["links"].items():
        tris = load_link_meshes(MODEL_DIR)[name]  # (n,3,3) link frame
        centers = np.array([s["center_m"] for s in entry["spheres"]])
        radii = np.array([s["radius_m"] for s in entry["spheres"]])
        # 4 random interior points per triangle + the 3 vertices
        u = rng.random((len(tris), 4, 1)); v = rng.random((len(tris), 4, 1))
        flip = (u + v) > 1.0
        u = np.where(flip, 1.0 - u, u); v = np.where(flip, 1.0 - v, v)
        w = 1.0 - u - v
        interior = (tris[:, None, 0] * u + tris[:, None, 1] * v
                    + tris[:, None, 2] * w).reshape(-1, 3)
        # witness points (cage-match r4): centroids + acute-face
        # circumcenters — the family where a short inflation hides
        cent = tris.mean(axis=1)
        a3, b3, c3 = tris[:, 0], tris[:, 1], tris[:, 2]
        ab, ac = b3 - a3, c3 - a3
        abn = (ab * ab).sum(1); acn = (ac * ac).sum(1)
        d_ = 2 * (abn * acn - ((ab * ac).sum(1)) ** 2)
        with np.errstate(divide="ignore", invalid="ignore"):
            s_ = (acn * (abn - (ab * ac).sum(1))) / d_
            t_ = (abn * (acn - (ab * ac).sum(1))) / d_
        acute = (np.isfinite(s_) & np.isfinite(t_) & (s_ >= 0) & (t_ >= 0)
                 & (s_ + t_ <= 1))
        circ = a3[acute] + ab[acute] * s_[acute, None] + ac[acute] * t_[acute, None]
        pts = np.concatenate([tris.reshape(-1, 3), interior, cent, circ])
        d = np.linalg.norm(pts[:, None, :] - centers[None, :, :], axis=2)
        inside = (d <= radii[None, :] + 1e-9).any(axis=1)
        assert inside.all(), (
            f"{name}: {(~inside).sum()} of {len(pts)} surface samples escape "
            "the sphere union — containment broken")


def test_inflation_is_grid_covering_radius(spheres):
    """The radius inflation must be ≥ the densify grid's covering radius
    h/√3 (any triangle point is within that of a sample) — h/2 was the
    cage-match's 0.23 mm hole."""
    h = 0.003  # DENSIFY_EDGE_M
    for name, entry in spheres["links"].items():
        assert entry["edge_inflation_m"] >= h / math.sqrt(3.0) - 1e-12
        assert entry["edge_inflation_m"] < 0.0025


# --- ACM: every exclusion carries its receipt ------------------------------

def test_acm_exclusions_have_reasons(acm):
    """Pair policy with receipts (Carnot): no bare 'adjacent' hand-waves."""
    assert acm["excluded_pairs"], "ACM must exclude at least the adjacent pairs"
    for entry in acm["excluded_pairs"]:
        assert entry["reason"] in {
            "parent_child_adjacent", "near_adjacent_default_touching",
        }, "only STRUCTURAL exclusions are permitted (the sampled-never " \
           "category was killed by the cage-match: sampling is not a proof)"
        d = entry["evidence"]["graph_distance"]
        if entry["reason"] == "parent_child_adjacent":
            assert d == 1
        else:
            assert tuple(sorted(entry["pair"])) in NEAR_ADJACENT_ALLOWLIST, (
                f"{entry['pair']}: near-adjacent exclusion NOT on the "
                "allowlist — the d==2+rate trapdoor is back")
        assert entry["justification"], f"{entry['pair']}: no receipt"


def test_acm_checked_pairs_are_the_complement(acm, spheres):
    n_links = len(spheres["links"])
    all_pairs = n_links * (n_links - 1) // 2
    assert len(acm["excluded_pairs"]) + len(acm["checked_pairs"]) == all_pairs



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
    assert v.n_samples == 1
    assert np.isfinite(v.min_distance_mm), \
        "a one-frame certificate must report the real pose distance"


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
    travel is almost always an author who wanted the forbidden short-cut.
    FAIL CLOSED (cage-match consensus): the sequence is REJECTED with
    WRAP_SEAM so the IR repair loop rewrites it — never silently drive the
    long way around."""
    a = q_of(kin, wrist_roll=kin.joints["wrist_roll"].lower + 0.05)
    b = q_of(kin, wrist_roll=kin.joints["wrist_roll"].upper - 0.05)
    v = gate.check_sequence([a, b])
    assert not v.admitted
    assert any(r.code == Reason.WRAP_SEAM for r in v.reasons)
    assert v.violating_step == 0


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
    # the REAL step the gate takes: n samples ⟹ n−1 intervals of dq/(n−1)
    # (round-1 of the cage-match caught the test proving dq/n — a smaller
    # step than the gate actually takes)
    sweep_mm = gate.sweep_upper_bound_mm(dq / (n_multi - 1))
    assert sweep_mm <= gate.sampling_allowance_mm + 1e-9
    # edge case: total sweep just OVER an integer multiple of the allowance
    target = 3.0 * gate.sampling_allowance_mm + 1e-6
    dq_edge = np.zeros(6)
    dq_edge[0] = target / gate.levers_mm["shoulder_pan"]
    n_edge = gate.samples_for_delta(dq_edge)
    assert gate.sweep_upper_bound_mm(dq_edge / (n_edge - 1)) \
        <= gate.sampling_allowance_mm + 1e-9


def test_derived_step_matches_research_regime(gate):
    """The derived bound must land in the research's measured regime:
    ~0.38° for 10 mm at worst-case single-joint lever (7.79 mm/deg)."""
    lever_mm_per_rad = gate.levers_mm["shoulder_pan"]
    step_deg = math.degrees(gate.sampling_allowance_mm / lever_mm_per_rad)
    assert 0.1 <= step_deg <= 1.0


# --- margin budget ---------------------------------------------------------

def test_encoder_term_tracks_artifact_lever(gate):
    """The encoder-quantization margin term must be derived from the
    ARTIFACT's lever bound, not research prose (amendment 9.1.8; cage-match
    r4 caught it measured against the ghost 446 mm sampled lever)."""
    cfg = json.loads(GATE_CONFIG_PATH.read_text())
    tick_deg = 360.0 / 4096.0
    required_mm = math.radians(tick_deg) * gate.levers_mm["shoulder_pan"]
    assert cfg["margin_budget_mm"]["encoder_quantization"]["value"] \
        >= required_mm - 1e-9, (
        f"encoder term below the artifact-derived {required_mm:.3f} mm")


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


def test_never_collided_pairs_stay_checked(acm):
    """The cage-match's subtraction: pairs that never collided across the
    20k sample are KEPT CHECKED (evidence recorded), never excluded —
    sampling dusts C-space, it does not cover it, and the closest such pair
    sat inside the hard margin."""
    never = [e for e in acm["checked_pairs"]
             if e["evidence"].get("never_collided_in_sample")]
    assert never, "expected at least one never-collided-yet-checked pair"
    for e in never:
        assert e["evidence"]["graph_distance"] > 1



# --- THE acceptance gate: timing with REAL pair semantics (Kelvin FATAL) ---

def test_timing_benchmark_real_pair_semantics(gate, kin):
    """RESEARCH's 20.5 µs figure used proxy pair semantics; the step-2 gate
    re-measures with the REAL checked-pair list and the REAL sphere set.
    Three honest measurements (cage-match: the first draft's labels oversold
    an easy 1-DOF pan as "full-workspace" and timed per-pose on the same
    cache-hot slice): (a) an ADMITTED wide pan sweep — the canonical chat
    admission shape, not a workspace tour; (b) per-pose cost over RANDOM
    in-limit poses across the workspace (mixed CLEAR/SOFT/HARD); (c) the
    REJECT path on a violating sequence. Assert ceilings are deliberately
    generous for CI hardware; the printed numbers are the record."""
    # (a) admitted wide pan sweep — a single-DOF admission case, chosen
    # because it admits end-to-end (the full-limit sweep rejects at the
    # extremes); it is NOT a workspace tour and is not claimed to be one.
    a, b = np.zeros(6), np.zeros(6)
    a[0], b[0] = math.radians(-80), math.radians(80)
    t0 = time.perf_counter()
    verdict = gate.check_sequence([a, b])
    dt_admission = time.perf_counter() - t0
    assert verdict.admitted, (
        f"benchmark sweep must admit — rejected at step {verdict.violating_step}: "
        f"{[str(r) for r in verdict.reasons]}")
    assert verdict.n_samples > 400, "sweep too short to be a meaningful benchmark"

    # (b) per-pose cost over RANDOM in-limit poses (mixed statuses, no
    # cache-friendly correlation between consecutive poses)
    rng = np.random.default_rng(41)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    rand_qs = rng.uniform(lo, hi, size=(200, 6))
    t0 = time.perf_counter()
    for q in rand_qs:
        gate.check_pose(q)
    per_pose = (time.perf_counter() - t0) / len(rand_qs)

    # (c) BOTH reject shapes (cage-match round 3: a start-reject is one
    # check_pose; a LATE reject that dies deep in the sweep costs nearly a
    # full admission — time both, never let the cheap one stand in)
    bad = np.array(rand_qs[0]); bad[1] = -1.6
    bad[2] = kin.joints["elbow_flex"].lower
    bad[3] = kin.joints["wrist_flex"].lower
    t0 = time.perf_counter()
    rej = gate.check_sequence([bad, np.zeros(6)])
    dt_reject_start = time.perf_counter() - t0
    assert not rej.admitted
    # late reject: start CLEAR, end HARD — dies deep into the interpolation
    late_end = q_of(kin, shoulder_lift=-1.6,
                    elbow_flex=kin.joints["elbow_flex"].lower,
                    wrist_flex=kin.joints["wrist_flex"].lower)
    t0 = time.perf_counter()
    rej_late = gate.check_sequence([q_of(kin), late_end])
    dt_reject_late = time.perf_counter() - t0
    assert not rej_late.admitted
    assert dt_reject_late < 0.250, "late reject blew the admission budget"
    # PROVE it died late: n_samples now counts EXECUTED checks, so a
    # genuinely-late rejection must have burned a deep fraction of the path
    assert rej_late.n_samples > 100, (
        f"'late' reject executed only {rej_late.n_samples} checks — "
        "it is wearing the late robe over an early exit")
    assert rej.n_samples <= 2, "start-reject should exit almost immediately"
    assert len(gate.checked_pairs) == 14, (
        "checked-pair count drifted — timing numbers are not comparable")

    print(f"\n[benchmark] admitted ±80° 1-DOF pan sweep "
          f"({verdict.n_samples} samples): {dt_admission*1000:.1f} ms; "
          f"per-pose over 200 random workspace poses: {per_pose*1e6:.1f} µs; "
          f"start-reject: {dt_reject_start*1000:.2f} ms; "
          f"late-reject: {dt_reject_late*1000:.1f} ms; "
          f"checked pairs: {len(gate.checked_pairs)}")
    assert dt_admission < 0.250, "admission blew the budget with real semantics"
    assert per_pose < 0.002, "streaming per-pose check too slow for 30 Hz"


# --- refusal on artifact drift (the constructor is a safety check too) -----

def test_gate_refuses_acm_from_different_sphere_set(tmp_path):
    """Chain of custody bake → spheres → ACM must be unbroken (cage-match:
    regenerating spheres while keeping the old ACM previously slipped
    through — the spheres hash was recorded but never verified)."""
    spheres = json.loads(SPHERES_PATH.read_text())
    spheres["links"]["base_link"]["spheres"][0]["radius_m"] += 1e-6
    p = tmp_path / "spheres.json"
    p.write_text(json.dumps(spheres))
    with pytest.raises(RuntimeError, match="DIFFERENT sphere set"):
        MotionGate(spheres_path=p)


def test_gate_refuses_acm_that_drops_a_pair(tmp_path):
    """The ACM must PARTITION the pair set — a pair absent from both lists
    would silently never be checked (RuntimeError, and it survives -O)."""
    acm = json.loads(ACM_PATH.read_text())
    acm["checked_pairs"] = acm["checked_pairs"][1:]
    p = tmp_path / "acm.json"
    p.write_text(json.dumps(acm))
    with pytest.raises(RuntimeError, match="partition"):
        MotionGate(acm_path=p)


def test_gate_refuses_mismatched_bake(tmp_path):
    """A sphere artifact generated from a DIFFERENT bake must be refused at
    load — silently running over drifted geometry is the failure mode the
    provenance hash exists to kill (amendment 9.1.8)."""
    tampered = json.loads(SPHERES_PATH.read_text())
    tampered["provenance"]["bake_sha256"] = "0" * 64
    p = tmp_path / "spheres.json"
    p.write_text(json.dumps(tampered))
    with pytest.raises(RuntimeError, match="DIFFERENT bake"):
        MotionGate(spheres_path=p)


def test_shoulder_table_clearance_is_pose_invariant(gate, kin):
    """shoulder_link's table exemption rests on 'its only DOF is pan about
    the vertical axis' — verify the invariant instead of trusting the prose
    (cage-match, Carnot): its lowest sphere-bottom must not vary across pan."""
    lows = []
    for pan in np.linspace(kin.joints["shoulder_pan"].lower,
                           kin.joints["shoulder_pan"].upper, 25):
        q = q_of(kin, shoulder_pan=float(pan))
        w = gate._world_spheres(q)
        o = gate._offsets["shoulder_link"]
        r = gate._radii["shoulder_link"]
        lows.append(float((w[o:o + len(r), 2] - r).min()))
    # measured spread is ~16 nm (the URDF pan axis carries a ~3 µrad export
    # tilt); 0.01 mm is the meaningful ceiling — far below every margin term
    assert max(lows) - min(lows) < 1e-5, (
        f"shoulder_link table clearance varies {1000*(max(lows)-min(lows)):.4f} mm "
        "across pan — the exemption's stated invariant is false")


def test_no_tunneling_through_sampled_segment(gate, kin):
    """Falsifier #8 made executable: between two checked samples the gap can
    dip at most S/2 (both endpoints checked; g(t) ≥ min(g0,g1) − S/2).
    Cage-match r4: a home-posture fixture is vacuous (24 mm of air) — so the
    lemma is exercised on segments found NEAR THE MARGIN: random SOFT-status
    poses (clearance inside the soft band), random direction, one full
    sampling allowance of multi-joint sweep, interior probed 8× denser than
    the gate samples."""
    rng = np.random.default_rng(59)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    sampling_term = gate.sampling_allowance_mm / 2.0
    tested = 0
    for q in rng.uniform(lo, hi, size=(3000, 6)):
        if gate.check_pose(q).status != GateStatus.SOFT:
            continue
        direction = rng.normal(size=6)
        direction /= np.abs(direction) @ gate._lever_mm_vec
        dq = direction * gate.sampling_allowance_mm  # sweep == allowance
        b = np.clip(q + dq, lo, hi)
        ga = gate.check_pose(q).min_distance_mm
        gb = gate.check_pose(b).min_distance_mm
        dense = kin.interpolate(q, b, 17)
        interior_min = min(gate.check_pose(x).min_distance_mm for x in dense)
        assert interior_min >= min(ga, gb) - sampling_term - 1e-6, (
            f"interior gap dipped {min(ga, gb) - interior_min:.3f} mm below "
            f"endpoints near the margin — exceeds budgeted S/2 = "
            f"{sampling_term} mm at q={q}")
        tested += 1
        if tested >= 25:
            break
    assert tested >= 10, "corpus found too few near-margin segments to test"
