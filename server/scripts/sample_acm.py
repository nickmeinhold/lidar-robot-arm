"""Offline allowed-collision-matrix sampler — spine step 2 (DESIGN §2.4).

MoveIt-style pair policy WITH RECEIPTS: start from all link pairs; a pair is
excluded only with a reason and evidence —

- ``parent_child_adjacent``: joined by a joint; contact at the joint is
  structural, not a collision to prevent.
- ``near_adjacent_default_touching``: grandchild geometry measured touching
  in ≥90% of sampled poses (MoveIt's "Adjacent/Default" category — always-
  touching by construction, e.g. wrist|moving_jaw).
- ``sampled_never_hull_proven``: zero sphere contacts across the sample AND
  the convex-hull LP proof (a mesh is a subset of its hull; disjoint hulls ⟹
  disjoint meshes — an instrument that never touches spheres) holds at every
  sampled pose. Sampling alone is a check, not a certificate
  (amendment 9.1.11).

    python -m server.scripts.sample_acm

Output ``so101_acm.json`` is a reviewed, committed artifact pinned to the
sphere set + bake it was measured against.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import sys
from pathlib import Path

import numpy as np

from server.so101_kinematics import JOINT_ORDER, SO101Kinematics
from server.scripts.generate_spheres import MODEL_DIR, load_link_pointclouds

OUT_PATH = MODEL_DIR / "so101_acm.json"
N_SAMPLES = 20_000
SEED = 2
NEAR_ADJACENT_RATE = 0.90
HULL_POSES = 1500


def sphere_min_gaps(kin, links, pairs, qs):
    """Per-pair min gap (m) and collision rate over poses qs."""
    centers = {n: np.array([s["center_m"] for s in e["spheres"]])
               for n, e in links.items()}
    radii = {n: np.array([s["radius_m"] for s in e["spheres"]])
             for n, e in links.items()}
    mins = {p: np.inf for p in pairs}
    hits = {p: 0 for p in pairs}
    for q in qs:
        fr = kin.frames(q)
        world = {n: (fr[n][:3, :3] @ centers[n].T).T + fr[n][:3, 3]
                 for n in links}
        for p in pairs:
            a, b = p
            d = (np.linalg.norm(world[a][:, None] - world[b][None, :], axis=2)
                 - radii[a][:, None] - radii[b][None, :]).min()
            if d < mins[p]:
                mins[p] = d
            if d < 0:
                hits[p] += 1
    return mins, hits


def hull_proof(kin, pairs, qs):
    """True iff hulls are disjoint at EVERY pose for every pair (scipy)."""
    from scipy.optimize import linprog
    from scipy.spatial import ConvexHull

    clouds = load_link_pointclouds(MODEL_DIR)
    hulls = {}
    for name, pts in clouds.items():
        hulls[name] = pts[ConvexHull(pts).vertices]

    def intersect(A, B):
        na, nb = len(A), len(B)
        A_eq = np.zeros((5, na + nb))
        A_eq[:3, :na] = A.T
        A_eq[:3, na:] = -B.T
        A_eq[3, :na] = 1.0
        A_eq[4, na:] = 1.0
        res = linprog(np.zeros(na + nb), A_eq=A_eq,
                      b_eq=np.array([0, 0, 0, 1.0, 1.0]),
                      bounds=[(0, None)] * (na + nb), method="highs")
        return res.status == 0

    proven = {}
    for a, b in pairs:
        ok = True
        for q in qs:
            fr = kin.frames(q)
            Ah = (fr[a][:3, :3] @ hulls[a].T).T + fr[a][:3, 3]
            Bh = (fr[b][:3, :3] @ hulls[b].T).T + fr[b][:3, 3]
            if intersect(Ah, Bh):
                ok = False
                break
        proven[(a, b)] = ok
    return proven


def main() -> None:
    spheres = json.loads((MODEL_DIR / "so101_spheres.json").read_text())
    links = spheres["links"]
    kin = SO101Kinematics()
    names = list(links)
    all_pairs = list(itertools.combinations(names, 2))

    # structural adjacency: parent-child across ANY joint (fixed included),
    # collapsed over massless frames
    bake = json.loads((MODEL_DIR / "so101_urdf.json").read_text())
    adjacent = set()
    for j in bake["joints"]:
        a, b = j["parent"], j["child"]
        if a in links and b in links:
            adjacent.add(tuple(sorted((a, b))))

    rng = np.random.default_rng(SEED)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    qs = rng.uniform(lo, hi, size=(N_SAMPLES, 6))

    pairs = [tuple(sorted(p)) for p in all_pairs]
    print(f"sampling {len(pairs)} pairs × {N_SAMPLES} poses …")
    mins, hits = sphere_min_gaps(kin, links, pairs, qs)

    excluded, checked = [], []
    never_candidates = []
    for p in sorted(pairs):
        rate = hits[p] / N_SAMPLES
        gap_mm = round(mins[p] * 1000.0, 1)
        ev = {"n_samples": N_SAMPLES, "min_gap_mm": gap_mm,
              "collision_rate": round(rate, 4)}
        if p in adjacent:
            excluded.append({"pair": list(p), "reason": "parent_child_adjacent",
                             "evidence": {**ev, "joint": "structural"}})
        elif rate >= NEAR_ADJACENT_RATE:
            excluded.append({"pair": list(p),
                             "reason": "near_adjacent_default_touching",
                             "evidence": ev})
        elif hits[p] == 0:
            never_candidates.append((p, ev))
        else:
            checked.append({"pair": list(p), "evidence": ev})

    if never_candidates:
        print(f"hull-proving {len(never_candidates)} sampled-never pairs …")
        proof_qs = rng.uniform(lo, hi, size=(HULL_POSES, 6))
        proven = hull_proof(kin, [p for p, _ in never_candidates], proof_qs)
        for p, ev in never_candidates:
            if proven[p]:
                excluded.append({"pair": list(p),
                                 "reason": "sampled_never_hull_proven",
                                 "evidence": {**ev, "hull_poses": HULL_POSES}})
            else:
                # sampling said never, hulls could not prove it — CHECK it;
                # conservative direction (over-approximate hulls of concave
                # links overlap where meshes may not)
                checked.append({"pair": list(p),
                                "evidence": {**ev, "hull_proof": "inconclusive"}})

    out = {
        "provenance": {
            "generator": "server/scripts/sample_acm.py",
            "spheres_sha256": hashlib.sha256(
                (MODEL_DIR / "so101_spheres.json").read_bytes()).hexdigest(),
            "bake_sha256": spheres["provenance"]["bake_sha256"],
            "params": {"n_samples": N_SAMPLES, "seed": SEED,
                       "near_adjacent_rate": NEAR_ADJACENT_RATE,
                       "hull_poses": HULL_POSES},
        },
        "excluded_pairs": excluded,
        "checked_pairs": checked,
    }
    OUT_PATH.write_text(json.dumps(out, indent=1))
    print(f"excluded {len(excluded)}, checked {len(checked)} → {OUT_PATH}")
    for e in excluded:
        print(f"  − {e['pair'][0]} | {e['pair'][1]}: {e['reason']} "
              f"{e['evidence']}")


if __name__ == "__main__":
    sys.exit(main())
