"""Offline allowed-collision-matrix sampler — spine step 2 (DESIGN §2.4).

Pair policy with receipts, STRUCTURAL exclusions only:

- ``parent_child_adjacent`` (kinematic-graph distance 1): joined by a joint;
  contact at the joint is structural, not a collision to prevent.
- ``near_adjacent_default_touching``: an ALLOWLISTED grandchild pair whose
  interpenetration is mechanically structural, confirmed by measured ≥90%
  touching. The allowlist is the load-bearing guard (cage-match round 3):
  graph distance 2 alone is NOT a license — base|upper_arm is also d=2, and
  a future sphere regen that fattens its contact rate past 90% must FAIL
  the build loudly, never silently exclude the very pinch pair the K=12
  base model exists to protect. Rate confirms the allowlist; it never
  creates an exclusion.

Pairs that never collided across the sample are KEPT CHECKED. The cage-match
killed the former ``sampled_never_hull_proven`` category: per-pose convex-hull
LP disjointness at finitely many random poses is not a continuous-C-space
proof, its solver-status mapping failed open on "unknown", and the pair it
blessed (lower_arm|moving_jaw, min gap 5.7 mm) sits inside the hard margin.
Checking a pair costs microseconds; a wrong exclusion is a false negative by
policy, forever. Subtraction over certification.

    python -m server.scripts.sample_acm

Output ``so101_acm.json`` is a reviewed, committed artifact pinned to the
sphere set + bake it was measured against.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import sys
from collections import deque
from pathlib import Path

import numpy as np

from server.so101_kinematics import JOINT_ORDER, SO101Kinematics
from server.scripts.generate_spheres import MODEL_DIR

OUT_PATH = MODEL_DIR / "so101_acm.json"
N_SAMPLES = 20_000
SEED = 2
NEAR_ADJACENT_RATE = 0.90
# The ONLY grandchild pair whose exclusion is mechanically justified: the
# moving jaw pivots on the gripper motor which is bolted INTO the wrist
# bracket — jaw-base and wrist geometry interleave by construction at every
# jaw angle (measured −24.9 mm, 100% of poses). Any other d=2 pair reaching
# the rate threshold is a broken sphere model, not a new weld.
NEAR_ADJACENT_ALLOWLIST = {("moving_jaw_so101_v1_link", "wrist_link")}


def link_graph_distances(bake: dict, names: list[str]) -> dict[tuple[str, str], int]:
    """BFS hop-count between links over the joint graph (all joint types)."""
    adj: dict[str, set[str]] = {n: set() for n in names}
    for j in bake["joints"]:
        a, b = j["parent"], j["child"]
        if a in adj and b in adj:
            adj[a].add(b)
            adj[b].add(a)
    out: dict[tuple[str, str], int] = {}
    for src in names:
        dist = {src: 0}
        q = deque([src])
        while q:
            u = q.popleft()
            for v in adj[u]:
                if v not in dist:
                    dist[v] = dist[u] + 1
                    q.append(v)
        for dst in names:
            if dst != src:
                out[tuple(sorted((src, dst)))] = dist.get(dst, 99)
    return out


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


def main() -> None:
    spheres_bytes = (MODEL_DIR / "so101_spheres.json").read_bytes()
    spheres = json.loads(spheres_bytes)
    links = spheres["links"]
    bake = json.loads((MODEL_DIR / "so101_urdf.json").read_text())
    kin = SO101Kinematics()
    names = list(links)
    pairs = [tuple(sorted(p)) for p in itertools.combinations(names, 2)]
    gdist = link_graph_distances(bake, names)

    rng = np.random.default_rng(SEED)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])
    qs = rng.uniform(lo, hi, size=(N_SAMPLES, 6))

    print(f"sampling {len(pairs)} pairs × {N_SAMPLES} poses …")
    mins, hits = sphere_min_gaps(kin, links, pairs, qs)

    excluded, checked = [], []
    for p in sorted(pairs):
        rate = hits[p] / N_SAMPLES
        gap_mm = round(float(mins[p]) * 1000.0, 1)
        d = gdist[p]
        ev = {"n_samples": N_SAMPLES, "min_gap_mm": gap_mm,
              "collision_rate": round(rate, 4), "graph_distance": d}
        if d == 1:
            excluded.append({"pair": list(p), "reason": "parent_child_adjacent",
                             "evidence": ev})
        elif p in NEAR_ADJACENT_ALLOWLIST and rate >= NEAR_ADJACENT_RATE:
            excluded.append({"pair": list(p),
                             "reason": "near_adjacent_default_touching",
                             "evidence": ev,
                             "justification": (
                                 "jaw pivots on the gripper motor bolted into "
                                 "the wrist bracket; geometries interleave by "
                                 "construction at every jaw angle")})
        elif p in NEAR_ADJACENT_ALLOWLIST:
            raise RuntimeError(
                f"allowlisted near-adjacent pair {p} measured only "
                f"{rate:.0%} touching — the mechanical justification no "
                "longer matches the model; review before building an ACM")
        elif rate >= NEAR_ADJACENT_RATE:
            raise RuntimeError(
                f"pair {p} (graph distance {d}) reads {rate:.0%} colliding "
                "but is NOT on the near-adjacent allowlist — a fattened "
                "sphere model must never silently un-check a pinch pair; "
                "fix the model or make the mechanical case for the "
                "allowlist (cage-match round 3: the d==2+rate rule was a "
                "trapdoor for base|upper_arm)")
        else:
            note = {"never_collided_in_sample": hits[p] == 0}
            checked.append({"pair": list(p), "evidence": {**ev, **note}})

    out = {
        "provenance": {
            "generator": "server/scripts/sample_acm.py",
            "spheres_sha256": hashlib.sha256(spheres_bytes).hexdigest(),
            "bake_sha256": spheres["provenance"]["bake_sha256"],
            "params": {"n_samples": N_SAMPLES, "seed": SEED,
                       "near_adjacent_rate": NEAR_ADJACENT_RATE},
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
