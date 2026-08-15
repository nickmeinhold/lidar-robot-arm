"""Offline sphere-cluster generator — spine step 2 (crucible DESIGN §2.4).

Derives the SO-101 collision model from the SAME meshes the bake references:
per-link k-means spheres over a DENSIFIED surface cloud (sample spacing
≤ DENSIFY_EDGE_M), radius = per-cluster MAX sample distance plus the grid's
COVERING RADIUS, DENSIFY_EDGE_M/√3 — the distance bound from any point of a
triangle to its nearest barycentric-grid sample (worst case: the circumcenter
of an equilateral grid cell, h/√3). Containment of the continuous surface
follows: every surface point is within h/√3 of a sample, and every sample is
inside its cluster's max radius. (The research's own "half longest edge" rule
was doubly wrong here: these CAD exports carry ~80 mm flat triangles, and
h/2 under-covers a triangular grid — the cage-match caught the 0.23 mm hole.)

The OUTPUT (`so101_spheres.json`) is a reviewed, committed artifact — a
safety judgment with provenance — regenerated only deliberately:

    python -m server.scripts.generate_spheres

Provenance written into the artifact: bake sha256 (the gate refuses a
mismatched bake at load — amendment 9.1.8: derived constants derive from
artifacts, not prose), per-link fit report, generator parameters, and the
per-joint lever-arm table measured over sampled poses (the sampling bound's
input).
"""
from __future__ import annotations

import hashlib
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

from server.so101_kinematics import (
    JOINT_ORDER, SO101Kinematics, _origin_matrix)

MODEL_DIR = Path(__file__).parent.parent / "static" / "models" / "SO101"
OUT_PATH = MODEL_DIR / "so101_spheres.json"

# Per-link cluster counts. The design's §7 names k-per-link an OPEN variable;
# K=6 everywhere (RESEARCH's default) was falsified at posture level during
# step-2 acceptance: base_link at K=6 grows a ~50 mm sphere that swallows the
# air above the base, reading home-posture pan sweeps at ~1 mm. Measured on
# 2026-08-15 (worst base|upper_arm gap over a ±80° pan sweep, elbow bent):
#   K_base=6 → 1.2 mm · K=10 → 13.4 mm · K=12 → 14.0 mm · K=16 → 15.5 mm
K_DEFAULT = 6
K_PER_LINK = {"base_link": 12, "upper_arm_link": 8}
KMEANS_ITERS = 60
KMEANS_SEED = 42
LEVER_POSES = 3000
LEVER_SEED = 1


def read_stl(path: Path) -> np.ndarray:
    """Binary STL → (n_triangles, 3, 3) float32 vertex array (meters)."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    return data[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def load_link_meshes(model_dir: Path = MODEL_DIR) -> dict[str, np.ndarray]:
    """{link_name: (n_tris, 3, 3) triangles in LINK frame} for every link
    with visual geometry, straight from the bake (single geometry truth)."""
    bake = json.loads((model_dir / "so101_urdf.json").read_text())
    out: dict[str, np.ndarray] = {}
    for link in bake["links"]:
        # the containment proof is over the bake's ONLY geometry channel —
        # refuse loudly if a future bake grows a separate collision-mesh set
        # this generator would silently ignore (cage-match r4)
        unexpected = set(link) - {"name", "visuals"}
        if unexpected:
            raise RuntimeError(
                f"bake link {link['name']} carries unexpected geometry "
                f"channels {unexpected} — extend the generator before "
                "trusting containment")
        tris = []
        for v in link.get("visuals", []):
            t = read_stl(model_dir / "assets" / v["mesh"])
            m = _origin_matrix(v["origin"]["xyz"], v["origin"]["rpy"])
            flat = t.reshape(-1, 3)
            flat = (m[:3, :3] @ flat.T).T + m[:3, 3]
            tris.append(flat.reshape(-1, 3, 3))
        if tris:
            out[link["name"]] = np.concatenate(tris)
    return out


def load_link_pointclouds(model_dir: Path = MODEL_DIR) -> dict[str, np.ndarray]:
    """{link_name: (N, 3) unique-ish vertex cloud in LINK frame}."""
    return {name: tris.reshape(-1, 3)
            for name, tris in load_link_meshes(model_dir).items()}


DENSIFY_EDGE_M = 0.003  # sample the surface at ≤3 mm spacing


def densify(tris: np.ndarray, max_edge: float = DENSIFY_EDGE_M) -> np.ndarray:
    """(n,3,3) triangles → (N,3) surface point cloud with grid spacing
    ≤ max_edge. Any point of any triangle lies within max_edge/√3 (the grid
    covering radius) of some sample, so cloud-containment + a max_edge/√3
    radius inflation contains the continuous surface."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    longest = np.maximum(np.maximum(
        np.linalg.norm(a - b, axis=1),
        np.linalg.norm(b - c, axis=1)),
        np.linalg.norm(c - a, axis=1))
    out = [tris.reshape(-1, 3)]
    for n_div in range(2, int(np.ceil(longest.max() / max_edge)) + 1):
        sel = (longest > (n_div - 1) * max_edge) & (longest <= n_div * max_edge)
        if not sel.any():
            continue
        ta, tb, tc = a[sel], b[sel], c[sel]
        # barycentric grid at resolution n_div
        for i in range(n_div + 1):
            for j in range(n_div + 1 - i):
                u, v = i / n_div, j / n_div
                w = 1.0 - u - v
                out.append(ta * u + tb * v + tc * w)
    return np.concatenate(out)


def kmeans(points: np.ndarray, k: int, iters: int, seed: int) -> np.ndarray:
    """Deterministic Lloyd's k-means, numpy only. Returns (k,) labels' centers
    via (centers, labels)."""
    rng = np.random.default_rng(seed)
    centers = points[rng.choice(len(points), size=k, replace=False)]
    labels = np.zeros(len(points), dtype=int)
    for _ in range(iters):
        d = np.linalg.norm(points[:, None, :] - centers[None, :, :], axis=2)
        new_labels = d.argmin(axis=1)
        if (new_labels == labels).all():
            break
        labels = new_labels
        for j in range(k):
            sel = points[labels == j]
            if len(sel):
                centers[j] = sel.mean(axis=0)
    return centers, labels


def fit_link(tris: np.ndarray, k: int) -> tuple[list[dict], dict]:
    """Per-link max-radius spheres over the DENSIFIED surface cloud.
    Radius = per-cluster max sample distance + the grid covering radius
    DENSIFY_EDGE_M/√3, so the true (continuous) surface is provably inside
    the union (any triangle point is within h/√3 of a sample: acute cells
    are bounded by the equilateral circumradius h/√3, obtuse cells by
    h/2 < h/√3)."""
    pts = densify(tris)
    centers, labels = kmeans(pts, k, KMEANS_ITERS, KMEANS_SEED)
    edge_inflation = DENSIFY_EDGE_M / math.sqrt(3.0)  # grid covering radius

    spheres = []
    for j in range(k):
        sel = pts[labels == j]
        r = float(np.linalg.norm(sel - centers[j], axis=1).max())
        spheres.append({
            "center_m": [float(c) for c in centers[j]],
            "radius_m": r + edge_inflation,
        })
    report = {
        "n_vertices": int(tris.reshape(-1, 3).shape[0]),
        "n_dense_points": int(len(pts)),
        "n_triangles": int(len(tris)),
        "max_raw_radius_m": max(s["radius_m"] for s in spheres) - edge_inflation,
        "edge_inflation_m": edge_inflation,
    }
    return spheres, report


def measure_levers(links: dict[str, list[dict]]) -> dict[str, float]:
    """Per-joint lever arm: max perpendicular distance from the joint's axis
    to any DOWNSTREAM sphere surface, over sampled poses (RESEARCH Q2's
    method, re-derived from THIS sphere set per amendment 9.1.8)."""
    kin = SO101Kinematics()
    rng = np.random.default_rng(LEVER_SEED)
    lo = np.array([kin.joints[n].lower for n in JOINT_ORDER])
    hi = np.array([kin.joints[n].upper for n in JOINT_ORDER])

    # downstream links per joint = child links of this joint and beyond
    chain_children = [kin.joints[n].child_link for n in JOINT_ORDER]
    downstream = {
        n: [c for c in chain_children[i:]] + (["moving_jaw_so101_v1_link"]
            if "moving_jaw_so101_v1_link" not in chain_children[i:] else [])
        for i, n in enumerate(JOINT_ORDER)
    }
    # moving_jaw is the gripper joint's child; ensure no duplicates + only
    # links that actually carry spheres
    downstream = {n: [c for c in dict.fromkeys(v) if c in links]
                  for n, v in downstream.items()}

    # Analytic pose-independent UPPER BOUND per joint (triangle inequality:
    # rotations preserve norms, so distance from the joint origin to any
    # downstream sphere surface ≤ Σ downstream joint-origin offsets + the
    # sphere's own ‖center‖+radius). The sampled max below is an ESTIMATE
    # used for reporting; the artifact's lever is max(bound, sampled) —
    # in practice the bound — so the sweep bound truly upper-bounds.
    bounds = {}
    for i, name in enumerate(JOINT_ORDER):
        best = 0.0
        cum = 0.0
        for k in range(i, len(JOINT_ORDER)):
            nk = JOINT_ORDER[k]
            if k > i:
                cum += float(np.linalg.norm(
                    kin.joints[nk].origin[:3, 3]))
            link_name = kin.joints[nk].child_link
            if link_name in links:
                reach = max(
                    float(np.linalg.norm(s_["center_m"]) + s_["radius_m"])
                    for s_ in links[link_name])
                best = max(best, cum + reach)
        bounds[name] = best

    levers = {n: 0.0 for n in JOINT_ORDER}
    qs = rng.uniform(lo, hi, size=(LEVER_POSES, 6))
    for q in qs:
        fr = kin.frames(q)
        # joint axis in world: local Z of the joint frame; joint frame =
        # parent chain @ origin (before Rz) — child frame shares the axis.
        for i, name in enumerate(JOINT_ORDER):
            child = kin.joints[name].child_link
            m = fr[child]
            axis_point = m[:3, 3]
            axis_dir = m[:3, 2]
            for link_name in downstream[name]:
                lm = fr[link_name]
                for s in links[link_name]:
                    c = lm[:3, :3] @ np.array(s["center_m"]) + lm[:3, 3]
                    v = c - axis_point
                    perp = np.linalg.norm(v - axis_dir * (v @ axis_dir))
                    lever = perp + s["radius_m"]
                    if lever > levers[name]:
                        levers[name] = lever
    return (
        {n: round(max(bounds[n], levers[n]) * 1000.0, 1) for n in JOINT_ORDER},
        {n: round(levers[n] * 1000.0, 1) for n in JOINT_ORDER},
    )


def main() -> None:
    bake_bytes = (MODEL_DIR / "so101_urdf.json").read_bytes()
    meshes = load_link_meshes(MODEL_DIR)
    links_out: dict[str, dict] = {}
    for name, tris in meshes.items():
        k = K_PER_LINK.get(name, K_DEFAULT)
        spheres, report = fit_link(tris, k)
        links_out[name] = {
            "spheres": spheres,
            "edge_inflation_m": report["edge_inflation_m"],
            "fit_report": report,
        }
        print(f"{name}: {report['n_vertices']} verts → {k} spheres, "
              f"max r {max(s['radius_m'] for s in spheres)*1000:.1f} mm "
              f"(edge inflation {report['edge_inflation_m']*1000:.2f} mm)")

    levers, levers_sampled = measure_levers(
        {n: e["spheres"] for n, e in links_out.items()})
    print("levers_mm (bound):", levers)
    print("levers_mm (sampled estimate):", levers_sampled)

    # v0 table plane: the surface the base stands on = the base mesh's lowest
    # point in the base frame (the URDF origin is NOT the tabletop — the base
    # mesh extends below it). Step 3 replaces this with a measured touch-test
    # (amendment 10.1.8).
    base_min_z = float(densify(meshes["base_link"])[:, 2].min())
    print(f"base_min_z_m: {base_min_z:.4f}")

    out = {
        "provenance": {
            "generator": "server/scripts/generate_spheres.py",
            "bake_sha256": hashlib.sha256(bake_bytes).hexdigest(),
            "params": {
                "k_default": K_DEFAULT, "k_per_link": K_PER_LINK,
                "kmeans_iters": KMEANS_ITERS,
                "kmeans_seed": KMEANS_SEED, "radius_rule": "max+covering_radius(h/sqrt(3))",
                "lever_poses": LEVER_POSES, "lever_seed": LEVER_SEED,
            },
        },
        "links": links_out,
        "levers_mm": levers,
        "levers_sampled_mm": levers_sampled,
        "base_min_z_m": base_min_z,
    }
    OUT_PATH.write_text(json.dumps(out, indent=1))
    print(f"wrote {OUT_PATH}")


if __name__ == "__main__":
    sys.exit(main())
