"""Shared kinematic model for the SO-100/SO-101 arm — the spine of See/Grab/Stay-safe.

This is Slice 1 of the AR-teleop shared-model plan
(`docs/crucible/ar-teleop-shared-model/`): a standalone forward-kinematics core
that turns 6 joint angles into a capsule per link. NO consumer is embedded here —
the collision gate (Slice 2), the AR overlay (Slice 4), and the viewer glow
(Slice 3) all hang off this one module.

CONVENTION (load-bearing — Fold #5): this reproduces the web viewer's EXISTING
angle convention (`server/static/viewer.html`), NOT a fresh URDF frame. The viewer
already renders a plausible arm and the controller already drives it correctly, so
that ONE validated convention is the source of truth. We harvest the URDF only for
link dimensions; the frame/axis mapping comes from the viewer. If the viewer's
hierarchy changes, these constants and the FK must change with it.

Coordinate frame (Three.js, right-handed, Y-up):
  +X right, +Y up, +Z toward the viewer. Each link extends along local -Y at rest.
  shoulder_yaw   : Ry  — whole arm about vertical, 0 = forward (-Z at pitch=pi/2)
  shoulder_pitch : Rx  — angle from DOWN. 0 = -Y (down), pi/2 = -Z (forward), pi = +Y (up)
  elbow_pitch    : Rx  — forearm relative to upper arm. 0 = straight
  wrist_pitch    : Rx
  wrist_roll     : Ry  — twist about the wrist link axis (reorients gripper, not link dir)
  gripper        : 0 = closed, 1 = open (finger x-offset only, not a rotation)

Because pitch/elbow/wrist_pitch are all Rx, within the yaw frame they accumulate;
each pivot origin is directly a capsule endpoint.
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

import numpy as np

# ─── Link dimensions (meters) — MIRRORED from viewer.html; keep in sync ──────
BASE_HEIGHT = 0.048
BASE_RADIUS = 0.035
UPPER_ARM_LEN = 0.1126
FOREARM_LEN = 0.1349
WRIST_LEN = 0.0601

# Gripper finger geometry (viewer.html), in the wrist-roll frame.
FINGER_H = 0.040
GRIP_MAX_HALF = 0.0175      # half of the 35 mm max opening
_FINGER_TOP_Y = -0.012      # fingers start just below the gripper base
_CLOSED_OFFSET = 0.004      # fingers stay slightly apart when "closed"

# ─── Collision-capsule radii (meters) ────────────────────────────────────────
# PLACEHOLDER radii = the viewer's RENDER radii. These are an OPEN VARIABLE
# (DESIGN.md): the real collision radii must be measured off the printed arm to
# the *bounding* radius including brackets/servo housings — conservative, so the
# gate loses a little workspace rather than being unsafe. Tuned by eye + camera +
# the Slice-0 load monitor in Slices 1-3. Bumped a touch above the render radii
# as a starting safety pad; do NOT treat as final.
_RENDER_LINK_R = 0.012
LINK_RADII = {
    "base": BASE_RADIUS,
    "upper_arm": _RENDER_LINK_R,
    "forearm": _RENDER_LINK_R * 0.85,
    "wrist": _RENDER_LINK_R * 0.7,
    "gripper_finger": 0.010,   # a jaw's bounding radius (finger box ~8x10 mm)
}

JOINT_NAMES = (
    "shoulder_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class Capsule:
    """A line-segment-with-radius collision primitive.

    p0, p1 are world-space endpoints (numpy (3,) float arrays); radius in meters.
    Closest-distance between two capsules = segment-segment distance minus the two
    radii (Slice 2 uses Ericson's ClosestPtSegmentSegment).
    """

    name: str
    p0: np.ndarray
    p1: np.ndarray
    radius: float

    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))


# ─── Homogeneous transform helpers (4x4) ─────────────────────────────────────
def _rot_x(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def _rot_y(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1],
    ], dtype=float)


def _trans(x: float, y: float, z: float) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _origin(m: np.ndarray) -> np.ndarray:
    """World position of a frame's origin."""
    return m[:3, 3].copy()


def _apply(m: np.ndarray, p: tuple[float, float, float]) -> np.ndarray:
    """Transform a local point by a 4x4 matrix, return world (3,)."""
    v = m @ np.array([p[0], p[1], p[2], 1.0])
    return v[:3]


def _norm_angles(angles: dict) -> dict:
    out = {k: 0.0 for k in JOINT_NAMES}
    out["gripper"] = 1.0
    for k, v in angles.items():
        if k not in out:
            raise KeyError(f"unknown joint {k!r}; expected one of {JOINT_NAMES}")
        out[k] = float(v)
    return out


def frames(angles: dict) -> dict[str, np.ndarray]:
    """Forward kinematics: joint angles → the 4x4 world transform of each pivot.

    Mirrors viewer.html's nested-Group hierarchy exactly:
        base → yawPivot(Ry) → pitchPivot(Rx) → [upper arm] → elbowPivot(Rx)
             → [forearm] → wristPitchPivot(Rx) → [wrist] → wristRollPivot(Ry) → gripper
    Returns a dict of frame-name → 4x4. Origins of successive frames are the
    capsule endpoints.
    """
    a = _norm_angles(angles)

    m_yaw = _trans(0, BASE_HEIGHT, 0) @ _rot_y(a["shoulder_yaw"])
    m_pitch = m_yaw @ _rot_x(a["shoulder_pitch"])                 # upper-arm start
    m_elbow = m_pitch @ _trans(0, -UPPER_ARM_LEN, 0) @ _rot_x(a["elbow_pitch"])
    m_wpitch = m_elbow @ _trans(0, -FOREARM_LEN, 0) @ _rot_x(a["wrist_pitch"])
    m_wroll = m_wpitch @ _trans(0, -WRIST_LEN, 0) @ _rot_y(a["wrist_roll"])

    return {
        "base": np.eye(4),
        "shoulder": m_pitch,
        "elbow": m_elbow,
        "wrist_pitch": m_wpitch,
        "wrist_roll": m_wroll,
    }


def capsules(angles: dict) -> list[Capsule]:
    """Joint angles → the collision capsule set (self-links + gripper jaws + base).

    Per DESIGN.md the geometric claim is honestly narrowed to "self + fixed-base
    collision"; world objects / operator / cables are the reactive-load reflex's
    job. The ground plane (table) is exposed separately via `GROUND_PLANE_Y`.
    """
    a = _norm_angles(angles)
    f = frames(a)

    p_shoulder = _origin(f["shoulder"])
    p_elbow = _origin(f["elbow"])
    p_wpitch = _origin(f["wrist_pitch"])
    p_wroll = _origin(f["wrist_roll"])

    caps = [
        # Fixed base column (the arm hitting its own base/mount).
        Capsule("base", np.array([0.0, 0.0, 0.0]),
                np.array([0.0, BASE_HEIGHT, 0.0]), LINK_RADII["base"]),
        Capsule("upper_arm", p_shoulder, p_elbow, LINK_RADII["upper_arm"]),
        Capsule("forearm", p_elbow, p_wpitch, LINK_RADII["forearm"]),
        Capsule("wrist", p_wpitch, p_wroll, LINK_RADII["wrist"]),
    ]

    # Gripper jaws: two fingers offset in local ±X, spanning the finger height,
    # in the wrist-roll frame. Opening widens the x-offset (matches viewer).
    half_open = a["gripper"] * GRIP_MAX_HALF
    x_off = _CLOSED_OFFSET + half_open
    finger_bottom_y = _FINGER_TOP_Y - FINGER_H
    m = f["wrist_roll"]
    for side, sx in (("left", -1.0), ("right", 1.0)):
        top = _apply(m, (sx * x_off, _FINGER_TOP_Y, 0.0))
        bot = _apply(m, (sx * x_off, finger_bottom_y, 0.0))
        caps.append(Capsule(f"gripper_{side}", top, bot, LINK_RADII["gripper_finger"]))

    return caps


# The table the arm is mounted on — a keep-out half-space y < GROUND_PLANE_Y.
# Slice 2's gate treats any capsule dipping below this (minus its radius) as a
# ground collision. The base sits at y=0, so the plane is y=0.
GROUND_PLANE_Y = 0.0


# ─── Collision layer (Slice 2) ───────────────────────────────────────────────

# Default keep-out buffer (meters). A commanded pose whose closest capsule pair
# is nearer than this is treated as colliding. OPEN VARIABLE (DESIGN.md): starts
# conservative, tightens once Slice 0 load spikes + Slice 2 cross-validation give
# real numbers. Larger = safer but loses workspace.
DEFAULT_MARGIN = 0.010

# Allowed Collision Matrix — pairs that touch BY DESIGN (share a joint / are
# mounted together), so their capsules always overlap at the pivot. Checking them
# would report permanent collision and the gate would reject every pose. Excluding
# them is what leaves only the REAL self-collisions. This is the #1 false-positive
# source in capsule self-collision. Order-independent (stored as frozensets).
_ACM: frozenset[frozenset[str]] = frozenset(
    frozenset(pair)
    for pair in (
        ("upper_arm", "forearm"),      # share the elbow
        ("forearm", "wrist"),          # share the wrist-pitch joint
        ("wrist", "gripper_left"),     # gripper mounts on the wrist
        ("wrist", "gripper_right"),
        ("gripper_left", "gripper_right"),  # jaws close together intentionally
        ("base", "upper_arm"),         # upper arm's shoulder sits atop the base
    )
)


def closest_segment_segment(
    p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray
) -> tuple[float, np.ndarray, np.ndarray]:
    """Closest distance between two 3D segments [p1,q1] and [p2,q2].

    Ericson, *Real-Time Collision Detection* §5.1.9 (ClosestPtSegmentSegment).
    Returns (distance, closest_pt_on_seg1, closest_pt_on_seg2). Handles the
    degenerate/parallel cases the naive line-line formula gets wrong.
    """
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(d1 @ d1)   # squared length of segment 1
    e = float(d2 @ d2)   # squared length of segment 2
    f = float(d2 @ r)
    eps = 1e-12

    if a <= eps and e <= eps:            # both segments are points
        s = t = 0.0
    elif a <= eps:                        # segment 1 is a point
        s = 0.0
        t = _clamp(f / e, 0.0, 1.0)
    else:
        c = float(d1 @ r)
        if e <= eps:                      # segment 2 is a point
            t = 0.0
            s = _clamp(-c / a, 0.0, 1.0)
        else:                             # general non-degenerate case
            b = float(d1 @ d2)
            denom = a * e - b * b         # >= 0
            s = _clamp((b * f - c * e) / denom, 0.0, 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clamp(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0
                s = _clamp((b - c) / a, 0.0, 1.0)

    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return float(np.linalg.norm(c1 - c2)), c1, c2


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def capsule_distance(a: Capsule, b: Capsule) -> float:
    """Signed clearance between two capsules: segment distance minus both radii.

    Negative = interpenetration. This is the quantity the gate thresholds on.
    """
    seg, _, _ = closest_segment_segment(a.p0, a.p1, b.p0, b.p1)
    return seg - a.radius - b.radius


def collisions(
    angles: dict, margin: float = DEFAULT_MARGIN, include_ground: bool = True
) -> list[tuple[str, str, float]]:
    """All colliding capsule pairs for a pose, each as (name_a, name_b, clearance).

    A pair collides when its clearance < margin. ACM-excluded (adjacent) pairs
    are skipped. Ground contact is reported as (link, "ground", clearance) where
    clearance = lowest capsule extent minus GROUND_PLANE_Y. Sorted worst-first.
    """
    caps = capsules(angles)
    hits: list[tuple[str, str, float]] = []

    for a, b in itertools.combinations(caps, 2):
        if frozenset((a.name, b.name)) in _ACM:
            continue
        clr = capsule_distance(a, b)
        if clr < margin:
            hits.append((a.name, b.name, clr))

    if include_ground:
        for c in caps:
            if c.name == "base":
                continue  # the base legitimately rests on the table
            lowest = min(c.p0[1], c.p1[1]) - c.radius
            clr = lowest - GROUND_PLANE_Y
            if clr < margin:
                hits.append((c.name, "ground", clr))

    hits.sort(key=lambda h: h[2])
    return hits


def collides(
    angles: dict, margin: float = DEFAULT_MARGIN, include_ground: bool = True
) -> tuple[bool, tuple[str, str] | None]:
    """Does this pose self-collide? Returns (colliding, worst_offending_pair).

    The gate's yes/no with the single worst pair named (for the viewer glow +
    Slice-0 cross-validation). Use `collisions()` for the full list.
    """
    hits = collisions(angles, margin, include_ground)
    if not hits:
        return False, None
    a, b, _ = hits[0]
    return True, (a, b)


def _demo() -> None:
    """Print capsule endpoints + a collision check for a couple of poses."""
    poses = {
        "horizontal forward (safe)": {"shoulder_pitch": math.pi / 2, "gripper": 1.0},
        "elbow folded back into base": {"shoulder_pitch": math.pi / 2,
                                        "elbow_pitch": math.pi},
    }
    for label, pose in poses.items():
        print(f"\n{label}:")
        for c in capsules(pose):
            print(f"  {c.name:14s} {np.round(c.p0, 4)} -> {np.round(c.p1, 4)}  r={c.radius:.4f}")
        hit, pair = collides(pose)
        print(f"  collides={hit}  worst_pair={pair}")


if __name__ == "__main__":
    _demo()
