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


def _demo() -> None:
    """Print capsule endpoints for the viewer's default pose (arm horizontal)."""
    default = {"shoulder_pitch": math.pi / 2, "gripper": 1.0}
    print("Default pose (shoulder_pitch=pi/2 → horizontal forward):")
    for c in capsules(default):
        print(f"  {c.name:14s} {np.round(c.p0, 4)} -> {np.round(c.p1, 4)}  r={c.radius:.4f}")


if __name__ == "__main__":
    _demo()
