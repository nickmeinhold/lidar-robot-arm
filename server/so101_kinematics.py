"""SO-101 forward kinematics — the ONE geometry truth, from the baked URDF.

Loads the SAME ``so101_urdf.json`` the browser viewer renders (single FK
source as a filesystem fact — crucible DESIGN §2.3) and exposes batched
forward kinematics over it. Every revolute axis in the SO-101 URDF is local
Z (pinned by ``test_urdf_bake.py``), so a joint's transform is just
``fixed_origin @ Rz(θ)`` — the whole core is numpy matrix chains.

Conventions
-----------
- **Frame**: ``urdf_calib_frame`` — angles are radians in the URDF's zero
  convention (zero = LeRobot calibration mid-pose). This is
  *convention-absolute*, not metrology (DESIGN §2.1).
- **Name bridge**: this repo's joint names map 1:1 onto URDF/LeRobot names
  (``NAME_BRIDGE``). Callers may use either; internals use URDF names.
- **Sign vector**: per-joint ±1 from the calibration commissioning record
  (DESIGN §2.2). Defaults to +1 until commissioned; the server refuses
  absolute-mode DRIVE without a commissioning record — pure FK math is fine.
- **Gripper dual-space**: humans/LLMs speak 0..1 fraction; the URDF speaks a
  revolute angle. ``gripper_fraction_to_angle`` is the ONE affine bridge,
  parameterized by the calibrated endpoints (defaults: URDF limits).
- **Interpolation is LINEAR, never circular-shortest** (Temper §10.1.4):
  every joint's legal range is a convex interval in angle space, so linear
  interpolation between two legal angles is legal at every intermediate
  point BY CONSTRUCTION. A circular shortest-path could route wrist_roll
  through its forbidden arc (its span is ~320°, not 360°); we never take it.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

BAKE_PATH = (
    Path(__file__).parent / "static" / "models" / "SO101" / "so101_urdf.json"
)

# Ours ↔ URDF/LeRobot. One table, used everywhere (DESIGN §2.1).
NAME_BRIDGE = {
    "shoulder_yaw": "shoulder_pan",
    "shoulder_pitch": "shoulder_lift",
    "elbow_pitch": "elbow_flex",
    "wrist_pitch": "wrist_flex",
    "wrist_roll": "wrist_roll",
    "gripper": "gripper",
}
URDF_TO_OURS = {v: k for k, v in NAME_BRIDGE.items()}

# The serial chain's revolute joints, base-outward. Order is load-bearing:
# angle arrays are indexed in this order.
JOINT_ORDER = (
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
)

TOOL_FRAME = "gripper_frame_link"


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF fixed-axis rpy → 3×3 rotation (Rz(y) @ Ry(p) @ Rx(r))."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _origin_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = _rpy_matrix(*rpy)
    m[:3, 3] = xyz
    return m


@dataclass(frozen=True)
class JointSpec:
    name: str
    origin: np.ndarray          # fixed 4×4 parent→joint transform
    lower: float                # URDF limit, radians
    upper: float
    child_link: str
    sign: int = 1               # calibration sign vector entry


@dataclass
class SO101Kinematics:
    """Batched FK over the baked URDF. Construct once, call ``frames``/``tool``.

    ``signs`` comes from the calibration commissioning record; ``gripper_map``
    is the calibrated (closed_angle, open_angle) pair for the fraction bridge.
    """

    bake_path: Path = BAKE_PATH
    signs: dict[str, int] = field(default_factory=dict)   # URDF names → ±1
    joints: dict[str, JointSpec] = field(init=False)
    _post: dict[str, list[tuple[str, np.ndarray]]] = field(init=False)
    gripper_map: tuple[float, float] = field(init=False)

    def __post_init__(self) -> None:
        bake = json.loads(Path(self.bake_path).read_text())
        by_name = {j["name"]: j for j in bake["joints"]}
        self.joints = {}
        for name in JOINT_ORDER:
            j = by_name[name]
            lim = j["limit"]
            lower, upper = (
                (lim["lower"], lim["upper"]) if isinstance(lim, dict)
                else (lim[0], lim[1])
            )
            self.joints[name] = JointSpec(
                name=name,
                origin=_origin_matrix(j["origin"]["xyz"], j["origin"]["rpy"]),
                lower=lower,
                upper=upper,
                child_link=j["child"],
                sign=self.signs.get(name, 1),
            )
        # Fixed joints hanging off each link (e.g. gripper_frame_joint):
        # child frames reachable without a DOF.
        self._post = {}
        for j in bake["joints"]:
            if j["type"] == "fixed":
                self._post.setdefault(j["parent"], []).append(
                    (j["child"], _origin_matrix(j["origin"]["xyz"], j["origin"]["rpy"]))
                )
        g = self.joints["gripper"]
        self.gripper_map = (g.lower, g.upper)  # closed, open — until calibrated

    # --- angle plumbing --------------------------------------------------

    def angles_array(self, angles: dict[str, float]) -> np.ndarray:
        """Dict (our names or URDF names, radians; gripper as FRACTION 0..1)
        → ordered radian array with signs applied."""
        out = np.zeros(len(JOINT_ORDER))
        for key, value in angles.items():
            name = NAME_BRIDGE.get(key, key)
            if name not in self.joints:
                raise KeyError(f"unknown joint {key!r}")
            if name == "gripper":
                value = self.gripper_fraction_to_angle(value)
            out[JOINT_ORDER.index(name)] = self.joints[name].sign * value
        return out

    def gripper_fraction_to_angle(self, fraction: float) -> float:
        closed, opened = self.gripper_map
        f = min(1.0, max(0.0, float(fraction)))
        return closed + f * (opened - closed)

    def clamp_to_limits(self, q: np.ndarray) -> np.ndarray:
        lo = np.array([self.joints[n].lower for n in JOINT_ORDER])
        hi = np.array([self.joints[n].upper for n in JOINT_ORDER])
        return np.clip(q, lo, hi)

    def limit_violations(self, q: np.ndarray) -> list[str]:
        out = []
        for i, name in enumerate(JOINT_ORDER):
            j = self.joints[name]
            if not (j.lower - 1e-9 <= q[i] <= j.upper + 1e-9):
                out.append(name)
        return out

    @staticmethod
    def interpolate(a: np.ndarray, b: np.ndarray, samples: int) -> np.ndarray:
        """(samples, 6) LINEAR interpolation a→b inclusive. Linear-in-interval
        is legal by convexity; never circular-shortest (see module docstring)."""
        t = np.linspace(0.0, 1.0, samples)[:, None]
        return a[None, :] * (1 - t) + b[None, :] * t

    # --- forward kinematics ----------------------------------------------

    def frames(self, q: np.ndarray) -> dict[str, np.ndarray]:
        """Radian array (6,) → {link_name: 4×4 world transform}."""
        out: dict[str, np.ndarray] = {"base_link": np.eye(4)}
        m = np.eye(4)
        for i, name in enumerate(JOINT_ORDER):
            spec = self.joints[name]
            c, s = math.cos(q[i]), math.sin(q[i])
            rz = np.array([
                [c, -s, 0.0, 0.0],
                [s, c, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ])
            m = m @ spec.origin @ rz
            out[spec.child_link] = m
            for child, fixed in self._post.get(spec.child_link, []):
                out[child] = m @ fixed
        return out

    def tool_position(self, q: np.ndarray) -> np.ndarray:
        """(3,) world position of the tool frame (gripper_frame_link)."""
        return self.frames(q)[TOOL_FRAME][:3, 3]

    def batch_link_origins(self, qs: np.ndarray) -> dict[str, np.ndarray]:
        """(N, 6) → {link_name: (N, 3) world positions}. Loop-free enough for
        the gate's needs at these sizes (measured ~µs/pose)."""
        names = None
        acc: dict[str, list[np.ndarray]] = {}
        for q in qs:
            fr = self.frames(q)
            if names is None:
                names = list(fr)
                acc = {n: [] for n in names}
            for n in names:
                acc[n].append(fr[n][:3, 3])
        return {n: np.array(v) for n, v in acc.items()}


_default: SO101Kinematics | None = None


def default_kinematics() -> SO101Kinematics:
    """Shared instance over the repo bake (uncommissioned signs — FK math only)."""
    global _default
    if _default is None:
        _default = SO101Kinematics()
    return _default
