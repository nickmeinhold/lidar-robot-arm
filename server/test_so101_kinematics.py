"""Fixtures for the SO-101 FK core (crucible spine step 1).

Property tests pin the geometry without circular self-reference; the placo/pin
oracle (CI-only, skipped when not installed) anchors correctness against an
independent implementation wired to the same URDF (DESIGN §2.3).
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from server.so101_kinematics import (
    JOINT_ORDER,
    NAME_BRIDGE,
    TOOL_FRAME,
    SO101Kinematics,
    default_kinematics,
)

K = default_kinematics()
ZERO = np.zeros(6)


def q(**ours) -> np.ndarray:
    return K.angles_array(ours)


# --- structure ---------------------------------------------------------------

def test_chain_produces_all_link_frames():
    frames = K.frames(ZERO)
    for link in ("base_link", "shoulder_link", "upper_arm_link",
                 "lower_arm_link", "wrist_link", "gripper_link",
                 "gripper_frame_link", "moving_jaw_so101_v1_link"):
        assert link in frames


def test_limits_imported_from_urdf():
    assert K.joints["shoulder_pan"].lower == pytest.approx(-1.91986)
    assert K.joints["elbow_flex"].upper == pytest.approx(1.69)


def test_name_bridge_round_trips():
    assert set(NAME_BRIDGE.values()) == set(JOINT_ORDER)


# --- geometric properties (non-circular ground truth) ------------------------

def test_zero_pose_is_finite_and_plausible():
    p = K.tool_position(ZERO)
    assert np.all(np.isfinite(p))
    # Tool must sit within the arm's physical reach envelope (~0.5 m links).
    assert 0.02 < np.linalg.norm(p) < 0.6


def test_shoulder_pan_is_a_pure_rotation_of_the_tool():
    """Panning the base must preserve the tool's distance from the pan axis
    and its height along that axis — true for ANY correct FK of this chain."""
    frames0 = K.frames(ZERO)
    pan_origin = frames0["shoulder_link"][:3, 3]
    pan_axis = frames0["shoulder_link"][:3, :3] @ np.array([0.0, 0.0, 1.0])

    def cyl(p):
        rel = p - pan_origin
        h = rel @ pan_axis
        r = np.linalg.norm(rel - h * pan_axis)
        return h, r

    h0, r0 = cyl(K.tool_position(ZERO))
    h1, r1 = cyl(K.tool_position(q(shoulder_yaw=math.radians(37))))
    assert h1 == pytest.approx(h0, abs=1e-9)
    assert r1 == pytest.approx(r0, abs=1e-9)


def test_distal_joint_moves_only_distal_links():
    f0 = K.frames(ZERO)
    f1 = K.frames(q(wrist_pitch=math.radians(30)))
    for link in ("shoulder_link", "upper_arm_link", "lower_arm_link"):
        assert np.allclose(f0[link], f1[link]), link
    assert not np.allclose(f0[TOOL_FRAME], f1[TOOL_FRAME])


def test_rigid_link_lengths_are_pose_invariant():
    a = K.frames(q(shoulder_pitch=0.4, elbow_pitch=-0.8, wrist_pitch=0.9))
    b = K.frames(q(shoulder_yaw=-1.0, elbow_pitch=1.2))
    for parent, child in (("shoulder_link", "upper_arm_link"),
                          ("upper_arm_link", "lower_arm_link"),
                          ("lower_arm_link", "wrist_link")):
        da = np.linalg.norm(a[child][:3, 3] - a[parent][:3, 3])
        db = np.linalg.norm(b[child][:3, 3] - b[parent][:3, 3])
        assert da == pytest.approx(db, abs=1e-12), (parent, child)


# --- sign vector & gripper bridge --------------------------------------------

def test_sign_vector_mirrors_motion():
    k_neg = SO101Kinematics(signs={"shoulder_lift": -1})
    plus = K.tool_position(q(shoulder_pitch=0.3))
    minus_via_sign = k_neg.tool_position(
        k_neg.angles_array({"shoulder_pitch": 0.3}))
    direct_minus = K.tool_position(q(shoulder_pitch=-0.3))
    assert np.allclose(minus_via_sign, direct_minus)
    assert not np.allclose(minus_via_sign, plus)


def test_gripper_fraction_bridge_endpoints():
    closed, opened = K.gripper_map
    assert K.gripper_fraction_to_angle(0.0) == pytest.approx(closed)
    assert K.gripper_fraction_to_angle(1.0) == pytest.approx(opened)
    assert K.gripper_fraction_to_angle(-3) == pytest.approx(closed)  # clamps


# --- interpolation: linear, never circular (Temper §10.1.4) ------------------

def test_interpolation_never_crosses_wrist_roll_forbidden_arc():
    """wrist_roll spans ~[-157°, +163°]: the ~40° arc through ±180° is
    FORBIDDEN. Circular shortest-path from +160° to -155° would cross it;
    linear interpolation stays inside the legal interval the whole way."""
    lo, hi = K.joints["wrist_roll"].lower, K.joints["wrist_roll"].upper
    a = np.zeros(6); b = np.zeros(6)
    i = JOINT_ORDER.index("wrist_roll")
    a[i] = hi - 0.05
    b[i] = lo + 0.05
    path = K.interpolate(a, b, 64)[:, i]
    assert np.all(path >= lo - 1e-9) and np.all(path <= hi + 1e-9)
    # And it is genuinely the long way round (monotone descent), not a wrap.
    assert np.all(np.diff(path) < 0)


def test_limit_violations_reported_by_name():
    bad = np.zeros(6)
    bad[JOINT_ORDER.index("elbow_flex")] = 2.5  # beyond ±1.69
    assert K.limit_violations(bad) == ["elbow_flex"]


# --- the independent oracle (CI-only) ----------------------------------------

@pytest.mark.skipif(
    not any(__import__("importlib").util.find_spec(m) for m in ("placo", "pinocchio")),
    reason="FK oracle library not installed (CI-only check)",
)
def test_fk_agrees_with_oracle_library():
    """Tool-frame agreement < 1 mm against an independent URDF FK over a
    sampled grid. Red oracle blocks kinematics merges (DESIGN §2.3)."""
    import importlib
    urdf = str(K.bake_path.parent / "so101_new_calib.urdf")
    rng = np.random.default_rng(6)
    samples = []
    for _ in range(25):
        qi = np.array([
            rng.uniform(K.joints[n].lower, K.joints[n].upper)
            for n in JOINT_ORDER
        ])
        samples.append(qi)

    if importlib.util.find_spec("pinocchio"):
        import pinocchio as pin
        model = pin.buildModelFromUrdf(urdf)
        data = model.createData()
        fid = model.getFrameId(TOOL_FRAME)
        for qi in samples:
            pin.framesForwardKinematics(model, data, qi)
            oracle = np.array(data.oMf[fid].translation)
            ours = K.tool_position(qi)
            assert np.linalg.norm(ours - oracle) < 1e-3, qi
    else:
        import placo
        robot = placo.RobotWrapper(urdf)
        for qi in samples:
            for name, val in zip(JOINT_ORDER, qi):
                robot.set_joint(name, float(val))
            robot.update_kinematics()
            oracle = robot.get_T_world_frame(TOOL_FRAME)[:3, 3]
            ours = K.tool_position(qi)
            assert np.linalg.norm(ours - oracle) < 1e-3, qi
