"""Standalone tests for the arm kinematic model (Slice 1).

No pytest dependency — plain asserts, runnable directly:
    python3 -m server.test_arm_kinematics

Verifies the FK core against poses computed BY HAND from the viewer convention,
plus pose-invariants (link capsule lengths must equal the fixed link dimensions
for ANY joint angles — a strong check that catches frame/axis bugs regardless of
the specific pose).
"""
from __future__ import annotations

import math

import numpy as np

from server.arm_kinematics import (
    BASE_HEIGHT,
    FOREARM_LEN,
    UPPER_ARM_LEN,
    WRIST_LEN,
    Capsule,
    capsule_distance,
    capsules,
    closest_segment_segment,
    collides,
    collisions,
    frames,
)

TOL = 1e-9


def _caps_by_name(angles: dict) -> dict:
    return {c.name: c for c in capsules(angles)}


def _assert_close(got, want, msg: str) -> None:
    got = np.asarray(got, dtype=float)
    want = np.asarray(want, dtype=float)
    if not np.allclose(got, want, atol=TOL):
        raise AssertionError(f"{msg}\n  got  {np.round(got, 6)}\n  want {np.round(want, 6)}")


def test_horizontal_forward() -> None:
    """shoulder_pitch=pi/2, all else 0 → arm horizontal along -Z at base height."""
    c = _caps_by_name({"shoulder_pitch": math.pi / 2})
    _assert_close(c["upper_arm"].p0, [0, BASE_HEIGHT, 0], "horizontal: shoulder origin")
    _assert_close(c["upper_arm"].p1, [0, BASE_HEIGHT, -UPPER_ARM_LEN], "horizontal: elbow")
    _assert_close(c["forearm"].p1, [0, BASE_HEIGHT, -(UPPER_ARM_LEN + FOREARM_LEN)],
                  "horizontal: wrist_pitch")
    _assert_close(c["wrist"].p1,
                  [0, BASE_HEIGHT, -(UPPER_ARM_LEN + FOREARM_LEN + WRIST_LEN)],
                  "horizontal: wrist_roll")
    print("ok  test_horizontal_forward")


def test_elbow_bend_up() -> None:
    """pitch=pi/2 + elbow=pi/2 → forearm folds to point straight up (+Y)."""
    c = _caps_by_name({"shoulder_pitch": math.pi / 2, "elbow_pitch": math.pi / 2})
    _assert_close(c["forearm"].p0, [0, BASE_HEIGHT, -UPPER_ARM_LEN], "bend: elbow")
    # Forearm now runs +Y from the elbow.
    _assert_close(c["forearm"].p1, [0, BASE_HEIGHT + FOREARM_LEN, -UPPER_ARM_LEN],
                  "bend: wrist_pitch points up")
    print("ok  test_elbow_bend_up")


def test_all_zero_hangs_down() -> None:
    """All zeros → arm hangs straight down (-Y) from the shoulder."""
    c = _caps_by_name({})
    _assert_close(c["upper_arm"].p0, [0, BASE_HEIGHT, 0], "hang: shoulder origin")
    _assert_close(c["upper_arm"].p1, [0, BASE_HEIGHT - UPPER_ARM_LEN, 0], "hang: elbow")
    _assert_close(c["wrist"].p1,
                  [0, BASE_HEIGHT - UPPER_ARM_LEN - FOREARM_LEN - WRIST_LEN, 0],
                  "hang: wrist_roll")
    print("ok  test_all_zero_hangs_down")


def test_yaw_rotates_into_minus_x() -> None:
    """pitch=pi/2 + yaw=pi/2 → horizontal arm swings from -Z to -X."""
    c = _caps_by_name({"shoulder_pitch": math.pi / 2, "shoulder_yaw": math.pi / 2})
    _assert_close(c["upper_arm"].p1, [-UPPER_ARM_LEN, BASE_HEIGHT, 0],
                  "yaw: elbow rotated into -X")
    print("ok  test_yaw_rotates_into_minus_x")


def test_link_lengths_are_pose_invariant() -> None:
    """For ANY joint angles the link capsule lengths must equal the fixed link
    dims — a convention-independent invariant that catches frame/axis bugs."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        angles = {
            "shoulder_yaw": rng.uniform(-math.pi, math.pi),
            "shoulder_pitch": rng.uniform(0, math.pi),
            "elbow_pitch": rng.uniform(0, math.pi),
            "wrist_pitch": rng.uniform(-math.pi, math.pi),
            "wrist_roll": rng.uniform(-math.pi, math.pi),
            "gripper": rng.uniform(0, 1),
        }
        c = _caps_by_name(angles)
        assert abs(c["upper_arm"].length() - UPPER_ARM_LEN) < TOL, "upper arm len"
        assert abs(c["forearm"].length() - FOREARM_LEN) < TOL, "forearm len"
        assert abs(c["wrist"].length() - WRIST_LEN) < TOL, "wrist len"
    print("ok  test_link_lengths_are_pose_invariant (200 random poses)")


def test_wrist_roll_moves_gripper_not_wrist() -> None:
    """Roll twists the gripper about the wrist axis: the wrist-roll pivot origin
    is unchanged, but the finger positions move."""
    base = {"shoulder_pitch": math.pi / 2}
    rolled = {"shoulder_pitch": math.pi / 2, "wrist_roll": math.pi / 2}
    o0 = frames(base)["wrist_roll"][:3, 3]
    o1 = frames(rolled)["wrist_roll"][:3, 3]
    _assert_close(o0, o1, "roll must not move the wrist-roll origin")

    lf0 = _caps_by_name(base)["gripper_left"].p0
    lf1 = _caps_by_name(rolled)["gripper_left"].p0
    assert not np.allclose(lf0, lf1, atol=1e-4), "roll must move the fingers"
    print("ok  test_wrist_roll_moves_gripper_not_wrist")


def test_gripper_open_wider_than_closed() -> None:
    """Fingers sit farther apart open than closed."""
    opened = _caps_by_name({"shoulder_pitch": math.pi / 2, "gripper": 1.0})
    closed = _caps_by_name({"shoulder_pitch": math.pi / 2, "gripper": 0.0})
    sep_open = np.linalg.norm(opened["gripper_left"].p0 - opened["gripper_right"].p0)
    sep_closed = np.linalg.norm(closed["gripper_left"].p0 - closed["gripper_right"].p0)
    assert sep_open > sep_closed, f"open {sep_open} should exceed closed {sep_closed}"
    print("ok  test_gripper_open_wider_than_closed")


# ─── Collision layer (Slice 2) ───────────────────────────────────────────────

def _v(*xs) -> np.ndarray:
    return np.array(xs, dtype=float)


def test_segseg_parallel_offset() -> None:
    """Two parallel unit segments along Z, offset 0.1 in X → distance 0.1."""
    d, _, _ = closest_segment_segment(_v(0, 0, 0), _v(0, 0, 1),
                                      _v(0.1, 0, 0), _v(0.1, 0, 1))
    assert abs(d - 0.1) < TOL, f"parallel offset dist {d}"
    print("ok  test_segseg_parallel_offset")


def test_segseg_crossing_skew() -> None:
    """A seg along X and a seg along Z, planes 0.05 apart in Y, crossing over the
    origin → closest approach 0.05."""
    d, _, _ = closest_segment_segment(_v(-1, 0, 0), _v(1, 0, 0),
                                      _v(0, 0.05, -1), _v(0, 0.05, 1))
    assert abs(d - 0.05) < TOL, f"skew crossing dist {d}"
    print("ok  test_segseg_crossing_skew")


def test_segseg_endpoint_clamping() -> None:
    """Non-overlapping colinear segments → distance is the gap between endpoints
    (the clamp path Ericson gets right and the line-line formula does not)."""
    d, _, _ = closest_segment_segment(_v(0, 0, 0), _v(0, 0, 1),
                                      _v(0, 0, 3), _v(0, 0, 5))
    assert abs(d - 2.0) < TOL, f"colinear gap dist {d}"
    print("ok  test_segseg_endpoint_clamping")


def test_capsule_distance_signs() -> None:
    """Clearance = segment distance minus both radii; negative when interpenetrating."""
    a = Capsule("a", _v(0, 0, 0), _v(0, 0, 1), 0.02)
    far = Capsule("b", _v(0.1, 0, 0), _v(0.1, 0, 1), 0.02)   # seg dist 0.1
    near = Capsule("c", _v(0.03, 0, 0), _v(0.03, 0, 1), 0.02)  # seg dist 0.03
    assert abs(capsule_distance(a, far) - (0.1 - 0.04)) < TOL, "far clearance"
    assert capsule_distance(a, near) < 0, "overlapping radii → negative clearance"
    print("ok  test_capsule_distance_signs")


def test_safe_pose_no_collision() -> None:
    """The extended horizontal pose is collision-free."""
    hit, pair = collides({"shoulder_pitch": math.pi / 2, "gripper": 1.0})
    assert not hit, f"extended pose should be safe, got {pair}"
    print("ok  test_safe_pose_no_collision")


def test_folded_pose_collides() -> None:
    """Fully folding the elbow swings the forearm back over the base → collision."""
    hit, pair = collides({"shoulder_pitch": math.pi / 2, "elbow_pitch": math.pi})
    assert hit, "folded-into-base pose should collide"
    print(f"ok  test_folded_pose_collides (pair={pair})")


def test_acm_excludes_adjacent_pairs() -> None:
    """No collision report ever names an ACM-excluded adjacent pair, even in a
    folded pose where they overlap most."""
    excluded = {
        frozenset(("upper_arm", "forearm")),
        frozenset(("forearm", "wrist")),
        frozenset(("wrist", "gripper_left")),
        frozenset(("wrist", "gripper_right")),
        frozenset(("gripper_left", "gripper_right")),
        frozenset(("base", "upper_arm")),
    }
    for pose in ({}, {"shoulder_pitch": math.pi / 2, "elbow_pitch": math.pi},
                 {"elbow_pitch": math.pi, "wrist_pitch": math.pi}):
        for a, b, _ in collisions(pose):
            assert frozenset((a, b)) not in excluded, f"ACM leak: {a}+{b} in {pose}"
    print("ok  test_acm_excludes_adjacent_pairs")


def test_ground_collision_reported() -> None:
    """A pose driving a link below the table reports a ground collision."""
    # All-zero hangs straight down through the table (y goes strongly negative).
    hits = collisions({})
    assert any(b == "ground" for _, b, _ in hits), f"expected ground hit, got {hits}"
    print("ok  test_ground_collision_reported")


def main() -> None:
    test_horizontal_forward()
    test_elbow_bend_up()
    test_all_zero_hangs_down()
    test_yaw_rotates_into_minus_x()
    test_link_lengths_are_pose_invariant()
    test_wrist_roll_moves_gripper_not_wrist()
    test_gripper_open_wider_than_closed()
    test_segseg_parallel_offset()
    test_segseg_crossing_skew()
    test_segseg_endpoint_clamping()
    test_capsule_distance_signs()
    test_safe_pose_no_collision()
    test_folded_pose_collides()
    test_acm_excludes_adjacent_pairs()
    test_ground_collision_reported()
    print("\nALL PASSED")


if __name__ == "__main__":
    main()
