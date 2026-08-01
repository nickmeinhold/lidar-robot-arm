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
    capsules,
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


def main() -> None:
    test_horizontal_forward()
    test_elbow_bend_up()
    test_all_zero_hangs_down()
    test_yaw_rotates_into_minus_x()
    test_link_lengths_are_pose_invariant()
    test_wrist_roll_moves_gripper_not_wrist()
    test_gripper_open_wider_than_closed()
    print("\nALL PASSED")


if __name__ == "__main__":
    main()
