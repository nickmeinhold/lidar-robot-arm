"""Bake-verification: the baked so101_urdf.json must faithfully represent the URDF.

The baked JSON is the single FK source both the browser viewer and (soon) the
Python spine consume — but its provenance is a bake script run outside this
repo's history. This test re-derives the joint tree from the URDF itself
(stdlib ElementTree, no deps) and asserts the bake matches, so a stale or
hand-edited bake can never silently diverge from the model it claims to be.
Crucible: docs/crucible/spatial-spine-motion-ir/ (DESIGN §2.3, falsifier #1).
"""
from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

MODEL_DIR = Path(__file__).parent / "static" / "models" / "SO101"
URDF_PATH = MODEL_DIR / "so101_new_calib.urdf"
BAKE_PATH = MODEL_DIR / "so101_urdf.json"


def _floats(s: str | None, default: str = "0 0 0") -> list[float]:
    return [float(x) for x in (s or default).split()]


@pytest.fixture(scope="module")
def urdf_joints() -> dict:
    root = ET.parse(URDF_PATH).getroot()
    joints = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        axis = j.find("axis")
        limit = j.find("limit")
        joints[j.get("name")] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": _floats(origin.get("xyz") if origin is not None else None),
            "rpy": _floats(origin.get("rpy") if origin is not None else None),
            "axis": _floats(axis.get("xyz") if axis is not None else None),
            "limit": (
                [float(limit.get("lower")), float(limit.get("upper"))]
                if limit is not None else None
            ),
        }
    return joints


@pytest.fixture(scope="module")
def baked() -> dict:
    return json.loads(BAKE_PATH.read_text())


def test_every_urdf_joint_is_baked(urdf_joints, baked):
    baked_names = {j["name"] for j in baked["joints"]}
    assert baked_names == set(urdf_joints), (
        f"joint set mismatch: only-in-urdf={set(urdf_joints) - baked_names}, "
        f"only-in-bake={baked_names - set(urdf_joints)}"
    )


def test_joint_tree_and_frames_match(urdf_joints, baked):
    for bj in baked["joints"]:
        uj = urdf_joints[bj["name"]]
        assert bj["type"] == uj["type"], bj["name"]
        assert bj["parent"] == uj["parent"], bj["name"]
        assert bj["child"] == uj["child"], bj["name"]
        assert bj["origin"]["xyz"] == pytest.approx(uj["xyz"], abs=1e-9), bj["name"]
        assert bj["origin"]["rpy"] == pytest.approx(uj["rpy"], abs=1e-4), bj["name"]
        assert bj["axis"] == pytest.approx(uj["axis"], abs=1e-9), bj["name"]


def test_revolute_limits_match(urdf_joints, baked):
    for bj in baked["joints"]:
        uj = urdf_joints[bj["name"]]
        if uj["type"] != "revolute":
            continue
        assert bj["limit"] is not None, f"{bj['name']} missing limit in bake"
        lower, upper = (
            (bj["limit"]["lower"], bj["limit"]["upper"])
            if isinstance(bj["limit"], dict)
            else (bj["limit"][0], bj["limit"][1])
        )
        assert lower == pytest.approx(uj["limit"][0], abs=1e-6), bj["name"]
        assert upper == pytest.approx(uj["limit"][1], abs=1e-6), bj["name"]


def test_all_revolute_axes_are_local_z(baked):
    """The FK core's `origin @ Rz(θ)` shortcut is valid ONLY because every
    revolute axis is local Z — pin that assumption where it can fail loudly."""
    for bj in baked["joints"]:
        if bj["type"] == "revolute":
            assert bj["axis"] == pytest.approx([0.0, 0.0, 1.0]), (
                f"{bj['name']}: axis {bj['axis']} breaks the Rz(θ) FK assumption"
            )


def test_mesh_files_exist(baked):
    for link in baked["links"]:
        for visual in link.get("visuals", []):
            mesh = visual.get("mesh")
            if mesh:
                assert (MODEL_DIR / "assets" / Path(mesh).name).exists(), mesh
