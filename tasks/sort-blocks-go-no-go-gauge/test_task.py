"""Preflight regressions for an intentionally stopped, infeasible candidate.

These tests reject the recorded layout. They are not task-scoring tests.
Set GAUGE_PREFLIGHT_SCRIPT to the external original gauge_preflight.py.
"""
import importlib.util
import inspect
import itertools
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.task_file import Box
from yam_arena.urdf import forward_kinematics

HERE = Path(__file__).resolve().parent


@pytest.fixture(scope="session")
def audit():
    path = os.environ.get("GAUGE_PREFLIGHT_SCRIPT")
    assert path and Path(path).is_file(), "Set GAUGE_PREFLIGHT_SCRIPT to the external audit script"
    spec = importlib.util.spec_from_file_location("original_gauge_preflight", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def recorded():
    return json.loads((HERE / "preflight.json").read_text())


@pytest.fixture
def clean(recorded):
    """Synthetic validator input, never a claim about an achievable trajectory."""
    for data in recorded["arms"].values():
        data.update(worst_ik_mm=0.08, worst_ik_deg=0.012, joint_limits=True, collision_count=0)
    return recorded


def test_real_constructor_signatures(audit):
    for obj in audit.scene():
        assert type(obj) is Box
        inspect.signature(Box).bind(**{
            name: getattr(obj, name) for name in ("name", "pos", "size", "color", "mass", "yaw")
        })
    for points in audit.plans().values():
        for point in points:
            inspect.signature(Waypoint).bind(point.duration, point.pos, point.finger, point.label, point.yaw)


@pytest.mark.parametrize("index,width", [(0, 25), (1, 30), (2, 35)])
def test_grasp_dimensions_from_real_box_bounds(audit, index, width):
    blocks = [obj for obj in audit.scene() if obj.free]
    obj = blocks[index]
    low, high = np.array(obj.pos) - np.array(obj.size) / 2, np.array(obj.pos) + np.array(obj.size) / 2
    size = (high - low) * 1000
    assert size[1] == pytest.approx(width)
    assert size[1] < 85
    assert size[2] == pytest.approx(20)
    assert size[2] >= 15
    assert obj.mass == pytest.approx(2700 * np.prod(obj.size))


def test_spawn_has_no_positive_volume_overlap(audit):
    objects = audit.scene()
    for a, b in itertools.combinations(objects, 2):
        alo, ahi = np.array(a.pos) - np.array(a.size) / 2, np.array(a.pos) + np.array(a.size) / 2
        blo, bhi = np.array(b.pos) - np.array(b.size) / 2, np.array(b.pos) + np.array(b.size) / 2
        assert not np.all(np.minimum(ahi, bhi) - np.maximum(alo, blo) > 1e-9), (a.name, b.name)
    for obj in objects:
        if obj.free:
            assert obj.pos[2] - obj.size[2] / 2 - audit.Z == pytest.approx(0.001)


def test_only_allowed_waypoints_and_release_intent(audit):
    plans = audit.plans()
    assert set(plans) == {"left", "right"}
    for points in plans.values():
        for point in points:
            assert type(point) is Waypoint
            assert point.duration > 0
            np.testing.assert_allclose(grip_rotation(point.yaw)[:, 2], (0, 0, -1), atol=1e-9)
            assert FINGER_OPEN <= point.finger <= 0
            if point.label.endswith("release"):
                assert point.finger == FINGER_OPEN
                block, target = point.label.split()[:2]
                width = audit.WIDTHS[int(block) - 1]
                support = audit.FLOOR_TOP if target == "tray" else (
                    audit.BASE_TOP if width < audit.GAPS[target][2] else audit.ANVIL_TOP
                )
                assert point.pos[2] - audit.GRIP_Z - support == pytest.approx(0.004)


@pytest.mark.parametrize("side", ["left", "right"])
def test_real_armplan_replayed_at_20_hz(audit, recorded, side):
    plan = ArmPlan(audit.plans()[side], station.BASES[side])
    count = math.ceil(plan.duration * 20) + 1
    assert count == recorded["arms"][side]["frames"] == 1921
    for step in range(count):
        action, _ = plan.action7(step / 20)
        assert np.all(action[:6] >= ik.LOWER)
        assert np.all(action[:6] <= ik.UPPER)
    actual = recorded["arms"][side]
    assert plan.worst_ik_mm == pytest.approx(actual["worst_ik_mm"], abs=1e-7)
    assert plan.worst_ik_deg == pytest.approx(actual["worst_ik_deg"], abs=1e-7)
    assert (plan.worst_ik_mm > 0.5) == (side == "left")


def test_actual_finger_mesh_intersects_unexpanded_floor(audit):
    plan = ArmPlan(audit.plans()["left"], station.BASES["left"])
    for step in range(651):
        action, label = plan.action7(step / 20)
    assert label == "1 tray release"
    finger = min(action[6] * FINGER_OPEN, -audit.WIDTHS[0] / 2)
    joints = dict(zip(ARM_JOINTS, action[:6])) | {"joint7": finger, "joint8": finger}
    fk = forward_kinematics(audit.YAM_URDF, joints)
    floor = next(o for o in audit.scene() if o.name == "red_floor")
    low = np.array(floor.pos) - np.array(floor.size) / 2
    high = np.array(floor.pos) + np.array(floor.size) / 2
    witnesses = []
    for link, name, vertices, edges, normals in audit.meshes():
        if link == "tip_left" and name == "tip_left_014.stl":
            rotation = fk[link][:3, :3]
            world = vertices @ rotation.T + fk[link][:3, 3] + station.BASES["left"]
            assert audit.sat_intersects(world, edges @ rotation.T, normals @ rotation.T, low, high)
            assert (high[2] - world[:, 2].min()) * 1000 == pytest.approx(6.46902144, abs=1e-5)
            witnesses.append(name)
    assert witnesses == ["tip_left_014.stl"]


def test_recorded_candidate_is_rejected(audit, recorded):
    assert audit.gate_failures(recorded) == ["left IK", "left fixed-structure clearance"]
    assert recorded["arms"]["left"]["collision_count"] == 212
    assert recorded["arms"]["right"]["collision_count"] == 0


def test_validator_accepts_clean_synthetic_input(audit, clean):
    assert audit.gate_failures(clean) == []


@pytest.mark.parametrize("field,value,reason", [
    ("worst_ik_mm", 0.500001, "left IK"),
    ("worst_ik_deg", 0.100001, "left IK"),
    ("collision_count", 1, "left fixed-structure clearance"),
    ("joint_limits", False, "left joint limits"),
])
def test_each_arm_gate_rejects_independently(audit, clean, field, value, reason):
    clean["arms"]["left"][field] = value
    assert reason in audit.gate_failures(clean)


@pytest.mark.parametrize("field,value", [("width", 85), ("height", 14.999)])
def test_dimension_gate_rejects_boundary_violations(audit, clean, field, value):
    clean["grasp_dimensions_mm"][0][field] = value
    assert "grasp dimensions: block_1" in audit.gate_failures(clean)


def test_overlap_gate_rejects(audit, clean):
    clean["overlaps"] = [["block_1", "gauge_base"]]
    assert "spawn overlap" in audit.gate_failures(clean)
