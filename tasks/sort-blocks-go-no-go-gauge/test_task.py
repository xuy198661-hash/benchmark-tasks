"""Real-framework feasibility regressions for the single revised layout."""
import importlib.util
import inspect
import itertools
import json
import math
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena import YAM_URDF
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY, TCP_OFFSET
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.task_file import Box
from yam_arena.urdf import forward_kinematics, rpy_matrix

HERE = Path(__file__).resolve().parent


def robot_collision_meshes():
    """Test-only mesh reader; no asset generation and no framework substitute."""
    result = []
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        for collision in link.findall('collision'):
            origin = collision.find('origin')
            transform = np.eye(4)
            if origin is not None:
                transform[:3, :3] = rpy_matrix(np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))
                transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            mesh = collision.find('geometry/mesh')
            path = YAM_URDF.parent/mesh.get('filename')
            data = path.read_bytes(); count = struct.unpack('<I', data[80:84])[0]
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
            tri = np.frombuffer(data, dtype=dtype, offset=84, count=count)['vertices'].astype(float)
            tri = tri*np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            tri = tri@transform[:3, :3].T+transform[:3, 3]
            vertices = np.unique(tri.reshape(-1, 3), axis=0)
            edges = np.concatenate((tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 1], tri[:, 0]-tri[:, 2]))
            normals = np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0])
            result.append((link.get('name'), path.name, vertices, edges, normals))
    return result


def convex_box_intersection(vertices, edges, normals, low, high):
    centre, half = (high+low)/2, (high-low)/2
    axes = np.concatenate((np.eye(3), normals, *[np.cross(edges, a) for a in np.eye(3)]))
    lengths = np.linalg.norm(axes, axis=1)
    axes = axes[lengths > 1e-10]/lengths[lengths > 1e-10, None]
    axes = np.unique(np.round(axes, 8), axis=0)
    projection = (vertices-centre)@axes.T
    radius = np.abs(axes)@half
    return bool(np.all(projection.min(axis=0) <= radius) and np.all(projection.max(axis=0) >= -radius))


def portable_audit_helpers():
    """Use the committed scene and real package when no external audit is set."""
    module = load(HERE/'task.py').module
    rows = []
    geom = robot_collision_meshes()
    for finger in (FINGER_OPEN, 0., *[-w/2 for w in module.WIDTHS]):
        fk = forward_kinematics(YAM_URDF, dict(zip(ARM_JOINTS, READY)) | {'joint7': finger, 'joint8': finger})
        inverse = np.linalg.inv(fk['gripper'])
        vertices = []
        for link, name, v, edges, normals in geom:
            if link in ('tip_left', 'tip_right'):
                t = inverse@fk[link]
                vertices.append(v@t[:3, :3].T+t[:3, 3])
        rows.append(float(np.concatenate(vertices)[:, 2].max()-TCP_OFFSET[2]))
    measurement = {'d_m': max(rows)}
    assert measurement['d_m'] == pytest.approx(module.GEOMETRY['tip_measurement']['d_m'], abs=1e-12)
    def failures(result):
        found = []
        for row in result['grasp_dimensions_mm']:
            if not (row['width'] < 85 and row['height'] >= 15):
                found.append('grasp dimensions: '+row['name'])
        if result['overlaps']:
            found.append('spawn overlap')
        for side, data in result['arms'].items():
            if data['worst_ik_mm'] >= .5 or data['worst_ik_deg'] >= .1:
                found.append(side+' IK')
            if not data['joint_limits']:
                found.append(side+' joint limits')
            if data['collision_count']:
                found.append(side+' fixed-structure clearance')
        return found
    return SimpleNamespace(scene=lambda: module.OBJECTS, plans=module.demo, GRIP_Z=module.GRIP_Z,
                           TIP_MEASUREMENT=measurement, WIDTHS=module.WIDTHS, Z=module.TABLE_Z,
                           GRIP_DX=module.GRIP_DX, GAPS=module.GAPS, BASE_TOP=module.BASE_TOP,
                           ANVIL_TOP=module.ANVIL_TOP, FLOOR_TOP=module.FLOOR_TOP, FINGER_OPEN=FINGER_OPEN,
                           YAM_URDF=YAM_URDF, ARM_JOINTS=ARM_JOINTS, meshes=lambda: geom,
                           sat_intersects=convex_box_intersection, gate_failures=failures)


@pytest.fixture(scope="session")
def audit():
    path = os.environ.get("GAUGE_PREFLIGHT_SCRIPT")
    if not path:
        return portable_audit_helpers()
    assert Path(path).is_file(), "GAUGE_PREFLIGHT_SCRIPT does not exist"
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
    assert size[2] == pytest.approx(40)
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
    assert audit.GRIP_Z - audit.TIP_MEASUREMENT['d_m'] >= .003
    assert any('handoff source clear' in w.label for ws in plans.values() for w in ws)
    for side, points in plans.items():
        for point in points:
            assert type(point) is Waypoint
            assert point.duration > 0
            np.testing.assert_allclose(grip_rotation(point.yaw)[:, 2], (0, 0, -1), atol=1e-9)
            assert FINGER_OPEN <= point.finger <= 0
            if ' tray ' in point.label or ' start ' in point.label:
                assert point.pos[1] * (1 if side == 'left' else -1) > 0
            if point.label.endswith("release"):
                assert point.finger == FINGER_OPEN
                block, target = point.label.split()[:2]
                width = audit.WIDTHS[int(block) - 1]
                support = audit.FLOOR_TOP if target == "tray" else (
                    audit.BASE_TOP if width < audit.GAPS[target][2] else audit.ANVIL_TOP
                )
                assert point.pos[2] - audit.GRIP_Z - support == pytest.approx(0.001)
                assert 0 <= point.pos[2] - audit.GRIP_Z - support < .002


@pytest.mark.parametrize("side", ["left", "right"])
def test_real_armplan_replayed_at_20_hz(audit, recorded, side):
    plan = ArmPlan(audit.plans()[side], station.BASES[side])
    count = math.ceil(plan.duration * 20) + 1
    assert count == recorded["arms"][side]["frames"] == 2925
    for step in range(count):
        action, _ = plan.action7(step / 20)
        assert np.all(action[:6] >= ik.LOWER)
        assert np.all(action[:6] <= ik.UPPER)
    actual = recorded["arms"][side]
    assert plan.worst_ik_mm == pytest.approx(actual["worst_ik_mm"], abs=1e-7)
    assert plan.worst_ik_deg == pytest.approx(actual["worst_ik_deg"], abs=1e-7)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1


def test_actual_finger_mesh_clears_expanded_fixed_structures(audit):
    plan = ArmPlan(audit.plans()["left"], station.BASES["left"])
    for step in range(1000):
        action, label = plan.action7(step / 20)
        if label == '1 go close':
            break
    assert label == "1 go close"
    finger = min(action[6] * FINGER_OPEN, -audit.WIDTHS[0] / 2)
    joints = dict(zip(ARM_JOINTS, action[:6])) | {"joint7": finger, "joint8": finger}
    fk = forward_kinematics(audit.YAM_URDF, joints)
    witnesses = []
    for link, name, vertices, edges, normals in audit.meshes():
        if link in ('tip_left', 'tip_right'):
            rotation = fk[link][:3, :3]
            world = vertices @ rotation.T + fk[link][:3, 3] + station.BASES["left"]
            for fixture in audit.scene():
                if fixture.free:
                    continue
                low = np.array(fixture.pos) - np.array(fixture.size) / 2 - .001
                high = np.array(fixture.pos) + np.array(fixture.size) / 2 + .001
                assert not audit.sat_intersects(world, edges @ rotation.T, normals @ rotation.T, low, high)
            assert world[:, 2].min() - audit.BASE_TOP >= .003
            witnesses.append(name)
    assert len(witnesses) == 92


def test_recorded_candidate_passes(audit, recorded):
    assert audit.gate_failures(recorded) == []
    assert recorded["arms"]["left"]["collision_count"] == 0
    assert recorded["arms"]["right"]["collision_count"] == 0
    assert all(a['min_aabb_clearance_m'] >= .003 for a in recorded['arms'].values())


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


# Full task scorer tests use real ObjectState and the task-file loader. Only the
# public gripper sensor boundary is replaced; no embodiment method is changed.
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix
import copy
import contextlib
import io
import subprocess
import sys


@pytest.fixture
def task(monkeypatch):
    path = Path(os.environ.get('GAUGE_TASK_SOURCE', HERE / 'task.py'))
    module = load(path).module
    far = [{'tcp': np.array((.1, .45, 1.1)), 'gap': .0939, 'axis': np.array((0., 1., 0.))},
           {'tcp': np.array((.1, -.45, 1.1)), 'gap': .0939, 'axis': np.array((0., 1., 0.))}]
    monkeypatch.setattr(module, 'read_grippers', lambda: (copy.deepcopy(far), 'test public sensor'))
    return module


def state(spec, pos=None, rotation=None, speed=0., spin=0.):
    pos = np.asarray(spec.pos if pos is None else pos, dtype=float)
    rotation = np.eye(3) if rotation is None else rotation
    half = np.abs(rotation) @ np.asarray(spec.size) / 2
    return ObjectState(pos.copy(), ik.matrix_to_quat_xyzw(rotation), np.array((speed, 0., 0.)),
                       np.array((0., spin, 0.)), pos-half, pos+half)


def initial(task):
    return {obj.name: state(obj) for obj in task.OBJECTS}


def tick(task, objects, count=12):
    return [task.success(objects) for _ in range(count)]


def put(task, objects, name, xyz, rotation=None, speed=0., spin=0.):
    objects[name] = state(task.SPECS[name], xyz, rotation, speed, spin)


def gauge_pose(task, objects, name, gauge, extra=0., held=False, rotation=None):
    x, y, gap = task.GAPS[gauge]
    top = task.BASE_TOP if task.SPECS[name].size[1] < gap else task.ANVIL_TOP
    put(task, objects, name, (x, y, top+task.HEIGHT/2+extra), rotation)


def inspect_block(task, objects, name):
    gauge_pose(task, objects, name, 'go')
    tick(task, objects, 20)
    if task.SPECS[name].size[1] < task.GAPS['go'][2]:
        gauge_pose(task, objects, name, 'no_go')
        tick(task, objects, 20)


def tray_pose(task, objects, name, tray=None, extra=0.):
    tray = task._expected(name) if tray is None else tray
    x, y = task.TRAYS[tray]
    if tray == 'red':
        y += -.04 if name == 'block_1' else .04
    put(task, objects, name, (x, y, task.FLOOR_TOP+task.HEIGHT/2+extra))


def finish_block(task, objects, name):
    inspect_block(task, objects, name)
    tray_pose(task, objects, name)
    tick(task, objects, 20)


def test_task_file_contract_and_no_eager_isaac_import():
    code = "from yam_arena.task_file import load; import sys; t=load(sys.argv[1]); assert t.score_over=='end'; assert t.success_hold_s==1.; assert not any(n.startswith(('isaaclab','isaacsim','omni')) for n in sys.modules)"
    result = subprocess.run([sys.executable, '-c', code, str(HERE/'task.py')], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_complete_task_demo_matches_audited_plan(task, audit):
    for name in task.BLOCKS:
        inspect.signature(Box).bind(**{key: getattr(task.SPECS[name], key)
                                      for key in ('name', 'pos', 'size', 'mass', 'color', 'yaw')})
    for side in ('left', 'right'):
        a, b = task.demo()[side], audit.plans()[side]
        assert len(a) == len(b)
        for x, y in zip(a, b):
            assert (x.duration, x.finger, x.label, x.yaw) == (y.duration, y.finger, y.label, y.yaw)
            np.testing.assert_allclose(x.pos, y.pos, atol=1e-12)
        assert sum(w.duration for w in a)*20 + 20 <= task.RUN_LIMIT


def test_all_final_score_tiers_and_success_hold(task):
    objects = initial(task)
    tick(task, objects)
    assert task.score(objects) == 0
    for i, name in enumerate(task.BLOCKS, 1):
        finish_block(task, objects, name)
        assert task.score(objects) == [.25, .5, .75][i-1]
        assert task.distance(objects) == 3-i
    assert all(tick(task, objects, 20))
    assert task.score(objects) == .75  # only framework success awards 1.0


@pytest.mark.parametrize('name,gauge', [('block_1', 'no_go'), ('block_2', 'no_go'), ('block_3', 'go')])
def test_hover_three_tenths_mm_never_inspects(task, name, gauge):
    objects = initial(task)
    tick(task, objects)
    if gauge == 'no_go':
        gauge_pose(task, objects, name, 'go'); tick(task, objects, 20)
    gauge_pose(task, objects, name, gauge, extra=.0003)
    tick(task, objects, 30)
    assert task._checks[name][gauge] is None
    assert task._gauge_reading(name, gauge, objects) is None
    assert task.score(objects) == 0


@pytest.mark.parametrize('side', [0, 1])
def test_clamped_block_exactly_at_gauge_height_never_inspects(task, monkeypatch, side):
    objects = initial(task); tick(task, objects)
    gauge_pose(task, objects, 'block_3', 'go')
    far, _ = task.read_grippers()
    far[side] = {'tcp': objects['block_3'].pos+np.array((task.GRIP_DX, 0., .005)),
                 'gap': .035, 'axis': np.array((0., 1., 0.))}
    monkeypatch.setattr(task, 'read_grippers', lambda: (far, 'test held'))
    tick(task, objects, 30)
    assert task._checks['block_3']['go'] is None
    assert task.score(objects) == 0


@pytest.mark.parametrize('angle', [3., 8., -8.])
def test_tilted_midair_wedge_is_no_contact(task, angle):
    objects = initial(task); tick(task, objects)
    r = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(angle))
    gauge_pose(task, objects, 'block_3', 'go', extra=.003, rotation=r)
    tick(task, objects, 30)
    assert task._checks['block_3']['go'] is None


@pytest.mark.parametrize('extra,valid', [(.0003, False), (.000001, False), (0., True), (-.000199, True), (-.000201, False)])
def test_signed_support_boundary(task, extra, valid):
    objects = initial(task); gauge_pose(task, objects, 'block_3', 'go', extra=extra)
    assert (task._gauge_reading('block_3', 'go', objects) == 'blocked') == valid


def test_gauge_support_requires_both_anvils_and_com(task):
    objects = initial(task); gauge_pose(task, objects, 'block_3', 'go')
    assert task._gauge_reading('block_3', 'go', objects) == 'blocked'
    s = task.SPECS['go_anvil_2']; objects[s.name] = state(s, np.array(s.pos)+(0, .03, 0))
    assert task._gauge_reading('block_3', 'go', objects) is None
    # A tiny edge contact with COM outside that single support is not support.
    objects = initial(task); gauge_pose(task, objects, 'block_3', 'go')
    assert not task._supported(objects['block_3'], task.SPECS['block_3'].size,
                               [task._surface('go_anvil_0', objects)])


def test_no_go_before_go_and_short_dwell_do_not_count(task):
    objects = initial(task); tick(task, objects)
    gauge_pose(task, objects, 'block_2', 'no_go'); tick(task, objects, 20)
    assert task._checks['block_2']['no_go'] is None
    gauge_pose(task, objects, 'block_2', 'go')
    # Let the 10-position window fill; then six samples still span only 0.25 s.
    tick(task, objects, 9)
    assert task._checks['block_2']['go'] is None
    task.success(objects)
    assert task._checks['block_2']['go'] is None
    tick(task, objects, 5)
    assert task._checks['block_2']['go'] is None
    task.success(objects)
    assert task._checks['block_2']['go'] == 'seated'


@pytest.mark.parametrize('event', ['impact', 'off_table'])
def test_irreversible_events_stay_zero_after_slow_restoration(task, event):
    objects = initial(task); tick(task, objects)
    if event == 'tip':
        r = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(25))
        put(task, objects, 'block_1', task.SPECS['block_1'].pos, r)
    elif event == 'impact':
        put(task, objects, 'block_1', task.SPECS['block_1'].pos, speed=1.2)
    elif event == 'off_table':
        put(task, objects, 'block_1', (.6, .6, task.TABLE_Z-.1))
    elif event == 'fixed_moved':
        spec = task.SPECS['gauge_base']; objects[spec.name] = state(spec, np.array(spec.pos)+(0, 0, .003))
    else:
        if event == 'wrong_tray':
            inspect_block(task, objects, 'block_1')
            tray_pose(task, objects, 'block_1', 'green')
        else:
            tray_pose(task, objects, 'block_1')
        tick(task, objects, 20)
    task.success(objects)
    assert task._failed
    # Slow return makes every per-frame displacement less than 0.10 m.
    target = initial(task)
    start = {n: np.array(s.pos) for n, s in objects.items()}
    for frac in np.linspace(0., 1., 101):
        for obj in task.OBJECTS:
            objects[obj.name] = state(obj, start[obj.name]+frac*(target[obj.name].pos-start[obj.name]))
        task.success(objects)
    assert task._failed and task.score(objects) == 0
    for name in task.BLOCKS:
        finish_block(task, objects, name)
    assert task.score(objects) == 0 and not task.success(objects)


def test_jump_direct_to_final_state_and_wrong_start_fail_closed(task):
    objects = initial(task)
    for name in task.BLOCKS:
        tray_pose(task, objects, name)
    tick(task, objects, 30)
    assert not task._started and task._invalid['scene'] == 'invalid initial layout'
    assert task.score(objects) == 0 and not task.success(objects)


def test_real_episode_jump_resets_but_returning_slowly_does_not(task):
    objects = initial(task); tick(task, objects)
    finish_block(task, objects, 'block_1')
    assert task.score(objects) == .25
    # This single-step >0.10 m jump, and only a full nominal layout, resets history.
    objects = initial(task); task.success(objects)
    assert not task._failed
    assert task._step == 1
    assert all(c['go'] is None for c in task._checks.values())
    assert task.score(objects) == 0


def test_yaw_180_is_not_tilt_and_can_be_inspected(task):
    objects = initial(task); tick(task, objects)
    yaw = axis_angle_matrix(np.array((0., 0., 1.)), math.pi)
    gauge_pose(task, objects, 'block_3', 'go', rotation=yaw)
    tick(task, objects, 20)
    assert not task._failed and task._checks['block_3']['go'] == 'blocked'


@pytest.mark.parametrize('extra,credited', [(.0003, False), (0., True)])
def test_tray_support_boundary_and_held_final_pose(task, monkeypatch, extra, credited):
    objects = initial(task); tick(task, objects); inspect_block(task, objects, 'block_1')
    tray_pose(task, objects, 'block_1', extra=extra); tick(task, objects, 20)
    assert (task.score(objects) == .25) == credited
    grips, _ = task.read_grippers()
    grips[1] = {'tcp': objects['block_1'].pos, 'gap': .025, 'axis': np.array((0., 1., 0.))}
    monkeypatch.setattr(task, 'read_grippers', lambda: (grips, 'test held final'))
    task.success(objects)
    assert task.score(objects) == 0 and not task.success(objects)


def test_rim_wrong_footprint_and_unsupported_stack_are_not_tray_placements(task):
    objects = initial(task); tick(task, objects); inspect_block(task, objects, 'block_1')
    tray_pose(task, objects, 'block_1', extra=task.HEIGHT)
    tick(task, objects, 20)
    assert task.score(objects) == 0
    p = objects['block_1'].pos.copy(); p[0] += .06; p[2] = task.TABLE_Z+.014+task.HEIGHT/2
    put(task, objects, 'block_1', p); tick(task, objects, 20)
    assert task.score(objects) == 0


@pytest.mark.parametrize('gap,allowed', [(.039999, False), (.040001, True)])
def test_gripper_opening_threshold(task, monkeypatch, gap, allowed):
    objects = initial(task); tick(task, objects); gauge_pose(task, objects, 'block_3', 'go')
    grips, _ = task.read_grippers()
    grips[0] = {'tcp': objects['block_3'].pos, 'gap': gap, 'axis': np.array((0., 1., 0.))}
    monkeypatch.setattr(task, 'read_grippers', lambda: (grips, 'threshold'))
    tick(task, objects, 20)
    assert (task._checks['block_3']['go'] == 'blocked') == allowed


def test_gripper_unavailable_fails_closed_and_debug_cadence(task, monkeypatch, capsys):
    monkeypatch.setenv('TASK_DEBUG', '1')
    monkeypatch.setattr(task, 'read_grippers', lambda: (None, 'unavailable test'))
    objects = initial(task); tick(task, objects, 41)
    lines = capsys.readouterr().out.splitlines()
    rows = [json.loads(line.split(' ', 1)[1]) for line in lines]
    assert [r['step'] for r in rows] == [1, 20, 40]
    assert all(r['gripper'] == 'unavailable test' for r in rows)
    gauge_pose(task, objects, 'block_3', 'go'); tick(task, objects, 30)
    assert task._checks['block_3']['go'] is None


def test_score_and_distance_are_read_only_including_arrays(task):
    objects = initial(task); tick(task, objects); finish_block(task, objects, 'block_1')
    def fingerprint():
        return repr((task._step, task._failed, task._checks, task._dwell,
                     {n: list(w) for n, w in task._windows.items()}, task._last_positions,
                     [(n, s.pos, s.low, s.high) for n, s in objects.items()]))
    before = fingerprint()
    for _ in range(5):
        assert task.score(objects) == .25
        assert task.distance(objects) == 2
    assert fingerprint() == before


def test_rest_position_window_overrides_frozen_velocity_and_detects_motion(task):
    objects = initial(task); tick(task, objects)
    name = 'block_3'; p = objects[name].pos.copy()
    # High stale velocity cannot prevent a full, stationary position window.
    put(task, objects, name, p, speed=.02, spin=.6); tick(task, objects, 12)
    assert task._still(name, objects)
    # Zero speed cannot hide motion once the window is full.
    for i in range(12):
        put(task, objects, name, p+np.array((i*.0003, 0., 0.))); task.success(objects)
    assert not task._still(name, objects)


def test_fifteen_degree_tilt_threshold_and_final_state_recheck(task):
    objects = initial(task); tick(task, objects)
    name = 'block_1'
    r = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(14.99))
    put(task, objects, name, np.array(task.SPECS[name].pos)+(0, 0, .05), r); task.success(objects)
    assert not task._failed
    r = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(15.01))
    put(task, objects, name, np.array(task.SPECS[name].pos)+(0, 0, .05), r); task.success(objects)
    assert not task._failed and 'block tipped' in task._invalid[name]


def test_public_reader_with_fake_public_physics_api(monkeypatch):
    module = load(os.environ.get('GAUGE_TASK_SOURCE', HERE/'task.py')).module
    class View:
        count = 1
        shared_metatype = SimpleNamespace(dof_names=['joint7', 'joint8'], link_names=['gripper', 'tip_left', 'tip_right'])
        def get_dof_positions(self):
            return np.array([[-.04695, -.04695]])
        def get_link_transforms(self):
            return np.array([[[.2, .1, 1., 0., 0., 0., 1.]]*3])
    paths = []
    def create(path):
        paths.append(path); return View()
    context = SimpleNamespace(instance=lambda: SimpleNamespace(physics_sim_view=SimpleNamespace(create_articulation_view=create)))
    monkeypatch.setitem(sys.modules, 'isaaclab', SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'isaaclab.sim', SimpleNamespace(SimulationContext=context))
    readings, reason = module.read_grippers()
    assert reason == 'ok' and len(readings) == 2
    assert paths == ['/World/envs/env_0/LeftArm', '/World/envs/env_0/RightArm']
    assert all(g['gap'] == pytest.approx(.0939) for g in readings)
    np.testing.assert_allclose(readings[0]['tcp'], [.2, .1, 1.134])


def test_public_reader_without_simulator_returns_reason():
    module = load(os.environ.get('GAUGE_TASK_SOURCE', HERE/'task.py')).module
    readings, reason = module.read_grippers()
    assert readings is None and reason


@pytest.mark.parametrize('offset,still', [(.001999, True), (.002001, False)])
def test_position_window_threshold(task, offset, still):
    objects = initial(task); tick(task, objects)
    name = 'block_3'; p = objects[name].pos.copy()
    put(task, objects, name, p+np.array((offset, 0., 0.))); task.success(objects)
    assert task._still(name, objects) == still


@pytest.mark.parametrize('offset,valid', [(.003999, True), (.004001, False)])
def test_initial_layout_radius_boundary(task, offset, valid):
    objects = initial(task); p = objects['block_1'].pos.copy(); p[0] += offset
    put(task, objects, 'block_1', p); task.success(objects)
    assert task._started == valid


@pytest.mark.parametrize('missing', [True, False])
def test_missing_or_changed_object_fails_closed(task, missing):
    objects = initial(task); tick(task, objects)
    if missing:
        del objects['block_1']
    else:
        old = objects['block_1']
        objects['block_1'] = ObjectState(old.pos, old.quat_xyzw, old.lin_vel, old.ang_vel, old.low, old.high+.01)
    assert not task.success(objects) and task.score(objects) == 0
    assert task._invalid and not task._failed


def test_debug_output_does_not_change_scores(task, monkeypatch):
    objects = initial(task); monkeypatch.setenv('TASK_DEBUG', '1')
    with contextlib.redirect_stdout(io.StringIO()):
        tick(task, objects)
        for name in task.BLOCKS:
            finish_block(task, objects, name)
    assert task.score(objects) == .75 and task.success(objects)


def test_full_demo_logic_20_hz_with_ideal_settling_and_interarm_clearance(task, audit, monkeypatch):
    """Kinematic replay with explicit idealised settling, NOT a physics result."""
    objects = initial(task)
    plans = {s: ArmPlan(task.demo()[s], station.BASES[s]) for s in ('left', 'right')}
    attached = {'left': None, 'right': None}
    sensor = []
    monkeypatch.setattr(task, 'read_grippers', lambda: (sensor, 'ideal replay'))
    # Aggregate each link's real collision meshes. Disjoint link AABBs give a
    # conservative clearance lower bound; no point-only gripper approximation.
    geom = {}
    for link, mesh, vertices, edges, normals in audit.meshes():
        geom.setdefault(link, []).append(vertices)
    geom = {n: np.concatenate(v) for n, v in geom.items()}
    fixtures = []
    for obj in task.OBJECTS:
        if not obj.free:
            fixtures.append((obj.name, np.array(obj.pos)-np.array(obj.size)/2,
                             np.array(obj.pos)+np.array(obj.size)/2))
    fixtures.append(('table', np.array(station.TABLE_CENTER)-np.array(station.TABLE_SIZE)/2,
                     np.array(station.TABLE_CENTER)+np.array(station.TABLE_SIZE)/2))
    corners = np.array(list(itertools.product((-1, 1), repeat=3)))
    for name, (kind, size, pos, quat) in station.camera_stand(station.top_camera()).items():
        half = np.asarray(size)/2 if kind == 'box' else np.array((size[0], size[0], size[1]/2+size[0]))
        vertices = (corners*half)@quat_xyzw_to_matrix(np.array(quat)).T+pos
        fixtures.append((name, vertices.min(axis=0), vertices.max(axis=0)))
    flow = np.array([f[1] for f in fixtures])-.003
    fhigh = np.array([f[2] for f in fixtures])+.003
    hold = 0
    minimum = float('inf')
    completion = None
    for step in range(1, round(max(p.duration for p in plans.values())*20)+41):
        sensor.clear()
        link_boxes = []
        for side, plan in plans.items():
            action, label = plan.action7(step/20)
            tcp, rotation = ik.tcp_pose(action[:6], station.BASES[side])
            tokens = label.split()
            name = 'block_'+tokens[0] if tokens and tokens[0].isdigit() else None
            gap = -2*action[6]*FINGER_OPEN
            if name and tokens[-1] == 'close' and gap <= task.SPECS[name].size[1]:
                attached[side] = name
            if attached[side]:
                held = attached[side]
                gap = max(gap, task.SPECS[held].size[1])
                if name == held and tokens[-1] == 'release' and gap >= task.SPECS[held].size[1]+.005:
                    where = tokens[1]
                    if where == 'tray':
                        tray_pose(task, objects, held)
                    else:
                        gauge_pose(task, objects, held, where)
                    attached[side] = None
                else:
                    centre = tcp+np.array((-task.GRIP_DX, 0., task.HEIGHT/2-task.GRIP_Z))
                    put(task, objects, held, centre)
            finger = -gap/2
            joints = dict(zip(ARM_JOINTS, action[:6])) | {'joint7': finger, 'joint8': finger}
            fk = forward_kinematics(audit.YAM_URDF, joints)
            boxes = []
            for link, vertices in geom.items():
                matrix = fk[link]
                world = vertices@matrix[:3, :3].T+matrix[:3, 3]+station.BASES[side]
                boxes.append((world.min(axis=0), world.max(axis=0)))
                if link != 'base':
                    delta = np.maximum(np.maximum(flow-world.max(axis=0), world.min(axis=0)-fhigh), 0)
                    near = np.flatnonzero(np.linalg.norm(delta, axis=1) == 0)
                    for j in near:
                        for hull_link, mesh, hull, edges, normals in audit.meshes():
                            if hull_link == link:
                                hull_world = hull@matrix[:3, :3].T+matrix[:3, 3]+station.BASES[side]
                                if np.any(hull_world.max(axis=0) < flow[j]) or np.any(hull_world.min(axis=0) > fhigh[j]):
                                    continue
                                assert not audit.sat_intersects(hull_world, edges@matrix[:3, :3].T,
                                    normals@matrix[:3, :3].T, flow[j], fhigh[j]), (step, side, link, mesh, fixtures[j][0])
            link_boxes.append(np.array(boxes))
            sensor.append({'tcp': tcp, 'gap': gap, 'axis': rotation[:, 0]})
        a, b = link_boxes
        delta = np.maximum(np.maximum(a[:, None, 0]-b[None, :, 1], b[None, :, 0]-a[:, None, 1]), 0)
        minimum = min(minimum, float(np.min(np.linalg.norm(delta, axis=2))))
        done = task.success(objects)
        hold = hold+1 if done else 0
        if hold >= 20 and completion is None:
            completion = step
    assert minimum >= .001
    assert completion is not None and completion <= task.RUN_LIMIT
    assert not task._failed, task._failed
    assert task.score(objects) == .75
    assert all(task._complete(n) for n in task.BLOCKS)
    print(json.dumps({'ideal_completion_step': completion, 'interarm_aabb_clearance_m': minimum}))


@pytest.mark.parametrize('offset,valid', [(.001999, True), (.002001, False)])
def test_gauge_centre_threshold(task, offset, valid):
    objects = initial(task); gauge_pose(task, objects, 'block_1', 'go')
    p = objects['block_1'].pos.copy(); p[1] += offset
    put(task, objects, 'block_1', p)
    assert (task._gauge_reading('block_1', 'go', objects) == 'seated') == valid


@pytest.mark.parametrize('angle,valid', [(1.999, True), (2.001, False)])
def test_gauge_yaw_threshold(task, angle, valid):
    objects = initial(task)
    r = axis_angle_matrix(np.array((0., 0., 1.)), math.radians(angle))
    gauge_pose(task, objects, 'block_1', 'go', rotation=r)
    assert (task._gauge_reading('block_1', 'go', objects) == 'seated') == valid


@pytest.mark.parametrize('offset,valid', [(.000499, True), (.000501, False)])
def test_fixed_structure_displacement_threshold(task, offset, valid):
    objects = initial(task); tick(task, objects)
    obj = task.SPECS['gauge_base']
    objects[obj.name] = state(obj, np.array(obj.pos)+(offset, 0., 0.))
    task.success(objects)
    assert (not task._invalid) == valid


@pytest.mark.parametrize('speed,spin,still', [(.009999, 0., True), (.010001, 0., False),
                                             (0., .499999, True), (0., .500001, False)])
def test_rest_fallback_explicit_velocity_boundaries(task, speed, spin, still):
    objects = initial(task)
    put(task, objects, 'block_1', task.SPECS['block_1'].pos, speed=speed, spin=spin)
    task.success(objects)
    assert task._still('block_1', objects) == still


@pytest.mark.parametrize('speed,valid', [(.799999, True), (.800001, False)])
def test_impact_speed_boundary(task, speed, valid):
    objects = initial(task); task.success(objects)
    put(task, objects, 'block_1', task.SPECS['block_1'].pos, speed=speed)
    task.success(objects)
    assert (not task._failed) == valid


@pytest.mark.parametrize('axis,value,valid', [(0, .030001, True), (0, .029999, False),
                                             (0, .549999, True), (0, .550001, False),
                                             (1, .569999, True), (1, .570001, False)])
def test_work_envelope_boundaries(task, axis, value, valid):
    objects = initial(task); tick(task, objects)
    p = objects['block_1'].pos.copy(); p[axis] = value
    put(task, objects, 'block_1', p); task.success(objects)
    assert (not task._invalid) == valid


@pytest.mark.parametrize('separation,valid', [(.079999, False), (.080001, True)])
def test_nearby_gripper_distance_boundary(task, monkeypatch, separation, valid):
    objects = initial(task); task.success(objects)
    gauge_pose(task, objects, 'block_1', 'go')
    grips, _ = task.read_grippers()
    grips[0] = {'tcp': objects['block_1'].pos.copy(), 'gap': .025, 'axis': np.array((0., 1., 0.))}
    grips[0]['tcp'][0] = objects['block_1'].high[0]+separation
    monkeypatch.setattr(task, 'read_grippers', lambda: (grips, 'boundary'))
    task.success(objects)
    assert task._released('block_1', objects) == valid


def test_all_object_numeric_parameters_are_native_float(task):
    for obj in task.OBJECTS:
        for key in ('pos', 'size', 'color', 'mass', 'yaw'):
            value = getattr(obj, key)
            for v in value if isinstance(value, (tuple, list)) else (value,):
                if v is not None:
                    assert type(v) is float, (obj.name, key, type(v))


@pytest.mark.parametrize('event', ['tip', 'fixed_moved', 'wrong_tray', 'untested_tray', 'work_bounds', 'malformed'])
def test_repair_requires_reinspection_then_can_score(task, event):
    objects = initial(task); tick(task, objects); inspect_block(task, objects, 'block_1')
    if event == 'tip':
        r = axis_angle_matrix(np.array((1., 0., 0.)), math.radians(25))
        put(task, objects, 'block_1', (.3, .2, task.TABLE_Z+.1), r)
    elif event == 'fixed_moved':
        obj=task.SPECS['gauge_base']; objects[obj.name]=state(obj,np.array(obj.pos)+(0,0,.003))
    elif event == 'wrong_tray':
        tray_pose(task, objects, 'block_1', 'green')
    elif event == 'untested_tray':
        task._clear_checks('block_1'); tray_pose(task, objects, 'block_1')
    elif event == 'work_bounds':
        put(task, objects, 'block_1', (.56, .2, task.TABLE_Z+.1))
    else:
        obj=objects['block_1']; objects['block_1']=ObjectState(obj.pos,obj.quat_xyzw,obj.lin_vel,obj.ang_vel,obj.low,obj.high+.01)
    tick(task, objects, 20)
    assert not task._failed and task._invalid
    assert task._checks['block_1']=={'go':None,'no_go':None}
    obj=task.SPECS['gauge_base']; objects[obj.name]=state(obj)
    tray_pose(task, objects, 'block_1'); tick(task, objects, 20)
    assert task.score(objects)==0  # uprighting/restoring alone cannot recycle credit
    finish_block(task, objects, 'block_1')
    assert not task._failed and not task._invalid and task.score(objects)==.25


def test_wrong_start_can_be_restored_before_episode_begins(task):
    objects=initial(task); tray_pose(task,objects,'block_1'); tick(task,objects)
    assert not task._started and task.score(objects)==0
    objects=initial(task); tick(task,objects)
    finish_block(task,objects,'block_1')
    assert task.score(objects)==.25


def test_tilted_final_block_never_scores_even_below_reset_threshold(task):
    objects=initial(task); tick(task,objects); finish_block(task,objects,'block_1')
    r=axis_angle_matrix(np.array((1.,0.,0.)),math.radians(2.))
    put(task,objects,'block_1',objects['block_1'].pos+(0,0,.001),r)
    task.success(objects)
    assert task.score(objects)==0 and not task._failed


def test_two_stage_placement_and_pick_holds(task):
    for points in task.demo().values():
        for i,p in enumerate(points):
            if p.label.endswith('lower second'):
                align,first=points[i-1],points[i-2]
                assert align.label.endswith('align') and align.duration>=1.5
                assert first.label.endswith('lower first') and first.pos==align.pos
                where=p.label.split()[1]
                support=task.ANVIL_TOP if where in task.GAPS else task.FLOOR_TOP
                assert first.pos[2]-task.GRIP_Z-support==pytest.approx(.010)
            if p.label.endswith('release'):
                assert points[i-1].label.endswith('hold') and points[i-1].duration>=1.5
                assert points[i-1].pos==p.pos
            if p.label.endswith('close'):
                assert points[i-1].label.endswith('align') and points[i-1].duration>=1.5


@pytest.mark.parametrize('name,gauge,error_mm,accepted', [
    ('block_3','go',.499,True),('block_3','go',.501,False),
    ('block_2','no_go',-.499,True),('block_2','no_go',-.501,False),
    ('block_1','go',2.5,True),('block_1','go',2.501,False),
    ('block_1','no_go',.5,True),('block_1','no_go',.501,False),
    ('block_2','go',0.,True),('block_2','go',.001,False)])
def test_actual_alignment_acceptance(task,name,gauge,error_mm,accepted):
    objects=initial(task); gauge_pose(task,objects,name,gauge)
    put(task,objects,name,objects[name].pos+(0,error_mm/1000,0))
    assert task.alignment(name,gauge,objects)['accepted']==accepted


def test_alignment_uses_rotated_full_block_not_nominal_width(task):
    objects=initial(task); r=axis_angle_matrix(np.array((0.,0.,1.)),math.radians(2))
    gauge_pose(task,objects,'block_2','go',rotation=r)
    assert not task.alignment('block_2','go',objects)['accepted']


def test_debug_every_critical_step_and_second_descent_measurement(task,monkeypatch,capsys):
    monkeypatch.setenv('TASK_DEBUG','1'); objects=initial(task); task.success(objects)
    row=next(r for r in task.demo_timeline() if r['label']=='1 go lower second')
    for k in range(row['start_step']-20,row['end_step']+21):
        assert task.debug_due(k)
    task._step=row['start_step']-1
    gauge_pose(task,objects,'block_1','go',extra=.021)
    task.success(objects)
    records=[json.loads(line.split(' ',1)[1]) for line in capsys.readouterr().out.splitlines()]
    record=records[-1]
    assert record['action_step']==row['start_step']
    assert len(record['second_descent_alignment'])==1
    assert record['second_descent_alignment'][0]['accepted']
    assert record['fixture_clearance'] is None
    assert record['debug_link_reason']=='debug link poses unavailable: no articulation views'
    assert set(('pos_m','quat_xyzw','bottom_corners_m','target_support_gap_mm','released','still')).issubset(record['contact']['block_1'])


def test_gjk_distances_witnesses_intersection_and_nonconvergence(task):
    geom=task.collision_geometry(); box=geom.CORNERS*.01
    for shift,expected in [((.03,0,0),.01),((.03,.03,0),math.sqrt(2)*.01),((0,0,0),0.)]:
        result=geom.convex_distance(box,box+shift)
        assert result['converged'] and result['distance_m']==pytest.approx(expected,abs=1e-9)
        assert np.linalg.norm(np.array(result['witness_a_m'])-result['witness_b_m'])==pytest.approx(expected,abs=1e-9)
    r=axis_angle_matrix(np.array((0.,0.,1.)),math.pi/4)
    rotated=box@r.T+(.04,0,0)
    result=geom.convex_distance(box,rotated)
    assert result['distance_m']==pytest.approx(.04-.01-math.sqrt(2)*.01,abs=1e-9)
    result=geom.convex_distance(box,rotated,max_iterations=1)
    assert not result['converged'] and result['distance_m'] is None


def test_public_pose_mesh_clearance_matches_cpu_geometry(task):
    geom=task.collision_geometry(); points=task.demo()['left']; plan=ArmPlan(points,station.BASES['left'])
    row=next(r for r in task.demo_timeline() if r['label']=='1 go hold')
    for k in range(row['start_step']+1): action,_=plan.action7(k/20)
    fk=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,action[:6]))|{'joint7':-.0125,'joint8':-.0125})
    poses={n:np.r_[fk[n][:3,3]+station.BASES['left'],ik.matrix_to_quat_xyzw(fk[n][:3,:3])].tolist()
           for n in ('gripper','tip_left','tip_right')}
    result=geom.measured_clearance({'link_poses_m_xyzw':poses},initial(task),task.SPECS,task.ANVIL_TOP)
    assert result['reason']=='ok' and result['minimum']['converged']
    assert result['minimum']['lower_m']>=.003 and result['tip_above_anvil_mm']>=3


def test_release_drop_below_two_mm(task):
    for points in task.demo().values():
        for p in points:
            if p.label.endswith('release'):
                number, where = p.label.split()[:2]
                width = task.WIDTHS[int(number)-1]
                support = task.FLOOR_TOP if where == 'tray' else (task.BASE_TOP if width < task.GAPS[where][2] else task.ANVIL_TOP)
                drop = p.pos[2]-task.GRIP_Z-support
                assert 0 <= drop < .002, (p.label, drop)


@pytest.mark.parametrize('failure', ['link_exception', 'distance_exception', 'missing_tips'])
def test_optional_debug_failures_never_change_scoring(task, monkeypatch, failure, capsys):
    def fail(*args):
        raise RuntimeError('intentional diagnostic failure')
    monkeypatch.setenv('TASK_DEBUG', '1')
    if failure == 'link_exception':
        monkeypatch.setattr(task, 'read_debug_link_poses', fail)
    elif failure == 'distance_exception':
        monkeypatch.setattr(task, 'read_debug_link_poses', lambda: ([{}, {}], 'ok'))
        monkeypatch.setattr(task, 'collision_geometry', fail)
    else:
        monkeypatch.setattr(task, 'read_debug_link_poses', lambda: (None, 'missing tips'))
    objects=initial(task); tick(task, objects)
    for name in task.BLOCKS:
        finish_block(task, objects, name)
    assert task.score(objects)==.75 and all(tick(task, objects, 20))
    assert task._grip_reason=='test public sensor' and not task._failed
    assert 'debug_link_reason' in capsys.readouterr().out


def test_debug_disabled_never_reads_optional_links_or_distances(task, monkeypatch):
    monkeypatch.delenv('TASK_DEBUG', raising=False)
    def fail():
        pytest.fail('optional diagnostics called without TASK_DEBUG')
    monkeypatch.setattr(task, 'read_debug_link_poses', fail)
    monkeypatch.setattr(task, 'collision_geometry', fail)
    objects=initial(task); tick(task, objects)
    for name in task.BLOCKS: finish_block(task, objects, name)
    assert task.score(objects)==.75


def test_scoring_reader_accepts_gripper_without_optional_finger_links(monkeypatch):
    module=load(HERE/'task.py').module
    class View:
        count=1
        shared_metatype=SimpleNamespace(dof_names=['joint7','joint8'],link_names=['gripper'])
        def get_dof_positions(self): return np.array([[-.04695,-.04695]])
        def get_link_transforms(self): return np.array([[[.2,.1,1.,0.,0.,0.,1.]]])
    context=SimpleNamespace(instance=lambda: SimpleNamespace(physics_sim_view=SimpleNamespace(create_articulation_view=lambda p: View())))
    monkeypatch.setitem(sys.modules,'isaaclab',SimpleNamespace())
    monkeypatch.setitem(sys.modules,'isaaclab.sim',SimpleNamespace(SimulationContext=context))
    readings,reason=module.read_grippers()
    assert reason=='ok' and len(readings)==2
    poses,debug_reason=module.read_debug_link_poses()
    assert poses is None and 'ValueError' in debug_reason
    assert module.read_grippers()[1]=='ok'


def test_run_limit_metadata_and_plan_budget(task):
    metadata=json.loads((HERE/'task.json').read_text())
    assert task.RUN_LIMIT==6000 and metadata['run_limit'].startswith('6,000')
    steps=round(max(sum(w.duration for w in p) for p in task.demo().values())*20)
    assert steps < task.RUN_LIMIT <= 20*steps
