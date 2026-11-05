"""Behavioral scoring checks and real YAM kinematics; no Isaac Sim or GPU required."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.analysis import fk_link_poses_world, hull_set_distance, world_hulls
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import Box, ObjectState, load
from yam_arena.urdf import quat_xyzw_to_matrix

TASK_PATH = Path(__file__).with_name("task.py")


def state(spec, pos=None, quat=(0.0, 0.0, 0.0, 1.0), velocity=(0.0, 0.0, 0.0)):
    pos = np.asarray(spec.pos if pos is None else pos, dtype=float)
    q = np.asarray(quat, dtype=float)
    rotation = quat_xyzw_to_matrix(q)
    if isinstance(spec, Box):
        half = np.abs(rotation) @ (np.asarray(spec.size) / 2)
    else:
        axis = rotation[:, 2]
        half = spec.height / 2 * np.abs(axis) + spec.radius * np.sqrt(np.maximum(0, 1 - axis**2))
    return ObjectState(pos, q, np.asarray(velocity, dtype=float), np.zeros(3), pos - half, pos + half)


def tilted(degrees, axis=0):
    q = np.zeros(4)
    q[axis], q[3] = np.sin(np.radians(degrees) / 2), np.cos(np.radians(degrees) / 2)
    return q


@pytest.fixture
def trial():
    task = load(TASK_PATH)
    module = task.module
    objects = {spec.name: state(spec) for spec in task.objects}
    module.reset_episode(objects, 20)
    return task, module, objects


def frame(module, objects, *, left_released=False, right_released=True, support_arm="left", contacts=None):
    """Synthetic sensor batches; released flags refer to cover/cartridge roles respectively."""
    cover, cartridge = objects["cover"], objects["cartridge"]
    extraction_arm = next(arm for arm in module.ARMS if arm != support_arm)
    if contacts is None:
        contacts = {arm: {name: np.zeros((module.SUBSTEPS, 2)) for name in module.CONTACT_OBJECTS}
                    for arm in module.ARMS}
        if not left_released:
            contacts[support_arm]["cover"][:] = 1.0
        if not right_released:
            contacts[extraction_arm]["cartridge"][:] = 1.0
    module.observe_robot(
        {
            support_arm: {
                "tcp": cover.pos + np.array([0, 0, 0.10 if left_released else 0]),
                "opening": 1.0 if left_released else 0.74,
            },
            extraction_arm: {
                "tcp": cartridge.pos + np.array([0, 0, 0.12 if right_released else 0]),
                "opening": 1.0 if right_released else 0.43,
            },
        }, contacts,
    )
    return module.success(objects)


def open_cover(module, objects, *, support_arm="left"):
    # Establish two-fingertip contact before starting the original one-second cover hold.
    for _ in range(2):
        assert not frame(module, objects, support_arm=support_arm)
    objects["cover"] = state(module.COVER, module.HOLD)
    for _ in range(20):
        assert not frame(module, objects, support_arm=support_arm)


def extract(module, objects, *, support_arm="left"):
    for _ in range(2):
        assert not frame(module, objects, right_released=False, support_arm=support_arm)
    objects["cartridge"] = state(module.CARTRIDGE, (0.25, 0.02, module.TOP + 0.13))
    assert not frame(module, objects, right_released=False, support_arm=support_arm)


def deliver(module, objects):
    open_cover(module, objects)
    extract(module, objects)
    assert module.score(objects) == 0.5
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(20):
        assert not frame(module, objects)


def test_real_loader_and_fixed_supports(trial):
    task, module, objects = trial
    assert task.scene == "station" and task.score_over == "end" and task.success_hold_s == 1.0
    assert task.free_objects == ("cover", "cartridge", "protected_cylinder")
    assert not module.RAIL_A.free and not module.RAIL_B.free and not module.MAT.free
    assert objects["cover"].low[2] - objects["cartridge"].high[2] == pytest.approx(0.02)
    assert not frame(module, objects) and module.score(objects) == 0.0


def test_ordered_trial_and_full_credit_cannot_bypass_external_hold(trial):
    _, module, objects = trial
    open_cover(module, objects)
    assert module.score(objects) == 0.25
    extract(module, objects)
    assert module.score(objects) == 0.5
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(19):
        assert not frame(module, objects)
    assert not module.diagnostics()["delivered"]
    assert not frame(module, objects)
    assert module.diagnostics()["delivered"] and module.score(objects) == 0.75
    objects["cover"] = state(module.COVER)
    assert frame(module, objects, left_released=True)
    # task_score, rather than score(), awards 1 after the embodiment's 20-step success hold.
    assert module.score(objects) == 0.75


def test_initial_occlusion_is_not_enough_to_earn_points(trial):
    _, module, objects = trial
    objects["cover"] = state(module.COVER, (0.25, 0.025, module.TOP + 0.14))
    for _ in range(30):
        frame(module, objects)
    assert module.score(objects) == 0.0


def test_cartridge_moving_before_twenty_cover_samples_is_fatal(trial):
    _, module, objects = trial
    objects["cover"] = state(module.COVER, module.HOLD)
    for _ in range(19):
        frame(module, objects)
    objects["cartridge"] = state(module.CARTRIDGE, (0.25, 0.02, module.TOP + 0.04))
    # Even if this frame reaches the count threshold, being outside the holding region must fail.
    objects["cover"] = state(module.COVER)
    assert not frame(module, objects)
    assert module.score(objects) == 0.0 and module.diagnostics()["failure"] is not None


def test_cover_must_stay_held_until_delivery_is_verified(trial):
    _, module, objects = trial
    open_cover(module, objects)
    extract(module, objects)
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(19):
        frame(module, objects)
    objects["cover"] = state(module.COVER)
    assert not frame(module, objects, left_released=True)
    objects["cover"] = state(module.COVER, module.HOLD)
    for _ in range(30):
        frame(module, objects)
    assert module.score(objects) == 0.0


@pytest.mark.parametrize("right_released", [False, True])
def test_placement_requires_open_gripper_and_clear_tcp(trial, right_released):
    _, module, objects = trial
    open_cover(module, objects)
    extract(module, objects)
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(30):
        if not right_released:
            frame(module, objects, right_released=False)
        else:
            # Open but still beside the object: open fingers alone do not prove withdrawal.
            contacts = {arm: {name: np.zeros((module.SUBSTEPS, 2)) for name in module.CONTACT_OBJECTS}
                        for arm in module.ARMS}
            contacts["left"]["cover"][:] = 1.0
            module.observe_robot(
                {"left": {"tcp": module.HOLD, "opening": 0.74}, "right": {"tcp": module.GOAL, "opening": 1.0}},
                contacts,
            )
            module.success(objects)
    assert not module.diagnostics()["delivered"] and module.score(objects) == 0.25


@pytest.mark.parametrize(
    "name,delta,angle",
    [
        ("protected_cylinder", (0.0051, 0, 0), 0),
        ("protected_cylinder", (0, 0, 0), 10.1),
        ("cover", (0, 0, 0), 25.1),
        ("cartridge", (0, 0, 0), 30.1),
        ("cartridge", (0, -0.40, 0), 0),
        ("cartridge", (0, 0, -0.08), 0),
    ],
)
def test_violation_stays_zero_after_object_is_restored(trial, name, delta, angle):
    task, module, objects = trial
    deliver(module, objects)
    spec = next(obj for obj in task.objects if obj.name == name)
    safe = objects[name]
    objects[name] = state(spec, safe.pos + np.asarray(delta), tilted(angle))
    assert not frame(module, objects)
    reason = module.diagnostics()["failure"]
    assert reason is not None and module.score(objects) == 0.0
    objects[name] = safe
    assert not frame(module, objects) and module.score(objects) == 0.0
    assert module.diagnostics()["failure"] == reason


def test_repeated_reset_clears_the_safety_latch(trial):
    task, module, objects = trial
    bottle = module.PROTECTED
    objects[bottle.name] = state(bottle, np.asarray(bottle.pos) + np.array([0.02, 0, 0]))
    frame(module, objects)
    assert module.diagnostics()["failure"] is not None
    objects = {spec.name: state(spec) for spec in task.objects}
    module.reset_episode(objects, 20)
    deliver(module, objects)
    assert module.score(objects) == 0.75 and module.diagnostics()["failure"] is None


def test_score_queries_do_not_advance_the_clock(trial):
    _, module, objects = trial
    objects["cover"] = state(module.COVER, module.HOLD)
    frame(module, objects)
    before = module.diagnostics()
    for _ in range(50):
        module.score(objects)
        module.distance(objects)
    assert module.diagnostics() == before and not before["opened"]


def test_yaw_is_not_mistaken_for_tilt(trial):
    _, module, objects = trial
    objects["protected_cylinder"] = state(module.PROTECTED, quat=tilted(130, axis=2))
    assert not frame(module, objects)
    assert module.diagnostics()["failure"] is None


def test_missing_robot_state_fails_closed(trial):
    _, module, objects = trial
    assert not module.success(objects)
    assert module.score(objects) == 0.0
    assert "telemetry" in module.diagnostics()["failure"]


@pytest.mark.parametrize("active_arm", ["left", "right"])
def test_one_arm_doing_entire_workflow_cannot_score_even_with_correct_final_poses(trial, active_arm):
    _, module, objects = trial
    # Leave the idle arm geometrically close with its gripper closed: this fooled the old proxy.
    # Only the active arm has physical contact, first with the cover, then the cartridge.
    idle_arm = next(arm for arm in module.ARMS if arm != active_arm)
    for name, pos in [("cover", module.HOLD), ("cartridge", (0.25, 0.02, module.TOP + 0.13)),
                      ("cartridge", module.GOAL), ("cover", module.COVER.pos)]:
        spec = module.COVER if name == "cover" else module.CARTRIDGE
        objects[name] = state(spec, pos)
        for _ in range(25):
            contacts = {arm: {obj: np.zeros((module.SUBSTEPS, 2)) for obj in module.CONTACT_OBJECTS}
                        for arm in module.ARMS}
            contacts[active_arm][name][:] = 1.0
            module.observe_robot({
                idle_arm: {"tcp": objects["cover"].pos, "opening": 0.74},
                active_arm: {"tcp": objects[name].pos, "opening": 0.43},
            }, contacts)
            assert not module.success(objects)
    assert np.allclose(objects["cartridge"].pos, module.GOAL)
    assert np.allclose(objects["cover"].pos, module.COVER.pos)
    assert module.score(objects) == 0.0
    assert module.diagnostics()["failure"] is not None


@pytest.mark.parametrize("opened_first", [False, True])
def test_skipping_extraction_to_correct_final_state_cannot_score(trial, opened_first):
    _, module, objects = trial
    if opened_first:
        open_cover(module, objects)
        # Even a real grasp at the source cannot replace the required extraction trajectory.
        for _ in range(2):
            frame(module, objects, right_released=False)
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(25):
        assert not frame(module, objects, right_released=not opened_first)
    objects["cover"] = state(module.COVER)
    for _ in range(25):
        assert not frame(module, objects, left_released=True)
    assert not module.diagnostics()["extracted"]
    assert not module.diagnostics()["delivered"]
    assert module.score(objects) == 0.0


def test_swapped_arm_roles_can_complete_same_goal(trial):
    _, module, objects = trial
    open_cover(module, objects, support_arm="right")
    extract(module, objects, support_arm="right")
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(20):
        assert not frame(module, objects, support_arm="right")
    objects["cover"] = state(module.COVER)
    assert frame(module, objects, left_released=True, support_arm="right")
    assert module.score(objects) == 0.75
    assert module.diagnostics()["support_arm"] == "right"
    assert module.diagnostics()["extraction_arm"] == "left"


def test_tcp_proximity_and_closed_gripper_without_contact_earns_nothing(trial):
    _, module, objects = trial
    objects["cover"] = state(module.COVER, module.HOLD)
    contacts = {arm: {name: np.zeros((module.SUBSTEPS, 2)) for name in module.CONTACT_OBJECTS}
                for arm in module.ARMS}
    for _ in range(25):
        assert not frame(module, objects, contacts=contacts)
    assert not module.diagnostics()["opened"] and module.score(objects) == 0.0


@pytest.mark.parametrize("fault", ["missing", "truncated", "nan", "single_touch"])
def test_contact_data_fault_or_brief_touch_cannot_fake_cooperation(trial, fault):
    _, module, objects = trial
    open_cover(module, objects)
    contacts = {arm: {name: np.zeros((module.SUBSTEPS, 2)) for name in module.CONTACT_OBJECTS}
                for arm in module.ARMS}
    contacts["left"]["cover"][:] = 1.0
    if fault == "missing":
        del contacts["right"]["cartridge"]
    elif fault == "truncated":
        contacts["right"]["cartridge"] = np.ones((module.SUBSTEPS - 1, 2))
    elif fault == "nan":
        contacts["right"]["cartridge"][:] = np.nan
    else:
        contacts["right"]["cartridge"][-1] = 1.0
    objects["cartridge"] = state(module.CARTRIDGE, (0.25, 0.02, module.TOP + 0.13))
    assert not frame(module, objects, right_released=False, contacts=contacts)
    assert module.score(objects) == 0.0 and module.diagnostics()["failure"] is not None


def test_contact_batch_cannot_be_reused_on_a_later_control_step(trial):
    _, module, objects = trial
    frame(module, objects)
    assert not module.success(objects)
    assert "contact telemetry" in module.diagnostics()["failure"]


def test_losing_support_contact_during_extraction_latches_failure(trial):
    _, module, objects = trial
    open_cover(module, objects)
    extract(module, objects)
    contacts = {arm: {name: np.zeros((module.SUBSTEPS, 2)) for name in module.CONTACT_OBJECTS}
                for arm in module.ARMS}
    contacts["right"]["cartridge"][:] = 1.0
    # Left TCP and fingers remain in their previous pose, but actual support has disappeared.
    assert not frame(module, objects, right_released=False, contacts=contacts)
    assert "cover" in module.diagnostics()["failure"]
    assert not frame(module, objects, right_released=False)
    assert module.score(objects) == 0.0


def test_placement_can_settle_after_release_without_earning_premature_delivery(trial):
    _, module, objects = trial
    open_cover(module, objects)
    extract(module, objects)
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL, velocity=(0.015, 0, 0))
    for _ in range(25):
        assert not frame(module, objects)
    assert module.diagnostics()["failure"] is None
    assert not module.diagnostics()["delivered"]
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    for _ in range(20):
        assert not frame(module, objects)
    assert module.score(objects) == 0.75


def test_contact_adapter_keeps_arm_object_identity_and_opposing_finger_forces(trial):
    _, module, _ = trial
    spec = importlib.util.spec_from_file_location("occluded_runner_test", TASK_PATH.with_name("run_task.py"))
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class Tensor:
        """Minimal normal-force tensor fixture; this is not a physical sensor verification."""
        def __init__(self, values):
            self.values = np.asarray(values)
            self.shape = self.values.shape

        def __getitem__(self, index):
            return Tensor(self.values[index])

        def norm(self):
            return np.linalg.norm(self.values)

    class Scene(dict):
        def update(self, dt):
            pass

    scene = Scene()
    for arm in module.ARMS:
        for tip in ("tip_left", "tip_right"):
            for name in module.CONTACT_OBJECTS:
                strength = 1.0 if (arm, name) == ("left", "cover") else 0.0
                if (arm, name) == ("right", "cartridge"):
                    strength = 2.0
                direction = 1 if tip == "tip_left" else -1
                matrix = Tensor([[[[direction * strength, 0.0, 0.0]]]])
                scene[f"audit_{arm}_{tip}_{name}"] = SimpleNamespace(
                    body_names=[tip], cfg=SimpleNamespace(prim_path=tip, filter_prim_paths_expr=[name]),
                    data=SimpleNamespace(normal_force_matrix_w=SimpleNamespace(torch=matrix)),
                )
    embodiment = SimpleNamespace(task=SimpleNamespace(module=module), _base=SimpleNamespace(scene=scene))
    runner.install_contact_sampling(embodiment)
    scene.update(1 / module.PHYSICS_HZ)
    assert not embodiment._contact_frames  # Settling/reset samples must not leak into the trial.
    embodiment._capture_contacts = True
    for _ in range(module.SUBSTEPS):
        scene.update(1 / module.PHYSICS_HZ)
    assert len(embodiment._contact_frames) == module.SUBSTEPS
    sample = embodiment._contact_frames[0]
    assert sample["left"]["cover"] == [1.0, 1.0]
    assert sample["right"]["cartridge"] == [2.0, 2.0]
    assert sample["right"]["cover"] == sample["left"]["cartridge"] == [0.0, 0.0]
    with pytest.raises(RuntimeError, match="one control period"):
        scene.update(1 / module.PHYSICS_HZ)


def test_rate_and_invalid_reset_layout_are_rejected(trial):
    _, module, objects = trial
    with pytest.raises(ValueError, match="20 Hz"):
        module.reset_episode(objects, 10)
    objects["cartridge"] = state(module.CARTRIDGE, module.GOAL)
    with pytest.raises(ValueError, match="initial layout"):
        module.reset_episode(objects, 20)


def test_both_demo_trajectories_are_reachable_at_twenty_hz(trial):
    task, _, _ = trial
    demo = task.demo()
    assert set(demo) == {"left", "right"}
    for arm, waypoints in demo.items():
        plan = ArmPlan(waypoints, base_pos=station.BASES[arm])
        actions = np.array([plan.action7((k + 1) / 20)[0] for k in range(round(plan.duration * 20) + 5)])
        assert plan.duration < 120
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        assert np.all((actions[:, 6] >= 0) & (actions[:, 6] <= 1))


def test_forearms_do_not_interpenetrate_at_retrieval_checkpoints(trial):
    task, _, _ = trial
    plans = {arm: ArmPlan(waypoints, base_pos=station.BASES[arm]) for arm, waypoints in task.demo().items()}
    # These checkpoints include the old holding zone's 25 mm link4/link4 overlap at t=13.15 s.
    checkpoints = {200, 240, 263, 280, 320}
    links = ["link3", "link4", "link5", "link6", "gripper", "tip_left", "tip_right"]
    for k in range(1, max(checkpoints) + 1):
        hulls = {}
        for arm, plan in plans.items():
            action, _ = plan.action7(k / 20)
            if k in checkpoints:
                joints = dict(zip(ARM_JOINTS, action[:6], strict=True))
                joints.update(joint7=action[6] * FINGER_OPEN, joint8=action[6] * FINGER_OPEN)
                hulls[arm] = world_hulls(fk_link_poses_world(joints, station.BASES[arm]), links=links)
        if hulls:
            distance, pair = hull_set_distance(hulls["left"], hulls["right"])
            assert distance >= -0.001, (k / 20, pair, distance)
