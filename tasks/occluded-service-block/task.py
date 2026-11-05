"""Move the red service cartridge onto the green mat and leave it released and stable.
Restore other movable equipment to its starting arrangement. Keep the yellow cylinder upright and
within 5 mm of its starting position. This is a cooperative two-arm task; either arm may take either
role. Only the grippers may manipulate objects; use no tools or improvised supports.
The station contains a blue rectangular panel, a red cube, two grey supports, a yellow cylinder,
and a green mat. Determine the relevant obstructions from the camera observations.
A temporary handling region is centred at (0.20, 0.23, table height + 0.14 m).
Stabilize temporary holds and released placements for at least one second. Coordinates are metres.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20
PHYSICS_HZ = 240
SUBSTEPS = PHYSICS_HZ // CONTROL_HZ
# Development estimates, NOT empirically calibrated physical constants.
CONTACT_ON_N = 0.5
CONTACT_OFF_N = 0.2
CONTACT_MIN_S = 0.10
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"  # A late violation must erase all earlier partial credit.

COVER = Box("cover", pos=(0.25, 0.025, TOP + 0.07), size=(0.18, 0.07, 0.02), mass=0.10, color=(0.15, 0.30, 0.85))
CARTRIDGE = Box("cartridge", pos=(0.25, 0.02, TOP + 0.02), size=(0.04, 0.04, 0.04), mass=0.08, color=(0.85, 0.12, 0.12))
RAIL_A = Box("rail_a", pos=(0.17, 0.025, TOP + 0.03), size=(0.02, 0.07, 0.06), color=(0.55, 0.55, 0.55))
RAIL_B = Box("rail_b", pos=(0.33, 0.025, TOP + 0.03), size=(0.02, 0.07, 0.06), color=(0.55, 0.55, 0.55))
PROTECTED = Cylinder(
    "protected_cylinder", pos=(0.33, -0.12, TOP + 0.06), radius=0.022, height=0.12, mass=0.08, color=(0.95, 0.75, 0.10)
)
MAT = Box("goal_mat", pos=(0.24, -0.20, TOP + 0.0005), size=(0.09, 0.09, 0.001), color=(0.15, 0.65, 0.25))
OBJECTS = [COVER, CARTRIDGE, RAIL_A, RAIL_B, PROTECTED, MAT]
FREE = (COVER, CARTRIDGE, PROTECTED)
HOLD = np.array([0.20, 0.23, TOP + 0.14])
GOAL = np.array([MAT.pos[0], MAT.pos[1], TOP + 0.021])
ARMS = ("left", "right")
CONTACT_OBJECTS = (COVER.name, CARTRIDGE.name)


def reset_episode(objects=None, control_hz=CONTROL_HZ):
    """Called by run_task.py after the real embodiment reset, never inferred from object positions."""
    global _history, _robot, _contact_input, _contact_state
    if not np.isclose(control_hz, CONTROL_HZ):
        raise ValueError("This task is defined at exactly 20 Hz")
    initial = {spec.name: np.asarray(spec.pos, dtype=float).copy() for spec in FREE}
    if objects is not None:
        for spec in FREE:
            obj = objects[spec.name]
            if not _finite(obj) or np.linalg.norm(obj.pos - initial[spec.name]) > 0.01 or _tilt(obj) > 5.0:
                raise ValueError(f"Invalid initial layout: {spec.name}; use object_jitter=0")
            initial[spec.name] = obj.pos.copy()
    _history = dict(
        initial=initial, failure=None, open_steps=0, opened=False, started=False, extracted=False,
        support_arm=None, extraction_arm=None, cooperation_steps=0, release_started=False,
        goal_steps=0, delivered=False, steps=0, contact_samples=0,
    )
    _robot = None
    _contact_input = None
    _contact_state = {
        (arm, name): {"active": np.zeros(2, dtype=bool), "duration_s": 0.0}
        for arm in ARMS for name in CONTACT_OBJECTS
    }


def observe_robot(arms, contacts=None):
    """Supply measured robot state and fresh (12 substeps, 2 fingertips) normal forces per arm/object.

    Contacts come from filtered physics sensors, never from TCP distance or policy assertions.
    Each batch is consumed once by success(); score()/distance() do not advance contact history.
    """
    global _robot, _contact_input
    _robot = arms
    _contact_input = contacts


def _update_contacts():
    global _contact_input
    batch, _contact_input = _contact_input, None
    try:
        forces = {
            (arm, name): np.asarray(batch[arm][name], dtype=float)
            for arm in ARMS for name in CONTACT_OBJECTS
        }
    except (KeyError, TypeError, ValueError):
        return "missing or malformed per-arm contact telemetry"
    if any(f.shape != (SUBSTEPS, 2) or not np.isfinite(f).all() or (f < 0).any() for f in forces.values()):
        return "invalid or incomplete physics-substep contact telemetry"
    for key, samples in forces.items():
        data = _contact_state[key]
        for sample in samples:
            data["active"] = sample >= np.where(data["active"], CONTACT_OFF_N, CONTACT_ON_N)
            data["duration_s"] = data["duration_s"] + 1 / PHYSICS_HZ if data["active"].all() else 0.0
    _history["contact_samples"] += SUBSTEPS
    return None


def _grasped(arm, name, objects):
    data = _contact_state[arm, name]
    return bool(
        data["duration_s"] + 1e-9 >= CONTACT_MIN_S
        and _robot[arm]["opening"] <= 0.85
        and np.linalg.norm(_robot[arm]["tcp"] - objects[name].pos) <= 0.03
    )


def _untouched(name):
    return not any(_contact_state[arm, name]["active"].any() for arm in ARMS)


def _released(arm, obj):
    return bool(_robot[arm]["opening"] >= 0.90 and np.linalg.norm(_robot[arm]["tcp"] - obj.pos) >= 0.06)


def _finite(obj):
    return all(
        np.isfinite(v).all() for v in (obj.pos, obj.quat_xyzw, obj.lin_vel, obj.ang_vel, obj.low, obj.high)
    ) and bool(np.linalg.norm(obj.quat_xyzw) > 1e-9)


def _tilt(obj):
    """Angle of the local +z axis to world +z, with ObjectState's XYZW quaternion."""
    q = np.asarray(obj.quat_xyzw, dtype=float)
    q = q / np.linalg.norm(q)
    cosine = 1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def _held(objects, arm=None):
    cover = objects[COVER.name]
    arm = arm or _history["support_arm"]
    if arm is None:
        return any(_held(objects, candidate) for candidate in ARMS)
    return bool(
        _robot is not None
        and _grasped(arm, COVER.name, objects)
        and np.linalg.norm(cover.pos[:2] - HOLD[:2]) <= 0.025
        and abs(cover.pos[2] - HOLD[2]) <= 0.02
        and cover.low[2] >= TOP + 0.10
        and _tilt(cover) <= 10.0
        and cover.resting(max_speed=0.02, max_spin=0.5)
    )


def _in_goal_region(objects):
    obj = objects[CARTRIDGE.name]
    return bool(
        np.linalg.norm(obj.pos[:2] - GOAL[:2]) <= 0.015
        and abs(obj.low[2] - (TOP + 0.001)) <= 0.005
        and abs(obj.pos[2] - GOAL[2]) <= 0.005
        and _tilt(obj) <= 10.0
    )


def _at_goal(objects):
    return _in_goal_region(objects) and objects[CARTRIDGE.name].resting()


def _placed(objects):
    arm = _history["extraction_arm"]
    return bool(arm is not None and _at_goal(objects) and _released(arm, objects[CARTRIDGE.name])
                and _untouched(CARTRIDGE.name))


def _home(objects):
    obj = objects[COVER.name]
    arm = _history["support_arm"]
    return bool(
        arm is not None
        and np.linalg.norm(obj.pos[:2] - np.asarray(COVER.pos[:2])) <= 0.01
        and abs(obj.pos[2] - COVER.pos[2]) <= 0.005
        and _tilt(obj) <= 5.0
        and obj.resting()
        and _released(arm, obj)
        and _untouched(COVER.name)
    )


def _violation(objects):
    if _robot is None:
        return "missing robot telemetry; launch with run_task.py"
    for arm in ARMS:
        data = _robot.get(arm, {})
        tcp = np.asarray(data.get("tcp", []))
        opening = data.get("opening", np.nan)
        if tcp.shape != (3,) or not np.isfinite(tcp).all() or not np.isfinite(opening) or not 0 <= opening <= 1:
            return f"non-finite robot state: {arm}"
    for spec in FREE:
        obj = objects[spec.name]
        if not _finite(obj):
            return f"non-finite state: {spec.name}"
        if (
            obj.low[0] < 0.04
            or obj.high[0] > 0.49
            or obj.low[1] < -0.34
            or obj.high[1] > 0.34
            or obj.low[2] < TOP - 0.01
            or obj.high[2] > TOP + 0.30
        ):
            return f"outside safe workspace: {spec.name}"
    bottle = objects[PROTECTED.name]
    if np.linalg.norm(bottle.pos - _history["initial"][PROTECTED.name]) > 0.005 or _tilt(bottle) > 10.0:
        return "protected cylinder moved more than 5 mm or tilted more than 10 degrees"
    if _tilt(objects[COVER.name]) > 25.0 or _tilt(objects[CARTRIDGE.name]) > 30.0:
        return "cover tilted more than 25 degrees or cartridge more than 30 degrees"
    return None


def success(objects):
    """Advance history exactly once per control step; the embodiment calls this before score/distance."""
    h = _history
    h["steps"] += 1
    if h["failure"] is not None:
        return False
    h["failure"] = _violation(objects)
    if h["failure"] is None:
        h["failure"] = _update_contacts()
    if h["failure"] is not None:
        return False
    if h["support_arm"] is None:
        h["support_arm"] = next((arm for arm in ARMS if _held(objects, arm)), None)
    held = _held(objects)
    h["open_steps"] = h["open_steps"] + 1 if held else 0
    h["opened"] = h["opened"] or h["open_steps"] >= CONTROL_HZ
    moved = np.linalg.norm(objects[CARTRIDGE.name].pos - h["initial"][CARTRIDGE.name]) > 0.01
    if moved and not h["started"]:
        if not h["opened"] or not held:
            h["failure"] = "cartridge moved before cover was held clear for 1 second"
            return False
        other = next(arm for arm in ARMS if arm != h["support_arm"])
        if not _grasped(other, CARTRIDGE.name, objects):
            h["failure"] = "extraction requires a different arm with sustained cartridge contact"
            return False
        h["extraction_arm"] = other
        h["started"] = True
    if h["started"] and not h["delivered"] and not held:
        h["failure"] = "cover left its holding zone before cartridge placement held for 1 second"
        return False
    if h["started"] and not h["delivered"]:
        grasped = _grasped(h["extraction_arm"], CARTRIDGE.name, objects)
        if not h["extracted"]:
            if not grasped:
                h["failure"] = "cartridge lost attributed grasp before extraction was verified"
                return False
            h["cooperation_steps"] += 1
            h["extracted"] = bool(objects[CARTRIDGE.name].low[2] >= TOP + 0.085)
            if _at_goal(objects):
                h["failure"] = "cartridge reached goal without the required extraction event"
                return False
        # Opening at the mat is allowed before the TCP retreats. Opening in transit is not.
        if not grasped:
            if not _in_goal_region(objects):
                h["failure"] = "cartridge released outside its goal region"
                return False
            h["release_started"] = True
        if h["release_started"] and not _in_goal_region(objects):
            h["failure"] = "cartridge left goal after release began"
            return False
    h["goal_steps"] = h["goal_steps"] + 1 if h["extracted"] and _placed(objects) else 0
    h["delivered"] = h["delivered"] or h["goal_steps"] >= CONTROL_HZ
    return bool(h["delivered"] and _placed(objects) and _home(objects))


def distance(objects):
    """Diagnostic distance only; history and safety determine the official task_score."""
    return float(np.linalg.norm(objects[CARTRIDGE.name].pos - GOAL))


def score(objects):
    """Final partial credit. Full credit is awarded only by task_score after sustained success."""
    if _history["failure"] is not None or _violation(objects) is not None:
        return 0.0
    if _history["delivered"] and _placed(objects):
        return 0.75
    if (_history["extracted"] and _held(objects) and objects[CARTRIDGE.name].low[2] >= TOP + 0.085
            and _grasped(_history["extraction_arm"], CARTRIDGE.name, objects)):
        return 0.5
    return 0.25 if _history["opened"] and _held(objects) else 0.0


def diagnostics():
    """JSON-safe terminal audit, including the first violation even if the objects were restored."""
    return {k: v for k, v in _history.items() if k != "initial"}


def demo():
    """World-frame waypoints; construct each ArmPlan with base_pos=station.BASES[arm].

    yam_scripted selects the station bases from SCENE and builds the ArmPlan instances.
    Physics success still requires an RTX simulation run.
    """
    travel = TOP + 0.14
    left = pick_place_waypoints(COVER.pos, tuple(HOLD), travel_z=travel)
    # Keep the grip closed: stop after carry instead of lowering and releasing at the holding point.
    left = [*left[:5], Waypoint(18.0, tuple(HOLD), FINGER_CLOSED, "hold cover while right arm retrieves")]
    left += [
        Waypoint(1.8, (COVER.pos[0], COVER.pos[1], travel), FINGER_CLOSED, "return cover above rails"),
        Waypoint(1.2, (COVER.pos[0], COVER.pos[1], COVER.pos[2] + 0.003), FINGER_CLOSED, "lower cover"),
        Waypoint(0.6, (COVER.pos[0], COVER.pos[1], COVER.pos[2] + 0.003), FINGER_OPEN, "release cover"),
        Waypoint(1.0, (COVER.pos[0], COVER.pos[1], travel), FINGER_OPEN, "retreat"),
    ]
    park = (GOAL[0], GOAL[1], travel)
    right = [
        Waypoint(1.5, park, FINGER_OPEN, "right arm to clear waiting pose"),
        Waypoint(7.0, park, FINGER_OPEN, "wait for cover and its one-second hold"),
    ]
    place = (GOAL[0], GOAL[1], GOAL[2] + 0.003)
    right += pick_place_waypoints(CARTRIDGE.pos, place, travel_z=travel)
    return {"left": left, "right": right}


# Safe initialization for a newly loaded module; run_task.py also resets it after every physical reset.
reset_episode()
