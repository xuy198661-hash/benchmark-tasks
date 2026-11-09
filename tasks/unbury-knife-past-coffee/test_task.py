"""CPU tests: scoring logic, demo geometry, and (with the real yam_arena) demo kinematics.

Without yam_arena installed, minimal stubs stand in for TABLE_TOP, the object constructors and the
waypoint helpers (copied from the framework's source); the kinematic tests are then skipped."""

import dataclasses
import importlib.util
import math
import pathlib
import sys
import types

import pytest

try:
    import yam_arena.demo  # noqa: F401
    import yam_arena.task_file  # noqa: F401
    REAL = True
except ImportError:
    REAL = False

    @dataclasses.dataclass
    class _Waypoint:
        duration: float
        pos: tuple
        finger: float
        label: str = ""
        yaw: float = 0.0

    def _pick_place(pick, place, *, travel_z, yaw=0.0, place_yaw=None, speed=1.0):
        (px, py, pz), (qx, qy, qz) = pick, place
        e = yaw if place_yaw is None else place_yaw
        k = 1.0 / speed
        O, C = -0.04695, 0.0
        return [
            _Waypoint(1.5 * k, (px, py, travel_z), O, "to the object", yaw),
            _Waypoint(1.5 * k, (px, py, pz), O, "descend", yaw),
            _Waypoint(0.8 * k, (px, py, pz), C, "close", yaw),
            _Waypoint(1.2 * k, (px, py, travel_z), C, "lift", yaw),
            _Waypoint(1.8 * k, (qx, qy, travel_z), C, "carry", e),
            _Waypoint(1.2 * k, (qx, qy, qz), C, "lower", e),
            _Waypoint(0.6 * k, (qx, qy, qz), O, "release", e),
            _Waypoint(1.0 * k, (qx, qy, travel_z), O, "retreat", e),
        ]

    def _spec(*a, **k):
        return (a, k)

    pkg = types.ModuleType("yam_arena")
    tf = types.ModuleType("yam_arena.task_file")
    tf.TABLE_TOP = {"station": 0.7475, "tabletop": 0.75}
    tf.Asset = tf.Box = tf.Cylinder = tf.Sphere = _spec
    dm = types.ModuleType("yam_arena.demo")
    dm.Waypoint, dm.pick_place_waypoints = _Waypoint, _pick_place
    dm.FINGER_OPEN, dm.FINGER_CLOSED = -0.04695, 0.0
    pkg.task_file, pkg.demo = tf, dm
    sys.modules.update({"yam_arena": pkg, "yam_arena.task_file": tf, "yam_arena.demo": dm})

HERE = pathlib.Path(__file__).parent
_spec_ = importlib.util.spec_from_file_location("task_under_test", HERE / "task.py")
task = importlib.util.module_from_spec(_spec_)
_spec_.loader.exec_module(task)
T0 = task.TABLE_Z
L = task.LAYOUT
ID = (0.0, 0.0, 0.0, 1.0)
HZ = 20


# ---------------------------------------------------------------------------
# Stub ObjectState with the same public fields and resting() signature
# ---------------------------------------------------------------------------
class S:
    def __init__(self, pos, quat=ID, low=None, high=None, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
        self.pos = tuple(pos)
        self.quat_xyzw = tuple(quat)
        self.low = tuple(low)
        self.high = tuple(high)
        self.lin_vel = tuple(lin_vel)
        self.ang_vel = tuple(ang_vel)

    def resting(self, max_speed=0.01, max_spin=0.5):
        return math.hypot(*self.lin_vel) < max_speed and math.hypot(*self.ang_vel) < max_spin


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def box(x, y, z, sx, sy, sz, vel=(0, 0, 0)):
    return S((x, y, T0 + z), ID, (x - sx / 2, y - sy / 2, T0 + z - sz / 2),
             (x + sx / 2, y + sy / 2, T0 + z + sz / 2), lin_vel=vel)


def cyl(x, y, r, h, quat=ID):
    return S((x, y, T0 + h / 2), quat, (x - r, y - r, T0), (x + r, y + r, T0 + h))


def sphere(x, y, z, r, vel=(0, 0, 0)):
    return S((x, y, T0 + z), ID, (x - r, y - r, T0 + z - r), (x + r, y + r, T0 + z + r), lin_vel=vel)


def knife(x, y, z, quat=ID, vel=(0, 0, 0)):
    c = (x, y, T0 + z)
    o = S(c, quat, c, c, lin_vel=vel)
    tip, butt = task._knife_ends(o)
    pad = task.KNIFE_HEIGHT / 2
    o.low = tuple(min(tip[i], butt[i]) - pad for i in range(3))
    o.high = tuple(max(tip[i], butt[i]) + pad for i in range(3))
    return o


def nominal():
    return {
        "knife": knife(0.26, 0.00, task.KNIFE_HEIGHT / 2),
        "towel_pad": box(0.26, 0.06, task._PAD_Z - 0.002, 0.120, 0.110, 0.008),
        "towel_sheet": box(0.26, 0.00, task._SHEET_Z - 0.004, 0.260, 0.240, 0.0015),
        "apple": sphere(0.26, 0.06, task._APPLE_Z - 0.007, 0.033),
        "bowl": cyl(0.24, 0.26, 0.080, 0.055),
        "plate": cyl(0.26, -0.30, 0.100, task.PLATE_HEIGHT),
        "mug": cyl(0.36, -0.16, 0.0425, 0.095),
    }


def with_(base, **kw):
    w = dict(base)
    w.update(kw)
    return w


def apple_in_bowl():
    return sphere(0.24, 0.26, 0.045, 0.033)


def towels_aside():
    return {
        "towel_sheet": box(0.48, 0.13, 0.001, 0.260, 0.240, 0.0015),
        "towel_pad": box(0.10, 0.12, 0.004, 0.120, 0.110, 0.008),
    }


def knife_on_plate(yaw_deg=-90.0, x=0.26, y=-0.30):
    return knife(x, y, task.PLATE_HEIGHT + task.KNIFE_HEIGHT / 2, quat=qz(yaw_deg))


def final_state():
    return with_(nominal(), apple=apple_in_bowl(), knife=knife_on_plate(), **towels_aside())


def reset():
    task._STATE.clear()
    task._STATE.update(task._fresh_state())


def run(frames):
    reset()
    out = None
    for f in frames:
        out = task.success(f)
    return out


# ---------------------------------------------------------------------------
# Loading and geometry
# ---------------------------------------------------------------------------
def test_objects_cover_the_scored_names():
    names = []
    for o in task.OBJECTS:
        names.append(o.name if REAL else (o[0][0] if o[0] else None))
    assert set(names) == set(L) == set(nominal())


def test_table_height_comes_from_the_scene_entry():
    assert task.TABLE_Z == task.TABLE_TOP[task.SCENE]


def test_knife_asset_has_been_chosen():
    assert task.KNIFE_ASSET != "SET_FROM_ARENA_LIBRARY", "pick the knife from Arena's object library"


def test_knife_frame_points_tip_to_plus_y_at_start():
    tip, butt = task._knife_ends(nominal()["knife"])
    assert tip[1] > butt[1]


def test_straight_carry_would_cross_the_keepout():
    """The design premise: the obvious route from the knife to the plate passes the mug."""
    gx0, gy0 = L["knife"]["xy"][0], L["knife"]["xy"][1] - task.GRASP_FROM_CENTRE
    gx1, gy1 = L["plate"]["xy"][0] - task.GRASP_FROM_CENTRE, L["plate"]["xy"][1]
    worst = math.inf
    for i in range(101):
        t = i / 100
        g = (gx0 + t * (gx1 - gx0), gy0 + t * (gy1 - gy0))
        k = knife(g[0] + task.GRASP_FROM_CENTRE, g[1], 0.10, quat=qz(-90))
        worst = min(worst, task._knife_mug_hdist({"knife": k, "mug": nominal()["mug"]}))
    assert worst < task.TH["keepout_radius"]


# ---------------------------------------------------------------------------
# Tiers
# ---------------------------------------------------------------------------
def test_nominal_layout_scores_zero_and_latches_nothing():
    n = nominal()
    assert run([n, n]) is False
    assert task.score(n) == 0.0
    assert task._STATE["zero"] is None and task._STATE["cap_half"] is None
    assert task._STATE["bad_layout"] is False
    assert task._STATE["apple_bruised"] is False


def test_apple_only():
    w = with_(nominal(), apple=apple_in_bowl())
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_apple_and_towels():
    w = with_(nominal(), apple=apple_in_bowl(), **towels_aside())
    run([nominal(), w])
    assert task.score(w) == 0.5


@pytest.mark.parametrize("yaw", [90, 180, -45])
def test_knife_on_plate_pointing_wrong_way_is_075(yaw):
    w = with_(final_state(), knife=knife_on_plate(yaw_deg=yaw))
    assert run([nominal(), w]) is False
    assert task.score(w) == 0.75


def test_full_success_and_score_capped_at_075():
    assert run([nominal(), final_state()]) is True
    assert task.score(final_state()) == 0.75


@pytest.mark.parametrize("yaw,ok", [(-90 + 24, True), (-90 - 24, True), (-90 + 26, False)])
def test_knife_direction_boundary(yaw, ok):
    assert task._knife_set(with_(final_state(), knife=knife_on_plate(yaw_deg=yaw))) is ok


def test_tiers_do_not_skip_towels():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_on_plate())
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_no_apple_means_zero_even_if_rest_done():
    w = with_(final_state(), apple=sphere(0.10, 0.25, task._APPLE_Z, 0.033))
    run([nominal(), w])
    assert task.score(w) == 0.0


# ---------------------------------------------------------------------------
# Irreversible events and recovery
# ---------------------------------------------------------------------------
def test_mug_tilt_spill_is_permanent():
    tipped = with_(nominal(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(12)))
    assert run([nominal(), tipped, final_state()]) is False
    assert task.score(final_state()) == 0.0


def test_mug_shift_spill_is_permanent():
    pushed = with_(nominal(), mug=cyl(0.36, -0.14, 0.0425, 0.095))
    run([nominal(), pushed, final_state()])
    assert task.score(final_state()) == 0.0


def test_slow_restore_does_not_clear_latch():
    frames = [nominal(), with_(nominal(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(12)))]
    for deg in (9, 6, 3, 0):
        frames.append(with_(nominal(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(deg))))
    frames.append(final_state())
    assert run(frames) is False
    assert task._STATE["zero"] == "coffee spilt"


def test_knife_off_table_is_zero():
    gone = with_(nominal(), knife=knife(0.6, -0.4, -0.3))
    run([nominal(), gone, final_state()])
    assert task.score(final_state()) == 0.0


def test_apple_touching_table_loses_apple_tier_for_good():
    on_table = with_(nominal(), apple=sphere(0.40, 0.10, 0.033, 0.033))
    run([nominal(), on_table, final_state()])
    assert task._STATE["apple_bruised"] is True
    assert task.score(final_state()) == 0.0
    assert task.success(final_state()) is False


def test_apple_low_in_bowl_is_not_a_bruise():
    low = with_(nominal(), apple=sphere(0.24, 0.26, 0.034, 0.033))
    run([nominal(), low])
    assert task._STATE["apple_bruised"] is False


def test_knife_through_keepout_caps_half():
    through = with_(nominal(), knife=knife(0.33, -0.12, 0.10, quat=qz(-90)))
    assert run([nominal(), through, final_state()]) is False
    assert task._STATE["cap_half"] == "knife near the mug"
    assert task.score(final_state()) == 0.5


def test_knife_on_table_is_not_lifted():
    assert task._knife_lifted(nominal()) is False
    assert task._knife_in_keepout(nominal()) is False


def test_dropped_knife_caps_half():
    falling = with_(nominal(), knife=knife(0.12, -0.30, 0.15, quat=qz(-90), vel=(0, 0, -1.2)))
    assert task._knife_in_keepout(falling) is False
    run([nominal(), falling, final_state()])
    assert task._STATE["cap_half"] == "knife dropped"
    assert task.score(final_state()) == 0.5


def test_transient_violation_on_last_frame_is_caught_without_history():
    reset()
    task.success(nominal())
    tipped_final = with_(final_state(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(12)))
    assert task.score(tipped_final) == 0.0


# ---------------------------------------------------------------------------
# Anti-cheat and robustness
# ---------------------------------------------------------------------------
def test_apple_held_above_bowl_does_not_count():
    w = with_(nominal(), apple=sphere(0.24, 0.26, 0.15, 0.033), **towels_aside())
    run([nominal(), w])
    assert task.score(w) == 0.0


def test_towel_dropped_into_bowl_is_not_cleared():
    w = with_(nominal(), apple=apple_in_bowl(),
              towel_sheet=box(0.24, 0.26, 0.02, 0.10, 0.10, 0.0015),
              towel_pad=towels_aside()["towel_pad"])
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_towel_on_plate_is_not_cleared():
    w = with_(nominal(), apple=apple_in_bowl(), towel_pad=towels_aside()["towel_pad"],
              towel_sheet=box(0.26, -0.30, 0.021, 0.26, 0.24, 0.0015))
    assert task._towels_cleared(w) is False


def test_towel_nudged_aside_but_not_cleared():
    w = with_(nominal(), apple=apple_in_bowl(), towel_pad=towels_aside()["towel_pad"],
              towel_sheet=box(0.30, 0.04, 0.001, 0.26, 0.24, 0.0015))
    assert task._towels_cleared(w) is False


def test_knife_held_above_plate_is_not_on_plate():
    w = with_(final_state(), knife=knife(0.26, -0.30, 0.12, quat=qz(-90)))
    run([nominal(), w])
    assert task.score(w) == 0.5


def test_knife_standing_on_plate_is_not_set():
    w = with_(final_state(), knife=knife(0.26, -0.30, task.PLATE_HEIGHT + 0.03, quat=qx(-30)))
    assert task._knife_set(w) is False


def test_moving_objects_block_success():
    k = knife_on_plate()
    k.lin_vel = (0.05, 0, 0)
    assert run([nominal(), with_(final_state(), knife=k)]) is False


def test_bad_initial_layout_fails_closed():
    wrong = with_(nominal(), mug=cyl(0.36, -0.25, 0.0425, 0.095))
    assert run([wrong, final_state()]) is False
    assert task.score(final_state()) == 0.0


@pytest.mark.parametrize("deg,spilt", [(7.9, False), (8.1, True)])
def test_mug_tilt_threshold_boundary(deg, spilt):
    w = with_(nominal(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(deg)))
    assert task._mug_spilt(w) is spilt


@pytest.mark.parametrize("yaw", [90, 180, -135])
def test_mug_yaw_is_not_tilt(yaw):
    w = with_(final_state(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qz(yaw)))
    assert run([nominal(), w]) is True


# ---------------------------------------------------------------------------
# Episode reset and purity
# ---------------------------------------------------------------------------
def test_teleport_back_to_layout_resets_latches():
    tipped = with_(nominal(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(12)))
    far = with_(tipped, apple=sphere(0.60, 0.30, 0.04, 0.033))
    run([nominal(), far])
    assert task._STATE["zero"] == "coffee spilt"
    task.success(nominal())
    assert task._STATE["zero"] is None
    assert task.success(final_state()) is True


def test_score_and_distance_are_read_only():
    run([nominal()])
    snap = repr(sorted(task._STATE.items()))
    tipped = with_(nominal(), mug=cyl(0.36, -0.16, 0.0425, 0.095, quat=qx(20)))
    for _ in range(3):
        task.score(tipped)
        task.distance(tipped)
    assert repr(sorted(task._STATE.items())) == snap


def test_distance_decreases_toward_goal():
    reset()
    assert task.distance(final_state()) < task.distance(nominal())


def test_debug_print_does_not_change_result(capsys, monkeypatch):
    monkeypatch.setattr(task, "DEBUG", True)
    frames = [nominal()] * 21 + [final_state()]
    assert run(frames) is True
    out = capsys.readouterr().out
    assert "task_progress steps=1 " in out and "task_progress steps=20 " in out


# ---------------------------------------------------------------------------
# Demo geometry (no kinematics): replay the waypoints at 20 Hz, interpolating
# the TCP linearly, with a grasped object rigidly attached to the grip.
# ---------------------------------------------------------------------------
def _samples(wps):
    prev_pos, prev_yaw = wps[0].pos, wps[0].yaw
    for w in wps:
        n = max(1, round(w.duration * HZ))
        for i in range(1, n + 1):
            t = i / n
            pos = tuple(prev_pos[j] + t * (w.pos[j] - prev_pos[j]) for j in range(3))
            yield w, pos, prev_yaw + t * (w.yaw - prev_yaw)
        prev_pos, prev_yaw = w.pos, w.yaw


def _carry(wps, start_centre, close_label, release_label):
    """Track an object gripped at the first `close_label` waypoint until the next `release_label`."""
    held, grip, yaw0, centre = False, None, None, start_centre
    path = []
    for w, pos, yaw in _samples(wps):
        if not held and w.label == close_label and grip is None:
            held, grip, yaw0 = True, pos, yaw
        if held:
            dx, dy = start_centre[0] - grip[0], start_centre[1] - grip[1]
            a = yaw - yaw0
            centre = (pos[0] + dx * math.cos(a) - dy * math.sin(a),
                      pos[1] + dx * math.sin(a) + dy * math.cos(a), pos[2])
            path.append((centre, a, pos[2] - grip[2]))
            if w.label == release_label:
                held = False
    return centre, path


def test_demo_knife_ends_on_plate_pointing_away_and_never_nears_the_mug():
    right = task.demo()["right"]
    kx, ky = L["knife"]["xy"]
    centre, path = _carry(right, (kx, ky), "close on the handle", "release")
    assert math.hypot(centre[0] - L["plate"]["xy"][0], centre[1] - L["plate"]["xy"][1]) < 0.005
    _, final_turn, _ = path[-1]
    assert abs(math.degrees(final_turn) + 90) < 1.0  # tip +y turned to +x
    mug = nominal()["mug"]
    closest = math.inf
    for c, a, rise in path:
        if rise <= task.TH["lift"]:
            continue
        k = knife(c[0], c[1], 0.10, quat=qz(math.degrees(a)))
        closest = min(closest, task._knife_mug_hdist({"knife": k, "mug": mug}))
    assert closest > task.TH["keepout_radius"] + 0.01


def test_demo_left_arm_places_apple_and_clears_towels():
    left = task.demo()["left"]
    picks = [w for w in left if w.label == "close"]
    releases = [w for w in left if w.label == "release"]
    assert len(picks) == len(releases) == 3
    a = releases[0].pos
    assert math.hypot(a[0] - L["bowl"]["xy"][0], a[1] - L["bowl"]["xy"][1]) < 0.005
    end = with_(final_state())
    for (name, size), p, r in zip(
        [("towel_sheet", L["towel_sheet"]["size"]), ("towel_pad", L["towel_pad"]["size"])], picks[1:], releases[1:]
    ):
        cx = r.pos[0] + L[name]["xy"][0] - p.pos[0]
        cy = r.pos[1] + L[name]["xy"][1] - p.pos[1]
        end[name] = box(cx, cy, size[2] / 2, *size)
    assert task._towels_cleared(end)


def test_right_arm_waits_until_the_left_arm_is_done():
    plan = task.demo()
    left_done = sum(w.duration for w in plan["left"])
    t, start = 0.0, None
    for w in plan["right"]:
        if w.label == "over the handle":
            start = t
        t += w.duration
    assert start >= left_done


def test_demo_fits_the_run_limit():
    plan = task.demo()
    longest = max(sum(w.duration for w in wps) for wps in plan.values())
    assert longest * HZ < 3600 * 0.5


# ---------------------------------------------------------------------------
# Demo kinematics (real yam_arena only)
# ---------------------------------------------------------------------------
def _worst_ik(wps, arm):
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    plan = ArmPlan(wps, station.BASES[arm])
    for k in range(round(plan.duration * HZ) + 5):
        plan.action7((k + 1) / HZ)
    return plan.worst_ik_mm, plan.worst_ik_deg


@pytest.mark.skipif(not REAL, reason="needs the real yam_arena")
@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_ik_is_exact(arm):
    mm, deg = _worst_ik(task.demo()[arm], arm)
    assert mm < 0.5 and deg < 0.1, (mm, deg)


@pytest.mark.skipif(not REAL, reason="needs the real yam_arena")
@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_keeps_1cm_of_reach_in_hand(arm):
    raised = [dataclasses.replace(w, pos=(w.pos[0], w.pos[1], w.pos[2] + 0.01)) for w in task.demo()[arm]]
    mm, deg = _worst_ik(raised, arm)
    assert mm < 0.5 and deg < 0.1, (mm, deg)
