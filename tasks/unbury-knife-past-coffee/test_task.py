"""CPU tests for the scoring logic. Runs against the real yam_arena if it is
installed; otherwise a minimal stub provides TABLE_TOP. Kinematic tests are
added together with demo()."""

import importlib.util
import math
import pathlib
import sys
import types

import pytest

try:
    import yam_arena.task_file  # noqa: F401
except ImportError:
    pkg = types.ModuleType("yam_arena")
    tf = types.ModuleType("yam_arena.task_file")
    tf.TABLE_TOP = {"station": 0.7475, "tabletop": 0.75}
    pkg.task_file = tf
    sys.modules["yam_arena"] = pkg
    sys.modules["yam_arena.task_file"] = tf

HERE = pathlib.Path(__file__).parent
_spec = importlib.util.spec_from_file_location("task_under_test", HERE / "task.py")
task = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(task)
T0 = task.TABLE_Z
L = task.LAYOUT
ID = (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# Stub ObjectState with the same public fields
# ---------------------------------------------------------------------------
class S:
    def __init__(self, pos, quat=ID, low=None, high=None, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
        self.pos = tuple(pos)
        self.quat_xyzw = tuple(quat)
        self.low = tuple(low)
        self.high = tuple(high)
        self.lin_vel = tuple(lin_vel)
        self.ang_vel = tuple(ang_vel)

    def resting(self, max_speed, max_spin):
        return math.hypot(*self.lin_vel) <= max_speed and math.hypot(*self.ang_vel) <= max_spin


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def box(x, y, z, sx, sy, sz, quat=ID, vel=(0, 0, 0)):
    return S((x, y, T0 + z), quat, (x - sx / 2, y - sy / 2, T0 + z - sz / 2),
             (x + sx / 2, y + sy / 2, T0 + z + sz / 2), lin_vel=vel)


def cyl(x, y, r, h, quat=ID, z0=0.0):
    return S((x, y, T0 + z0 + h / 2), quat, (x - r, y - r, T0 + z0), (x + r, y + r, T0 + z0 + h))


def sphere(x, y, z, r, vel=(0, 0, 0)):
    return S((x, y, T0 + z), ID, (x - r, y - r, T0 + z - r), (x + r, y + r, T0 + z + r), lin_vel=vel)


def knife(x, y, z, quat=ID, vel=(0, 0, 0)):
    c = (x, y, T0 + z)
    o = S(c, quat, c, c, lin_vel=vel)
    tip, butt = task._knife_ends(o)
    pad = 0.008
    o.low = tuple(min(tip[i], butt[i]) - pad for i in range(3))
    o.high = tuple(max(tip[i], butt[i]) + pad for i in range(3))
    return o


def nominal():
    return {
        "plate": cyl(0.26, 0.00, 0.110, 0.010),
        "knife": knife(0.26, 0.00, 0.017),
        "towel_pad": box(0.26, 0.04, 0.025, 0.120, 0.110, 0.008),
        "towel_sheet": box(0.26, 0.00, 0.035, 0.260, 0.240, 0.0015),
        "apple": sphere(0.26, 0.02, 0.076, 0.040),
        "bowl": cyl(0.24, 0.26, 0.075, 0.065),
        "mug": cyl(0.24, -0.17, 0.0425, 0.095),
        "glass": cyl(0.22, -0.30, 0.036, 0.085),
    }


def knife_in_glass_correct():
    # tip near the glass floor, axis 30 deg from vertical, handle up and to the left
    tip = (0.22, -0.31, 0.020)
    d = (0.0, -math.sin(math.radians(30)), -math.cos(math.radians(30)))  # centre -> tip
    c = (tip[0] - d[0] * 0.085, tip[1] - d[1] * 0.085, tip[2] - d[2] * 0.085)
    return knife(*c, quat=qx(-120))


def knife_upright_in_glass():
    return knife(0.22, -0.30, 0.020 + 0.085, quat=qx(-90))


def with_(base, **kw):
    w = dict(base)
    w.update(kw)
    return w


def apple_in_bowl():
    return sphere(0.24, 0.26, 0.045, 0.040)


def towels_aside():
    return {
        # free table: in front of the plate on the left, and near the robot on the left
        "towel_sheet": box(0.50, 0.15, 0.001, 0.260, 0.240, 0.0015),
        "towel_pad": box(0.10, 0.12, 0.004, 0.120, 0.110, 0.008),
    }


def final_state():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_in_glass_correct(), **towels_aside())
    return w


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
# Sanity of the helper geometry
# ---------------------------------------------------------------------------
def test_final_knife_geometry_is_what_the_test_claims():
    k = knife_in_glass_correct()
    tip, butt = task._knife_ends(k)
    assert tip[2] < butt[2]
    assert abs(task._knife_axis_tilt_deg(k) - 30) < 0.5
    assert math.hypot(tip[0] - 0.22, tip[1] + 0.30) < L["glass"]["inner_radius"]


def test_knife_frame_points_tip_to_plus_y_at_start():
    tip, butt = task._knife_ends(nominal()["knife"])
    assert tip[1] > butt[1]


# ---------------------------------------------------------------------------
# Tiers
# ---------------------------------------------------------------------------
def test_nominal_layout_scores_zero_and_latches_nothing():
    n = nominal()
    assert run([n, n]) is False
    assert task.score(n) == 0.0
    assert task._STATE["zero"] is None and task._STATE["cap_half"] is None
    assert task._STATE["bad_layout"] is False


def test_apple_only():
    w = with_(nominal(), apple=apple_in_bowl())
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_apple_and_towels():
    w = with_(nominal(), apple=apple_in_bowl(), **towels_aside())
    run([nominal(), w])
    assert task.score(w) == 0.5


def test_knife_in_glass_but_upright_is_075_not_success():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_upright_in_glass(), **towels_aside())
    assert run([nominal(), w]) is False
    assert task.score(w) == 0.75


def test_full_success_and_score_capped_at_075():
    assert run([nominal(), final_state()]) is True
    assert task.score(final_state()) == 0.75


def test_tiers_do_not_skip_towels():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_in_glass_correct())
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_no_apple_means_zero_even_if_rest_done():
    w = with_(final_state(), apple=sphere(0.10, 0.20, 0.076, 0.040))
    run([nominal(), w])
    assert task.score(w) == 0.0


# ---------------------------------------------------------------------------
# Irreversible events and recovery
# ---------------------------------------------------------------------------
def test_mug_tilt_spill_is_permanent():
    tipped = with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(12)))
    assert run([nominal(), tipped, final_state()]) is False
    assert task.score(final_state()) == 0.0


def test_mug_shift_spill_is_permanent():
    pushed = with_(nominal(), mug=cyl(0.24, -0.15, 0.0425, 0.095))
    run([nominal(), pushed, final_state()])
    assert task.score(final_state()) == 0.0


def test_slow_restore_does_not_clear_latch():
    frames = [nominal(), with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(12)))]
    for deg in (9, 6, 3, 0):
        frames.append(with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(deg))))
    frames.append(final_state())
    assert run(frames) is False
    assert task._STATE["zero"] == "coffee spilt"


def test_glass_knocked_over_then_righted_is_zero():
    fallen = with_(nominal(), glass=cyl(0.22, -0.30, 0.036, 0.085, quat=qx(80)))
    run([nominal(), fallen, final_state()])
    assert task.score(final_state()) == 0.0


def test_knife_off_table_is_zero():
    gone = with_(nominal(), knife=knife(0.6, -0.4, -0.3))
    run([nominal(), gone, final_state()])
    assert task.score(final_state()) == 0.0


def test_apple_touching_table_loses_apple_tier_for_good():
    on_table = with_(nominal(), apple=sphere(0.40, 0.10, 0.040, 0.040))
    run([nominal(), on_table, final_state()])
    assert task.score(final_state()) == 0.0
    assert task.success(final_state()) is False


def test_knife_through_keepout_caps_half():
    through = with_(nominal(), knife=knife(0.24, -0.12, 0.10))
    assert run([nominal(), through, final_state()]) is False
    assert task._STATE["cap_half"] == "knife near the mug"
    assert task.score(final_state()) == 0.5


def test_knife_route_behind_mug_is_clean():
    behind = with_(nominal(), knife=knife(0.15, -0.17, 0.10))
    assert run([nominal(), behind, final_state()]) is True


def test_knife_on_plate_never_counts_as_keepout():
    assert task._knife_in_keepout(nominal()) is False


def test_dropped_knife_caps_half():
    falling = with_(nominal(), knife=knife(0.12, -0.30, 0.15, vel=(0, 0, -1.2)))
    assert task._knife_in_keepout(falling) is False
    run([nominal(), falling, final_state()])
    assert task._STATE["cap_half"] == "knife dropped"
    assert task.score(final_state()) == 0.5


def test_transient_violation_on_last_frame_is_caught_without_history():
    reset()
    task.success(nominal())
    tipped_final = with_(final_state(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(12)))
    assert task.score(tipped_final) == 0.0


# ---------------------------------------------------------------------------
# Anti-cheat and robustness
# ---------------------------------------------------------------------------
def test_apple_held_above_bowl_does_not_count():
    w = with_(nominal(), apple=sphere(0.24, 0.26, 0.15, 0.040), **towels_aside())
    run([nominal(), w])
    assert task.score(w) == 0.0


def test_towel_dropped_into_bowl_is_not_cleared():
    w = with_(nominal(), apple=apple_in_bowl(),
              towel_sheet=box(0.24, 0.26, 0.02, 0.10, 0.10, 0.0015),
              towel_pad=towels_aside()["towel_pad"])
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_towel_left_on_mug_is_not_cleared():
    w = with_(nominal(), apple=apple_in_bowl(),
              towel_sheet=box(0.24, -0.17, 0.096, 0.26, 0.24, 0.0015),
              towel_pad=towels_aside()["towel_pad"])
    assert task._towels_cleared(w) is False


def test_knife_handle_down_is_not_in_glass():
    upside = knife(0.22, -0.30, 0.020 + 0.085, quat=qx(90))
    w = with_(final_state(), knife=upside)
    run([nominal(), w])
    assert task.score(w) == 0.5


def test_moving_objects_block_success():
    k = knife_in_glass_correct()
    k.lin_vel = (0.05, 0, 0)
    assert run([nominal(), with_(final_state(), knife=k)]) is False


def test_bad_initial_layout_fails_closed():
    wrong = with_(nominal(), mug=cyl(0.24, -0.25, 0.0425, 0.095))
    assert run([wrong, final_state()]) is False
    assert task.score(final_state()) == 0.0


@pytest.mark.parametrize("deg,spilt", [(7.9, False), (8.1, True)])
def test_mug_tilt_threshold_boundary(deg, spilt):
    w = with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(deg)))
    assert task._mug_spilt(w) is spilt


@pytest.mark.parametrize("yaw", [90, 180, -135])
def test_yaw_is_not_tilt(yaw):
    w = with_(final_state(),
              mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qz(yaw)),
              glass=cyl(0.22, -0.30, 0.036, 0.085, quat=qz(yaw)))
    assert run([nominal(), w]) is True


# ---------------------------------------------------------------------------
# Episode reset and purity
# ---------------------------------------------------------------------------
def test_teleport_back_to_layout_resets_latches():
    tipped = with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(12)))
    far = with_(tipped, apple=sphere(0.60, 0.30, 0.04, 0.04))
    run([nominal(), far])
    assert task._STATE["zero"] == "coffee spilt"
    task.success(nominal())
    assert task._STATE["zero"] is None
    assert task.success(final_state()) is True


def test_score_and_distance_are_read_only():
    run([nominal()])
    snap = repr(sorted(task._STATE.items()))
    tipped = with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(20)))
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


def test_table_height_comes_from_the_scene_entry():
    assert task.TABLE_Z == task.TABLE_TOP[task.SCENE]


def test_layout_names_match_scoring_names():
    assert set(L) == set(nominal())
