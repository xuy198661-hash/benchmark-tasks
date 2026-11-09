"""CPU tests for the scoring logic and object list. Uses the real yam_arena if
it is installed; otherwise a minimal stub with the same constructor
signatures. Kinematic tests are added together with demo()."""

import importlib.util
import math
import pathlib
import sys
import types

import pytest

try:
    import yam_arena.task_file  # noqa: F401
    REAL_FRAMEWORK = True
except ImportError:
    REAL_FRAMEWORK = False

    class _Spec:
        def __init__(self, name, *args, **kwargs):
            self.name = name
            self.args = args
            self.__dict__.update(kwargs)

    tf = types.ModuleType("yam_arena.task_file")
    tf.TABLE_TOP = {"station": 0.7475, "tabletop": 0.75}
    for _n in ("Box", "Cylinder", "Sphere", "Asset"):
        setattr(tf, _n, type(_n, (_Spec,), {}))
    pkg = types.ModuleType("yam_arena")
    pkg.task_file = tf
    sys.modules["yam_arena"] = pkg
    sys.modules["yam_arena.task_file"] = tf

HERE = pathlib.Path(__file__).parent
_spec = importlib.util.spec_from_file_location("task_under_test", HERE / "task.py")
task = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(task)
T0 = task.TABLE_Z
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

    def resting(self, max_speed=0.01, max_spin=0.5):
        return math.hypot(*self.lin_vel) < max_speed and math.hypot(*self.ang_vel) < max_spin


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def from_to(a, d):
    """Shortest-arc quaternion (xyzw) turning unit vector a onto unit vector d."""
    n = math.sqrt(sum(c * c for c in d))
    d = tuple(c / n for c in d)
    dot = sum(x * y for x, y in zip(a, d))
    if dot < -0.999999:
        perp = (0.0, 0.0, 1.0) if abs(a[2]) < 0.9 else (1.0, 0.0, 0.0)
        ax = (a[1] * perp[2] - a[2] * perp[1], a[2] * perp[0] - a[0] * perp[2], a[0] * perp[1] - a[1] * perp[0])
        m = math.sqrt(sum(c * c for c in ax))
        return (ax[0] / m, ax[1] / m, ax[2] / m, 0.0)
    c = (a[1] * d[2] - a[2] * d[1], a[2] * d[0] - a[0] * d[2], a[0] * d[1] - a[1] * d[0])
    q = (c[0], c[1], c[2], 1.0 + dot)
    m = math.sqrt(sum(v * v for v in q))
    return tuple(v / m for v in q)


def box(x, y, z, sx, sy, sz, quat=ID):
    return S((x, y, T0 + z), quat, (x - sx / 2, y - sy / 2, T0 + z - sz / 2), (x + sx / 2, y + sy / 2, T0 + z + sz / 2))


def cyl(x, y, r, h, quat=ID):
    return S((x, y, T0 + h / 2), quat, (x - r, y - r, T0), (x + r, y + r, T0 + h))


def sphere(x, y, z, r, vel=(0, 0, 0)):
    return S((x, y, T0 + z), ID, (x - r, y - r, T0 + z - r), (x + r, y + r, T0 + z + r), lin_vel=vel)


def knife(x, y, z, tip_dir=(0.0, 1.0, 0.0), vel=(0, 0, 0)):
    """A knife whose tip points along tip_dir, whatever the asset's own axis is."""
    c = (x, y, T0 + z)
    o = S(c, from_to(task.KNIFE_LOCAL_AXIS, tip_dir), c, c, lin_vel=vel)
    tip, butt = task._knife_ends(o)
    pad = 0.008
    o.low = tuple(min(tip[i], butt[i]) - pad for i in range(3))
    o.high = tuple(max(tip[i], butt[i]) + pad for i in range(3))
    return o


def knife_leaning(tip, toward_y=-1.0, tilt_deg=30.0):
    """Knife with its tip at `tip` (z above table) and its handle rising at tilt_deg from vertical."""
    s, c = math.sin(math.radians(tilt_deg)), math.cos(math.radians(tilt_deg))
    d = (0.0, -toward_y * s, -c)  # centre -> tip
    h = task.LAYOUT["knife"]["length"] / 2
    return knife(tip[0] - d[0] * h, tip[1] - d[1] * h, tip[2] - d[2] * h, tip_dir=d)


def nominal():
    kz = task.KNIFE_HEIGHT / 2
    return {
        "knife": knife(0.26, 0.00, kz),
        "towel_pad": box(0.26, 0.045, task._PAD_Z - 0.002, 0.120, 0.110, 0.008),
        "towel_sheet": box(0.26, 0.00, task._SHEET_Z - 0.003, 0.260, 0.220, 0.0015),
        "apple": sphere(0.26, 0.04, task._APPLE_Z - 0.006, 0.040),
        "bowl": cyl(0.24, 0.26, 0.075, 0.065),
        "mug": cyl(0.24, -0.17, 0.0425, 0.095),
        "holder": cyl(0.22, -0.30, 0.036, 0.085),
    }


def with_(base, **kw):
    w = dict(base)
    w.update(kw)
    return w


def apple_in_bowl():
    return sphere(0.24, 0.26, 0.045, 0.040)


def towels_aside():
    return {
        "towel_sheet": box(0.47, 0.10, 0.001, 0.260, 0.220, 0.0015),
        "towel_pad": box(0.10, 0.12, 0.004, 0.120, 0.110, 0.008),
    }


def knife_set_correctly():
    return knife_leaning((0.22, -0.31, 0.020), toward_y=-1.0)


def knife_upright_in_holder():
    return knife(0.22, -0.30, 0.020 + 0.085, tip_dir=(0, 0, -1))


def final_state():
    return with_(nominal(), apple=apple_in_bowl(), knife=knife_set_correctly(), **towels_aside())


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
# Object list and constants
# ---------------------------------------------------------------------------
def test_objects_match_layout_and_avoid_reserved_names():
    names = [o.name for o in task.OBJECTS]
    assert sorted(names) == sorted(task.LAYOUT)
    reserved = {"table", "floor", "ground_plane", "dome_light", "lamp", "d435", "camera_mast", "stand_foot",
                "stand_mast", "stand_arm", "camera_housing", "left_robot", "right_robot", "top_cam", "left_cam",
                "right_cam"}
    assert not reserved & set(names)
    assert all(n.isidentifier() for n in names)


def test_only_the_bowl_is_fixed():
    fixed = sorted(o.name for o in task.OBJECTS if getattr(o, "mass", None) is None)
    assert fixed == ["bowl"]


def test_stacked_objects_start_above_what_they_rest_on():
    by = {o.name: o for o in task.OBJECTS}
    pad_bottom = by["towel_pad"].pos[2] - by["towel_pad"].size[2] / 2
    sheet_bottom = by["towel_sheet"].pos[2] - by["towel_sheet"].size[2] / 2
    pad_top = by["towel_pad"].pos[2] + by["towel_pad"].size[2] / 2
    apple_bottom = by["apple"].pos[2] - by["apple"].radius
    sheet_top = by["towel_sheet"].pos[2] + by["towel_sheet"].size[2] / 2
    assert pad_bottom > T0 + task.KNIFE_HEIGHT
    assert sheet_bottom > pad_top
    assert apple_bottom > sheet_top


def test_sheet_covers_the_knife_and_apple_sits_over_the_pad():
    sx, sy, _ = task.LAYOUT["towel_sheet"]["size"]
    zl, zh = task.KNIFE_ZONE
    cx, cy = task.LAYOUT["towel_sheet"]["xy"]
    assert cx - sx / 2 <= zl[0] and zh[0] <= cx + sx / 2
    assert cy - sy / 2 <= zl[1] and zh[1] <= cy + sy / 2
    px, py = task.LAYOUT["towel_pad"]["xy"]
    psx, psy, _ = task.LAYOUT["towel_pad"]["size"]
    ax, ay = task.LAYOUT["apple"]["xy"]
    assert abs(ax - px) < psx / 2 and abs(ay - py) < psy / 2


def test_knife_yaw_turns_asset_axis_to_plus_y():
    q = qz(math.degrees(task.KNIFE_YAW))
    v = task._rotate(q, task.KNIFE_LOCAL_AXIS)
    assert abs(v[0]) < 1e-9 and abs(v[1] - 1) < 1e-9


def test_table_height_comes_from_the_scene_entry():
    assert task.TABLE_Z == task.TABLE_TOP[task.SCENE]


@pytest.mark.skipif("TODO" in task.KNIFE_ASSET + task.HOLDER_ASSET, reason="library asset names not filled in yet")
def test_asset_names_are_filled():
    assert "TODO" not in task.KNIFE_ASSET + task.HOLDER_ASSET


# ---------------------------------------------------------------------------
# Helper geometry
# ---------------------------------------------------------------------------
def test_final_knife_geometry_is_what_the_test_claims():
    k = knife_set_correctly()
    tip, butt = task._knife_ends(k)
    assert tip[2] < butt[2]
    assert abs(task._knife_axis_tilt_deg(k) - 30) < 0.5
    assert task._tip_in_holder(final_state())


def test_knife_tip_points_left_at_start():
    tip, butt = task._knife_ends(nominal()["knife"])
    assert tip[1] > butt[1] and abs(tip[2] - butt[2]) < 1e-9


def test_holder_geometry_is_read_from_its_bounds():
    w = nominal()
    assert abs(task._holder_rim(w) - 0.085) < 1e-9
    assert abs(task._holder_inner_radius(w) - (0.036 - task.TH["holder_wall"])) < 1e-9


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
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife(0.30, -0.05, 0.01), **towels_aside())
    run([nominal(), w])
    assert task.score(w) == 0.5


def test_knife_upright_in_holder_is_075_not_success():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_upright_in_holder(), **towels_aside())
    assert run([nominal(), w]) is False
    assert task.score(w) == 0.75


def test_full_success_and_score_capped_at_075():
    assert run([nominal(), final_state()]) is True
    assert task.score(final_state()) == 0.75


def test_tiers_do_not_skip_towels():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_set_correctly())
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_no_apple_means_zero_even_if_rest_done():
    w = with_(final_state(), apple=sphere(0.10, 0.20, 0.10, 0.040))
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


def test_holder_knocked_over_then_righted_is_zero():
    fallen = with_(nominal(), holder=cyl(0.22, -0.30, 0.036, 0.085, quat=qx(80)))
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


def test_pulling_the_knife_out_from_under_drops_the_apple():
    knife_gone = knife(0.18, -0.25, 0.10)
    apple_fell = sphere(0.27, 0.03, 0.040, 0.040)
    run([nominal(), with_(nominal(), knife=knife_gone, apple=apple_fell), final_state()])
    assert task._STATE["apple_bruised"] is True
    assert task.score(final_state()) == 0.0


def test_knife_carried_through_keepout_caps_half():
    through = with_(nominal(), knife=knife(0.24, -0.12, 0.10))
    assert run([nominal(), through, final_state()]) is False
    assert task._STATE["cap_half"] == "knife near the mug"
    assert task.score(final_state()) == 0.5


def test_knife_route_behind_mug_is_clean():
    behind = with_(nominal(), knife=knife(0.15, -0.17, 0.10))
    assert run([nominal(), behind, final_state()]) is True


def test_knife_lying_on_counter_never_counts_as_keepout():
    assert task._knife_in_keepout(nominal()) is False


def test_knife_standing_in_holder_and_leaning_toward_mug_is_not_carried():
    lean = knife_leaning((0.22, -0.30, 0.030), toward_y=+1.0)
    w = with_(final_state(), knife=lean)
    assert task._knife_lifted(w) and task._tip_in_holder(w)
    assert task._knife_in_keepout(w) is False


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
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife(0.30, -0.05, 0.01),
              towel_sheet=box(0.24, 0.26, 0.02, 0.10, 0.10, 0.0015), towel_pad=towels_aside()["towel_pad"])
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_towel_left_on_mug_is_not_cleared():
    w = with_(nominal(), towel_sheet=box(0.24, -0.17, 0.096, 0.26, 0.22, 0.0015))
    assert task._towels_cleared(w) is False


def test_towels_left_where_the_knife_was_are_not_cleared():
    w = with_(nominal(), apple=apple_in_bowl(), knife=knife_set_correctly(),
              towel_sheet=box(0.26, 0.00, 0.001, 0.26, 0.22, 0.0015),
              towel_pad=box(0.26, 0.045, 0.004, 0.12, 0.11, 0.008))
    run([nominal(), w])
    assert task.score(w) == 0.25


def test_knife_handle_down_is_not_in_holder():
    upside = knife(0.22, -0.30, 0.020 + 0.085, tip_dir=(0, 0, 1))
    w = with_(final_state(), knife=upside)
    run([nominal(), w])
    assert task.score(w) == 0.5


def test_moving_objects_block_success():
    k = knife_set_correctly()
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
    w = with_(final_state(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qz(yaw)),
              holder=cyl(0.22, -0.30, 0.036, 0.085, quat=qz(yaw)))
    assert run([nominal(), w]) is True


# ---------------------------------------------------------------------------
# Episode reset and purity
# ---------------------------------------------------------------------------
def test_teleport_back_to_layout_resets_latches():
    tipped = with_(nominal(), mug=cyl(0.24, -0.17, 0.0425, 0.095, quat=qx(12)))
    far = with_(tipped, apple=sphere(0.60, 0.30, 0.10, 0.04))
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
    assert "task_progress steps=1 " in out and "knife_tip=" in out and "task_progress steps=20 " in out
