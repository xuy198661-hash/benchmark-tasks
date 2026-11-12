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


def mug(ax, ay, quat=ID):
    """Library mug as the task assumes it: pos at the bounding-box centre, handle along body +x."""
    h = task.MUG_RIM_HEIGHT
    off = task._rotate(quat, task.MUG_AXIS_FROM_POS)
    c = (ax - off[0], ay - off[1], T0 + h / 2)
    r = task.MUG_BODY_RADIUS
    return S(c, quat, (ax - r, ay - r, T0), (ax + r + 2 * task.MUG_HANDLE_OFFSET, ay + r, T0 + h))


def nominal():
    return {
        "knife": knife(0.26, 0.00, task.KNIFE_HEIGHT / 2),
        "towel_pad": box(*L["towel_pad"]["xy"], task._PAD_Z - task.SPAWN_GAP, *L["towel_pad"]["size"]),
        "towel_sheet": box(*L["towel_sheet"]["xy"], task._SHEET_Z - 2 * task.SPAWN_GAP, *L["towel_sheet"]["size"]),
        "apple": sphere(*L["apple"]["xy"], task._APPLE_Z - 3 * task.SPAWN_GAP, 0.033),
        "bowl": cyl(0.24, 0.26, 0.080, 0.055),
        "plate": cyl(0.26, -0.30, 0.100, task.PLATE_HEIGHT),
        "mug": mug(0.32, -0.16),
    }


def with_(base, **kw):
    w = dict(base)
    w.update(kw)
    return w


def apple_in_bowl():
    return sphere(0.24, 0.26, 0.045, 0.033)


def towels_aside():
    """Where the scripted demo leaves the towels (see test_demo_left_arm_places_apple_and_clears_towels)."""
    sheet, pad = L["towel_sheet"]["size"], L["towel_pad"]["size"]
    return {
        "towel_sheet": box(0.45, 0.18, sheet[2] / 2, *sheet),
        "towel_pad": box(0.10, 0.23, pad[2] / 2, *pad),
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


def test_mug_carries_a_full_cup_and_there_is_no_coffee_body():
    specs = {o.name if REAL else o[0][0]: o for o in task.OBJECTS}
    assert "coffee" not in specs and "coffee" not in L
    assert task.MUG_MASS == pytest.approx(0.55)
    if not REAL:  # the stub keeps the constructor arguments; the real Asset's fields are not part of the contract
        assert specs["mug"][1]["mass"] == task.MUG_MASS


def test_mug_measurements_record_their_source():
    info = task.MUG_INFO
    assert info["source_url"].endswith("/IsaacLab/Objects/Mug/mug.usd") and len(info["source_sha256"]) == 64
    assert info["mug_extent_m"] == pytest.approx(0.120)
    assert 0.0 < task.MUG_BODY_RADIUS < task.MUG_EXTENT / 2 and task.MUG_HANDLE_OFFSET > 0
    assert (task.HERE / "assets/mug/make_mug_info.py").is_file()


def test_textures_carry_no_embedded_metadata():
    """Double-blind: no text, EXIF or content-credential chunks (they can name tools, people or paths)."""
    import struct
    textures = sorted((task.HERE / "assets").rglob("*.png"))
    assert textures
    for path in textures:
        data, i, found = path.read_bytes(), 8, []
        while i < len(data):
            n, = struct.unpack(">I", data[i:i + 4])
            kind = data[i + 4:i + 8].decode("latin-1")
            if kind in ("tEXt", "iTXt", "zTXt", "eXIf", "caBX"):
                found.append(kind)
            i += 12 + n
        assert not found, (path.name, found)


def test_task_does_not_replace_framework_functions():
    """Only the public task-file contract: no framework attribute is reassigned (no monkeypatching)."""
    source = (HERE / "task.py").read_text()
    assert "station." not in source and "placement" not in source


def test_table_height_comes_from_the_scene_entry():
    assert task.TABLE_Z == task.TABLE_TOP[task.SCENE]


def _usda_points(path):
    """All point3f values of the visual mesh in a .usda (parsed as text; no pxr needed)."""
    import re
    text = path.read_text()
    block = text.split('def Mesh "visual"', 1)[1].split("point3f[] points = [", 1)[1].split("]", 1)[0]
    return [tuple(map(float, t)) for t in re.findall(r"\(([-\d.e]+), ([-\d.e]+), ([-\d.e]+)\)", block)]


def test_local_asset_files_exist():
    """Every model registered from assets/ is present."""
    missing = [rel for rel in task.LOCAL_ASSETS.values() if not (task.HERE / rel).is_file()]
    assert not missing, f"missing model files: {missing}"


def test_knife_usd_matches_the_constants():
    path = task.HERE / task.LOCAL_ASSETS[task.KNIFE_ASSET]
    pts = _usda_points(path)
    assert len(pts) > 1000
    lo = [min(p[i] for p in pts) for i in range(3)]
    hi = [max(p[i] for p in pts) for i in range(3)]
    assert abs((hi[1] - lo[1]) - L["knife"]["length"]) < 1e-4, "length along y"
    assert hi[1] - lo[1] > 5 * max(hi[0] - lo[0], hi[2] - lo[2]), "long axis is y"
    assert abs((hi[2] - lo[2]) - task.KNIFE_HEIGHT) < 5e-4, "height"
    assert all(abs(lo[i] + hi[i]) < 1e-5 for i in range(3)), "bounding-box centre at the origin"
    assert abs(hi[1] - task.KNIFE_TIP_LOCAL[1]) < 1e-4 and abs(lo[1] - task.KNIFE_BUTT_LOCAL[1]) < 1e-4
    assert lo[1] < task.KNIFE_HANDLE_END_Y < 0 < hi[1]
    text = path.read_text()
    assert "PhysicsRigidBodyAPI" in text and text.count("PhysicsCollisionAPI") == 3
    assert (path.parent / "knife_texture.png").is_file()


def _knife_bounds():
    """Body-frame bounding box of the knife's visual mesh (the model is not rotated at spawn)."""
    pts = _usda_points(task.HERE / task.LOCAL_ASSETS[task.KNIFE_ASSET])
    return [min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]


def _rect(name):
    (x, y), (sx, sy, _) = L[name]["xy"], L[name]["size"]
    return x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2


def test_wad_lies_on_the_table_beside_the_knife():
    """The wad must not rest on the knife: a rigid plate on a ridge tilts (4-6 degrees in the GPU run)."""
    assert task.KNIFE_SPAWN_YAW == 0.0
    lo, _ = _knife_bounds()
    _, pad_x1, _, _ = _rect("towel_pad")
    assert L["knife"]["xy"][0] + lo[0] - pad_x1 >= 0.003
    assert task._PAD_Z - L["towel_pad"]["size"][2] / 2 == pytest.approx(task.SPAWN_GAP)


def test_sheet_clears_the_knife():
    """The sheet rests on the wad only; the wad is thicker than the knife with 3 mm to spare."""
    assert L["towel_pad"]["size"][2] - task.KNIFE_HEIGHT >= 0.003


def test_apple_and_sheet_are_carried_by_the_wad_alone():
    """Every support under the apple lies on the table, so it is level by construction. The apple
    and the sheet's centre of mass sit over the wad with at least 20 mm to spare on every side."""
    x0, x1, y0, y1 = _rect("towel_pad")
    for name in ("apple", "towel_sheet"):
        x, y = L[name]["xy"]
        assert min(x - x0, x1 - x, y - y0, y1 - y) >= 0.020, name


def test_spawn_stack_has_no_initial_overlap():
    pad_top = task._PAD_Z + L["towel_pad"]["size"][2] / 2
    sheet_half = L["towel_sheet"]["size"][2] / 2
    assert task._SHEET_Z - sheet_half - pad_top == pytest.approx(task.SPAWN_GAP)
    assert task._APPLE_Z - L["apple"]["radius"] - (task._SHEET_Z + sheet_half) == pytest.approx(task.SPAWN_GAP)


def test_sheet_hides_the_blade_and_shows_the_handle():
    lo, hi = _knife_bounds()
    kx, ky = L["knife"]["xy"]
    x0, x1, y0, y1 = _rect("towel_sheet")
    assert x0 + 0.010 <= kx + lo[0] and kx + hi[0] <= x1 - 0.010, "blade covered across its width"
    assert y1 - 0.010 >= ky + task.KNIFE_TIP_LOCAL[1], "tip covered"
    assert y0 <= ky + task.KNIFE_HANDLE_END_Y - 0.005, "the metal starts under the sheet"
    assert y0 - (ky + task.KNIFE_BUTT_LOCAL[1]) >= 0.040, "at least 40 mm of handle in view"
    assert y0 - (ky - task.GRASP_FROM_CENTRE) >= 0.015, "the grip is clear of the sheet"


def test_towels_start_clear_of_bowl_mug_and_plate():
    for name in ("towel_sheet", "towel_pad"):
        x0, x1, y0, y1 = _rect(name)
        for xy, r in ((L["bowl"]["xy"], L["bowl"]["extent"] / 2 + 0.010),
                      (L["mug"]["axis_xy"], task.TH["keepout_radius"]),
                      (L["plate"]["xy"], L["plate"]["radius"] + 0.010)):
            assert not task._rect_hits_disc((x0, y0), (x1, y1), xy, r), (name, xy)


@pytest.mark.parametrize("name,width,min_depth", [("bowl", 0.160, 0.040), ("plate", 0.200, 0.010)])
def test_container_usd_has_portable_texture_and_open_collision(name, width, min_depth):
    """The actual USD collider must retain the cavity under the opening, at the scene's scale."""
    pytest.importorskip("pxr")
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    stage = Usd.Stage.Open(str(task.HERE / task.LOCAL_ASSETS[f"{name}_ycb_local"]))
    root = stage.GetDefaultPrim()
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    assert UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
    assert UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get() is True
    bounds = UsdGeom.BBoxCache(0, ["default"]).ComputeWorldBound(root).ComputeAlignedRange()
    assert max(bounds.GetSize()[:2]) == pytest.approx(width, abs=1e-6)
    assert all(abs(a + b) < 1e-6 for a, b in zip(bounds.GetMin(), bounds.GetMax()))

    visual = UsdGeom.Mesh(stage.GetPrimAtPath(f"{root.GetPath()}/visual"))
    material, _ = UsdShade.MaterialBindingAPI(visual).ComputeBoundMaterial()
    assert material
    texture = stage.GetPrimAtPath(f"{material.GetPath()}/Texture").GetAttribute("inputs:file").Get()
    assert not pathlib.Path(texture.path).is_absolute()
    assert pathlib.Path(texture.resolvedPath).is_file()
    uv = UsdGeom.PrimvarsAPI(visual).GetPrimvar("st")
    assert len(uv.ComputeFlattened()) == len(visual.GetFaceVertexIndicesAttr().Get())
    assert len(visual.GetNormalsAttr().Get()) == len(uv.ComputeFlattened())

    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert colliders
    centre_hits = []
    for prim in colliders:
        assert UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "none"
        mesh = UsdGeom.Mesh(prim)
        assert set(mesh.GetFaceVertexCountsAttr().Get()) == {3}
        points = np.asarray(mesh.GetPointsAttr().Get())
        faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        triangles = points[faces].astype(float)
        # A ray down the opening must hit the container floor, below its rim.
        u = triangles[:, 1, :2] - triangles[:, 0, :2]
        v = triangles[:, 2, :2] - triangles[:, 0, :2]
        q = -triangles[:, 0, :2]
        det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
        valid = abs(det) > 1e-14
        denominator = np.where(valid, det, 1.0)
        b = (q[:, 0] * v[:, 1] - q[:, 1] * v[:, 0]) / denominator
        c = (u[:, 0] * q[:, 1] - u[:, 1] * q[:, 0]) / denominator
        hit = valid & (b >= -1e-10) & (c >= -1e-10) & (b + c <= 1 + 1e-10)
        z = triangles[:, 0, 2] + b * (triangles[:, 1, 2] - triangles[:, 0, 2])
        z += c * (triangles[:, 2, 2] - triangles[:, 0, 2])
        centre_hits.extend(z[hit])
    assert centre_hits, "the container needs a collision floor"
    assert bounds.GetMax()[2] - max(centre_hits) > min_depth, "collision must not cap the opening"


def test_grip_is_on_the_handle():
    """The demo grips the knife on its handle, between the handle's end and where the handle ends."""
    y = -task.GRASP_FROM_CENTRE
    assert task.KNIFE_BUTT_LOCAL[1] + 0.01 < y < task.KNIFE_HANDLE_END_Y - 0.01


def test_local_assets_register_once(monkeypatch):
    calls = []

    class FakeRegistry:
        components = {}

        def is_registered(self, key, ensure_loaded=True):
            assert ensure_loaded is False, "loading the whole library before Isaac Sim starts crashes it"
            return key in self.components

        def register(self, component, key=None):
            assert key not in self.components
            self.components[key] = component
            calls.append(key)

    mod = types.ModuleType("isaaclab_arena.assets.registries")
    mod.AssetRegistry = FakeRegistry
    for name in ("isaaclab_arena", "isaaclab_arena.assets"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "isaaclab_arena.assets.registries", mod)
    assert task.register_local_assets() is True
    assert task.register_local_assets() is True
    assert sorted(calls) == sorted(task.LOCAL_ASSETS)
    knife = FakeRegistry.components[task.KNIFE_ASSET]
    assert knife.usd_path == str(task.HERE / task.LOCAL_ASSETS[task.KNIFE_ASSET])


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
    tipped = with_(nominal(), mug=mug(0.32, -0.16, quat=qx(12)))
    assert run([nominal(), tipped, final_state()]) is False
    assert task.score(final_state()) == 0.0


def test_mug_settling_a_little_is_not_spill():
    settled = with_(nominal(), mug=mug(0.32, -0.158))
    assert run([nominal(), settled, final_state()]) is True


def test_mug_speed_is_diagnostic_only():
    """A fast mug that stays upright and in place is recorded but does not change the score (for now)."""
    hit = nominal()
    hit["mug"].lin_vel = (0.5, 0.0, 0.0)
    assert run([nominal(), hit, final_state()]) is True
    assert task._STATE["mug_speed_max"] == pytest.approx(0.5)
    assert task._STATE["zero"] is None


def test_spill_is_measured_from_start_not_from_layout(monkeypatch):
    """If the mug's pos turns out not to be its bounding-box centre, a calibrated
    POS_OFFSET keeps the layout check passing and the shift is still measured from the start."""
    before = nominal()
    monkeypatch.setitem(task.POS_OFFSET, "mug", (0.012, 0.0))
    monkeypatch.setattr(task, "MUG_AXIS_FROM_POS", (-(task.MUG_HANDLE_OFFSET + 0.012), 0.0, 0.0))
    first = with_(before, mug=S(tuple(c + d for c, d in zip(before["mug"].pos, (0.012, 0, 0))), ID,
                                before["mug"].low, before["mug"].high))
    reset()
    task.success(first)
    assert task._STATE["bad_layout"] is False
    assert task._STATE["zero"] is None


def test_mug_shift_spill_is_permanent():
    pushed = with_(nominal(), mug=mug(0.32, -0.14))
    run([nominal(), pushed, final_state()])
    assert task.score(final_state()) == 0.0


def test_slow_restore_does_not_clear_latch():
    frames = [nominal(), with_(nominal(), mug=mug(0.32, -0.16, quat=qx(12)))]
    for deg in (9, 6, 3, 0):
        frames.append(with_(nominal(), mug=mug(0.32, -0.16, quat=qx(deg))))
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


def test_rotated_sphere_box_does_not_falsely_bruise_the_apple():
    # Recorded first state of smoke v2: cached rotated cube bottom is below the
    # table, but the true 33 mm sphere bottom is 9.121 mm ABOVE the table.
    a = S((0.173294, 0.077312, 0.789621), qx(40),
          (0.12, 0.02, 0.736579), (0.23, 0.13, 0.842663))
    w = with_(nominal(), apple=a)
    assert task._apple_bottom(w) == pytest.approx(0.009121)
    assert not task._apple_touches_table(w)
    run([nominal(), w, final_state()])
    assert not task._STATE["apple_bruised"]
    assert task.success(final_state())


def test_rotating_in_bowl_does_not_change_sphere_height_check():
    a = apple_in_bowl()
    a.quat_xyzw = qx(45)
    a.low = (a.low[0], a.low[1], T0 - 0.010)
    assert task._apple_in_bowl(with_(final_state(), apple=a))


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
    tipped_final = with_(final_state(), mug=mug(0.32, -0.16, quat=qx(12)))
    assert task.score(tipped_final) == 0.0


# ---------------------------------------------------------------------------
# Anti-cheat and robustness
# ---------------------------------------------------------------------------
def test_apple_held_above_bowl_does_not_count():
    w = with_(nominal(), apple=sphere(0.24, 0.26, 0.15, 0.033), **towels_aside())
    run([nominal(), w])
    assert task.score(w) == 0.0


@pytest.mark.parametrize("bottom,ok", [(-T0, False), (-0.010, False), (-0.0031, False),
                                       (-0.0029, True), (0.001, True)])
def test_apple_below_table_does_not_count_as_in_bowl(bottom, ok):
    a = sphere(*L["bowl"]["xy"], bottom + L["apple"]["radius"], L["apple"]["radius"])
    w = with_(final_state(), apple=a)
    assert run([nominal(), w]) is ok
    assert task.score(w) == (0.75 if ok else 0.0)


@pytest.mark.parametrize("name", ["towel_sheet", "towel_pad"])
@pytest.mark.parametrize("bottom,ok", [(-T0, False), (-0.010, False), (-0.0031, False),
                                       (-0.0029, True), (0.0, True)])
def test_towel_below_table_does_not_count_as_cleared(name, bottom, ok):
    towel = towels_aside()[name]
    size = L[name]["size"]
    w = with_(final_state(), **{name: box(*towel.pos[:2], bottom + size[2] / 2, *size)})
    assert run([nominal(), w]) is ok
    assert task.score(w) == (0.75 if ok else 0.25)


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
    wrong = with_(nominal(), mug=mug(0.32, -0.25))
    assert run([wrong, final_state()]) is False
    assert task.score(final_state()) == 0.0


@pytest.mark.parametrize("deg,spilt", [(7.9, False), (8.1, True)])
def test_mug_tilt_threshold_boundary(deg, spilt):
    reset()
    w = with_(nominal(), mug=mug(0.32, -0.16, quat=qx(deg)))
    assert task._mug_spilt(w) is spilt


@pytest.mark.parametrize("yaw", [90, 180, -135])
def test_mug_yaw_is_not_tilt(yaw):
    w = with_(final_state(), mug=mug(0.32, -0.16, quat=qz(yaw)))
    assert run([nominal(), w]) is True


# ---------------------------------------------------------------------------
# Episode reset and purity
# ---------------------------------------------------------------------------
def test_teleport_back_to_layout_resets_latches():
    tipped = with_(nominal(), mug=mug(0.32, -0.16, quat=qx(12)))
    far = with_(tipped, apple=sphere(0.60, 0.30, 0.04, 0.033))
    run([nominal(), far])
    assert task._STATE["zero"] == "coffee spilt"
    task.success(nominal())
    assert task._STATE["zero"] is None
    assert task.success(final_state()) is True


def test_score_and_distance_are_read_only():
    run([nominal()])
    snap = repr(sorted(task._STATE.items()))
    tipped = with_(nominal(), mug=mug(0.32, -0.16, quat=qx(20)))
    for _ in range(3):
        task.score(tipped)
        task.distance(tipped)
    assert repr(sorted(task._STATE.items())) == snap


def test_distance_decreases_toward_goal():
    reset()
    assert task.distance(final_state()) < task.distance(nominal())


def test_debug_print_does_not_change_result(capsys, monkeypatch):
    import json

    monkeypatch.setattr(task, "DEBUG", True)
    frames = [nominal()] * 21 + [final_state()]
    assert run(frames) is True
    out = capsys.readouterr().out
    assert "task_progress steps=1 " in out and "task_progress steps=20 " in out
    assert out.count("task_calib ") == 1 and "knife_quat_xyzw=" in out
    snapshots = [json.loads(line.removeprefix("task_state ")) for line in out.splitlines()
                 if line.startswith("task_state ")]
    assert [s["step"] for s in snapshots] == [1, 20, 21]
    assert set(snapshots[0]["objects"]) == set(L)
    assert snapshots[0]["objects"]["mug"]["lin_vel"] == [0.0, 0.0, 0.0]
    assert snapshots[0]["mug_speed_max"] == 0.0
    assert snapshots[0]["checks"]["apple_in_bowl"] is False


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


def test_calibration_stage_holds_both_arms_clear_of_objects(monkeypatch):
    monkeypatch.setenv("TASK_CALIBRATION", "1")
    plan = task.demo()
    assert set(plan) == {"left", "right"}
    for arm, waypoints in plan.items():
        assert sum(w.duration for w in waypoints) >= 5.0
        assert all(w.finger == task.FINGER_OPEN for w in waypoints)
        assert all(w.pos[2] >= T0 + 0.15 for w in waypoints)


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
