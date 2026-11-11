"""Station table after someone cut fruit on it. Near the middle, a large white paper-towel sheet lies on something
lumpy, with a red apple on top. Under the sheet, out of sight, a folded paper-towel pad lies on a small knife with a
red handle, lying flat on the table with its blade toward the robot's left. To the left is an empty bowl. To the
right, close to the robot, is a clean plate. Further out on the right, between the towels and the plate, stands a
white mug full of hot coffee.
"""

import json
import math
import os
import pathlib

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Sphere

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TABLE_Z = TABLE_TOP[SCENE]  # TABLE_TOP is a dict keyed by scene

# ---------------------------------------------------------------------------
# Models shipped in assets/ and registered in Arena's asset registry under these
# names, so the scene uses them through the task file's ordinary Asset objects.
# yam_arena only reads usd_path from a registered class. Arena's object library
# itself is imported later, after Isaac Sim starts; its registry module is plain
# Python, so this runs at task-file load. is_registered(..., ensure_loaded=False)
# avoids importing the whole library before the simulator exists.
# ---------------------------------------------------------------------------
HERE = pathlib.Path(__file__).resolve().parent
LOCAL_ASSETS = {
    "knife_ycb_local": "assets/knife/knife.usda",
    "bowl_ycb_local": "assets/bowl/bowl.usda",
    "plate_ycb_local": "assets/plate/plate.usda",
    "coffee_fitted_local": "assets/coffee/coffee.usda",
}


def register_local_assets() -> bool:
    """Register LOCAL_ASSETS with Arena's registry; False where Arena is not installed (CPU tests)."""
    try:
        from isaaclab_arena.assets.registries import AssetRegistry
    except ModuleNotFoundError:
        return False
    registry = AssetRegistry()
    for key, rel in LOCAL_ASSETS.items():
        if not registry.is_registered(key, ensure_loaded=False):
            registry.register(type(key, (), {"registry_name": key, "usd_path": str(HERE / rel)}), key)
    return True


register_local_assets()

# Knife: assets/knife/knife.usda (YCB 032_knife scan) lies on its underside with its tip toward +y, scaled to
# 170 mm, bounding-box centre at the origin. These values are read off that file (see assets/knife/README.md).
KNIFE_ASSET = "knife_ycb_local"
KNIFE_SPAWN_YAW = 0.0  # rad: the file already points the tip to +y
KNIFE_HEIGHT = 0.0119  # m, table to the top of the knife lying on the table
KNIFE_TIP_LOCAL = (0.0, 0.085, 0.0)  # blade tip, from the bounding-box centre, body frame
KNIFE_BUTT_LOCAL = (0.0, -0.085, 0.0)  # end of the handle
KNIFE_HANDLE_END_Y = -0.017  # body-frame y where the red handle ends (the metal neck and blade follow)
PLATE_HEIGHT = 0.020  # m, table to the plate's top surface: CONFIRM against the plate model

# Official mug geometry measured offline at 120 mm total width, including its
# handle. The loose coffee frustum has 2 mm radial wall clearance; its surface
# starts 8 mm below the rim. GPU collision cooking still needs verification.
COFFEE_INFO = json.loads((HERE / "assets/coffee/coffee_info.json").read_text())
COFFEE_ASSET = "coffee_fitted_local"
MUG_ASSET = "mug"  # Isaac Lab's mug (Objects/Mug/mug.usd), reachable from the GPU machine
MUG_YAW = 0.0  # handle points to +x, away from the robot
MUG_EXTENT = COFFEE_INFO["mug_extent_m"]
MUG_BODY_RADIUS = COFFEE_INFO["mug_body_radius_m"]
MUG_RIM_HEIGHT = COFFEE_INFO["mug_rim_height_m"]
MUG_INNER_FLOOR = COFFEE_INFO["mug_inner_floor_m"]
MUG_AXIS_FROM_POS = tuple(COFFEE_INFO["mug_axis_from_bbox_center_m"])
MUG_HANDLE_OFFSET = -MUG_AXIS_FROM_POS[0]
# Offset of each object's ObjectState.pos from the xy it was spawned at, (dx, dy); zero until measured.
POS_OFFSET = {}

# ---------------------------------------------------------------------------
# Layout (metres, world frame: x forward of the arm bases, y left, z up).
# Arm bases at x 0, y +-0.305. Development estimates; check with reach_map.py.
# ---------------------------------------------------------------------------
LAYOUT = {
    "knife": {"xy": (0.26, 0.00), "length": 0.170},
    "towel_pad": {"xy": (0.26, 0.00), "size": (0.120, 0.110, 0.008)},
    "towel_sheet": {"xy": (0.26, 0.00), "size": (0.260, 0.240, 0.0015)},
    "apple": {"xy": (0.26, 0.00), "radius": 0.033},
    "bowl": {"xy": (0.24, 0.26), "extent": 0.160},
    "plate": {"xy": (0.26, -0.30), "radius": 0.100},
    "mug": {"axis_xy": (0.32, -0.16)},
    "coffee": {"height_below_rim": 0.008},
}
_MAX = LAYOUT["mug"]["axis_xy"]
_MX, _MY = MUG_AXIS_FROM_POS[:2]
LAYOUT["mug"]["xy"] = (_MAX[0] - _MX * math.cos(MUG_YAW) + _MY * math.sin(MUG_YAW),
                       _MAX[1] - _MX * math.sin(MUG_YAW) - _MY * math.cos(MUG_YAW))
LAYOUT["coffee"]["xy"] = _MAX
LAYOUT["coffee"]["radius"] = max(COFFEE_INFO["coffee_radii_m"])
LAYOUT["coffee"]["height"] = COFFEE_INFO["coffee_height_m"]
APPLE_RED = (0.629, 0.104, 0.138)  # the framework's apple colour
WHITE = (0.93, 0.93, 0.91)
PAPER = (0.97, 0.97, 0.95)
COFFEE = (0.20, 0.11, 0.05)

_PAD_Z = KNIFE_HEIGHT + LAYOUT["towel_pad"]["size"][2] / 2 + 0.002
_SHEET_Z = KNIFE_HEIGHT + LAYOUT["towel_pad"]["size"][2] + LAYOUT["towel_sheet"]["size"][2] / 2 + 0.004
_APPLE_Z = _SHEET_Z + LAYOUT["apple"]["radius"] + 0.003

OBJECTS = [
    Asset("knife", KNIFE_ASSET, xy=LAYOUT["knife"]["xy"], extent=LAYOUT["knife"]["length"], mass=0.050,
          yaw=KNIFE_SPAWN_YAW),
    Box("towel_pad", pos=(*LAYOUT["towel_pad"]["xy"], TABLE_Z + _PAD_Z), size=LAYOUT["towel_pad"]["size"],
        mass=0.003, color=PAPER),
    Box("towel_sheet", pos=(*LAYOUT["towel_sheet"]["xy"], TABLE_Z + _SHEET_Z),
        size=LAYOUT["towel_sheet"]["size"], mass=0.004, color=PAPER),
    Sphere("apple", pos=(*LAYOUT["apple"]["xy"], TABLE_Z + _APPLE_Z), radius=LAYOUT["apple"]["radius"],
           mass=0.12, color=APPLE_RED),
    Asset("bowl", "bowl_ycb_local", xy=LAYOUT["bowl"]["xy"], extent=LAYOUT["bowl"]["extent"]),
    Asset("plate", "plate_ycb_local", xy=LAYOUT["plate"]["xy"], extent=2 * LAYOUT["plate"]["radius"]),
    Asset("mug", MUG_ASSET, xy=LAYOUT["mug"]["xy"], extent=MUG_EXTENT, mass=0.30, color=WHITE, yaw=MUG_YAW),
    Asset("coffee", COFFEE_ASSET, xy=LAYOUT["coffee"]["xy"], extent=COFFEE_INFO["coffee_extent_m"],
          mass=0.25),
]


def _install_coffee_placement():
    """Elevate only this coffee asset; Arena's Asset API has no starting-height field.

    The adapter lives in this task, changes no installed file, and is idempotent
    across task reloads. All other assets use the unmodified placement function.
    """
    try:
        from yam_arena import station
    except ImportError:
        return False
    original = getattr(station.placement, "_unbury_original", station.placement)

    def placement(item, lo, hi):
        scale, pos, rot = original(item, lo, hi)
        if item.asset == COFFEE_ASSET:
            pos = (pos[0], pos[1], pos[2] + COFFEE_INFO["coffee_bottom_above_mug_bottom_m"])
        return scale, pos, rot

    placement._unbury_original = original
    station.placement = placement
    return True


_install_coffee_placement()

# Thresholds: development estimates, to be calibrated on GPU (see README).
TH = {
    "rest_speed": 0.01,  # m/s
    "rest_spin": 0.10,  # rad/s
    "mug_tilt_deg": 8.0,  # spill proxy
    "mug_shift": 0.015,  # cup axis, measured from where it stood at the start
    "coffee_off_axis": 0.012,  # coffee centre this far from the cup axis has slid out
    "coffee_rise": 0.020,  # coffee centre this far above its start has been lifted out
    "keepout_radius": MUG_BODY_RADIUS + 0.030,  # around the cup axis, no height limit
    "lift": 0.015,  # knife's lowest point above the table
    "touch": 0.003,  # table contact tolerance; also the maximum permitted penetration below the table
    "drop_speed": 0.8,  # m/s downward
    "apple_axis": 0.040,
    "apple_z_max": 0.060,  # apple centre height when sitting in the bowl
    "towel_high_max": 0.030,  # a cleared towel lies low on the table
    "towel_moved": 0.10,  # a cleared towel has moved this far from where it started
    "knife_flat_deg": 10.0,
    "knife_dir_deg": 25.0,  # tip within this of +x (pointing away from the robot)
    "knife_on_plate_z": 0.020,  # knife's lowest point at most this far above the plate top
    "reset_jump": 0.08,  # m per call, beyond any real motion
    "nominal_tol": 0.010,
    "layout_tol": 0.020,
}

DEBUG = os.environ.get("TASK_DEBUG") == "1"


# ---------------------------------------------------------------------------
# Geometry helpers (pure functions)
# ---------------------------------------------------------------------------
def _rotate(q, v):
    x, y, z, w = (float(c) for c in q)
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _tilt_deg(q, axis=(0.0, 0.0, 1.0)):
    """Angle between the body's local axis and world up, in degrees."""
    a = _rotate(q, axis)
    return math.degrees(math.acos(max(-1.0, min(1.0, a[2]))))


def _z(p):
    return float(p[2]) - TABLE_Z


def _hdist(p, xy):
    return math.hypot(float(p[0]) - xy[0], float(p[1]) - xy[1])


def _point(o, local):
    d = _rotate(o.quat_xyzw, local)
    return (float(o.pos[0]) + d[0], float(o.pos[1]) + d[1], float(o.pos[2]) + d[2])


def _knife_ends(k):
    return _point(k, KNIFE_TIP_LOCAL), _point(k, KNIFE_BUTT_LOCAL)


def _knife_axis(k):
    tip, butt = _knife_ends(k)
    return (tip[0] - butt[0], tip[1] - butt[1], tip[2] - butt[2])


def _rect_hits_disc(low, high, xy, r):
    cx = min(max(xy[0], float(low[0])), float(high[0]))
    cy = min(max(xy[1], float(low[1])), float(high[1]))
    return math.hypot(cx - xy[0], cy - xy[1]) < r


def _rects_overlap(a_low, a_high, b_low, b_high):
    return (a_low[0] < b_high[0] and b_low[0] < a_high[0]
            and a_low[1] < b_high[1] and b_low[1] < a_high[1])


def _rest(o):
    return o.resting(max_speed=TH["rest_speed"], max_spin=TH["rest_spin"])


# ---------------------------------------------------------------------------
# Instantaneous checks
# ---------------------------------------------------------------------------
def _mug_axis(o):
    p = _point(o["mug"], MUG_AXIS_FROM_POS)
    return (p[0], p[1])


def _start(name):
    """Where an object stood at the start of the episode (read-only)."""
    return _STATE.get("start", {}).get(name)


def _mug_shift(o):
    ref = _start("mug_axis") or LAYOUT["mug"]["axis_xy"]
    return _hdist(_mug_axis(o), ref)


def _coffee_out(o):
    c = o["coffee"]
    z0 = _start("coffee_z")
    rose = z0 is not None and float(c.pos[2]) - z0 > TH["coffee_rise"]
    return _hdist(c.pos, _mug_axis(o)) > TH["coffee_off_axis"] or rose


def _mug_spilt(o):
    return (_tilt_deg(o["mug"].quat_xyzw) > TH["mug_tilt_deg"]
            or _mug_shift(o) > TH["mug_shift"]
            or _coffee_out(o))


def _knife_lifted(o):
    return _z(o["knife"].low) > TH["lift"]


def _knife_mug_hdist(o):
    tip, butt = _knife_ends(o["knife"])
    mug_xy = _mug_axis(o)
    best = math.inf
    for i in range(9):
        t = i / 8
        p = (tip[0] + t * (butt[0] - tip[0]), tip[1] + t * (butt[1] - tip[1]))
        best = min(best, _hdist(p, mug_xy))
    return best


def _knife_in_keepout(o):
    return _knife_lifted(o) and _knife_mug_hdist(o) < TH["keepout_radius"]


def _knife_dropped(o):
    return float(o["knife"].lin_vel[2]) < -TH["drop_speed"]


def _knife_off_table(o):
    return _z(o["knife"].pos) < -0.02


def _apple_touches_table(o):
    a = o["apple"]
    in_bowl_column = _hdist(a.pos, LAYOUT["bowl"]["xy"]) < LAYOUT["bowl"]["extent"] / 2
    return _apple_bottom(o) < TH["touch"] and not in_bowl_column


def _apple_bottom(o):
    # Arena rotates the cached bounding BOX, even for a sphere. Its low.z can
    # grow downward as the apple spins. The analytic sphere bottom is invariant.
    return _z(o["apple"].pos) - LAYOUT["apple"]["radius"]


def _apple_in_bowl(o):
    a = o["apple"]
    return (_hdist(a.pos, LAYOUT["bowl"]["xy"]) < TH["apple_axis"]
            and _apple_bottom(o) >= -TH["touch"]
            and _z(a.pos) < TH["apple_z_max"])


def _towels_cleared(o):
    k = o["knife"]
    for name in ("towel_sheet", "towel_pad"):
        t = o[name]
        if _z(t.low) < -TH["touch"] or _z(t.high) > TH["towel_high_max"]:
            return False
        if _hdist(t.pos, LAYOUT[name]["xy"]) < TH["towel_moved"]:
            return False
        if _rects_overlap(t.low, t.high, k.low, k.high):
            return False
        if _rect_hits_disc(t.low, t.high, LAYOUT["plate"]["xy"], LAYOUT["plate"]["radius"]):
            return False
        for c in ("bowl", "mug"):
            if _hdist(t.pos, LAYOUT[c]["xy"]) < 0.06:
                return False
    return True


def _knife_on_plate(o):
    k = o["knife"]
    return (_hdist(k.pos, LAYOUT["plate"]["xy"]) < LAYOUT["plate"]["radius"]
            and TH["touch"] < _z(k.low) < PLATE_HEIGHT + TH["knife_on_plate_z"])


def _knife_set(o):
    if not _knife_on_plate(o):
        return False
    tip, butt = _knife_ends(o["knife"])
    r = LAYOUT["plate"]["radius"] + 0.005
    if _hdist(tip, LAYOUT["plate"]["xy"]) > r or _hdist(butt, LAYOUT["plate"]["xy"]) > r:
        return False
    ax, ay, az = _knife_axis(o["knife"])
    flat = abs(math.degrees(math.atan2(az, math.hypot(ax, ay)))) <= TH["knife_flat_deg"]
    pointing_away = abs(math.degrees(math.atan2(ay, ax))) <= TH["knife_dir_deg"]
    return flat and pointing_away


def _all_rest(o):
    return all(_rest(o[n]) for n in ("apple", "knife", "mug", "coffee", "towel_sheet", "towel_pad"))


def _expected_xy(name):
    x, y = LAYOUT[name]["xy"]
    dx, dy = POS_OFFSET.get(name, (0.0, 0.0))
    return (x + dx, y + dy)


def _nominal(o):
    for name, spec in LAYOUT.items():
        if name not in o:
            return False
        if _hdist(o[name].pos, _expected_xy(name)) > TH["nominal_tol"]:
            return False
    return True


# ---------------------------------------------------------------------------
# History: latches only irreversible, instantaneous events. Updated in
# success() only (the framework calls it after every control step);
# score() and distance() never write.
# ---------------------------------------------------------------------------
_STATE = {}


def _fresh_state():
    return {
        "steps": 0,
        "prev": None,
        "bad_layout": False,
        "zero": None,  # reason for a permanent zero
        "cap_half": None,  # reason for the 0.5 cap
        "apple_bruised": False,
        "mug_tilt_max": 0.0,
        "mug_shift_max": 0.0,
        "knife_mug_min": math.inf,  # closest approach of the lifted knife, for calibration
        "start": {},
    }


_STATE.update(_fresh_state())


def _positions(o):
    return {n: tuple(float(c) for c in o[n].pos) for n in LAYOUT if n in o}


def _episode_started(o):
    prev = _STATE["prev"]
    if prev is None:
        return True
    jumped = any(
        n in prev and math.dist(prev[n], p) > TH["reset_jump"] for n, p in _positions(o).items()
    )
    return jumped and _nominal(o)


def _observe(o):
    if _episode_started(o):
        _STATE.clear()
        _STATE.update(_fresh_state())
        _STATE["bad_layout"] = not all(
            n in o and _hdist(o[n].pos, _expected_xy(n)) <= TH["layout_tol"] for n in LAYOUT
        )
        _STATE["start"] = {"mug_axis": _mug_axis(o), "coffee_z": float(o["coffee"].pos[2])}
    _STATE["steps"] += 1
    _STATE["prev"] = _positions(o)

    m = o["mug"]
    _STATE["mug_tilt_max"] = max(_STATE["mug_tilt_max"], _tilt_deg(m.quat_xyzw))
    _STATE["mug_shift_max"] = max(_STATE["mug_shift_max"], _mug_shift(o))
    if _knife_lifted(o):
        _STATE["knife_mug_min"] = min(_STATE["knife_mug_min"], _knife_mug_hdist(o))

    if _STATE["zero"] is None:
        if _mug_spilt(o):
            _STATE["zero"] = "coffee spilt"
        elif _knife_off_table(o):
            _STATE["zero"] = "knife off the table"
    if _STATE["cap_half"] is None:
        if _knife_in_keepout(o):
            _STATE["cap_half"] = "knife near the mug"
        elif _knife_dropped(o):
            _STATE["cap_half"] = "knife dropped"
    if _apple_touches_table(o):
        _STATE["apple_bruised"] = True


def _zero_reason(o):
    if _STATE.get("bad_layout"):
        return "bad initial layout"
    if _STATE.get("zero"):
        return _STATE["zero"]
    if _mug_spilt(o):  # final-frame recheck, no write
        return "coffee spilt"
    if _knife_off_table(o):
        return "knife off the table"
    return None


def _cap_reason(o):
    if _STATE.get("cap_half"):
        return _STATE["cap_half"]
    if _knife_in_keepout(o):
        return "knife near the mug"
    return None


def _tier(o):
    if _zero_reason(o):
        return 0.0
    apple = _apple_in_bowl(o) and _rest(o["apple"]) and not _STATE.get("apple_bruised")
    if not apple:
        return 0.0
    if not _towels_cleared(o):
        return 0.25
    if not _knife_on_plate(o):
        return 0.5
    t = 1.0 if (_knife_set(o) and _all_rest(o)) else 0.75
    if _cap_reason(o):
        t = min(t, 0.5)
    return t


# ---------------------------------------------------------------------------
# Task-file contract
# ---------------------------------------------------------------------------
def _calib_fields(objects, name):
    o = objects[name]

    def r(v):
        return "(" + ",".join("%.4f" % float(c) for c in v) + ")"

    return "%s_pos=%s %s_quat_xyzw=%s %s_low=%s %s_high=%s" % (
        name, r(o.pos), name, r(o.quat_xyzw), name, r(o.low), name, r(o.high))


def _debug_snapshot(objects, ready):
    """Diagnostics only: preserve measurements needed to adjust a run offline."""
    def values(v):
        return [round(float(c), 6) for c in v]

    return {
        "step": _STATE["steps"],
        "table_z": TABLE_Z,
        "ready": ready,
        "score": score(objects),
        "zero_reason": _zero_reason(objects),
        "cap_reason": _cap_reason(objects),
        "bad_layout": _STATE["bad_layout"],
        "apple_bruised": _STATE["apple_bruised"],
        "apple_bottom_above_table": round(_apple_bottom(objects), 6),
        "layout_xy_errors": {name: round(_hdist(objects[name].pos, _expected_xy(name)), 6) for name in LAYOUT},
        "checks": {
            "apple_in_bowl": _apple_in_bowl(objects),
            "towels_cleared": _towels_cleared(objects),
            "knife_on_plate": _knife_on_plate(objects),
            "knife_set": _knife_set(objects),
            "all_rest": _all_rest(objects),
        },
        "mug_axis": values(_mug_axis(objects)),
        "mug_tilt_deg": round(_tilt_deg(objects["mug"].quat_xyzw), 4),
        "coffee_axis_gap": round(_hdist(objects["coffee"].pos, _mug_axis(objects)), 6),
        "objects": {
            name: {field: values(getattr(objects[name], field))
                   for field in ("pos", "quat_xyzw", "low", "high", "lin_vel", "ang_vel")}
            for name in LAYOUT
        },
    }


def success(objects):
    _observe(objects)
    ok = _tier(objects) == 1.0
    if DEBUG and _STATE["steps"] == 1:
        p = objects["plate"]
        print(
            "task_calib table_z=%.4f %s plate_high_z=%.4f"
            % (TABLE_Z, " ".join(_calib_fields(objects, n) for n in LAYOUT),
               float(p.high[2]))
        )
    if DEBUG and _STATE["steps"] % 20 in (0, 1):
        print(
            "task_progress steps=%d fail=%s cap=%s score=%.2f mug_tilt_max=%.2fdeg mug_shift_max=%.1fmm "
            "knife_mug_min=%.1fmm"
            % (
                _STATE["steps"],
                _zero_reason(objects),
                _cap_reason(objects),
                score(objects),
                _STATE["mug_tilt_max"],
                _STATE["mug_shift_max"] * 1000,
                _STATE["knife_mug_min"] * 1000 if _STATE["knife_mug_min"] < math.inf else -1.0,
            )
        )
        print("task_state " + json.dumps(_debug_snapshot(objects, ok), separators=(",", ":")), flush=True)
    return ok


def score(objects):
    """Partial credit, read-only. 1.0 is reached only through success()."""
    return min(_tier(objects), 0.75)


def distance(objects):
    """Read-only progress measure in metres (lower is better)."""
    d = _hdist(objects["apple"].pos, LAYOUT["bowl"]["xy"])
    if not _towels_cleared(objects):
        d += 0.25
    d += _hdist(objects["knife"].pos, LAYOUT["plate"]["xy"])
    return float(d)


# ---------------------------------------------------------------------------
# Scripted demo: left arm clears the apple and both towels, then the right arm
# lifts the knife by the middle of its handle, carries it past the mug on the
# robot's side, turns the blade to point away and lays it on the plate.
# ---------------------------------------------------------------------------
GRASP_FROM_CENTRE = 0.050  # m from the knife's centre toward the handle end: mid-handle
TRAVEL_Z = 0.10  # carry height above the table
KNIFE_ROUTE_X = 0.13  # x of the grip while the knife passes the mug
TURN_DY = -0.04  # y of the turn relative to the plate centre: far enough from the mug for the tip's sweep


def _left_plan():
    t = TABLE_Z
    ax, ay = LAYOUT["apple"]["xy"]
    bx, by = LAYOUT["bowl"]["xy"]
    sx, sy = LAYOUT["towel_sheet"]["xy"]
    near_edge = sx - LAYOUT["towel_sheet"]["size"][0] / 2
    px, py = LAYOUT["towel_pad"]["xy"]
    pad_edge = py + LAYOUT["towel_pad"]["size"][1] / 2
    plan = []
    plan += pick_place_waypoints((ax, ay, t + _APPLE_Z), (bx, by, t + 0.075), travel_z=t + 0.14)
    # Fingers along x. The left arm mirrors the right, so it turns the grip to -pi/2: turning to +pi/2
    # from the bowl leaves its wrist 30 degrees short (IK check). A parallel grip is the same either way.
    plan += pick_place_waypoints((near_edge, 0.05, t + _SHEET_Z), (0.35, 0.18, t + 0.02),
                                 travel_z=t + TRAVEL_Z, yaw=-math.pi / 2)
    plan += pick_place_waypoints((px, pad_edge, t + _PAD_Z), (0.10, 0.175, t + 0.02),
                                 travel_z=t + TRAVEL_Z, yaw=0.0)
    return plan


def _right_plan(wait_s):
    t = TABLE_Z
    kx, ky = LAYOUT["knife"]["xy"]
    gx, gy = kx, ky - GRASP_FROM_CENTRE  # on the handle, which points to -y
    lx, ly = LAYOUT["plate"]["xy"]
    place = (lx - GRASP_FROM_CENTRE, ly)  # knife centre over the plate once the tip points to +x
    up = t + TRAVEL_Z
    half = math.pi / 2
    return [
        Waypoint(2.0, (0.20, -0.30, t + 0.15), FINGER_OPEN, "out of the way", 0.0),
        Waypoint(wait_s, (0.20, -0.30, t + 0.15), FINGER_OPEN, "wait for the towels", 0.0),
        Waypoint(1.5, (gx, gy, up), FINGER_OPEN, "over the handle", half),
        Waypoint(1.5, (gx, gy, t + 0.015), FINGER_OPEN, "descend", half),
        Waypoint(0.8, (gx, gy, t + 0.015), FINGER_CLOSED, "close on the handle", half),
        Waypoint(1.2, (gx, gy, up), FINGER_CLOSED, "lift", half),
        Waypoint(1.2, (KNIFE_ROUTE_X, gy, up), FINGER_CLOSED, "pull in", half),
        # Pass the mug with the knife still along y (tip trailing), and turn it only once the
        # tip's sweep stays clear of the mug: turning first swings the tip 135 mm toward it.
        Waypoint(1.8, (KNIFE_ROUTE_X, ly + TURN_DY, up), FINGER_CLOSED, "pass the mug", half),
        Waypoint(1.5, (KNIFE_ROUTE_X, ly + TURN_DY, up), FINGER_CLOSED, "turn the blade away", 0.0),
        Waypoint(1.2, (place[0], place[1], up), FINGER_CLOSED, "over the plate", 0.0),
        Waypoint(1.2, (place[0], place[1], t + PLATE_HEIGHT + 0.025), FINGER_CLOSED, "lower", 0.0),
        Waypoint(0.6, (place[0], place[1], t + PLATE_HEIGHT + 0.025), FINGER_OPEN, "release", 0.0),
        Waypoint(1.0, (place[0], place[1], up), FINGER_OPEN, "retreat", 0.0),
    ]


def demo():
    if os.environ.get("TASK_CALIBRATION") == "1":
        # A separate 5-second stage for checking reset stability before grasping.
        return {side: [Waypoint(6.0, (0.20, y, TABLE_Z + 0.15), FINGER_OPEN, "calibration hold", 0.0)]
                for side, y in (("left", 0.30), ("right", -0.30))}
    left = _left_plan()
    wait = sum(w.duration for w in left) - 2.0 + 1.0
    return {"left": left, "right": _right_plan(wait)}
