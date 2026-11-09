"""Station table after someone cut fruit on it. Near the middle, a large white paper-towel sheet lies on
something lumpy, with a red apple on top. Under the sheet, out of sight, a folded paper-towel pad lies on a
small kitchen knife that lies flat on the table. To the left is an empty bowl. To the right, close to the
robot, is a clean white plate. Further out on the right, between the towels and the plate, stands a white
mug full of hot coffee.
"""

import math
import os

from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder, Sphere

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TABLE_Z = TABLE_TOP[SCENE]  # TABLE_TOP is a dict keyed by scene

# ---------------------------------------------------------------------------
# Values that can only be read off the loaded knife model on GPU. Estimates
# until then; see README "Calibration".
# ---------------------------------------------------------------------------
KNIFE_ASSET = "SET_FROM_ARENA_LIBRARY"  # registry name of a kitchen knife in Arena's object library
KNIFE_SPAWN_YAW = 0.0  # rad; chosen so the knife lies along y with its tip toward +y
KNIFE_HEIGHT = 0.016  # m, table to the top of the knife lying flat
KNIFE_TIP_LOCAL = (0.0, 0.085, 0.0)  # tip end in the knife's body frame, from its bounding-box centre
KNIFE_BUTT_LOCAL = (0.0, -0.085, 0.0)  # handle end
PLATE_HEIGHT = 0.020  # m, table to the plate's top surface

# ---------------------------------------------------------------------------
# Layout (metres, world frame: x forward of the arm bases, y left, z up).
# Arm bases at x 0, y +-0.305. Development estimates; check with reach_map.py.
# ---------------------------------------------------------------------------
LAYOUT = {
    "knife": {"xy": (0.26, 0.00), "length": 0.170},
    "towel_pad": {"xy": (0.26, 0.06), "size": (0.120, 0.110, 0.008)},
    "towel_sheet": {"xy": (0.26, 0.00), "size": (0.260, 0.240, 0.0015)},
    "apple": {"xy": (0.26, 0.06), "radius": 0.033},
    "bowl": {"xy": (0.24, 0.26), "extent": 0.160},
    "plate": {"xy": (0.26, -0.30), "radius": 0.100},
    "mug": {"xy": (0.36, -0.16), "radius": 0.0425, "height": 0.095},
}
APPLE_RED = (0.629, 0.104, 0.138)  # the framework's apple colour
WHITE = (0.93, 0.93, 0.91)
PAPER = (0.97, 0.97, 0.95)

_PAD_Z = KNIFE_HEIGHT + LAYOUT["towel_pad"]["size"][2] / 2 + 0.002
_SHEET_Z = KNIFE_HEIGHT + LAYOUT["towel_pad"]["size"][2] + LAYOUT["towel_sheet"]["size"][2] / 2 + 0.004
_APPLE_Z = _SHEET_Z + LAYOUT["apple"]["radius"] + 0.003

OBJECTS = [
    Asset("knife", KNIFE_ASSET, xy=LAYOUT["knife"]["xy"], extent=LAYOUT["knife"]["length"], mass=0.040,
          yaw=KNIFE_SPAWN_YAW),
    Box("towel_pad", pos=(*LAYOUT["towel_pad"]["xy"], TABLE_Z + _PAD_Z), size=LAYOUT["towel_pad"]["size"],
        mass=0.003, color=PAPER),
    Box("towel_sheet", pos=(*LAYOUT["towel_sheet"]["xy"], TABLE_Z + _SHEET_Z),
        size=LAYOUT["towel_sheet"]["size"], mass=0.004, color=PAPER),
    Sphere("apple", pos=(*LAYOUT["apple"]["xy"], TABLE_Z + _APPLE_Z), radius=LAYOUT["apple"]["radius"],
           mass=0.12, color=APPLE_RED),
    Asset("bowl", "bowl_ycb_robolab", xy=LAYOUT["bowl"]["xy"], extent=LAYOUT["bowl"]["extent"], color=WHITE),
    Asset("plate", "plate_small_vomp_robolab", xy=LAYOUT["plate"]["xy"], extent=2 * LAYOUT["plate"]["radius"],
          color=WHITE),
    Cylinder("mug", pos=(*LAYOUT["mug"]["xy"], TABLE_Z + LAYOUT["mug"]["height"] / 2 + 0.001),
             radius=LAYOUT["mug"]["radius"], height=LAYOUT["mug"]["height"], mass=0.38, color=WHITE),
]

# Thresholds: development estimates, to be calibrated on GPU (see README).
TH = {
    "rest_speed": 0.01,  # m/s
    "rest_spin": 0.10,  # rad/s
    "mug_tilt_deg": 8.0,  # spill proxy
    "mug_shift": 0.015,  # spill proxy
    "keepout_radius": 0.0725,  # mug radius + 30 mm, no height limit
    "lift": 0.015,  # knife's lowest point above the table
    "touch": 0.003,  # lowest point this close to the table counts as touching
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
def _mug_spilt(o):
    m = o["mug"]
    return (_tilt_deg(m.quat_xyzw) > TH["mug_tilt_deg"]
            or _hdist(m.pos, LAYOUT["mug"]["xy"]) > TH["mug_shift"])


def _knife_lifted(o):
    return _z(o["knife"].low) > TH["lift"]


def _knife_mug_hdist(o):
    tip, butt = _knife_ends(o["knife"])
    mug_xy = (float(o["mug"].pos[0]), float(o["mug"].pos[1]))
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
    return _z(a.low) < TH["touch"] and not in_bowl_column


def _apple_in_bowl(o):
    a = o["apple"]
    return (_hdist(a.pos, LAYOUT["bowl"]["xy"]) < TH["apple_axis"]
            and _z(a.pos) < TH["apple_z_max"])


def _towels_cleared(o):
    k = o["knife"]
    for name in ("towel_sheet", "towel_pad"):
        t = o[name]
        if _z(t.high) > TH["towel_high_max"]:
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
    return all(_rest(o[n]) for n in ("apple", "knife", "mug", "towel_sheet", "towel_pad"))


def _nominal(o):
    for name, spec in LAYOUT.items():
        if name not in o:
            return False
        if _hdist(o[name].pos, spec["xy"]) > TH["nominal_tol"]:
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
            n in o and _hdist(o[n].pos, s["xy"]) <= TH["layout_tol"] for n, s in LAYOUT.items()
        )
    _STATE["steps"] += 1
    _STATE["prev"] = _positions(o)

    m = o["mug"]
    _STATE["mug_tilt_max"] = max(_STATE["mug_tilt_max"], _tilt_deg(m.quat_xyzw))
    _STATE["mug_shift_max"] = max(_STATE["mug_shift_max"], _hdist(m.pos, LAYOUT["mug"]["xy"]))
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
def success(objects):
    _observe(objects)
    ok = _tier(objects) == 1.0
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
# lifts the knife by its handle, turns it to point away, carries it past the
# mug on the robot's side and lays it on the plate.
# ---------------------------------------------------------------------------
GRASP_FROM_CENTRE = 0.04  # m from the knife's centre toward the handle end
TRAVEL_Z = 0.10  # carry height above the table
KNIFE_ROUTE_X = 0.13  # x of the knife grip while passing the mug


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
    place = (lx - GRASP_FROM_CENTRE, ly)  # knife centre over the plate once it points to +x
    up = t + TRAVEL_Z
    half = math.pi / 2
    return [
        Waypoint(2.0, (0.20, -0.30, t + 0.15), FINGER_OPEN, "out of the way", 0.0),
        Waypoint(wait_s, (0.20, -0.30, t + 0.15), FINGER_OPEN, "wait for the towels", 0.0),
        Waypoint(1.5, (gx, gy, up), FINGER_OPEN, "over the handle", half),
        Waypoint(1.5, (gx, gy, t + 0.015), FINGER_OPEN, "descend", half),
        Waypoint(0.8, (gx, gy, t + 0.015), FINGER_CLOSED, "close on the handle", half),
        Waypoint(1.2, (gx, gy, up), FINGER_CLOSED, "lift", half),
        Waypoint(1.5, (gx, gy, up), FINGER_CLOSED, "turn tip away", 0.0),
        Waypoint(1.2, (KNIFE_ROUTE_X, gy, up), FINGER_CLOSED, "pull in", 0.0),
        Waypoint(1.8, (KNIFE_ROUTE_X, ly, up), FINGER_CLOSED, "pass the mug", 0.0),
        Waypoint(1.2, (place[0], place[1], up), FINGER_CLOSED, "over the plate", 0.0),
        Waypoint(1.2, (place[0], place[1], t + PLATE_HEIGHT + 0.025), FINGER_CLOSED, "lower", 0.0),
        Waypoint(0.6, (place[0], place[1], t + PLATE_HEIGHT + 0.025), FINGER_OPEN, "release", 0.0),
        Waypoint(1.0, (place[0], place[1], up), FINGER_OPEN, "retreat", 0.0),
    ]


def demo():
    left = _left_plan()
    wait = sum(w.duration for w in left) - 2.0 + 1.0
    return {"left": left, "right": _right_plan(wait)}
