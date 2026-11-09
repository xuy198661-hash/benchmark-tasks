"""A breakfast table after someone cut fruit. In the middle sits a white plate. A large white paper
towel is spread over the whole plate, and a red apple rests on top of it. Under the towel, out of
sight, a folded paper-towel pad lies on a small fruit knife that lies flat across the plate. To the
left of the plate is an empty white bowl. To the right of the plate is a white mug full of hot coffee,
and further right, slightly nearer the robot, is a short empty drinking glass.
"""

import math
import os

from yam_arena.task_file import TABLE_TOP

SCENE = "station"
# TABLE_TOP is a dict keyed by scene, e.g. {"station": 0.7475, "tabletop": 0.75}.
TABLE_Z = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

# ---------------------------------------------------------------------------
# Layout (metres). x forward of the line through the arm bases, y left (+) of
# the midpoint between them, z above the table top. Every number here is a
# development estimate: re-check against reach_map.py and the loaded assets.
# ---------------------------------------------------------------------------
LAYOUT = {
    "plate": {"xy": (0.26, 0.00), "radius": 0.110, "height": 0.010},
    "knife": {"xy": (0.26, 0.00), "length": 0.170, "blade": 0.080, "handle": 0.090},
    "towel_pad": {"xy": (0.26, 0.04), "size": (0.120, 0.110, 0.008)},
    "towel_sheet": {"xy": (0.26, 0.00), "size": (0.260, 0.240, 0.0015)},
    "apple": {"xy": (0.26, 0.02), "radius": 0.040},
    "bowl": {"xy": (0.24, 0.26), "inner_radius": 0.070, "height": 0.065, "floor": 0.005},
    "mug": {"xy": (0.24, -0.17), "radius": 0.0425, "height": 0.095},
    "glass": {"xy": (0.22, -0.30), "inner_radius": 0.032, "height": 0.085, "base": 0.012},
}

# Knife frame: local +y points from the handle end to the tip, origin at the
# knife's geometric centre. VERIFY against the knife asset's bounding box.
KNIFE_TIP_LOCAL = (0.0, LAYOUT["knife"]["length"] / 2, 0.0)
KNIFE_BUTT_LOCAL = (0.0, -LAYOUT["knife"]["length"] / 2, 0.0)

# Thresholds: development estimates, to be calibrated on GPU (see README).
TH = {
    "rest_speed": 0.01,  # m/s
    "rest_spin": 0.10,  # rad/s
    "mug_tilt_deg": 8.0,  # spill proxy
    "mug_shift": 0.015,  # spill proxy
    "glass_fall_deg": 30.0,  # knocked over
    "glass_ok_deg": 5.0,
    "glass_ok_shift": 0.030,
    "keepout_radius": 0.0725,  # mug radius + 30 mm, no height limit
    "lift": 0.015,  # above the plate top
    "touch": 0.003,  # lowest point this close to the table counts as touching
    "drop_speed": 0.8,  # m/s downward
    "apple_axis": 0.035,
    "apple_z_max": 0.060,  # apple centre height when sitting in the bowl
    "tip_below_rim": 0.025,
    "butt_above_rim": 0.040,
    "knife_tilt_min_deg": 12.0,  # below this it cannot stand on its own
    "knife_tilt_max_deg": 45.0,
    "towel_high_max": 0.030,  # a cleared towel lies low on the table
    "reset_jump": 0.08,  # m per call, beyond any real motion
    "nominal_tol": 0.010,
    "layout_tol": 0.020,
}

DEBUG = os.environ.get("TASK_DEBUG") == "1"


# ---------------------------------------------------------------------------
# Geometry helpers (pure functions, no framework access)
# ---------------------------------------------------------------------------
def _rotate(q, v):
    x, y, z, w = q
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
    return p[2] - TABLE_Z


def _hdist(p, xy):
    return math.hypot(p[0] - xy[0], p[1] - xy[1])


def _point(o, local):
    d = _rotate(o.quat_xyzw, local)
    return (o.pos[0] + d[0], o.pos[1] + d[1], o.pos[2] + d[2])


def _knife_ends(k):
    return _point(k, KNIFE_TIP_LOCAL), _point(k, KNIFE_BUTT_LOCAL)


def _knife_axis_tilt_deg(k):
    tip, butt = _knife_ends(k)
    dx, dy, dz = butt[0] - tip[0], butt[1] - tip[1], butt[2] - tip[2]
    n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-9
    return math.degrees(math.acos(max(-1.0, min(1.0, dz / n))))


def _rect_hits_disc(low, high, xy, r):
    cx = min(max(xy[0], low[0]), high[0])
    cy = min(max(xy[1], low[1]), high[1])
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


def _glass_fallen(o):
    return _tilt_deg(o["glass"].quat_xyzw) > TH["glass_fall_deg"]


def _knife_lifted(o):
    return _z(o["knife"].low) > LAYOUT["plate"]["height"] + TH["lift"]


def _knife_in_keepout(o):
    if not _knife_lifted(o):
        return False
    tip, butt = _knife_ends(o["knife"])
    mug_xy = (o["mug"].pos[0], o["mug"].pos[1])
    for i in range(5):
        t = i / 4
        p = (tip[0] + t * (butt[0] - tip[0]), tip[1] + t * (butt[1] - tip[1]))
        if _hdist(p, mug_xy) < TH["keepout_radius"]:
            return True
    return False


def _knife_dropped(o):
    k = o["knife"]
    return k.lin_vel[2] < -TH["drop_speed"]


def _knife_off_table(o):
    return _z(o["knife"].pos) < -0.02


def _apple_touches_table(o):
    return _z(o["apple"].low) < TH["touch"]


def _apple_in_bowl(o):
    a = o["apple"]
    return (_hdist(a.pos, LAYOUT["bowl"]["xy"]) < TH["apple_axis"]
            and _z(a.pos) < TH["apple_z_max"])


def _towels_cleared(o):
    plate = o["plate"]
    plate_xy = (plate.pos[0], plate.pos[1])
    k = o["knife"]
    for name in ("towel_sheet", "towel_pad"):
        t = o[name]
        if _z(t.high) > TH["towel_high_max"]:
            return False
        if _rect_hits_disc(t.low, t.high, plate_xy, LAYOUT["plate"]["radius"]):
            return False
        if _rects_overlap(t.low, t.high, k.low, k.high):
            return False
        for c in ("bowl", "mug", "glass"):
            if _hdist(t.pos, LAYOUT[c]["xy"]) < 0.06:
                return False
    return True


def _knife_in_glass(o):
    tip, _ = _knife_ends(o["knife"])
    g = o["glass"]
    rim = LAYOUT["glass"]["height"]  # glass standing on the table
    return (_hdist(tip, (g.pos[0], g.pos[1])) < LAYOUT["glass"]["inner_radius"]
            and _z(tip) < rim - 0.010)


def _knife_set(o):
    tip, butt = _knife_ends(o["knife"])
    rim = LAYOUT["glass"]["height"]
    tilt = _knife_axis_tilt_deg(o["knife"])
    g = o["glass"]
    return (_knife_in_glass(o)
            and _z(tip) <= rim - TH["tip_below_rim"]
            and _z(butt) >= rim + TH["butt_above_rim"]
            and TH["knife_tilt_min_deg"] <= tilt <= TH["knife_tilt_max_deg"]
            and _tilt_deg(g.quat_xyzw) <= TH["glass_ok_deg"]
            and _hdist(g.pos, LAYOUT["glass"]["xy"]) <= TH["glass_ok_shift"])


def _all_rest(o):
    return all(_rest(o[n]) for n in ("apple", "knife", "glass", "mug", "towel_sheet", "towel_pad"))


def _nominal(o):
    for name, spec in LAYOUT.items():
        if name not in o:
            return False
        if _hdist(o[name].pos, spec["xy"]) > TH["nominal_tol"]:
            return False
    return True


# ---------------------------------------------------------------------------
# History: latches only irreversible, instantaneous events. Updated in
# success() only; score() and distance() never write.
# ---------------------------------------------------------------------------
_STATE = {}


def _fresh_state():
    return {
        "steps": 0,
        "prev": None,
        "bad_layout": False,
        "zero": None,  # reason string for a permanent zero
        "cap_half": None,  # reason string for the 0.5 cap
        "apple_bruised": False,
        "mug_tilt_max": 0.0,
        "mug_shift_max": 0.0,
    }


_STATE.update(_fresh_state())


def _positions(o):
    return {n: tuple(o[n].pos) for n in LAYOUT if n in o}


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

    if _STATE["zero"] is None:
        if _mug_spilt(o):
            _STATE["zero"] = "coffee spilt"
        elif _glass_fallen(o):
            _STATE["zero"] = "glass knocked over"
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
    # Final-frame recheck without writing history.
    if _mug_spilt(o):
        return "coffee spilt"
    if _glass_fallen(o):
        return "glass knocked over"
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
    if not _knife_in_glass(o):
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
            "task_progress steps=%d fail=%s cap=%s score=%.2f mug_tilt_max=%.2fdeg mug_shift_max=%.1fmm"
            % (
                _STATE["steps"],
                _zero_reason(objects),
                _cap_reason(objects),
                score(objects),
                _STATE["mug_tilt_max"],
                _STATE["mug_shift_max"] * 1000,
            )
        )
    return ok


def score(objects):
    """Partial credit, read-only. 1.0 is reached only through success()."""
    return min(_tier(objects), 0.75)


def distance(objects):
    """Read-only progress measure in metres (lower is better)."""
    a = objects["apple"]
    d = _hdist(a.pos, LAYOUT["bowl"]["xy"])
    if not _towels_cleared(objects):
        d += 0.25
    tip, _ = _knife_ends(objects["knife"])
    g = objects["glass"]
    d += math.dist(tip, (g.pos[0], g.pos[1], TABLE_Z + LAYOUT["glass"]["base"]))
    return d


# OBJECTS and demo() are added after the yam_arena constructors have been
# inspected (Box/Cylinder/Asset signatures, asset paths, Waypoint).
