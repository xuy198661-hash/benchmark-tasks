"""A kitchen counter after someone cut fruit. In the middle of the counter a small fruit knife lies flat,
out of sight: a folded paper-towel pad covers its blade, and a larger white paper towel is laid over
both. A red apple rests on top of the towel. To the left is an empty white bowl. To the right is a white
mug full of hot coffee, and further right, slightly nearer the robot, is an empty open container for
the knife.
"""

import math
import os

from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder, Sphere

SCENE = "station"
# TABLE_TOP is a dict keyed by scene: {"station": 0.7475, "tabletop": 0.75}.
TABLE_Z = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

# ---------------------------------------------------------------------------
# Library assets. The bowl's name is confirmed (yam_arena.station.ITEMS).
# The knife and holder names come from the Arena object library on the GPU
# machine and are still to be filled in.
# ---------------------------------------------------------------------------
BOWL_ASSET = "bowl_ycb_robolab"
KNIFE_ASSET = "TODO_knife_asset"
HOLDER_ASSET = "TODO_open_container_asset"

# VERIFY on GPU with TASK_DEBUG=1 (step 1 prints the knife's tip and butt):
# the knife asset's own axis from the handle end to the tip, and its height
# when lying flat.
KNIFE_LOCAL_AXIS = (1.0, 0.0, 0.0)
KNIFE_HEIGHT = 0.020

APPLE_RED = (0.629, 0.104, 0.138)  # yam_arena.station.APPLE_RED
BOWL_GLAZE = tuple(c / 255 for c in (219.5, 214.5, 203.5))  # yam_arena.station.BOWL_GLAZE
PAPER_WHITE = (0.95, 0.95, 0.93)
MUG_WHITE = (0.92, 0.91, 0.87)

# ---------------------------------------------------------------------------
# Layout (metres, world frame): x forward from the arm bases, y left, z above
# the table top. Development estimates: re-check with reach_map.py.
# ---------------------------------------------------------------------------
LAYOUT = {
    "knife": {"xy": (0.26, 0.00), "length": 0.170},
    "towel_pad": {"xy": (0.26, 0.045), "size": (0.120, 0.110, 0.008)},
    "towel_sheet": {"xy": (0.26, 0.00), "size": (0.260, 0.220, 0.0015)},
    "apple": {"xy": (0.26, 0.04), "radius": 0.040},
    "bowl": {"xy": (0.24, 0.26), "extent": 0.150},
    "mug": {"xy": (0.24, -0.17), "radius": 0.0425, "height": 0.095},
    "holder": {"xy": (0.22, -0.30), "extent": 0.072},
}
# The knife's starting footprint; the towels must end up off it.
KNIFE_ZONE = ((0.240, -0.095), (0.280, 0.095))

_HALF = LAYOUT["knife"]["length"] / 2
KNIFE_TIP_LOCAL = tuple(_HALF * a for a in KNIFE_LOCAL_AXIS)
KNIFE_BUTT_LOCAL = tuple(-_HALF * a for a in KNIFE_LOCAL_AXIS)
# Yaw that turns the asset's axis to +y, so the tip points left and the handle right.
KNIFE_YAW = math.pi / 2 - math.atan2(KNIFE_LOCAL_AXIS[1], KNIFE_LOCAL_AXIS[0])

_PAD_Z = KNIFE_HEIGHT + LAYOUT["towel_pad"]["size"][2] / 2 + 0.002
_SHEET_Z = KNIFE_HEIGHT + LAYOUT["towel_pad"]["size"][2] + 0.003
_APPLE_Z = _SHEET_Z + LAYOUT["apple"]["radius"] + 0.003

OBJECTS = [
    Asset("knife", KNIFE_ASSET, xy=LAYOUT["knife"]["xy"], extent=LAYOUT["knife"]["length"], mass=0.04, yaw=KNIFE_YAW),
    Box("towel_pad", pos=(*LAYOUT["towel_pad"]["xy"], TABLE_Z + _PAD_Z), size=LAYOUT["towel_pad"]["size"],
        mass=0.01, color=PAPER_WHITE),
    Box("towel_sheet", pos=(*LAYOUT["towel_sheet"]["xy"], TABLE_Z + _SHEET_Z), size=LAYOUT["towel_sheet"]["size"],
        mass=0.01, color=PAPER_WHITE),
    Sphere("apple", pos=(*LAYOUT["apple"]["xy"], TABLE_Z + _APPLE_Z), radius=LAYOUT["apple"]["radius"],
           mass=0.19, color=APPLE_RED),
    Asset("bowl", BOWL_ASSET, xy=LAYOUT["bowl"]["xy"], extent=LAYOUT["bowl"]["extent"], color=BOWL_GLAZE),
    Cylinder("mug", pos=(*LAYOUT["mug"]["xy"], TABLE_Z + LAYOUT["mug"]["height"] / 2),
             radius=LAYOUT["mug"]["radius"], height=LAYOUT["mug"]["height"], mass=0.38, color=MUG_WHITE),
    Asset("holder", HOLDER_ASSET, xy=LAYOUT["holder"]["xy"], extent=LAYOUT["holder"]["extent"], mass=0.22),
]

# Thresholds: development estimates, to be calibrated on GPU (see README).
TH = {
    "rest_speed": 0.01,  # m/s
    "rest_spin": 0.10,  # rad/s
    "mug_tilt_deg": 8.0,  # spill proxy
    "mug_shift": 0.015,  # spill proxy
    "holder_fall_deg": 30.0,  # knocked over
    "holder_ok_deg": 5.0,
    "holder_ok_shift": 0.030,
    "holder_wall": 0.004,  # inner radius = half the holder's width minus this
    "keepout_radius": 0.0725,  # mug radius + 30 mm, no height limit
    "lift": 0.015,  # knife's lowest point above the table
    "touch": 0.003,  # lowest point this close to the table counts as touching
    "drop_speed": 0.8,  # m/s downward
    "apple_axis": 0.035,
    "apple_below_rim": 0.005,
    "tip_below_rim": 0.025,
    "butt_above_rim": 0.040,
    "knife_tilt_min_deg": 12.0,  # below this it cannot stand on its own
    "knife_tilt_max_deg": 45.0,
    "towel_high_max": 0.015,  # a cleared towel lies flat on the table
    "container_clear": 0.060,  # a towel centre this close to a container is in or on it
    "reset_jump": 0.08,  # m per call, beyond any real motion
    "nominal_tol": 0.010,
    "layout_tol": 0.020,
}

DEBUG = os.environ.get("TASK_DEBUG") == "1"


# ---------------------------------------------------------------------------
# Geometry helpers (pure functions)
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
    return float(p[2]) - TABLE_Z


def _hdist(p, xy):
    return math.hypot(float(p[0]) - xy[0], float(p[1]) - xy[1])


def _xy(o):
    return (float(o.pos[0]), float(o.pos[1]))


def _point(o, local):
    d = _rotate(o.quat_xyzw, local)
    return (float(o.pos[0]) + d[0], float(o.pos[1]) + d[1], float(o.pos[2]) + d[2])


def _knife_ends(k):
    return _point(k, KNIFE_TIP_LOCAL), _point(k, KNIFE_BUTT_LOCAL)


def _knife_axis_tilt_deg(k):
    tip, butt = _knife_ends(k)
    dx, dy, dz = butt[0] - tip[0], butt[1] - tip[1], butt[2] - tip[2]
    n = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-9
    return math.degrees(math.acos(max(-1.0, min(1.0, dz / n))))


def _rects_overlap(a_low, a_high, b_low, b_high):
    return (a_low[0] < b_high[0] and b_low[0] < a_high[0]
            and a_low[1] < b_high[1] and b_low[1] < a_high[1])


def _rest(o):
    return o.resting(max_speed=TH["rest_speed"], max_spin=TH["rest_spin"])


# Container geometry is read from the holder's own bounds, not hard-coded.
def _holder_rim(o):
    return _z(o["holder"].high)


def _holder_inner_radius(o):
    h = o["holder"]
    return min(float(h.high[0] - h.low[0]), float(h.high[1] - h.low[1])) / 2 - TH["holder_wall"]


# ---------------------------------------------------------------------------
# Instantaneous checks
# ---------------------------------------------------------------------------
def _mug_spilt(o):
    m = o["mug"]
    return (_tilt_deg(m.quat_xyzw) > TH["mug_tilt_deg"]
            or _hdist(m.pos, LAYOUT["mug"]["xy"]) > TH["mug_shift"])


def _holder_fallen(o):
    return _tilt_deg(o["holder"].quat_xyzw) > TH["holder_fall_deg"]


def _knife_lifted(o):
    return _z(o["knife"].low) > TH["lift"]


def _tip_in_holder(o):
    tip, _ = _knife_ends(o["knife"])
    return (_hdist(tip, _xy(o["holder"])) < _holder_inner_radius(o)
            and _z(tip) < _holder_rim(o) - 0.010)


def _knife_in_keepout(o):
    """The knife is being carried near the mug. A knife already standing in the holder is not carried."""
    if not _knife_lifted(o) or _tip_in_holder(o):
        return False
    tip, butt = _knife_ends(o["knife"])
    mug_xy = _xy(o["mug"])
    for i in range(5):
        t = i / 4
        p = (tip[0] + t * (butt[0] - tip[0]), tip[1] + t * (butt[1] - tip[1]))
        if _hdist(p, mug_xy) < TH["keepout_radius"]:
            return True
    return False


def _knife_dropped(o):
    return float(o["knife"].lin_vel[2]) < -TH["drop_speed"]


def _knife_off_table(o):
    return _z(o["knife"].pos) < -0.02


def _apple_touches_table(o):
    return _z(o["apple"].low) < TH["touch"]


def _apple_in_bowl(o):
    a, b = o["apple"], o["bowl"]
    return (_hdist(a.pos, _xy(b)) < TH["apple_axis"]
            and _z(a.pos) < _z(b.high) - TH["apple_below_rim"])


def _towels_cleared(o):
    zlow, zhigh = KNIFE_ZONE
    for name in ("towel_sheet", "towel_pad"):
        t = o[name]
        if _z(t.high) > TH["towel_high_max"]:
            return False
        if _rects_overlap(t.low, t.high, zlow, zhigh):
            return False
        for c in ("bowl", "mug", "holder"):
            if _hdist(t.pos, _xy(o[c])) < TH["container_clear"]:
                return False
    return True


def _knife_set(o):
    tip, butt = _knife_ends(o["knife"])
    rim = _holder_rim(o)
    tilt = _knife_axis_tilt_deg(o["knife"])
    h = o["holder"]
    return (_tip_in_holder(o)
            and _z(tip) <= rim - TH["tip_below_rim"]
            and _z(butt) >= rim + TH["butt_above_rim"]
            and TH["knife_tilt_min_deg"] <= tilt <= TH["knife_tilt_max_deg"]
            and _tilt_deg(h.quat_xyzw) <= TH["holder_ok_deg"]
            and _hdist(h.pos, LAYOUT["holder"]["xy"]) <= TH["holder_ok_shift"])


def _all_rest(o):
    return all(_rest(o[n]) for n in ("apple", "knife", "holder", "mug", "towel_sheet", "towel_pad"))


def _near_layout(o, tol):
    return all(n in o and _hdist(o[n].pos, s["xy"]) <= tol for n, s in LAYOUT.items())


# ---------------------------------------------------------------------------
# History: latches irreversible, instantaneous events. Written in success()
# only (the framework calls it after every control step); score() and
# distance() never write.
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
    }


_STATE.update(_fresh_state())


def _positions(o):
    return {n: tuple(float(v) for v in o[n].pos) for n in LAYOUT if n in o}


def _episode_started(o):
    prev = _STATE["prev"]
    if prev is None:
        return True
    jumped = any(n in prev and math.dist(prev[n], p) > TH["reset_jump"] for n, p in _positions(o).items())
    return jumped and _near_layout(o, TH["nominal_tol"])


def _observe(o):
    if _episode_started(o):
        _STATE.clear()
        _STATE.update(_fresh_state())
        _STATE["bad_layout"] = not _near_layout(o, TH["layout_tol"])
    _STATE["steps"] += 1
    _STATE["prev"] = _positions(o)

    m = o["mug"]
    _STATE["mug_tilt_max"] = max(_STATE["mug_tilt_max"], _tilt_deg(m.quat_xyzw))
    _STATE["mug_shift_max"] = max(_STATE["mug_shift_max"], _hdist(m.pos, LAYOUT["mug"]["xy"]))

    if _STATE["zero"] is None:
        if _mug_spilt(o):
            _STATE["zero"] = "coffee spilt"
        elif _holder_fallen(o):
            _STATE["zero"] = "holder knocked over"
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
    if _holder_fallen(o):
        return "holder knocked over"
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
    if not (_apple_in_bowl(o) and _rest(o["apple"]) and not _STATE.get("apple_bruised")):
        return 0.0
    if not _towels_cleared(o):
        return 0.25
    if not _tip_in_holder(o):
        return 0.5
    t = 1.0 if (_knife_set(o) and _all_rest(o)) else 0.75
    if _cap_reason(o):
        t = min(t, 0.5)
    return t


def _debug(o):
    s = _STATE
    line = "task_progress steps=%d fail=%s cap=%s score=%.2f mug_tilt_max=%.2fdeg mug_shift_max=%.1fmm" % (
        s["steps"], _zero_reason(o), _cap_reason(o), score(o), s["mug_tilt_max"], s["mug_shift_max"] * 1000)
    if s["steps"] == 1:
        tip, butt = _knife_ends(o["knife"])
        line += (" | knife_tip=(%.3f,%.3f,%.3f) knife_butt=(%.3f,%.3f,%.3f) knife_low_z=%.4f knife_high_z=%.4f"
                 " holder_rim=%.4f holder_inner_r=%.4f bowl_rim=%.4f" % (
                     *tip, *butt, _z(o["knife"].low), _z(o["knife"].high),
                     _holder_rim(o), _holder_inner_radius(o), _z(o["bowl"].high)))
    print(line)


# ---------------------------------------------------------------------------
# Task-file contract
# ---------------------------------------------------------------------------
def success(objects):
    _observe(objects)
    ok = _tier(objects) == 1.0
    if DEBUG and _STATE["steps"] % 20 in (0, 1):
        _debug(objects)
    return ok


def score(objects):
    """Partial credit, read-only. 1.0 is reached only through success()."""
    return min(_tier(objects), 0.75)


def distance(objects):
    """Read-only progress measure in metres (lower is better)."""
    d = _hdist(objects["apple"].pos, _xy(objects["bowl"]))
    if not _towels_cleared(objects):
        d += 0.25
    tip, _ = _knife_ends(objects["knife"])
    h = objects["holder"]
    d += math.dist(tip, (float(h.pos[0]), float(h.pos[1]), float(h.low[2]) + 0.012))
    return d


# demo() is added once yam_arena.demo.Waypoint has been inspected: standing the
# knife up needs a tilted grip that pick_place_waypoints does not produce.
