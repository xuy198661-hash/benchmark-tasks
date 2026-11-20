"""Three silver aluminium bars lie flat beside a central dark gauge base.
Two short pairs of anvils form its wider GO gap and narrower NO-GO gap.
A green tray is on the left and a red tray is on the right. One loose bar
starts on the left and two start on the right; all long edges face forward.
"""
from collections import deque
import json
import math
import os
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.constants import TCP_OFFSET
from yam_arena.urdf import quat_xyzw_to_matrix

HERE = Path(__file__).resolve().parent
GEOMETRY = json.loads((HERE / 'geometry.json').read_text())
SCENE = 'station'
TABLE_Z = TABLE_TOP[SCENE]
SCORE_OVER = 'end'
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20
RUN_LIMIT = 2600
OBJECTS = [Box(**row) for row in GEOMETRY['objects']]
SPECS = {o.name: o for o in OBJECTS}
BLOCKS = tuple(o.name for o in OBJECTS if o.free)
WIDTHS = tuple(GEOMETRY['widths_m'])
GAPS = GEOMETRY['gap_centres']
TRAYS = GEOMETRY['tray_centres']
HEIGHT = GEOMETRY['height_m']
LENGTH = GEOMETRY['length_m']
GRIP_Z = GEOMETRY['grip_z_m']
GRIP_DX = GEOMETRY['grip_dx_m']
HOVER = TABLE_Z + GEOMETRY['hover_above_table_m']
BASE_TOP = TABLE_Z + GEOMETRY['base_top_m']
ANVIL_TOP = TABLE_Z + GEOMETRY['anvil_top_m']
FLOOR_TOP = TABLE_Z + GEOMETRY['floor_top_m']
REST_WINDOW = 10
REST_RADIUS = .002
DWELL_FRAMES = 7  # seven samples span six intervals = 0.3 s at 20 Hz
MAX_SPEED = .01
MAX_SPIN = .5
CONTACT_EPS = 5e-10  # floating-point roundoff only; no positive physical gap tolerance
MAX_PENETRATION = .0002
SUPPORT_MARGIN = .001
MIN_PATCH_AREA = 1e-6
MAX_GAUGE_YAW = math.radians(2)
TIP_LIMIT = math.radians(15)
TRAY_MARGIN = .001
START_RADIUS = .004
RESET_JUMP = .10
RELEASE_MARGIN = .005
GRIP_NEAR = .08
MAX_IMPACT_SPEED = .8

# Only success() advances episode history; score() and distance() never mutate it.
_step = 0
_started = False
_failed = ''
_last_positions = None
_windows = {n: deque(maxlen=REST_WINDOW) for n in BLOCKS}
_dwell = {n: {'go': 0, 'no_go': 0} for n in BLOCKS}
_checks = {n: {'go': None, 'no_go': None} for n in BLOCKS}
_grips = None
_grip_reason = 'not read'
_views = None
_view_identity = None


def _numpy(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu()
    return np.asarray(value.numpy() if hasattr(value, 'numpy') else value, dtype=float)


def read_grippers():
    """Read only public Isaac Lab / physics-view APIs, after simulator startup.

    No embodiment method wrapping, monkeypatching, or yam_arena private state.
    This exact public-API path still requires a GPU smoke test.
    """
    global _views, _view_identity
    try:
        from isaaclab.sim import SimulationContext
        sim = SimulationContext.instance()
        if sim is None:
            return None, 'no SimulationContext'
        physics = sim.physics_sim_view
        if physics is None:
            return None, 'no public physics_sim_view'
        if _views is None or _view_identity != id(physics):
            views = []
            for name in ('LeftArm', 'RightArm'):
                view = physics.create_articulation_view('/World/envs/env_0/' + name)
                if view.count != 1:
                    raise ValueError('expected one ' + name)
                views.append(view)
            _views, _view_identity = views, id(physics)
        readings = []
        for view in _views:
            meta = view.shared_metatype
            names, links = list(meta.dof_names), list(meta.link_names)
            q = _numpy(view.get_dof_positions())[0]
            pose = _numpy(view.get_link_transforms())[0, links.index('gripper')]
            rotation = quat_xyzw_to_matrix(pose[3:7])
            gap = -float(q[names.index('joint7')] + q[names.index('joint8')])
            tcp = pose[:3] + rotation @ np.array(TCP_OFFSET)
            if not (np.all(np.isfinite(tcp)) and -.0001 <= gap <= .0941):
                raise ValueError('invalid gripper state')
            readings.append({'tcp': tcp, 'gap': gap, 'axis': rotation[:, 0]})
        return readings, 'ok'
    except Exception as error:
        _views = None
        return None, 'gripper unavailable: ' + type(error).__name__


def _valid(objects):
    try:
        for obj in OBJECTS:
            state = objects[obj.name]
            for value, length in ((state.pos, 3), (state.quat_xyzw, 4),
                                  (state.lin_vel, 3), (state.ang_vel, 3),
                                  (state.low, 3), (state.high, 3)):
                if np.shape(value) != (length,) or not np.all(np.isfinite(value)):
                    return False
            if np.linalg.norm(state.quat_xyzw) < .5:
                return False
            if not np.allclose((state.low+state.high)/2, state.pos, atol=.0005, rtol=0):
                return False
            # Detect changed/destroyed shapes instead of trusting only their AABBs.
            r = quat_xyzw_to_matrix(state.quat_xyzw)
            if not np.allclose(state.high - state.low, np.abs(r) @ obj.size, atol=.0005, rtol=0):
                return False
        return True
    except (KeyError, ValueError, TypeError, AttributeError):
        return False


def _nominal(objects):
    if not _valid(objects):
        return False
    for obj in OBJECTS:
        state = objects[obj.name]
        if np.linalg.norm(state.pos - obj.pos) > (START_RADIUS if obj.free else .0005):
            return False
        r = quat_xyzw_to_matrix(state.quat_xyzw)
        if r[2, 2] < math.cos(math.radians(1)):
            return False
        if abs(r[0, 0]) < math.cos(math.radians(2)):
            return False
    return True


def _bottom(state, size):
    corners = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1]])
    r = quat_xyzw_to_matrix(state.quat_xyzw)
    return (corners * np.asarray(size) / 2) @ r.T + state.pos


def _clip(poly, low, high):
    """Clip an ordered XY polygon to an axis-aligned support rectangle."""
    poly = np.asarray(poly, dtype=float)
    for axis, boundary, sign in ((0, low[0], 1), (0, high[0], -1),
                                 (1, low[1], 1), (1, high[1], -1)):
        out = []
        if not len(poly):
            return np.empty((0, 2))
        for a, b in zip(poly, np.roll(poly, -1, axis=0)):
            da, db = sign * (a[axis] - boundary), sign * (b[axis] - boundary)
            if da >= 0:
                out.append(a)
            if (da >= 0) != (db >= 0):
                out.append(a + da / (da - db) * (b - a))
        poly = np.asarray(out)
    return poly


def _hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for seq, chain in ((points, lower), (reversed(points), upper)):
        for p in seq:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], p) <= 0:
                chain.pop()
            chain.append(p)
    return np.array(lower[:-1] + upper[:-1])


def _inside(poly, point, margin=0.):
    if len(poly) < 3:
        return False
    edge = np.roll(poly, -1, axis=0) - poly
    delta = point - poly
    signed = (edge[:, 0]*delta[:, 1] - edge[:, 1]*delta[:, 0]) / np.linalg.norm(edge, axis=1)
    return bool(np.all(signed >= margin - 1e-12))


def _supported(state, size, surfaces):
    """A non-gapped contact patch on every required surface, with COM inside hull.

    A surface is (XY-low, XY-high, top-z). No positive proximity band is accepted.
    Small *negative* separation allows solver penetration, never air clearance.
    """
    bottom = _bottom(state, size)
    normal = quat_xyzw_to_matrix(state.quat_xyzw)[:, 2]
    if normal[2] <= 0:
        return False
    patches = []
    for low, high, top in surfaces:
        poly = _clip(bottom[:, :2], low, high)
        if len(poly) < 3:
            return False
        area = abs(np.sum(poly[:, 0]*np.roll(poly[:, 1], -1) - poly[:, 1]*np.roll(poly[:, 0], -1))) / 2
        if area < MIN_PATCH_AREA:
            return False
        heights = bottom[0, 2] - ((poly-bottom[0, :2]) @ normal[:2]) / normal[2]
        gap = heights - top
        if np.max(gap) > CONTACT_EPS or np.min(gap) < -MAX_PENETRATION:
            return False
        patches.extend(poly)
    return _inside(_hull(patches), state.pos[:2], SUPPORT_MARGIN)


def _surface(name, objects):
    obj = objects[name]
    return np.asarray(obj.low[:2]), np.asarray(obj.high[:2]), float(obj.high[2])


def _released(name, objects):
    if _grips is None or len(_grips) != 2:
        return False
    obj, size = objects[name], np.asarray(SPECS[name].size)
    rotation = quat_xyzw_to_matrix(obj.quat_xyzw)
    for grip in _grips:
        tcp, axis, gap = np.asarray(grip['tcp']), np.asarray(grip['axis']), grip['gap']
        if not (np.all(np.isfinite(tcp)) and np.all(np.isfinite(axis)) and math.isfinite(gap)):
            return False
        separation = np.maximum(np.maximum(obj.low-tcp, tcp-obj.high), 0)
        if np.linalg.norm(separation) <= GRIP_NEAR:
            width = float(np.abs(rotation.T @ axis) @ size)
            if gap < width + RELEASE_MARGIN:
                return False
    return True


def _still(name, objects):
    samples = _windows[name]
    if len(samples) < REST_WINDOW:
        return objects[name].resting(max_speed=MAX_SPEED, max_spin=MAX_SPIN)
    # Check all pairs, including earlier positions; ignore frozen velocity readings.
    p = np.asarray([*samples, objects[name].pos])
    return bool(np.max(np.linalg.norm(p[:, None]-p[None, :], axis=2)) <= REST_RADIUS)


def _gauge_reading(name, gauge, objects):
    state, size = objects[name], SPECS[name].size
    r = quat_xyzw_to_matrix(state.quat_xyzw)
    if r[2, 2] < math.cos(math.radians(.5)) or abs(r[0, 0]) < math.cos(MAX_GAUGE_YAW):
        return None
    x, y, gap = GAPS[gauge]
    bottom = _bottom(state, size)
    # Entire footprint in the proper gauge lane; no neighbouring structure counts.
    if np.max(np.abs(bottom[:, 0]-x)) > .059 or abs(state.pos[1]-y) > .002:
        return None
    if np.max(np.abs(bottom[:, 1]-y)) < gap/2:
        if _supported(state, size, [_surface('gauge_base', objects)]):
            return 'seated'
    else:
        surfaces = [_surface(f'{gauge}_anvil_{i}', objects) for i in (0, 2)]
        if _supported(state, size, surfaces):
            return 'blocked'
    return None


def _tray(name, tray, objects):
    state, size = objects[name], SPECS[name].size
    r = quat_xyzw_to_matrix(state.quat_xyzw)
    if r[2, 2] < math.cos(math.radians(.5)):
        return False
    x, y = TRAYS[tray]
    footprint = _bottom(state, size)[:, :2]
    if np.any(np.abs(footprint-(x, y)) > np.array((.070, .110))-TRAY_MARGIN):
        return False
    return _supported(state, size, [_surface(tray+'_floor', objects)])


def _expected(name):
    width = SPECS[name].size[1]
    return 'green' if GAPS['no_go'][2] < width < GAPS['go'][2] else 'red'


def _complete(name):
    c = _checks[name]
    return c['go'] == 'blocked' or (c['go'] == 'seated' and c['no_go'] is not None)


def _credited(name, objects):
    return (_complete(name) and _released(name, objects) and _still(name, objects)
            and _tray(name, _expected(name), objects))


def _count(objects):
    if not _started or _failed or not _valid(objects):
        return 0
    return sum(bool(_credited(name, objects)) for name in BLOCKS)


def success(objects):
    global _step, _started, _failed, _last_positions, _grips, _grip_reason
    _step += 1
    valid = _valid(objects)
    nominal = _nominal(objects) if valid else False
    positions = {name: np.array(objects[name].pos, copy=True) for name in BLOCKS} if valid else None
    jumped = (positions is not None and _last_positions is not None and any(
        np.linalg.norm(positions[n]-_last_positions[n]) > RESET_JUMP for n in BLOCKS))
    if nominal and jumped:
        _step = 1
        _started, _failed = True, ''
        for name in BLOCKS:
            _windows[name].clear()
            _dwell[name] = {'go': 0, 'no_go': 0}
            _checks[name] = {'go': None, 'no_go': None}
    elif not _started:
        _started = True
        if not nominal:
            _failed = 'invalid initial layout'
    _grips, _grip_reason = read_grippers()
    if not valid:
        _failed = _failed or 'missing, malformed or changed object'
    else:
        for obj in OBJECTS:
            state = objects[obj.name]
            r = quat_xyzw_to_matrix(state.quat_xyzw)
            if not obj.free:
                if np.linalg.norm(state.pos-obj.pos) > .0005 or np.max(np.abs(r-np.eye(3))) > .001:
                    _failed = _failed or 'fixed structure moved'
                continue
            name = obj.name
            _windows[name].append(np.array(state.pos, copy=True))
            if r[2, 2] < math.cos(TIP_LIMIT):
                _failed = _failed or 'block tipped'
            if (state.low[2] < TABLE_Z-.002 or not .03 <= state.pos[0] <= .55
                    or abs(state.pos[1]) > .57 or np.linalg.norm(state.lin_vel) > MAX_IMPACT_SPEED):
                _failed = _failed or 'block knocked out or impact'
            released, still = _released(name, objects), _still(name, objects)
            for gauge in ('go', 'no_go'):
                permitted = gauge == 'go' or _checks[name]['go'] == 'seated'
                reading = _gauge_reading(name, gauge, objects) if permitted and released and still else None
                _dwell[name][gauge] = _dwell[name][gauge]+1 if reading else 0
                if reading and _dwell[name][gauge] >= DWELL_FRAMES and _checks[name][gauge] is None:
                    _checks[name][gauge] = reading
            if released and still:
                for tray in TRAYS:
                    if _tray(name, tray, objects) and (tray != _expected(name) or not _complete(name)):
                        _failed = _failed or 'wrong or uninspected tray placement'
    _last_positions = positions if positions is not None else _last_positions
    done = not _failed and _count(objects) == len(BLOCKS)
    if os.environ.get('TASK_DEBUG') and (_step == 1 or _step % 20 == 0):
        contact_debug = None
        if valid:
            contact_debug = {}
            for name in BLOCKS:
                z = _bottom(objects[name], SPECS[name].size)[:, 2]
                contact_debug[name] = {
                    'bottom_base_gap_mm': [float((z.min()-BASE_TOP)*1000), float((z.max()-BASE_TOP)*1000)],
                    'bottom_anvil_gap_mm': [float((z.min()-ANVIL_TOP)*1000), float((z.max()-ANVIL_TOP)*1000)],
                    'reading': {g: _gauge_reading(name, g, objects) for g in GAPS}}
        print('task_progress ' + json.dumps({'step': _step, 'checks': _checks,
              'credited': _count(objects), 'failed': _failed, 'gripper': _grip_reason,
              'pad_gaps_mm': None if _grips is None else [g['gap']*1000 for g in _grips],
              'tcp_m': None if _grips is None else [np.asarray(g['tcp']).tolist() for g in _grips],
              'contact': contact_debug}), flush=True)
    return bool(done)


def score(objects):
    # Highest achieved final placement tier, never an accumulated reward.
    count = _count(objects)
    return .75 if count >= 3 else .5 if count >= 2 else .25 if count >= 1 else 0.


def distance(objects):
    return float(len(BLOCKS)-_count(objects))  # remaining released, inspected placements


def demo():
    """Nominal geometric demonstration; all credited checks still require real support.

    Uses known manufacturing widths for open-loop release heights and destination
    routing. It demonstrates mechanics, not a perception-driven inspection policy.
    """
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
    out = {'left': [], 'right': []}
    parks = {'left': (.10, .40, HOVER), 'right': (.10, -.40, HOVER)}
    def add(side, duration, pos, finger, label):
        out[side].append(Waypoint(duration, tuple(pos), finger, label))
        other = 'right' if side == 'left' else 'left'
        out[other].append(Waypoint(duration, parks[other], FINGER_OPEN, 'park'))
    for s in out:
        out[s].append(Waypoint(2., parks[s], FINGER_OPEN, 'initial park'))
    for i, name in enumerate(BLOCKS, 1):
        x, y, _ = SPECS[name].pos
        width = SPECS[name].size[1]
        side = 'left' if y > 0 else 'right'
        def pick(x, y, bottom, where):
            p = (x+GRIP_DX, y, bottom+GRIP_Z)
            for duration, pos, grip, label in (
                (1.5, (*p[:2], HOVER), FINGER_OPEN, 'approach'),
                (1.5, p, FINGER_OPEN, 'descend'), (.8, p, FINGER_CLOSED, 'close'),
                (1.2, (*p[:2], HOVER), FINGER_CLOSED, 'lift')):
                add(side, duration, pos, grip, f'{i} {where} {label}')
        def place(x, y, bottom, where):
            p = (x+GRIP_DX, y, bottom+GRIP_Z+.004)
            for duration, pos, grip, label in (
                (1.8, (*p[:2], HOVER), FINGER_CLOSED, 'carry'),
                (1.2, p, FINGER_CLOSED, 'lower'), (.5, p, FINGER_CLOSED, 'hold'),
                (.7, p, FINGER_OPEN, 'release'), (1., (*p[:2], HOVER), FINGER_OPEN, 'retreat'),
                (.8, (*p[:2], HOVER), FINGER_OPEN, 'settle')):
                add(side, duration, pos, grip, f'{i} {where} {label}')
        pick(x, y, TABLE_Z, 'start')
        for gauge, (gx, gy, gap) in GAPS.items():
            bottom = BASE_TOP if width < gap else ANVIL_TOP
            place(gx, gy, bottom, gauge)
            last = (gx, gy, bottom, gauge)
            if gauge == 'go' and width > gap:
                break
            if gauge == 'go':
                pick(*last)
        tray = _expected(name)
        destination = 'left' if tray == 'green' else 'right'
        if destination != side:
            add(side, 1.5, parks[side], FINGER_OPEN, f'{i} handoff source clear')
            side = destination
        pick(*last)
        tx, ty = TRAYS[tray]
        if tray == 'red':
            ty += -.04 if i == 1 else .04
        place(tx, ty, FLOOR_TOP, 'tray')
        add(side, 1.5, parks[side], FINGER_OPEN, f'{i} park')
    for s in out:
        out[s].append(Waypoint(1.5, parks[s], FINGER_OPEN, 'finish'))
    return out
