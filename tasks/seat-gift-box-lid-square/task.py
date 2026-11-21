"""A burgundy gift-box base with a small wooden gift stands on the right half of the table.
Its pale matching lid sits beside it, nearer the right arm, with the flat top facing up.
The box has an open cavity and the lid has a continuous skirt; the table is otherwise clear.
"""
from __future__ import annotations

from collections import deque
import itertools
import importlib.util
from functools import lru_cache
import json
import math
import os
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET
from yam_arena.demo import Waypoint
from yam_arena.task_file import Asset, Box, TABLE_TOP

HERE = Path(__file__).resolve().parent
SCENE = "station"
TABLE_Z = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
CONTROL_HZ = 20
RUN_LIMIT = 1200
INFO = {n: json.loads((HERE / "assets" / (n + "_info.json")).read_text())
        for n in ("base", "lid", "layout", "gripper")}
D = INFO["layout"]
GRIP_TARGET = -float(D["grip_target_gap_m"])/2
# All thresholds are development estimates; physical rationale is documented in README.
T = dict(start_xy=.003, start_z=.003, start_angle_deg=2., reset_jump=.10,
         base_drift=.020, base_height=.003, base_tilt_deg=2.,
         topple_base_deg=15., topple_lid_deg=30., below_table=.010,
         flight_speed=.75, gift_margin=.001, rest_position=.002,
         rest_angle_deg=.5, rest_speed=.010, rest_spin=.10,
         hands_clear=.030, seat_height=.0015, center=.0005,
         yaw_deg=1.5, tilt_deg=1., partial_center=.001,
         partial_yaw_deg=3., partial_tilt_deg=2., engagement=.010,
         engagement_center=.008, engagement_tilt_deg=10.,
         rim_height=.002, rim_center=.025, rim_tilt_deg=5.)

try:
    from isaaclab_arena.assets.registries import AssetRegistry
except ModuleNotFoundError:
    AssetRegistry = None

class BaseAsset:
    registry_name = "seat_gift_box_lid_square_base"
    tags = ["object", "graspable"]
    usd_path = str(HERE / "assets" / "base.usda")

class LidAsset:
    registry_name = "seat_gift_box_lid_square_lid"
    tags = ["object", "graspable"]
    usd_path = str(HERE / "assets" / "lid.usda")

if AssetRegistry is not None:
    for asset_class in (BaseAsset, LidAsset):
        if not AssetRegistry().is_registered(asset_class.registry_name, ensure_loaded=False):
            AssetRegistry().register(asset_class, asset_class.registry_name)

GIFT_SIZE = np.array(D["gift_size_m"])
OBJECTS = [
    Asset("base", BaseAsset.registry_name, xy=tuple(float(v) for v in D["base_xy_m"]),
          extent=float(INFO["base"]["extent_m"]), mass=float(INFO["base"]["mass_kg"]),
          color=(.52, .13, .20), yaw=float(D["base_yaw_rad"]), convex=False),
    Asset("lid", LidAsset.registry_name, xy=tuple(float(v) for v in D["lid_xy_m"]),
          extent=float(INFO["lid"]["extent_m"]), mass=float(INFO["lid"]["mass_kg"]),
          color=(.88, .75, .60), yaw=float(D["lid_yaw_rad"]), convex=False),
    Box("gift", pos=(*(float(v) for v in D["base_xy_m"]),
                     float(TABLE_Z + D["spawn_gap_m"] + INFO["base"]["panel_high_m"]
                           + D["spawn_gap_m"] + GIFT_SIZE[2]/2)),
        size=tuple(float(v) for v in GIFT_SIZE), color=(.60, .39, .17),
        mass=float(np.prod(GIFT_SIZE) * D["gift_density_kg_m3"]), yaw=float(0)),
]
SIZES = {"base": np.array(INFO["base"]["size_m"]),
         "lid": np.array(INFO["lid"]["size_m"]), "gift": GIFT_SIZE}
CORNERS = np.array(list(itertools.product((-1, 1), repeat=3)))
NOMINAL = {
    "base": np.r_[D["base_xy_m"], TABLE_Z+D["spawn_gap_m"]+SIZES["base"][2]/2],
    "lid": np.r_[D["lid_xy_m"], TABLE_Z+D["spawn_gap_m"]+SIZES["lid"][2]/2],
    "gift": np.array(OBJECTS[2].pos),
}


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError("invalid quaternion")
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def tilt_deg(q):
    return float(np.degrees(np.arccos(np.clip(rotation(q)[2, 2], -1., 1.))))


def angle_deg(a, b):
    c = (np.trace(rotation(a).T @ rotation(b))-1)/2
    return float(np.degrees(np.arccos(np.clip(c, -1., 1.))))


def yaw_gap_deg(a, b):
    # A rectangular box is equivalent after a half turn about the vertical.
    delta = math.atan2(rotation(a)[1, 0], rotation(a)[0, 0])-math.atan2(rotation(b)[1, 0], rotation(b)[0, 0])
    return abs(math.degrees((delta+math.pi/2) % math.pi-math.pi/2))


def vertices(name, state):
    return (CORNERS*SIZES[name]/2) @ rotation(state.quat_xyzw).T + state.pos


def fingerprint(objects):
    values = []
    for n in SIZES:
        o = objects[n]
        for attr, shape in (("pos", (3,)), ("quat_xyzw", (4,)), ("lin_vel", (3,)),
                            ("ang_vel", (3,)), ("low", (3,)), ("high", (3,))):
            a = np.asarray(getattr(o, attr), dtype=float)
            if a.shape != shape or not np.all(np.isfinite(a)):
                raise ValueError("invalid object state")
            values.extend(a.tolist())
        rotation(o.quat_xyzw)
    return tuple(values)


def nominal_layout(objects):
    return all(np.linalg.norm(objects[n].pos[:2]-NOMINAL[n][:2]) <= T["start_xy"]
               and abs(objects[n].pos[2]-NOMINAL[n][2]) <= T["start_z"]
               and tilt_deg(objects[n].quat_xyzw) <= T["start_angle_deg"]
               and yaw_gap_deg(objects[n].quat_xyzw, (0, 0, 0, 1)) <= T["start_angle_deg"]
               for n in SIZES)


def violations(objects):
    base, lid, gift = (objects[n] for n in ("base", "lid", "gift"))
    bad = []
    if np.linalg.norm(base.pos[:2]-NOMINAL["base"][:2]) > T["base_drift"]:
        bad.append("base_moved")
    if tilt_deg(base.quat_xyzw) > T["topple_base_deg"]:
        bad.append("base_toppled")
    if tilt_deg(lid.quat_xyzw) > T["topple_lid_deg"]:
        bad.append("lid_toppled")
    for n in ("base", "lid"):
        if vertices(n, objects[n])[:, 2].min() < TABLE_Z-T["below_table"]:
            bad.append(n+"_fell")
    if any(np.linalg.norm(objects[n].lin_vel) > T["flight_speed"] for n in SIZES):
        bad.append("object_launched")
    # True gift corners in the base's frame, not rotated world AABBs.
    local = (vertices("gift", gift)-base.pos) @ rotation(base.quat_xyzw)
    inner = np.array(INFO["base"]["inner_xy_m"])/2
    floor = INFO["base"]["panel_high_m"]-SIZES["base"][2]/2
    if (np.any(np.abs(local[:, :2]) > inner+T["gift_margin"])
            or local[:, 2].min() < floor-T["gift_margin"]
            or local[:, 2].max() > SIZES["base"][2]/2+T["gift_margin"]):
        bad.append("gift_left_box")
    return tuple(bad)


def geometry_measurements(objects):
    b, l = objects["base"], objects["lid"]
    center = float(np.linalg.norm(l.pos[:2]-b.pos[:2]))
    top = l.pos[2]+SIZES["lid"][2]/2
    bottom = l.pos[2]-SIZES["lid"][2]/2
    rim = b.pos[2]+SIZES["base"][2]/2
    return dict(center_m=center, top_error_m=top-(rim+INFO["lid"]["panel_thickness_m"]),
                bottom_relative_to_rim_m=bottom-rim,
                yaw_deg=yaw_gap_deg(l.quat_xyzw, b.quat_xyzw), tilt_deg=tilt_deg(l.quat_xyzw),
                base_ok=bool(abs(b.pos[2]-(TABLE_Z+SIZES["base"][2]/2)) <= T["base_height"]
                         and tilt_deg(b.quat_xyzw) <= T["base_tilt_deg"]))


def geometry_tier(m):
    if not m["base_ok"]:
        return 0.
    if (abs(m["top_error_m"]) <= T["seat_height"] and m["center_m"] <= T["partial_center"]
            and m["yaw_deg"] <= T["partial_yaw_deg"] and m["tilt_deg"] <= T["partial_tilt_deg"]):
        return .75
    if (m["bottom_relative_to_rim_m"] <= -T["engagement"]
            and m["bottom_relative_to_rim_m"] >= -INFO["lid"]["skirt_height_m"]-T["seat_height"]
            and m["center_m"] <= T["engagement_center"] and m["tilt_deg"] <= T["engagement_tilt_deg"]):
        return .5
    if (abs(m["bottom_relative_to_rim_m"]) <= T["rim_height"]
            and m["center_m"] <= T["rim_center"] and m["tilt_deg"] <= T["rim_tilt_deg"]):
        return .25
    return 0.


def to_numpy(value):
    # Tensor frontends differ. These conversions are read-only.
    if hasattr(value, "numpy"):
        try:
            return np.asarray(value.numpy(), dtype=float)
        except (TypeError, RuntimeError):
            pass
    if hasattr(value, "detach"):
        return np.asarray(value.detach().cpu().numpy(), dtype=float)
    return np.asarray(value, dtype=float)


def read_grippers():
    """Public Isaac Lab/PhysX reads only. No embodiment hooks, writes, or private attributes.

    Native PhysX link transforms are xyz + XYZW. This route still needs GPU smoke validation.
    Views are deliberately reacquired so an episode/stage reset cannot leave cached handles.
    """
    phase = "simulation_context"
    try:
        from isaaclab.sim import SimulationContext
        sim = SimulationContext.instance()
        if sim is None:
            return None, "simulation_context_unavailable"
        out = {}
        for side, prim in (("left", "LeftArm"), ("right", "RightArm")):
            phase = "articulation_view_"+side
            view = sim.physics_sim_view.create_articulation_view("/World/envs/env_0/"+prim)
            if view.count != 1:
                return None, "articulation_count_"+side
            phase = "metadata_"+side
            meta = view.shared_metatype
            names, links = list(meta.dof_names), list(meta.link_names)
            phase = "positions_"+side
            q = to_numpy(view.get_dof_positions())[0]
            transforms = to_numpy(view.get_link_transforms())[0]
            fingers = np.array([q[names.index(n)] for n in ("joint7", "joint8")])
            if not np.all(np.isfinite(fingers)) or np.any(fingers < FINGER_OPEN-.002) or np.any(fingers > .002):
                return None, "invalid_finger_position_"+side
            poses = {n: transforms[links.index(n)].copy() for n in ("gripper", "tip_left", "tip_right")}
            if any(p.shape != (7,) or not np.all(np.isfinite(p)) for p in poses.values()):
                return None, "invalid_gripper_pose_"+side
            for p in poses.values():
                rotation(p[3:])
            out[side] = dict(gap_m=float(-fingers.sum()), poses=poses)
        return out, "ok"
    except Exception as exc:
        return None, "gripper_read_failed_"+phase+"_"+type(exc).__name__


def hands_clear(objects, hands):
    if hands is None or set(hands) != {"left", "right"}:
        return False
    for hand in hands.values():
        if not 0. <= hand["gap_m"] <= 2*abs(FINGER_OPEN)+.004:
            return False
        for link, bounds in INFO["gripper"]["link_bounds_m"].items():
            pose = hand["poses"][link]
            low, high = np.array(bounds["low"]), np.array(bounds["high"])
            points = ((low+high)/2+CORNERS*(high-low)/2) @ rotation(pose[3:]).T+pose[:3]
            gl, gh = points.min(0), points.max(0)
            for name in SIZES:
                v = vertices(name, objects[name]);lo,hi = v.min(0),v.max(0)
                gap = np.linalg.norm(np.maximum(np.maximum(lo-gh, gl-hi), 0))
                if gap < T["hands_clear"]:
                    return False
    return True


class Runtime:
    def __init__(self):
        self.initialized = False
        self.step = 0
        self.latches = set()
        self.previous = {}
        self.history = deque(maxlen=10)
        self.signature = None
        self.clear = False
        self.still = False
        self.read_reason = "not_observed"
        self.gaps = {}
        self.video_mode = False
        self.debug_grasp_reference = None

RUNTIME = Runtime()


def stable(objects, history):
    if len(history) < 10:
        return all(np.linalg.norm(o.lin_vel) <= T["rest_speed"]
                   and np.linalg.norm(o.ang_vel) <= T["rest_spin"] for o in objects.values())
    return all(np.linalg.norm(objects[n].pos-p[n][0]) <= T["rest_position"]
               and angle_deg(objects[n].quat_xyzw, p[n][1]) <= T["rest_angle_deg"]
               for n in SIZES for p in history)


def eligible(objects):
    try:
        return (RUNTIME.initialized and not RUNTIME.latches
                and fingerprint(objects) == RUNTIME.signature and not violations(objects)
                and RUNTIME.still and (RUNTIME.clear or RUNTIME.video_mode))
    except (KeyError, ValueError, TypeError, AttributeError):
        return False


def score(objects):
    """Read-only final-frame partial credit; full credit belongs to the framework's success hold."""
    return geometry_tier(geometry_measurements(objects)) if eligible(objects) else 0.


def distance(objects):
    """Read-only geometric diagnostic in metres; no history or latch updates."""
    try:
        fingerprint(objects)
        m = geometry_measurements(objects)
        return float(m["center_m"]+abs(m["top_error_m"])+.01*math.radians(m["yaw_deg"]+m["tilt_deg"]))
    except (KeyError, ValueError, TypeError, AttributeError):
        return 1.


def success(objects):
    """The only per-control-step observer. Isaac Lab calls this before score and distance."""
    global RUNTIME
    try:
        sig = fingerprint(objects)
        is_nominal = nominal_layout(objects)
        jumped = bool(RUNTIME.previous) and any(np.linalg.norm(objects[n].pos-RUNTIME.previous[n]) > T["reset_jump"] for n in SIZES)
        if jumped and is_nominal:
            RUNTIME = Runtime()  # A slow return never resets irreversible history.
        r = RUNTIME
        if not r.initialized:
            r.initialized = True
            if not is_nominal:
                r.latches.add("invalid_start")
        elif jumped:
            r.latches.add("unexplained_jump")
        r.step += 1
        r.latches.update(violations(objects))
        r.previous = {n: objects[n].pos.copy() for n in SIZES}
        r.history.append({n: (objects[n].pos.copy(), objects[n].quat_xyzw.copy()) for n in SIZES})
        r.signature = sig
        r.still = stable(objects, r.history)
        r.video_mode = os.environ.get("TASK_RELEASE_MODE", "automatic") == "video"
        hands, reason = read_grippers() if not r.video_mode else (None, "video_review_required")
        r.read_reason = reason
        r.clear = hands_clear(objects, hands)
        r.gaps = {} if hands is None else {n: h["gap_m"] for n, h in hands.items()}
        m = geometry_measurements(objects)
        good = (eligible(objects) and r.clear and not r.video_mode and geometry_tier(m) == .75
                and m["center_m"] <= T["center"] and m["yaw_deg"] <= T["yaw_deg"]
                and m["tilt_deg"] <= T["tilt_deg"])
        if os.environ.get("TASK_DEBUG") and debug_due(r.step):
            print_progress(objects,hands,reason,m,good)
        return bool(good)
    except (KeyError, ValueError, TypeError, AttributeError) as exc:
        r = RUNTIME
        r.step += 1
        r.initialized = True
        r.latches.add("invalid_state")
        r.signature = None
        if os.environ.get("TASK_DEBUG") and debug_due(r.step):
            print("task_progress "+json.dumps(dict(step=r.step, partial=0., reason=type(exc).__name__, latches=sorted(r.latches))), flush=True)
        return False


def demo():
    """Nominal open-loop downward pick/place; physics and gripper API need GPU validation."""
    px,py = D["lid_xy_m"];qx,qy = D["base_xy_m"]
    g = D["grip_above_lid_bottom_m"];hover = TABLE_Z+D["travel_above_table_m"]
    left=(.12,.30,hover);right=(.12,-.30,hover)
    pick=(px,py,TABLE_Z+g)
    place=(qx,qy,TABLE_Z+D["seated_lid_bottom_m"]+g+D["release_gap_m"])
    first=(qx,qy,TABLE_Z+SIZES['base'][2]+.010+g)
    w=[Waypoint(2.,right,FINGER_OPEN,"park"),
       Waypoint(1.,(px,py,hover),FINGER_OPEN,"approach"),Waypoint(1.,pick,FINGER_OPEN,"descend"),
       Waypoint(1.5,pick,FINGER_OPEN,"pick align"),Waypoint(2.,pick,GRIP_TARGET,"close"),
       Waypoint(1.5,pick,GRIP_TARGET,"grip hold"),Waypoint(2.,(px,py,hover),GRIP_TARGET,"lift"),
       Waypoint(2.5,(qx,qy,hover),GRIP_TARGET,"carry"),Waypoint(1.5,first,GRIP_TARGET,"lower first"),
       Waypoint(1.5,first,GRIP_TARGET,"place align"),Waypoint(3.,place,GRIP_TARGET,"lower second"),
       Waypoint(1.5,place,GRIP_TARGET,"settle lid"),Waypoint(2.,place,FINGER_OPEN,"release"),
       Waypoint(1.5,place,FINGER_OPEN,"release hold"),Waypoint(2.,(qx,qy,hover),FINGER_OPEN,"retreat"),
       Waypoint(2.,right,FINGER_OPEN,"park clear"),Waypoint(2.,right,FINGER_OPEN,"final hold")]
    return {"left":[Waypoint(x.duration,left,FINGER_OPEN,"park") for x in w],"right":w}

@lru_cache(maxsize=1)
def collision_geometry():
    spec=importlib.util.spec_from_file_location('gift_collision_geometry',HERE/'collision_geometry.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m


@lru_cache(maxsize=1)
def demo_timeline():
    rows=[];step=0
    for p in demo()['right']:
        end=step+round(p.duration*CONTROL_HZ)
        rows.append(dict(start_step=step,end_step=end,label=p.label,waypoint_target_tcp_m=list(p.pos)))
        step=end
    return tuple(rows)


def debug_due(step):
    critical=('descend','pick align','close','grip hold','lift','carry','lower first','place align',
              'lower second','settle lid','release','release hold')
    return step==1 or step%20==0 or any(r['label'] in critical and r['start_step']-20<=step<=r['end_step']+20
                                      for r in demo_timeline())


def seating_alignment(objects):
    lid,base=objects['lid'],objects['base']
    # Project all actual base corners into the lid frame, preserving tilt/yaw.
    local=(vertices('base',base)-lid.pos)@rotation(lid.quat_xyzw)
    inner=np.asarray(INFO['lid']['inner_xy_m'])/2
    gaps=np.array([inner[0]+local[:,0].min(),inner[0]-local[:,0].max(),
                   inner[1]+local[:,1].min(),inner[1]-local[:,1].max()])
    delta=(lid.pos-base.pos)@rotation(base.quat_xyzw)
    yaw=yaw_gap_deg(lid.quat_xyzw,base.quat_xyzw)
    relative_tilt=float(np.degrees(np.arccos(np.clip((rotation(base.quat_xyzw).T@rotation(lid.quat_xyzw))[2,2],-1,1))))
    return dict(xy_error_mm=(delta[:2]*1000).tolist(),side_order=['-x','+x','-y','+y'],
                side_clearance_mm=(gaps*1000).tolist(),yaw_deg=yaw,relative_tilt_deg=relative_tilt,
                accepted=bool(np.all(np.abs(delta[:2])<=.0005) and yaw<=.5 and relative_tilt<=.5 and gaps.min()>=.001))


def debug_contact(hands,objects):
    g=collision_geometry();out={}
    for side,hand in hands.items():
        rows=[];tips=[]
        for link,mesh,v in g.collision_hulls():
            if link not in hand['poses']:continue
            pose=hand['poses'][link];w=v@rotation(pose[3:]).T+pose[:3]
            if link.startswith('tip_'):tips.append(w)
            rows.append((link,mesh,w))
        fixtures=[]
        for name in ('lid','base'):
            obj=objects[name];r=rotation(obj.quat_xyzw)
            for panel in INFO[name]['panels']:
                local=g.CORNERS*np.asarray(panel['size_m'])/2+panel['center_m']-SIZES[name]/2*np.array([0,0,1])
                fixtures.append((name+'/'+panel['name'],local@r.T+obj.pos))
        # Report the nearest pair and all genuinely intersecting convex hulls.
        nearest=g.nearest_fixture(rows,fixtures);hits=[]
        for link,mesh,w in rows:
            for name,box in fixtures:
                delta=np.maximum(np.maximum(box.min(0)-w.max(0),w.min(0)-box.max(0)),0)
                if np.linalg.norm(delta)>1e-8:continue
                distance=g.convex_distance(w,box)
                if distance['converged'] and distance['upper_m']<=1e-8:
                    hits.append(dict(link=link,mesh=mesh,panel=name,witness_m=distance['witness_a_m']))
        out[side]=dict(nearest=nearest,intersection_pairs=hits,
                       fingertip_above_table_mm=1000*(min(v[:,2].min() for v in tips)-TABLE_Z))
    return out


def print_progress(objects,hands,reason,m,good):
    # Diagnostic exceptions never escape into success()'s invalid-state handler.
    r=RUNTIME;row=dict(step=r.step,partial=score(objects),full_candidate=bool(good),
        latches=sorted(r.latches),still=r.still,hands_clear=r.clear,gripper_reason=reason,
        gripper_gaps_m=r.gaps,release_mode='video' if r.video_mode else 'automatic',**m)
    try:
        phases=[p for p in demo_timeline() if p['start_step']<=r.step<p['end_step']]
        row['phase']=phases
        row['objects']={n:dict(pos_m=o.pos.tolist(),quat_xyzw=o.quat_xyzw.tolist(),
            lin_vel_m_s=o.lin_vel.tolist(),ang_vel_rad_s=o.ang_vel.tolist()) for n,o in objects.items()}
        row['base_displacement_mm']=((objects['base'].pos-NOMINAL['base'])*1000).tolist()
        row['base_tilt_deg']=tilt_deg(objects['base'].quat_xyzw)
        row['alignment']=seating_alignment(objects)
        row['second_descent_entry']=any(p['label']=='lower second' and p['start_step']==r.step for p in phases)
        if hands is not None:
            row['link_poses_m_xyzw']={s:{n:p.tolist() for n,p in h['poses'].items()} for s,h in hands.items()}
            row['tcp_m']={s:(h['poses']['gripper'][:3]+rotation(h['poses']['gripper'][3:])@np.asarray(TCP_OFFSET)).tolist() for s,h in hands.items()}
            grip=hands['right']['poses']['gripper'];rot=rotation(grip[3:]);tcp=np.asarray(row['tcp_m']['right'])
            relative=(objects['lid'].pos-tcp)@rot
            relative_rot=rot.T@rotation(objects['lid'].quat_xyzw)
            row['lid_relative_tcp_m']=relative.tolist()
            row['lid_relative_tcp_rotation']=relative_rot.tolist()
            if r.debug_grasp_reference is None and any(p['label']=='grip hold' for p in phases):
                r.debug_grasp_reference=(relative.copy(),relative_rot.copy())
            if r.debug_grasp_reference is not None:
                p0,r0=r.debug_grasp_reference
                row['grasp_translation_change_mm']=float(np.linalg.norm(relative-p0)*1000)
                row['grasp_rotation_change_deg']=float(np.degrees(np.arccos(np.clip((np.trace(r0.T@relative_rot)-1)/2,-1,1))))
            row['contact_geometry']=debug_contact(hands,objects)
        row['diagnostic_reason']='ok'
    except Exception as exc:
        row['diagnostic_reason']='debug_unavailable_'+type(exc).__name__
    print('task_progress '+json.dumps(row,sort_keys=True),flush=True)
