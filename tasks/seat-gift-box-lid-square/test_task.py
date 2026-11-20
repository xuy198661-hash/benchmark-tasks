"""CPU contracts and adversarial scoring tests using real yam_arena types and kinematics.

Pose fixtures isolate scorer logic; they are not rigid-body simulation results.
Set TASK_GPU_TESTS=1 only inside a live Isaac Lab YAM scene for the optional reader check.
"""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import inspect
import itertools
import json
import math
import os
from pathlib import Path
import pickle
import re
import struct
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from pxr import Usd, UsdGeom, UsdPhysics
from yam_arena import YAM_URDF, ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import Asset, Box, ObjectState, load
from yam_arena.urdf import forward_kinematics, rpy_matrix, quat_xyzw_to_matrix

HERE = Path(__file__).resolve().parent
TASK_PATH = Path(os.environ.get('GIFT_BOX_TASK_FILE', HERE/'task.py'))


def module():
    spec=importlib.util.spec_from_file_location('gift_task_under_test', TASK_PATH)
    m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
    return m


def quat(yaw=0., tilt=0.):
    y,t=math.radians(yaw)/2,math.radians(tilt)/2
    return np.array([math.cos(y)*math.sin(t),math.sin(y)*math.sin(t),math.sin(y)*math.cos(t),math.cos(y)*math.cos(t)])


def state(m,name,pos,q=None,velocity=0.,spin=0.):
    p=np.array(pos,dtype=float);q=np.array([0,0,0,1] if q is None else q,dtype=float)
    v=(m.CORNERS*m.SIZES[name]/2)@m.rotation(q).T+p
    return ObjectState(pos=p,quat_xyzw=q,lin_vel=np.array([velocity,0,0]),ang_vel=np.array([0,0,spin]),low=v.min(0),high=v.max(0))


def start(m):
    return {n:state(m,n,p) for n,p in m.NOMINAL.items()}


def far_hands(m):
    return {side:dict(gap_m=2*abs(FINGER_OPEN),poses={n:np.array([.05,y,m.TABLE_Z+.22,0,0,0,1]) for n in ('gripper','tip_left','tip_right')}) for side,y in [('left',.40),('right',-.40)]}


@pytest.fixture
def task(monkeypatch):
    m=module();monkeypatch.setattr(m,'read_grippers',lambda:(far_hands(m),'ok'))
    monkeypatch.delenv('TASK_DEBUG',raising=False);monkeypatch.delenv('TASK_RELEASE_MODE',raising=False)
    return m


def move(m,objects,name,pos,q=None,steps=30):
    previous=objects[name];q=previous.quat_xyzw if q is None else q
    for k in range(1,steps+1):
        p=previous.pos+(np.array(pos)-previous.pos)*k/steps
        objects[name]=state(m,name,p,q)
        m.success(objects)
    return objects


def ready(m,level=.75,**kwargs):
    o=start(m);m.success(o)
    o['base']=state(m,'base',o['base'].pos-[0,0,m.D['spawn_gap_m']])
    o['gift']=state(m,'gift',o['gift'].pos-[0,0,2*m.D['spawn_gap_m']])
    m.success(o)
    lid=m.SIZES['lid'];base=m.SIZES['base'];x,y=m.D['base_xy_m']
    # Lift, translate, lower without any >100 mm one-step position changes.
    move(m,o,'lid',(*m.D['lid_xy_m'],m.TABLE_Z+.12))
    move(m,o,'lid',(x,y,m.TABLE_Z+.12))
    if level==.75:bottom=m.D['seated_lid_bottom_m'];shift=0
    elif level==.5:bottom=base[2]-.012;shift=0
    elif level==.25:bottom=base[2];shift=.008
    else:bottom=0.;shift=.08
    pos=np.array([x,y+shift,m.TABLE_Z+bottom+lid[2]/2])+np.array(kwargs.get('offset',(0,0,0)))
    move(m,o,'lid',pos,quat(kwargs.get('yaw',0),kwargs.get('tilt',0)))
    for _ in range(12):m.success(o)
    return o


def test_real_task_loader_and_constructor_signatures():
    f=load(TASK_PATH);m=module()
    assert f.scene=='station' and f.score_over=='end' and f.success_hold_s==1.
    assert f.docs and set(f.names)=={'base','lid','gift'}
    for obj in m.OBJECTS:
        inspect.signature(type(obj)).bind(**vars(obj))
        assert isinstance(obj,(Asset,Box))
    for seq in m.demo().values():
        for w in seq:inspect.signature(Waypoint).bind(**vars(w))
    assert len(m.OBJECTS)==3
    assert not any(n.startswith(('isaacsim','omni.isaac')) for n in sys.modules)


def test_object_numeric_parameters_are_native_floats():
    # NumPy float64 values silently make reset poses torch.float64, which Warp
    # rejects. isinstance(v, float) also accepts np.float64, so check exact types.
    vectors = ("pos", "size", "xy", "color")
    scalars = ("radius", "height", "extent", "mass", "yaw")
    for obj in module().OBJECTS:
        for field in vectors + scalars:
            if not hasattr(obj, field):
                continue
            value = getattr(obj, field)
            if value is None:
                continue
            values = value if field in vectors else (value,)
            for index, v in enumerate(values):
                assert type(v) is float, (
                    f"{obj.name}.{field}[{index}] must be a native float, "
                    f"got {type(v).__module__}.{type(v).__name__}"
                )


@pytest.mark.parametrize('tier',[0.,.25,.5,.75])
def test_every_tier(task,tier):
    o=ready(task,tier);assert task.score(o)==tier


def test_highest_tier_not_sum(task):
    o=ready(task);assert task.geometry_tier(task.geometry_measurements(o))==.75
    assert task.score(o)==.75 and task.score(o)<1.


def test_full_credit_requires_framework_hold(task):
    o=ready(task);held=0
    for _ in range(19):
        held=held+1 if task.success(o) else 0
        assert held<round(task.SUCCESS_HOLD_S*20)
    held=held+1 if task.success(o) else 0
    assert held==20 and task.score(o)==.75


def test_idle_and_wrong_start_fail_closed(task):
    o=start(task)
    for _ in range(30):assert not task.success(o)
    assert task.score(o)==0.
    m=module();m.read_grippers=lambda:(far_hands(m),'ok')
    o=start(m);o['lid']=state(m,'lid',o['lid'].pos+[.006,0,0]);m.success(o)
    move(m,o,'lid',m.NOMINAL['lid']);assert 'invalid_start' in m.RUNTIME.latches
    assert m.score(o)==0.


def test_jump_directly_to_terminal_is_rejected(task):
    o=start(task);task.success(o)
    o['lid']=state(task,'lid',(*task.D['base_xy_m'],task.TABLE_Z+task.D['seated_lid_bottom_m']+task.SIZES['lid'][2]/2))
    assert not task.success(o);assert task.score(o)==0.
    assert 'unexplained_jump' in task.RUNTIME.latches


EVENTS=['base_moved','base_toppled','lid_toppled','base_fell','lid_fell','object_launched','gift_left_box','unexplained_jump']


def event_state(m,o,event):
    name='base' if event.startswith('base_') else 'lid'
    p=o[name].pos.copy();q=None;v=0.
    if event=='base_moved':p[0]+=.021
    elif event=='base_toppled':q=quat(tilt=16)
    elif event=='lid_toppled':q=quat(tilt=31)
    elif event in ('base_fell','lid_fell'):p[2]=m.TABLE_Z-m.T['below_table']-.001+m.SIZES[name][2]/2
    elif event=='object_launched':v=.751
    elif event=='gift_left_box':name='gift';p=o[name].pos.copy();p[0]+=.020
    elif event=='unexplained_jump':p[0]+=.101
    o[name]=state(m,name,p,q,velocity=v)
    return name


@pytest.mark.parametrize('event',EVENTS)
def test_irreversible_events_survive_slow_restoration(task,event):
    o=ready(task);assert task.score(o)==.75
    saved=deepcopy(o);name=event_state(task,o,event)
    assert not task.success(o);assert event in task.RUNTIME.latches;assert task.score(o)==0.
    move(task,o,name,saved[name].pos,saved[name].quat_xyzw,steps=60)
    for _ in range(30):task.success(o)
    assert event in task.RUNTIME.latches and task.score(o)==0.


def test_invalid_state_is_latched(task):
    o=start(task);o['lid']=state(task,'lid',o['lid'].pos)
    o['lid'].pos[0]=np.nan
    assert not task.success(o);assert task.score(o)==0.
    o=start(task);task.success(o);assert 'invalid_state' in task.RUNTIME.latches


def test_final_frame_is_rechecked_without_mutation(task):
    o=ready(task);before=pickle.dumps(task.RUNTIME.__dict__)
    o['gift']=state(task,'gift',o['gift'].pos+[.025,0,0])
    assert task.score(o)==0.
    assert pickle.dumps(task.RUNTIME.__dict__)==before


def test_slow_nominal_return_does_not_reset(task):
    o=ready(task);event_state(task,o,'base_moved');task.success(o)
    for n in task.SIZES:move(task,o,n,task.NOMINAL[n],steps=60)
    assert task.nominal_layout(o) and task.RUNTIME.latches and task.score(o)==0.


def test_episode_reset_requires_jump_and_nominal_layout(task):
    o=ready(task);event_state(task,o,'base_moved');task.success(o)
    nominal=start(task)
    assert np.linalg.norm(o['lid'].pos-nominal['lid'].pos)>.10
    task.success(nominal)
    assert task.RUNTIME.step==1 and not task.RUNTIME.latches and task.score(nominal)==0.


def test_jump_to_non_nominal_does_not_reset(task):
    o=ready(task);event_state(task,o,'base_moved');task.success(o)
    o['lid']=state(task,'lid',o['lid'].pos+[.11,0,0]);task.success(o)
    assert {'base_moved','unexplained_jump'}<=task.RUNTIME.latches


@pytest.mark.parametrize('yaw',[0,180,-180])
def test_half_turn_is_not_tilt(task,yaw):
    o=ready(task,yaw=yaw)
    assert task.tilt_deg(o['lid'].quat_xyzw)<1e-5
    assert task.success(o) and task.score(o)==.75


@pytest.mark.parametrize('field,value,inside',[
    ('offset',(.0004999,0,0),True),('offset',(.0005001,0,0),False),
    ('offset',(0,0,.0014999),True),('offset',(0,0,.0015001),False),
    ('offset',(0,0,-.0014999),True),('offset',(0,0,-.0015001),False),
    ('yaw',1.4999,True),('yaw',1.5001,False),
    ('tilt',.9999,True),('tilt',1.0001,False),
])
def test_success_threshold_both_sides(task,field,value,inside):
    o=ready(task,**{field:value});assert task.success(o) is inside


@pytest.mark.parametrize('field,delta,accepted',[
    ('center_m',.0009999,True),('center_m',.0010001,False),
    ('yaw_deg',2.9999,True),('yaw_deg',3.0001,False),
    ('tilt_deg',1.9999,True),('tilt_deg',2.0001,False),
])
def test_three_quarter_tier_boundaries(task,field,delta,accepted):
    o=ready(task);m=task.geometry_measurements(o);m[field]=delta
    assert (task.geometry_tier(m)==.75) is accepted


@pytest.mark.parametrize('name,limit,axis', [('base_drift',.020,0),('below_table',.010,2),('flight_speed',.75,0)])
@pytest.mark.parametrize('direction',[-1,1])
def test_event_threshold_both_sides(task,name,limit,axis,direction):
    o=start(task);delta=limit+direction*1e-7
    if name=='base_drift':o['base']=state(task,'base',o['base'].pos+[delta,0,0]);event='base_moved'
    elif name=='below_table':o['lid']=state(task,'lid',(o['lid'].pos[0],o['lid'].pos[1],task.TABLE_Z-delta+task.SIZES['lid'][2]/2));event='lid_fell'
    else:o['lid']=state(task,'lid',o['lid'].pos,velocity=delta);event='object_launched'
    assert (event in task.violations(o)) is (direction>0)


@pytest.mark.parametrize('name,angle,event',[('base',15,'base_toppled'),('lid',30,'lid_toppled')])
@pytest.mark.parametrize('direction',[-1,1])
def test_topple_boundary(task,name,angle,event,direction):
    o=start(task);o[name]=state(task,name,o[name].pos,quat(tilt=angle+direction*.0001))
    assert (event in task.violations(o)) is (direction>0)


def test_rest_window_uses_every_sample_and_frozen_velocity(task):
    o=ready(task);history=deepcopy(task.RUNTIME.history)
    for p in history:p['lid']=(o['lid'].pos+np.array([.0020001,0,0]),o['lid'].quat_xyzw)
    assert not task.stable(o,history)
    for p in history:p['lid']=(o['lid'].pos+np.array([.0019999,0,0]),o['lid'].quat_xyzw)
    assert task.stable(o,history)
    o['lid']=state(task,'lid',o['lid'].pos,velocity=.02,spin=.2)
    assert not task.stable(o,[])
    for _ in range(12):task.success(o)
    assert task.score(o)==.75


def test_held_lid_cannot_receive_credit(task,monkeypatch):
    o=ready(task);hands=far_hands(task)
    hands['right']['poses']['gripper'][:3]=o['lid'].pos
    hands['right']['gap_m']=task.SIZES['lid'][1]
    monkeypatch.setattr(task,'read_grippers',lambda:(hands,'ok'))
    for _ in range(30):assert not task.success(o)
    assert task.score(o)==0.


def test_unavailable_reader_is_fail_closed(task,monkeypatch,capsys):
    o=ready(task);monkeypatch.setattr(task,'read_grippers',lambda:(None,'simulation_context_unavailable'))
    monkeypatch.setenv('TASK_DEBUG','1')
    for _ in range(20):assert not task.success(o)
    assert task.score(o)==0. and 'simulation_context_unavailable' in capsys.readouterr().out


def test_video_mode_requires_manual_release_review(task,monkeypatch):
    o=ready(task);monkeypatch.setenv('TASK_RELEASE_MODE','video')
    for _ in range(30):assert not task.success(o)
    assert task.score(o)==.75 and task.RUNTIME.read_reason=='video_review_required'


def test_score_and_distance_are_read_only(task,monkeypatch):
    o=ready(task)
    monkeypatch.setattr(task,'read_grippers',lambda:pytest.fail('read-only functions must not read live hardware'))
    before=pickle.dumps(task.RUNTIME.__dict__);objects=pickle.dumps(o)
    for _ in range(25):assert task.score(o)==.75;assert math.isfinite(task.distance(o))
    assert pickle.dumps(task.RUNTIME.__dict__)==before and pickle.dumps(o)==objects


def test_debug_prints_only_at_required_steps(task,monkeypatch,capsys):
    monkeypatch.setenv('TASK_DEBUG','1');o=start(task)
    for _ in range(45):task.success(o)
    rows=[json.loads(line.split(' ',1)[1]) for line in capsys.readouterr().out.splitlines() if line.startswith('task_progress ')]
    assert [r['step'] for r in rows]==[1,20,40]
    assert all(r['partial']==0 for r in rows)


def test_public_reader_uses_only_read_apis(monkeypatch):
    m=module();calls=[]
    class View:
        count=1
        shared_metatype=types.SimpleNamespace(dof_names=['joint8','unused','joint7'],link_names=['tip_right','gripper','tip_left'])
        def get_dof_positions(self):calls.append('get_dof_positions');return np.array([[FINGER_OPEN,0,FINGER_OPEN]])
        def get_link_transforms(self):calls.append('get_link_transforms');return np.array([[[.05,-.4,1,0,0,0,1]]*3])
    class Physics:
        def create_articulation_view(self,path):calls.append(path);return View()
    sim=types.SimpleNamespace(physics_sim_view=Physics())
    fake=types.ModuleType('isaaclab.sim');fake.SimulationContext=types.SimpleNamespace(instance=lambda:sim)
    monkeypatch.setitem(sys.modules,'isaaclab.sim',fake)
    h,reason=m.read_grippers();assert reason=='ok' and set(h)=={'left','right'}
    assert h['left']['gap_m']==pytest.approx(.0939)
    assert calls.count('get_dof_positions')==2 and calls.count('get_link_transforms')==2
    assert set(c for c in calls if c.startswith('/'))=={'/World/envs/env_0/LeftArm','/World/envs/env_0/RightArm'}
    sim.physics_sim_view=types.SimpleNamespace(create_articulation_view=lambda path:types.SimpleNamespace(count=0))
    assert m.read_grippers()[0] is None
    sim.physics_sim_view=Physics()
    monkeypatch.setattr(View,'get_link_transforms',lambda self:np.zeros((1,3,7)))
    assert m.read_grippers()[0] is None
    m.success(start(m))
    assert not m.RUNTIME.latches and m.score(start(m))==0.


def test_no_isaac_import_at_module_scope_or_embodiment_hooks():
    tree=ast.parse(TASK_PATH.read_text())
    def check(node,in_function=False):
        nested=in_function or isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef))
        if isinstance(node,ast.ImportFrom) and (node.module or '').startswith(('isaaclab.sim','isaacsim','omni.')):assert in_function
        if isinstance(node,ast.Attribute):assert not (isinstance(node.value,ast.Name) and node.value.id in ('sim','view','meta') and node.attr.startswith('_'))
        for c in ast.iter_child_nodes(node):check(c,nested)
    check(tree)
    assert 'YamArenaEmbodiment' not in TASK_PATH.read_text()


def stl_triangles(path):
    b=path.read_bytes();n=struct.unpack('<I',b[80:84])[0]
    return np.frombuffer(b,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)['v'].astype(float)/1000


@pytest.mark.parametrize('name',['base','lid'])
def test_actual_mesh_dimensions_com_and_usda_colliders(task,name):
    info=task.INFO[name];tri=stl_triangles(HERE/'assets'/(name+'.stl'));v=tri.reshape(-1,3)
    assert np.ptp(v,axis=0)==pytest.approx(info['size_m'],abs=1e-9)
    tet=np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2]))/6;volume=tet.sum()
    com=(tri.sum(1)/4*tet[:,None]).sum(0)/volume
    assert com==pytest.approx(info['center_of_mass_m'],abs=1e-9)
    assert volume*1240==pytest.approx(info['mass_kg'])
    edges={}
    for f in tri:
        for i in range(3):
            e=tuple(sorted((tuple(f[i]),tuple(f[(i+1)%3]))));edges[e]=edges.get(e,0)+1
    assert set(edges.values())=={2}
    stage=Usd.Stage.Open(str(HERE/'assets'/(name+'.usda')));root=stage.GetDefaultPrim()
    assert root.HasAPI(UsdPhysics.RigidBodyAPI)
    assert np.array(UsdPhysics.MassAPI(root).GetCenterOfMassAttr().Get())==pytest.approx(com,abs=1e-8)
    colliders=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(colliders)==5 and all(p.IsA(UsdGeom.Cube) for p in colliders)
    assert not any(p.IsA(UsdGeom.Mesh) for p in stage.Traverse())
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default'])
    box=cache.ComputeWorldBound(root).ComputeAlignedBox()
    assert np.array(box.GetSize())==pytest.approx(info['size_m'],abs=1e-8)
    assert hashlib.sha256((HERE/'assets'/(name+'.stl')).read_bytes()).hexdigest()==info['stl_sha256']


def test_grasp_and_clearance_from_models(task):
    lid=task.INFO['lid'];base=task.INFO['base']
    assert lid['size_m'][1]<.085 and lid['skirt_height_m']>=.015
    assert task.D['grip_above_lid_bottom_m']>=.015
    assert (np.array(lid['inner_xy_m'])-np.array(base['size_m'][:2]))/2==pytest.approx([.002,.002])
    # With +/-0.2 mm linear size error on both mating pieces, nominal side gap remains >=1.8 mm.
    assert min(task.D['clearance_per_side_m'])-.0002>0


def test_spawn_has_no_interpenetration(task):
    bounds=[]
    for name in ('base','lid'):
        for panel in task.INFO[name]['panels']:
            c=np.array(panel['center_m'])+np.r_[task.D[name+'_xy_m'],task.TABLE_Z+task.D['spawn_gap_m']]
            s=np.array(panel['size_m']);bounds.append((name,c-s/2,c+s/2))
    o=start(task)['gift'];bounds.append(('gift',o.low,o.high))
    for a,b in itertools.combinations(bounds,2):
        if a[0]==b[0]:continue
        assert not np.all(np.minimum(a[2],b[2])-np.maximum(a[1],b[1])>1e-9)
    assert start(task)['gift'].low[2]-(task.TABLE_Z+task.D['spawn_gap_m']+task.INFO['base']['panel_high_m'])==pytest.approx(.001)


def test_asset_metadata_has_no_private_paths_or_embedded_image_metadata():
    forbidden=[b'/Users/',b'/home/',b'C:\\',b'Exif',b'caBX',b'xmpmeta']
    for p in (HERE/'assets').iterdir():
        if p.suffix.lower() in ('.jpg','.jpeg'):
            # Only marker payloads, never entropy-coded pixel bytes.
            data=p.read_bytes();i=2;payloads=[]
            while i<len(data):
                assert data[i]==255
                marker=data[i+1];i+=2
                if marker in (0xDA,0xD9):break
                size=int.from_bytes(data[i:i+2],'big');payload=data[i+2:i+size];i+=size
                if 0xE1<=marker<=0xEF or marker==0xFE:payloads.append(payload)
            data=b''.join(payloads)
        elif p.suffix.lower() in ('.png','.webp'):
            pytest.fail('Add chunk-aware metadata scanning before introducing a new image format')
        else:data=p.read_bytes()
        assert not any(s in data for s in forbidden),p.name
    assert not any(p.suffix.lower() in ('.jpg','.jpeg','.png','.webp') for p in (HERE/'assets').iterdir())


def collision_meshes():
    result=[]
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh');p=YAM_URDF.parent/mesh.get('filename');raw=p.read_bytes();count=struct.unpack('<I',raw[80:84])[0]
            tri=np.frombuffer(raw,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=count)['v'].astype(float)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            origin=collision.find('origin')
            if origin is not None:tri=tri@rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).T+np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            edges=np.concatenate([tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]])
            normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
            result.append((link.get('name'),p.name,np.unique(tri.reshape(-1,3),axis=0),edges,normals))
    return result


def sat_hit(v,edges,normals,low,high):
    axes=np.concatenate([np.eye(3),normals,*[np.cross(edges,a) for a in np.eye(3)]])
    lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-9]/lengths[lengths>1e-9,None];axes=np.unique(np.round(axes,9),axis=0)
    projections=(v-(low+high)/2)@axes.T;r=np.abs(axes)@((high-low)/2)
    return bool(np.all(projections.min(0)<=r) and np.all(projections.max(0)>=-r))


def test_collision_checker_detects_actual_finger_fixture_overlap():
    geometries=collision_meshes();link,name,v,e,n=next(x for x in geometries if x[1]=='tip_right_008.stl')
    assert sat_hit(v,e,n,v.min(0)-.001,v.max(0)+.001)
    assert not sat_hit(v,e,n,v.min(0)+1,v.max(0)+1)


def test_demo_20hz_ik_limits_release_and_all_collision_mesh_clearances(task):
    fixed=[('table',np.array(station.TABLE_CENTER)-np.array(station.TABLE_SIZE)/2,np.array(station.TABLE_CENTER)+np.array(station.TABLE_SIZE)/2)]
    for name,(kind,size,pos,q) in station.camera_stand(station.top_camera()).items():
        half=np.array(size)/2 if kind=='box' else np.array([size[0],size[0],size[1]/2+size[0]])
        v=(task.CORNERS*half)@quat_xyzw_to_matrix(np.array(q)).T+pos;fixed.append((name,v.min(0),v.max(0)))
    for p in task.INFO['base']['panels']:
        c=np.array(p['center_m'])+np.r_[task.D['base_xy_m'],task.TABLE_Z];s=np.array(p['size_m']);fixed.append(('base/'+p['name'],c-s/2,c+s/2))
    geom=collision_meshes();routes=task.demo();duration=max(sum(w.duration for w in ws) for ws in routes.values())
    assert duration+3+task.SUCCESS_HOLD_S<task.RUN_LIMIT/20
    release=next(w for w in routes['right'] if w.label=='release')
    expected=task.TABLE_Z+task.D['seated_lid_bottom_m']+task.D['grip_above_lid_bottom_m']
    assert .003<=release.pos[2]-expected<=.005
    assert all(w.yaw==0 and isinstance(w,Waypoint) for ws in routes.values() for w in ws)
    for side,ws in routes.items():
        plan=ArmPlan(ws,station.BASES[side]);frames=math.ceil(duration*20)+1
        for k in range(frames):
            action,label=plan.action7(k/20);q=action[:6]
            assert np.all(q>=ik.LOWER) and np.all(q<=ik.UPPER)
            assert plan.worst_ik_mm<=.5 and plan.worst_ik_deg<=.1,(side,k,label,plan.worst_ik_mm)
            finger=plan.target(k/20)[2]
            if side=='right' and label in ('close','lift','carry','lower','settle lid','release'):finger=min(finger,-task.SIZES['lid'][1]/2)
            joints=dict(zip(ARM_JOINTS,q));joints.update(joint7=finger,joint8=finger);fk=forward_kinematics(YAM_URDF,joints)
            for link,mesh,v,e,n in geom:
                if link=='base':continue
                R=fk[link][:3,:3];world=v@R.T+fk[link][:3,3]+station.BASES[side];lo,hi=world.min(0),world.max(0)
                for fixture,fl,fh in fixed:
                    if np.linalg.norm(np.maximum(np.maximum(fl-hi,lo-fh),0))>=.001:continue
                    assert not sat_hit(world,e@R.T,n@R.T,fl-.001,fh+.001),(side,k,label,link,mesh,fixture)
        assert frames==471


@pytest.mark.skipif(os.environ.get('TASK_GPU_TESTS')!='1',reason='Requires an active Isaac Lab GPU YAM scene; CPU fixtures do not validate the native articulation view')
def test_live_gpu_gripper_reader():
    m=module();hands,reason=m.read_grippers();assert hands is not None,reason
    assert set(hands)=={'left','right'}
    assert all(0<=h['gap_m']<=.0979 for h in hands.values())


@pytest.mark.parametrize('quantity',['start_xy','start_z','start_angle_deg'])
@pytest.mark.parametrize('direction',[-1,1])
def test_nominal_layout_boundary(task,quantity,direction):
    o=start(task);p=o['lid'].pos.copy();q=None
    if quantity=='start_xy':p[0]+=task.T[quantity]+direction*1e-7
    elif quantity=='start_z':p[2]+=task.T[quantity]+direction*1e-7
    else:q=quat(yaw=task.T[quantity]+direction*.0001)
    o['lid']=state(task,'lid',p,q)
    assert task.nominal_layout(o) is (direction<0)


@pytest.mark.parametrize('direction',[-1,1])
def test_reset_jump_boundary(task,direction):
    o=start(task);task.success(o);task.RUNTIME.latches.add('base_moved')
    task.RUNTIME.previous['lid']=o['lid'].pos+np.array([task.T['reset_jump']+direction*1e-7,0,0])
    task.success(o)
    assert (not task.RUNTIME.latches) is (direction>0)


@pytest.mark.parametrize('quantity',['base_height','base_tilt_deg'])
@pytest.mark.parametrize('direction',[-1,1])
def test_base_final_pose_boundary(task,quantity,direction):
    o=ready(task);p=o['base'].pos.copy();q=None
    if quantity=='base_height':p[2]+=task.T[quantity]+direction*1e-7
    else:q=quat(tilt=task.T[quantity]+direction*.0001)
    o['base']=state(task,'base',p,q)
    assert task.geometry_measurements(o)['base_ok'] is (direction<0)


@pytest.mark.parametrize('quantity',['engagement','engagement_center','engagement_tilt_deg','rim_height','rim_center','rim_tilt_deg'])
@pytest.mark.parametrize('direction',[-1,1])
def test_partial_tier_remaining_boundaries(task,quantity,direction):
    o=ready(task,.5 if quantity.startswith('engagement') else .25);m=task.geometry_measurements(o)
    if quantity=='engagement':m['bottom_relative_to_rim_m']=-(task.T[quantity]+direction*1e-7);expected=direction>0
    elif quantity.endswith('center'):m['center_m']=task.T[quantity]+direction*1e-7;expected=direction<0
    elif quantity.endswith('tilt_deg'):m['tilt_deg']=task.T[quantity]+direction*.0001;expected=direction<0
    else:m['bottom_relative_to_rim_m']=task.T[quantity]+direction*1e-7;expected=direction<0
    tier=.5 if quantity.startswith('engagement') else .25
    assert (task.geometry_tier(m)==tier) is expected


@pytest.mark.parametrize('direction',[-1,1])
def test_deep_lid_does_not_gain_engagement(task,direction):
    o=ready(task,.5);m=task.geometry_measurements(o)
    m['bottom_relative_to_rim_m']=-task.INFO['lid']['skirt_height_m']-task.T['seat_height']+direction*1e-7
    assert (task.geometry_tier(m)==.5) is (direction>0)


@pytest.mark.parametrize('quantity',['rest_speed','rest_spin','rest_angle_deg'])
@pytest.mark.parametrize('direction',[-1,1])
def test_rest_remaining_boundaries(task,quantity,direction):
    o=ready(task)
    if quantity=='rest_angle_deg':
        history=deepcopy(task.RUNTIME.history)
        for p in history:p['lid']=(o['lid'].pos,quat(yaw=task.T[quantity]+direction*.0001))
        result=task.stable(o,history)
    else:
        args={'velocity':task.T[quantity]+direction*1e-7} if quantity=='rest_speed' else {'spin':task.T[quantity]+direction*1e-7}
        o['lid']=state(task,'lid',o['lid'].pos,**args);result=task.stable(o,[])
    assert result is (direction<0)


@pytest.mark.parametrize('direction',[-1,1])
def test_gift_corner_containment_boundary(task,direction):
    o=ready(task)
    dx=task.INFO['base']['inner_xy_m'][0]/2+task.T['gift_margin']-task.SIZES['gift'][0]/2+direction*1e-7
    o['gift']=state(task,'gift',o['gift'].pos+[dx,0,0])
    assert ('gift_left_box' in task.violations(o)) is (direction>0)


@pytest.mark.parametrize('direction',[-1,1])
def test_full_gripper_clearance_boundary(task,direction):
    o=ready(task);h=far_hands(task);lo=np.array(task.INFO['gripper']['link_bounds_m']['gripper']['low'])
    pose=h['right']['poses']['gripper'];pose[:3]=o['lid'].pos
    pose[0]+=task.SIZES['lid'][0]/2-lo[0]+task.T['hands_clear']+direction*1e-7
    assert task.hands_clear(o,h) is (direction>0)


def test_debug_is_observational_even_for_failed_base_pose(task,monkeypatch,capsys):
    o=ready(task);o['base']=state(task,'base',o['base'].pos+[0,0,.0031])
    task.RUNTIME.step=19
    task.success(o);without=deepcopy(task.RUNTIME.__dict__)
    m=module();m.read_grippers=lambda:(far_hands(m),'ok');m.RUNTIME.__dict__=deepcopy(without)
    m.RUNTIME.step=19
    # Both observations start with the same previous sample and history.
    task.RUNTIME.__dict__=deepcopy(without);task.RUNTIME.step=19
    task.success(o)
    monkeypatch.setenv('TASK_DEBUG','1');m.success(o)
    assert pickle.dumps(m.RUNTIME.__dict__)==pickle.dumps(task.RUNTIME.__dict__)
    assert 'invalid_state' not in m.RUNTIME.latches
    assert 'task_progress ' in capsys.readouterr().out


def test_demo_has_ten_mm_upward_reach_reserve(task):
    for side,seq in task.demo().items():
        raised=[Waypoint(w.duration,(w.pos[0],w.pos[1],w.pos[2]+.010),w.finger,w.label,w.yaw) for w in seq]
        plan=ArmPlan(raised,station.BASES[side])
        for k in range(math.ceil(plan.duration*20)+1):
            plan.action7(k/20)
            assert plan.worst_ik_mm<=.5 and plan.worst_ik_deg<=.1,(side,k,plan.worst_ik_mm)
