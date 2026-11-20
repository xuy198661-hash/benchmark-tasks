"""Recompute the committed scene's complete 20 Hz CPU audit; never simulates physics."""
import argparse
import importlib.util
import json
import math
import platform
from pathlib import Path
import numpy as np
from yam_arena import ik, station, YAM_URDF
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix

HERE = Path(__file__).resolve().parent

def audit():
    task = load(HERE/'task.py').module
    geometry = task.collision_geometry()
    hulls = geometry.collision_hulls()
    fixtures = [(o.name, geometry.CORNERS*np.asarray(o.size)/2+o.pos)
                for o in task.OBJECTS if not o.free]
    fixtures.append(('table', geometry.CORNERS*np.asarray(station.TABLE_SIZE)/2+station.TABLE_CENTER))
    for name, (kind, size, pos, quat) in station.camera_stand(station.top_camera()).items():
        half = np.asarray(size)/2 if kind == 'box' else np.array((size[0], size[0], size[1]/2+size[0]))
        vertices = (geometry.CORNERS*half) @ quat_xyzw_to_matrix(np.array(quat)).T+pos
        fixtures.append((name, geometry.CORNERS*(vertices.max(0)-vertices.min(0))/2+(vertices.max(0)+vertices.min(0))/2))
    flo, fhi = np.array([v.min(0) for _, v in fixtures]), np.array([v.max(0) for _, v in fixtures])
    objects=task.OBJECTS; overlaps=[]
    for i, a in enumerate(objects):
        for b in objects[i+1:]:
            alo, ahi=np.array(a.pos)-np.array(a.size)/2,np.array(a.pos)+np.array(a.size)/2
            blo, bhi=np.array(b.pos)-np.array(b.size)/2,np.array(b.pos)+np.array(b.size)/2
            if np.all(np.minimum(ahi,bhi)-np.maximum(alo,blo)>1e-9): overlaps.append([a.name,b.name])
    result={'environment':{'python':platform.python_version(),'numpy':np.__version__,'isaac_sim':False},
            'grasp_dimensions_mm':[{'name':o.name,'width':1000*o.size[1],'height':1000*o.size[2]} for o in objects if o.free],
            'tip_measurement':task.GEOMETRY['tip_measurement'],'grip_z_m':task.GRIP_Z,'overlaps':overlaps,
            'clearance_required_m':.003,'arms':{}}
    for side, points in task.demo().items():
        p=ArmPlan(points,station.BASES[side]); count=round(p.duration*20)+1
        result['duration_s']=p.duration
        minimum=math.inf; exact=None; bad=[]; limits=True; tipmin=math.inf; worst=None; seen=-1
        for k in range(count):
            action,label=p.action7(k/20); q=action[:6]
            target,_,_,_=p.target(k/20); actual,_=ik.tcp_pose(q,station.BASES[side])
            error=float(np.linalg.norm(target-actual))
            if error>seen: seen=error; worst={'step':k,'label':label,'position_error_mm':error*1000}
            limits &= bool(np.all(q>=ik.LOWER) and np.all(q<=ik.UPPER))
            finger=action[6]*FINGER_OPEN
            if label.split()[0].isdigit(): finger=min(finger,-task.WIDTHS[int(label.split()[0])-1]/2)
            fk=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,q))|{'joint7':finger,'joint8':finger})
            world=[]
            for link,mesh,v in hulls:
                if link=='base': continue
                m=fk[link]; w=v@m[:3,:3].T+m[:3,3]+station.BASES[side]
                world.append((link,mesh,w))
                if link.startswith('tip_') and any(word in label for word in ('descend','lower','hold','release','align','close')):
                    tipmin=min(tipmin,float(w[:,2].min()-task.ANVIL_TOP))
            low=np.array([v.min(0) for _,_,v in world]); high=np.array([v.max(0) for _,_,v in world])
            delta=np.maximum(np.maximum(flo[None]-high[:,None],low[:,None]-fhi[None]),0)
            bounds=np.linalg.norm(delta,axis=2); minimum=min(minimum,float(bounds.min()))
            threshold=exact['upper_m'] if exact else math.inf
            pairs=np.argwhere(bounds<threshold)
            pairs=sorted(pairs,key=lambda pair:bounds[tuple(pair)])
            for i,j in pairs:
                if exact and bounds[i,j]>=exact['upper_m']: break
                d=geometry.convex_distance(world[i][2],fixtures[j][1])
                if not d['converged']: raise RuntimeError(('GJK did not converge',side,k,world[i][:2],fixtures[j][0],d))
                if d['lower_m']<.003: bad.append({'step':k,'label':label,'link':world[i][0],'fixture':fixtures[j][0],**d})
                if exact is None or d['upper_m']<exact['upper_m']:
                    exact=dict(d,step=k,label=label,link=world[i][0],mesh=world[i][1],fixture=fixtures[j][0])
            if k%500==0: print(side,k,'/',count,'minimum mm',exact['upper_m']*1000,flush=True)
        result['arms'][side]={'frames':count,'worst_ik_mm':p.worst_ik_mm,'worst_ik_deg':p.worst_ik_deg,
            'joint_limits':limits,'collision_count':len(bad),'collisions':bad[:20],
            'worst_pose':worst,'min_aabb_clearance_m':minimum,'minimum_convex_clearance':exact,
            'min_tip_above_anvil_during_manipulation_m':tipmin}
        assert p.worst_ik_mm<.5 and p.worst_ik_deg<.1 and limits and not bad and tipmin>=.003
    assert not overlaps
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('output',type=Path); args=parser.parse_args()
    result=audit(); args.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
