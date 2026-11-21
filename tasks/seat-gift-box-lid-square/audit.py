"""CPU gate for the original printable gift-box candidate; real YAM ArmPlan and collision meshes."""
from pathlib import Path
import argparse, hashlib, inspect, itertools, json, math, platform, struct
import xml.etree.ElementTree as ET
import numpy as np
from yam_arena import YAM_URDF, ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import Asset, Box, TABLE_TOP, load
from yam_arena.urdf import forward_kinematics, rpy_matrix, quat_xyzw_to_matrix
Z=TABLE_TOP['station']


def inputs(folder):
    a=folder/'assets'
    return {n:json.loads((a/(n+'_info.json')).read_text()) for n in ('base','lid','layout')}


def plans(data):
    return load(Path(__file__).resolve().parent/'task.py').module.demo()


def meshes():
    out=[]
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        for c in link.findall('collision'):
            m=c.find('geometry/mesh');p=YAM_URDF.parent/m.get('filename');raw=p.read_bytes();n=struct.unpack('<I',raw[80:84])[0]
            tri=np.frombuffer(raw,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)['v'].astype(float)*np.fromstring(m.get('scale','1 1 1'),sep=' ')
            o=c.find('origin')
            if o is not None:tri=tri@rpy_matrix(np.fromstring(o.get('rpy','0 0 0'),sep=' ')).T+np.fromstring(o.get('xyz','0 0 0'),sep=' ')
            vertices=np.unique(tri.reshape(-1,3),axis=0)
            edges=np.concatenate([tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]])
            normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
            out.append((link.get('name'),p.name,vertices,edges,normals))
    return out


def intersects(vertices, edges, normals, low, high):
    axes=np.concatenate([np.eye(3),normals,*[np.cross(edges,a) for a in np.eye(3)]])
    lengths=np.linalg.norm(axes,axis=1);axes=axes[lengths>1e-9]/lengths[lengths>1e-9,None];axes=np.unique(np.round(axes,9),axis=0)
    center=(high+low)/2;radius=np.abs(axes)@((high-low)/2);p=(vertices-center)@axes.T
    return bool(np.all(p.min(0)<=radius) and np.all(p.max(0)>=-radius))


def fixtures():
    parts=[('table',np.array(station.TABLE_CENTER)-np.array(station.TABLE_SIZE)/2,np.array(station.TABLE_CENTER)+np.array(station.TABLE_SIZE)/2)]
    corners=np.array(list(itertools.product((-1,1),repeat=3)))
    for name,(kind,size,pos,quat) in station.camera_stand(station.top_camera()).items():
        R=quat_xyzw_to_matrix(np.array(quat))
        half=np.array(size)/2 if kind=='box' else np.array([size[0],size[0],size[1]/2+size[0]])
        v=(corners*half)@R.T+pos;parts.append((name,v.min(0),v.max(0)))
    return parts


def objects(data):
    return load(Path(__file__).resolve().parent/'task.py').module.OBJECTS


def gates(result):
    failures=[]
    if not(result['grasp']['width_mm']<85 and result['grasp']['height_mm']>=15):failures.append('grasp dimensions')
    if not result['allowed_primitives']:failures.append('motion primitives')
    if result['spawn_overlaps']:failures.append('spawn overlap')
    for side,a in result['arms'].items():
        if a['worst_ik_mm']>.5 or a['worst_ik_deg']>.1:failures.append(side+' IK')
        if not a['joint_limits']:failures.append(side+' joint limits')
        if a['fixed_collision_count']:failures.append(side+' fixed clearance')
        if a['base_collision_count']:failures.append(side+' base rim clearance')
    return failures


def audit(folder,output):
    data=inputs(folder);d=data['layout'];spec=objects(data);ws=plans(data)
    bounds=[]
    for name in ('base','lid'):
        for p in data[name]['panels']:
            c=np.array(p['center_m'])+np.r_[d[name+'_xy_m'],Z+d['spawn_gap_m']];s=np.array(p['size_m'])
            bounds.append((name+'/'+p['name'],c-s/2,c+s/2))
    gift=spec[-1];bounds.append(('gift',np.array(gift.pos)-np.array(gift.size)/2,np.array(gift.pos)+np.array(gift.size)/2))
    overlaps=[]
    for (an,al,ah),(bn,bl,bh) in itertools.combinations(bounds,2):
        if an.split('/')[0]!=bn.split('/')[0] and np.all(np.minimum(ah,bh)-np.maximum(al,bl)>1e-9):overlaps.append([an,bn])
    base=[(n,lo-np.array([0,0,d['spawn_gap_m']]),hi-np.array([0,0,d['spawn_gap_m']])) for n,lo,hi in bounds if n.startswith('base/')]
    fixed=fixtures();geom=meshes()
    result={'environment':{'python':platform.python_version(),'numpy':np.__version__,'platform':platform.platform(),'isaac_sim':False,'real_yam_arena':True},'grasp':{'object':'lid','width_mm':data['lid']['size_m'][1]*1000,'height_mm':data['lid']['skirt_height_m']*1000,'tcp_above_bottom_mm':d['grip_above_lid_bottom_m']*1000},'allowed_primitives':all(isinstance(w,Waypoint) and len(w.pos)==3 and w.yaw==0 for seq in ws.values() for w in seq),'constructor_bind_count':len(spec),'spawn_overlaps':overlaps,'duration_s':max(sum(w.duration for w in seq) for seq in ws.values()),'release_gap_mm':d['release_gap_m']*1000,'collision_margin_mm':1.,'arms':{}}
    for side,seq in ws.items():
        p=ArmPlan(seq,station.BASES[side]);fixed_hits=[];base_hits=[];limits=True;min_gap=float('inf');closest=None;worst=None;worst_error=-1;frames=math.ceil(p.duration*20)+1
        for k in range(frames):
            t=k/20;a,label=p.action7(t);q=a[:6];limits &= bool(np.all(q>=ik.LOWER) and np.all(q<=ik.UPPER));want,_,finger,_=p.target(t);actual,_=ik.tcp_pose(q,station.BASES[side]);error=float(np.linalg.norm(actual-want))
            if error>worst_error:worst_error=error;worst={'step':k,'label':label,'target_tcp_m':want.tolist(),'actual_tcp_m':actual.tolist(),'q_rad':q.tolist()}
            if side=='right' and label in ('close','grip hold','lift','carry','lower first','place align','lower second','settle lid','release'):finger=min(finger,-data['lid']['size_m'][1]/2)
            joints=dict(zip(ARM_JOINTS,q));joints.update(joint7=finger,joint8=finger);fk=forward_kinematics(YAM_URDF,joints)
            for link,name,v,e,n in geom:
                if link=='base':continue
                R=fk[link][:3,:3];vworld=v@R.T+fk[link][:3,3]+station.BASES[side];low=vworld.min(0);high=vworld.max(0)
                for fixture,fl,fh in fixed+base:
                    delta=np.maximum(np.maximum(fl-high,low-fh),0);gap=float(np.linalg.norm(delta))
                    if gap<min_gap:min_gap=gap;closest={'step':k,'label':label,'link':link,'mesh':name,'fixture':fixture}
                    if gap<.001 and intersects(vworld,e@R.T,n@R.T,fl-.001,fh+.001):
                        hit={'step':k,'t_s':t,'label':label,'link':link,'mesh':name,'fixture':fixture}
                        (base_hits if fixture.startswith('base/') else fixed_hits).append(hit)
            if k%200==0:print(side,k,'/',frames,'IK mm',p.worst_ik_mm,'fixed hits',len(fixed_hits),'base hits',len(base_hits),flush=True)
        result['arms'][side]={'frames':frames,'worst_ik_mm':p.worst_ik_mm,'worst_ik_deg':p.worst_ik_deg,'joint_limits':limits,'worst_pose':worst,'fixed_collision_count':len(fixed_hits),'base_collision_count':len(base_hits),'fixed_hits_first_30':fixed_hits[:30],'base_hits_first_30':base_hits[:30],'min_aabb_gap_m':min_gap,'closest':closest}
    result['failures']=gates(result);result['passed']=not result['failures'];output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('output',type=Path);a=p.parse_args();audit(a.folder,a.output)
