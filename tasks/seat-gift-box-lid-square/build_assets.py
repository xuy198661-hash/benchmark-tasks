"""Original printable PLA box pair; no third-party mesh. All units in metres except STL (mm)."""
from pathlib import Path
import argparse, hashlib, itertools, json, struct
import numpy as np

DENSITY = 1240.0  # Solid PLA; mass verification on the actual print is outstanding.

def panels(length, width, height, thickness, lid=False, top_thickness=None):
    t=thickness
    panel_t=t if top_thickness is None else top_thickness
    floor_z=height-panel_t/2 if lid else panel_t/2
    wall_z=(height-panel_t)/2 if lid else (height+panel_t)/2
    return [
        ('panel',(length,width,panel_t),(0,0,floor_z)),
        ('side_positive',(length,t,height-panel_t),(0,(width-t)/2,wall_z)),
        ('side_negative',(length,t,height-panel_t),(0,-(width-t)/2,wall_z)),
        ('end_positive',(t,width-2*t,height-panel_t),((length-t)/2,0,wall_z)),
        ('end_negative',(t,width-2*t,height-panel_t),(-(length-t)/2,0,wall_z)),
    ]

def union_triangles(parts):
    bounds=[(np.array(c)-np.array(s)/2,np.array(c)+np.array(s)/2) for _,s,c in parts]
    grid=[sorted(set(round(float(v),9) for lo,hi in bounds for v in (lo[a],hi[a]))) for a in range(3)]
    occupied=set()
    for idx in itertools.product(*(range(len(g)-1) for g in grid)):
        mid=np.array([(grid[a][i]+grid[a][i+1])/2 for a,i in enumerate(idx)])
        if any(np.all(mid>lo-1e-12) and np.all(mid<hi+1e-12) for lo,hi in bounds):occupied.add(idx)
    faces=[]
    for idx in sorted(occupied):
        for a in range(3):
            b,c=(a+1)%3,(a+2)%3
            for sign in (-1,1):
                nb=list(idx);nb[a]+=sign
                if tuple(nb) in occupied:continue
                verts=[]
                for db,dc in ((0,0),(1,0),(1,1),(0,1)):
                    v=[grid[j][idx[j]] for j in range(3)];v[a]=grid[a][idx[a]+(sign>0)];v[b]=grid[b][idx[b]+db];v[c]=grid[c][idx[c]+dc];verts.append(v)
                if sign<0:verts=verts[::-1]
                faces.extend(([verts[0],verts[1],verts[2]],[verts[0],verts[2],verts[3]]))
    return np.array(faces)

def write_stl(path, tri):
    data=bytearray(b'Original gift box design; millimetres'.ljust(80,b' '))+struct.pack('<I',len(tri))
    for f in tri*1000:
        n=np.cross(f[1]-f[0],f[2]-f[0]);n/=np.linalg.norm(n)
        data.extend(struct.pack('<12fH',*n,*f.ravel(),0))
    path.write_bytes(data)

def measure_stl(path):
    data=path.read_bytes();n=struct.unpack('<I',data[80:84])[0]
    tri=np.frombuffer(data,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)['v'].astype(float)/1000
    v=tri.reshape(-1,3); tet=np.einsum('ij,ij->i',tri[:,0],np.cross(tri[:,1],tri[:,2]))/6
    volume=tet.sum();com=np.sum((tri.sum(axis=1)/4)*tet[:,None],axis=0)/volume
    edges={}
    for face in tri:
        for j in range(3):
            pair=tuple(sorted((tuple(face[j]),tuple(face[(j+1)%3]))));edges[pair]=edges.get(pair,0)+1
    if volume<=0 or any(n!=2 for n in edges.values()):raise ValueError('STL is not a closed positive-volume solid')
    return {'low_m':v.min(0).tolist(),'high_m':v.max(0).tolist(),'size_m':np.ptp(v,axis=0).tolist(),'center_of_mass_m':com.tolist(),'volume_m3':float(volume),'mass_kg':float(volume*DENSITY),'triangles':len(tri),'watertight_edges':True,'stl_sha256':hashlib.sha256(data).hexdigest()}

def vector(v):return '('+', '.join(f'{float(a):.12g}' for a in v)+')'

def write_usda(path, parts, info):
    lines=['#usda 1.0','(','    defaultPrim = "GiftBox"','    metersPerUnit = 1','    upAxis = "Z"',')','def Xform "GiftBox" (','    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]',')','{','    bool physics:rigidBodyEnabled = true',f'    float physics:mass = {info["mass_kg"]:.12g}',f'    point3f physics:centerOfMass = {vector(info["center_of_mass_m"])}']
    for name,size,center in parts:
        lines += [f'    def Cube "{name}" (', '        prepend apiSchemas = ["PhysicsCollisionAPI"]','    )','    {','        double size = 1',f'        double3 xformOp:translate = {vector(center)}',f'        double3 xformOp:scale = {vector(size)}','        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]','        bool physics:collisionEnabled = true','    }']
    lines+=['}'];path.write_text('\n'.join(lines)+'\n')

def build(out):
    out.mkdir(parents=True,exist_ok=True)
    definitions={'base':(.100,.066,.040,.002,False),'lid':(.112,.078,.028,.004,True,.002)}
    result={}
    for name,args in definitions.items():
        part=panels(*args);stl=out/(name+'.stl');tri=union_triangles(part)
        if name=='lid':
            # Deterministic centroid refinement on every exterior triangle. Keeps
            # a closed union mesh; distinct file sizes also expose asset swaps.
            mid=tri.mean(axis=1)
            tri=np.concatenate([np.stack([tri[:,i],tri[:,(i+1)%3],mid],axis=1) for i in range(3)])
        write_stl(stl,tri);info=measure_stl(stl)
        info.update({'units':'m (STL files use mm)','source':'Original printable design; no physical sample measured','material':'PLA, solid walls and floor, 100 percent infill','density_kg_m3':DENSITY,'print_linear_tolerance_m':.0002,'wall_thickness_m':args[3],'rest_pose':'flat floor down' if name=='base' else 'panel up, skirt down','panels':[{'name':n,'size_m':list(s),'center_m':list(c)} for n,s,c in part]})
        # Derive actual cavity dimensions from the authored panel model, not a hand-entered task constant.
        sides=[p for p in part if p[0].startswith('side')];ends=[p for p in part if p[0].startswith('end')]
        inner_x=min(c[0]-s[0]/2 for _,s,c in ends if c[0]>0)*2
        inner_y=min(c[1]-s[1]/2 for _,s,c in sides if c[1]>0)*2
        panel=part[0];panel_low=panel[2][2]-panel[1][2]/2;panel_high=panel[2][2]+panel[1][2]/2
        info.update({'inner_xy_m':[inner_x,inner_y],'panel_low_m':panel_low,'panel_high_m':panel_high,'panel_thickness_m':panel[1][2],'skirt_height_m':panel_low if name=='lid' else None,'extent_m':max(info['size_m'][:2])})
        write_usda(out/(name+'.usda'),part,info)
        info['usda_sha256']=hashlib.sha256((out/(name+'.usda')).read_bytes()).hexdigest()
        (out/(name+'_info.json')).write_text(json.dumps(info,indent=2)+'\n');result[name]=info
    base,lid=result['base'],result['lid']
    layout={'base_xy_m':[.24,-.10],'lid_xy_m':[.20,-.26],'base_yaw_rad':0.,'lid_yaw_rad':0.,'spawn_gap_m':.001,'gift_size_m':[.060,.038,.020],'gift_density_kg_m3':700.,'grip_above_lid_bottom_m':.019,'release_gap_m':.001,'grip_target_gap_m':.0774,'travel_above_table_m':.12,'clearance_per_side_m':[(lid['inner_xy_m'][a]-base['size_m'][a])/2 for a in range(2)],'seated_lid_bottom_m':base['size_m'][2]-lid['panel_low_m'],'seated_lid_top_m':base['size_m'][2]+lid['panel_thickness_m'],'print_instructions':'Print base floor down and lid panel down without supports, 0.2 mm layers, 0.4 mm nozzle, 100 percent infill, PLA. Remove elephant foot without enlarging the cavity beyond the stated tolerance; inspect the finished dimensions. Invert the lid to panel-up for the scene.'}
    (out/'layout_info.json').write_text(json.dumps(layout,indent=2)+'\n')
    measure_gripper(out)
    print(json.dumps({'base':base['size_m'],'lid':lid['size_m'],'lid_inner':lid['inner_xy_m'],'clearance':layout['clearance_per_side_m'],'mass_kg':[base['mass_kg'],lid['mass_kg']]},indent=2))


def measure_gripper(out):
    # Actual collision-link bounds, measured from the installed YAM mesh sources.
    import xml.etree.ElementTree as ET
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix
    bounds={}
    for link in ET.parse(YAM_URDF).getroot().findall('link'):
        name=link.get('name')
        if name not in ('gripper','tip_left','tip_right'):continue
        points=[]
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh');raw=(YAM_URDF.parent/mesh.get('filename')).read_bytes()
            count=struct.unpack('<I',raw[80:84])[0]
            v=np.frombuffer(raw,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=count)['v'].astype(float).reshape(-1,3)
            v*=np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            origin=collision.find('origin')
            if origin is not None:v=v@rpy_matrix(np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).T+np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            points.extend(v)
        v=np.array(points);bounds[name]={'low':v.min(0).tolist(),'high':v.max(0).tolist()}
    result={'source':'Measured from installed YAM URDF collision meshes; meshes are not redistributed','link_bounds_m':bounds}
    (out/'gripper_info.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);build(p.parse_args().output)
