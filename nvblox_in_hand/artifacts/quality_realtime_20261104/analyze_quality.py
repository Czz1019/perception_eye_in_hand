from pathlib import Path
import json
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import yaml
p=Path('/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/quality_realtime_20261104')
cfg=yaml.safe_load(Path('/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo/config/collision_world_gazebo_tall.yml').read_text())
boxes=cfg['world_model']['coll_objs']['cube']
boxes={k:v for k,v in boxes.items() if k!='ground'}
# Metric only uses world-frame geometry after checking saved world<-base TF.
r=json.loads((p/'results.json').read_text())
tf=r.get('world_from_base_link')
if not tf:raise RuntimeError('No verified world<-base_link transform')
from scipy.spatial.transform import Rotation
rot=Rotation.from_quat(tf['quaternion_xyzw']).as_matrix();trans=np.asarray(tf['translation'])
def sdf_box(x,c,h):
    q=np.abs(x-c)-h
    return np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)
def ref_samples(b):
    c=np.asarray(b['pose'][:3]);h=np.asarray(b['dims'])/2;parts=[]
    for axis in range(3):
        other=[a for a in range(3) if a!=axis]
        grids=[np.linspace(c[a]-h[a],c[a]+h[a],max(2,int(b['dims'][a]/.01)+1)) for a in other]
        v,w=np.meshgrid(*grids,indexing='ij')
        for sign in [-1,1]:
            x=np.zeros((v.size,3));x[:,axis]=c[axis]+sign*h[axis]
            x[:,other[0]]=v.ravel();x[:,other[1]]=w.ravel();parts.append(x)
    x=np.unique(np.concatenate(parts),axis=0)
    return x[x[:,2]>=.02]
reference={k:ref_samples(v) for k,v in boxes.items()}
result={'method':'mesh_vertex_to_known_wall_box_surface_and_reference_surface_proximity',
        'coverage_threshold_m':.03,'reference_grid_step_approx_m':.01,
        'limitations':['Wall neighborhood only, not full-scene ground-truth error.',
                      'Surface proximity is not observed voxel coverage and includes unobservable backsides.',
                      'Vertex-weighted distances are affected by tessellation and do not equal uniform area error.'],
        'stages':{}}
for name in ['mesh_static','mesh_after_motion','mesh_final']:
    path=p/(name+'.ply')
    if not path.exists():continue
    mesh=trimesh.load(path,process=False)
    x=np.asarray(mesh.vertices)@rot.T+trans
    region=np.zeros(len(x),bool)
    errors=[]
    for b in boxes.values():
        c=np.asarray(b['pose'][:3]);h=np.asarray(b['dims'])/2
        region|=np.all(np.abs(x-c)<=h+.08,axis=1)
        errors.append(np.abs(sdf_box(x,c,h)))
    region&=(x[:,2]>=.02)&(x[:,2]<=.58)
    d=np.min(np.stack(errors),axis=0)[region]
    stage={'vertices':len(x),'triangles':len(mesh.faces),'wall_neighborhood_vertices':int(region.sum()),
           'wall_neighborhood_definition':'union of wall boxes expanded 0.08m, z>=0.02m and z<=0.58m'}
    if len(d):stage.update({'wall_surface_abs_error_median_m':float(np.median(d)),
        'wall_surface_abs_error_p95_m':float(np.percentile(d,95)),
        'wall_surface_abs_error_mean_m':float(np.mean(d)),
        'fraction_within_0.02m':float(np.mean(d<=.02)),
        'fraction_beyond_0.04m':float(np.mean(d>.04))})
    tree=cKDTree(x)
    stage['reference_wall_surface_proximity_fraction']={k:float(np.mean(tree.query(v)[0]<=.03)) for k,v in reference.items()}
    result['stages'][name]=stage
(p/'quality_metrics.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
