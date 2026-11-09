from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;r=json.loads((p/'results.json').read_text());tf=r['world_from_base_link'];rot=Rotation.from_quat(tf['quaternion_xyzw']).as_matrix();tr=np.array(tf['translation']);out={'description':'World AABB and XY column projection of exported observed ESDF voxel centres. Projection is not physical reconstructed surface area or complete free space.','snapshots':{}}
for name in ['before','overhead']:
 a=np.load(p/(name+'_esdf.npy'));xyz=np.column_stack([a['x'],a['y'],a['z']])@rot.T+tr
 xy=np.round(xyz[:,:2]/.03).astype(np.int64);n=len(np.unique(xy,axis=0));out['snapshots'][name]={'points':len(xyz),'world_min_m':xyz.min(axis=0).tolist(),'world_max_m':xyz.max(axis=0).tolist(),'observed_xy_columns':n,'projected_column_area_m2':n*.03*.03}
(p/'footprint.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
