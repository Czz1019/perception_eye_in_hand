import json
from pathlib import Path
import numpy as np
p=Path(__file__).resolve().parent;reference=p.parents[1]/'ur7e_nvblox_in_hand/artifacts/lab_accuracy_20261110';q=np.load(reference/'esdf_roi_query.npz')['xyz']
for phase in ('static','multiview'):
 folder=p/phase
 if not (folder/'esdf_response.json').exists():continue
 meta=json.loads((folder/'esdf_response.json').read_text());data=np.load(folder/'esdf_response.npy').reshape(meta['dims']);origin=np.array(meta['origin']);v=meta['voxel_size'];idx=np.rint((q-origin)/v-.5).astype(int);assert np.max(abs(origin+(idx+.5)*v-q))<1e-5
 d=data[tuple(idx.T)];observed=np.isfinite(d)&(abs(d)<999);np.savez_compressed(folder/'esdf_roi_query.npz',xyz=q,distance=d,observed=observed)
 a=np.zeros((int(observed.sum()),7),np.float32);a[:,:3]=q[observed];a[:,3]=d[observed];a[:,4]=1;a[:,5]=d[observed]<0;np.save(folder/'esdf_roi_observed.npy',a)
 # Reuse previous scene geometry and accuracy methods, explicitly label service ROI cloud.
 s=(reference/'analyze.py').read_text().replace("p=Path(os.environ.get('UR7E_LAB_TEST_OUT',str(Path(__file__).resolve().parent)))", 'p=Path('+repr(str(folder))+')').replace("p/'esdf_native.npy'", "p/'esdf_roi_observed.npy'").replace("'native_observed_points'", "'roi_observed_points_from_service'").replace('UR7e Python package | lab bench', 'workspaces official ROS NvbloxNode | lab bench')
 (folder/'analyze_snapshot.py').write_text(s)
 print(phase,'observed',observed.sum())
