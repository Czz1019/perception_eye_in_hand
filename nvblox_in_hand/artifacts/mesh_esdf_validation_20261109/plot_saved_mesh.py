"""Scientific comparison of saved native PLYs at identical axes and camera."""
from pathlib import Path
import re,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
p=Path(__file__).resolve().parent;r=json.loads((p/'results.json').read_text());t=r['world_from_base_link'];rot=Rotation.from_quat(t['quaternion_xyzw']).as_matrix();trans=np.array(t['translation'])
fig=plt.figure(figsize=(13,10))
for i,(mode,stage) in enumerate([('voxel0p03_r1','static'),('voxel0p03_r1','measure'),('voxel0p02_r1','static'),('voxel0p02_r1','measure')]):
 file=p/('mesh_'+mode+('_static' if stage=='static' else '')+'.ply')
 with file.open() as f:
  h=''
  while not h.endswith('end_header\n'):h+=f.readline()
  n=int(re.search(r'element vertex (\d+)',h)[1]);nf=int(re.search(r'element face (\d+)',h)[1]);v=np.array([list(map(float,f.readline().split())) for _ in range(n)]);faces=np.array([list(map(int,f.readline().split()))[1:] for _ in range(nf)])
 xyz=v[:,:3]@rot.T+trans;tri=xyz[faces];center=tri.mean(1)
 # One fixed obstacle ROI in world coordinates, retaining all triangles whose centroids lie inside.
 keep=(center[:,0]>.15)&(center[:,0]<.65)&(center[:,1]>-.5)&(center[:,1]<.56)&(center[:,2]>-.02)&(center[:,2]<.62)
 color=v[:,6:9][faces].mean(1)/255
 ax=fig.add_subplot(2,2,i+1,projection='3d');ax.add_collection3d(Poly3DCollection(tri[keep],facecolors=color[keep],edgecolors='none',rasterized=True))
 ax.set(xlim=(.15,.65),ylim=(-.5,.56),zlim=(-.02,.62),xlabel='world X (m)',ylabel='world Y (m)',zlabel='Z (m)');ax.set_box_aspect((.5,1.06,.64));ax.view_init(elev=27,azim=-145)
 ax.set_title(('0.03 m' if '03' in mode else '0.02 m')+' | '+('stationary' if stage=='static' else 'after lateral scan'))
fig.suptitle('Saved native mesh, same world ROI / axes / view; triangle mean vertex RGB\nOffline PLY plot, not an RViz screenshot',fontsize=13)
fig.tight_layout(rect=(0,0,1,.94));fig.savefig(p/'saved_mesh_comparison.png',dpi=150);plt.close(fig)
