"""Show environmental evidence separately from explicitly admitted uncertainty."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Circle
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
from ur7e_nvblox.depth import known_distances
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output);c=load_mapping(out/'mapping.yaml');path=next((out/'maps').glob('*/partial_policy.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c));snap=f.snapshot
with np.load(path) as d:admitted=d['admitted']
with np.load(path.parent/'robot_ownership.npz') as d:owned=d['owned'];m=json.loads(str(d['metadata']))
with np.load(path.parent/'native_esdf.npz') as d:occupied=known_distances(d['distance'],float(d['unknown']))&(d['distance']<=.02)
labels=np.zeros_like(f.source);labels[f.source>0]=1;labels[(f.source==0)&admitted]=2;labels[owned]=3;labels[occupied]=4;labels=labels.reshape(snap.distance.shape)
g=Geometry(c['robot_urdf'],c['collision_spheres']);cs=g.spheres(m['freeze_q']);fig,axes=plt.subplots(1,2,figsize=(11,6),layout='constrained')
colors=ListedColormap(['#dddddd','#4c9bc2','#efbd60','#65aa78','#333333']);norm=BoundaryNorm(np.arange(-.5,5.5),5)
for ax,y in zip(axes,[0.,-.10]):
 iy=int(round((y-snap.origin[1])/snap.resolution));extent=[snap.origin[0]-.01,snap.origin[0]+(snap.dimensions[0]-.5)*snap.resolution,snap.origin[2]-.01,snap.origin[2]+(snap.dimensions[2]-.5)*snap.resolution]
 im=ax.imshow(labels[:,iy,:],origin='lower',extent=extent,cmap=colors,norm=norm,interpolation='nearest')
 for center,radius in zip(cs,g.radii+.03):
  cut=radius**2-(center[1]-y)**2
  if cut>0:ax.add_patch(Circle(center[[0,2]],np.sqrt(cut),fill=False,color='black',linewidth=.7))
 ax.set(xlim=(-.3,.75),ylim=(-.05,1.25),xlabel='base x [m]',ylabel='base z [m]',title=f'y = {y:.2f} m')
fig.colorbar(im,ax=axes,ticks=range(5),shrink=.7).ax.set_yticklabels(['Blocked unknown','Environmental free evidence','Admitted UNKNOWN (risk)','URDF interior','Measured occupied'])
fig.suptitle('Partial-map policy: yellow remains unknown; no scan corridor or fixture truth')
fig.savefig(out/'partial_policy_sections.png',dpi=160);plt.close(fig)
