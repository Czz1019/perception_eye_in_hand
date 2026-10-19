"""Map cross sections showing sensed freedom, owned interiors and unknowns."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage_continuous.config import load_mapping,identity
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');path=next((out/'maps').glob('*/robot_ownership.npz'));f=FrozenMap.load(path.parent/'frozen_map.npz',identity(c))
with np.load(path,allow_pickle=False) as data:owned=data['owned'];q=json.loads(str(data['metadata']))['freeze_q']
g=Geometry(c['robot_urdf'],c['collision_spheres']);centres=g.spheres(q);snap=f.snapshot
labels=f.source.copy();labels[owned]=3;labels=labels.reshape(snap.distance.shape)
fig,axs=plt.subplots(1,3,figsize=(12,6),constrained_layout=True)
for ax,yslice in zip(axs,[0.,-.1,-.2]):
 yi=int(round((yslice-snap.origin[1])/snap.resolution));r=snap.resolution
 extent=[snap.origin[0]-r/2,snap.origin[0]+(snap.dimensions[0]-.5)*r,snap.origin[2]-r/2,snap.origin[2]+(snap.dimensions[2]-.5)*r]
 im=ax.imshow(labels[:,yi,:],origin='lower',extent=extent,cmap=ListedColormap(['#ddd','#4c9bc6','#edbc62','#64ad77']),norm=BoundaryNorm([-.5,.5,1.5,2.5,3.5],4))
 for centre,radius in zip(centres,g.radii+c['collision_margin']):
  delta=abs(centre[1]-yslice)
  if delta<radius:ax.add_patch(plt.Circle(centre[[0,2]],np.sqrt(radius**2-delta**2),fill=False,color='#333',linewidth=.7))
 ax.set(xlim=(-.3,.35),ylim=(-.05,1.2),xlabel='x (m)',ylabel='z (m)',title=f'y = {yslice:.2f} m')
cbar=fig.colorbar(im,ax=axs,ticks=[0,1,2,3],shrink=.65);cbar.ax.set_yticklabels(['Unknown/occupied','Depth rays','Observed-top inference','URDF interior'])
fig.suptitle('Freeze pose: outlines = collision spheres + 30 mm environment margin')
fig.savefig(out/'ownership_sections.png',dpi=160)
