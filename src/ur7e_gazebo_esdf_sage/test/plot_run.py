"""Export measured trajectory and independent clearance evidence."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage.config import load_mapping
from design_scan import Fixture
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
r=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8')[::8]
q=np.stack([r[n] for n in r.dtype.names[4:10]],axis=-1);centers=g.spheres_batch(q)
xyz=np.array([g.fk(v)['tool0'][:3,3] for v in q]);t=r['sim_time'];t-=t[0]
planning=np.isin(r['phase'],['REACHING','GOAL_REACHED','SOLVING_GUIDE'])
fig=plt.figure(figsize=(12,5));ax=fig.add_subplot(121,projection='3d')
ax.plot(*xyz[~planning].T,color='#7f8c8d',label='Measured prescan TCP',linewidth=1)
ax.plot(*xyz[planning].T,color='#0072b2',label='Measured SAGE TCP',linewidth=2)
for center in [[.5,.2,.23],[.5,-.28,.23]]:
 ax.bar3d(center[0]-.12,center[1]-.05,0,.24,.1,.46,color='#dd8452',alpha=.3)
u,v=np.mgrid[0:2*np.pi:24j,0:np.pi:12j]
ax.plot_surface(.65+.1*np.cos(u)*np.sin(v),.1*np.sin(u)*np.sin(v),.1+.1*np.cos(v),color='#dd8452',alpha=.3)
ax.set(xlabel='base x [m]',ylabel='base y [m]',zlabel='base z [m]',title='Gazebo: actual feedback trajectory');ax.legend(fontsize=8)
ax=fig.add_subplot(122)
ax.plot(t,(Fixture().query(centers)-g.radii).min(-1)*1000,label='Whole-body to fixture')
ax.plot(t,g.self_clearance_batch(centers)*1000,label='Self clearance')
ax.axhline(30,color='C0',linestyle='--',linewidth=1);ax.axhline(15,color='C1',linestyle='--',linewidth=1)
ax.set(xlabel='Simulation time since first sample [s]',ylabel='Clearance [mm]',title='Independent geometric audit');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.tight_layout();fig.savefig(out/'trajectory_evidence.png',dpi=160);plt.close(fig)
