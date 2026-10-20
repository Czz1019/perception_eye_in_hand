"""Measured motion and online-update timeline, independent of planner input."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ur7e_gazebo_esdf_sage.config import load_mapping
from ur7e_real_static.model import Geometry
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
c=load_mapping(out/'mapping.yaml');g=Geometry(c['robot_urdf'],c['collision_spheres'])
e=json.loads((out/'moving_scenario.json').read_text());start=e[0]['sim_time'];tr=[json.loads(x) for x in (out/'moving_truth.jsonl').read_text().splitlines()];u=[json.loads(x) for x in (out/'online/updates.jsonl').read_text().splitlines()]
r=np.genfromtxt(out/'execution.csv',delimiter=',',names=True,dtype=None,encoding='utf8');r=r[(r['sim_time']>=start)&(r['sim_time']<=e[-1]['sim_time'])][::4];q=np.stack([r[n] for n in r.dtype.names[4:10]],-1);tool=np.array([g.fk(x)['tool0'][:3,3] for x in q])
fig,ax=plt.subplots(3,1,figsize=(12,8),constrained_layout=True,sharex=True)
ax[0].plot(r['sim_time']-start,tool[:,1],label='Tool y')
for trial in ['crossing','blocked']:
 rows=[x for x in tr if x['trial']==trial]
 if rows:ax[0].plot([x['stamp']-start for x in rows],[x['position'][1] for x in rows],label='Obstacle y: '+trial)
ax[0].set_ylabel('Position y (m)');ax[0].legend(loc='upper right')
u=[x for x in u if start<=x['commit_sim_time']<=e[-1]['sim_time']];t=np.array([x['commit_sim_time'] for x in u]);ax[1].plot(t-start,[x['occupied_cells'] for x in u]);ax[1].set_ylabel('Online occupied cells')
ax[2].plot(t[1:]-start,np.diff(t),label='Commit interval');ax[2].plot(t-start,[x['latency'] for x in u],alpha=.6,label='Receive-to-commit');ax[2].axhline(.5,color='r',linestyle='--',label='Map age limit');ax[2].set_ylabel('Time (s)');ax[2].set_xlabel('Simulation time since online ready (s)');ax[2].legend()
for row in e:
 if row['kind'] in ['crossing_begin','crossing_reached','unblocking_begin','resumed_reached']:
  for axis in ax:axis.axvline(row['sim_time']-start,color='gray',alpha=.3)
  ax[0].annotate(row['kind'],(row['sim_time']-start,1),xycoords=('data','axes fraction'),rotation=30,fontsize=8,ha='right')
for axis in ax:axis.grid(alpha=.2)
fig.savefig(out/'moving_timeline.png',dpi=150)
