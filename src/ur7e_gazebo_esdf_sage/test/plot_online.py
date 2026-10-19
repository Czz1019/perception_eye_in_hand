"""Plot measured online updates and independent experiment events."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();out=Path(a.output)
updates=[json.loads(s) for s in (out/'online/updates.jsonl').read_text().splitlines()]
events=json.loads((out/'online_scenario.json').read_text());start=events[0]['sim_time']
t=np.array([u['commit_sim_time'] for u in updates])-start
fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True,constrained_layout=True)
axes[0].plot(t,[u['occupied_cells'] for u in updates]);axes[0].set_ylabel('Added occupied cells')
axes[1].plot(t,[u['latency'] for u in updates]);axes[1].axhline(.5,color='r',linestyle='--',label='Acceptance age limit')
axes[1].set_ylabel('Receive-to-commit (s)');axes[1].set_xlabel('Simulation time since baseline ready (s)');axes[1].legend()
for e in events:
 if e['kind'] in ['spawn_completed','detected','delete_completed','ray_cleared','resumed_reached']:
  x=e['sim_time']-start
  axes[0].axvline(x,color='gray',alpha=.5);axes[0].annotate(e['kind'],(x,1),xycoords=('data','axes fraction'),rotation=45,fontsize=8)
for ax in axes:ax.grid(alpha=.2)
fig.savefig(out/'online_timeline.png',dpi=160)
