"""Read-only post-run audit. Fixture truth is never returned to the planner."""
import argparse
import json
from pathlib import Path
import numpy as np
from ur7e_real_static.field import FrozenMap
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage.config import load_mapping, identity
from ur7e_gazebo_esdf_sage.field import PLANNING_POLICY
from design_scan import Fixture

p = argparse.ArgumentParser()
p.add_argument('output')
a = p.parse_args()
out = Path(a.output)
c = load_mapping(out/'mapping.yaml')
session = next((out/'maps').glob('*/frozen_map.npz')).parent
frozen = FrozenMap.load(session/'frozen_map.npz', identity(c))
snap = frozen.snapshot
z, y, x = np.indices(snap.distance.shape)
points = snap.origin + snap.resolution*np.stack([x, y, z], -1).reshape(-1, 3)
source = frozen.source.reshape(snap.distance.shape)
with np.load(session/'tabletop_evidence.npz') as data:
    reliable = data['reliable'].copy()
    upper = data['upper_height'].copy()
    counts = data['view_count'].copy()
eligible = reliable[None] & (points[:, 2].reshape(source.shape) > upper[None])
truth = Fixture().query(points)
inferred = frozen.source == 2
free = frozen.source > 0
rows = np.genfromtxt(out/'execution.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
g = Geometry(c['robot_urdf'], c['collision_spheres'])
q = np.stack([rows[n] for n in rows.dtype.names[4:10]], -1)
command = np.stack([rows[n] for n in rows.dtype.names[10:16]], -1)
fixture_gaps, self_gaps = [], []
for i in range(0, len(q), 512):
    centers = g.spheres_batch(q[i:i+512])
    fixture_gaps.extend((Fixture().query(centers)-g.radii).min(-1))
    self_gaps.extend(g.self_clearance_batch(centers))
stopped = rows['phase'] == 'STOPPED'
result = dict(
    planning_policy=frozen.metadata['planning_policy'], map_uid=frozen.uid,
    frames=frozen.metadata['sequence'], reliable_columns=int(reliable.sum()),
    inferred_cells=int(inferred.sum()),
    inferred_without_column_support=int(np.count_nonzero(inferred & ~eligible.reshape(-1))),
    inferred_inside_fixture=int(np.count_nonzero(inferred & (truth <= 0))),
    all_free_inside_fixture=int(np.count_nonzero(free & (truth <= 0))),
    min_inferred_fixture_distance_m=float(truth[inferred].min()),
    max_free_distance_overestimate_m=float(np.max(snap.distance.reshape(-1)[free]-truth[free])),
    measured_samples=len(q), min_measured_fixture_gap_m=float(min(fixture_gaps)),
    min_measured_self_gap_m=float(min(self_gaps)),
    reaching_samples=int(np.count_nonzero(rows['phase'] == 'REACHING')),
    stopped_samples=int(stopped.sum()),
    stopped_command_range_rad=float(np.ptp(command[stopped], axis=0).max()) if stopped.any() else None)
(out/'tabletop_audit.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
assert result['planning_policy'] == PLANNING_POLICY
assert result['inferred_cells'] > 0 and result['inferred_without_column_support'] == 0
assert np.all(counts[reliable] >= c['tabletop']['min_distinct_views'])
assert result['all_free_inside_fixture'] == 0
assert result['max_free_distance_overestimate_m'] < 1e-5
assert result['min_measured_fixture_gap_m'] > c['collision_margin']
assert result['min_measured_self_gap_m'] > c['self_margin']
if stopped.any():
    assert result['stopped_command_range_rad'] < 1e-8

# Spatial coverage evidence, including the blocked robot footprint.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
extent = [snap.origin[0]-.01, snap.origin[0]+(snap.distance.shape[2]-.5)*snap.resolution,
          snap.origin[1]-.01, snap.origin[1]+(snap.distance.shape[1]-.5)*snap.resolution]
im = axes[0].imshow(np.where(reliable, upper, np.nan), origin='lower', extent=extent, cmap='viridis')
fig.colorbar(im, ax=axes[0], label='Observed top + margin (m)')
axes[0].set_title('Reliable columns; white = unsupported')
zi = int(round((.6-snap.origin[2])/snap.resolution))
from matplotlib.colors import ListedColormap, BoundaryNorm
im = axes[1].imshow(source[zi], origin='lower', extent=extent,
                    cmap=ListedColormap(['#dddddd', '#2987ba', '#e6aa35']), norm=BoundaryNorm([-.5,.5,1.5,2.5], 3))
fig.colorbar(im, ax=axes[1], ticks=[0,1,2], label='0 unknown/occupied; 1 rays; 2 top inference')
axes[1].set_title('Planning map at z = 0.60 m')
centers = g.spheres(q[-1])
for ax in axes:
    ax.scatter(centers[:,0], centers[:,1], s=13, c='red', label='Final arm sphere centers (XY)')
    ax.set(xlabel='x (m)', ylabel='y (m)');ax.legend(fontsize=7)
fig.savefig(out/'tabletop_coverage.png', dpi=160)
