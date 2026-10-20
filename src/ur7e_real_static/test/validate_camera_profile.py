"""Offline validation of this machine's camera profile; never connects hardware."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from ur7e_real_static.camera_collision import vendor_body
from ur7e_real_static.config import load_config
from ur7e_real_static.model import Geometry


def clearance(g, qs):
    centers = g.spheres_batch(qs)
    gaps = np.linalg.norm(centers[:, g.pair_a]-centers[:, g.pair_b], axis=-1)
    gaps -= g.radii[g.pair_a]+g.radii[g.pair_b]
    row, pair = np.unravel_index(np.argmin(gaps), gaps.shape)
    a, b = g.pair_a[pair], g.pair_b[pair]
    return dict(min_gap_m=float(gaps[row, pair]), sample_index=int(row),
                pair=[g.links[g.link_ids[a]], g.links[g.link_ids[b]]],
                sphere_indices=[int(a), int(b)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    ws, output = Path(args.workspace).resolve(), Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    package = ws/'src/ur7e_real_static'
    c = load_config(package/'config/real_d435i.yaml')
    g = Geometry(c['robot_urdf'], c['collision_spheres'])
    old = Geometry(c['robot_urdf'], '/home/wqj/ur7e_real_setup/collision_spheres_preview.yaml')
    data = yaml.safe_load(Path(c['collision_spheres']).read_text())
    metadata = data['camera_collision']
    vertices, _ = vendor_body(metadata['source_description'])
    transform = np.asarray(metadata['T_tool0_body'])
    vertices = vertices @ transform[:3, :3].T+transform[:3, 3]
    payload = g.link_ids == g.links.index('tool0')
    distances = []
    for start in range(0, len(vertices), 8192):
        distances.extend(np.min(np.linalg.norm(vertices[start:start+8192, None]-g.local[payload], axis=-1)
                                -g.radii[payload], axis=1))
    report = dict(hardware_connected=False, environment_collision_validated=False,
                  camera_spheres=int(payload.sum()), total_spheres=len(g.radii),
                  camera_radius_m=float(g.radii[payload][0]), padding_m=metadata['padding_m'],
                  vendor_points_checked=len(vertices), max_vendor_point_signed_distance_m=float(max(distances)),
                  self_margin_m=c['self_margin'], unmodelled=metadata['unmodelled'])
    assert max(distances) <= -metadata['padding_m']+1e-9
    assert [(a, b) for a, b in old.pairs if a < 16] == [(a, b) for a, b in g.pairs if a < 16]
    # Every new camera sphere checks exactly the same arm links as the old one.
    old_checked = {old.links[old.link_ids[b]] for a, b in old.pairs if a == 16}
    for a in np.flatnonzero(payload):
        assert {g.links[g.link_ids[b]] for aa, b in g.pairs if aa == a} == old_checked
    reference = json.loads((ws/'log/real_first_scan_20260927/frozen_start_check.json').read_text())['q']
    route = yaml.safe_load((ws/'src/ur7e_auto_reconstruction/config/waypoints.yaml').read_text())
    names = ['historical_scan_end']+[w['name'] for w in route['waypoints']]
    qs = np.array([reference]+[w['q_rad'] for w in route['waypoints']])
    report['poses'] = [dict(name=name, old=clearance(old, [q]), new=clearance(g, [q]),
                           passes_self_margin=clearance(g, [q])['min_gap_m'] >= c['self_margin'])
                       for name, q in zip(names, qs)]
    assert report['poses'][0]['passes_self_margin']
    # The historical scan end is not claimed to be the current real start.
    report['old_waypoint_edges'] = []
    for i, (a, b) in enumerate(zip(qs[1:-1], qs[2:])):
        motion = g.motion_bound*float(np.sum(abs(b-a)))
        count = max(2, int(np.ceil(motion/.004))+1)
        values = []
        for start in range(0, count, 256):
            alpha = np.arange(start, min(count, start+256))/(count-1)
            values.append(clearance(g, a+(b-a)*alpha[:, None]))
        minimum = min(v['min_gap_m'] for v in values)
        lower_bound = float(minimum-motion/(count-1))  # two moving surfaces
        report['old_waypoint_edges'].append(dict(start=names[i+1], end=names[i+2], samples=count,
            sampled_min_gap_m=minimum, conservative_self_gap_m=lower_bound,
            passes_self_margin=lower_bound >= c['self_margin']))
    # Random states ensure collision rejection remains active; no ignore-pair changes.
    random_q = np.random.default_rng(40).uniform(-np.pi, np.pi, (256, 6))
    random_gaps = [clearance(g, [q])['min_gap_m'] for q in random_q]
    report['random_states_below_margin'] = sum(v < c['self_margin'] for v in random_gaps)
    assert report['random_states_below_margin'] > 0
    import torch
    from ur7e_real_static.planning import build_task
    task = build_task(g, c, output/'sage_task')
    try:
        rollout = task.controller.rollout_fn
        np.testing.assert_allclose(rollout.local.cpu(), g.local, atol=1e-7)
        np.testing.assert_allclose(rollout.radii.cpu(), g.radii, atol=1e-7)
        np.testing.assert_array_equal(rollout.pair_a.cpu(), g.pair_a)
        np.testing.assert_array_equal(rollout.pair_b.cpu(), g.pair_b)
        tq = torch.tensor(qs, **task.tensor_args)
        robot = rollout.dynamics_model.robot_model
        centers = np.empty((len(qs), len(g.radii), 3))
        with torch.no_grad():
            for i, link in enumerate(g.links):
                position, rotation = robot.compute_forward_kinematics(tq, torch.zeros_like(tq), link_name=link)
                ids = g.link_ids == i
                centers[:, ids] = (g.local[ids] @ rotation.cpu().numpy().transpose(0, 2, 1)
                                    +position.cpu().numpy()[:, None, :])
        error = float(np.max(abs(centers-g.spheres_batch(qs))))
        assert error < 2e-5
        cpu_gap = g.self_clearance_batch(g.spheres_batch(qs))
        gpu_gap = g.self_clearance_batch(centers)
        np.testing.assert_allclose(cpu_gap, gpu_gap, atol=2e-5)
        report['sage'] = dict(actual_cuda=True, cpu_gpu_sphere_max_error_m=error,
                              collision_spheres_and_pairs_match=True)
    finally:
        task.close()
    before = json.loads((output/'unchanged_before.json').read_text())
    changed = [name for name, sha in before.items()
               if hashlib.sha256(Path(name).read_bytes()).hexdigest() != sha]
    assert not changed, changed
    report['unchanged_source_files'] = len(before)
    report['adaptation_checks_passed'] = True
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
