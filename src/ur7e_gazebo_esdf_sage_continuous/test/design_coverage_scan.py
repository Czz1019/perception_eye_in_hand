"""Commission extra observation poses, never generate planning free space.

Only this offline check uses fixture geometry. The resulting camera poses must
still be exercised with Gazebo depth and the unchanged planning-map policy.
"""
import argparse
from pathlib import Path
import numpy as np
import yaml
from design_scan import Fixture
from ur7e_real_static.model import Geometry
from ur7e_gazebo_esdf_sage_continuous.field import scan_samples

ROOT = Path(__file__).resolve().parents[1]
NEAR_VIEWS = [
    [-1.75, -1.8674702523932392, 1.0767620399301674, .9409578163297588, -.7872664448233198, .8296068884341119],
    [-1.75, -1.497305660392845, .9605877854871274, .76041388604236, -1.0692640713408843, .7934271088463187],
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', default=str(ROOT/'config/scan_coverage.yaml'))
    args = p.parse_args()
    c = yaml.safe_load((ROOT/'config/scan.yaml').read_text())
    g = Geometry(ROOT/'config/real_calibrated.urdf', ROOT/'config/collision_spheres.yaml')
    views = list(c['views'])
    for pose, bases in zip(NEAR_VIEWS, [np.linspace(-1.75, 4.25, 13), np.linspace(4.25, -1.75, 13)]):
        for base in bases:
            q = pose.copy(); q[0] = float(base); views.append(q)
    views.append(c['views'][0])
    qs = scan_samples(np.array(views), .00025)
    assert np.all((qs >= g.lower) & (qs <= g.upper))
    environment = self_gap = np.inf
    for start in range(0, len(qs), 512):
        centers = g.spheres_batch(qs[start:start+512])
        environment = min(environment, float((Fixture().query(centers)-g.radii).min()))
        self_gap = min(self_gap, float(g.self_clearance_batch(centers).min()))
    padding = float(g.motion_bound*np.max(np.sum(abs(np.diff(qs, axis=0)), axis=1))/2)
    assert environment-padding > c['prior_clearance']
    assert self_gap-2*padding > .015
    cert = dict(min_self_gap=self_gap, min_environment_gap=environment,
                samples=len(qs), sampling_padding=padding)
    c.update(views=views, commissioning=cert, certification_step_rad=.00025)
    Path(args.output).write_text(yaml.safe_dump(c, sort_keys=False))
    print(dict(views=len(views), **cert))


if __name__ == '__main__':
    main()
