"""Render model envelopes in tool0 at the archived scan-end configuration."""
import json
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from ur7e_real_static.model import Geometry
from ur7e_real_static.camera_collision import vendor_body


def main():
    ws = Path(__file__).resolve().parents[3]
    package = ws/'src/ur7e_real_static'
    output = package/'validation/camera_collision'
    urdf = '/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf'
    old = Geometry(urdf, '/home/wqj/ur7e_real_setup/collision_spheres_preview.yaml')
    new = Geometry(urdf, package/'config/collision_spheres_d435i.yaml')
    metadata = yaml.safe_load((package/'config/collision_spheres_d435i.yaml').read_text())['camera_collision']
    points, _ = vendor_body(metadata['source_description'])
    transform = np.array(metadata['T_tool0_body'])
    points = points @ transform[:3, :3].T+transform[:3, 3]
    q = json.loads((ws/'log/real_first_scan_20260927/frozen_start_check.json').read_text())['q']
    tool = new.fk(q)['tool0']
    u, v = np.meshgrid(np.linspace(0, 2*np.pi, 36), np.linspace(0, np.pi, 20))
    unit = np.stack([np.cos(u)*np.sin(v), np.sin(u)*np.sin(v), np.cos(v)], axis=-1)
    fig = plt.figure(figsize=(13, 6.5))
    for i, (g, title) in enumerate([(old, 'Before: one 53.35 mm sphere'),
                                   (new, 'After: six 27.12 mm spheres')]):
        ax = fig.add_subplot(1, 2, i+1, projection='3d')
        centers = (g.spheres(q)-tool[:3, 3]) @ tool[:3, :3]
        for center, radius, link in zip(centers, g.radii, g.link_ids):
            name = g.links[link]
            if name not in ['tool0', 'wrist_1_link', 'wrist_2_link']: continue
            surface = (center+radius*unit)*1000
            color = '#168cbb' if name == 'tool0' else ('#e0a04e' if name == 'wrist_1_link' else '#ae688c')
            ax.plot_surface(*surface.transpose(2, 0, 1), color=color, alpha=.19, linewidth=0)
        mesh = points[::120]*1000
        ax.scatter(*mesh.T, s=1, color='#14374b', alpha=.8)
        ax.set(xlabel='tool0 X (mm)', ylabel='tool0 Y (mm)', zlabel='tool0 Z (mm)',
               xlim=(-135, 50), ylim=(-135, 50), zlim=(-125, 60), title=title)
        ax.set_box_aspect((1, 1, 1));ax.view_init(elev=27, azim=-125)
    fig.suptitle('UR7e + calibrated D435i collision envelopes', fontsize=17)
    handles = [Line2D([], [], color=c, linewidth=5, label=label) for c, label in
               [('#168cbb', 'Camera cover'), ('#14374b', 'Vendor camera geometry'),
                ('#e0a04e', 'Wrist 1 spheres'), ('#ae688c', 'Wrist 2 sphere')]]
    fig.legend(handles=handles, loc='lower center', ncol=4, bbox_to_anchor=(.5, .04))
    fig.text(.5, .015, 'Archived pose only | 5 mm body padding retained | bracket, gripper and cable not modelled',
             ha='center', fontsize=10)
    fig.subplots_adjust(top=.87, bottom=.17, wspace=.12)
    fig.savefig(output/'comparison.png', dpi=180)
    print(output/'comparison.png')


if __name__ == '__main__':
    main()
