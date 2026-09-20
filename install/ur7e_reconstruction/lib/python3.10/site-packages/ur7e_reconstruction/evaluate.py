"""Compare a measured PLY against scene primitives; reference data never enters fusion."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import yaml
from ament_index_python.packages import get_package_share_directory


def read_ply(path):
    header, payload = Path(path).read_bytes().split(b'end_header\n', 1)
    n = int(next(s.split()[-1] for s in header.decode().splitlines() if s.startswith('element vertex')))
    v = np.frombuffer(payload[:15*n], dtype=[('p', '<f4', (3,)), ('c', 'u1', (3,))])
    f = np.frombuffer(payload[15*n:], dtype=[('n', 'u1'), ('i', '<i4', (3,))])
    return v['p'].copy(), f['i'].copy(), v['c'].copy()


def box_reference(center, dims, step=.005):
    low, high = np.array(center)-np.array(dims)/2, np.array(center)+np.array(dims)/2
    low[2] = max(.01, low[2])  # Exclude the buried portion and the ground contact seam.
    faces = {}
    for axis in range(3):
        for side in range(2):
            if axis == 2 and side == 0:
                continue
            other = [a for a in range(3) if a != axis]
            a, b = [np.arange(low[i]+step/2, high[i], step) for i in other]
            aa, bb = np.meshgrid(a, b)
            points = np.empty((aa.size, 3))
            points[:, axis] = (low if side == 0 else high)[axis]
            points[:, other[0]], points[:, other[1]] = aa.ravel(), bb.ravel()
            faces['xyz'[axis] + ('-' if side == 0 else '+')] = points
    return faces


def box_distance(points, center, dims):
    q = np.abs(points-center)-np.array(dims)/2
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)


def evaluate(path, scene, tolerance=.025):
    vertices, triangles, colors = read_ply(path)
    if not len(triangles):
        raise ValueError('No reconstructed triangles available for comparison')
    # Centroids and edge midpoints supplement vertices when querying observed surface coverage.
    tv = vertices[triangles]
    samples = np.concatenate([vertices, tv.mean(axis=1), (tv[:,0]+tv[:,1])/2,
                              (tv[:,1]+tv[:,2])/2, (tv[:,0]+tv[:,2])/2])
    tree = cKDTree(samples)
    references, distances, report = {}, [], {}
    objects = scene['world_model']['coll_objs']
    for name, obj in objects['cube'].items():
        if name == 'ground':
            continue
        if not np.allclose(obj['pose'][3:], [0, 0, 0, 1]):
            raise ValueError('Evaluator currently requires axis-aligned scene walls')
        faces = box_reference(obj['pose'][:3], obj['dims'])
        references[name] = np.concatenate(list(faces.values()))
        distance = abs(box_distance(vertices, obj['pose'][:3], obj['dims']))
        distances.append(distance)
        coverage = {face: float(np.mean(tree.query(p)[0] <= tolerance)) for face,p in faces.items()}
        measured = (distance < tolerance) & (vertices[:,2] > .03)
        report[name] = dict(face_coverage=coverage, median_rgb=(np.median(colors[measured],axis=0).tolist() if measured.any() else [0., 0., 0.]),
                            measured_vertices=int(measured.sum()))
    for name, obj in objects['sphere'].items():
        n = 10000
        z = 1-2*(np.arange(n)+.5)/n
        theta = np.arange(n)*np.pi*(3-np.sqrt(5))
        unit = np.column_stack([np.sqrt(1-z*z)*np.cos(theta), np.sqrt(1-z*z)*np.sin(theta), z])
        ref = unit*obj['radius']+obj['position']
        references[name] = ref[ref[:,2] >= .01]
        distance = abs(np.linalg.norm(vertices-obj['position'],axis=1)-obj['radius'])
        distances.append(distance)
        measured = (distance < tolerance) & (vertices[:,2] > .025)
        pts = vertices[measured]
        if len(pts) < 4:
            report[name] = dict(measured_vertices=len(pts), median_rgb=[0.,0.,0.], fitted_center=None, fitted_radius_m=None)
            continue
        solution = np.linalg.lstsq(np.column_stack([2*pts, np.ones(len(pts))]), np.sum(pts*pts,axis=1),rcond=None)[0]
        center = solution[:3];radius=np.sqrt(solution[3]+center@center)
        report[name] = dict(measured_vertices=int(measured.sum()), median_rgb=(np.median(colors[measured],axis=0).tolist() if measured.any() else [0., 0., 0.]),
                            fitted_center=center.tolist(), fitted_radius_m=float(radius))
    for name, ref in references.items():
        error = tree.query(ref)[0]
        report[name].update(coverage=float(np.mean(error <= tolerance)),
                            reference_distance_p95_m=float(np.percentile(error,95)))
    nearest = np.min(np.column_stack(distances), axis=1)
    nonground = vertices[:,2] > .035
    precision = nearest[nonground]
    report.update(vertices=len(vertices), triangles=len(triangles), tolerance_m=tolerance,
                  object_distance_median_m=float(np.median(precision)),
                  object_distance_p95_m=float(np.percentile(precision,95)),
                  object_outlier_fraction=float(np.mean(precision > tolerance)))
    # A visible-surface criterion, never claiming hidden/buried surfaces have been observed.
    report['pass'] = bool(all(report[n]['coverage'] >= .90 for n in references)
                          and report['object_distance_p95_m'] <= tolerance
                          and report['sphere1']['median_rgb'][0] > 1.5*report['sphere1']['median_rgb'][2]
                          and all(report[n]['median_rgb'][2] > 1.2*report[n]['median_rgb'][0] for n in objects['cube'] if n!='ground'))
    return report, (vertices, triangles, colors), references


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh', type=Path, default=Path.home()/'nvblox_output/ur7e_reconstruction/base_link_surface.ply')
    parser.add_argument('--scene', type=Path, default=Path(get_package_share_directory('ur7e_d435_gazebo'))/'config/collision_world_gazebo_tall.yml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    scene = yaml.safe_load(args.scene.read_text())
    report, mesh, references = evaluate(args.mesh, scene)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        fig = plt.figure(figsize=(14, 10))
        v,t,c = mesh
        for i, azimuth in enumerate([-55, 125]):
            for j in range(2):
                ax = fig.add_subplot(2,2,2*i+j+1,projection='3d')
                if j == 0:
                    for name, obj in scene['world_model']['coll_objs']['cube'].items():
                        if name == 'ground':
                            continue
                        lo = np.array(obj['pose'][:3])-np.array(obj['dims'])/2
                        hi = np.array(obj['pose'][:3])+np.array(obj['dims'])/2
                        lo[2] = max(0., lo[2])
                        faces = []
                        for axis in range(3):
                            other = [a for a in range(3) if a != axis]
                            for side in range(2):
                                points = []
                                for a,b in [(0,0),(0,1),(1,1),(1,0)]:
                                    point = lo.copy()
                                    point[axis] = (lo if side==0 else hi)[axis]
                                    point[other[0]] = (lo if a==0 else hi)[other[0]]
                                    point[other[1]] = (lo if b==0 else hi)[other[1]]
                                    points.append(point)
                                faces.append(points)
                        ax.add_collection3d(Poly3DCollection(faces,facecolors='#8080cc',edgecolors='#555588',linewidths=.5))
                    for obj in scene['world_model']['coll_objs']['sphere'].values():
                        u,w = np.meshgrid(np.linspace(0,2*np.pi,60), np.linspace(0,np.pi,40))
                        r = obj['radius'];x,y,z = obj['position']
                        ax.plot_surface(x+r*np.cos(u)*np.sin(w),y+r*np.sin(u)*np.sin(w),z+r*np.cos(w),
                                        color='#cc3333',linewidth=0,antialiased=True)
                else:
                    ax.add_collection3d(Poly3DCollection(v[t], facecolors=c[t].mean(axis=1)/255, linewidths=0, rasterized=True))
                ax.set(xlim=(.1,.8),ylim=(-.65,.65),zlim=(0,.6),xlabel='base X (m)',ylabel='base Y (m)',zlabel='Z (m)',
                       title='Gazebo reference geometry' if j == 0 else 'Measured RGB-D reconstruction')
                ax.set_box_aspect((.7,1.3,.6));ax.view_init(28,azimuth)
        fig.tight_layout();fig.savefig(args.plot,dpi=170);plt.close(fig)
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
