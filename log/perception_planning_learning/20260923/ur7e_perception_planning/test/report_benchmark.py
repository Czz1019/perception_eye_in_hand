"""Offline evidence summary and trajectory plot from benchmark feedback logs."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from ur7e_perception_planning_core.geometry import RobotGeometry
from run_benchmark import wall_detour_check


def summarize(data):
    samples = data['statuses']
    durations = {}
    for a, b in zip(samples, samples[1:]):
        key = a['status']
        durations[key] = durations.get(key, 0.) + b['wall']-a['wall']
    ages = [s['map_age_sim'] for s in samples if s.get('map_age_sim') is not None]
    plans = list({s['plans']: s['plan_ms'] for s in samples if s.get('plan_ms', 0) > 0}.values())
    stats = list({s['latest']['sequence']: s['latest'] for s in data['mapper_stats']
                  if s.get('latest', {}).get('receive_to_result_ms') is not None}.values())
    fields = {}
    for key in ['receive_to_result_ms', 'gpu_wait_ms', 'fusion_query_ms', 'free_field_ms']:
        values = [s[key] for s in stats]
        fields[key] = np.percentile(values, [50, 95, 100]).tolist() if values else []
    return dict(passed=data['passed'], seconds=data['duration'],
                goal_error_m=data['latest']['goal_error'], completed_views=data['latest']['completed_views'],
                true_sphere_clearance_m=data['min_geometry_clearance'], self_clearance_m=data['min_self_clearance'],
                map_rate_hz=data['map_rate'], status_seconds=durations,
                map_age_p50_p95_max=np.percentile(ages, [50, 95, 100]).tolist() if ages else [],
                plan_ms_p50_p95_max=np.percentile(plans, [50, 95, 100]).tolist() if plans else [],
                mapper_timing_p50_p95_max=fields)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    data = {name: json.loads((args.directory/f'{name}.json').read_text()) for name in ['baseline', 'wall']}
    result = {name: summarize(d) for name, d in data.items()}
    result['wall']['detour'] = wall_detour_check(data['wall'], data['wall']['latest']['goal'])
    (args.directory/'analysis.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        robot = RobotGeometry()
        fixture = yaml.safe_load((Path(get_package_share_directory('ur7e_sage_planner'))/
                                 'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
        fig = plt.figure(figsize=(10, 7)); ax = fig.add_subplot(111, projection='3d')
        for name, box in fixture['cube'].items():
            if name == 'ground':
                continue
            center, half = np.asarray(box['pose'][:3]), np.asarray(box['dims'])/2
            vertices = np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])*half+center
            faces = [[vertices[i] for i in ids] for ids in [[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]]]
            ax.add_collection3d(Poly3DCollection(faces, alpha=.22, facecolor='gray', edgecolor='gray'))
        for ball in fixture['sphere'].values():
            u,v=np.meshgrid(np.linspace(0,2*np.pi,24),np.linspace(0,np.pi,16))
            x,y,z=ball['position'];r=ball['radius']
            ax.plot_surface(x+r*np.cos(u)*np.sin(v),y+r*np.sin(u)*np.sin(v),z+r*np.cos(v),alpha=.5,color='orange')
        for name, label, color in [('baseline','Initial observation and first goal','tab:blue'),('wall','Retarget: observe and go around wall','tab:green')]:
            xyz=np.array([robot.fk(np.array(q))['tool0'][:3,3] for q in data[name]['joint_states']])
            ax.plot(*xyz.T, color=color, label=label)
            ax.scatter(*xyz[-1], color=color, s=40)
            if name=='wall':
                ax.plot(*np.array([xyz[0],data[name]['latest']['goal']]).T,'r--',label='Blocked straight line')
        ax.set(xlabel='base_link X (m)',ylabel='Y (m)',zlabel='Z (m)',xlim=(-.1,.8),ylim=(-.6,.6),zlim=(0,1.2))
        ax.view_init(elev=24, azim=-55);ax.legend(loc='upper left',fontsize=8)
        ax.set_title('Measured tool0 trajectory; walls shown only for independent validation')
        fig.tight_layout();fig.savefig(args.plot,dpi=150);plt.close(fig)


if __name__ == '__main__':
    main()
