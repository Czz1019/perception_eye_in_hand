"""Own a cold simulation, test default reach and a wall-crossing retarget, clean up."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from ur7e_perception_planning_core.geometry import RobotGeometry
import rclpy
from geometry_msgs.msg import PoseStamped


def wall_detour_check(data, goal):
    """Fixture truth is confined to validation, never supplied to the planner."""
    geometry = RobotGeometry()
    tcp = np.asarray([geometry.fk(np.asarray(q))['tool0'][:3, 3] for q in data['joint_states']])
    if not len(tcp):
        return dict(passed=False, reason='No actual joint samples')
    fixture = yaml.safe_load((Path(get_package_share_directory('ur7e_sage_planner'))/
                             'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
    straight = np.linspace(tcp[0], goal, 1001)
    crossed, actual_hits = [], []
    for name, box in fixture['cube'].items():
        center, half = np.asarray(box['pose'][:3]), np.asarray(box['dims'])/2
        if np.any(np.all(abs(straight-center) < half, axis=1)):
            crossed.append(name)
        if np.any(np.all(abs(tcp-center) < half, axis=1)):
            actual_hits.append(name)
    # Densify actual feedback intervals and charge half-sample motion padding.
    # This checks the joint interpolation model, not a Gazebo contact sensor.
    minimum, self_minimum = np.inf, np.inf
    joints = np.asarray(data['joint_states'])
    for a, b in zip(joints, joints[1:]):
        travel = 1.4*float(np.sum(abs(b-a)))
        count = max(2, int(np.ceil(travel/.004))+1)
        centers = geometry.spheres_batch(np.linspace(a, b, count))
        padding = travel/(2*(count-1))
        for box in fixture['cube'].values():
            delta = abs(centers-np.asarray(box['pose'][:3]))-np.asarray(box['dims'])/2
            distance = np.linalg.norm(np.maximum(delta, 0), axis=-1)+np.minimum(delta.max(axis=-1), 0)
            minimum = min(minimum, float(np.min(distance-geometry.radii))-padding)
        for ball in fixture['sphere'].values():
            distance = np.linalg.norm(centers-ball['position'], axis=-1)-ball['radius']-geometry.radii
            minimum = min(minimum, float(distance.min())-padding)
        self_minimum = min(self_minimum, float(geometry.self_clearance_batch(centers).min())-2*padding)
    return dict(passed=bool(crossed) and not actual_hits and data['passed'] and minimum > .015 and self_minimum > .008,
                swept_fixture_clearance_m=float(minimum), swept_self_clearance_m=float(self_minimum),
                straight_line_intersects=crossed, actual_tcp_inside=actual_hits,
                actual_start=tcp[0].tolist(), actual_end=tcp[-1].tolist(),
                path_length_m=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).sum()),
                max_tcp_height_m=float(tcp[:, 2].max()))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tag', required=True)
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--save-maps', action='store_true')
    parser.add_argument('--repeat-wall', action='store_true', help='Return and cross again using the accumulated map')
    parser.add_argument('--duration', type=float, default=240.)
    parser.add_argument('--wall-goal', type=float, nargs=3, default=[.45, .4, .55])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    output = root/'log/perception_planning_fast_demo'/args.tag
    output.mkdir(parents=True, exist_ok=False)
    processes = []
    launch = None
    try:
        with (output/'launch.log').open('w') as log:
            command = [str(root/'run_ur7e_perception_planning.sh')]
            if args.no_rviz:
                command.append('run_rviz:=false')
            launch = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('wall_benchmark_goal_sender')
        publisher = node.create_publisher(PoseStamped, '/target_pose', 1)
        results = {}
        phases = [('baseline', [.5, 0., .45]), ('wall', args.wall_goal)]
        if args.repeat_wall:
            phases += [('return', [.5, 0., .45]), ('wall_repeat', args.wall_goal)]
        for name, goal in phases:
            with (output/f'{name}.log').open('w') as log:
                command = [sys.executable, '-B', str(Path(__file__).with_name('validate_demo.py')),
                           '--output', str(output/f'{name}.json'), '--duration', str(args.duration),
                           '--goal', *map(str, goal)]
                if args.save_maps:
                    command.append('--save-maps')
                observer = subprocess.Popen(command, cwd=root, stdout=log, stderr=subprocess.STDOUT)
                processes.append(observer)
            if name != 'baseline':
                deadline = time.monotonic()+2.
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                msg = PoseStamped(); msg.header.frame_id = 'world'
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
                msg.pose.orientation.w = 1.
                publisher.publish(msg)
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=.1)
            observer.wait(timeout=args.duration+20)
            data = json.loads((output/f'{name}.json').read_text())
            results[name] = {key: data[key] for key in ('passed', 'duration', 'latest', 'min_geometry_clearance', 'map_rate')}
            if name in ('wall', 'wall_repeat'):
                results[name]['detour'] = wall_detour_check(data, goal)
                results[name]['passed'] &= results[name]['detour']['passed']
            (output/'summary.json').write_text(json.dumps(results, indent=2))
            print(name, json.dumps(results[name]), flush=True)
            if not results[name]['passed']:
                return 1
        node.destroy_node(); rclpy.shutdown()
        return 0
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate(); process.wait(timeout=5)
        if launch is not None and launch.poll() is None:
            # Let ros2 launch forward SIGINT once; signalling its whole group
            # would interrupt Python cleanup a second time.
            launch.send_signal(signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGTERM)
                launch.wait(timeout=5)


if __name__ == '__main__':
    raise SystemExit(main())
