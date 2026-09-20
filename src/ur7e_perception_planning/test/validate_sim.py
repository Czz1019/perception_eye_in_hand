"""Gazebo integration exercise. Publishes goals only with --exercise.

The YAML obstacles below are a measurement oracle in this test, not planner input.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import yaml
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory
from ur7e_perception_planning.msg import PlanningMap
from ur7e_perception_planning_core.geometry import RobotGeometry, JOINTS
from ur7e_perception_planning_core.grid import map_arrays


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exercise', action='store_true')
    parser.add_argument('--faults', action='store_true')
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=60.)
    args = parser.parse_args()
    if args.faults and not args.exercise:
        parser.error('--faults requires --exercise because fault injection commands robot motion')
    rclpy.init()
    node = rclpy.create_node('validate_perception_planning', parameter_overrides=[Parameter('use_sim_time', value=True)])
    tf = Buffer(node=node); listener = TransformListener(tf, node)
    geometry = RobotGeometry()
    oracle = yaml.safe_load((Path(get_package_share_directory('ur7e_sage_planner'))/'config/collision_world_gazebo_tall.yml').read_text())['world_model']['coll_objs']
    best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    samples, maps, clearances, positions, tf_errors = [], [], [], [], []
    latest = {}
    phases = []
    expected_goal = None
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)

    def state_cb(msg):
        q = np.array([msg.position[msg.name.index(j)] for j in JOINTS]); c = geometry.spheres(q)
        d = []
        for box in oracle['cube'].values():
            delta = np.abs(c-np.array(box['pose'][:3]))-np.array(box['dims'])/2
            d.extend((np.linalg.norm(np.maximum(delta, 0), axis=1)+np.minimum(delta.max(axis=1), 0)-geometry.radii).tolist())
        for ball in oracle['sphere'].values():
            d.extend((np.linalg.norm(c-ball['position'], axis=1)-geometry.radii-ball['radius']).tolist())
        clearances.append(min(d)); positions.append(q.tolist())
        try:
            pose = tf.lookup_transform('base_link', 'tool0', Time.from_msg(msg.header.stamp)).transform.translation
            tf_errors.append(float(np.linalg.norm(geometry.fk(q)['tool0'][:3, 3]-[pose.x, pose.y, pose.z])))
        except TransformException:
            pass

    def status_cb(msg):
        record = json.loads(msg.data); latest.clear(); latest.update(record)
        samples.append(dict(wall=time.monotonic(), **record))

    def map_cb(msg):
        maps.append((time.monotonic(), msg.sequence, msg.epoch,
                     msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9))
        if len(maps) % 10 == 1:
            raw, planning, source = map_arrays(msg)
            np.savez_compressed(output.with_suffix('.map.npz'), raw=raw,
                                distance=planning, source=source,
                                origin=[msg.origin.x, msg.origin.y, msg.origin.z], dims=msg.dimensions, res=msg.resolution)

    node.create_subscription(JointState, '/joint_states', state_cb, best)
    node.create_subscription(String, '/perception_planning/status', status_cb, best)
    node.create_subscription(PlanningMap, '/perception_planning/map', map_cb, 1)
    pub = node.create_publisher(PoseStamped, '/target_pose', 1)

    def observe(seconds, until_goal=False):
        start = time.monotonic(); streak = 0; last_sample = -1
        while time.monotonic()-start < seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            if len(samples) != last_sample:
                last_sample = len(samples)
                matching = expected_goal is None or np.allclose(latest.get('goal', [999]*3), expected_goal, atol=1e-6)
                if until_goal and matching and latest.get('status') == 'GOAL_REACHED' and latest.get('goal_error', 1) < .02:
                    streak += 1
                    if streak >= 3: break
                else:
                    streak = 0
        return dict(duration=time.monotonic()-start, latest=dict(latest))

    def goal(xyz):
        nonlocal expected_goal
        expected_goal = xyz
        msg = PoseStamped(); msg.header.frame_id = 'base_link'; msg.pose.orientation.w = 1.
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, xyz)
        for _ in range(3):
            pub.publish(msg)
            observe(.2)

    # Cold CUDA initialization and Gazebo spawning are startup, not goal-motion
    # time. Wait for fresh input before timing/reissuing the commissioned goal.
    ready_start = time.monotonic()
    while time.monotonic()-ready_start < 120:
        rclpy.spin_once(node, timeout_sec=.1)
        if (latest.get('map_sequence') is not None and positions
                and latest.get('status') not in ('STALE_MAP', 'WAITING_FOR_MAP', 'STALE_JOINT_STATE')
                and latest.get('error') is None):
            break
    if args.exercise:
        initial_file = Path(get_package_share_directory('ur7e_reconstruction'))/'config/high_initial_positions.yaml'
        initial = np.array(list(yaml.safe_load(initial_file.read_text()).values()))
        goal(geometry.fk(initial)['tool0'][:3, 3]+[-.02, -.03, 0.])
    phases.append(dict(name='default_goal', **observe(args.duration, until_goal=True)))
    if args.exercise:
        goal([.22, .15, .86])
        phases.append(dict(name='second_goal', **observe(args.duration, until_goal=True)))
        goal([.4, .2, .3])
        phases.append(dict(name='inside_wall_goal', **observe(20)))
        goal([.22, .15, .86])
        phases.append(dict(name='return_goal', **observe(args.duration, until_goal=True)))
    faults = {}
    if args.faults:
        def call(service_type, name, request):
            client = node.create_client(service_type, name)
            if not client.wait_for_service(timeout_sec=5): raise RuntimeError(name+' unavailable')
            future = client.call_async(request)
            start = time.monotonic()
            while not future.done() and time.monotonic()-start < 8: rclpy.spin_once(node, timeout_sec=.1)
            if not future.done() or not future.result().success: raise RuntimeError('Service failed: '+name)
            node.destroy_client(client)
        # Inject loss of perception during motion, not only after a reached goal.
        before_motion = np.array(positions[-1])
        goal(geometry.fk(initial)['tool0'][:3, 3]+[-.02, -.03, 0.])
        moving_start = time.monotonic()
        while time.monotonic()-moving_start < 30:
            observe(.1)
            if (np.max(np.abs(np.array(positions[-1])-before_motion)) > .001
                    and latest.get('status') == 'PLANNING'):
                break
        faults['pause_during_motion'] = bool(latest.get('status') == 'PLANNING'
            and np.max(np.abs(np.array(positions[-1])-before_motion)) > .001)
        call(SetBool, '/perception_planning/mapping_enabled', SetBool.Request(data=False))
        observe(4)
        start_index = len(positions); observe(2)
        faults['map_pause_status'] = latest.get('status')
        faults['map_pause_joint_motion'] = float(np.max(np.ptp(np.asarray(positions[start_index:]), axis=0)))
        seq = latest.get('map_sequence', 0)
        call(SetBool, '/perception_planning/mapping_enabled', SetBool.Request(data=True))
        resumed = observe(60, until_goal=True)
        faults['resume_duration'] = resumed['duration']
        faults['map_resumed'] = latest.get('map_sequence', 0) > seq
        epoch = latest.get('map_epoch')
        call(Trigger, '/ur7e_nvblox/reset_map', Trigger.Request())
        reset_start = time.monotonic()
        streak = 0
        while time.monotonic()-reset_start < 60:
            before = len(samples)
            observe(.1)
            if len(samples) == before:
                continue
            ready = (latest.get('map_epoch') != epoch and latest.get('map_epoch') is not None
                     and latest.get('status') in ('PLANNING', 'GOAL_REACHED', 'BLOCKED'))
            streak = streak+1 if ready else 0
            if streak >= 3:
                break
        faults['reset_recovery_duration'] = time.monotonic()-reset_start
        faults['reset_epoch_changed'] = latest.get('map_epoch') != epoch
        faults['post_reset_status'] = latest.get('status')
    q = np.asarray(positions)
    intervals = np.diff([item[0] for item in maps])
    result = dict(phases=phases, faults=faults, status_samples=samples, map_samples=maps,
                  map_rate_wall=(len(maps)-1)/(maps[-1][0]-maps[0][0]) if len(maps)>1 else 0,
                  map_interval_p95=float(np.percentile(intervals, 95)) if len(intervals) else None,
                  min_ground_truth_clearance=min(clearances) if clearances else None,
                  max_fk_tf_error=max(tf_errors) if tf_errors else None,
                  joint_motion_l2=float(np.linalg.norm(np.ptp(q, axis=0))) if q.size else 0)
    checks = dict(runtime_no_errors=bool(samples) and not any(s.get('error') for s in samples),
                  maps_updated=len(maps)>5, robot_moved=result['joint_motion_l2']>.02,
                  fk_matches_tf=bool(tf_errors) and max(tf_errors)<1e-4,
                  geometry_clear=bool(clearances) and min(clearances)>.03)
    for phase in phases:
        record = phase['latest']
        if phase['name'] == 'inside_wall_goal':
            checks[phase['name']] = record.get('goal_error', 0)>.1 and record.get('status') == 'BLOCKED'
        else:
            checks[phase['name']] = record.get('status') == 'GOAL_REACHED' and record.get('goal_error', 1)<.02
    if args.faults:
        checks.update(map_pause_during_motion=faults['pause_during_motion'],
                      map_pause_holds=faults['map_pause_status']=='STALE_MAP' and faults['map_pause_joint_motion']<1e-4,
                      map_resumes=faults['map_resumed'], map_reset=faults['reset_epoch_changed'] and
                      faults['post_reset_status'] in ('PLANNING','GOAL_REACHED','BLOCKED'))
    result['checks'] = checks
    result['passed'] = all(checks.values())
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('status_samples','map_samples')}, indent=2), flush=True)
    node.destroy_node(); rclpy.shutdown()
    if not result['passed']: raise SystemExit(1)


if __name__ == '__main__':
    main()
