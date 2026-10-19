"""Read-only integration check against a running scene and mapper."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ur7e_nvblox.depth import known_distances, transform_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--best-effort', action='store_true', help='For full_grid_reliable=false')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_ur7e_esdf', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=10), node=node)
    listener = TransformListener(buffer, node)
    state = {'clouds': [], 'stats': None, 'pose_errors': [], 'known_cells': [], 'last_stamp': -1}
    pending = []

    def cloud(msg):
        assert msg.header.frame_id == 'base_link', msg.header.frame_id
        fields = {f.name: f.offset for f in msg.fields}
        assert {'x', 'y', 'z', 'distance'} <= fields.keys(), fields
        assert msg.width > 0 and msg.height == 1 and len(msg.data) == msg.row_step
        dtype = np.dtype({'names': ['x', 'y', 'z', 'distance'],
                          'formats': ['<f4']*4,
                          'offsets': [fields[k] for k in ('x', 'y', 'z', 'distance')],
                          'itemsize': msg.point_step})
        values = np.frombuffer(msg.data, dtype=dtype)
        d = values['distance']
        assert np.all(np.isfinite(values['x'])) and np.all(np.isfinite(values['z']))
        stats = state['stats']
        if not stats or not stats.get('latest'):
            return
        unknown = stats['latest']['unknown_distance_m']
        valid = known_distances(d, unknown)
        assert valid.sum() > 100, f'Only {valid.sum()} observed grid cells'
        assert np.any(d[valid] > 0), 'No free-space distance samples'
        assert np.any(np.abs(d[valid]) < 0.06), 'No samples near a reconstructed surface'
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        assert ns > state['last_stamp'], 'Duplicate/nonmonotonic output timestamp'
        state['last_stamp'] = ns
        state['clouds'].append((time.monotonic(), ns))
        state['known_cells'].append(int(valid.sum()))

    def pose(msg):
        assert msg.header.frame_id == 'base_link'
        pending.append(msg)
        del pending[:-100]

    subscriptions = [
        node.create_subscription(PointCloud2, '/ur7e_nvblox/esdf_3d', cloud,
                                 QoSProfile(depth=2, reliability=(ReliabilityPolicy.BEST_EFFORT
                                     if args.best_effort else ReliabilityPolicy.RELIABLE))),
        node.create_subscription(PoseStamped, '/ur7e_nvblox/camera_pose', pose, qos_profile_sensor_data),
        node.create_subscription(String, '/ur7e_nvblox/stats', lambda m: state.update(stats=json.loads(m.data)), 2),
    ]
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for msg in list(pending):
                try:
                    tf = buffer.lookup_transform('base_link', 'camera_depth_optical_frame', Time.from_msg(msg.header.stamp))
                except TransformException:
                    continue
                expected = transform_matrix(tf.transform)
                p, q = msg.pose.position, msg.pose.orientation
                from scipy.spatial.transform import Rotation
                actual_rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                error = max(float(np.max(np.abs(expected[:3, 3] - [p.x, p.y, p.z]))),
                            float(np.max(np.abs(expected[:3, :3] - actual_rotation))))
                assert error < 1e-5, f'Output pose disagrees with image-time TF: {error}'
                state['pose_errors'].append(error)
                pending.remove(msg)
            samples = state['clouds']
            if len(samples) >= 10 and samples[-1][0] - samples[0][0] >= args.duration and len(state['pose_errors']) >= 5:
                break
        else:
            raise RuntimeError(f'Timed out: clouds={len(state["clouds"])}, checked poses={len(state["pose_errors"])}, stats={state["stats"]}')
        stats = state['stats']
        assert stats['error'] is None, stats
        assert '/envs/whole_control/' in stats['python_executable'], stats['python_executable']
        samples = state['clouds']
        wall_hz = (len(samples)-1) / (samples[-1][0]-samples[0][0])
        sim_hz = (len(samples)-1) / ((samples[-1][1]-samples[0][1])/1e9)
        report = {
            'result': 'PASS', 'messages': len(samples), 'wall_hz': wall_hz, 'simulation_hz': sim_hz,
            'known_cells_min': min(state['known_cells']), 'known_cells_max': max(state['known_cells']),
            'tf_poses_checked': len(state['pose_errors']), 'max_pose_matrix_error': max(state['pose_errors']),
            'stats': stats,
        }
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
