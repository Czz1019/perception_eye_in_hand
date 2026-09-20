"""Read-only validation of a running reconstruction node."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation

from ur7e_nvblox.depth import transform_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_reconstruction', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(cache_time=Duration(seconds=15), node=node)
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    state = {'meshes': [], 'points': 0, 'stats': None, 'pose_errors': []}
    pending = []

    def on_mesh(msg):
        assert msg.markers and msg.markers[0].action == Marker.DELETEALL
        triangles = [m for m in msg.markers if m.action == Marker.ADD]
        if not triangles:
            return
        for m in triangles:
            assert m.type == Marker.TRIANGLE_LIST and m.header.frame_id == 'base_link'
            assert len(m.points) % 3 == 0 and len(m.colors) == len(m.points)
            v = np.array([[p.x, p.y, p.z] for p in m.points])
            assert np.isfinite(v).all()
            assert all(c.a > 0 for c in m.colors)
        stamp = triangles[0].header.stamp
        ns = stamp.sec*1_000_000_000 + stamp.nanosec
        assert not state['meshes'] or ns > state['meshes'][-1][1]
        state['meshes'].append((time.monotonic(), ns, sum(len(m.points)//3 for m in triangles)))

    def on_cloud(msg):
        assert msg.header.frame_id == 'base_link'
        assert {f.name for f in msg.fields} == {'x', 'y', 'z', 'rgb'}
        assert len(msg.data) == msg.width*msg.point_step
        state['points'] = max(state['points'], msg.width)

    subscriptions = [
        node.create_subscription(MarkerArray, '/ur7e_reconstruction/mesh', on_mesh, qos),
        node.create_subscription(PointCloud2, '/ur7e_reconstruction/surface_points', on_cloud, qos),
        node.create_subscription(PoseStamped, '/ur7e_reconstruction/camera_pose', pending.append, 5),
        node.create_subscription(String, '/ur7e_reconstruction/stats', lambda m: state.update(stats=json.loads(m.data)), 2),
    ]
    deadline = time.monotonic()+args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for msg in list(pending):
                assert msg.header.frame_id == 'base_link'
                try:
                    tf = buffer.lookup_transform('base_link', 'camera_depth_optical_frame', Time.from_msg(msg.header.stamp))
                except TransformException:
                    continue
                expected = transform_matrix(tf.transform)
                p, q = msg.pose.position, msg.pose.orientation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                error = max(np.abs(expected[:3, 3]-[p.x, p.y, p.z]).max(), np.abs(expected[:3, :3]-rotation).max())
                assert error < 1e-5, error
                state['pose_errors'].append(float(error))
                pending.remove(msg)
            pending[:] = pending[-100:]
            if len(state['meshes']) >= args.samples and state['points'] > 100 and len(state['pose_errors']) >= 2 and state['stats']:
                break
        else:
            raise RuntimeError(f'Insufficient valid reconstruction: {state}')
        stats = state['stats']
        assert stats['error'] is None and stats['esdf_enabled'] is False
        assert '/envs/whole_control/' in stats['python_executable']
        assert not any(name.startswith('/ur7e_reconstruction/esdf') for name, _ in node.get_topic_names_and_types())
        samples = state['meshes']
        report = {
            'result': 'PASS', 'mesh_messages': len(samples),
            'mesh_wall_hz': (len(samples)-1)/(samples[-1][0]-samples[0][0]),
            'triangles_min': min(s[2] for s in samples), 'triangles_max': max(s[2] for s in samples),
            'surface_points': state['points'], 'tf_poses_checked': len(state['pose_errors']),
            'max_pose_matrix_error': max(state['pose_errors']), 'stats': stats,
        }
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
