#!/usr/bin/env python3
"""Read-only benchmark by default; --exercise-simulation explicitly permits motion/reset.
Run in the same isolated ROS_DOMAIN_ID as Gazebo. Never use with real hardware.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger, Empty
from tf2_ros import Buffer, TransformListener, TransformException


class Probe(Node):
    def __init__(self):
        super().__init__('gazebo_mapping_verification')
        self.stats = None
        self.cloud = None
        self.observed = 0
        self.poses = []
        self.errors = []
        self.pending = []
        self.joints = None
        self.samples = []
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/realtime_nvblox/stats', self.stats_cb, 10)
        self.create_subscription(PointCloud2, '/realtime_nvblox/esdf_3d', self.cloud_cb, 1)
        self.create_subscription(PointCloud2, '/realtime_nvblox/esdf_observed', self.observed_cb, best)
        self.create_subscription(PoseStamped, '/realtime_nvblox/camera_pose', self.pose_cb, best)
        self.create_subscription(JointState, '/joint_states', self.joints_cb, best)
        self.command = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)

    def stats_cb(self, msg):
        self.stats = json.loads(msg.data)
        self.samples.append(dict(wall=time.monotonic(), **self.stats))

    def cloud_cb(self, msg):
        assert msg.header.frame_id == 'base_link'
        assert msg.point_step == 20
        assert [f.name for f in msg.fields] == ['x', 'y', 'z', 'rgb', 'distance']
        self.cloud = np.frombuffer(msg.data, dtype='<f4').reshape(-1, 5).copy()

    def observed_cb(self, msg):
        self.observed = msg.width
        data = np.frombuffer(msg.data, dtype='<f4').reshape(-1, 5)
        assert np.all(np.isfinite(data[:, 4])) and np.all(np.abs(data[:, 4]) <= .500001)

    def joints_cb(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def pose_cb(self, msg):
        self.pending.append(msg)

    def spin(self, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)
            pending, self.pending = self.pending, []
            for msg in pending:
                try:
                    tf = self.buffer.lookup_transform('base_link', 'camera_depth_optical_frame', Time.from_msg(msg.header.stamp))
                except TransformException:
                    if len(self.pending) < 100:
                        self.pending.append(msg)
                    continue
                p, q = msg.pose.position, msg.pose.orientation
                t, r = tf.transform.translation, tf.transform.rotation
                error = np.linalg.norm(np.array([p.x-t.x, p.y-t.y, p.z-t.z]))
                qa, qb = np.array([q.x,q.y,q.z,q.w]), np.array([r.x,r.y,r.z,r.w])
                self.errors.append([float(error), float(min(np.linalg.norm(qa-qb), np.linalg.norm(qa+qb)))])
                self.poses.append([p.x,p.y,p.z])
            if self.stats and self.stats.get('error'):
                raise RuntimeError(self.stats['error'])

    def service(self, name, kind=Trigger):
        client = self.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Unavailable service: '+name)
        future = client.call_async(kind.Request())
        end = time.monotonic()+30
        while not future.done() and time.monotonic() < end:
            self.spin(.05)
        if not future.done():
            raise TimeoutError(name)
        response = future.result()
        self.destroy_client(client)
        if hasattr(response, 'success'):
            assert response.success, response.message
            return response.message
        return 'completed'

    def wait_for_gazebo(self):
        end = time.monotonic()+15
        while time.monotonic() < end:
            self.spin(.2)
            services = dict(self.get_service_names_and_types())
            if '/pause_physics' in services and '/reset_simulation' in services and self.joints:
                return
        raise RuntimeError(
            'Gazebo services/joint_states not discovered within 15 s. '
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}; "
            'start scene and mapper in the SAME domain; check gzserver exit errors. '
            f'Available services: {sorted(services)}')

    def wait_ready(self, epoch=None):
        end = time.monotonic()+60
        while time.monotonic() < end:
            self.spin(.2)
            if self.stats and self.stats['total_cells'] > 0 and self.stats['known_cells'] > 0 and (epoch is None or self.stats['map_epoch'] >= epoch):
                return
        raise TimeoutError('No populated ESDF in expected epoch')


def rate_window(a, b):
    dt = b['wall']-a['wall']
    ds = (b['sim_time_ns']-a['sim_time_ns'])/1e9
    return dict(wall_seconds=dt, simulation_seconds=ds, real_time_factor=ds/dt,
                wall_hz={k:(b['totals'].get(k,0)-a['totals'].get(k,0))/dt for k in ('depth_received','depth_integrated','esdf_published')},
                latency=b['latency'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exercise-simulation', action='store_true')
    parser.add_argument('--prepare-view', action='store_true', help='Only move the simulated wrist to view the obstacles, then exit')
    parser.add_argument('--reset-simulation', action='store_true', help='Also test Gazebo global clock reset (some Classic control plugins stop joint states)')
    parser.add_argument('--view', choices=('walls', 'sphere'), default='walls', help='Gazebo observation pose')
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', default='artifacts/gazebo_perception/verification.json')
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    report = {}
    initial = None
    try:
        if args.exercise_simulation or args.prepare_view:
            node.wait_for_gazebo()
            names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
            assert node.joints is not None
            original = np.array([node.joints[n] for n in names])
            view = np.array([0., -1.57, 1.1, -.90879335, -3.21580312, -.69520875])
            if args.view == 'sphere':
                view[0] = 1.0
            start = time.monotonic()
            while time.monotonic()-start < 5:
                u = min(1., (time.monotonic()-start)/4.)
                u = u*u*(3-2*u)
                node.command.publish(Float64MultiArray(data=((1-u)*original+u*view).tolist()))
                node.spin(.05)
            report['view_joint_positions_rad'] = view.tolist()
            report['view_reset'] = node.service('/realtime_nvblox/reset_map')
            if args.prepare_view:
                node.wait_ready()
                report['passed'] = True
                return
        node.wait_ready()
        node.spin(2)
        a = node.samples[-1]
        node.spin(args.seconds)
        report['stationary'] = rate_window(a, node.samples[-1])
        baseline = node.cloud.copy()
        report['grid_cells'] = len(baseline)
        report['observed_cells'] = node.observed
        assert node.observed > 0, 'Camera is not viewing obstacles in the query grid; run --prepare-view first'
        report['unknown_cells'] = int(len(baseline)-node.stats['known_cells'])
        if args.exercise_simulation:
            # Both Gazebo-specific services must exist before any simulated motion.
            services = dict(node.get_service_names_and_types())
            assert '/gazebo/get_entity_state' in services or '/get_entity_state' in services or '/reset_simulation' in services
            assert '/pause_physics' in services and '/reset_simulation' in services
            names = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
            assert node.joints is not None
            initial = [node.joints[n] for n in names]
            a = node.samples[-1]
            start = time.monotonic()
            pose_start = len(node.poses)
            try:
                while time.monotonic()-start < args.seconds:
                    elapsed = time.monotonic()-start
                    positions = initial.copy()
                    positions[0] += .08*math.sin(2*math.pi*elapsed/args.seconds)
                    node.command.publish(Float64MultiArray(data=positions))
                    node.spin(.1)
            finally:
                node.command.publish(Float64MultiArray(data=initial))
            node.spin(2)
            report['moving'] = rate_window(a,node.samples[-1])
            poses = np.asarray(node.poses[pose_start:])
            report['camera_motion_extent_m'] = float(np.linalg.norm(np.ptp(poses, axis=0)))
            current = node.cloud
            # Sparse block ordering/counts change as new regions are observed.
            # Match world voxel coordinates rather than comparing row indices.
            def keys(points):
                return np.ascontiguousarray(np.rint(points[:, :3]*1e5).astype(np.int64)).view(
                    np.dtype((np.void, 24))).ravel()
            _, before_indices, after_indices = np.intersect1d(
                keys(baseline), keys(current), return_indices=True)
            baseline, current = baseline[before_indices], current[after_indices]
            common = (np.abs(baseline[:,4]) < .15) & (np.abs(current[:,4]) < .5)
            delta = np.abs(baseline[common,4]-current[common,4])
            assert len(delta) > 20, 'Insufficient common surface cells for stability measurement'
            report['static_surface_consistency'] = dict(common_cells=int(common.sum()), mean_change_m=float(delta.mean()), p95_change_m=float(np.percentile(delta,95)), max_change_m=float(delta.max()))
            # Separate fixed-world regions; do not mistake changes in visible
            # point count/centroid for movement of an obstacle.
            xyz = baseline[:, :3]
            regions = {
                'ground': np.abs(xyz[:, 2]) <= .05,
                'wall_positive_y': np.all(np.abs(xyz-np.array([.4,.2,.2])) <= [.2,.1,.35], axis=1),
                'wall_negative_y': np.all(np.abs(xyz-np.array([.4,-.3,.2])) <= [.2,.1,.35], axis=1),
                'sphere': np.linalg.norm(xyz-np.array([.4,.4,.1]),axis=1) <= .15,
            }
            report['static_regions'] = {}
            for name, region in regions.items():
                differences = np.abs(baseline[common & region,4]-current[common & region,4])
                report['static_regions'][name] = dict(common_cells=len(differences),
                    p95_change_m=float(np.percentile(differences,95)) if len(differences) else None)
            assert report['camera_motion_extent_m'] > .005
            assert len(delta) > 20 and np.percentile(delta,95) < .06
            report['saved_map'] = node.service('/realtime_nvblox/save_map')
            path = Path(report['saved_map'])
            report['saved_map_bytes'] = path.stat().st_size
            assert report['saved_map_bytes'] > 0
            epoch = node.stats['map_epoch']
            report['reset_map_response'] = node.service('/realtime_nvblox/reset_map')
            node.wait_ready(epoch+1)
            report['reset_service_epoch'] = node.stats['map_epoch']
            if args.reset_simulation:
                epoch = node.stats['map_epoch']
                node.service('/reset_simulation', Empty)
                node.wait_ready(epoch+1)
                report['clock_reset_epoch'] = node.stats['map_epoch']
        assert len(node.errors) > 10
        report['pose_tf_samples'] = len(node.errors)
        report['max_pose_tf_errors'] = np.max(node.errors,axis=0).tolist()
        assert max(report['max_pose_tf_errors']) < 1e-6
        report['final_stats'] = node.stats
        report['passed'] = True
    except Exception as error:
        report['passed'] = False
        report['error'] = repr(error)
        raise
    finally:
        if initial is not None:
            node.command.publish(Float64MultiArray(data=initial))
            node.spin(.2)
        path = Path(args.output)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
