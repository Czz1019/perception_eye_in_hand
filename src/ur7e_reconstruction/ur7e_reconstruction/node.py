"""Eye-in-hand TSDF reconstruction with image-time TF and separate mesh publication."""
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import message_filters
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.ros_publishers import pose_stamped
from realtime_nvblox.stats import RuntimeStats
from ur7e_nvblox.depth import depth_meters, intrinsics_from_info, transform_matrix
from ur7e_reconstruction.color import rgb_image
from ur7e_reconstruction.surface import Surface, SurfaceMapper, save_ply, surface_cloud, surface_markers


class ReconstructionNode(Node):
    def __init__(self):
        super().__init__('ur7e_reconstruction')
        defaults = {
            'world_frame': 'base_link', 'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/depth/camera_info', 'depth_frame': 'camera_depth_optical_frame',
            'color_topic': '/camera/color/image_raw', 'color_info_topic': '/camera/color/camera_info',
            'color_frame': 'camera_color_optical_frame', 'color_pose_frame': 'camera_sim_color_optical_frame',
            'sync_slop_sec': 0.035, 'max_joint_velocity_rad_s': 0.025,
            'depth_stride': 2, 'min_depth_m': 0.2, 'max_depth_m': 3.0, 'voxel_size_m': 0.025,
            'tf_timeout_sec': 0.15, 'max_frame_age_sec': 0.5, 'mesh_hz': 1.0,
            'roi_min_m': [-1.0, -1.0, -0.1], 'roi_max_m': [1.5, 1.0, 1.5],
            'output_directory': '~/nvblox_output/ur7e_reconstruction',
        }
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        c = self.cfg
        for k in ('max_depth_m', 'voxel_size_m', 'tf_timeout_sec', 'max_frame_age_sec', 'mesh_hz'):
            if not np.isfinite(c[k]) or c[k] <= 0:
                raise ValueError(f'{k} must be finite and positive')
        if c['depth_stride'] < 1 or not 0 <= c['min_depth_m'] < c['max_depth_m']:
            raise ValueError('Invalid depth stride/range')
        if any(len(c[k]) != 3 or not np.isfinite(c[k]).all() for k in ('roi_min_m', 'roi_max_m')):
            raise ValueError('ROI requires three finite coordinates')
        if np.any(np.asarray(c['roi_min_m']) >= c['roi_max_m']):
            raise ValueError('ROI minimum must be smaller than maximum')
        if not c['world_frame'] or not c['depth_frame']:
            raise ValueError('Frame names must not be empty')
        self.world = c['world_frame']
        self.stats = RuntimeStats()
        self.frames, self.results = LatestSlot(), LatestSlot()
        self.info = self.color_info = None
        self.joint_stamp = -1
        self.joint_sample = None
        self.joint_speed = float("inf")
        self._state = threading.Lock()
        self._epoch, self._last_input = 0, -1
        self._last_clock = None
        self._last_result_sequence = 0
        self._summary = {}
        self._surface = None
        self._error = None
        self._integration_enabled = True
        self._stop = threading.Event()
        self.tf = Buffer(cache_time=Duration(seconds=10), node=self)
        self.listener = TransformListener(self.tf, self)
        input_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # A new RViz subscriber receives the most recent complete mesh, even while paused.
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CameraInfo, c['camera_info_topic'], self._on_info, input_qos)
        self.create_subscription(CameraInfo, c['color_info_topic'], self._on_color_info, input_qos)
        self.create_subscription(JointState, '/joint_states', self._on_joints, input_qos)
        self.depth_sub = message_filters.Subscriber(self, Image, c['depth_topic'], qos_profile=input_qos)
        self.color_sub = message_filters.Subscriber(self, Image, c['color_topic'], qos_profile=input_qos)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.color_sub], 8, c['sync_slop_sec'])
        self.synchronizer.registerCallback(self._on_depth)
        self.create_subscription(ClockMsg, '/clock', self._on_clock, input_qos)
        self.mesh_pub = self.create_publisher(MarkerArray, '~/mesh', output_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/surface_points', output_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '~/camera_pose', 5)
        self.stats_pub = self.create_publisher(String, '~/stats', 2)
        self.create_service(Trigger, '~/reset_map', self._reset)
        self.create_service(SetBool, '~/set_integration_enabled', self._set_integration)
        self.create_service(Trigger, '~/save_mesh', self._save)
        self.create_timer(1.0, self._report, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.worker = threading.Thread(target=self._mapping_loop, name='tsdf-fusion', daemon=True)
        self.publisher = threading.Thread(target=self._publish_loop, name='mesh-publisher', daemon=True)
        self.worker.start()
        self.publisher.start()
        self.get_logger().info(f'TSDF surface reconstruction: {self.world} <- {c["depth_frame"]}; '
                               f'Python={sys.executable}; no SLAM or ESDF updates.')

    def _on_info(self, msg):
        self.info = msg

    def _on_color_info(self, msg):
        self.color_info = msg

    def _on_joints(self, msg):
        self.joint_stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        # Gazebo reports nonzero constraint-solver velocities even at a held position.
        # Estimate real motion from positions over a short simulation-time window.
        sample = (self.joint_stamp, dict(zip(msg.name, msg.position)))
        if self.joint_sample is None or sample[0] < self.joint_sample[0]:
            self.joint_sample = sample
            self.joint_speed = float('inf')
        elif sample[0] - self.joint_sample[0] >= 150_000_000:
            old_stamp, old = self.joint_sample
            self.joint_speed = max((abs(v-old[n]) for n, v in sample[1].items() if n in old), default=float('inf')) / ((sample[0]-old_stamp)/1e9)
            self.joint_sample = sample

    def _on_clock(self, msg):
        ns = msg.clock.sec*1_000_000_000 + msg.clock.nanosec
        if self._last_clock is not None and ns < self._last_clock:
            self._request_reset()
            self.get_logger().warning('Simulation time reset; clearing reconstructed surface.')
        self._last_clock = ns

    def _request_reset(self):
        with self._state:
            self._epoch += 1
            self._last_input = -1
            self._surface, self._summary = None, {}
        self.stats.inc('reset_requested')

    def _on_depth(self, msg, color):
        self.stats.inc('depth_received')
        if not self._integration_enabled:
            self.stats.inc('integration_paused')
            return
        info = self.info
        if info is None or self.color_info is None:
            self.stats.inc('camera_info_missing')
            return
        ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        if ns <= 0 or abs(self.get_clock().now().nanoseconds-ns)/1e9 > self.cfg['max_frame_age_sec']:
            self.stats.inc('timestamp_rejected')
            return
        if self.cfg['max_joint_velocity_rad_s'] > 0 and (
                abs(ns - self.joint_stamp) > 500_000_000 or
                self.joint_speed > self.cfg['max_joint_velocity_rad_s']):
            self.stats.inc('moving_frames_skipped')
            return
        with self._state:
            if ns <= self._last_input:
                self.stats.inc('out_of_order_depth')
                return
            self._last_input = ns
            self.frames.put((self._epoch, msg, info, ns, color, self.color_info))

    def _mapping_loop(self):
        sequence, active_epoch, intrinsics, mapper, next_mesh = 0, -1, None, None, 0.0
        try:
            while not self._stop.is_set():
                seq, item = self.frames.wait_newer(sequence, timeout=0.2)
                if item is None:
                    continue
                self.stats.inc('depth_overwritten', max(0, seq-sequence-1))
                sequence = seq
                epoch, msg, info, ns, color, color_info = item
                if epoch != self._epoch or not self._integration_enabled:
                    continue
                if (msg.header.frame_id != self.cfg['depth_frame'] or info.header.frame_id != msg.header.frame_id
                        or (msg.width, msg.height) != (info.width, info.height)
                        or color.header.frame_id != self.cfg['color_frame']
                        or color_info.header.frame_id != color.header.frame_id
                        or (color.width, color.height) != (color_info.width, color_info.height)):
                    self.stats.inc('calibration_mismatch')
                    self.get_logger().warning('Depth frame/CameraInfo do not match configuration.', throttle_duration_sec=5)
                    continue
                try:
                    tf = self.tf.lookup_transform(self.world, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=self.cfg['tf_timeout_sec']))
                    color_tf = self.tf.lookup_transform(self.world, self.cfg['color_pose_frame'],
                            Time.from_msg(color.header.stamp), timeout=Duration(seconds=self.cfg['tf_timeout_sec']))
                except TransformException as exc:
                    self.stats.inc('tf_missing')
                    self.get_logger().warning(f'Waiting for image-time TF: {exc}', throttle_duration_sec=5)
                    continue
                try:
                    intr = intrinsics_from_info(info, self.cfg['depth_stride'])
                    depth = depth_meters(msg, self.cfg['depth_stride'], self.cfg['min_depth_m'], self.cfg['max_depth_m'])
                    pose = transform_matrix(tf.transform)
                    color_intr = intrinsics_from_info(color_info, self.cfg['depth_stride'])
                    rgb = rgb_image(color, self.cfg['depth_stride'])
                    color_pose = transform_matrix(color_tf.transform)
                except ValueError as exc:
                    self.stats.inc('invalid_depth')
                    self.get_logger().warning(str(exc), throttle_duration_sec=5)
                    continue
                if not np.any(depth):
                    self.stats.inc('empty_depth')
                    continue
                if active_epoch != epoch or intrinsics != (intr, color_intr):
                    started = time.perf_counter()
                    mapper = None
                    mapper = SurfaceMapper(intr, self.cfg, color_intr)
                    active_epoch, intrinsics, next_mesh = epoch, (intr, color_intr), 0.0
                    self.stats.inc('map_initialized')
                    self.stats.add_latency('initialization', (time.perf_counter()-started)*1000)
                    self.get_logger().info(f'Mapper ready: {intr.width}x{intr.height}, voxel={self.cfg["voxel_size_m"]} m')
                if epoch != self._epoch or abs(self.get_clock().now().nanoseconds-ns)/1e9 > self.cfg['max_frame_age_sec']:
                    self.stats.inc('stale_depth')
                    continue
                if not self._integration_enabled:
                    continue
                started = time.perf_counter()
                mapper.integrate(depth, pose, rgb, color_pose)
                self.stats.inc("color_integrated")
                self.stats.inc('depth_integrated')
                self.stats.add_latency('depth_integrate_cpu', (time.perf_counter()-started)*1000)
                if time.monotonic() < next_mesh:
                    continue
                started = time.perf_counter()
                surface = mapper.extract()
                self.stats.add_latency('mesh_extract', (time.perf_counter()-started)*1000)
                self.stats.inc('mesh_extracted')
                self.results.put((epoch, ns, pose, surface))
                next_mesh = time.monotonic() + 1/self.cfg['mesh_hz']
        except Exception as exc:
            self._fail(exc)

    def _fail(self, exc):
        if rclpy.ok() and not self._stop.is_set():
            self._error = repr(exc)
            self.get_logger().error(f'Reconstruction failed: {self._error}')
            self._stop.set()

    def _publish_loop(self):
        cleared_epoch = -1
        empty = Surface(np.empty((0, 3), np.float32), np.empty((0, 3), np.int32), np.empty((0, 3), np.uint8))
        try:
            while not self._stop.is_set():
                epoch = self._epoch
                if epoch != cleared_epoch:
                    self.mesh_pub.publish(surface_markers(empty, self.world, self.get_clock().now().nanoseconds))
                    self.cloud_pub.publish(surface_cloud(empty, self.world, self.get_clock().now().nanoseconds))
                    cleared_epoch = epoch
                seq, result = self.results.wait_newer(self._last_result_sequence, timeout=0.2)
                if result is None:
                    continue
                self.stats.inc('mesh_skipped', max(0, seq-self._last_result_sequence-1))
                self._last_result_sequence = seq
                epoch, ns, pose, surface = result
                if epoch != self._epoch:
                    continue
                started = time.perf_counter()
                markers = surface_markers(surface, self.world, ns)
                cloud = surface_cloud(surface, self.world, ns)
                with self._state:
                    if epoch != self._epoch:
                        continue
                self.mesh_pub.publish(markers)
                self.cloud_pub.publish(cloud)
                self.pose_pub.publish(pose_stamped(pose, self.world, ns))
                with self._state:
                    if epoch == self._epoch:
                        self._surface = surface
                        self._summary = {
                            'map_epoch': epoch, 'input_stamp_ns': ns,
                            'vertices': len(surface.vertices), 'triangles': len(surface.triangles),
                            'camera_position_base': pose[:3, 3].tolist(),
                        }
                self.stats.add_latency('mesh_pack_publish', (time.perf_counter()-started)*1000)
                self.stats.inc('mesh_published')
        except Exception as exc:
            self._fail(exc)

    def _report(self):
        self.stats_pub.publish(String(data=json.dumps(self.stats.snapshot({
            'world_frame': self.world, 'python_executable': sys.executable,
            'esdf_enabled': False, 'color_enabled': True, 'integration_enabled': self._integration_enabled, 'error': self._error, 'latest': self._summary,
        }), allow_nan=False)))
        if self._error:
            rclpy.shutdown()

    def _set_integration(self, request, response):
        self._integration_enabled = request.data
        response.success = True
        response.message = 'Integration enabled' if request.data else 'Integration paused; existing mesh retained'
        return response

    def _reset(self, request, response):
        self._request_reset()
        response.success, response.message = True, 'Surface cleared; next valid depth frame starts a new map.'
        return response

    def _save(self, request, response):
        with self._state:
            surface = self._surface
        try:
            if surface is None or not len(surface.triangles):
                raise RuntimeError('No reconstructed surface available yet')
            path = save_ply(surface, Path(self.cfg['output_directory']).expanduser() / 'base_link_surface.ply')
            response.success, response.message = True, str(path)
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def close(self):
        self._stop.set()
        self.frames.close()
        self.results.close()
        self.worker.join()
        self.publisher.join()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = ReconstructionNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node is not None and node._error:
        raise RuntimeError(node._error)


if __name__ == '__main__':
    main()
