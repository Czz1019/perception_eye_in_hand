from __future__ import annotations

import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.ros_publishers import esdf_cloud, pose_stamped
from realtime_nvblox.stats import RuntimeStats
from ur7e_nvblox.depth import (
    depth_calibration, depth_meters, intrinsics_from_info, known_distances, transform_matrix,
)


class TfMapper(Node):
    def __init__(self):
        super().__init__('ur7e_nvblox')
        defaults = {
            'world_frame': 'base_link', 'depth_topic': '/camera/depth/image_rect_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'depth_frame': 'camera_depth_optical_frame', 'depth_stride': 2,
            'min_depth_m': 0.2, 'max_depth_m': 4.0, 'voxel_size_m': 0.03,
            'tf_timeout_sec': 0.15, 'max_frame_age_sec': 0.5, 'esdf_hz': 10.0,
            'grid_resolution_m': 0.05, 'grid_size_m': [2.0, 2.0, 1.5],
            'grid_center_m': [0.0, 0.0, 0.5], 'max_visualized_distance_m': 0.5,
            'full_grid_reliable': True,
            'output_directory': '~/nvblox_output/ur7e_sim',
        }
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self._validate_config()
        self.world = self.cfg['world_frame']
        self.grid = dict(zip(('size_x_m', 'size_y_m', 'size_z_m'), self.cfg['grid_size_m']))
        self.grid['resolution_m'] = self.cfg['grid_resolution_m']
        self.stats = RuntimeStats()
        self.frames, self.results = LatestSlot(), LatestSlot()
        self.info = None
        self._state_lock = threading.Lock()
        self._backend_lock = threading.Lock()
        self._epoch = 0
        self._last_input_ns = -1
        self._clock_ns = None
        self._backend = None
        self._backend_intrinsics = None
        self._error = None
        self._last_output_sequence = 0
        self._summary = {}
        self._stop = threading.Event()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        input_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'], self._on_info, input_qos)
        self.create_subscription(Image, self.cfg['depth_topic'], self._on_depth, input_qos)
        self.create_subscription(ClockMsg, '/clock', self._on_clock, qos)
        grid_qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE
                             if self.cfg['full_grid_reliable'] else ReliabilityPolicy.BEST_EFFORT))
        self.cloud_pub = self.create_publisher(PointCloud2, '~/esdf_3d', grid_qos)
        self.observed_pub = self.create_publisher(PointCloud2, '~/esdf_observed', qos)
        self.pose_pub = self.create_publisher(PoseStamped, '~/camera_pose', qos)
        self.stats_pub = self.create_publisher(String, '~/stats', 2)
        self.create_service(Trigger, '~/reset_map', self._reset_service)
        self.create_service(Trigger, '~/save_map', self._save_service)
        # Wall timers still report missing /clock, TF or images while simulation is paused.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self._publish_stats, clock=steady)
        self.worker = threading.Thread(target=self._worker, name='tf-depth-mapper', daemon=True)
        self.publisher_worker = threading.Thread(target=self._publisher_loop, name='esdf-publisher', daemon=True)
        self.worker.start()
        self.publisher_worker.start()
        self.get_logger().info(
            f'Depth {self.cfg["depth_topic"]}; timestamped TF {self.world} <- '
            f'{self.cfg["depth_frame"]}; Python {sys.executable}. Waiting for camera/TF.')

    def _validate_config(self):
        c = self.cfg
        for name in ('max_depth_m', 'voxel_size_m', 'tf_timeout_sec', 'max_frame_age_sec',
                     'esdf_hz', 'grid_resolution_m', 'max_visualized_distance_m'):
            if not np.isfinite(c[name]) or c[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if c['depth_stride'] < 1 or not 0 <= c['min_depth_m'] < c['max_depth_m']:
            raise ValueError('Invalid depth stride or depth limits')
        for key in ('grid_size_m', 'grid_center_m'):
            if len(c[key]) != 3 or not np.all(np.isfinite(c[key])):
                raise ValueError(f'{key} must contain three finite coordinates')
        if min(c['grid_size_m']) <= 0 or not c['world_frame'] or not c['depth_frame']:
            raise ValueError('Grid dimensions and frame names must be nonempty/positive')
        count = np.prod(np.ceil(np.asarray(c['grid_size_m']) / c['grid_resolution_m']) + 1)
        if count > 2_000_000:
            raise ValueError('Query grid exceeds 2 million samples; increase grid_resolution_m')

    def _on_info(self, msg):
        self.info = msg

    def _on_clock(self, msg):
        ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self._clock_ns is not None and ns < self._clock_ns:
            self._request_reset()
            self.get_logger().warning('Simulation time moved backwards; clearing map before next fusion.')
        self._clock_ns = ns

    def _request_reset(self):
        with self._state_lock:
            self._epoch += 1
            self._last_input_ns = -1
        self._summary = {}
        self.stats.inc('map_reset_requested')

    def _on_depth(self, msg):
        self.stats.inc('depth_received')
        info = self.info
        if info is None:
            self.stats.inc('camera_info_missing')
            return
        ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if ns <= 0:  # A zero TF timestamp means "latest", which is deliberately forbidden.
            self.stats.inc('invalid_timestamp')
            return
        # DDS may deliver images from the previous simulation epoch after /clock
        # jumps backwards. Reject these before updating the monotonic input guard.
        # A small lead is allowed because /clock and image callbacks are asynchronous.
        age = (self.get_clock().now().nanoseconds - ns) / 1e9
        if abs(age) > self.cfg['max_frame_age_sec']:
            self.stats.inc('depth_outside_clock_window')
            return
        with self._state_lock:
            if ns <= self._last_input_ns:
                self.stats.inc('out_of_order_depth')
                return
            self._last_input_ns = ns
            self.frames.put((self._epoch, msg, info, ns, time.perf_counter()))

    def _worker(self):
        sequence, active_epoch, next_esdf = 0, -1, 0.0
        try:
            while not self._stop.is_set():
                new_sequence, item = self.frames.wait_newer(sequence, timeout=0.2)
                if item is None:
                    continue
                self.stats.inc('depth_overwritten', max(0, new_sequence - sequence - 1))
                sequence = new_sequence
                epoch, msg, info, ns, received_at = item
                with self._state_lock:
                    if epoch != self._epoch:
                        continue
                if (msg.header.frame_id != self.cfg['depth_frame'] or
                        info.header.frame_id != msg.header.frame_id or
                        (msg.width, msg.height) != (info.width, info.height)):
                    self.stats.inc('calibration_mismatch')
                    self.get_logger().warning('Depth and CameraInfo dimensions/frame do not match configured depth_frame.', throttle_duration_sec=5)
                    continue
                age = (self.get_clock().now().nanoseconds - ns) / 1e9
                if abs(age) > self.cfg['max_frame_age_sec']:
                    self.stats.inc('stale_depth')
                    continue
                try:
                    tf_started = time.perf_counter()
                    tf = self.tf_buffer.lookup_transform(
                        self.world, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                        timeout=Duration(seconds=self.cfg['tf_timeout_sec']))
                    self.stats.add_latency('tf_lookup', (time.perf_counter() - tf_started) * 1000)
                except TransformException as exc:
                    self.stats.inc('tf_missing')
                    self.get_logger().warning(f'Waiting for image-time TF: {exc}', throttle_duration_sec=5)
                    continue
                try:
                    intr = intrinsics_from_info(info, self.cfg['depth_stride'])
                    depth = depth_meters(msg, self.cfg['depth_stride'], self.cfg['min_depth_m'], self.cfg['max_depth_m'])
                    pose = transform_matrix(tf.transform)
                except ValueError as exc:
                    self.stats.inc('invalid_depth')
                    self.get_logger().warning(str(exc), throttle_duration_sec=5)
                    continue
                if not np.any(depth):
                    self.stats.inc('empty_depth')
                    continue
                with self._backend_lock:
                    if active_epoch != epoch or self._backend_intrinsics != intr:
                        initialize_started = time.perf_counter()
                        # Explicitly release old map before allocating another GPU mapper.
                        self._backend = None
                        self._backend = NvbloxBackend(depth_calibration(intr), {
                            'voxel_size_m': self.cfg['voxel_size_m'],
                            'max_integration_distance_m': self.cfg['max_depth_m'],
                        })
                        self._backend.prepare_esdf_grid(self.grid)
                        self._backend_intrinsics, active_epoch = intr, epoch
                        next_esdf = 0.0
                        self.stats.inc('map_initialized')
                        self.stats.add_latency('backend_initialization', (time.perf_counter() - initialize_started) * 1000)
                        self.get_logger().info(f'Mapper ready: {intr.width}x{intr.height}, {self.cfg["voxel_size_m"]} m voxels, frame={self.world}')
                    age = (self.get_clock().now().nanoseconds - ns) / 1e9
                    with self._state_lock:
                        current_epoch = self._epoch
                    if epoch != current_epoch or abs(age) > self.cfg['max_frame_age_sec']:
                        self.stats.inc('stale_depth')
                        continue
                    backend = self._backend
                    started = time.perf_counter()
                    backend.integrate_depth(depth, pose)
                    self.stats.inc('depth_integrated')
                    if time.monotonic() < next_esdf:
                        continue
                    backend.update_esdf()
                    points, distances = backend.query_esdf_grid(self.cfg['grid_center_m'], self.grid)
                    known = known_distances(distances, backend.esdf_unknown_distance)
                    summary = {
                        'input_stamp_ns': ns, 'known_cells': int(known.sum()),
                        'total_cells': int(len(distances)), 'map_epoch': epoch,
                        'unknown_distance_m': float(backend.esdf_unknown_distance),
                        'depth_encoding': msg.encoding, 'input_frame': msg.header.frame_id,
                        'input_size': [msg.width, msg.height],
                        'integration_size': [intr.width, intr.height],
                        'camera_position_base': pose[:3, 3].tolist(),
                        'ros_clock_minus_image_sec': (self.get_clock().now().nanoseconds - ns) / 1e9,
                        'receive_to_result_ms': (time.perf_counter() - received_at) * 1000,
                    }
                self.stats.inc('esdf_updated')
                self.stats.add_latency('fusion_and_esdf', (time.perf_counter() - started) * 1000)
                self.results.put((epoch, ns, pose, points, distances, summary))
                next_esdf = time.monotonic() + 1.0 / self.cfg['esdf_hz']
        except Exception as exc:
            self._worker_error('Mapping', exc)

    def _worker_error(self, source, exc):
        if rclpy.ok() and not self._stop.is_set():
            self._error = repr(exc)
            self.get_logger().error(f'{source} worker failed: {self._error}')
            self._stop.set()

    def _publisher_loop(self):
        # Reliable publication may block on DDS. A separate latest-only worker
        # prevents it from delaying incoming /clock, images and /tf callbacks.
        try:
            while not self._stop.is_set():
                _, item = self.results.wait_newer(self._last_output_sequence, timeout=0.2)
                if item is not None:
                    self._publish_latest()
        except Exception as exc:
            self._worker_error('Publisher', exc)

    def _publish_latest(self):
        sequence, item = self.results.get_if_newer(self._last_output_sequence)
        if item is None:
            return
        self._last_output_sequence = sequence
        epoch, ns, pose, points, distances, summary = item
        with self._state_lock:
            if epoch != self._epoch:
                return
        if abs((self.get_clock().now().nanoseconds - ns) / 1e9) > self.cfg['max_frame_age_sec']:
            self.stats.inc('stale_output')
            return
        self._summary = summary
        unknown = summary['unknown_distance_m']
        max_d = self.cfg['max_visualized_distance_m']
        self.cloud_pub.publish(esdf_cloud(points, distances, self.world, ns, max_d,
                                         full_grid=True, unknown_distance_m=unknown))
        if self.observed_pub.get_subscription_count():
            self.observed_pub.publish(esdf_cloud(points, distances, self.world, ns, max_d,
                                                unknown_distance_m=unknown))
        self.pose_pub.publish(pose_stamped(pose, self.world, ns))
        self.stats.inc('esdf_published')

    def _publish_stats(self):
        report = self.stats.snapshot({
            'world_frame': self.world, 'python_executable': sys.executable,
            'error': self._error, 'latest': self._summary,
        })
        self.stats_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        if self._error:
            rclpy.shutdown()

    def _reset_service(self, request, response):
        self._request_reset()
        response.success = True
        response.message = 'Map reset requested; a fresh mapper will be created on the next valid frame.'
        return response

    def _save_service(self, request, response):
        try:
            with self._backend_lock:
                if self._backend is None:
                    raise RuntimeError('No map: waiting for valid depth and TF')
                path = Path(self.cfg['output_directory']).expanduser() / 'base_link_map.nvblox'
                self._backend.save_map(path)
            response.success, response.message = True, str(path)
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def close(self):
        self._stop.set()
        self.frames.close()
        self.results.close()
        self.worker.join()
        self.publisher_worker.join()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = TfMapper()
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
