"""Gazebo depth + exposure-time TF -> shared NvbloxBackend (no camera SDK/SLAM)."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from realtime_nvblox.gazebo_depth import calibration_from_info, depth_meters, stamp_ns, stamp_rejection, transform_matrix
from realtime_nvblox.latest_slot import LatestSlot
from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.ros_publishers import esdf_cloud, pose_stamped
from realtime_nvblox.stats import RuntimeStats
from realtime_nvblox.reconstruction_visuals import clearance_cloud, clear_mesh, reconstruction_mesh, distance_legend


@dataclass
class Input:
    epoch: int
    msg: Image
    info: CameraInfo
    received: float


@dataclass
class Output:
    epoch: int
    stamp: int
    pose: np.ndarray
    points: np.ndarray
    distances: np.ndarray
    unknown: float
    received: float
    dense: tuple | None = None


class GazeboNvbloxNode(Node):
    def __init__(self):
        super().__init__('realtime_nvblox')
        from ament_index_python.packages import get_package_share_directory
        default = Path(get_package_share_directory('realtime_nvblox'))/'config/gazebo_robot.yaml'
        self.declare_parameter('config', str(default))
        self.cfg = yaml.safe_load(Path(self.get_parameter('config').value).expanduser().read_text())
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.sim_time = self.cfg.get('use_sim_time', True)
        if not isinstance(self.sim_time, bool):
            raise ValueError('use_sim_time must be boolean')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=self.sim_time)])
        for key in ('voxel_size_m', 'tf_timeout_sec', 'max_frame_age_sec', 'esdf_hz'):
            if not np.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(key + ' must be positive')
        if self.cfg['depth_stride'] < 1 or int(self.cfg['depth_stride']) != self.cfg['depth_stride']:
            raise ValueError('depth_stride must be a positive integer')
        self.dense_debug = self.cfg.get('publish_dense_debug', False)
        if self.dense_debug:
            self.grid = dict(zip(('size_x_m', 'size_y_m', 'size_z_m'), self.cfg['grid_size_m']))
            self.grid['resolution_m'] = self.cfg['grid_resolution_m']
            NvbloxBackend.grid_offsets(self.grid)
        self.esdf_extras = self.cfg.get('publish_esdf_extras', False)
        self.near_obstacle_band = float(self.cfg.get('near_obstacle_distance_m', .15))
        if not np.isfinite(self.near_obstacle_band) or self.near_obstacle_band <= 0:
            raise ValueError('near_obstacle_distance_m must be positive')
        self.enable_mesh = self.cfg.get('enable_mesh', True)
        self.mesh_hz = float(self.cfg.get('mesh_hz', 2.))
        self.clearance_thresholds = self.cfg.get('clearance_thresholds_m', [.05, .15, .3])
        if (len(self.clearance_thresholds) != 3 or not np.isfinite(self.clearance_thresholds).all()
                or not 0 < self.clearance_thresholds[0] < self.clearance_thresholds[1]
                < self.clearance_thresholds[2] <= self.cfg['max_visualized_distance_m']):
            raise ValueError('Require 0 < three clearance thresholds <= visualization band')
        if not np.isfinite(self.mesh_hz) or self.mesh_hz <= 0:
            raise ValueError('mesh_hz must be positive')
        self.mesh_slot = LatestSlot()
        self.state = threading.RLock()
        self.stop_event = threading.Event()
        self.depth_slot, self.result_slot = LatestSlot(), LatestSlot()
        self.taken = 0
        self.epoch = 0
        self.now_ns = self.floor_ns = self.last_input_ns = 0
        self.info = None
        self.static_tf = {}
        self.stats = RuntimeStats()
        for name in ('depth_received', 'depth_integrated', 'depth_overwritten', 'tf_missing', 'stale_depth',
                     'mesh_updated', 'mesh_published', 'esdf_updated', 'esdf_published', 'invalid_stamp', 'future_depth', 'out_of_order', 'map_resets'):
            self.stats.inc(name, 0)
        self.extra = dict(input_stamp_ns=0, camera_position_base=[], receive_to_result_ms=0.,
                          known_cells=0, total_cells=0, error='', pose_source='image_timestamp_tf', output_mode='sparse_observed_voxels',
                          observed_cells=0, allocated_tsdf_blocks=0, mesh_triangles=0, mesh_stamp_ns=0)
        self.save_requests = queue.Queue(maxsize=1)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.full_pub = (self.create_publisher(PointCloud2, '~/esdf_3d', best) if self.esdf_extras else None)
        self.dense_pub = (self.create_publisher(PointCloud2, '~/esdf_grid_debug', best)
                          if self.dense_debug else None)
        self.observed_pub = self.create_publisher(PointCloud2, '~/esdf_observed', best)
        self.mesh_pub = self.create_publisher(MarkerArray, '~/mesh', reliable)
        self.clearance_pub = (self.create_publisher(PointCloud2, '~/esdf_clearance', best) if self.esdf_extras else None)
        self.surface_pub = (self.create_publisher(PointCloud2, '~/esdf_surface', best) if self.esdf_extras else None)
        self.legend_pub = (self.create_publisher(MarkerArray, '~/esdf_legend', reliable) if self.esdf_extras else None)
        self.pose_pub = self.create_publisher(PoseStamped, '~/camera_pose', best)
        self.stats_pub = self.create_publisher(String, '~/stats', reliable)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'], self._info_cb, best)
        self.create_subscription(Image, self.cfg['depth_topic'], self._depth_cb, best)
        if self.sim_time:
            self.create_subscription(ClockMsg, '/clock', self._clock_cb, best)
        static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self._static_cb, static_qos)
        self.service_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/reset_map', self._reset_cb, callback_group=self.service_group)
        self.create_service(Trigger, '~/save_map', self._save_cb, callback_group=self.service_group)
        self.create_service(Trigger, '~/save_mesh', self._save_mesh_cb, callback_group=self.service_group)
        # Keep diagnostics alive even when Gazebo is paused; frame age still uses /clock only.
        self.create_timer(1., self._stats_cb, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.mapping = threading.Thread(target=self._mapping_guarded, name='gazebo_mapping', daemon=True)
        self.publishing = threading.Thread(target=self._publish_guarded, name='gazebo_publisher', daemon=True)
        self.mesh_publishing = threading.Thread(target=self._mesh_publish_guarded, name='mesh_publisher', daemon=True)
        self.mesh_publishing.start()
        self.mapping.start()
        self.publishing.start()
        self.get_logger().info('ROS depth input; sparse observed 3D ESDF; exact image-stamp TF; no camera SDK or SLAM')

    def _info_cb(self, msg):
        with self.state:
            self.info = msg

    def _static_cb(self, msg):
        with self.state:
            for t in msg.transforms:
                self.static_tf[t.child_frame_id] = t

    def _reset_locked(self, reason):
        self.epoch += 1
        self.floor_ns = self.now_ns
        self.last_input_ns = 0
        self.depth_slot.put(None)
        self.result_slot.put(None)
        if hasattr(self, 'mesh_slot'):
            self.mesh_slot.put((self.epoch, self.now_ns, None))
        self.extra.update(input_stamp_ns=0, camera_position_base=[], known_cells=0, total_cells=0,
                          observed_cells=0, allocated_tsdf_blocks=0, mesh_triangles=0, mesh_stamp_ns=0)
        self.stats.inc('map_resets')
        self.get_logger().info(f'Map epoch {self.epoch}: {reason}; old queued/in-flight results are discarded')

    def _clock_cb(self, msg):
        now = stamp_ns(msg.clock)
        with self.state:
            backwards = now < self.now_ns
            self.now_ns = now
            if backwards:
                self._reset_locked('clock moved backwards')
                self.tf_buffer.clear()
                for transform in self.static_tf.values():
                    self.tf_buffer.set_transform_static(transform, 'gazebo_epoch_restore')

    def _depth_cb(self, msg):
        self.stats.inc('depth_received')
        with self.state:
            if not self.sim_time:
                self.now_ns = self.get_clock().now().nanoseconds
            if self.info is None:
                self.stats.inc('camera_info_missing')
                return
            if (msg.header.frame_id != self.cfg['depth_frame'] or self.info.header.frame_id != msg.header.frame_id or
                    (msg.width, msg.height) != (self.info.width, self.info.height)):
                self.stats.inc('camera_info_mismatch')
                return
            stamp = stamp_ns(msg.header.stamp)
            # A small positive lead can be DDS ordering between Image and /clock.
            # Worker waits for /clock to catch up, never fuses a future timestamp.
            rejection = stamp_rejection(stamp, self.now_ns, self.floor_ns, self.last_input_ns,
                                        int(self.cfg['max_frame_age_sec']*1e9))
            if rejection and rejection != 'future_depth':
                self.stats.inc(rejection)
                return
            if rejection == 'future_depth' and stamp-self.now_ns > int(self.cfg['max_frame_age_sec']*1e9):
                self.stats.inc(rejection)
                return
            sequence, pending = self.depth_slot.latest()
            if sequence > self.taken and pending is not None:
                self.stats.inc('depth_overwritten')
            self.depth_slot.put(Input(self.epoch, msg, self.info, time.perf_counter()))

    def _reset_cb(self, request, response):
        with self.state:
            self._reset_locked('reset_map service')
            response.success, response.message = True, f'Map reset requested; epoch={self.epoch}'
        return response

    async def _save_mesh_cb(self, request, response):
        return await self._save_cb(request, response, kind='mesh')

    async def _save_cb(self, request, response, kind='map'):
        future = Future()
        with self.state:
            if self.stop_event.is_set():
                response.success, response.message = False, 'Mapping worker is stopped'
                return response
            epoch = self.epoch
        try:
            self.save_requests.put_nowait((epoch, future, kind))
        except queue.Full:
            response.success, response.message = False, 'Another map save is pending'
            return response
        response.success, response.message = await future
        return response

    def _save_pending(self, backend, epoch):
        try:
            requested_epoch, future, kind = self.save_requests.get_nowait()
        except queue.Empty:
            return
        with self.state:
            valid = backend is not None and epoch == self.epoch == requested_epoch
        if not valid:
            future.set_result((False, 'No map in the requested epoch'))
            return
        try:
            folder = Path(self.cfg['output_directory']).expanduser()
            path = (backend.save_mesh(folder/'base_link_mesh.ply') if kind == 'mesh'
                    else backend.save_map(folder/'base_link_map.nvblox'))
            future.set_result((True, str(path)))
        except Exception as error:
            future.set_result((False, str(error)))

    def _input_valid(self, item):
        with self.state:
            if not self.sim_time:
                self.now_ns = self.get_clock().now().nanoseconds
            if item.epoch != self.epoch:
                return False
            rejection = stamp_rejection(stamp_ns(item.msg.header.stamp), self.now_ns, self.floor_ns,
                                        self.last_input_ns, int(self.cfg['max_frame_age_sec']*1e9))
        if rejection:
            self.stats.inc(rejection)
        return rejection is None

    def _lookup(self, item):
        # Never Time(), nearest pose, or a second hand-eye transform.
        return self.tf_buffer.lookup_transform(self.cfg['world_frame'], item.msg.header.frame_id,
            Time.from_msg(item.msg.header.stamp), timeout=Duration(seconds=self.cfg['tf_timeout_sec']))

    def _mapping_guarded(self):
        try:
            self._mapping_loop()
        except Exception as error:
            with self.state:
                self.extra['error'] = repr(error)
            self.get_logger().error(f'Mapping stopped: {error!r}')
            self.stop_event.set()
            self.result_slot.close()
            self._save_pending(None, -1)

    def _mapping_loop(self):
        sequence, local_epoch, backend, intrinsics = 0, -1, None, None
        last_esdf = last_mesh = 0
        while not self.stop_event.is_set():
            with self.state:
                if local_epoch != self.epoch:
                    backend = None  # only this worker owns and destroys CUDA mapper
                    intrinsics = None
                    local_epoch = self.epoch
                    last_esdf = last_mesh = 0
            self._save_pending(backend, local_epoch)
            new_sequence, item = self.depth_slot.wait_newer(sequence, timeout=.05)
            with self.state:
                sequence = new_sequence
                self.taken = new_sequence
            if item is None:
                continue
            # A reset may happen while wait_newer() is blocked. Never fuse a
            # new-epoch item into the backend retained from the previous loop.
            if item.epoch != local_epoch:
                continue
            stamp = stamp_ns(item.msg.header.stamp)
            # Handle cross-topic delivery order with a bounded steady-clock wait.
            deadline = time.monotonic()+self.cfg['tf_timeout_sec']
            while not self.stop_event.is_set():
                with self.state:
                    waiting = item.epoch == self.epoch and stamp > self.now_ns
                if not waiting or time.monotonic() >= deadline:
                    break
                self.stop_event.wait(.002)
            if not self._input_valid(item):
                continue
            try:
                transform = self._lookup(item)
            except TransformException:
                self.stats.inc('tf_missing')
                continue
            if not self._input_valid(item):
                continue
            try:
                calibration = calibration_from_info(item.info, self.cfg['depth_stride'])
                depth = depth_meters(item.msg, self.cfg['depth_stride'], self.cfg['min_depth_m'], self.cfg['max_depth_m'])
                pose = transform_matrix(transform)
            except ValueError as error:
                self.stats.inc('invalid_depth')
                self.get_logger().warning(str(error))
                continue
            if not np.any(depth):
                self.stats.inc('empty_depth')
                continue
            if intrinsics is not None and intrinsics != calibration.depth_intrinsics:
                with self.state:
                    self._reset_locked('CameraInfo changed')
                continue
            if backend is None:
                backend = NvbloxBackend(calibration, {'voxel_size_m': self.cfg['voxel_size_m'],
                                                      'max_integration_distance_m': self.cfg['max_depth_m']})
                intrinsics = calibration.depth_intrinsics
                self.get_logger().info(f'CUDA mapper ready: depth={intrinsics.width}x{intrinsics.height}, epoch={local_epoch}')
            # CUDA initialization can take seconds. Do not fuse the frame used to initialize if now stale.
            if not self._input_valid(item):
                continue
            start = time.perf_counter()
            backend.integrate_depth(depth, pose)
            with self.state:
                if item.epoch != self.epoch:
                    continue
                self.last_input_ns = stamp
            self.stats.inc('depth_integrated')
            self.stats.add_latency('integrate_submit', (time.perf_counter()-start)*1000)
            if (self.enable_mesh and self.mesh_pub.get_subscription_count()
                    and (not last_mesh or stamp-last_mesh >= int(1e9/self.mesh_hz))):
                mesh_start = time.perf_counter()
                backend.update_mesh()
                vertices, triangles, _ = backend.mesh_numpy()
                with self.state:
                    if item.epoch == self.epoch:
                        self.mesh_slot.put((item.epoch, stamp, (vertices.copy(), triangles.copy())))
                self.stats.inc('mesh_updated')
                self.stats.add_latency('mesh_update_readback', (time.perf_counter()-mesh_start)*1000)
                last_mesh = stamp
            if last_esdf and stamp-last_esdf < int(1e9/self.cfg['esdf_hz']):
                continue
            backend.update_esdf()
            self.stats.inc('esdf_updated')
            points, distances = backend.observed_esdf_numpy(
                max_distance_m=None if self.esdf_extras else self.near_obstacle_band)
            dense = None
            if self.dense_pub is not None and self.dense_pub.get_subscription_count():
                dense = backend.query_esdf_grid(np.asarray(self.cfg['grid_center_m']), self.grid)
            self.stats.add_latency('mapping_with_gpu_readback', (time.perf_counter()-start)*1000)
            last_esdf = stamp
            with self.state:
                if item.epoch == self.epoch:
                    self.extra['allocated_tsdf_blocks'] = backend.mapper.tsdf_layer_view(0).num_blocks()
                    self.result_slot.put(Output(item.epoch, stamp, pose, points, distances,
                                                backend.esdf_unknown_distance, item.received, dense))
        self._save_pending(None, -1)

    def _publish_guarded(self):
        try:
            sequence = 0
            while not self.stop_event.is_set():
                sequence, item = self.result_slot.wait_newer(sequence, timeout=.1)
                if item is None:
                    continue
                full = (esdf_cloud(item.points, item.distances, self.cfg['world_frame'], item.stamp,
                                   self.cfg['max_visualized_distance_m'], True, item.unknown)
                        if self.full_pub is not None and self.full_pub.get_subscription_count() else None)
                dense = (esdf_cloud(*item.dense, self.cfg['world_frame'], item.stamp,
                                    self.cfg['max_visualized_distance_m'], True, item.unknown)
                         if item.dense is not None else None)
                observed = clearance_cloud(item.points, item.distances, self.cfg['world_frame'], item.stamp,
                                           self.near_obstacle_band, item.unknown, self.clearance_thresholds)
                clearance = (clearance_cloud(item.points, item.distances, self.cfg['world_frame'], item.stamp,
                    self.cfg['max_visualized_distance_m'], item.unknown, self.clearance_thresholds)
                    if self.clearance_pub is not None and self.clearance_pub.get_subscription_count() else None)
                surface = (esdf_cloud(item.points, item.distances, self.cfg['world_frame'], item.stamp,
                    self.cfg['voxel_size_m'], False, item.unknown)
                    if self.surface_pub is not None and self.surface_pub.get_subscription_count() else None)
                known = np.isfinite(item.distances) & ~np.isclose(np.abs(item.distances), abs(item.unknown), rtol=1e-5, atol=1e-3)
                with self.state:
                    if item.epoch != self.epoch:
                        continue
                    if full is not None:
                        self.full_pub.publish(full)
                    if dense is not None:
                        self.dense_pub.publish(dense)
                    self.observed_pub.publish(observed)
                    if clearance is not None:
                        self.clearance_pub.publish(clearance)
                    if surface is not None:
                        self.surface_pub.publish(surface)
                    self.pose_pub.publish(pose_stamped(item.pose, self.cfg['world_frame'], item.stamp))
                    latency = (time.perf_counter()-item.received)*1000
                    self.extra.update(input_stamp_ns=item.stamp, camera_position_base=item.pose[:3, 3].tolist(),
                                      receive_to_result_ms=latency, known_cells=int(known.sum()), total_cells=len(item.distances),
                                      observed_cells=observed.width)
                    self.stats.inc('esdf_published')
                    self.stats.add_latency('receive_to_result', latency)
        except Exception as error:
            with self.state:
                self.extra['error'] = repr(error)
            self.get_logger().error(f'Publishing stopped: {error!r}')
            self.stop_event.set()
            self.depth_slot.close()

    def _mesh_publish_guarded(self):
        try:
            sequence, previous_count = 0, 0
            while not self.stop_event.is_set():
                sequence, item = self.mesh_slot.wait_newer(sequence, timeout=.1)
                if item is None:
                    continue
                epoch, stamp, geometry = item
                if geometry is None:
                    markers = clear_mesh(self.cfg['world_frame'], stamp, 0, previous_count)
                    count, triangles = 0, 0
                else:
                    markers, count, triangles = reconstruction_mesh(
                        *geometry, self.cfg['world_frame'], stamp, previous_count)
                with self.state:
                    if epoch != self.epoch:
                        continue
                    self.mesh_pub.publish(markers)
                    previous_count = count
                    self.extra.update(mesh_triangles=triangles,
                                      mesh_stamp_ns=stamp)
                    self.stats.inc('mesh_published')
        except Exception as error:
            with self.state:
                self.extra['mesh_error'] = repr(error)
            self.get_logger().error(f'Mesh publishing stopped: {error!r}')

    def _stats_cb(self):
        with self.state:
            extra = dict(self.extra, map_epoch=self.epoch, sim_time_ns=self.now_ns)
        report = self.stats.snapshot(extra)
        report.update(report['totals'])
        self.stats_pub.publish(String(data=json.dumps(report)))
        if self.legend_pub is not None and self.legend_pub.get_subscription_count():
            self.legend_pub.publish(distance_legend(self.cfg['world_frame'], extra['sim_time_ns'],
                self.clearance_thresholds, self.cfg['max_visualized_distance_m']))

    def close(self):
        self.stop_event.set()
        self.depth_slot.close()
        self.result_slot.close()
        self.mesh_slot.close()
        self.mapping.join(timeout=15)
        self.publishing.join(timeout=5)
        self.mesh_publishing.join(timeout=5)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboNvbloxNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
