"""Extend the existing TF mapper with self filtering and an atomic planning map.

Depth decoding, timestamp rejection, TF buffering, reset/save services and native
nvBlox all come from the existing packages. No scene obstacle coordinates enter
this mapper. The initial clearance is an explicit commissioning assumption.
"""
import time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformException
from std_srvs.srv import SetBool
from ur7e_nvblox.node import TfMapper
from ur7e_nvblox.depth import depth_meters, intrinsics_from_info, depth_calibration, transform_matrix, known_distances
from realtime_nvblox.nvblox_backend import NvbloxBackend
from realtime_nvblox.ros_publishers import esdf_cloud, pose_stamped
from ur7e_perception_planning.msg import PlanningMap
from .geometry import RobotGeometry
from .runtime import GPU_LOCK
from .grid import grid_points, observed_free, conservative_field, pack_map


class PlanningMapper(TfMapper):
    def _on_info(self, msg):
        previous = self.info
        if previous is not None and (previous.width != msg.width or previous.height != msg.height
                                     or previous.header.frame_id != msg.header.frame_id
                                     or list(previous.p) != list(msg.p) or list(previous.k) != list(msg.k)):
            self._request_reset()
        super()._on_info(msg)

    def _validate_config(self):
        super()._validate_config()
        if self.cfg['world_frame'] != 'base_link':
            raise ValueError('This integration requires world_frame=base_link')
        self.initial_clearance = self.declare_parameter('initial_clearance_m', .12).value
        if not 0 <= self.initial_clearance <= .15:
            raise ValueError('initial_clearance_m must be in [0,.15] and physically commissioned')
        self.geometry = RobotGeometry()
        initial_file = Path(get_package_share_directory('ur7e_reconstruction'))/'config/high_initial_positions.yaml'
        self.commissioned_centers = self.geometry.spheres(np.array(list(yaml.safe_load(initial_file.read_text()).values())))
        self.instance_epoch = time.time_ns()
        self.origin, self.dims, self.points = grid_points(self.cfg['grid_center_m'], self.cfg['grid_size_m'], self.cfg['grid_resolution_m'])
        self.map_pub = self.create_publisher(PlanningMap, '/perception_planning/map',
                                            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.map_sequence = 0
        self.mapping_enabled = True
        self.create_service(SetBool, '/perception_planning/mapping_enabled', self._mapping_enabled)

    def _mapping_enabled(self, request, response):
        self.mapping_enabled = request.data
        response.success = True
        response.message = 'Mapping enabled' if request.data else 'Mapping paused; planner freshness watchdog remains active'
        return response

    def _robot_centers(self, stamp):
        centers = np.empty_like(self.geometry.local)
        for i, link in enumerate(self.geometry.links):
            transform = self.tf_buffer.lookup_transform('base_link', link, stamp)
            pose = transform_matrix(transform.transform)
            ids = self.geometry.link_ids == i
            centers[ids] = self.geometry.local[ids] @ pose[:3, :3].T + pose[:3, 3]
        return centers

    def _worker(self):
        sequence, active_epoch, next_update = 0, -1, 0.
        sources = np.zeros(len(self.points), dtype=np.uint8)
        try:
            while not self._stop.is_set():
                new_sequence, item = self.frames.wait_newer(sequence, timeout=.2)
                if item is None:
                    continue
                sequence = new_sequence
                epoch, msg, info, ns, received = item
                if not self.mapping_enabled or epoch != self._epoch or time.monotonic() < next_update:
                    continue
                if (msg.header.frame_id != self.cfg['depth_frame'] or info.header.frame_id != msg.header.frame_id
                        or (msg.width, msg.height) != (info.width, info.height)):
                    self.stats.inc('calibration_mismatch'); continue
                if abs((self.get_clock().now().nanoseconds-ns)/1e9) > self.cfg['max_frame_age_sec']:
                    self.stats.inc('stale_depth'); continue
                picked = time.perf_counter()
                try:
                    stamp = Time.from_msg(msg.header.stamp)
                    tf = self.tf_buffer.lookup_transform(self.world, msg.header.frame_id, stamp,
                                                        timeout=Duration(seconds=self.cfg['tf_timeout_sec']))
                    centers = self._robot_centers(stamp)
                except TransformException:
                    self.stats.inc('tf_missing'); continue
                transformed = time.perf_counter()
                pose = transform_matrix(tf.transform)
                intr = intrinsics_from_info(info, self.cfg['depth_stride'])
                depth = depth_meters(msg, self.cfg['depth_stride'], self.cfg['min_depth_m'], self.cfg['max_depth_m'])
                # Remove depth returns on the arm/payload. Masked pixels stay invalid,
                # so they cannot clear the occluded space behind the robot.
                v, u = np.indices(depth.shape)
                camera_points = np.stack(((u-intr.cx)*depth/intr.fx, (v-intr.cy)*depth/intr.fy, depth), -1)
                world_points = camera_points @ pose[:3, :3].T + pose[:3, 3]
                robot_pixels = np.zeros(depth.shape, bool)
                for center, radius in zip(centers, self.geometry.radii):
                    robot_pixels |= np.sum((world_points-center)**2, axis=-1) < (radius+.015)**2
                # The fixed pedestal is not a moving collision link. Without
                # masking it, the camera maps the robot base as an obstacle
                # touching the attached shoulder and prevents all motion.
                robot_pixels |= ((world_points[..., 0]**2+world_points[..., 1]**2 < .115**2)
                                 & (world_points[..., 2] >= 0) & (world_points[..., 2] < .17))
                depth[robot_pixels] = 0.
                if not np.any(depth):
                    continue
                started = time.monotonic()
                with self._backend_lock, GPU_LOCK:
                    if active_epoch != epoch or self._backend_intrinsics != intr:
                        self._backend = None
                        self._backend = NvbloxBackend(depth_calibration(intr), {
                            'voxel_size_m': self.cfg['voxel_size_m'], 'max_integration_distance_m': self.cfg['max_depth_m']})
                        self._backend_intrinsics, active_epoch = intr, epoch
                        sources[:] = 0
                        # The prior is anchored to the commissioned pose, including
                        # after resets; it must never grow along the robot's path.
                        for center, radius in zip(self.commissioned_centers, self.geometry.radii):
                            inside = np.sum((self.points-center)**2, axis=1) <= (radius+self.initial_clearance)**2
                            sources[inside] = 2
                        self.stats.inc('map_initialized')
                    if epoch != self._epoch:
                        continue
                    if abs((self.get_clock().now().nanoseconds-ns)/1e9) > self.cfg['max_frame_age_sec']:
                        self.stats.inc('stale_after_gpu_wait'); continue
                    self._backend.integrate_depth(depth, pose)
                    self._backend.update_esdf()
                    raw = self._backend.query_esdf(self.points).reshape(-1)
                    unknown = self._backend.esdf_unknown_distance
                known = known_distances(raw, unknown)
                # Both explicit rays and positive native ESDF observations are
                # evidence. Unknown sentinel values never become free evidence.
                seen = observed_free(self.points, depth, pose, intr, .05 + np.sqrt(3)*self.cfg['grid_resolution_m']/2)
                sources[seen | (known & (raw > .02))] = 1
                planning, published_sources = conservative_field(raw, known, sources, self.points, self.dims,
                                                                  self.cfg['grid_resolution_m'])
                self.map_sequence += 1
                summary = dict(known_cells=int(known.sum()), free_cells=int((published_sources > 0).sum()),
                               prior_cells=int((published_sources == 2).sum()), map_epoch=epoch,
                               unknown_distance_m=float(unknown), sequence=self.map_sequence,
                               receive_to_result_ms=(time.perf_counter()-received)*1000,
                               queue_ms=(picked-received)*1000,
                               tf_ms=(transformed-picked)*1000,
                               filter_ms=(started-transformed)*1000,
                               self_filtered_pixels=int(robot_pixels.sum()))
                self.stats.inc('depth_integrated'); self.stats.inc('esdf_updated')
                self.stats.add_latency('fusion_and_esdf', (time.monotonic()-started)*1000)
                self.results.put((epoch, ns, pose, raw, planning, published_sources, summary))
                next_update = time.monotonic() + 1/self.cfg['esdf_hz']
        except Exception as exc:
            self._worker_error('Planning mapper', exc)

    def _publish_latest(self):
        sequence, item = self.results.get_if_newer(self._last_output_sequence)
        if item is None:
            return
        self._last_output_sequence = sequence
        epoch, ns, pose, raw, planning, sources, summary = item
        if epoch != self._epoch or abs((self.get_clock().now().nanoseconds-ns)/1e9) > self.cfg['max_frame_age_sec']:
            self.stats.inc('stale_output'); return
        msg = PlanningMap()
        msg.header.frame_id = 'base_link'
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        msg.epoch, msg.sequence = self.instance_epoch+epoch, summary['sequence']
        msg.origin.x, msg.origin.y, msg.origin.z = self.origin.tolist()
        msg.resolution = float(self.cfg['grid_resolution_m'])
        msg.dimensions = self.dims.astype(np.uint32).tolist()
        msg.unknown_distance = summary['unknown_distance_m']
        pack_map(msg, raw, planning, sources)
        consumer = getattr(self, 'map_consumer', None)
        if consumer is not None:
            consumer(msg)
        self.map_pub.publish(msg)
        if self.observed_pub.get_subscription_count():
            self.observed_pub.publish(esdf_cloud(self.points, raw, self.world, ns, self.cfg['max_visualized_distance_m'],
                                                 unknown_distance_m=summary['unknown_distance_m']))
        self.pose_pub.publish(pose_stamped(pose, self.world, ns))
        self._summary = summary
        self.stats.inc('esdf_published')
