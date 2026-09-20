"""ROS I/O, SAGE worker and independent CPU execution guard."""
import csv
import json
import threading
import time
from pathlib import Path
import numpy as np
import torch
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from ur7e_nvblox.depth import transform_matrix
from ur7e_perception_planning.msg import PlanningMap
from .geometry import JOINTS, RobotGeometry
from .grid import Snapshot
from .sage import build_task
from .runtime import GPU_LOCK


def check_segment(geometry, snapshot, start, end, margin=.025):
    """Check the whole short joint segment, including self collision and floor.

    Samples are <= .003 rad apart. 1.4 m per rad is a conservative link-point
    motion bound for this arm; half-sample displacement is charged as padding.
    """
    delta = np.asarray(end)-start
    if not np.isfinite(delta).all() or np.max(np.abs(delta)) > .15:
        return False, -1., 'invalid_segment'
    if hasattr(geometry, 'lower') and (np.any(end < geometry.lower) or np.any(end > geometry.upper)):
        return False, -1., 'joint_limit'
    count = max(2, int(np.ceil(np.max(np.abs(delta))/.003))+1,
                int(np.ceil(1.4*np.sum(np.abs(delta))/.002))+1)
    swept_padding = 1.4*np.sum(np.abs(delta))/(2*(count-1))
    minimum = float('inf')
    for alpha in np.linspace(0, 1, count):
        centers = geometry.spheres(start+alpha*delta)
        distance = float(np.min(snapshot.query(centers)-geometry.radii))
        minimum = min(minimum, distance)
        if distance < margin+swept_padding:
            return False, minimum, 'esdf_or_unknown'
        if geometry.self_clearance(centers) < .008+2*swept_padding:
            return False, minimum, 'self_collision'
        if np.min(centers[:, 2]-geometry.radii) < margin+swept_padding:
            return False, minimum, 'floor'
    return True, minimum, 'clear'


class Planner(Node):
    def __init__(self):
        super().__init__('perception_planner')
        defaults = dict(max_joint_speed=.08, planning_dt=.1, servo_hz=50., map_timeout_wall=3.,
                        map_timeout_sim=.8, joint_timeout_wall=.5, command_timeout_wall=1.5,
                        collision_margin=.015, particles=384, autostart=True,
                        goal_offset=[-.06, -.06, 0.], output_directory='~/nvblox_output/perception_planning')
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.geometry = RobotGeometry()
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.state = None
        self.snapshot = None
        self.goal = None
        self.target = None
        self.q_command = None
        self.enabled = self.cfg['autostart']
        self.status = 'WAITING_FOR_STATE_AND_MAP'
        self.guard_reason = 'waiting'
        self.minimum = None
        self.plan_count = 0
        self.sage_count = 0
        self.guard_rejections = 0
        self.accepted_commands = 0
        self.plan_ms = 0.
        self.optimize_ms = 0.
        self.admission_ms = 0.
        self.error = None
        self.command_source = 'hold'
        self.preview = None
        self.ee = None
        self.last_plan_map = None
        self.last_sim = None
        self.last_map_epoch = None
        self.generation = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10), node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PlanningMap, '/perception_planning/map', self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(JointState, '/joint_states', self.on_state, best)
        self.create_subscription(PoseStamped, '/target_pose', self.on_goal, 1)
        self.pub_command = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 1)
        self.pub_status = self.create_publisher(String, '/perception_planning/status', 1)
        # RViz MarkerArray displays request reliable delivery by default. A
        # reliable publisher also serves subscribers configured as best effort.
        self.pub_markers = self.create_publisher(
            MarkerArray, '/perception_planning/markers',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_ee = self.create_publisher(PoseStamped, '/ee_pose', best)
        self.create_service(SetBool, '/perception_planning/enable', self.on_enable)
        self.create_timer(.5, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.run_dir = Path(self.cfg['output_directory']).expanduser()/time.strftime('%Y%m%d_%H%M%S')
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.log = (self.run_dir/'execution.csv').open('w', buffering=1)
        self.csv = csv.writer(self.log)
        self.csv.writerow(['wall_time', 'sim_time', 'map_sequence', 'status', 'guard', 'clearance',
                           *['q_actual_'+str(i) for i in range(6)], *['q_cmd_'+str(i) for i in range(6)],
                           *['q_next_'+str(i) for i in range(6)], 'command_source'])
        self.planner_thread = threading.Thread(target=self.plan_loop, name='sage-esdf', daemon=True)
        self.servo_thread = threading.Thread(target=self.servo_loop, name='guarded-servo', daemon=True)
        self.planner_thread.start(); self.servo_thread.start()
        self.get_logger().info(f'Low-speed online ESDF planning; logs: {self.run_dir}')

    def on_state(self, msg):
        try:
            ids = [msg.name.index(name) for name in JOINTS]
            q = np.asarray([msg.position[i] for i in ids])
            dq = np.asarray([msg.velocity[i] for i in ids])
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if not np.isfinite(q).all() or not np.isfinite(dq).all() or stamp <= 0:
                return
            with self.lock:
                if self.state is not None and stamp < self.state[2]:
                    self.snapshot = self.target = None
                    self.generation += 1
                self.state = (q, dq, stamp, time.monotonic())
        except (ValueError, IndexError):
            self.get_logger().warning('Incomplete joint state rejected', throttle_duration_sec=5)

    def on_map(self, msg):
        try:
            snapshot = Snapshot.from_message(msg)
            now = self.get_clock().now().nanoseconds*1e-9
            if abs(now-snapshot.stamp) > self.cfg['map_timeout_sim']:
                return
            with self.lock:
                if self.snapshot is not None and snapshot.epoch == self.snapshot.epoch and snapshot.sequence <= self.snapshot.sequence:
                    return
                if self.last_map_epoch != snapshot.epoch:
                    self.target = None; self.generation += 1
                self.last_map_epoch = snapshot.epoch
                self.snapshot = snapshot
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=5)

    def on_goal(self, msg):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if not msg.header.frame_id or not np.isfinite(p).all():
            return
        if msg.header.frame_id != 'base_link':
            try:
                tf = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp))
                matrix = transform_matrix(tf.transform)
                p = matrix[:3, :3]@p+matrix[:3, 3]
            except TransformException as exc:
                self.get_logger().warning(f'Goal rejected: {exc}'); return
        with self.lock:
            self.goal = p; self.target = None; self.generation += 1
        self.get_logger().info(f'New base_link goal: {p.tolist()}')

    def on_enable(self, request, response):
        with self.lock:
            self.enabled = request.data; self.target = None; self.generation += 1
        response.success = True
        response.message = 'Enabled; waiting for fresh state/map' if request.data else 'Disabled; holding position'
        return response

    def inputs(self):
        with self.lock:
            return self.state, self.snapshot, None if self.goal is None else self.goal.copy(), self.generation

    def freshness(self, state, snapshot):
        wall, sim = time.monotonic(), self.get_clock().now().nanoseconds*1e-9
        if not self.enabled:
            return 'DISABLED'
        if state is None or wall-state[3] > self.cfg['joint_timeout_wall'] or abs(sim-state[2]) > .25:
            return 'STALE_JOINT_STATE'
        if snapshot is None:
            return 'WAITING_FOR_MAP'
        if wall-snapshot.received > self.cfg['map_timeout_wall'] or abs(sim-snapshot.stamp) > self.cfg['map_timeout_sim']:
            return 'STALE_MAP'
        return None

    def plan_loop(self):
        try:
            torch.set_num_threads(2)
            with GPU_LOCK:
                task = build_task(self.run_dir, self.cfg['planning_dt'], self.cfg['particles'])
            self.task = task
            rollout = task.controller.rollout_fn
            hold_generation, hold_position = None, None
            while not self.stop.is_set():
                started = time.monotonic()
                state, snapshot, goal, generation = self.inputs()
                reason = self.freshness(state, snapshot)
                if reason:
                    self.stop.wait(.05); continue
                q, dq = state[:2]
                ee_transform = self.geometry.fk(q)['tool0']
                self.ee = ee_transform[:3, 3]
                if goal is None:
                    goal = self.ee+np.asarray(self.cfg['goal_offset'])
                    with self.lock:
                        self.goal = goal
                if np.linalg.norm(goal-self.ee) < .012:
                    if hold_generation != generation:
                        hold_generation, hold_position = generation, q.copy()
                    with self.lock:
                        if generation == self.generation:
                            self.target = (hold_position.copy(), time.monotonic(), snapshot.epoch, generation)
                    self.status = 'GOAL_REACHED'
                    self.stop.wait(.1); continue
                hold_generation = None
                dt = self.cfg['planning_dt']
                jac = np.column_stack([(self.geometry.fk(q+np.eye(6)[i]*1e-4)['tool0'][:3, 3]-self.ee)/1e-4 for i in range(6)])
                velocity_seed = jac.T @ np.linalg.solve(jac@jac.T+.0025*np.eye(3), goal-self.ee)
                velocity_seed = np.clip(velocity_seed, -self.cfg['max_joint_speed'], self.cfg['max_joint_speed'])
                candidate = q+velocity_seed*dt
                admission_started = time.monotonic()
                ok = False
                self.optimize_ms = 0.
                source = 'checked_cartesian_refinement'
                # Terminal controller has the same hard whole-arm checks. If it
                # succeeds, leave the GPU available to mapping instead of running
                # a full MPPI optimization whose result would be overridden.
                if np.linalg.norm(goal-self.ee) < .10:
                    ok, _, _ = check_segment(self.geometry, snapshot, q, candidate, self.cfg['collision_margin'])
                    ok = ok and np.min(snapshot.query(self.geometry.spheres(candidate))-self.geometry.radii) >= self.cfg['collision_margin']+.01
                if not ok:
                    source = 'sage'
                    optimization_started = time.monotonic()
                    current = np.concatenate((q, np.clip(dq, -.15, .15), np.zeros(6), [state[2]]))
                    with GPU_LOCK, torch.no_grad():
                        task.update_params(goal_ee_pos=goal, goal_ee_rot=ee_transform[:3, :3])
                        rollout.set_map(snapshot)
                        acceleration_seed = torch.as_tensor(np.clip((velocity_seed-dq)/dt, -.5, .5), **task.tensor_args)
                        task.controller.mean_action = .4*task.controller.mean_action + .6*acceleration_seed[None]
                        result = task.controller.optimize(torch.as_tensor(current, **task.tensor_args)[None], shift_steps=1)
                        if task.controller.top_trajs is not None:
                            self.preview = task.controller.top_trajs[:3].detach().cpu().numpy().copy()
                        acceleration = result[0][0].detach().cpu().numpy()
                    self.optimize_ms = (time.monotonic()-optimization_started)*1000
                    self.sage_count += 1
                    velocity = np.clip(dq+acceleration*dt, -self.cfg['max_joint_speed'], self.cfg['max_joint_speed'])
                    candidate = q+velocity*dt
                    ok, _, _ = check_segment(self.geometry, snapshot, q, candidate, self.cfg['collision_margin'])
                    ok = ok and np.min(snapshot.query(self.geometry.spheres(candidate))-self.geometry.radii) >= self.cfg['collision_margin']+.01
                self.plan_ms = (time.monotonic()-started)*1000
                self.admission_ms = (time.monotonic()-admission_started)*1000-self.optimize_ms
                self.plan_count += 1; self.last_plan_map = snapshot.sequence
                with self.lock:
                    if generation == self.generation and not self.freshness(*self.inputs()[:2]):
                        hold = q.copy() if self.q_command is None else self.q_command.copy()
                        self.target = (candidate if ok else hold, time.monotonic(), snapshot.epoch, generation)
                        self.command_source = source if ok else 'blocked_hold'
                self.status = 'PLANNING' if ok else 'BLOCKED'
                self.stop.wait(max(.02, dt-(time.monotonic()-started)))
        except Exception as exc:
            import traceback
            self.error = traceback.format_exc()
            self.get_logger().error(f'Planning failed: {self.error}')
            with self.lock:
                self.enabled = False; self.target = None

    def servo_loop(self):
        period = 1/self.cfg['servo_hz']
        previous_sim = None
        while not self.stop.wait(period):
            try:
                state, snapshot, goal, generation = self.inputs()
                if state is None:
                    continue
                q, dq, sim, _ = state
                with self.lock:
                    target = self.target
                dt = 0. if previous_sim is None else np.clip(sim-previous_sim, 0., .05)
                previous_sim = sim
                reason = self.freshness(state, snapshot)
                if not reason and (target is None or time.monotonic()-target[1] > self.cfg['command_timeout_wall']
                                   or target[2] != snapshot.epoch or target[3] != generation):
                    reason = 'STALE_PLAN'
                if self.q_command is None:
                    self.q_command = q.copy()
                # A hold never pursues the previous target. Tracking divergence
                # also invalidates the planned segment.
                if not reason and np.max(np.abs(self.q_command-q)) > .035:
                    reason = 'TRACKING_ERROR'
                if reason:
                    # Freeze the last issued (short, checked) position command;
                    # do not pursue the old planner target or chase feedback.
                    self.minimum = None
                    self.guard_reason = reason
                else:
                    step = np.clip(target[0]-self.q_command, -self.cfg['max_joint_speed']*dt, self.cfg['max_joint_speed']*dt)
                    candidate = self.q_command+step
                    ok, minimum, why = check_segment(self.geometry, snapshot, q, candidate, self.cfg['collision_margin'])
                    self.minimum = minimum
                    if ok:
                        self.q_command = candidate
                        self.accepted_commands += 1
                    else:
                        self.guard_rejections += 1
                    self.guard_reason = why
                self.pub_command.publish(Float64MultiArray(data=self.q_command.tolist()))
                self.csv.writerow([time.time(), sim, snapshot.sequence if snapshot else -1,
                                   self.status, self.guard_reason, self.minimum, *q, *self.q_command,
                                   *(target[0] if target else self.q_command), self.command_source])
            except Exception:
                import traceback
                self.error = traceback.format_exc()
                with self.lock:
                    self.enabled = False; self.target = None

    def publish_status(self):
        state, snapshot, goal, generation = self.inputs()
        sim = self.get_clock().now().nanoseconds*1e-9
        record = dict(status=self.freshness(state, snapshot) or self.status,
                      guard=self.guard_reason, enabled=self.enabled, error=self.error,
                      plans=self.plan_count, sage_optimizations=self.sage_count, plan_ms=self.plan_ms, minimum_clearance=self.minimum,
                      optimize_ms=self.optimize_ms, admission_ms=self.admission_ms,
                      accepted_commands=self.accepted_commands, guard_rejections=self.guard_rejections,
                      command_source=self.command_source,
                      map_sequence=None if snapshot is None else snapshot.sequence,
                      map_epoch=None if snapshot is None else snapshot.epoch,
                      last_plan_map=self.last_plan_map,
                      map_age_sim=None if snapshot is None else sim-snapshot.stamp,
                      goal=None if goal is None else goal.tolist(),
                      ee=None if self.ee is None else self.ee.tolist(),
                      goal_error=None if goal is None or self.ee is None else float(np.linalg.norm(goal-self.ee)))
        self.pub_status.publish(String(data=json.dumps(record, allow_nan=False)))
        (self.run_dir/'latest_status.json').write_text(json.dumps(record, indent=2, allow_nan=False))
        if state is None:
            return
        centers = self.geometry.spheres(state[0])
        markers = MarkerArray()
        for i, (center, radius) in enumerate(zip(centers, self.geometry.radii)):
            marker = Marker(); marker.header.frame_id = 'base_link'; marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'robot_collision'; marker.id = i; marker.type = Marker.SPHERE
            marker.pose.position = Point(x=float(center[0]), y=float(center[1]), z=float(center[2]))
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = float(2*radius)
            marker.color.r = 1.; marker.color.g = .8; marker.color.a = .3
            markers.markers.append(marker)
        if goal is not None:
            marker = Marker(); marker.header.frame_id = 'base_link'; marker.ns = 'goal'; marker.id = 100
            marker.type = Marker.SPHERE; marker.pose.orientation.w = 1.
            marker.pose.position = Point(x=float(goal[0]), y=float(goal[1]), z=float(goal[2]))
            marker.scale.x = marker.scale.y = marker.scale.z = .035
            marker.color.g = 1.; marker.color.a = 1.
            markers.markers.append(marker)
        preview = self.preview
        if preview is not None and preview.ndim == 3:
            for i, trajectory in enumerate(preview):
                marker = Marker(); marker.header.frame_id = 'base_link'; marker.ns = 'sage_candidates'; marker.id = 200+i
                marker.type = Marker.LINE_STRIP; marker.pose.orientation.w = 1.; marker.scale.x = .003
                marker.color.b = 1.; marker.color.g = .6; marker.color.a = .7
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in trajectory]
                markers.markers.append(marker)
        self.pub_markers.publish(markers)
        pose = PoseStamped(); pose.header.frame_id = 'base_link'; pose.header.stamp = self.get_clock().now().to_msg()
        ee = self.geometry.fk(state[0])['tool0'][:3, 3]
        pose.pose.position = Point(x=float(ee[0]), y=float(ee[1]), z=float(ee[2])); pose.pose.orientation.w = 1.
        self.pub_ee.publish(pose)

    def close(self):
        self.stop.set()
        self.servo_thread.join(timeout=3)
        self.planner_thread.join(timeout=5)
        import rclpy
        if self.state is not None and rclpy.ok():
            self.pub_command.publish(Float64MultiArray(data=self.state[0].tolist()))
        self.log.close()
