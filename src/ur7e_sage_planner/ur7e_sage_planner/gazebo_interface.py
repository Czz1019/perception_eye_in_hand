"""ROS/Gazebo interface and visualization extracted from the original simulation helpers."""
import os
import queue
import time
from threading import Lock
import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Pose, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from storm_kit.util_file import join_path, get_mpc_configs_path
FORWARD_POSITION_CMD_TOPIC = '/forward_position_controller/commands'

def _log(message):
    print(message, flush=True)

def transform_point(position, orientation_xyzw, point):
    """将点从机器人坐标系变换到世界坐标系"""
    rot = Rotation.from_quat(orientation_xyzw)
    return rot.apply(point) + np.array(position)


def inv_transform_point(position, orientation_xyzw, point):
    """将点从世界坐标系变换到机器人坐标系"""
    rot = Rotation.from_quat(orientation_xyzw).inv()
    return rot.apply(np.array(point) - np.array(position))


class GazeboRobotInterface(Node):
    """
    Gazebo 机器人 ROS2 接口
    
    订阅:
        /joint_states - 关节状态
        /target_pose - 目标位置（用于动态更新目标）
    发布:
        /forward_position_controller/commands - 关节位置指令
        /ee_pose - 末端位置
        /visualization_marker_array - 可视化标记
    """
    
    def __init__(self, joint_names: list, control_rate: float = 50.0):
        super().__init__('storm_mpc_reach_static')
        
        self.joint_names = joint_names
        self.n_dof = len(joint_names)
        self.control_rate = control_rate
        self.control_dt = 1.0 / control_rate
        
        # 线程锁
        self._lock = Lock()
        
        # 当前状态
        self._positions = None
        self._velocities = None
        self._prev_velocities = None
        self._prev_time = None
        self._state_received = False
        self._state_count = 0
        self._cmd_count = 0
        self._runtime_control_topic = FORWARD_POSITION_CMD_TOPIC
        
        # 目标位置（从 ROS2 接收）
        self._target_pos = None
        
        # QoS 配置
        qos = QoSProfile(depth=10)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # 订阅关节状态
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, qos
        )
        
        # 订阅目标位置（用于动态更新目标）
        self.sub_target = self.create_subscription(
            PoseStamped, '/target_pose', self._target_callback, qos
        )
        
        # 发布位置指令 (ForwardPositionController)
        self.pub_position_cmd = self.create_publisher(
            Float64MultiArray, self._runtime_control_topic, qos_reliable
        )
        
        # 发布末端位置
        self.pub_ee_pose = self.create_publisher(
            PoseStamped, '/ee_pose', qos
        )
        
        # 发布可视化标记
        self.pub_markers = self.create_publisher(
            MarkerArray, '/visualization_marker_array', qos
        )
        self.pub_top_traj_markers = self.create_publisher(
            MarkerArray, '/mppi_top_traj_markers', qos
        )
        self.pub_collision_sphere_markers = self.create_publisher(
            MarkerArray, '/collision_sphere_markers', qos
        )
        self._prev_collision_marker_count = 0
        
        self.get_logger().info(f'Gazebo Robot Interface 初始化完成')
        self.get_logger().info(f'  控制频率: {control_rate} Hz')
        self.get_logger().info(f'  关节数: {self.n_dof}')
        self.get_logger().info(f'  订阅: /joint_states, /target_pose')
        self.get_logger().info(f'  发布: {self._runtime_control_topic}, /ee_pose, /visualization_marker_array')
        self.get_logger().info('  额外发布: /mppi_top_traj_markers')
        self.get_logger().info('  额外发布: /collision_sphere_markers')
        self.get_logger().info(
            '  运行时关节控制约束: 仅通过 ros2_control/forward_position_controller 下发命令; '
            '除 Gazebo 初始姿态外, 不直接设置机械臂关节位置'
        )
    
    def _joint_state_callback(self, msg: JointState):
        """处理关节状态消息"""
        positions = np.zeros(self.n_dof)
        velocities = np.zeros(self.n_dof)
        
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    velocities[i] = msg.velocity[idx]
        
        with self._lock:
            self._positions = positions
            self._velocities = velocities
            self._state_received = True
            self._state_count += 1
    
    def _target_callback(self, msg: PoseStamped):
        """处理目标位置消息"""
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        with self._lock:
            self._target_pos = pos
        self.get_logger().info(f'收到目标位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')
    
    def get_joint_positions(self) -> np.ndarray:
        """获取当前关节位置"""
        with self._lock:
            if self._positions is None:
                return None
            return self._positions.copy()
    
    def get_joint_velocities(self) -> np.ndarray:
        """获取当前关节速度"""
        with self._lock:
            if self._velocities is None:
                return None
            return self._velocities.copy()
    
    def get_state(self) -> dict:
        """
        获取完整机器人状态
        
        Returns:
            dict: {'position': np.array, 'velocity': np.array, 'acceleration': np.array}
        """
        with self._lock:
            if not self._state_received:
                return None
            
            pos = self._positions.copy()
            vel = self._velocities.copy()
        
        # 估算加速度
        current_time = time.time()
        if self._prev_velocities is not None and self._prev_time is not None:
            dt = max(current_time - self._prev_time, 0.001)
            accel = (vel - self._prev_velocities) / dt
        else:
            accel = np.zeros(self.n_dof)
        
        self._prev_velocities = vel.copy()
        self._prev_time = current_time
        
        return {
            'position': pos,
            'velocity': vel,
            'acceleration': accel
        }
    
    def get_target_position(self) -> np.ndarray:
        """获取目标位置（来自 ROS2 话题，读取后清除）"""
        with self._lock:
            if self._target_pos is not None:
                pos = self._target_pos.copy()
                self._target_pos = None
                return pos
            return None
    
    def send_position_command(self, positions: np.ndarray):
        """发送关节位置指令"""
        if self._runtime_control_topic != FORWARD_POSITION_CMD_TOPIC:
            raise RuntimeError(
                f'非法控制话题: {self._runtime_control_topic}, 仅允许 {FORWARD_POSITION_CMD_TOPIC}'
            )
        msg = Float64MultiArray()
        msg.data = positions.tolist()
        self.pub_position_cmd.publish(msg)
        self._cmd_count += 1
    
    def publish_ee_pose(self, position: np.ndarray, orientation: np.ndarray = None):
        """发布末端位置"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        
        if orientation is not None:
            msg.pose.orientation.x = float(orientation[0])
            msg.pose.orientation.y = float(orientation[1])
            msg.pose.orientation.z = float(orientation[2])
            msg.pose.orientation.w = float(orientation[3])
        else:
            msg.pose.orientation.w = 1.0
        
        self.pub_ee_pose.publish(msg)

    def publish_live_goal_ee_markers(self, goal_pos: np.ndarray, ee_pos: np.ndarray):
        """发布轻量目标/末端 marker，供高频 RViz 刷新使用。"""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        goal_marker = Marker()
        goal_marker.header.frame_id = "world"
        goal_marker.header.stamp = stamp
        goal_marker.ns = "goal"
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(goal_pos[0])
        goal_marker.pose.position.y = float(goal_pos[1])
        goal_marker.pose.position.z = float(goal_pos[2])
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.06
        goal_marker.scale.y = 0.06
        goal_marker.scale.z = 0.06
        goal_marker.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.8)
        marker_array.markers.append(goal_marker)

        ee_marker = Marker()
        ee_marker.header.frame_id = "world"
        ee_marker.header.stamp = stamp
        ee_marker.ns = "ee"
        ee_marker.id = 1
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD
        ee_marker.pose.position.x = float(ee_pos[0])
        ee_marker.pose.position.y = float(ee_pos[1])
        ee_marker.pose.position.z = float(ee_pos[2])
        ee_marker.pose.orientation.w = 1.0
        ee_marker.scale.x = 0.05
        ee_marker.scale.y = 0.05
        ee_marker.scale.z = 0.05
        ee_marker.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.8)
        marker_array.markers.append(ee_marker)

        self.pub_markers.publish(marker_array)
    
    def publish_markers(
        self,
        obstacles: dict,
        goal_pos: np.ndarray,
        ee_pos: np.ndarray,
        collision_spheres=None,
    ):
        """
        发布可视化标记到 RViz
        
        Args:
            obstacles: 障碍物配置
            goal_pos: 目标位置 (世界坐标系)
            ee_pos: 末端位置 (世界坐标系)
        """
        marker_array = MarkerArray()
        marker_id = 0
        
        # 1. 目标标记（红色球）
        goal_marker = Marker()
        goal_marker.header.frame_id = "world"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "goal"
        goal_marker.id = marker_id
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(goal_pos[0])
        goal_marker.pose.position.y = float(goal_pos[1])
        goal_marker.pose.position.z = float(goal_pos[2])
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.06
        goal_marker.scale.y = 0.06
        goal_marker.scale.z = 0.06
        goal_marker.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.8)
        marker_array.markers.append(goal_marker)
        marker_id += 1
        
        # 2. 末端标记（绿色球）
        ee_marker = Marker()
        ee_marker.header.frame_id = "world"
        ee_marker.header.stamp = self.get_clock().now().to_msg()
        ee_marker.ns = "ee"
        ee_marker.id = marker_id
        ee_marker.type = Marker.SPHERE
        ee_marker.action = Marker.ADD
        ee_marker.pose.position.x = float(ee_pos[0])
        ee_marker.pose.position.y = float(ee_pos[1])
        ee_marker.pose.position.z = float(ee_pos[2])
        ee_marker.pose.orientation.w = 1.0
        ee_marker.scale.x = 0.05
        ee_marker.scale.y = 0.05
        ee_marker.scale.z = 0.05
        ee_marker.color = ColorRGBA(r=0.1, g=0.9, b=0.1, a=0.8)
        marker_array.markers.append(ee_marker)
        marker_id += 1
        
        # 3. 障碍物标记
        if obstacles:
            coll_objs = obstacles.get('world_model', {}).get('coll_objs', {})
            
            # 球体障碍物
            for name, params in coll_objs.get('sphere', {}).items():
                sphere_marker = Marker()
                sphere_marker.header.frame_id = "world"
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = "obstacles"
                sphere_marker.id = marker_id
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                
                pos = params.get('position', [0, 0, 0])
                radius = params.get('radius', 0.1)
                
                sphere_marker.pose.position.x = float(pos[0])
                sphere_marker.pose.position.y = float(pos[1])
                sphere_marker.pose.position.z = float(pos[2])
                sphere_marker.pose.orientation.w = 1.0
                sphere_marker.scale.x = radius * 2
                sphere_marker.scale.y = radius * 2
                sphere_marker.scale.z = radius * 2
                if name == 'dynamic_ball':
                    sphere_marker.color = ColorRGBA(r=0.2, g=0.35, b=0.9, a=0.75)
                else:
                    sphere_marker.color = ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.6)
                marker_array.markers.append(sphere_marker)
                marker_id += 1
            
            # 立方体障碍物
            for name, params in coll_objs.get('cube', {}).items():
                cube_marker = Marker()
                cube_marker.header.frame_id = "world"
                cube_marker.header.stamp = self.get_clock().now().to_msg()
                cube_marker.ns = "obstacles"
                cube_marker.id = marker_id
                cube_marker.type = Marker.CUBE
                cube_marker.action = Marker.ADD
                
                pose = params.get('pose', [0, 0, 0, 0, 0, 0, 1])
                dims = params.get('dims', [0.1, 0.1, 0.1])
                
                cube_marker.pose.position.x = float(pose[0])
                cube_marker.pose.position.y = float(pose[1])
                cube_marker.pose.position.z = float(pose[2])
                cube_marker.pose.orientation.x = float(pose[3])
                cube_marker.pose.orientation.y = float(pose[4])
                cube_marker.pose.orientation.z = float(pose[5])
                cube_marker.pose.orientation.w = float(pose[6])
                cube_marker.scale.x = float(dims[0])
                cube_marker.scale.y = float(dims[1])
                cube_marker.scale.z = float(dims[2])
                cube_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.8, a=0.6)
                marker_array.markers.append(cube_marker)
                marker_id += 1
        
        self.pub_markers.publish(marker_array)

        collision_marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        current_count = 0

        if collision_spheres:
            for marker_id, sphere in enumerate(collision_spheres):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = stamp
                marker.ns = "collision_spheres"
                marker.id = int(sphere.get("marker_id", marker_id))
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.frame_locked = True
                marker.pose.position.x = float(sphere["center_world"][0])
                marker.pose.position.y = float(sphere["center_world"][1])
                marker.pose.position.z = float(sphere["center_world"][2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = 2.0 * sphere["radius"]
                marker.scale.y = 2.0 * sphere["radius"]
                marker.scale.z = 2.0 * sphere["radius"]
                marker.color = ColorRGBA(r=1.0, g=0.78, b=0.12, a=0.45)
                collision_marker_array.markers.append(marker)
            current_count = len(collision_spheres)

        for marker_id in range(current_count, self._prev_collision_marker_count):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = stamp
            marker.ns = "collision_spheres"
            marker.id = marker_id
            marker.action = Marker.DELETE
            collision_marker_array.markers.append(marker)

        self._prev_collision_marker_count = current_count
        self.pub_collision_sphere_markers.publish(collision_marker_array)

    def publish_top_trajectories(self, top_trajs_world):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear_marker = Marker()
        clear_marker.header.frame_id = "world"
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if top_trajs_world is None or len(top_trajs_world) == 0:
            self.pub_top_traj_markers.publish(marker_array)
            return

        for traj_id, traj_points in enumerate(top_trajs_world[:5]):
            line_marker = Marker()
            line_marker.header.frame_id = "world"
            line_marker.header.stamp = stamp
            line_marker.ns = "mppi_top_trajs"
            line_marker.id = traj_id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.002
            line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.95)
            line_marker.points = []

            for point in np.asarray(traj_points, dtype=np.float64):
                if not np.all(np.isfinite(point)):
                    continue
                point_msg = Point()
                point_msg.x = float(point[0])
                point_msg.y = float(point[1])
                point_msg.z = float(point[2])
                line_marker.points.append(point_msg)

            if len(line_marker.points) >= 2:
                marker_array.markers.append(line_marker)

        self.pub_top_traj_markers.publish(marker_array)
    
    def is_connected(self) -> bool:
        """检查是否已连接到机器人"""
        return self._state_received
    
    def get_state_count(self) -> int:
        """获取接收的状态数量"""
        return self._state_count
    
    def get_cmd_count(self) -> int:
        """获取发送的指令数量"""
        return self._cmd_count


def _drain_mp_queue(mp_queue) -> None:
    if mp_queue is None:
        return
    while True:
        try:
            mp_queue.get_nowait()
        except queue.Empty:
            break
        except Exception:
            break


def _close_mp_queue(mp_queue) -> None:
    for method_name in ("close", "cancel_join_thread"):
        method = getattr(mp_queue, method_name, None)
        if callable(method):
            try:
                method()
            except Exception:
                pass


def _shutdown_control_process(control_process, join_timeout: float = 2.0) -> None:
    if control_process is None:
        return

    control_process.done = True
    _drain_mp_queue(getattr(control_process, "result_queue", None))

    done_message = {"state": None, "dt": None, "done": True, "params": None}
    opt_queue = getattr(control_process, "opt_queue", None)
    if opt_queue is not None:
        try:
            opt_queue.put_nowait(done_message)
        except queue.Full:
            _drain_mp_queue(opt_queue)
            try:
                opt_queue.put_nowait(done_message)
            except Exception:
                pass
        except Exception:
            pass

    opt_process = getattr(control_process, "opt_process", None)
    if opt_process is not None:
        opt_process.join(timeout=join_timeout)
        if opt_process.is_alive():
            _log("后台 MPC 进程未在超时内退出，强制终止...")
            opt_process.terminate()
            opt_process.join(timeout=join_timeout)

    if opt_queue is not None:
        _close_mp_queue(opt_queue)
    result_queue = getattr(control_process, "result_queue", None)
    if result_queue is not None:
        _close_mp_queue(result_queue)


class CollisionSphereVisualizer:
    def __init__(self, robot_collision_params: dict):
        sphere_config = os.path.expanduser(robot_collision_params["collision_spheres"])
        if not os.path.isabs(sphere_config):
            sphere_config = join_path(get_mpc_configs_path(), sphere_config)

        with open(sphere_config) as f:
            sphere_params = yaml.safe_load(f)

        self.link_names = list(robot_collision_params["link_objs"])
        raw_spheres_by_link = sphere_params["collision_spheres"]
        self.spheres_by_link = {}
        marker_id = 0
        for link_name in self.link_names:
            self.spheres_by_link[link_name] = []
            for sphere in raw_spheres_by_link.get(link_name, []):
                sphere_entry = dict(sphere)
                sphere_entry["marker_id"] = marker_id
                self.spheres_by_link[link_name].append(sphere_entry)
                marker_id += 1
        self.total_sphere_count = marker_id

    def get_world_spheres(
        self,
        link_pos_robot: np.ndarray,
        link_rot_robot: np.ndarray,
        robot_pos_world: np.ndarray,
        robot_quat_xyzw: np.ndarray,
    ):
        world_spheres = []
        for link_idx, link_name in enumerate(self.link_names):
            link_pos = link_pos_robot[link_idx]
            link_rot = link_rot_robot[link_idx]
            for sphere in self.spheres_by_link.get(link_name, []):
                center_local = np.asarray(sphere["center"], dtype=np.float64)
                center_robot = link_rot @ center_local + link_pos
                center_world = transform_point(robot_pos_world, robot_quat_xyzw, center_robot)
                world_spheres.append(
                    {
                        "marker_id": int(sphere["marker_id"]),
                        "link_name": link_name,
                        "center_world": center_world,
                        "radius": float(sphere["radius"]),
                    }
                )
        return world_spheres


class TallGazeboRobotInterface(GazeboRobotInterface):
    def __init__(self, joint_names: list, control_rate: float = 50.0):
        super().__init__(joint_names, control_rate=control_rate)


def _compute_link_poses_robot_frame(rollout_fn, q: np.ndarray, dq: np.ndarray, tensor_args: dict):
    q_tensor = torch.as_tensor(q, **tensor_args).unsqueeze(0)
    dq_tensor = torch.as_tensor(dq, **tensor_args).unsqueeze(0)
    robot_model = rollout_fn.dynamics_model.robot_model
    robot_model.compute_fk_and_jacobian(
        q_tensor,
        dq_tensor,
        rollout_fn.exp_params["model"]["ee_link_name"],
    )

    link_pos_robot = []
    link_rot_robot = []
    for link_name in rollout_fn.dynamics_model.link_names:
        link_pos, link_rot = robot_model.get_link_pose(link_name)
        link_pos_robot.append(link_pos[0].detach().cpu().numpy())
        link_rot_robot.append(link_rot[0].detach().cpu().numpy())

    return np.stack(link_pos_robot, axis=0), np.stack(link_rot_robot, axis=0)


