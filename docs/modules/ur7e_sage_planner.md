# ur7e_sage_planner：独立 SAGE 仿真规划与伺服

[返回项目文档索引](../README.md)

## 职责和边界

依赖外部 storm_kit，控制核心不在本仓库。固定场景配置参与碰撞规划；这是与感知地图规划不同的基线入口。

## 输入、处理与输出

**数据链路：** 关节反馈 + 目标 + 包内模型/障碍配置 → SAGE_MPPI → 可选细化/JointServoExecutor → 仿真控制器。

resources 解析资源；clean entry 构建 task 与控制器；循环读取反馈优化；目标改变时严格重置时序；可选执行器以更高频率平滑输出；独立记录器核查运动平滑度。

## 目录结构

```text
ur7e_sage_planner/
├── assets/
├── config/
│   ├── collision_world_gazebo_tall.yml
│   ├── initial_positions.yaml
│   ├── reach_static_clean.rviz
│   ├── ur7e_collision_spheres.yml
│   ├── ur7e_reacher_gazebo_tall_sage_clean.yml
│   └── ur7e_robot_gazebo.yml
├── launch/
│   └── planning.launch.py
├── licenses/
│   ├── STORM-LICENSE
│   └── UR-Description-LICENSE
├── scripts/
│   ├── sage_planner
│   └── verify_planning
├── ur7e_sage_planner/
│   ├── __init__.py
│   ├── deployment_refinement.py
│   ├── entrypoint.py
│   ├── gazebo_interface.py
│   ├── gazebo_obstacle_utils.py
│   ├── joint_servo_executor.py
│   ├── reach_static_ur7e_tall.py
│   ├── reach_static_ur7e_tall_servo_executor.py
│   ├── record_joint_smoothness.py
│   └── resources.py
├── validation/
├── weights/
├── CMakeLists.txt
├── README.md
├── THIRD_PARTY.md
└── package.xml
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/planning.launch.py`

[查看源码](../../src/ur7e_sage_planner/launch/planning.launch.py)

选择解释器与规划入口，可选启动场景及可视化。

源码说明：Start the packaged planner, optionally with the packaged tall D435 scene.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_planner(context)` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `generate_launch_description()` | 26 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('start_scene', default_value='true')
DeclareLaunchArgument('gazebo_gui', default_value='true')
DeclareLaunchArgument('rviz', default_value='true')
DeclareLaunchArgument('execution_layer', default_value='servo')
DeclareLaunchArgument('executor_frequency', default_value='200')
DeclareLaunchArgument('max_joint_speed', default_value='0.6')
DeclareLaunchArgument('max_joint_acceleration', default_value='0.0')
DeclareLaunchArgument('max_steps', default_value='0')
DeclareLaunchArgument('planner_args', default_value='')
```

### `scripts/sage_planner`

[查看源码](../../src/ur7e_sage_planner/scripts/sage_planner)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
SAGE_ENV_PREFIX="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
if [[ ! -x "$SAGE_ENV_PREFIX/bin/python" ]]; then
    echo "whole_control Python not found; set SAGE_CONDA_PREFIX to its conda prefix." >&2
    exit 1
fi
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
export CONDA_PREFIX="$SAGE_ENV_PREFIX"
export PATH="$SAGE_ENV_PREFIX/bin:$PATH"
exec "$SAGE_ENV_PREFIX/bin/python" -B -m ur7e_sage_planner.entrypoint "$@"
```

### `scripts/verify_planning`

[查看源码](../../src/ur7e_sage_planner/scripts/verify_planning)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

源码说明：Verify a running Gazebo planning loop and, optionally, send a second target.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 16 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 33 |
| `create_subscription` | `PoseStamped` | `'/ee_pose'` | 29 |
| `create_subscription` | `JointState` | `'/joint_states'` | 30 |
| `create_subscription` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 31 |

启动/命令行参数声明：

```python
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--timeout', type=float, default=120)
parser.add_argument('--threshold', type=float, default=0.02)
parser.add_argument('--retarget', action='store_true', help='after the default goal, command [0.5, 0.0, 0.45]')
```

### `ur7e_sage_planner/__init__.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

源码说明：Packaged clean SAGE planning and Gazebo servo execution.

### `ur7e_sage_planner/deployment_refinement.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/deployment_refinement.py)

到点保持、近目标参数细化、笛卡尔修正和停滞恢复等可选策略，与 SAGE 控制核心分开。

源码说明：Deployment refinement helpers for SAGE Gazebo examples.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_reset_controller_distribution(mpc)` | 20 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `GoalHoldController` | 26 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L27）、`reset`（L53）、`force_activate`（L59）、`update`（L65） |
| 类 `NearGoalRefinementController` | 95 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L96）、`_get_goal_position_weight`（L137）、`_set_goal_position_weight`（L150）、`_get_scalar_cost_weight`（L165）、`_set_scalar_cost_weight`（L176）、`_apply_refine_params`（L193）、`_restore_nominal_params`（L215）、`reset`（L232）、`update`（L236） |
| 类 `CartesianGoalRefiner` | 255 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L256）、`reset`（L292）、`update`（L299）、`compute_command`（L316）、`smooth_command`（L349）、`postprocess_command`（L375） |
| 类 `StallMonitor` | 402 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L403）、`reset`（L420）、`_history_motion`（L424）、`update`（L432）、`should_recover`（L435） |
| 类 `DeploymentRefinementStack` | 465 | Runtime wrapper for deployment-specific heuristics. |
| 方法 | — | `__init__`（L473）、`_apply_exploration_boost`（L580）、`_restore_exploration_boost`（L595）、`_sync_reset`（L603）、`reset_all`（L609）、`on_goal_changed`（L628）、`update_modes`（L642）、`maybe_get_override_command`（L697）、`maybe_trigger_recovery`（L764） |

### `ur7e_sage_planner/entrypoint.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/entrypoint.py)

命令行入口与运行环境准备，将参数转交节点或规划运行模式。

源码说明：Run the copied planner with the selected interpreter and a writable run directory.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 9 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('--entry', choices=['servo', 'direct', 'smoke', 'record'], default='servo')
```

### `ur7e_sage_planner/gazebo_interface.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/gazebo_interface.py)

接收关节/目标并发布控制器位置命令、末端、碰撞球及轨迹标记，管理控制工作进程退出。

源码说明：ROS/Gazebo interface and visualization extracted from the original simulation helpers.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_log(message)` | 19 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `transform_point(position, orientation_xyzw, point)` | 22 | 将点从机器人坐标系变换到世界坐标系 |
| `inv_transform_point(position, orientation_xyzw, point)` | 28 | 将点从世界坐标系变换到机器人坐标系 |
| 类 `GazeboRobotInterface` | 34 | Gazebo 机器人 ROS2 接口 |
| 方法 | — | `__init__`（L47）、`_joint_state_callback`（L119）、`_target_callback`（L137）、`get_joint_positions`（L148）、`get_joint_velocities`（L155）、`get_state`（L162）、`get_target_position`（L193）、`send_position_command`（L202）、`publish_ee_pose`（L213）、`publish_live_goal_ee_markers`（L232）、`publish_markers`（L273）、`publish_top_trajectories`（L426）、`is_connected`（L467）、`get_state_count`（L471）、`get_cmd_count`（L475） |
| `_drain_mp_queue(mp_queue)` | 480 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_close_mp_queue(mp_queue)` | 492 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_shutdown_control_process(control_process, join_timeout: float=2.0)` | 502 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `CollisionSphereVisualizer` | 538 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L539）、`get_world_spheres`（L560） |
| 类 `TallGazeboRobotInterface` | 586 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L587） |
| `_compute_link_poses_robot_frame(rollout_fn, q: np.ndarray, dq: np.ndarray, tensor_args: dict)` | 591 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`storm_kit.util_file`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/joint_states'` | 76 |
| `create_subscription` | `PoseStamped` | `'/target_pose'` | 81 |
| `create_publisher` | `Float64MultiArray` | `self._runtime_control_topic` | 86 |
| `create_publisher` | `PoseStamped` | `'/ee_pose'` | 91 |
| `create_publisher` | `MarkerArray` | `'/visualization_marker_array'` | 96 |
| `create_publisher` | `MarkerArray` | `'/mppi_top_traj_markers'` | 99 |
| `create_publisher` | `MarkerArray` | `'/collision_sphere_markers'` | 102 |

### `ur7e_sage_planner/gazebo_obstacle_utils.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/gazebo_obstacle_utils.py)

将盒、球等场景原语转换为 SDF，并通过 Gazebo SpawnEntity/DeleteEntity 服务管理模型。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `iter_primitive_obstacles(world_params: dict, include_ground: bool=False)` | 6 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `count_primitive_obstacles(world_params: dict, include_ground: bool=False)` | 30 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_wait_for_service(client, timeout_sec: float)` | 41 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_wait_for_future(future, timeout_sec: float)` | 49 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_build_box_sdf(model_name: str, dims: list[float])` | 65 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_build_sphere_sdf(model_name: str, radius: float)` | 94 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `spawn_gazebo_obstacles(node, world_params: dict, model_prefix: str='sim_tall', include_ground: bool=False, service_timeout_sec: float=8.0)` | 123 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `SpawnEntity` | `'/spawn_entity'` | 130 |
| `create_client` | `DeleteEntity` | `'/delete_entity'` | 131 |

### `ur7e_sage_planner/joint_servo_executor.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/joint_servo_executor.py)

独立 JointServoExecutor 按速度/可选加速度上限逼近 planner 目标；GazeboBackend 发指令，ServoExecutionLogger 记录 CSV。

源码说明：Joint-space smoothing executor for SAGE-MPPI command outputs.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `parse_joint_limit(value: str \| float, n_dof: int, name: str)` | 24 | Parse a scalar or comma-separated joint limit into a vector. |
| 类 `GazeboBackend` | 41 | Publish joint position commands to the existing Gazebo ros2_control topic. |
| 方法 | — | `__init__`（L44）、`publish`（L53） |
| 类 `ServoExecutionLogger` | 63 | CSV logger for planner target, smoothed command, and actual joint state. |
| 方法 | — | `__init__`（L66）、`path`（L91）、`log`（L94）、`close`（L136）、`_vector_or_nan`（L141）、`_format_float`（L152） |
| 类 `JointServoExecutor` | 161 | High-frequency bounded-rate joint command executor. |
| 方法 | — | `__init__`（L164）、`start`（L205）、`set_target`（L221）、`stop`（L252）、`get_current_command`（L257）、`_run_loop`（L261）、`_step_toward`（L307） |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `Float64MultiArray` | `controller_topic` | 50 |

### `ur7e_sage_planner/reach_static_ur7e_tall.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/reach_static_ur7e_tall.py)

clean SAGE 主链：构建 SageReacherTask/SAGE_MPPI、处理目标变化、可选细化，提供 offline-smoke 和 Gazebo 两种运行方式。

源码说明：UR7e SAGE clean pipeline entry for the Gazebo tall scene.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_build_tensor_args(use_cuda: bool)` | 47 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_load_robot_and_world_params()` | 55 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_drain_mp_queue(mp_queue)` | 63 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_reset_control_process_timing_strict(control_process, t_step: float, control_dt: float)` | 75 | Reset the ControlProcess timeline after abrupt goal changes. |
| `_recover_command_strict(mpc, t_step: float, state: dict, control_dt: float)` | 92 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_get_robot_pose_world(robot_params: dict)` | 97 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_make_clean_task(tensor_args: dict)` | 105 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_get_execution_mode(mpc)` | 116 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_apply_execution_mode(mpc)` | 124 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_configure_default_goal(mpc, robot_pos_world, robot_quat_xyzw, inv_transform_point_fn)` | 135 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_get_clean_top_ee_trajs_world(mpc, robot_pos_world: np.ndarray, robot_quat_xyzw: np.ndarray, current_ee_pos_world: np.ndarray, transform_point_fn, max_trajs: int=5)` | 151 | Return world-frame top trajectories with the current EE position prepended. |
| `_apply_refinement_overrides(refinement_cfg: dict, args)` | 211 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_run_offline_smoke(args)` | 237 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_run_gazebo_main(args)` | 308 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 815 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`storm_kit.mpc.control.sage_mppi`, `storm_kit.mpc.task.sage_reacher_task`, `ur7e_sage_planner.deployment_refinement`, `ur7e_sage_planner.gazebo_interface`, `ur7e_sage_planner.gazebo_obstacle_utils`, `ur7e_sage_planner.resources`。

启动/命令行参数声明：

```python
parser.add_argument('--cuda', action='store_true', default=True, help='使用 CUDA 加速 (默认: True)')
parser.add_argument('--no-cuda', dest='cuda', action='store_false', help='禁用 CUDA')
parser.add_argument('--rate', type=float, default=50.0, help='控制频率 Hz (默认: 50)')
parser.add_argument('--max-steps', type=int, default=0, help='Gazebo 模式下最大控制步数，<=0 表示不限')
parser.add_argument('--viz-update-every', type=int, default=5, help='可视化刷新步数间隔')
parser.add_argument('--offline-smoke', action='store_true', help='运行无需 ROS2/Gazebo 的 clean correctness smoke test')
parser.add_argument('--enable-deployment-refinement', action='store_true', help='显式启用 deployment refinement')
parser.add_argument('--disable-deployment-refinement', action='store_true', help='显式禁用 deployment refinement')
parser.add_argument('--enable-cartesian-refinement', action='store_true', help='显式启用 Cartesian refinement')
parser.add_argument('--disable-cartesian-refinement', action='store_true', help='显式禁用 Cartesian refinement')
parser.add_argument('--spawn-obstacles', action='store_true', help='create or replace sage_clean obstacles when using an empty scene')
```

### `ur7e_sage_planner/reach_static_ur7e_tall_servo_executor.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/reach_static_ur7e_tall_servo_executor.py)

复用 clean planner，将 q_next 交给可切换的关节伺服层后输出 Gazebo 指令。

源码说明：Clean SAGE tall Gazebo entry with a switchable joint execution layer.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_log(message: str)` | 64 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_parse_joint_names(value: str)` | 68 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_default_log_dir()` | 75 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_run_gazebo_main(args)` | 80 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 579 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`storm_kit.mpc.control.sage_mppi`, `ur7e_sage_planner.deployment_refinement`, `ur7e_sage_planner.gazebo_interface`, `ur7e_sage_planner.gazebo_obstacle_utils`, `ur7e_sage_planner.joint_servo_executor`, `ur7e_sage_planner.reach_static_ur7e_tall`, `ur7e_sage_planner.resources`。

启动/命令行参数声明：

```python
parser.add_argument('--cuda', action='store_true', default=True, help='use CUDA when available')
parser.add_argument('--no-cuda', dest='cuda', action='store_false', help='disable CUDA')
parser.add_argument('--rate', type=float, default=50.0, help='MPPI/control loop rate in Hz')
parser.add_argument('--max-steps', type=int, default=0, help='maximum MPPI loop steps; <=0 means unlimited')
parser.add_argument('--viz-update-every', type=int, default=5, help='visualization update interval in MPPI steps')
parser.add_argument('--enable-deployment-refinement', action='store_true', help='force enable deployment refinement')
parser.add_argument('--disable-deployment-refinement', action='store_true', help='force disable deployment refinement')
parser.add_argument('--enable-cartesian-refinement', action='store_true', help='force enable Cartesian refinement')
parser.add_argument('--disable-cartesian-refinement', action='store_true', help='force disable Cartesian refinement')
parser.add_argument('--execution-layer', choices=('servo', 'direct'), default='servo', help='output execution layer')
parser.add_argument('--executor-frequency', type=float, default=200.0, help='JointServoExecutor output frequency in Hz')
parser.add_argument('--max-joint-speed', default='0.6', help='scalar or comma-separated rad/s joint speed limit')
parser.add_argument('--max-joint-acceleration', default='0.0', help='0 disables acceleration limiting; otherwise scalar or comma-separated rad/s^2')
parser.add_argument('--controller-topic', default=DEFAULT_CONTROLLER_TOPIC, help='Gazebo ros2_control command topic')
parser.add_argument('--joint-names', type=_parse_joint_names, default=DEFAULT_JOINT_NAMES, help='comma-separated joint names')
parser.add_argument('--executor-log-dir', default=_default_log_dir(), help='directory for servo_execution_log.csv')
parser.add_argument('--executor-log-every', type=int, default=1, help='log every N executor ticks')
parser.add_argument('--spawn-obstacles', action='store_true', help='create or replace sage_clean obstacles when using an empty scene')
```

### `ur7e_sage_planner/record_joint_smoothness.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/record_joint_smoothness.py)

记录真实仿真反馈与下发命令，按时间计算速度、加速度、jerk 和平滑度并导出表格/图。

源码说明：Record UR7e joint smoothness data from the clean SAGE tall Gazebo run.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_safe_name(name: str)` | 45 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_finite_or_none(value)` | 49 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_rms(values: np.ndarray)` | 58 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_max_abs(values: np.ndarray)` | 65 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_integral_square(values: np.ndarray, t: np.ndarray)` | 72 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_derivative(values: np.ndarray, t: np.ndarray)` | 79 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_choose_time(rows: list[dict], min_dt: float)` | 102 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_rows_to_array(rows: list[dict], key: str, keep: np.ndarray, n_joints: int)` | 122 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_compute_metrics(label: str, t: np.ndarray, position: np.ndarray, joint_names: list[str], velocity_msg: np.ndarray \| None=None)` | 128 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_write_csv(path: Path, header: list[str], rows: Iterable[Iterable])` | 201 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_format_csv_value(value)` | 208 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_write_joint_csv(output_dir: Path, joint_names: list[str], t: np.ndarray, wall_time: np.ndarray, ros_time: np.ndarray, position: np.ndarray, velocity_msg: np.ndarray)` | 216 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_write_command_csv(output_dir: Path, joint_names: list[str], t: np.ndarray, wall_time: np.ndarray, command_position: np.ndarray)` | 244 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_plot_series(output_path: Path, title: str, ylabel: str, t: np.ndarray, values: np.ndarray, joint_names: list[str], x_min: float \| None=None, x_max: float \| None=None)` | 270 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_write_report(path: Path, summary: dict)` | 302 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_fmt(value)` | 342 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_per_joint_table(per_joint: dict)` | 348 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `JointSmoothnessRecorder` | 368 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L369）、`_joint_state_callback`（L386）、`_command_callback`（L410）、`save`（L418）、`_write_plots`（L506）、`_write_command_plots`（L555） |
| `_parse_joint_names(value: str)` | 609 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 616 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `args.joint_state_topic` | 382 |
| `create_subscription` | `Float64MultiArray` | `args.command_topic` | 383 |

启动/命令行参数声明：

```python
parser.add_argument('--output-dir', required=True, help='directory for CSV, plots, and summaries')
parser.add_argument('--joint-state-topic', default='/joint_states')
parser.add_argument('--command-topic', default='/forward_position_controller/commands')
parser.add_argument('--joint-names', type=_parse_joint_names, default=DEFAULT_JOINT_NAMES)
parser.add_argument('--min-dt', type=float, default=1e-05, help='minimum accepted sample interval')
parser.add_argument('--no-plots', action='store_true', help='write CSV/JSON/Markdown only')
```

### `ur7e_sage_planner/resources.py`

[查看源码](../../src/ur7e_sage_planner/ur7e_sage_planner/resources.py)

解析 package:// 资源为安装路径，生成当前运行专属 YAML；读取 SAGE_RUN_DIR/SAGE_TASK_FILE。

源码说明：Resolve installed resources without reading STORM's examples/content/weights.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_resolve(value)` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_resolved_yaml(source, name)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_sage_planner/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/collision_world_gazebo_tall.yml](../../src/ur7e_sage_planner/config/collision_world_gazebo_tall.yml) | 配置分区/键： |
| [config/initial_positions.yaml](../../src/ur7e_sage_planner/config/initial_positions.yaml) | 配置分区/键： |
| [config/reach_static_clean.rviz](../../src/ur7e_sage_planner/config/reach_static_clean.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [config/ur7e_collision_spheres.yml](../../src/ur7e_sage_planner/config/ur7e_collision_spheres.yml) | 配置分区/键： |
| [config/ur7e_reacher_gazebo_tall_sage_clean.yml](../../src/ur7e_sage_planner/config/ur7e_reacher_gazebo_tall_sage_clean.yml) | 配置分区/键： |
| [config/ur7e_robot_gazebo.yml](../../src/ur7e_sage_planner/config/ur7e_robot_gazebo.yml) | 配置分区/键： |
| [package.xml](../../src/ur7e_sage_planner/package.xml) | ROS 包名、依赖及构建类型声明。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_sage_planner/README.md)
- [THIRD_PARTY.md](../../src/ur7e_sage_planner/THIRD_PARTY.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
- `licenses/`：第三方许可证。
