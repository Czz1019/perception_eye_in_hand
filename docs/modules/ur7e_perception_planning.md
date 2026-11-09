# ur7e_perception_planning：在线感知与受保护 SAGE 集成

[返回项目文档索引](../README.md)

## 职责和边界

有 online/demo/adaptive 等配置与路径；demo 的预置观察走廊/装置先验必须与纯感知地图区分。控制发布频率不等于地图更新或优化频率。

## 输入、处理与输出

**数据链路：** 深度/TF → PlanningMapper → 原子 PlanningMap → EsdfTask/SAGE → CPU 整臂检查 → 低速仿真命令。

mapper 基于射线与原生 ESDF 更新来源和保守距离；Planner 检查反馈/地图时效并更新 SAGE；伺服线程重新检查短连接；combined 模式以内存快照连接两者并协调 GPU。

## 目录结构

```text
ur7e_perception_planning/
├── config/
│   ├── demo.yaml
│   ├── demo_scan.yaml
│   └── online.yaml
├── launch/
│   ├── bringup.launch.py
│   └── scene.launch.py
├── msg/
│   └── PlanningMap.msg
├── rviz/
│   └── online.rviz
├── scripts/
│   └── perception_planning
├── test/
│   ├── benchmark_esdf_pipeline.py
│   ├── report_benchmark.py
│   ├── run_benchmark.py
│   ├── run_learning_comparison.py
│   ├── test_adaptive.py
│   ├── test_cuda_edt.py
│   ├── test_demo.py
│   ├── test_entrypoint.py
│   ├── test_field_schedule.py
│   ├── test_integration.py
│   ├── validate_demo.py
│   └── validate_sim.py
├── ur7e_perception_planning_core/
│   ├── __init__.py
│   ├── adaptive_demo.py
│   ├── demo.py
│   ├── edt.py
│   ├── entrypoint.py
│   ├── frames.py
│   ├── geometry.py
│   ├── grid.py
│   ├── mapper.py
│   ├── planner.py
│   ├── routing.py
│   ├── runtime.py
│   ├── sage.py
│   ├── self_filter.py
│   └── speed.py
├── validation/
├── CMakeLists.txt
├── README.md
├── ToDoList.md
└── package.xml
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/bringup.launch.py`

[查看源码](../../src/ur7e_perception_planning/launch/bringup.launch.py)

组装应用进程、配置路径和可选可视化；参数可改变实际启动路径。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 11 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('start_scene', default_value='true')
DeclareLaunchArgument('gazebo_gui', default_value='false', description='Open gzclient; headless by default to avoid remote model-list requests')
DeclareLaunchArgument('run_rviz', default_value='true')
DeclareLaunchArgument('config', default_value=str(share / 'config/demo.yaml'))
```

### `launch/scene.launch.py`

[查看源码](../../src/ur7e_perception_planning/launch/scene.launch.py)

复用仿真场景和初始姿态，并降低渲染负载以便在线规划。

源码说明：Same scene and initial pose, reduced sensor rendering load for online planning.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_remove_comments(node)` | 16 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_setup(context)` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `generate_launch_description()` | 64 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('gazebo_gui', default_value='false', description='Open the Gazebo GUI')
```

### `scripts/perception_planning`

[查看源码](../../src/ur7e_perception_planning/scripts/perception_planning)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
PP_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$PP_ENV"
export PATH="$PP_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=2
export MKL_NUM_THREADS=2
exec "$PP_ENV/bin/python" -B -m ur7e_perception_planning_core.entrypoint "$@"
```

### `test/benchmark_esdf_pipeline.py`

[查看源码](../../src/ur7e_perception_planning/test/benchmark_esdf_pipeline.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Same-data CPU/CUDA microbenchmarks; no ROS devices or obstacle truth in production.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `distribution(values)` | 20 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 24 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.types`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_perception_planning_core.edt`, `ur7e_perception_planning_core.grid`。

启动/命令行参数声明：

```python
p.add_argument('--output', required=True)
p.add_argument('--reference', required=True)
```

### `test/report_benchmark.py`

[查看源码](../../src/ur7e_perception_planning/test/report_benchmark.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline evidence summary and trajectory plot from benchmark feedback logs.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `summarize(data)` | 12 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 35 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`。

启动/命令行参数声明：

```python
parser.add_argument('directory', type=Path)
parser.add_argument('--plot', type=Path)
```

### `test/run_benchmark.py`

[查看源码](../../src/ur7e_perception_planning/test/run_benchmark.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Own a cold simulation, test default reach and a wall-crossing retarget, clean up.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `wall_detour_check(data, goal)` | 18 | Fixture truth is confined to validation, never supplied to the planner. |
| `main()` | 59 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 81 |

启动/命令行参数声明：

```python
parser.add_argument('--tag', required=True)
parser.add_argument('--no-rviz', action='store_true')
parser.add_argument('--save-maps', action='store_true')
parser.add_argument('--repeat-wall', action='store_true', help='Return and cross again using the accumulated map')
parser.add_argument('--duration', type=float, default=240.0)
parser.add_argument('--wall-goal', type=float, nargs=3, default=[0.45, 0.4, 0.55])
```

### `test/run_learning_comparison.py`

[查看源码](../../src/ur7e_perception_planning/test/run_learning_comparison.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Headless cold-start A->B/wall benchmark, optionally importing an untouched source snapshot.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 20 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 40 |

启动/命令行参数声明：

```python
p.add_argument('--output', required=True)
p.add_argument('--source-root', help='Directory containing backed-up ur7e_nvblox and ur7e_perception_planning')
p.add_argument('--esdf-hz', type=float, default=5.0)
p.add_argument('--duration', type=float, default=180.0)
```

### `test/test_adaptive.py`

[查看源码](../../src/ur7e_perception_planning/test/test_adaptive.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_batched_swept_geometry_matches_scalar_fk()` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_guide_refines_sampling_without_reducing_physical_margin()` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_cached_cropped_edt_matches_full_grid_and_reacts_to_obstacles()` | 38 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_route_goes_around_wall_when_direct_edge_is_blocked()` | 64 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_speed_requires_stable_maps_and_drops_on_latency()` | 84 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_ray_self_filter_masks_noisy_robot_returns_but_preserves_foreground()` | 95 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_mapping_priority_and_reentrant_gpu_lock()` | 105 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_exact_time_queue_uses_ready_older_image_without_latest_tf_fallback()` | 124 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_demo_rejects_sky_facing_path_even_in_free_space()` | 138 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_recovery_does_not_escape_below_execution_clearance()` | 147 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.demo`, `ur7e_perception_planning_core.frames`, `ur7e_perception_planning_core.geometry`, `ur7e_perception_planning_core.grid`, `ur7e_perception_planning_core.routing`, `ur7e_perception_planning_core.runtime`, `ur7e_perception_planning_core.self_filter`, `ur7e_perception_planning_core.speed`。

### `test/test_cuda_edt.py`

[查看源码](../../src/ur7e_perception_planning/test/test_cuda_edt.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_exact_cuda_edt_matches_scipy_obstacles_and_boundaries()` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.edt`。

### `test/test_demo.py`

[查看源码](../../src/ur7e_perception_planning/test/test_demo.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Demo-specific commissioning and SAGE velocity action contracts.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_scan_corridor_commissioned_against_fixture()` | 17 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_goal_ik_rejects_unobserved_endpoint()` | 40 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_guide_rejects_obstruction_between_valid_endpoints()` | 49 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_goal_ik_changes_redundant_posture_to_clear_intermediate_path()` | 60 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_far_measured_return_proves_free_ray_without_inventing_near_surface()` | 83 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_braking_proposal_does_not_overshoot_nearby_observation_pose()` | 94 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_sage_velocity_actions_and_joint_guide(tmp_path)` | 109 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_direct_map_handoff_keeps_freshness_and_epoch_guards()` | 139 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.demo`, `ur7e_perception_planning_core.geometry`, `ur7e_perception_planning_core.grid`, `ur7e_perception_planning_core.planner`, `ur7e_perception_planning_core.sage`。

### `test/test_entrypoint.py`

[查看源码](../../src/ur7e_perception_planning/test/test_entrypoint.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Distinguish a DDS take interrupted by shutdown from a live runtime fault.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_take_error_during_shutdown_and_live_context(monkeypatch, context_alive)` | 12 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core`。

### `test/test_field_schedule.py`

[查看源码](../../src/ur7e_perception_planning/test/test_field_schedule.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_deferred_growth_is_never_more_optimistic_and_blocking_is_immediate()` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_deferred_free_growth_refreshes_after_deadline()` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.grid`。

### `test/test_integration.py`

[查看源码](../../src/ur7e_perception_planning/test/test_integration.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Contracts that can otherwise silently invert or bypass collision checking.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `geometry()` | 21 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `initial()` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `task(tmp_path_factory)` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_cpu_gpu_fk_and_calibrated_payload(task, geometry, initial)` | 36 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_grid_order_sign_outside_and_gpu_agreement(task)` | 48 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_sage_rollout_really_consumes_esdf(task, geometry, initial)` | 60 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_unknown_is_not_free_and_obstacle_overrides_prior()` | 76 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_depth_rays_do_not_clear_occlusion_or_invalid_pixels()` | 90 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_atomic_map_rejects_invalid_dimensions_frame_and_nan()` | 100 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_lossless_map_transport_and_corrupt_payload()` | 110 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_guard_checks_middle_of_segment()` | 121 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_commissioned_initial_prior_against_static_scene(geometry, initial)` | 135 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning.msg`, `ur7e_perception_planning_core.geometry`, `ur7e_perception_planning_core.grid`, `ur7e_perception_planning_core.planner`, `ur7e_perception_planning_core.sage`。

### `test/validate_demo.py`

[查看源码](../../src/ur7e_perception_planning/test/validate_demo.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Observe an A-to-B run; scene truth is an independent test oracle only.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `state_cb(msg)` | 21 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `status_cb(msg)` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `map_cb(msg)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `stats_cb(msg)` | 45 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning.msg`, `ur7e_perception_planning_core.geometry`, `ur7e_perception_planning_core.grid`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/joint_states'` | 43 |
| `create_subscription` | `String` | `'/perception_planning/status'` | 44 |
| `create_subscription` | `String` | `'/ur7e_nvblox/stats'` | 51 |
| `create_subscription` | `PlanningMap` | `'/perception_planning/map'` | 52 |

启动/命令行参数声明：

```python
p.add_argument('--output', required=True)
p.add_argument('--duration', type=float, default=600)
p.add_argument('--goal', type=float, nargs=3, default=[0.5, 0, 0.45])
p.add_argument('--save-maps', action='store_true')
```

### `test/validate_sim.py`

[查看源码](../../src/ur7e_perception_planning/test/validate_sim.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Gazebo integration exercise. Publishes goals only with --exercise.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 26 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_perception_planning.msg`, `ur7e_perception_planning_core.geometry`, `ur7e_perception_planning_core.grid`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/joint_states'` | 75 |
| `create_subscription` | `String` | `'/perception_planning/status'` | 76 |
| `create_subscription` | `PlanningMap` | `'/perception_planning/map'` | 77 |
| `create_publisher` | `PoseStamped` | `'/target_pose'` | 78 |
| `create_client` | `service_type` | `name` | 127 |

启动/命令行参数声明：

```python
parser.add_argument('--exercise', action='store_true')
parser.add_argument('--faults', action='store_true')
parser.add_argument('--output', required=True)
parser.add_argument('--duration', type=float, default=60.0)
```

### `ur7e_perception_planning_core/__init__.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

源码说明：Additive perception/planning integration; existing packages are not patched.

### `ur7e_perception_planning_core/adaptive_demo.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/adaptive_demo.py)

持续观测直到找到可检查路线，再使用在线 SAGE 跟踪。

源码说明：Observe only until a checked route exists, then track it with online SAGE.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `run_demo(node, task)` | 11 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_perception_planning_core/demo.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/demo.py)

固定仿真装置的观察准备与 SAGE 到点 demo，包含明确的观察走廊先验和独立路径检查。

源码说明：Static-fixture observation preparation and SAGE-only point reaching.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `scan_configuration()` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `scan_samples(views)` | 24 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `scan_prior(points, geometry)` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `camera_axis_z(geometry, qs)` | 46 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `goal_candidates(geometry, snapshot, start, xyz)` | 53 | Map-checked endpoint IK branches; paths still require separate checks. |
| `goal_ik(geometry, snapshot, start, xyz)` | 80 | Find a goal branch with a checked joint interpolation. |
| `guide_is_clear(geometry, snapshot, start, end)` | 110 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `velocity_proposal(start, goal, durations, maximum)` | 150 | A braking proposal over SAGE's actual (nonuniform) prediction horizon. |
| `run_demo(node, task)` | 171 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_perception_planning_core/edt.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/edt.py)

CudaEDT 对每个轴做精确可分离距离变换；轴过大等情况下采用 SciPy 路径。

源码说明：Exact separable Euclidean distance transform on the already available CUDA GPU.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `CudaEDT` | 11 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L12）、`__call__`（L16） |

### `ur7e_perception_planning_core/entrypoint.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/entrypoint.py)

命令行入口与运行环境准备，将参数转交节点或规划运行模式。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 5 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('mode', choices=['mapper', 'planner', 'combined'])
```

### `ur7e_perception_planning_core/frames.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/frames.py)

从待处理图像中选择能够取得严格图像时刻 TF 的最新一帧。

源码说明：Select the newest image with its exact-time transform available.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `newest_ready(entries, resolve)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_perception_planning_core/geometry.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/geometry.py)

RobotGeometry 解析 URDF，计算 CPU/批量 FK、整臂碰撞球和自碰撞间距，供独立执行检查和本体过滤。

源码说明：Small CPU FK for the independent execution guard and depth self filtering.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `RobotGeometry` | 13 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L14）、`fk`（L54）、`spheres`（L71）、`self_clearance`（L79）、`poses_batch`（L83）、`camera_axis_z`（L104）、`spheres_batch`（L108）、`self_clearance_batch`（L117） |

### `ur7e_perception_planning_core/grid.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/grid.py)

定义 [z,y,x] 网格和压缩消息协议，判断射线自由空间、构造保守规划距离，并提供带离散化余量的查询。

源码说明：Explicit grid contract and conservative CPU/GPU distance lookup.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `pack_map(msg, native, planning, sources)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `map_arrays(msg)` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `grid_points(center, size, resolution)` | 33 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `observed_free(points, depth, pose, intr, padding)` | 40 | Project cell centres into a min-filtered image; never clear behind a return. |
| `integration_depth(ray_depth, maximum)` | 54 | Keep only surfaces inside the fusion range; never invent a capped surface. |
| `conservative_field(raw, known, free_source, points, dims, resolution, cache=None, min_interval=0.0)` | 65 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `Snapshot` | 108 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `from_message`（L119）、`query`（L131） |

### `ur7e_perception_planning_core/mapper.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/mapper.py)

PlanningMapper 继承 TfMapper，增加本体过滤、观测自由来源、保守场和原子 PlanningMap 快照。

源码说明：Extend the existing TF mapper with self filtering and an atomic planning map.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `PlanningMapper` | 28 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `_on_info`（L29）、`_validate_config`（L37）、`_mapping_enabled`（L71）、`_robot_centers`（L77）、`_worker`（L86）、`_publish_latest`（L215） |

直接项目/规划依赖：`realtime_nvblox.ros_publishers`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_nvblox.node`, `ur7e_perception_planning.msg`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PlanningMap` | `'/perception_planning/map'` | 64 |
| `create_service` | `SetBool` | `'/perception_planning/mapping_enabled'` | 69 |

### `ur7e_perception_planning_core/planner.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/planner.py)

Planner 接收反馈、目标和地图，运行 SAGE 工作循环与独立 CPU 伺服保护；check_segment 检查整条短连接。

源码说明：ROS I/O, SAGE worker and independent CPU execution guard.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `check_segment(geometry, snapshot, start, end, margin=0.025)` | 29 | Check the whole short joint segment, including self collision and floor. |
| 类 `Planner` | 67 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L68）、`on_state`（L135）、`on_map`（L151）、`on_snapshot`（L157）、`on_goal`（L175）、`on_enable`（L190）、`inputs`（L197）、`freshness`（L201）、`speed_limit`（L213）、`plan_loop`（L223）、`servo_loop`（L310）、`publish_status`（L360）、`close`（L415） |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_perception_planning.msg`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/joint_states'` | 111 |
| `create_subscription` | `PoseStamped` | `'/target_pose'` | 112 |
| `create_publisher` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 113 |
| `create_publisher` | `String` | `'/perception_planning/status'` | 114 |
| `create_publisher` | `MarkerArray` | `'/perception_planning/markers'` | 117 |
| `create_publisher` | `PoseStamped` | `'/ee_pose'` | 120 |
| `create_service` | `SetBool` | `'/perception_planning/enable'` | 121 |
| `create_subscription` | `PlanningMap` | `'/perception_planning/map'` | 109 |

### `ur7e_perception_planning_core/routing.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/routing.py)

基于感知距离场进行路网/双向 RRT、目标 IK、路径简化和有界恢复；搜索失败则保持/继续观测。

源码说明：Bounded joint-space search using only the perceived planning distance field.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `joint_route(geometry, snapshot, start, goals, anchors=(), budget=2.0, allow_rrt=True)` | 8 | Checked roadmap first, then bidirectional RRT; None means hold/observe. |
| `shortcut(geometry, snapshot, path)` | 90 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `goal_route(geometry, snapshot, start, xyz, anchors=(), budget=2.0, allow_rrt=True)` | 100 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `recovery_step(geometry, snapshot, start, margin=0.015)` | 109 | Restore the extra planning buffer without violating the execution margin. |

### `ur7e_perception_planning_core/runtime.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/runtime.py)

MappingPriorityLock 在组合模式下协调单 GPU 使用，建图优先，CPU 检查与 ROS I/O 仍独立推进。

源码说明：One GPU owner at a time in combined mode; ROS I/O and CPU guard stay live.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `MappingPriorityLock` | 6 | Mapping goes before queued optimizations; active GPU work is not preempted. |
| 方法 | — | `__init__`（L8）、`acquire`（L12）、`release`（L27）、`__enter__`（L36）、`__exit__`（L40）、`mapping`（L44） |

### `ur7e_perception_planning_core/sage.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/sage.py)

EsdfRollout/EsdfTask 将冻结地图距离加入 SAGE rollout；不修改外部 STORM 源码。

源码说明：SAGE extension using a frozen GPU ESDF snapshot; no STORM source edits.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `EsdfRollout` | 13 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L14）、`set_map`（L28）、`query`（L34）、`cost_fn`（L42） |
| 类 `EsdfTask` | 73 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `get_rollout_fn`（L74）、`_build_controller_params`（L77）、`init_aux`（L89）、`update_params`（L95）、`close`（L98） |
| `build_task(run_dir, dt=0.1, particles=384, velocity_control=False, max_joint_speed=0.08)` | 102 | 按本包模型和配置构建 SAGE 任务，具体控制参数由配置决定。 |

直接项目/规划依赖：`storm_kit.mpc.rollout.sage_arm_reacher`, `storm_kit.mpc.task.sage_reacher_task_v3`。

### `ur7e_perception_planning_core/self_filter.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/self_filter.py)

沿相机射线按机器人碰撞球屏蔽本体像素，避免将本体当作环境障碍。

源码说明：Ray/sphere self filtering robust to depth noise along the viewing ray.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `robot_depth_mask(depth, pose, intr, centers, radii, link_ids, padding=0.025, depth_tolerance=0.06)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_perception_planning_core/speed.py`

[查看源码](../../src/ur7e_perception_planning/ur7e_perception_planning_core/speed.py)

按测得延迟计算保守速度档位，供静态场景 demo 使用。

源码说明：Conservative speed tiers for the static-scene demo, driven by measured delay.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `latency_speed(maximum, age, recent_delays, plan_ms)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_perception_planning/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/demo.yaml](../../src/ur7e_perception_planning/config/demo.yaml) | 配置分区/键：`/ur7e_nvblox` |
| [config/demo_scan.yaml](../../src/ur7e_perception_planning/config/demo_scan.yaml) | 配置分区/键： |
| [config/online.yaml](../../src/ur7e_perception_planning/config/online.yaml) | 配置分区/键：`/ur7e_nvblox` |
| [msg/PlanningMap.msg](../../src/ur7e_perception_planning/msg/PlanningMap.msg) | ROS 自定义消息契约。 |
| [package.xml](../../src/ur7e_perception_planning/package.xml) | ROS 包名、依赖及构建类型声明。 |
| [rviz/online.rviz](../../src/ur7e_perception_planning/rviz/online.rviz) | RViz 显示、固定坐标系及订阅话题。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_perception_planning/README.md)
- [ToDoList.md](../../src/ur7e_perception_planning/ToDoList.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
