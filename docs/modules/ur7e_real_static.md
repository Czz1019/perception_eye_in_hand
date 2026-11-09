# ur7e_real_static：真机静态地图规划及显式执行

[返回项目文档索引](../README.md)

## 职责和边界

默认 execution_enabled=false、commissioned=false，机器人 URDF/碰撞模型必须提供。静态地图按场景/模型/会话判断有效性，execution_gate 有意不使用单纯地图年龄阈值。

## 输入、处理与输出

**数据链路：** 手动停稳预扫描 → 冻结保守地图 → 目标路线/SAGE 离线优化 → 轨迹预览 → 显式 FollowJointTrajectory 执行。

采集线程生成完整距离场；freeze_map 绑定身份与校验和；on_goal 生成路线并时间参数化；execute 经 execution_gate 后提交 Action；反馈/watchdog 检查跟踪、模型及场景有效性。

## 目录结构

```text
ur7e_real_static/
├── config/
│   ├── collision_spheres.template.yaml
│   ├── collision_spheres_d435i.yaml
│   ├── handeye_d435i.yaml
│   ├── real.yaml
│   ├── real_d435i.yaml
│   └── view.rviz
├── launch/
│   └── bringup.launch.py
├── report/
├── scripts/
│   └── real_static
├── test/
│   ├── benchmark_capture.py
│   ├── fixtures.py
│   ├── plot_camera_profile.py
│   ├── run_launch_smoke.py
│   ├── run_mock_integration.py
│   ├── test_camera_collision.py
│   ├── test_capture_upgrade.py
│   ├── test_core.py
│   ├── test_gpu.py
│   ├── test_lifecycle.py
│   └── validate_camera_profile.py
├── ur7e_real_static/
│   ├── __init__.py
│   ├── camera_collision.py
│   ├── capture.py
│   ├── config.py
│   ├── export_description.py
│   ├── field.py
│   ├── model.py
│   ├── node.py
│   ├── planning.py
│   ├── state.py
│   ├── trajectory.py
│   └── visualization.py
├── validation/
├── CMakeLists.txt
├── README.md
├── REAL_TEST_GUIDE.md
├── ToDoList.md
└── package.xml
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/bringup.launch.py`

[查看源码](../../src/ur7e_real_static/launch/bringup.launch.py)

组装应用进程、配置路径和可选可视化；参数可改变实际启动路径。

源码说明：Start only the application. Robot/camera drivers are deliberately external.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 11 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('config', description='Absolute path to completed real.yaml')
DeclareLaunchArgument('run_rviz', default_value='false')
```

### `scripts/real_static`

[查看源码](../../src/ur7e_real_static/scripts/real_static)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
RS_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$RS_ENV"
export PATH="$RS_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$RS_ENV/bin/python" -B -m ur7e_real_static.node "$@"
```

### `test/benchmark_capture.py`

[查看源码](../../src/ur7e_real_static/test/benchmark_capture.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Same-map query and field benchmark, including the saved pre-upgrade FieldBuilder.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `summarize(values)` | 20 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 24 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.types`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_real_static.field`。

启动/命令行参数声明：

```python
parser.add_argument('--baseline-field', required=True)
parser.add_argument('--output', required=True)
```

### `test/fixtures.py`

[查看源码](../../src/ur7e_real_static/test/fixtures.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Simulation-only test data; never installed as a real commissioning profile.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `config(directory)` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `free_map(c)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.grid`, `ur7e_real_static.config`, `ur7e_real_static.field`, `ur7e_real_static.model`。

### `test/plot_camera_profile.py`

[查看源码](../../src/ur7e_real_static/test/plot_camera_profile.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Render model envelopes in tool0 at the archived scan-end configuration.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 14 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_real_static.camera_collision`, `ur7e_real_static.model`。

### `test/run_launch_smoke.py`

[查看源码](../../src/ur7e_real_static/test/run_launch_smoke.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Launch the actual entrypoint without devices and check clean SIGINT exit.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 11 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('--output', required=True)
```

### `test/run_mock_integration.py`

[查看源码](../../src/ur7e_real_static/test/run_mock_integration.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Actual ROS + nvBlox + SAGE + simulated FollowJointTrajectory; no robot connection.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `MockHardware` | 23 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L24）、`goal`（L34）、`cancel`（L37）、`execute`（L38）、`state`（L61）、`depth`（L69） |
| `main()` | 84 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.ros_publishers`, `ur7e_perception_planning_core.geometry`, `ur7e_real_static.field`, `ur7e_real_static.node`, `ur7e_real_static.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PoseStamped` | `c['target_topic']` | 111 |
| `create_publisher` | `JointState` | `c['joint_topic']` | 28 |
| `create_publisher` | `Image` | `c['depth_topic']` | 29 |
| `create_publisher` | `CameraInfo` | `c['camera_info_topic']` | 29 |
| `create_client` | `cls` | `'/ur7e_real_static/' + name` | 103 |
| `create_subscription` | `PointCloud2` | `'/ur7e_real_static/surface_esdf'` | 148 |
| `create_client` | `Trigger` | `'/ur7e_real_static/freeze_map'` | 136 |

启动/命令行参数声明：

```python
parser.add_argument('--output', required=True)
parser.add_argument('--depth-size', type=int, nargs=2, default=[160, 90])
parser.add_argument('--depth-encoding', choices=['16UC1', '32FC1'], default='32FC1')
parser.add_argument('--depth-hz', type=float, default=10.0)
```

### `test/test_camera_collision.py`

[查看源码](../../src/ur7e_real_static/test/test_camera_collision.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_cover_includes_volume_slab_boundaries_and_padding()` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `camera_profile(tmp_path)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_calibrated_cover_and_existing_pairs(tmp_path)` | 50 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_different_mount_is_rejected(tmp_path)` | 61 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_cover_cannot_be_silently_shrunk(tmp_path, change)` | 71 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_movable_mount_and_old_map_rejected(tmp_path)` | 83 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_real_static.camera_collision`, `ur7e_real_static.config`, `ur7e_real_static.field`, `ur7e_real_static.model`。

### `test/test_capture_upgrade.py`

[查看源码](../../src/ur7e_real_static/test/test_capture_upgrade.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_cpu_cuda_fields_match_full_reference_without_simulation_floor(tmp_path)` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_capture_rejects_duplicate_and_pre_scan_stamps()` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_performance_options_do_not_change_map_identity(tmp_path)` | 49 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_visualization_enqueue_never_calls_dds()` | 55 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_failed_map_save_does_not_freeze_scan(tmp_path)` | 63 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_capture_initialization_failure_is_reported_and_worker_survives()` | 76 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.stats`, `ur7e_real_static.capture`, `ur7e_real_static.config`, `ur7e_real_static.field`, `ur7e_real_static.node`。

### `test/test_core.py`

[查看源码](../../src/ur7e_real_static/test/test_core.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_requires_explicit_real_configuration(tmp_path)` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_model_single_calibrated_chain_and_payload_required(tmp_path)` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_snapshot_roundtrip_identity_checksum_and_unknown(tmp_path)` | 39 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_field_empty_depth_never_clears_unknown_and_obstacle_overrides_prior(tmp_path)` | 50 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_quintic_limits_and_collision_checks(tmp_path)` | 61 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_frozen_map_age_is_not_control_freshness(tmp_path)` | 75 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_unknown_corridor_blocks_even_with_free_goal(tmp_path)` | 89 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_real_static.config`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.state`, `ur7e_real_static.trajectory`。

### `test/test_gpu.py`

[查看源码](../../src/ur7e_real_static/test/test_gpu.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Runs the actual installed whole_control SAGE/CUDA backend, not a fake optimizer.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_actual_sage_plan_uses_calibrated_model_and_static_esdf(tmp_path)` | 8 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_shuffled_urdf_order_keeps_actual_sage_fk_consistent(tmp_path)` | 17 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_sage_detours_around_synthetic_obstacle(tmp_path)` | 31 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.grid`, `ur7e_real_static.config`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.planning`, `ur7e_real_static.trajectory`。

### `test/test_lifecycle.py`

[查看源码](../../src/ur7e_real_static/test/test_lifecycle.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_late_goal_acceptance_after_stop_is_canceled()` | 7 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_rejected_action_does_not_own_motion()` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_new_target_cancels_active_action_and_never_starts_parallel_plan()` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_unknown_action_acceptance_latches_unconfirmed_and_blocks_reexecution()` | 34 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_cancellation_result_alone_does_not_release_motion()` | 45 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_real_static.node`。

### `test/validate_camera_profile.py`

[查看源码](../../src/ur7e_real_static/test/validate_camera_profile.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Offline validation of this machine's camera profile; never connects hardware.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `clearance(g, qs)` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 26 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_real_static.camera_collision`, `ur7e_real_static.config`, `ur7e_real_static.model`, `ur7e_real_static.planning`。

启动/命令行参数声明：

```python
parser.add_argument('--workspace', required=True)
parser.add_argument('--output', required=True)
```

### `ur7e_real_static/__init__.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

源码说明：No device connection or motion occurs on import.

### `ur7e_real_static/camera_collision.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/camera_collision.py)

从相机几何生成保守多球包络并核对安装变换；支架、线缆等仍需对应的实际模型。

源码说明：Offline camera-body sphere generation and runtime mounting consistency checks.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `origin(element)` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `fixed_transform(root, parent, child)` | 27 | Require a wholly fixed descendant chain; never bake in a movable joint. |
| `cover_box(low, high, count=6, padding=0.005)` | 44 | Partition the longest axis; each sphere encloses an entire closed slab. |
| `validate_camera_cover(root, spheres, metadata)` | 63 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `vendor_body(description)` | 85 | Expand the installed D435i description, retaining mesh scene transforms. |
| `generate(urdf, arm_spheres, description)` | 124 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 151 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('--urdf', required=True)
parser.add_argument('--arm-spheres', required=True)
parser.add_argument('--description', required=True)
parser.add_argument('--output', required=True)
```

### `ur7e_real_static/capture.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/capture.py)

停稳深度采集工作循环：匹配图像时刻关节与 TF，过滤本体，融合并提交距离场。

源码说明：Stationary-view nvBlox capture. No Gazebo priors and no robot commands.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `worker(node)` | 15 | 工作循环入口；处理停止通知与异常，将实际处理转交工作函数。 |
| `_worker(node)` | 25 | 实际后台处理循环；按本文件职责读取输入、校验并提交结果。 |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_perception_planning_core.runtime`, `ur7e_perception_planning_core.self_filter`。

### `ur7e_real_static/config.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/config.py)

对真机参数严格校验；identity 将模型文件内容和建图几何参数绑定到冻结地图。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `load_config(path)` | 9 | 读取配置并执行本包规定的合并、路径解析和字段/范围检查。 |
| `identity(c)` | 56 | 将模型文件与关键建图配置绑定为身份标识，供冻结/重载一致性检查。 |

### `ur7e_real_static/export_description.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/export_description.py)

订阅并原样保存 robot_description 字符串，避免 ROS CLI YAML 转义影响 URDF。

源码说明：Save the live robot_description String verbatim, without ROS CLI YAML quoting.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 11 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `String` | `a.topic` | 17 |

启动/命令行参数声明：

```python
p.add_argument('--output', required=True)
p.add_argument('--topic', default='/robot_description')
```

### `ur7e_real_static/field.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/field.py)

FieldBuilder 从有效射线与原生距离构建保守场；FrozenMap 保存来源、元数据和校验和，并校验身份后加载。

源码说明：Conservative static ESDF; immutable, checksummed persistence without pickle.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `digest(metadata, arrays)` | 13 | 对输入元数据/数组或计划生成 SHA256，用于身份/完整性核验。 |
| 类 `FrozenMap` | 20 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `create`（L27）、`save`（L42）、`load`（L51） |
| 类 `FieldBuilder` | 61 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L62）、`update`（L74） |

直接项目/规划依赖：`ur7e_perception_planning_core.edt`, `ur7e_perception_planning_core.grid`。

### `ur7e_real_static/model.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/model.py)

Geometry 扩展通用 CPU FK，以委托 URDF、碰撞球和相机外参统一规划与 TF 核对。

源码说明：Use one commissioned URDF and sphere model for TF checks, CPU and SAGE.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Geometry` | 11 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L12）、`camera_axis_z`（L93）、`write_urdf`（L97） |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`。

### `ur7e_real_static/node.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/node.py)

RealStatic 汇集扫描、冻结、重载、场景确认、目标规划、轨迹预览及显式 Action 执行，并通过 watchdog 监控有效性。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `RealStatic` | 39 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L40）、`service`（L79）、`idle_required`（L89）、`on_info`（L94）、`info_record`（L105）、`on_depth`（L108）、`on_state`（L123）、`state_at`（L137）、`settled_at`（L144）、`start_scan`（L149）、`freeze_map`（L159）、`load_map`（L174）、`confirm_scene`（L184）、`on_goal`（L192）、`current_state`（L232）、`live_tf_matches`（L238）、`recent_tf_matches`（L245）、`trajectory_message`（L251）、`execute`（L259）、`accepted`（L285）、`feedback`（L296）、`finished`（L307）、`cancel`（L324）、`stop_motion`（L339）、`invalidate`（L343）、`watchdog`（L346）、`publish_cloud`（L379）、`clear_visuals`（L383）、`publish_preview`（L386）、`status`（L396）、`close`（L423） |
| `main()` | 431 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.stats`, `ur7e_nvblox.depth`, `ur7e_perception_planning_core.geometry`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `c['joint_topic']` | 57 |
| `create_subscription` | `CameraInfo` | `c['camera_info_topic']` | 58 |
| `create_subscription` | `Image` | `c['depth_topic']` | 59 |
| `create_subscription` | `PoseStamped` | `c['target_topic']` | 60 |
| `create_publisher` | `String` | `'~/status'` | 61 |
| `create_publisher` | `PointCloud2` | `'~/esdf'` | 63 |
| `create_publisher` | `PointCloud2` | `'~/surface_esdf'` | 64 |
| `create_publisher` | `JointTrajectory` | `'~/preview_trajectory'` | 65 |
| `create_publisher` | `MarkerArray` | `'~/markers'` | 66 |
| `create_service` | `SetBool` | `'~/confirm_scene'` | 71 |
| `create_service` | `Trigger` | `'~/' + name` | 70 |

启动/命令行参数声明：

```python
parser.add_argument('--config', required=True)
```

### `ur7e_real_static/planning.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/planning.py)

对单份不可变地图做路线搜索及离线 SAGE 优化，生成规划结果；本模块不发送运动指令。

源码说明：Offline SAGE refinement on one immutable map; no commands are sent here.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `route(g, snapshot, start, xyz, anchors, c, cancel=lambda : False)` | 10 | 在给定模型、地图与预算内寻找可检查的关节引导路径；不能搜索成功时抛错/返回失败。 |
| `build_task(g, c, directory)` | 62 | 按本包模型和配置构建 SAGE 任务，具体控制参数由配置决定。 |
| `plan(g, frozen, start, xyz, c, directory, cancel=lambda : False)` | 115 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`storm_kit.mpc.rollout.sage_arm_reacher`, `ur7e_perception_planning_core.demo`, `ur7e_perception_planning_core.runtime`, `ur7e_perception_planning_core.sage`。

### `ur7e_real_static/state.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/state.py)

execution_gate 汇总执行前提：配置允许、场景确认、地图/轨迹版本、反馈时效、停稳、起点、TF 和标定。

源码说明：Small deterministic state rules, also exercised without ROS/hardware.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `execution_gate(c, frozen, trajectory, state, now_ros, now_wall, confirmed, tf_ok, calibration_ok)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_real_static/trajectory.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/trajectory.py)

整臂密集连接检查与简化；五次时间参数化后校验速度、加速度、碰撞及模型一致性。

源码说明：Dense whole-arm checks and stop-at-waypoint quintic time parameterization.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `edge_clear(g, snapshot, a, b, margin=0.03, self_margin=0.015)` | 7 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `simplify(g, snapshot, points, margin, self_margin)` | 23 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `Trajectory` | 34 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `sample`（L42） |
| `parameterize(points, speed, acceleration, map_uid, identity, goal, iterations=0)` | 49 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `validate(g, frozen, trajectory, c)` | 60 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `model_matches(g, q, poses, position_tolerance=0.003, angle_tolerance=0.01)` | 74 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_real_static/visualization.py`

[查看源码](../../src/ur7e_real_static/ur7e_real_static/visualization.py)

独立显示工作线程，以最新结果为准并按订阅状态发布，避免显示阻塞采集。

源码说明：Subscriber-gated, latest-only visualization; never part of the motion gate.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `worker(node)` | 7 | 工作循环入口；处理停止通知与异常，将实际处理转交工作函数。 |

直接项目/规划依赖：`realtime_nvblox.ros_publishers`。

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_real_static/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/collision_spheres.template.yaml](../../src/ur7e_real_static/config/collision_spheres.template.yaml) | 配置分区/键： |
| [config/collision_spheres_d435i.yaml](../../src/ur7e_real_static/config/collision_spheres_d435i.yaml) | 配置分区/键：`robot_name` |
| [config/handeye_d435i.yaml](../../src/ur7e_real_static/config/handeye_d435i.yaml) | 配置分区/键： |
| [config/real.yaml](../../src/ur7e_real_static/config/real.yaml) | 配置分区/键： |
| [config/real_d435i.yaml](../../src/ur7e_real_static/config/real_d435i.yaml) | 配置分区/键： |
| [config/view.rviz](../../src/ur7e_real_static/config/view.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [package.xml](../../src/ur7e_real_static/package.xml) | ROS 包名、依赖及构建类型声明。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_real_static/README.md)
- [REAL_TEST_GUIDE.md](../../src/ur7e_real_static/REAL_TEST_GUIDE.md)
- [ToDoList.md](../../src/ur7e_real_static/ToDoList.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
- `report/`：历史实验报告、观测/模型数据和配图/生成脚本。
