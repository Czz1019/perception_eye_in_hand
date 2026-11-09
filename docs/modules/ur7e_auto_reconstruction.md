# ur7e_auto_reconstruction：RTDE 自动彩色扫描

[返回项目文档索引](../README.md)

## 职责和边界

使用独立 RTDE 控制，不调用 SAGE、不做 ESDF。轨迹计划明确 collision_checked=false/self_collision_checked=false，不能解释为通用避障扫描。

## 输入、处理与输出

**数据链路：** 实时 RTDE 起点 + 8 点路线 → 显式 prepare/start → 五次关节段 → 停稳释放控制 → 门控 RGB-D 融合。

启动接收反馈并显示；prepare 生成含哈希的计划；start 核对审核计划与起点；每点到达、停稳、关闭控制接口后采集；下一段从实测反馈重新起步；结束冻结表面。

## 目录结构

```text
ur7e_auto_reconstruction/
├── config/
│   ├── control.yaml
│   ├── mapping.yaml
│   ├── view.rviz
│   └── waypoints.yaml
├── launch/
│   └── bringup.launch.py
├── report/
├── scripts/
│   ├── auto_mapper
│   ├── auto_scan
│   └── offline_preview
├── test/
│   ├── mock_camera.py
│   ├── mock_mapper.py
│   ├── run_faults.py
│   ├── run_integration.py
│   ├── run_launch_smoke.py
│   └── test_control.py
├── ur7e_auto_reconstruction/
│   ├── __init__.py
│   ├── mapper.py
│   ├── node.py
│   ├── preview.py
│   ├── trajectory.py
│   └── transport.py
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

[查看源码](../../src/ur7e_auto_reconstruction/launch/bringup.launch.py)

组装应用进程、配置路径和可选可视化；参数可改变实际启动路径。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `setup(context)` | 12 | 解析 launch 上下文中的路径/开关，准备资源并返回实际启动动作。 |
| `generate_launch_description()` | 33 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument(k, default_value=v)
```

上述字典循环实际声明的参数如下；路径表达式在启动时解析：

| 参数 | 默认值/表达式 |
| --- | --- |
| `mapping` | `str(share / 'config/mapping.yaml')` |
| `control` | `str(share / 'config/control.yaml')` |
| `waypoints` | `str(share / 'config/waypoints.yaml')` |
| `allow_motion` | `'false'` |
| `approved_plan` | `''` |
| `run_rviz` | `'true'` |
| `run_controller` | `'true'` |
| `load` | `''` |

### `scripts/auto_mapper`

[查看源码](../../src/ur7e_auto_reconstruction/scripts/auto_mapper)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
AUTO_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$AUTO_ENV"
export PATH="$AUTO_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$AUTO_ENV/bin/python" -B -m ur7e_auto_reconstruction.mapper "$@"
```

### `scripts/auto_scan`

[查看源码](../../src/ur7e_auto_reconstruction/scripts/auto_scan)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
AUTO_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$AUTO_ENV"
export PATH="$AUTO_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$AUTO_ENV/bin/python" -B -m ur7e_auto_reconstruction.node "$@"
```

### `scripts/offline_preview`

[查看源码](../../src/ur7e_auto_reconstruction/scripts/offline_preview)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
AUTO_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$AUTO_ENV"
export PATH="$AUTO_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$AUTO_ENV/bin/python" -B -m ur7e_auto_reconstruction.preview "$@"
```

### `test/mock_camera.py`

[查看源码](../../src/ur7e_auto_reconstruction/test/mock_camera.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Synthetic scene on domain 84 only. No device/RTDE access.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Scene` | 15 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L16）、`state`（L23）、`images`（L28） |

直接项目/规划依赖：`realtime_nvblox.ros_publishers`, `ur7e_auto_reconstruction.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/ur7e_auto/joint_states'` | 19 |
| `create_publisher` | `CameraInfo if 'info' in k else Image` | `c[k]` | 20 |

### `test/mock_mapper.py`

[查看源码](../../src/ur7e_auto_reconstruction/test/mock_mapper.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Use the production mapper, asserting that no ESDF API can be called.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `forbidden(*args, **kwargs)` | 7 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`, `ur7e_auto_reconstruction.mapper`。

### `test/run_faults.py`

[查看源码](../../src/ur7e_auto_reconstruction/test/run_faults.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Fast coordinator fault-injection on domain 86, with no RTDE/network hardware.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Feedback` | 23 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L24）、`snapshot`（L25）、`close`（L28） |
| 类 `Control` | 31 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L32）、`keepalive`（L37）、`period`（L38）、`send`（L39）、`wait`（L40）、`close`（L41） |
| 类 `Mapper` | 45 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L46）、`start`（L55）、`warmup`（L57）、`freeze`（L58）、`capture`（L61）、`report`（L67） |
| `wait(p, timeout=8)` | 76 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_auto_reconstruction.node`, `ur7e_auto_reconstruction.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `String` | `'~/capture_status'` | 48 |
| `create_service` | `Trigger` | `'~/start_scan'` | 50 |
| `create_service` | `Trigger` | `'~/freeze_scan'` | 51 |
| `create_service` | `Trigger` | `'~/warmup'` | 52 |
| `create_service` | `SetBool` | `'~/capture'` | 54 |

### `test/run_integration.py`

[查看源码](../../src/ur7e_auto_reconstruction/test/run_integration.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：ROS services + mock RTDE + real CUDA reconstruction; isolated domain 84.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `FakeReceiver` | 29 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L30）、`snapshot`（L33）、`close`（L37） |
| 类 `FakeControl` | 41 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L42）、`keepalive`（L48）、`period`（L49）、`send`（L50）、`wait`（L55）、`close`（L56） |
| `wait(pred, timeout=30)` | 62 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `call(name, kind=Trigger, data=None)` | 69 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `launch(args, name)` | 79 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_auto_reconstruction.node`, `ur7e_auto_reconstruction.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `kind` | `name` | 70 |

### `test/run_launch_smoke.py`

[查看源码](../../src/ur7e_auto_reconstruction/test/run_launch_smoke.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Installed launcher, namespaced calibrated TF, and frozen-map visualization; domain 85.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `state()` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_auto_reconstruction.trajectory`, `ur7e_nvblox.depth`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `JointState` | `'/ur7e_auto/joint_states'` | 23 |
| `create_subscription` | `String` | `'/ur7e_auto_mapper/status'` | 26 |
| `create_subscription` | `PointCloud2` | `'/ur7e_auto_mapper/surface_points'` | 27 |

### `test/test_control.py`

[查看源码](../../src/ur7e_auto_reconstruction/test/test_control.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `inputs()` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_all_selected_segments_have_bounded_derivatives(inputs)` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_plan_change_stale_start_and_offline_rejected(inputs)` | 33 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_invalid_joint_targets(q)` | 43 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `Feedback` | 47 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L48）、`snapshot`（L50） |
| 类 `Control` | 56 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L57）、`period`（L59）、`send`（L60）、`wait`（L62）、`close`（L63） |
| `wait_for(p, timeout=2)` | 66 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_servo_stops_on_fault(inputs, fault)` | 74 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_cancel_and_no_catchup(inputs)` | 94 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_speed_scaling_preserves_command_acceleration(inputs)` | 107 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `FrozenRTDE` | 121 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L122）、`getTimestamp`（L123）、`getActualQ`（L124）、`getActualQd`（L125）、`getRobotMode`（L126）、`getSafetyMode`（L127）、`getSpeedScaling`（L128）、`getTargetSpeedFraction`（L129）、`disconnect`（L130） |
| `test_repeated_rtde_timestamp_is_not_fresh()` | 133 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_model_hash_and_waypoint_fk_checked(inputs, tmp_path)` | 142 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_servoj_does_not_overwrite_command_with_watchdog(inputs)` | 151 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_failed_servo_stop_is_reported_and_disconnect_still_attempted(inputs)` | 165 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_auto_reconstruction.trajectory`, `ur7e_auto_reconstruction.transport`。

### `ur7e_auto_reconstruction/__init__.py`

[查看源码](../../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

### `ur7e_auto_reconstruction/mapper.py`

[查看源码](../../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/mapper.py)

GatedReconstruction 继承手动重建器，增加每停靠点采集门、lease、有效帧计数、预热和观测记录。

源码说明：Add an explicit per-stop capture gate without altering the manual mapper.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `GatedReconstruction` | 18 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L19）、`warmup`（L39）、`_warmup`（L54）、`close`（L72）、`on_lease`（L76）、`capture`（L79）、`on_pair`（L100）、`pick_ready`（L111）、`new_session`（L118）、`record_view`（L125）、`capture_report`（L150） |
| `main()` | 164 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_real_reconstruction.inputs`, `ur7e_real_reconstruction.node`, `ur7e_reconstruction.color`, `ur7e_reconstruction.surface`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_service` | `SetBool` | `'~/capture'` | 33 |
| `create_service` | `Trigger` | `'~/warmup'` | 34 |
| `create_publisher` | `String` | `'~/capture_status'` | 35 |
| `create_subscription` | `Empty` | `'/ur7e_auto_scan/capture_lease'` | 36 |

启动/命令行参数声明：

```python
p.add_argument('--config', required=True)
p.add_argument('--load', default='')
```

### `ur7e_auto_reconstruction/node.py`

[查看源码](../../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/node.py)

Scanner 协调 prepare/start/cancel；从实时反馈准备路线，逐段运动、停稳、释放控制、采集，再重新取得下一段控制权。

源码说明：Explicit preview/start/cancel coordinator. No control connection at startup.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Scanner` | 26 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L27）、`on_mapper`（L57）、`publish_state`（L64）、`report`（L77）、`prepare`（L87）、`publish_route`（L107）、`start`（L117）、`abort`（L141）、`require_mapper`（L146）、`tick`（L150）、`wait`（L161）、`call`（L169）、`settle`（L187）、`stationary_state`（L200）、`acquire_servo`（L212）、`release_servo`（L240）、`run`（L249）、`close`（L345） |
| `main()` | 354 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`, `ur7e_real_reconstruction.node`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `JointState` | `'/ur7e_auto/joint_states'` | 43 |
| `create_publisher` | `String` | `'~/status'` | 44 |
| `create_publisher` | `MarkerArray` | `'~/route'` | 45 |
| `create_publisher` | `Empty` | `'~/capture_lease'` | 46 |
| `create_subscription` | `String` | `'/ur7e_auto_mapper/capture_status'` | 47 |
| `create_service` | `Trigger` | `'~/prepare'` | 50 |
| `create_service` | `Trigger` | `'~/start'` | 51 |
| `create_service` | `Trigger` | `'~/cancel'` | 52 |
| `create_client` | `kind` | `'/ur7e_auto_mapper/' + name` | 48 |

启动/命令行参数声明：

```python
p.add_argument('--approved-plan', default='')
p.add_argument('--allow-motion', choices=['true', 'false'], default='false')
p.add_argument('--' + flag, required=True)
```

### `ur7e_auto_reconstruction/preview.py`

[查看源码](../../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/preview.py)

生成可供查看的离线路线和诊断图；本文件不取得机器人控制权。

源码说明：Reviewable offline artifacts; never imports an RTDE control interface.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `render(plan, directory, geometry=None)` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 57 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
p.add_argument('--' + name, required=True)
```

### `ur7e_auto_reconstruction/trajectory.py`

[查看源码](../../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/trajectory.py)

检查六关节、外参/模型哈希和 waypoint 一致性；生成同步五次插值，绑定并校验审核计划与实际起点。

源码说明：Offline joint-space path generation; no hardware imports or connections.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `digest(value)` | 11 | 对输入元数据/数组或计划生成 SHA256，用于身份/完整性核验。 |
| `vector(q)` | 15 | 转换并要求六个有限关节角，保持原始关节角表示，不自动绕回。 |
| 类 `Segment` | 22 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L23）、`sample`（L33） |
| `load_inputs(mapping_path, control_path, waypoints_path)` | 39 | 加载建图/控制/路线，校验字段、数值、模型哈希和观察点一致性。 |
| `make_plan(q0, control, route, geometry, source='live_rtde')` | 82 | 从指定实际起点构造分段路线、时长、诊断和完整计划哈希。 |
| `verify_plan(plan, q, control, route, geometry)` | 108 | 核验计划完整性、当前输入及实际起点是否仍与审核计划一致。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`, `ur7e_real_reconstruction.inputs`, `ur7e_real_static.model`。

### `ur7e_auto_reconstruction/transport.py`

[查看源码](../../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/transport.py)

Receiver 单独接收 RTDE；RTDEControl 检查本机控制冲突及文件锁；Servo 高频单写者执行并检查反馈、跟踪、周期和心跳。

源码说明：RTDE receive-only connection is separate from explicit control acquisition.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Receiver` | 11 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L12）、`_run`（L24）、`snapshot`（L51）、`close`（L59） |
| `local_control_conflicts()` | 65 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `RTDEControl` | 79 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L80）、`keepalive`（L97）、`period`（L101）、`send`（L104）、`wait`（L112）、`close`（L115） |
| 类 `Servo` | 132 | Single writer at 500 Hz; ROS calls and CUDA run outside this thread/process. |
| 方法 | — | `__init__`（L134）、`ping`（L151）、`move`（L158）、`_run`（L164）、`close`（L216） |

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_auto_reconstruction/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/control.yaml](../../src/ur7e_auto_reconstruction/config/control.yaml) | 配置分区/键：`robot_ip` |
| [config/mapping.yaml](../../src/ur7e_auto_reconstruction/config/mapping.yaml) | 配置分区/键：`robot_urdf` |
| [config/view.rviz](../../src/ur7e_auto_reconstruction/config/view.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [config/waypoints.yaml](../../src/ur7e_auto_reconstruction/config/waypoints.yaml) | 配置分区/键：`schema_version` |
| [package.xml](../../src/ur7e_auto_reconstruction/package.xml) | ROS 包名、依赖及构建类型声明。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_auto_reconstruction/README.md)
- [REAL_TEST_GUIDE.md](../../src/ur7e_auto_reconstruction/REAL_TEST_GUIDE.md)
- [ToDoList.md](../../src/ur7e_auto_reconstruction/ToDoList.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
- `report/`：历史实验报告、观测/模型数据和配图/生成脚本。
