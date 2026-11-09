# ur7e_auto_esdf：RTDE 自动 ESDF 扫描

[返回项目文档索引](../README.md)

## 职责和边界

不输出彩色 Mesh、不执行 SAGE 目标任务；地图冻结成功与 planning_ready 是不同条件。扫描路线同样不是经环境/完整自身碰撞检查的通用路径。

## 输入、处理与输出

**数据链路：** 自动扫描控制 + 门控停稳深度 → TSDF/原生 ESDF → FrozenMap 与逐点深度证据。

Scanner 协调八点；共享自动重建 transport；EsdfMapper 提供独立深度门和 lease；采集工作进程使用 EfficientBackend/FieldBuilder；冻结时记录地图身份、来源和统计。

## 目录结构

```text
ur7e_auto_esdf/
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
│   └── test_mapper.py
├── ur7e_auto_esdf/
│   ├── __init__.py
│   ├── capture.py
│   ├── mapper.py
│   ├── node.py
│   ├── preview.py
│   └── trajectory.py
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

[查看源码](../../src/ur7e_auto_esdf/launch/bringup.launch.py)

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

[查看源码](../../src/ur7e_auto_esdf/scripts/auto_mapper)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
AUTO_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$AUTO_ENV"
export PATH="$AUTO_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$AUTO_ENV/bin/python" -B -m ur7e_auto_esdf.mapper "$@"
```

### `scripts/auto_scan`

[查看源码](../../src/ur7e_auto_esdf/scripts/auto_scan)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
AUTO_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$AUTO_ENV"
export PATH="$AUTO_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$AUTO_ENV/bin/python" -B -m ur7e_auto_esdf.node "$@"
```

### `scripts/offline_preview`

[查看源码](../../src/ur7e_auto_esdf/scripts/offline_preview)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
AUTO_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$AUTO_ENV"
export PATH="$AUTO_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$AUTO_ENV/bin/python" -B -m ur7e_auto_esdf.preview "$@"
```

### `test/mock_camera.py`

[查看源码](../../src/ur7e_auto_esdf/test/mock_camera.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Synthetic scene on domain 84 only. No device/RTDE access.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Scene` | 15 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L16）、`state`（L23）、`images`（L28） |

直接项目/规划依赖：`realtime_nvblox.ros_publishers`, `ur7e_auto_esdf.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/ur7e_auto_esdf/joint_states'` | 19 |
| `create_publisher` | `CameraInfo if 'info' in k else Image` | `c[k]` | 20 |

### `test/mock_mapper.py`

[查看源码](../../src/ur7e_auto_esdf/test/mock_mapper.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

直接项目/规划依赖：`ur7e_auto_esdf.mapper`。

### `test/run_faults.py`

[查看源码](../../src/ur7e_auto_esdf/test/run_faults.py)

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

直接项目/规划依赖：`ur7e_auto_esdf.node`, `ur7e_auto_esdf.trajectory`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `String` | `'~/capture_status'` | 48 |
| `create_service` | `Trigger` | `'~/start_scan'` | 50 |
| `create_service` | `Trigger` | `'~/freeze_scan'` | 51 |
| `create_service` | `Trigger` | `'~/warmup'` | 52 |
| `create_service` | `SetBool` | `'~/capture'` | 54 |

### `test/run_integration.py`

[查看源码](../../src/ur7e_auto_esdf/test/run_integration.py)

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

直接项目/规划依赖：`ur7e_auto_esdf.node`, `ur7e_auto_esdf.trajectory`, `ur7e_nvblox.depth`, `ur7e_real_static.config`, `ur7e_real_static.field`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `kind` | `name` | 70 |

### `test/run_launch_smoke.py`

[查看源码](../../src/ur7e_auto_esdf/test/run_launch_smoke.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Installed launch + frozen-map replay + synthetic TF, no RTDE node.

直接项目/规划依赖：`ur7e_auto_esdf.trajectory`, `ur7e_nvblox.depth`, `ur7e_perception_planning_core.geometry`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `JointState` | `'/ur7e_auto_esdf/joint_states'` | 26 |
| `create_subscription` | `PointCloud2` | `'/ur7e_auto_esdf_mapper/' + name` | 25 |

### `test/test_mapper.py`

[查看源码](../../src/ur7e_auto_esdf/test/test_mapper.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Mapper gates and persistence only; no RTDE or physical camera connections.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `dummy()` | 19 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_bad_capture_request_rejected(fault)` | 27 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_epoch_cutoff_and_drift_do_not_credit_another_view()` | 38 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_route_and_mapping_use_same_calibrated_model()` | 50 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_frozen_map_saved_for_static_planner_but_not_declared_planning_ready(tmp_path)` | 57 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_freeze_rejects_open_gate()` | 70 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_auto_esdf.mapper`, `ur7e_auto_esdf.trajectory`, `ur7e_real_static.config`, `ur7e_real_static.field`。

### `ur7e_auto_esdf/__init__.py`

[查看源码](../../src/ur7e_auto_esdf/ur7e_auto_esdf/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

### `ur7e_auto_esdf/capture.py`

[查看源码](../../src/ur7e_auto_esdf/ur7e_auto_esdf/capture.py)

停稳深度采集工作循环：匹配图像时刻关节与 TF，过滤本体，融合并提交距离场。

源码说明：Stationary-view nvBlox capture. No Gazebo priors and no robot commands.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `worker(node)` | 16 | 工作循环入口；处理停止通知与异常，将实际处理转交工作函数。 |
| `_worker(node)` | 26 | 实际后台处理循环；按本文件职责读取输入、校验并提交结果。 |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_perception_planning_core.runtime`, `ur7e_perception_planning_core.self_filter`, `ur7e_real_static.field`, `ur7e_real_static.trajectory`。

### `ur7e_auto_esdf/mapper.py`

[查看源码](../../src/ur7e_auto_esdf/ur7e_auto_esdf/mapper.py)

EsdfMapper 管理深度采集门、lease、预热、冻结和重载；保存规划冻结场及原生 ESDF，不输出 RGB Mesh。

源码说明：Scan-only ESDF mapper: no target subscription, Action client or robot control.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `EsdfMapper` | 32 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L40）、`service`（L78）、`on_info`（L87）、`on_depth`（L90）、`on_lease`（L97）、`capture_allowed`（L99）、`start_scan`（L104）、`capture`（L116）、`record_capture`（L133）、`publish_cloud`（L144）、`freeze_scan`（L148）、`load`（L174）、`warmup`（L185）、`_warmup`（L196）、`report`（L209）、`close`（L218） |
| `main()` | 224 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.stats`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`, `ur7e_real_reconstruction.node`, `ur7e_real_static.config`, `ur7e_real_static.field`, `ur7e_real_static.model`, `ur7e_real_static.node`, `ur7e_real_static.visualization`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PointCloud2` | `'~/esdf'` | 59 |
| `create_publisher` | `PointCloud2` | `'~/surface_esdf'` | 60 |
| `create_publisher` | `String` | `'~/capture_status'` | 61 |
| `create_subscription` | `JointState` | `c['joint_topic']` | 62 |
| `create_subscription` | `CameraInfo` | `c['camera_info_topic']` | 63 |
| `create_subscription` | `Image` | `c['depth_topic']` | 64 |
| `create_subscription` | `Empty` | `'/ur7e_auto_esdf_scan/capture_lease'` | 65 |
| `create_service` | `Trigger` | `'~/warmup'` | 68 |
| `create_service` | `SetBool` | `'~/capture'` | 69 |
| `create_service` | `Trigger` | `'~/' + name` | 67 |

启动/命令行参数声明：

```python
p.add_argument('--config', required=True)
p.add_argument('--load', default='')
```

### `ur7e_auto_esdf/node.py`

[查看源码](../../src/ur7e_auto_esdf/ur7e_auto_esdf/node.py)

自动 ESDF 扫描协调器，控制流程与自动彩色重建相似，停稳释放伺服后才打开深度采集门。

源码说明：Explicit preview/start/cancel coordinator. No control connection at startup.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Scanner` | 27 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L28）、`on_mapper`（L58）、`publish_state`（L65）、`report`（L78）、`prepare`（L88）、`publish_route`（L108）、`start`（L118）、`abort`（L142）、`require_mapper`（L147）、`tick`（L151）、`wait`（L162）、`call`（L170）、`settle`（L188）、`stationary_state`（L201）、`acquire_servo`（L213）、`release_servo`（L241）、`run`（L250）、`close`（L349） |
| `main()` | 358 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_auto_reconstruction.transport`, `ur7e_perception_planning_core.geometry`, `ur7e_real_reconstruction.node`, `ur7e_real_static.config`, `ur7e_real_static.field`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `JointState` | `'/ur7e_auto_esdf/joint_states'` | 44 |
| `create_publisher` | `String` | `'~/status'` | 45 |
| `create_publisher` | `MarkerArray` | `'~/route'` | 46 |
| `create_publisher` | `Empty` | `'~/capture_lease'` | 47 |
| `create_subscription` | `String` | `'/ur7e_auto_esdf_mapper/capture_status'` | 48 |
| `create_service` | `Trigger` | `'~/prepare'` | 51 |
| `create_service` | `Trigger` | `'~/start'` | 52 |
| `create_service` | `Trigger` | `'~/cancel'` | 53 |
| `create_client` | `kind` | `'/ur7e_auto_esdf_mapper/' + name` | 49 |

启动/命令行参数声明：

```python
p.add_argument('--approved-plan', default='')
p.add_argument('--allow-motion', choices=['true', 'false'], default='false')
p.add_argument('--' + flag, required=True)
```

### `ur7e_auto_esdf/preview.py`

[查看源码](../../src/ur7e_auto_esdf/ur7e_auto_esdf/preview.py)

生成可供查看的离线路线和诊断图；本文件不取得机器人控制权。

源码说明：Reviewable offline artifacts; never imports an RTDE control interface.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `render(plan, directory, geometry=None)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 58 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
p.add_argument('--' + name, required=True)
```

### `ur7e_auto_esdf/trajectory.py`

[查看源码](../../src/ur7e_auto_esdf/ur7e_auto_esdf/trajectory.py)

校验扫描关节路线与标定指纹，从当前起点生成五次插值和可审核计划；不做 SAGE 避障搜索。

源码说明：Offline joint-space path generation; no hardware imports or connections.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `digest(value)` | 12 | 对输入元数据/数组或计划生成 SHA256，用于身份/完整性核验。 |
| `vector(q)` | 16 | 转换并要求六个有限关节角，保持原始关节角表示，不自动绕回。 |
| 类 `Segment` | 23 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L24）、`sample`（L34） |
| `load_inputs(mapping_path, control_path, waypoints_path)` | 40 | 加载建图/控制/路线，校验字段、数值、模型哈希和观察点一致性。 |
| `make_plan(q0, control, route, geometry, source='live_rtde')` | 83 | 从指定实际起点构造分段路线、时长、诊断和完整计划哈希。 |
| `verify_plan(plan, q, control, route, geometry)` | 109 | 核验计划完整性、当前输入及实际起点是否仍与审核计划一致。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`, `ur7e_real_static.config`, `ur7e_real_static.model`。

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_auto_esdf/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/control.yaml](../../src/ur7e_auto_esdf/config/control.yaml) | 配置分区/键：`robot_ip` |
| [config/mapping.yaml](../../src/ur7e_auto_esdf/config/mapping.yaml) | 配置分区/键：`robot_urdf` |
| [config/view.rviz](../../src/ur7e_auto_esdf/config/view.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [config/waypoints.yaml](../../src/ur7e_auto_esdf/config/waypoints.yaml) | 配置分区/键：`schema_version` |
| [package.xml](../../src/ur7e_auto_esdf/package.xml) | ROS 包名、依赖及构建类型声明。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_auto_esdf/README.md)
- [REAL_TEST_GUIDE.md](../../src/ur7e_auto_esdf/REAL_TEST_GUIDE.md)
- [ToDoList.md](../../src/ur7e_auto_esdf/ToDoList.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
- `report/`：历史实验报告、观测/模型数据和配图/生成脚本。
