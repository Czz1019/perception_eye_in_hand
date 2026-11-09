# ur7e_real_reconstruction：手动真机 RGB-D 重建

[返回项目文档索引](../README.md)

## 职责和边界

节点只读，未创建运动命令发布器或 Action 客户端。scan_design 是离线观察姿态设计，不是执行程序。

## 输入、处理与输出

**数据链路：** 外部驱动 RGB-D、CameraInfo、关节、TF → 停稳校验/本体过滤 → 彩色 TSDF → surface.npz/PLY 与观测档案。

IDLE → start_scan 请求 → STARTING/SCANNING；严格匹配图像时刻关节和 TF，融合有效 RGB-D；freeze_scan → FREEZING → 最后一次提取并保存 → FROZEN；可直接 load 回看。

## 目录结构

```text
ur7e_real_reconstruction/
├── config/
│   ├── real.yaml
│   └── view.rviz
├── hardware/
│   ├── start_camera.sh
│   ├── start_camera_mount.sh
│   └── start_robot_driver.sh
├── launch/
│   └── bringup.launch.py
├── report/
├── scan_design/
│   ├── assets/
│   ├── OBSERVATION_PLAN.md
│   ├── analysis.log
│   ├── analyze_candidates.py
│   ├── candidate_arrays.npz
│   ├── candidates.json
│   ├── final_diagnostics.log
│   ├── reference_direct_entry_check.json
│   ├── render_plan.py
│   ├── route_analysis.json
│   ├── route_analysis.log
│   ├── select_route.py
│   ├── selected_observation_poses.yaml
│   ├── selected_visibility.npz
│   └── validation.json
├── scripts/
│   └── real_reconstruction
├── test/
│   ├── run_integration.py
│   └── test_inputs.py
├── ur7e_real_reconstruction/
│   ├── __init__.py
│   ├── inputs.py
│   └── node.py
├── validation/
├── CMakeLists.txt
├── README.md
├── REAL_TEST_GUIDE.md
├── ToDoList.md
└── package.xml
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `hardware/start_camera.sh`

[查看源码](../../src/ur7e_real_reconstruction/hardware/start_camera.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 PYTHONDONTWRITEBYTECODE=1
exec ros2 launch realsense2_camera rs_launch.py \
 camera_namespace:=camera camera_name:=camera serial_no:=_349622072343 \
 enable_depth:=true enable_color:=true rgb_camera.color_profile:=640x480x15 \
 enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
 pointcloud.enable:=false align_depth.enable:=false depth_module.depth_profile:=640x480x15
```

### `hardware/start_camera_mount.sh`

[查看源码](../../src/ur7e_real_reconstruction/hardware/start_camera_mount.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
exec ros2 run tf2_ros static_transform_publisher --frame-id tool0 --child-frame-id camera_link --x -0.0406379061113624 --y -0.05902311421350448 --z -0.012384254831755863 --qx 0.0025872583947125077 --qy -0.002933202909281628 --qz 0.9251005977793049 --qw -0.3797019705019513
```

### `hardware/start_robot_driver.sh`

[查看源码](../../src/ur7e_real_reconstruction/hardware/start_robot_driver.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source /home/wqj/ur_arm/ros_ur_driver/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONDONTWRITEBYTECODE=1
exec ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur7e robot_ip:=192.168.56.100 reverse_ip:=192.168.56.38 \
  kinematics_params_file:=/home/wqj/ur_arm/my_robot_calibration.yaml \
  use_fake_hardware:=false headless_mode:=false \
  activate_joint_controller:=false launch_rviz:=false
```

### `launch/bringup.launch.py`

[查看源码](../../src/ur7e_real_reconstruction/launch/bringup.launch.py)

组装应用进程、配置路径和可选可视化；参数可改变实际启动路径。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 10 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('config', default_value=str(share / 'config/real.yaml'))
DeclareLaunchArgument('run_rviz', default_value='true')
DeclareLaunchArgument('load', default_value='', description='Saved surface.npz; preview without scanning')
```

### `scan_design/analyze_candidates.py`

[查看源码](../../src/ur7e_real_reconstruction/scan_design/analyze_candidates.py)

离线计算候选观察姿态的可见性、净空与雅可比指标。

源码说明：Offline observation-pose design only. No ROS node, RTDE connection or controller.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `visibility(T)` | 24 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `clearance(q)` | 33 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `jacobian_sigma(q)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_real_static.model`。

### `scan_design/render_plan.py`

[查看源码](../../src/ur7e_real_reconstruction/scan_design/render_plan.py)

汇总离线路线的诊断和图像，供观察方案评审。

源码说明：Finalize offline route diagnostics and figures; never contacts hardware.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `sigma(q)` | 13 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_real_static.model`。

### `scan_design/select_route.py`

[查看源码](../../src/ur7e_real_reconstruction/scan_design/select_route.py)

离线选择观察路线并采样审计，输出姿态和路线诊断，不实现硬件执行。

源码说明：Offline selection and sampled path audit; does not enable or implement execution.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `visibility(T)` | 34 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `audit(a, b, dense=False)` | 53 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`, `ur7e_real_static.model`。

### `scripts/real_reconstruction`

[查看源码](../../src/ur7e_real_reconstruction/scripts/real_reconstruction)

可执行包装器，选择解释器/环境并转交应用入口；实际命令见下方源码，安装名称见构建清单。

```bash
#!/usr/bin/env bash
set -eo pipefail
RS_ENV="${SAGE_CONDA_PREFIX:-$HOME/miniconda3/envs/whole_control}"
export CONDA_PREFIX="$RS_ENV"
export PATH="$RS_ENV/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
exec "$RS_ENV/bin/python" -B -m ur7e_real_reconstruction.node "$@"
```

### `test/run_integration.py`

[查看源码](../../src/ur7e_real_reconstruction/test/run_integration.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Isolated-domain ROS + real CUDA fusion of synthetic RGB-D; never commands a robot.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Hardware` | 31 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L32）、`state`（L38）、`images`（L45） |
| `wait(predicate, timeout=30)` | 64 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `service(name)` | 72 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`, `realtime_nvblox.ros_publishers`, `ur7e_perception_planning_core.geometry`, `ur7e_real_reconstruction.inputs`, `ur7e_real_reconstruction.node`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `Trigger` | `'/ur7e_real_reconstruction/' + name` | 73 |
| `create_publisher` | `JointState` | `c['joint_topic']` | 35 |
| `create_subscription` | `PointCloud2` | `'/ur7e_real_reconstruction/surface_points'` | 111 |
| `create_publisher` | `CameraInfo if 'info' in k else Image` | `c[k]` | 36 |

### `test/test_inputs.py`

[查看源码](../../src/ur7e_real_reconstruction/test/test_inputs.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `sample(t, q=0.0, v=0.0)` | 7 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_stationarity_requires_bracketing_and_uninterrupted_history()` | 12 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_color_rectification_handles_raw_distortion_and_bgr_padding()` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_joint_stream_without_velocity_can_seed_and_detect_motion()` | 38 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_perception_planning_core.geometry`, `ur7e_real_reconstruction.inputs`。

### `ur7e_real_reconstruction/__init__.py`

[查看源码](../../src/ur7e_real_reconstruction/ur7e_real_reconstruction/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

### `ur7e_real_reconstruction/inputs.py`

[查看源码](../../src/ur7e_real_reconstruction/ur7e_real_reconstruction/inputs.py)

校验真机配置/CameraInfo；ColorRectifier 处理彩色畸变；JointHistory 匹配图像时刻且要求停稳。

源码说明：Real sensor validation, rectification and stationary image-time joint matching.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `load_config(path)` | 13 | 读取配置并执行本包规定的合并、路径解析和字段/范围检查。 |
| `stamp(msg)` | 36 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `camera_record(info)` | 40 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `signature(info)` | 45 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `ColorRectifier` | 49 | Raw RealSense color is not assumed rectified or depth-aligned. |
| 方法 | — | `__init__`（L51）、`decode`（L72） |
| 类 `JointHistory` | 78 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L79）、`add`（L81）、`stationary_at`（L96） |

直接项目/规划依赖：`ur7e_nvblox.depth`, `ur7e_perception_planning_core.geometry`, `ur7e_reconstruction.color`。

### `ur7e_real_reconstruction/node.py`

[查看源码](../../src/ur7e_real_reconstruction/ur7e_real_reconstruction/node.py)

Reconstruction 提供 start_scan/freeze_scan，检查 RGB-D 同步、TF/模型/标定并过滤本体；导出彩色 surface 与观测证据。

源码说明：Read-only RGB-D reconstruction. No action clients, goals or command publishers.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `write_json(path, value)` | 39 | 先写临时 JSON 文件再替换目标文件，避免部分写入被当成完整结果。 |
| 类 `Reconstruction` | 44 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L45）、`on_info`（L86）、`on_color_info`（L87）、`on_state`（L88）、`on_pair`（L91）、`start_scan`（L102）、`freeze_scan`（L110）、`new_session`（L118）、`metadata`（L129）、`export`（L134）、`record_view`（L143）、`pick_ready`（L159）、`mapping_loop`（L188）、`publish_loop`（L247）、`report`（L260）、`close`（L273） |
| `main()` | 277 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.ros_publishers`, `ur7e_nvblox.depth`, `ur7e_perception_planning_core.self_filter`, `ur7e_real_static.model`, `ur7e_real_static.trajectory`, `ur7e_reconstruction.color`, `ur7e_reconstruction.surface`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `MarkerArray` | `'~/mesh'` | 58 |
| `create_publisher` | `PointCloud2` | `'~/surface_points'` | 59 |
| `create_publisher` | `PoseStamped` | `'~/camera_pose'` | 60 |
| `create_publisher` | `String` | `'~/status'` | 61 |
| `create_subscription` | `CameraInfo` | `c['camera_info_topic']` | 62 |
| `create_subscription` | `CameraInfo` | `c['color_info_topic']` | 63 |
| `create_subscription` | `JointState` | `c['joint_topic']` | 64 |
| `create_service` | `Trigger` | `'~/start_scan'` | 69 |
| `create_service` | `Trigger` | `'~/freeze_scan'` | 70 |

启动/命令行参数声明：

```python
parser.add_argument('--config', required=True)
parser.add_argument('--load', default='')
```

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_real_reconstruction/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/real.yaml](../../src/ur7e_real_reconstruction/config/real.yaml) | 配置分区/键： |
| [config/view.rviz](../../src/ur7e_real_reconstruction/config/view.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [package.xml](../../src/ur7e_real_reconstruction/package.xml) | ROS 包名、依赖及构建类型声明。 |
| [scan_design/selected_observation_poses.yaml](../../src/ur7e_real_reconstruction/scan_design/selected_observation_poses.yaml) | 配置分区/键：`schema_version` |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_real_reconstruction/README.md)
- [REAL_TEST_GUIDE.md](../../src/ur7e_real_reconstruction/REAL_TEST_GUIDE.md)
- [ToDoList.md](../../src/ur7e_real_reconstruction/ToDoList.md)
- `validation/`：历史测试日志、审计 JSON、图像及验证说明。
- `report/`：历史实验报告、观测/模型数据和配图/生成脚本。
- `scan_design/`：离线观察点设计、候选/选择数组及路线评估结果。
- `hardware/`：外部相机、安装 TF 和机器人驱动启动脚本，独立于只读应用。
