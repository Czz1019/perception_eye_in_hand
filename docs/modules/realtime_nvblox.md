# realtime_nvblox：实时感知基础库

[返回项目文档索引](../README.md)

## 职责和边界

本包是底层实时建图库。robot 模式可使用 TF/PoseStamped，Gazebo 入口直接使用图像时刻 TF；并非所有路径都启用 SLAM。

## 输入、处理与输出

**数据链路：** 直接 RealSense RGB-D/双目/IMU → cuVSLAM 或机器人位姿 → TSDF/颜色/ESDF → ROS/文件。

runtime.start 初始化采集与后端；VIO 线程生成位姿；mapping 线程按图像时间取位姿后融合；独立 ESDF 查询线程查询规则网格；ROS 层异步发布结果。

## 目录结构

```text
realtime_nvblox/
├── .pytest_cache/
│   ├── v/
│   ├── .gitignore
│   ├── CACHEDIR.TAG
│   └── README.md
├── config/
│   ├── default.yaml
│   ├── esdf_benchmark.yaml
│   ├── esdf_final.yaml
│   ├── gazebo_robot.yaml
│   └── robot_eye_in_hand.yaml
├── examples/
│   ├── esdf_rate_monitor.py
│   ├── verify_gazebo_mapping.py
│   └── verify_gazebo_rejection.py
├── launch/
│   ├── gazebo_mapping.launch.py
│   └── reconstruction.launch.py
├── realtime_nvblox/
│   ├── config/
│   ├── tests/
│   ├── ROBOT_MAPPING.md
│   ├── __init__.py
│   ├── check_env.py
│   ├── cli.py
│   ├── config.py
│   ├── cuvslam_backend.py
│   ├── default.yaml
│   ├── gazebo_depth.py
│   ├── gazebo_ros_node.py
│   ├── latest_slot.py
│   ├── nvblox_backend.py
│   ├── pose_buffer.py
│   ├── queues.py
│   ├── rate_limiter.py
│   ├── realsense_source.py
│   ├── robot_eye_in_hand.yaml
│   ├── robot_pose.py
│   ├── ros_node.py
│   ├── ros_publishers.py
│   ├── runtime.py
│   ├── runtime_loader.py
│   ├── stats.py
│   └── types.py
├── resource/
├── rviz/
│   ├── gazebo_esdf.rviz
│   └── realtime_nvblox.rviz
├── scripts/
│   ├── build_ros.sh
│   ├── check_environment.sh
│   ├── check_runtime.sh
│   ├── diagnose_native_libraries.sh
│   ├── install_dependencies.sh
│   ├── run_standalone.sh
│   └── start_clean.sh
├── test/
│   ├── test_config.py
│   ├── test_gazebo_depth.py
│   └── test_pose_buffer.py
├── DEPLOY.md
├── GAZEBO_MAPPING.md
├── LICENSE
├── README.md
├── package.xml
├── requirements.txt
├── setup.cfg
└── setup.py
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `examples/esdf_rate_monitor.py`

[查看源码](../../src/realtime_nvblox/examples/esdf_rate_monitor.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Monitor` | 9 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L10）、`cb`（L24）、`report`（L27） |
| `main()` | 35 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `PointCloud2` | `'/realtime_nvblox/esdf_3d'` | 21 |

### `examples/verify_gazebo_mapping.py`

[查看源码](../../src/realtime_nvblox/examples/verify_gazebo_mapping.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Read-only benchmark by default; --exercise-simulation explicitly permits motion/reset.<br>Run in the same isolated ROS_DOMAIN_ID as Gazebo. Never use with real hardware.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Probe` | 24 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L25）、`stats_cb`（L45）、`cloud_cb`（L49）、`observed_cb`（L55）、`joints_cb`（L60）、`pose_cb`（L63）、`spin`（L66）、`service`（L87）、`wait_ready`（L104） |
| `rate_window(a, b)` | 113 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 121 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `String` | `'/realtime_nvblox/stats'` | 38 |
| `create_subscription` | `PointCloud2` | `'/realtime_nvblox/esdf_3d'` | 39 |
| `create_subscription` | `PointCloud2` | `'/realtime_nvblox/esdf_observed'` | 40 |
| `create_subscription` | `PoseStamped` | `'/realtime_nvblox/camera_pose'` | 41 |
| `create_subscription` | `JointState` | `'/joint_states'` | 42 |
| `create_publisher` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 43 |
| `create_client` | `kind` | `name` | 88 |

启动/命令行参数声明：

```python
parser.add_argument('--exercise-simulation', action='store_true')
parser.add_argument('--prepare-view', action='store_true', help='Only move the simulated wrist to view the obstacles, then exit')
parser.add_argument('--reset-simulation', action='store_true', help='Also test Gazebo global clock reset (some Classic control plugins stop joint states)')
parser.add_argument('--view', choices=('walls', 'sphere'), default='walls', help='Gazebo observation pose')
parser.add_argument('--seconds', type=float, default=20)
parser.add_argument('--output', default='artifacts/gazebo_perception/verification.json')
```

### `examples/verify_gazebo_rejection.py`

[查看源码](../../src/realtime_nvblox/examples/verify_gazebo_rejection.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Isolated synthetic ROS test: exact-time TF failure must never fall back to latest.<br>Requires a separate ROS_DOMAIN_ID with no Gazebo/mapper running.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 23 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.ros_publishers`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `Clock` | `'/clock'` | 32 |
| `create_publisher` | `CameraInfo` | `'/camera/depth/camera_info'` | 33 |
| `create_publisher` | `Image` | `'/camera/depth/image_rect_raw'` | 34 |
| `create_subscription` | `String` | `'/realtime_nvblox/stats'` | 37 |

启动/命令行参数声明：

```python
parser.add_argument('--output', default='artifacts/gazebo_perception/rejection.json')
```

### `launch/gazebo_mapping.launch.py`

[查看源码](../../src/realtime_nvblox/launch/gazebo_mapping.launch.py)

独立仿真深度/TF mapper 启动；不创建 RealSenseSource 或 cuVSLAM。

源码说明：Independent Gazebo mapper; no RealSenseSource or cuVSLAM runtime.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 12 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('config', default_value=str(share / 'config/gazebo_robot.yaml'))
DeclareLaunchArgument('python_executable', default_value='/usr/bin/python3')
DeclareLaunchArgument('run_rviz', default_value='false')
```

### `launch/reconstruction.launch.py`

[查看源码](../../src/realtime_nvblox/launch/reconstruction.launch.py)

启动实时 mapper 及可选 RViz，向节点传入配置与 ROS 参数。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 10 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('config', default_value=default_config)
DeclareLaunchArgument('run_rviz', default_value='false')
```

### `realtime_nvblox/__init__.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

源码说明：Python-first realtime nvblox reconstruction.

### `realtime_nvblox/check_env.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/check_env.py)

检查 Python、wheel、CUDA 与动态库路径，输出部署诊断。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_print_path_warnings()` | 16 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 29 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.runtime_loader`。

### `realtime_nvblox/cli.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/cli.py)

standalone 模式：加载配置、启动 runtime 并在退出时保存/清理。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 11 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.config`, `realtime_nvblox.runtime`。

启动/命令行参数声明：

```python
parser.add_argument('--config', default=None)
parser.add_argument('--duration', type=float, default=0.0, help='Seconds; 0 means until Ctrl+C')
```

### `realtime_nvblox/config.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/config.py)

加载、合并或校验配置；具体字段及身份绑定规则见本文件函数。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `deep_merge(base: dict, override: dict)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `load_config(path: str \| Path \| None=None)` | 20 | 读取配置并执行本包规定的合并、路径解析和字段/范围检查。 |

### `realtime_nvblox/cuvslam_backend.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/cuvslam_backend.py)

封装双目/IMU 标定与 cuVSLAM 跟踪，将连续里程计转换为 PoseSample。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `CuVslamBackend` | 10 | PyCuVSLAM v15-compatible stereo VIO wrapper. |
| 方法 | — | `__init__`（L18）、`_pose_from_matrix`（L31）、`_camera`（L35）、`_imu`（L45）、`_create_tracker`（L61）、`register_imu`（L84）、`_matrix_from_pose`（L93）、`track`（L99） |

直接项目/规划依赖：`realtime_nvblox.runtime_loader`, `realtime_nvblox.types`。

### `realtime_nvblox/gazebo_depth.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/gazebo_depth.py)

ROS 深度解码、内参与齐次变换构造，以及时间戳拒绝规则；不依赖相机 SDK。

源码说明：ROS depth decoding and exact-time geometry, independent of camera SDKs.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `stamp_ns(stamp)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `intrinsics_from_info(info, stride: int=1)` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `depth_meters(msg, stride: int=1, min_depth_m: float=0.2, max_depth_m: float=4.0)` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `calibration_from_info(info, stride: int=1)` | 42 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `transform_matrix(transform)` | 47 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `stamp_rejection(stamp: int, now: int, floor: int, last: int, max_age_ns: int)` | 57 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.types`。

### `realtime_nvblox/gazebo_ros_node.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/gazebo_ros_node.py)

独立 Gazebo 深度与图像时刻 TF 建图入口；管理 reset/save、时钟回退、后台建图和发布。

源码说明：Gazebo depth + exposure-time TF -> shared NvbloxBackend (no camera SDK/SLAM).

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Input` | 38 | 相关状态与操作的封装；方法见下一行。 |
| 类 `Output` | 46 | 相关状态与操作的封装；方法见下一行。 |
| 类 `GazeboNvbloxNode` | 56 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L57）、`_info_cb`（L115）、`_static_cb`（L119）、`_reset_locked`（L124）、`_clock_cb`（L134）、`_depth_cb`（L145）、`_reset_cb`（L171）、`_save_cb`（L177）、`_save_pending`（L192）、`_input_valid`（L208）、`_lookup`（L218）、`_mapping_guarded`（L223）、`_mapping_loop`（L234）、`_publish_guarded`（L317）、`_stats_cb`（L347）、`close`（L354） |
| `main(args=None)` | 362 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.gazebo_depth`, `realtime_nvblox.latest_slot`, `realtime_nvblox.nvblox_backend`, `realtime_nvblox.ros_publishers`, `realtime_nvblox.stats`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PointCloud2` | `'~/esdf_3d'` | 95 |
| `create_publisher` | `PointCloud2` | `'~/esdf_observed'` | 96 |
| `create_publisher` | `PoseStamped` | `'~/camera_pose'` | 97 |
| `create_publisher` | `String` | `'~/stats'` | 98 |
| `create_subscription` | `CameraInfo` | `self.cfg['camera_info_topic']` | 99 |
| `create_subscription` | `Image` | `self.cfg['depth_topic']` | 100 |
| `create_subscription` | `ClockMsg` | `'/clock'` | 101 |
| `create_subscription` | `TFMessage` | `'/tf_static'` | 103 |
| `create_service` | `Trigger` | `'~/reset_map'` | 105 |
| `create_service` | `Trigger` | `'~/save_map'` | 106 |

### `realtime_nvblox/latest_slot.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/latest_slot.py)

线程安全的最新值交换槽，序号用于判断新结果；生产者覆盖旧值避免积压。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `LatestSlot` | 9 | Thread-safe latest-only handoff. |
| 方法 | — | `__init__`（L17）、`put`（L23）、`get_if_newer`（L32）、`wait_newer`（L38）、`latest`（L48）、`close`（L52） |

### `realtime_nvblox/nvblox_backend.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/nvblox_backend.py)

串行封装 GPU Mapper 的深度/颜色融合、ESDF、网格查询和地图/网格保存。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `preload_nvblox_runtime()` | 18 | Load the nvblox shared library bundled with the nvblox_torch wheel. |
| 类 `NvbloxBackend` | 36 | Thread-safe wrapper around one nvblox_torch Mapper. |
| 方法 | — | `__init__`（L44）、`_initialize`（L65）、`_pose`（L90）、`integrate_depth`（L93）、`integrate_color`（L105）、`update_esdf`（L110）、`update_mesh`（L114）、`mesh_numpy`（L118）、`_prepare_esdf_query`（L127）、`query_esdf`（L138）、`query_esdf_with_gradients`（L145）、`_grid_key_from_cfg`（L153）、`grid_offsets`（L162）、`prepare_esdf_grid`（L174）、`query_esdf_grid`（L191）、`save_map`（L219）、`save_mesh`（L226） |

直接项目/规划依赖：`realtime_nvblox.types`。

### `realtime_nvblox/pose_buffer.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/pose_buffer.py)

保存带时间戳位姿，对平移插值、旋转 Slerp；支持有界等待与最近邻容差。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `PoseBuffer` | 13 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L14）、`add`（L18）、`latest`（L28）、`query`（L32）、`_nearest_locked`（L44）、`_query_locked`（L53） |

直接项目/规划依赖：`realtime_nvblox.types`。

### `realtime_nvblox/queues.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/queues.py)

有容量上限的队列，满时丢弃最旧元素并保留新输入。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `DropOldestQueue` | 10 | Bounded queue that always preserves the newest data. |
| 方法 | — | `__init__`（L13）、`put`（L18）、`get`（L28）、`get_nowait`（L31）、`qsize`（L34）、`empty`（L37） |

### `realtime_nvblox/rate_limiter.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/rate_limiter.py)

按频率保持调度相位；长时间停顿后丢弃累计欠账，避免追赶式突发。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `RateLimiter` | 7 | Preserve phase across jitter; discard accumulated debt after long stalls. |
| 方法 | — | `__init__`（L10）、`remaining`（L16）、`ready`（L22） |

### `realtime_nvblox/realsense_source.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/realsense_source.py)

直接管理 RealSense 视频、彩色及 IMU 采集，读取标定并区分投射器开关帧。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_intrinsics(profile)` | 23 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_extrinsics_matrix(source_profile, target_profile)` | 32 | Return T_target_source from librealsense source.get_extrinsics_to(target). |
| `_device_timestamp_ns(frame)` | 41 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `EmitterClassifier` | 45 | Python equivalent of the RealSense splitter's emitter-state selection. |
| 方法 | — | `__init__`（L53）、`is_on`（L60） |
| 类 `SourceQueues` | 93 | 相关状态与操作的封装；方法见下一行。 |
| 类 `RealSenseSource` | 100 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L101）、`_timestamp_ns`（L128）、`_serial`（L147）、`_nearest_motion_fps`（L161）、`_calibrate_once`（L175）、`start`（L243）、`_motion_callback`（L340）、`_video_loop`（L369）、`_color_loop`（L412）、`stop`（L432） |

直接项目/规划依赖：`realtime_nvblox.queues`, `realtime_nvblox.types`。

### `realtime_nvblox/robot_pose.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/robot_pose.py)

插值 base→tool 运动后施加手眼外参，得到图像时刻相机位姿。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `rigid_transform(value, name: str)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `RobotPoseProvider` | 20 | Interpolate base-from-tool before applying the hand-eye lever arm. |
| 方法 | — | `__init__`（L23）、`configure_calibration`（L67）、`add`（L76）、`query`（L82） |

直接项目/规划依赖：`realtime_nvblox.pose_buffer`, `realtime_nvblox.types`。

### `realtime_nvblox/ros_node.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/ros_node.py)

将 runtime 事件转换为 ROS 位姿、轨迹、网格、ESDF、统计及保存服务；支持机器人位姿输入。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `RealtimeNvbloxNode` | 27 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L28）、`_pose_matrix`（L95）、`_lookup_tool_pose`（L106）、`_on_robot_pose`（L118）、`_publish_static_extrinsics`（L138）、`_on_pose`（L148）、`_on_mesh`（L161）、`_on_slice`（L168）、`_publish_latest_esdf3d`（L174）、`_on_stats`（L201）、`_save_map`（L206）、`_save_mesh`（L214）、`_health`（L222）、`stop`（L228） |
| `main(args=None)` | 232 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.config`, `realtime_nvblox.ros_publishers`, `realtime_nvblox.runtime`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `PoseStamped` | `'/realtime_nvblox/camera_pose'` | 45 |
| `create_publisher` | `PathMsg` | `'/realtime_nvblox/camera_path'` | 46 |
| `create_publisher` | `MarkerArray` | `'/realtime_nvblox/mesh'` | 47 |
| `create_publisher` | `PointCloud2` | `'/realtime_nvblox/esdf_slice'` | 48 |
| `create_publisher` | `PointCloud2` | `'/realtime_nvblox/esdf_3d'` | 49 |
| `create_publisher` | `String` | `'/realtime_nvblox/stats'` | 50 |
| `create_service` | `Trigger` | `'/realtime_nvblox/save_map'` | 87 |
| `create_service` | `Trigger` | `'/realtime_nvblox/save_mesh'` | 88 |
| `create_subscription` | `msg_type` | `pcfg['topic']` | 74 |

### `realtime_nvblox/ros_publishers.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/ros_publishers.py)

把 NumPy 几何打包为 PointCloud2、MarkerArray、Pose 和 TF；距离着色保留数值字段。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `ns_to_stamp(timestamp_ns: int)` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `header(frame_id: str, timestamp_ns: int)` | 21 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `pose_stamped(T: np.ndarray, frame_id: str, timestamp_ns: int)` | 28 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `transform_stamped(T_parent_child: np.ndarray, parent: str, child: str, timestamp_ns: int)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_distance_to_rgb_u32(distances: np.ndarray, max_distance_m: float, unknown_mask: np.ndarray \| None=None)` | 47 | Convert absolute ESDF distance to packed 0x00RRGGBB colors. |
| `esdf_cloud(points: np.ndarray, distances: np.ndarray, frame_id: str, timestamp_ns: int, max_distance_m: float, full_grid: bool=False, unknown_distance_m: float \| None=None)` | 112 | Pack ESDF into a single colored PointCloud2. |
| `mesh_markers(vertices: np.ndarray, triangles: np.ndarray, colors: np.ndarray, frame_id: str, timestamp_ns: int, max_triangles: int=30000, chunk_triangles: int=10000)` | 197 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `realtime_nvblox/runtime.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/runtime.py)

组织采集、VIO、融合、3D ESDF 查询线程和回调；robot 模式可改用机器人位姿。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `RealtimeNvbloxRuntime` | 27 | RealSense + cuVSLAM + nvblox_torch realtime reconstruction runtime. |
| 方法 | — | `__init__`（L38）、`on`（L74）、`_emit`（L79）、`start`（L86）、`_set_error`（L112）、`_vio_loop_guarded`（L119）、`_mapping_loop_guarded`（L125）、`_esdf3d_loop_guarded`（L131）、`_vio_loop`（L137）、`add_robot_pose`（L203）、`_pose_for`（L210）、`_grid_2d`（L229）、`_center_from_pose`（L237）、`_mapping_loop`（L255）、`_esdf3d_loop`（L352）、`_query_center`（L388）、`_output_esdf_slice`（L393）、`_extra_stats`（L409）、`_publish_stats`（L435）、`save_map`（L438）、`save_mesh`（L443）、`stop`（L448）、`run_forever`（L470） |

直接项目/规划依赖：`realtime_nvblox.cuvslam_backend`, `realtime_nvblox.latest_slot`, `realtime_nvblox.nvblox_backend`, `realtime_nvblox.pose_buffer`, `realtime_nvblox.queues`, `realtime_nvblox.rate_limiter`, `realtime_nvblox.realsense_source`, `realtime_nvblox.robot_pose`, `realtime_nvblox.stats`, `realtime_nvblox.types`。

### `realtime_nvblox/runtime_loader.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/runtime_loader.py)

定位 cuVSLAM/nvblox wheel，预加载其配套动态库，避免加载到其他环境的同名库。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_dist_package_dir(distribution_name: str, package_name: str)` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `cuvslam_package_dir()` | 40 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `nvblox_torch_package_dir()` | 44 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `preload_cuvslam_runtime()` | 48 | Preload the ``libcuvslam.so`` that belongs to the installed cuvslam wheel. |
| `preload_nvblox_runtime()` | 76 | Preload the ``libnvblox_lib.so`` that belongs to nvblox_torch wheel. |

### `realtime_nvblox/stats.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/stats.py)

线程安全地累计计数、速率与延迟，并导出统计快照。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `RuntimeStats` | 12 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L13）、`inc`（L21）、`add_latency`（L25）、`snapshot`（L29）、`write_json`（L56） |

### `realtime_nvblox/tests/test_gpu_esdf.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/tests/test_gpu_esdf.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_depth_fusion_grid_query_and_buffer_ownership()` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`, `realtime_nvblox.types`。

### `realtime_nvblox/tests/test_robot_esdf.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/tests/test_robot_esdf.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_rate_does_not_halve_near_target(input_hz)` | 19 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_rate_skips_missed_deadlines_without_burst()` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_exact_pose_and_strict_no_extrapolation()` | 34 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `robot_config()` | 43 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `calibration()` | 47 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_provided_handeye_and_color_to_rig_conversion()` | 51 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_tool_interpolation_preserves_camera_lever_arm()` | 71 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_missing_or_invalid_handeye_rejected()` | 91 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_grid_symmetric_inside_box_with_exact_spacing(resolution)` | 102 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_grid_invalid_resolution_rejected()` | 113 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_realsense_rotation_matches_sdk()` | 118 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `runtime(monkeypatch)` | 130 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_query_keeps_early_request_and_uses_latest_snapshot(monkeypatch)` | 139 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_dirty_esdf_updated_without_another_depth_frame(monkeypatch)` | 171 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_robot_timestamp_domain_checked()` | 200 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_robot_video_accepts_every_depth_without_ir_or_emitter_classifier()` | 218 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_robot_runtime_start_does_not_initialize_cuvslam(monkeypatch)` | 243 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_rate_alternating_arrival_jitter_does_not_halve()` | 259 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.config`, `realtime_nvblox.nvblox_backend`, `realtime_nvblox.pose_buffer`, `realtime_nvblox.queues`, `realtime_nvblox.rate_limiter`, `realtime_nvblox.realsense_source`, `realtime_nvblox.robot_pose`, `realtime_nvblox.types`。

### `realtime_nvblox/tests/test_ros_robot_pose.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/tests/test_ros_robot_pose.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_ros_tf_lookup_uses_exposure_time_and_rejects_extrapolation()` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.ros_node`, `realtime_nvblox.stats`。

### `realtime_nvblox/types.py`

[查看源码](../../src/realtime_nvblox/realtime_nvblox/types.py)

定义相机内参、标定、IMU、RGB-D 帧、位姿、Mesh、ESDF 及查询请求的数据结构。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `CameraIntrinsics` | 10 | 相关状态与操作的封装；方法见下一行。 |
| 类 `Calibration` | 20 | 相关状态与操作的封装；方法见下一行。 |
| 类 `ImuSample` | 34 | 相关状态与操作的封装；方法见下一行。 |
| 类 `VioFrame` | 41 | 相关状态与操作的封装；方法见下一行。 |
| 类 `DepthFrame` | 48 | 相关状态与操作的封装；方法见下一行。 |
| 类 `ColorFrame` | 54 | 相关状态与操作的封装；方法见下一行。 |
| 类 `PoseSample` | 60 | 相关状态与操作的封装；方法见下一行。 |
| 类 `MeshData` | 70 | 相关状态与操作的封装；方法见下一行。 |
| 类 `EsdfData` | 78 | 相关状态与操作的封装；方法见下一行。 |
| 类 `Esdf3DRequest` | 86 | 相关状态与操作的封装；方法见下一行。 |

### `scripts/build_ros.sh`

[查看源码](../../src/realtime_nvblox/scripts/build_ros.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail

# This project needs ROS 2 only for the optional publisher/RViz layer.
# Never source isaac_ros-dev here.
source /opt/ros/humble/setup.bash
WS=${1:-$HOME/realtime_nvblox_ws}
cd "$WS"
rm -rf build/realtime_nvblox install/realtime_nvblox

# For deployment we intentionally use a normal install.  This avoids depending
# on setuptools' legacy "develop --editable" behavior.  If you specifically
# want symlink-install while developing, install setuptools<80 and run colcon
# manually with --symlink-install.
colcon build --packages-select realtime_nvblox
```

### `scripts/check_environment.sh`

[查看源码](../../src/realtime_nvblox/scripts/check_environment.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail
python3 -m realtime_nvblox.check_env
```

### `scripts/check_runtime.sh`

[查看源码](../../src/realtime_nvblox/scripts/check_runtime.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/realtime_nvblox_ws/install/local_setup.bash" ]]; then
  source "$HOME/realtime_nvblox_ws/install/local_setup.bash"
fi

echo '--- nodes ---'
ros2 node list | grep realtime_nvblox || true

echo '--- topics ---'
ros2 topic list | grep realtime_nvblox || true

echo '--- pose rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/camera_pose || true

echo '--- mesh rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/mesh || true

echo '--- ESDF slice rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/esdf_slice || true

echo '--- 3D ESDF rate ---'
timeout 6 ros2 topic hz /realtime_nvblox/esdf_3d || true

echo '--- stats ---'
timeout 3 ros2 topic echo /realtime_nvblox/stats --once || true
```

### `scripts/diagnose_native_libraries.sh`

[查看源码](../../src/realtime_nvblox/scripts/diagnose_native_libraries.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
from importlib import metadata
from pathlib import Path

for dist_name, pkg_name, py_pattern, core_name in [
    ('cuvslam', 'cuvslam', 'pycuvslam*.so', 'libcuvslam.so'),
    ('nvblox-torch', 'nvblox_torch', 'lib/nvblox_torch/cpp/libpy_nvblox.so', 'lib/nvblox/libnvblox_lib.so'),
]:
    d = metadata.distribution(dist_name)
    pkg = Path(d.locate_file(pkg_name)).resolve()
    print(f'{dist_name}: {pkg}')
    if '*' in py_pattern:
        matches = list(pkg.glob(py_pattern))
    else:
        matches = [pkg / py_pattern]
    for m in matches:
        print(' binding:', m)
    print(' core   :', pkg / core_name)
PY

echo
echo 'LD_LIBRARY_PATH entries:'
echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | nl -ba

echo
CUV_DIR=$(python3 - <<'PY'
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution('cuvslam').locate_file('cuvslam')).resolve())
PY
)
CUV_BIND=$(find "$CUV_DIR" -maxdepth 2 -name 'pycuvslam*.so' -print -quit)
if [[ -n "$CUV_BIND" ]]; then
  echo 'cuVSLAM binding resolution:'
  ldd "$CUV_BIND" | grep -E 'libcuvslam|not found|cublas|cusolver|cusparse|cudart' || true
fi

echo
NVB_DIR=$(python3 - <<'PY'
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution('nvblox-torch').locate_file('nvblox_torch')).resolve())
PY
)
NVB_BIND="$NVB_DIR/lib/nvblox_torch/cpp/libpy_nvblox.so"
if [[ -f "$NVB_BIND" ]]; then
  echo 'nvblox binding resolution:'
  ldd "$NVB_BIND" | grep -E 'libnvblox_lib|libtorch|libc10|not found' || true
fi
```

### `scripts/install_dependencies.sh`

[查看源码](../../src/realtime_nvblox/scripts/install_dependencies.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail

PYTHON=${PYTHON:-python3}
PIP=("$PYTHON" -m pip install --no-cache-dir)
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  PIP+=(--user)
fi

echo '[1/7] Python packaging tools'
# IMPORTANT: ROS 2 Humble/colcon --symlink-install is currently incompatible
# with setuptools >= 80 because colcon still invokes setup.py develop --editable.
# Do NOT upgrade setuptools to latest here.
"${PIP[@]}" --upgrade pip wheel 'setuptools<80'

echo '[2/7] Numeric + RealSense dependencies'
"${PIP[@]}" numpy==1.26.4 'scipy>=1.10,<1.16' 'PyYAML>=6.0' \
  pyrealsense2==2.57.7.10387

echo '[3/7] PyTorch CUDA 12.8'
"${PIP[@]}" --index-url https://download.pytorch.org/whl/cu128 \
  torch==2.9.1 torchvision==0.24.1

echo '[4/7] nvblox_torch official Ubuntu 22.04 CUDA12 wheel'
"${PIP[@]}" --no-deps \
  'https://github.com/nvidia-isaac/nvblox/releases/download/v0.0.10/nvblox_torch-0.0.10%2Bcu12ubuntu22-py3-none-linux_x86_64.whl'

echo '[5/7] PyCuVSLAM v15 official Ubuntu 22.04/Python3.10/CUDA12 wheel'
"${PIP[@]}" --no-deps \
  'https://github.com/nvidia-isaac/cuVSLAM/releases/download/v15.0.0/cuvslam-15.0.0%2Bcu12-cp310-cp310-manylinux_2_35_x86_64.whl'

echo '[6/7] Remaining nvblox_torch Python dependencies'
"${PIP[@]}" 'open3d>=0.19' 'opencv-python>=4.8' 'einops>=0.7' 'imageio>=2.30' \
  'matplotlib>=3.7' 'nvtx>=0.2' 'timm>=1.0' 'transforms3d>=0.4'

echo '[7/7] Install this package editable for standalone use'
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
"$PYTHON" -m pip install --user -e "$ROOT"

echo
echo 'Dependency installation finished.'
echo 'IMPORTANT: for standalone testing, open a NEW terminal and do NOT source isaac_ros-dev.'
echo 'Then run:'
echo '  cd ~/realtime_nvblox_ws/src/realtime_nvblox'
echo '  python3 -m realtime_nvblox.check_env'
```

### `scripts/run_standalone.sh`

[查看源码](../../src/realtime_nvblox/scripts/run_standalone.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec python3 -m realtime_nvblox.cli --config "$ROOT/config/default.yaml" "$@"
```

### `scripts/start_clean.sh`

[查看源码](../../src/realtime_nvblox/scripts/start_clean.sh)

环境/部署辅助脚本：按下列实际命令准备依赖、解释器或执行入口。

```bash
#!/usr/bin/env bash
set -euo pipefail

ROOT="${REALTIME_NVBLOX_ROOT:-$HOME/realtime_nvblox_ws/src/realtime_nvblox}"
CONFIG="${REALTIME_NVBLOX_CONFIG:-$ROOT/config/default.yaml}"

if [[ ! -d "$ROOT/realtime_nvblox" ]]; then
  echo "ERROR: realtime_nvblox project not found: $ROOT" >&2
  exit 2
fi

# Re-exec in an intentionally clean process environment.  This makes the
# command safe even when ~/.bashrc has sourced Isaac ROS or other workspaces.
exec env -i \
  HOME="$HOME" \
  USER="${USER:-$(id -un)}" \
  PATH="$HOME/.local/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  LD_LIBRARY_PATH="/usr/local/cuda/lib64" \
  ROOT="$ROOT" \
  CONFIG="$CONFIG" \
  bash --noprofile --norc -c '
    set -euo pipefail
    cd "$ROOT"

    CUV_DIR=$(python3 - <<"PY"
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution("cuvslam").locate_file("cuvslam")).resolve())
PY
    )

    NVB_DIR=$(python3 - <<"PY"
from importlib import metadata
from pathlib import Path
print(Path(metadata.distribution("nvblox-torch").locate_file("nvblox_torch")).resolve())
PY
    )

    TORCH_LIB=$(python3 - <<"PY"
import os, torch
print(os.path.join(os.path.dirname(torch.__file__), "lib"))
PY
    )

    export LD_LIBRARY_PATH="$CUV_DIR:$NVB_DIR/lib/nvblox:$TORCH_LIB:/usr/local/cuda/lib64"

    echo "[realtime_nvblox] clean runtime"
    echo "[realtime_nvblox] config: $CONFIG"
    exec python3 -m realtime_nvblox.cli --config "$CONFIG" "$@"
  ' bash "$@"
```

### `setup.py`

[查看源码](../../src/realtime_nvblox/setup.py)

ament_python 打包入口，注册 console_scripts 并安装 launch、配置等资源。

### `test/test_config.py`

[查看源码](../../src/realtime_nvblox/test/test_config.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_default_config()` | 4 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.config`。

### `test/test_gazebo_depth.py`

[查看源码](../../src/realtime_nvblox/test/test_gazebo_depth.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `info()` | 14 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_padding_endianness_units_stride(encoding, kind, scale, big)` | 21 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_invalid_depth_and_buffer()` | 32 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_rectified_p_fallback_k_and_scaled_pixel_geometry()` | 41 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_sim_time_epoch_bounds()` | 51 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_gazebo_import_does_not_import_hardware_or_slam()` | 60 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_lookup_interpolates_only_requested_stamp_and_rejects_latest()` | 68 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_reset_clears_queues_and_rejects_inflight_epoch()` | 93 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.gazebo_depth`, `realtime_nvblox.gazebo_ros_node`, `realtime_nvblox.latest_slot`, `realtime_nvblox.stats`。

### `test/test_pose_buffer.py`

[查看源码](../../src/realtime_nvblox/test/test_pose_buffer.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_pose_interpolation()` | 6 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.pose_buffer`, `realtime_nvblox.types`。

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [config/default.yaml](../../src/realtime_nvblox/config/default.yaml) | 配置分区/键：`frames` |
| [config/esdf_benchmark.yaml](../../src/realtime_nvblox/config/esdf_benchmark.yaml) | 配置分区/键：`frames` |
| [config/esdf_final.yaml](../../src/realtime_nvblox/config/esdf_final.yaml) | 配置分区/键：`frames` |
| [config/gazebo_robot.yaml](../../src/realtime_nvblox/config/gazebo_robot.yaml) | 配置分区/键： |
| [config/robot_eye_in_hand.yaml](../../src/realtime_nvblox/config/robot_eye_in_hand.yaml) | 配置分区/键： |
| [package.xml](../../src/realtime_nvblox/package.xml) | ROS 包名、依赖及构建类型声明。 |
| [realtime_nvblox/config/default.yaml](../../src/realtime_nvblox/realtime_nvblox/config/default.yaml) | 配置分区/键：`frames` |
| [realtime_nvblox/config/esdf_benchmark.yaml](../../src/realtime_nvblox/realtime_nvblox/config/esdf_benchmark.yaml) | 配置分区/键：`frames` |
| [realtime_nvblox/config/esdf_final.yaml](../../src/realtime_nvblox/realtime_nvblox/config/esdf_final.yaml) | 配置分区/键：`frames` |
| [realtime_nvblox/default.yaml](../../src/realtime_nvblox/realtime_nvblox/default.yaml) | 配置分区/键：`pose_source` |
| [realtime_nvblox/robot_eye_in_hand.yaml](../../src/realtime_nvblox/realtime_nvblox/robot_eye_in_hand.yaml) | 配置分区/键： |
| [requirements.txt](../../src/realtime_nvblox/requirements.txt) | Python 依赖清单；并不保证系统库/GPU 环境齐备。 |
| [rviz/gazebo_esdf.rviz](../../src/realtime_nvblox/rviz/gazebo_esdf.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [rviz/realtime_nvblox.rviz](../../src/realtime_nvblox/rviz/realtime_nvblox.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [setup.cfg](../../src/realtime_nvblox/setup.cfg) | ament_python 脚本安装路径。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [DEPLOY.md](../../src/realtime_nvblox/DEPLOY.md)
- [GAZEBO_MAPPING.md](../../src/realtime_nvblox/GAZEBO_MAPPING.md)
- [README.md](../../src/realtime_nvblox/README.md)
