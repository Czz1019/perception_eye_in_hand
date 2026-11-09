# ur7e_nvblox：基于 TF 的眼在手上 ESDF

[返回项目文档索引](../README.md)

## 职责和边界

不直连 RealSense 设备、不运行 SLAM。dense 是规则查询网格，sparse 是候选已观测体素的原生 ESDF 查询；显示点云不自动等于规划地图。

## 输入、处理与输出

**数据链路：** ROS 深度 + CameraInfo + 图像时刻 TF → EfficientBackend → base_link 原生 ESDF。

TfMapper 验证帧时效、内参和 TF；后台融合深度并更新 ESDF；按 dense/sparse 模式组织输出；显示和融合分线程。

## 目录结构

```text
ur7e_nvblox/
├── config/
│   └── gazebo.yaml
├── launch/
│   └── mapping.launch.py
├── resource/
├── rviz/
│   └── esdf.rviz
├── test/
│   ├── run_stream_benchmark.py
│   ├── test_depth.py
│   ├── test_efficient_esdf.py
│   ├── test_gpu_plane.py
│   └── test_timestamp_regression.py
├── ur7e_nvblox/
│   ├── __init__.py
│   ├── depth.py
│   ├── esdf.py
│   ├── node.py
│   └── verify.py
├── PERFORMANCE.md
├── README.md
├── TEST_RESULTS.md
├── package.xml
├── setup.cfg
└── setup.py
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/mapping.launch.py`

[查看源码](../../src/ur7e_nvblox/launch/mapping.launch.py)

加载 gazebo.yaml，选择 Python 解释器并按开关启动 RViz。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 11 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('config', default_value=str(share / 'config/gazebo.yaml'))
DeclareLaunchArgument('python_executable', default_value=str(python))
DeclareLaunchArgument('run_rviz', default_value='false')
```

### `setup.py`

[查看源码](../../src/ur7e_nvblox/setup.py)

ament_python 打包入口，注册 console_scripts 并安装 launch、配置等资源。

### `test/run_stream_benchmark.py`

[查看源码](../../src/ur7e_nvblox/test/run_stream_benchmark.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：ROS synthetic plane/moving camera benchmark; isolated domain, no robot commands.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 23 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.gazebo_ros_node`, `ur7e_nvblox.node`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `Image` | `'/test/depth'` | 40 |
| `create_publisher` | `CameraInfo` | `'/test/info'` | 40 |
| `create_subscription` | `PointCloud2` | `prefix + '/esdf_3d'` | 60 |
| `create_subscription` | `String` | `prefix + '/stats'` | 61 |

启动/命令行参数声明：

```python
parser.add_argument('--output', required=True)
parser.add_argument('--mode', choices=['dense', 'sparse'], default='dense')
parser.add_argument('--reference-config', help='Use the untouched reference Gazebo frontend with this YAML')
parser.add_argument('--seconds', type=float, default=15.0)
```

### `test/test_depth.py`

[查看源码](../../src/ur7e_nvblox/test/test_depth.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_millimeters_endianness_and_padding()` | 9 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_float_invalid_depth_and_decimation()` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_decimated_camera_rays_are_unchanged()` | 24 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_camera_to_base_transform_direction()` | 33 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`ur7e_nvblox.depth`。

### `test/test_efficient_esdf.py`

[查看源码](../../src/ur7e_nvblox/test/test_efficient_esdf.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_prepared_and_sparse_are_native_esdf_with_owned_outputs()` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.types`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`。

### `test/test_gpu_plane.py`

[查看源码](../../src/ur7e_nvblox/test/test_gpu_plane.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：A real CUDA/backend geometry test, enabled explicitly with RUN_GPU_TESTS=1.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_plane_esdf_in_translated_base_frame()` | 13 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`, `realtime_nvblox.types`, `ur7e_nvblox.depth`。

### `test/test_timestamp_regression.py`

[查看源码](../../src/ur7e_nvblox/test/test_timestamp_regression.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Regressions for stale DDS frames following a simulation clock reset.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `test_pre_reset_image_does_not_poison_new_epoch()` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.stats`, `ur7e_nvblox.node`。

### `ur7e_nvblox/__init__.py`

[查看源码](../../src/ur7e_nvblox/ur7e_nvblox/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

源码说明：ROS depth + timestamped TF adapter; no RealSense device or SLAM dependency.

### `ur7e_nvblox/depth.py`

[查看源码](../../src/ur7e_nvblox/ur7e_nvblox/depth.py)

按 ROS step 和字节序解码深度为米值，处理降采样后的内参，构造标定/变换并识别原生未知距离。

源码说明：Depth decoding and calibrated pinhole geometry, independent of ROS runtime.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `depth_meters(msg, stride=1, minimum=0.2, maximum=4.0)` | 8 | Decode ROS depth respecting row padding and byte order; decimate in pixels. |
| `intrinsics_from_info(info, stride=1)` | 27 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `depth_calibration(intrinsics)` | 41 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `transform_matrix(transform)` | 48 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `known_distances(distances, unknown_value)` | 56 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.types`。

### `ur7e_nvblox/esdf.py`

[查看源码](../../src/ur7e_nvblox/ur7e_nvblox/esdf.py)

EfficientBackend 继承 NvbloxBackend，复用 GPU 查询缓冲；稀疏输出用 TSDF 权重筛选坐标、用原生 ESDF 查询距离。

源码说明：Local optimizations; the original realtime_nvblox backends are not modified.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `EfficientBackend` | 11 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `prepare_query`（L12）、`query_prepared`（L22）、`prepare_fixed_grid`（L30）、`observed_esdf_numpy`（L35） |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`。

### `ur7e_nvblox/node.py`

[查看源码](../../src/ur7e_nvblox/ur7e_nvblox/node.py)

TfMapper 订阅深度与内参，查询曝光时刻 TF；后台融合/更新 ESDF，独立发布点云并处理 reset/save。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `TfMapper` | 33 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L34）、`_validate_config`（L92）、`_on_info`（L111）、`_on_clock`（L114）、`_request_reset`（L121）、`_on_depth`（L128）、`_worker`（L152）、`_worker_error`（L251）、`_publisher_loop`（L257）、`_publish_latest`（L268）、`_publish_stats`（L292）、`_reset_service`（L301）、`_save_service`（L307）、`close`（L319） |
| `main(args=None)` | 327 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.ros_publishers`, `realtime_nvblox.stats`, `ur7e_nvblox.depth`, `ur7e_nvblox.esdf`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `CameraInfo` | `self.cfg['camera_info_topic']` | 70 |
| `create_subscription` | `Image` | `self.cfg['depth_topic']` | 71 |
| `create_subscription` | `ClockMsg` | `'/clock'` | 72 |
| `create_publisher` | `PointCloud2` | `'~/esdf_3d'` | 75 |
| `create_publisher` | `PointCloud2` | `'~/esdf_observed'` | 76 |
| `create_publisher` | `PoseStamped` | `'~/camera_pose'` | 77 |
| `create_publisher` | `String` | `'~/stats'` | 78 |
| `create_service` | `Trigger` | `'~/reset_map'` | 79 |
| `create_service` | `Trigger` | `'~/save_map'` | 80 |

### `ur7e_nvblox/verify.py`

[查看源码](../../src/ur7e_nvblox/ur7e_nvblox/verify.py)

对已运行节点进行只读集成核查，检查话题、TF、输出等。

源码说明：Read-only integration check against a running scene and mapper.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 21 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_nvblox.depth`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `PointCloud2` | `'/ur7e_nvblox/esdf_3d'` | 67 |
| `create_subscription` | `PoseStamped` | `'/ur7e_nvblox/camera_pose'` | 70 |
| `create_subscription` | `String` | `'/ur7e_nvblox/stats'` | 71 |

启动/命令行参数声明：

```python
parser.add_argument('--timeout', type=float, default=60)
parser.add_argument('--duration', type=float, default=10)
parser.add_argument('--output', type=Path)
parser.add_argument('--best-effort', action='store_true', help='For full_grid_reliable=false')
```

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [config/gazebo.yaml](../../src/ur7e_nvblox/config/gazebo.yaml) | 配置分区/键：`/**` |
| [package.xml](../../src/ur7e_nvblox/package.xml) | ROS 包名、依赖及构建类型声明。 |
| [rviz/esdf.rviz](../../src/ur7e_nvblox/rviz/esdf.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [setup.cfg](../../src/ur7e_nvblox/setup.cfg) | ament_python 脚本安装路径。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [PERFORMANCE.md](../../src/ur7e_nvblox/PERFORMANCE.md)
- [README.md](../../src/ur7e_nvblox/README.md)
- [TEST_RESULTS.md](../../src/ur7e_nvblox/TEST_RESULTS.md)
