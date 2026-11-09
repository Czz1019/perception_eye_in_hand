# ur7e_reconstruction：仿真眼在手彩色重建

[返回项目文档索引](../README.md)

## 职责和边界

只重建表面，不更新 ESDF。evaluate 和 check_scan 使用参考场景做离线评估，参考几何不进入 TSDF 融合。

## 输入、处理与输出

**数据链路：** 深度/颜色 + 各自图像时刻 TF → TSDF 表面与颜色 → surface points、Mesh、PLY。

节点解码图像并融合；SurfaceMapper 提取三角网格；独立线程发布；可选 Scanner 按预定关节姿态慢速移动并开关融合。

## 目录结构

```text
ur7e_reconstruction/
├── config/
│   ├── gazebo.yaml
│   ├── high_initial_positions.yaml
│   └── scan_views.yaml
├── launch/
│   ├── reconstruction.launch.py
│   └── scan_scene.launch.py
├── resource/
├── rviz/
│   └── reconstruction.rviz
├── test/
│   └── test_surface.py
├── ur7e_reconstruction/
│   ├── __init__.py
│   ├── check_scan.py
│   ├── color.py
│   ├── evaluate.py
│   ├── node.py
│   ├── scan.py
│   ├── surface.py
│   └── verify.py
├── README.md
├── TEST_RESULTS.md
├── package.xml
├── setup.cfg
└── setup.py
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/reconstruction.launch.py`

[查看源码](../../src/ur7e_reconstruction/launch/reconstruction.launch.py)

启动彩色重建、可选仿真扫描与 RViz，读取建图及扫描配置。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 11 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('config', default_value=str(share / 'config/gazebo.yaml'))
DeclareLaunchArgument('python_executable', default_value=str(Path.home() / 'miniconda3/envs/whole_control/bin/python'))
DeclareLaunchArgument('run_rviz', default_value='true')
DeclareLaunchArgument('scan', default_value='false')
```

### `launch/scan_scene.launch.py`

[查看源码](../../src/ur7e_reconstruction/launch/scan_scene.launch.py)

复用场景资源，并采用适合扫描的初始关节姿态。

源码说明：Reuse the tested scene assets with a higher initial observation pose.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_remove_comments(node)` | 16 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_setup(context)` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `generate_launch_description()` | 53 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('gazebo_gui', default_value='true', description='Open the Gazebo GUI')
```

### `setup.py`

[查看源码](../../src/ur7e_reconstruction/setup.py)

ament_python 打包入口，注册 console_scripts 并安装 launch、配置等资源。

### `test/test_surface.py`

[查看源码](../../src/ur7e_reconstruction/test/test_surface.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `sample()` | 13 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_roi_preserves_connectivity_and_removes_outside_faces()` | 18 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_empty_snapshot_clears_old_rviz_triangles()` | 25 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_ply_and_cloud_preserve_metric_vertices(tmp_path)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_real_gpu_plane_reconstruction_without_esdf()` | 51 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_crop_keeps_actual_vertex_colors()` | 72 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_rgb_padded_rows_bgr_conversion()` | 79 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `test_real_gpu_rgb_integration_with_separate_camera_origin()` | 88 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`, `realtime_nvblox.types`, `ur7e_reconstruction.color`, `ur7e_reconstruction.surface`。

### `ur7e_reconstruction/__init__.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/__init__.py)

Python 包标识；有些文件只有包级说明，无运行逻辑。

源码说明：Depth/TF-based surface reconstruction. No SLAM or ESDF processing.

### `ur7e_reconstruction/check_scan.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/check_scan.py)

用已知仿真几何对离线路线进行采样净空检查，不向重建融合注入参考几何。

源码说明：Offline sampled clearance check against the unchanged Gazebo scene geometry.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `fk(link, q)` | 12 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `origin(elem)` | 22 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `clearance(q)` | 46 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 60 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('--views', type=Path, default=Path(get_package_share_directory('ur7e_reconstruction')) / 'config/scan_views.yaml')
parser.add_argument('--output', type=Path, required=True)
```

### `ur7e_reconstruction/color.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/color.py)

解码 ROS RGB/BGR 等颜色图像，保留行步长处理，不假定深度和彩色已经对齐。

源码说明：Decode ROS color images without assuming depth alignment or tightly packed rows.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `rgb_image(msg, stride=1)` | 5 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `ur7e_reconstruction/evaluate.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/evaluate.py)

读取生成的 PLY，与盒、球等参考场景比较重建误差；真值仅用于评价。

源码说明：Compare a measured PLY against scene primitives; reference data never enters fusion.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `read_ply(path)` | 11 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `box_reference(center, dims, step=0.005)` | 19 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `box_distance(points, center, dims)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `evaluate(path, scene, tolerance=0.025)` | 42 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 103 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('--mesh', type=Path, default=Path.home() / 'nvblox_output/ur7e_reconstruction/base_link_surface.ply')
parser.add_argument('--scene', type=Path, default=Path(get_package_share_directory('ur7e_d435_gazebo')) / 'config/collision_world_gazebo_tall.yml')
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--plot', type=Path)
```

### `ur7e_reconstruction/node.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/node.py)

深度/颜色/图像时刻 TF 重建主节点，维护融合开关、reset/save、独立网格发布和状态。

源码说明：Eye-in-hand TSDF reconstruction with image-time TF and separate mesh publication.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `ReconstructionNode` | 33 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L34）、`_on_info`（L106）、`_on_color_info`（L109）、`_on_joints`（L112）、`_on_clock`（L125）、`_request_reset`（L132）、`_on_depth`（L139）、`_mapping_loop`（L164）、`_fail`（L236）、`_publish_loop`（L242）、`_report`（L282）、`_set_integration`（L290）、`_reset`（L296）、`_save`（L301）、`close`（L313） |
| `main(args=None)` | 321 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`realtime_nvblox.latest_slot`, `realtime_nvblox.ros_publishers`, `realtime_nvblox.stats`, `ur7e_nvblox.depth`, `ur7e_reconstruction.color`, `ur7e_reconstruction.surface`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `CameraInfo` | `c['camera_info_topic']` | 82 |
| `create_subscription` | `CameraInfo` | `c['color_info_topic']` | 83 |
| `create_subscription` | `JointState` | `'/joint_states'` | 84 |
| `create_subscription` | `ClockMsg` | `'/clock'` | 90 |
| `create_publisher` | `MarkerArray` | `'~/mesh'` | 91 |
| `create_publisher` | `PointCloud2` | `'~/surface_points'` | 92 |
| `create_publisher` | `PoseStamped` | `'~/camera_pose'` | 93 |
| `create_publisher` | `String` | `'~/stats'` | 94 |
| `create_service` | `Trigger` | `'~/reset_map'` | 95 |
| `create_service` | `SetBool` | `'~/set_integration_enabled'` | 96 |
| `create_service` | `Trigger` | `'~/save_mesh'` | 97 |

### `ur7e_reconstruction/scan.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/scan.py)

Gazebo 慢速关节扫描，按反馈检查到点并控制建图开关；该文件会向仿真控制器发指令。

源码说明：Slow, feedback-checked Gazebo scanning using the existing position controller.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Scanner` | 16 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L17）、`joints`（L34）、`report`（L40）、`spin`（L43）、`wait_until`（L48）、`count`（L56）、`set_integration`（L59）、`move`（L66）、`run`（L84） |
| `main(args=None)` | 118 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `JointState` | `'/joint_states'` | 27 |
| `create_subscription` | `String` | `'/ur7e_reconstruction/stats'` | 28 |
| `create_publisher` | `Float64MultiArray` | `'/forward_position_controller/commands'` | 29 |
| `create_client` | `Trigger` | `'/ur7e_reconstruction/save_mesh'` | 30 |
| `create_client` | `SetBool` | `'/ur7e_reconstruction/set_integration_enabled'` | 31 |

### `ur7e_reconstruction/surface.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/surface.py)

SurfaceMapper 只融合 TSDF/颜色；提取、裁剪和重编号 Mesh，生成彩色点云/标记，并原子导出 PLY。

源码说明：TSDF surface extraction, visualization and portable mesh export.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `Surface` | 19 | 相关状态与操作的封装；方法见下一行。 |
| `crop_surface(vertices, triangles, minimum, maximum, colors=None)` | 25 | Keep complete triangles in the requested base-frame ROI and reindex vertices. |
| 类 `ColorBackend` | 45 | Configure a fresh native mapper, retaining the original backend's public API. |
| 方法 | — | `_initialize`（L47） |
| 类 `SurfaceMapper` | 65 | Fuse RGB-D and extract the measured colored surface without any ESDF updates. |
| 方法 | — | `__init__`（L67）、`integrate`（L78）、`extract`（L83） |
| `surface_cloud(surface, frame_id, stamp_ns)` | 89 | Fused mesh vertices, not an instantaneous depth-camera cloud. |
| `surface_markers(surface, frame_id, stamp_ns, chunk_triangles=4000)` | 105 | A complete snapshot; DELETEALL removes old chunks on reset or shrinking meshes. |
| `save_ply(surface, path)` | 130 | Atomic binary PLY export, preserving all displayed vertices and triangles. |

直接项目/规划依赖：`realtime_nvblox.nvblox_backend`, `realtime_nvblox.ros_publishers`, `ur7e_nvblox.depth`。

### `ur7e_reconstruction/verify.py`

[查看源码](../../src/ur7e_reconstruction/ur7e_reconstruction/verify.py)

对已运行节点进行只读集成核查，检查话题、TF、输出等。

源码说明：Read-only validation of a running reconstruction node.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 23 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

直接项目/规划依赖：`ur7e_nvblox.depth`。

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_subscription` | `MarkerArray` | `'/ur7e_reconstruction/mesh'` | 60 |
| `create_subscription` | `PointCloud2` | `'/ur7e_reconstruction/surface_points'` | 61 |
| `create_subscription` | `PoseStamped` | `'/ur7e_reconstruction/camera_pose'` | 62 |
| `create_subscription` | `String` | `'/ur7e_reconstruction/stats'` | 63 |

启动/命令行参数声明：

```python
parser.add_argument('--timeout', type=float, default=90)
parser.add_argument('--samples', type=int, default=5)
parser.add_argument('--output', type=Path)
```

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [config/gazebo.yaml](../../src/ur7e_reconstruction/config/gazebo.yaml) | 配置分区/键：`/**` |
| [config/high_initial_positions.yaml](../../src/ur7e_reconstruction/config/high_initial_positions.yaml) | 配置分区/键：`shoulder_pan_joint` |
| [config/scan_views.yaml](../../src/ur7e_reconstruction/config/scan_views.yaml) | 配置分区/键：`joint_names` |
| [package.xml](../../src/ur7e_reconstruction/package.xml) | ROS 包名、依赖及构建类型声明。 |
| [rviz/reconstruction.rviz](../../src/ur7e_reconstruction/rviz/reconstruction.rviz) | RViz 显示、固定坐标系及订阅话题。 |
| [setup.cfg](../../src/ur7e_reconstruction/setup.cfg) | ament_python 脚本安装路径。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_reconstruction/README.md)
- [TEST_RESULTS.md](../../src/ur7e_reconstruction/TEST_RESULTS.md)
