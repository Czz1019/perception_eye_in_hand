# ur7e_d435_gazebo：UR7e + D435 仿真及标定资源

[返回项目文档索引](../README.md)

## 职责和边界

负责提供仿真传感器和控制器，不负责 TSDF 融合或 SAGE 算法。包中 third_party 插件保留独立版权信息。

## 输入、处理与输出

**数据链路：** URDF/Xacro、场景配置 → Gazebo/ros2_control/相机插件 → 图像、内参、TF 和关节反馈。

scene.launch.py 展开模型并启动服务；本地编译的相机插件将 Gazebo 图像转换成 ROS 消息；spawn_scene 创建障碍；几何/标定脚本独立核查。

## 目录结构

```text
ur7e_d435_gazebo/
├── config/
│   ├── handeye_20260907/
│   ├── collision_world_gazebo_tall.yml
│   ├── controllers.yaml
│   ├── d435_side_mount_validation.json
│   ├── initial_positions.yaml
│   ├── ur7e_d435_handeye_20260907.yaml
│   ├── ur7e_d435_side_sim_mount.yaml
│   └── ur7e_d435_sim_clearance.yaml
├── docs/
│   ├── HANDEYE_20260907_IMPORT.md
│   ├── HANDEYE_DIAGNOSIS.md
│   ├── handeye_20260907_geometry.json
│   ├── handeye_20260907_runtime.json
│   ├── measured_mount_geometry.json
│   ├── measured_mount_runtime.json
│   └── simulation_mount_geometry.json
├── launch/
│   ├── calibration_marker.launch.py
│   └── scene.launch.py
├── licenses/
│   ├── RealSense-Apache-2.0.txt
│   └── UR-Description-LICENSE
├── meshes/
├── scripts/
│   ├── evaluate_handeye.py
│   ├── gazebo_obstacle_utils.py
│   ├── import_handeye_20260907.py
│   ├── mount_geometry.py
│   ├── rectify_calibration_image.py
│   ├── spawn_scene.py
│   └── verify_scene.py
├── src/
│   └── check_mount_meshes.cpp
├── test/
│   └── test_handeye.py
├── third_party/
├── urdf/
│   ├── realsense/
│   ├── d435_handeye_mount.urdf.xacro
│   ├── d435_side_mount.urdf.xacro
│   ├── ur7e_base.urdf.xacro
│   └── ur7e_d435.urdf.xacro
├── CMakeLists.txt
├── README.md
├── THIRD_PARTY.md
└── package.xml
```

## 源文件逐项说明

以下覆盖运行源码、launch、脚本及测试源码。`validation/`、`report/` 内嵌的历史数据/报告生成脚本在目录说明中单列，不作为生产入口。

### `launch/calibration_marker.launch.py`

[查看源码](../../src/ur7e_d435_gazebo/launch/calibration_marker.launch.py)

启动矫正彩色图和 ArUco 标记检测，用于标定；机器人/相机驱动由外部启动。

源码说明：Detect the calibration marker from rectified D435 color images.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `generate_launch_description()` | 12 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('marker_size', description='Measured black-square side length, metres (exclude white margin)')
DeclareLaunchArgument('marker_id', default_value='582')
```

### `launch/scene.launch.py`

[查看源码](../../src/ur7e_d435_gazebo/launch/scene.launch.py)

展开 UR7e 和 D435 外侧安装模型，检查 Gazebo master，启动仿真、机器人、控制器和场景障碍。

源码说明：UR7e + outer-side D435 + tall walls, without external workspace paths.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `_remove_comments(node)` | 20 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_check_gazebo_master_available(uri)` | 29 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `_setup(context)` | 45 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `generate_launch_description()` | 100 | 声明启动参数并返回 ROS LaunchDescription，按配置组装进程与可选组件。 |

启动/命令行参数声明：

```python
DeclareLaunchArgument('gazebo_gui', default_value='true', description='Open the Gazebo GUI')
DeclareLaunchArgument('calibration_file', default_value=str(Path(get_package_share_directory('ur7e_d435_gazebo')) / 'config/ur7e_d435_handeye_20260907.yaml'), description='Eye-in-hand YAML; default preserves the measured transform')
```

### `scripts/evaluate_handeye.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/evaluate_handeye.py)

读取保存的 easy_handeye2 采样，以固定 base→marker 约束计算手眼残差。

源码说明：Evaluate saved easy_handeye2 samples using the fixed base-to-marker constraint.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `matrix(transform)` | 12 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `residuals(robot, tracking, handeye)` | 24 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `evaluate(samples, calibration)` | 37 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 68 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('samples', type=Path, help='Saved easy_handeye2 .samples file')
parser.add_argument('calibration', type=Path, help='Saved eye-in-hand .calib or YAML')
parser.add_argument('--output', type=Path)
```

### `scripts/gazebo_obstacle_utils.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/gazebo_obstacle_utils.py)

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

### `scripts/import_handeye_20260907.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/import_handeye_20260907.py)

交叉检查毫米 YAML 与米制 JSON，再转换为 ROS 手眼标定 YAML。

源码说明：Cross-check the supplied mm YAML and metre JSON, then export ROS hand-eye YAML.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `convert(legacy, report)` | 12 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `main()` | 60 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

启动/命令行参数声明：

```python
parser.add_argument('legacy_yaml', type=Path)
parser.add_argument('calibration_json', type=Path)
parser.add_argument('output_yaml', type=Path)
```

### `scripts/mount_geometry.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/mount_geometry.py)

展开 URDF link/visual-origin 变换，检查相机及机械臂几何安装关系。

源码说明：Check the generated URDF geometry, including mesh and visual-origin transforms.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `origin_transform(origin)` | 10 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `link_transforms(robot, positions)` | 26 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| `check_mount_geometry(description, positions, executable)` | 57 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |

### `scripts/rectify_calibration_image.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/rectify_calibration_image.py)

用对应 CameraInfo 建立矫正映射，向 aruco_ros 提供真实矫正后的图像和匹配内参。

源码说明：Provide truly rectified color images and matching intrinsics for aruco_ros.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `rectification_maps(info)` | 15 | 参见上文文件职责和函数体；此处保留实际签名便于定位。 |
| 类 `RectifyCalibrationImage` | 34 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `__init__`（L35）、`receive_marker`（L47）、`receive_info`（L50）、`receive_image`（L57） |
| `main()` | 74 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_publisher` | `Image` | `'image_rect'` | 39 |
| `create_publisher` | `CameraInfo` | `'rectified_camera_info'` | 40 |
| `create_subscription` | `CameraInfo` | `'camera_info'` | 41 |
| `create_subscription` | `Image` | `'image_raw'` | 42 |
| `create_subscription` | `PoseStamped` | `'marker_pose'` | 45 |

### `scripts/spawn_scene.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/spawn_scene.py)

读取包内场景配置，经 Gazebo 服务创建高墙与球体等仿真障碍。

源码说明：Spawn the three tall-scene obstacles from this package's installed config.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 12 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

### `scripts/verify_scene.py`

[查看源码](../../src/ur7e_d435_gazebo/scripts/verify_scene.py)

检查实际相机话题、标定 TF、控制器和 Gazebo 模型是否符合场景配置。

源码说明：Check actual camera messages, calibration TF, controllers and Gazebo models.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| `main()` | 22 | 解析命令行或 ROS 参数，初始化该文件对应入口并负责退出清理；实际模式见文件职责。 |

ROS 接口声明（表达式为源码原值；`~/` 需按节点名展开，配置引用需按选用 YAML 解析）：

| 操作 | 消息/服务类型 | 名称或配置表达式 | 行 |
| --- | --- | --- | --- |
| `create_client` | `ListControllers` | `'/controller_manager/list_controllers'` | 53 |
| `create_client` | `GetParameters` | `'/robot_state_publisher/get_parameters'` | 54 |
| `create_subscription` | `TFMessage` | `'/tf_static'` | 51 |
| `create_subscription` | `msg_type` | `topic` | 47 |

启动/命令行参数声明：

```python
parser.add_argument('--output', type=Path)
parser.add_argument('--timeout', type=float, default=60.0)
parser.add_argument('--calibration', type=Path, help='Same YAML passed to scene.launch.py')
```

### `src/check_mount_meshes.cpp`

[查看源码](../../src/ur7e_d435_gazebo/src/check_mount_meshes.cpp)

从标准输入读入相机/机械臂变换后的 mesh/box，构建 FCL 碰撞体，逐对输出 collision 和 distance_m JSON；检测到碰撞本身仍返回 0，调用方须读取 JSON。

### `test/test_handeye.py`

[查看源码](../../src/ur7e_d435_gazebo/test/test_handeye.py)

验证或示例入口；具体覆盖项见下面提取的测试/工具函数。运行前需检查是否依赖 ROS、GPU、Gazebo 或会修改测试场景。

源码说明：Run after sourcing ROS and the workspace: python3 -m unittest discover -s .../test.

| 符号 | 源码行 | 作用/组成 |
| --- | --- | --- |
| 类 `GazeboStartupTest` | 25 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `test_occupied_master_is_rejected_before_launching_nodes`（L26） |
| 类 `HandeyeGeometryTest` | 40 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `setUpClass`（L42）、`description`（L47）、`test_measured_transform_and_camera_internal_geometry`（L51）、`test_measured_overlap_is_reported`（L66）、`test_explicit_simulation_mount_has_clearance`（L71）、`test_default_charuco_mount_matches_source_and_has_clearance`（L76） |
| 类 `CalibrationImportTest` | 87 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `setUp`（L88）、`test_units_and_quaternion_preserve_matrix`（L93）、`test_wrong_units_and_mismatched_files_are_rejected`（L99）、`test_reference_intrinsics_are_cross_checked`（L109） |
| 类 `CalibrationInputTest` | 116 | 相关状态与操作的封装；方法见下一行。 |
| 方法 | — | `test_rectified_intrinsics_and_distortion`（L117）、`test_fixed_marker_residual_detects_bad_calibration`（L135） |

### `third_party/realsense_gazebo_plugin/include/realsense_gazebo_plugin/RealSensePlugin.hpp`

[查看源码](../../src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/include/realsense_gazebo_plugin/RealSensePlugin.hpp)

声明相机参数、Gazebo 相机资源、Load 和帧回调接口。

### `third_party/realsense_gazebo_plugin/include/realsense_gazebo_plugin/gazebo_ros_realsense.hpp`

[查看源码](../../src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/include/realsense_gazebo_plugin/gazebo_ros_realsense.hpp)

声明派生 ROS 插件、图像/点云发布器及消息填充接口。

### `third_party/realsense_gazebo_plugin/src/RealSensePlugin.cpp`

[查看源码](../../src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/RealSensePlugin.cpp)

Gazebo 相机传感器插件基类：读取 SDF，连接颜色/红外/深度传感器及新帧回调。

### `third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp`

[查看源码](../../src/ur7e_d435_gazebo/third_party/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp)

ROS 相机插件：注册 ModelPlugin，建立 Image+CameraInfo 发布器，将仿真颜色/红外/深度及可选点云转换为 ROS 消息。

## 配置、资源与构建

| 文件 | 作用/顶层配置分区 |
| --- | --- |
| [CMakeLists.txt](../../src/ur7e_d435_gazebo/CMakeLists.txt) | 编译/接口生成、Python 和资源安装规则。 |
| [config/collision_world_gazebo_tall.yml](../../src/ur7e_d435_gazebo/config/collision_world_gazebo_tall.yml) | 配置分区/键： |
| [config/controllers.yaml](../../src/ur7e_d435_gazebo/config/controllers.yaml) | 配置分区/键：`controller_manager` |
| [config/handeye_20260907/legacy_calibration.yaml](../../src/ur7e_d435_gazebo/config/handeye_20260907/legacy_calibration.yaml) | 配置分区/键： |
| [config/initial_positions.yaml](../../src/ur7e_d435_gazebo/config/initial_positions.yaml) | 配置分区/键： |
| [config/ur7e_d435_handeye_20260907.yaml](../../src/ur7e_d435_gazebo/config/ur7e_d435_handeye_20260907.yaml) | 配置分区/键： |
| [config/ur7e_d435_side_sim_mount.yaml](../../src/ur7e_d435_gazebo/config/ur7e_d435_side_sim_mount.yaml) | 配置分区/键： |
| [config/ur7e_d435_sim_clearance.yaml](../../src/ur7e_d435_gazebo/config/ur7e_d435_sim_clearance.yaml) | 配置分区/键： |
| [package.xml](../../src/ur7e_d435_gazebo/package.xml) | ROS 包名、依赖及构建类型声明。 |
| [urdf/d435_handeye_mount.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/d435_handeye_mount.urdf.xacro) | 机器人/相机参数化描述。 |
| [urdf/d435_side_mount.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/d435_side_mount.urdf.xacro) | 机器人/相机参数化描述。 |
| [urdf/realsense/_d435.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/realsense/_d435.urdf.xacro) | 机器人/相机参数化描述。 |
| [urdf/realsense/_materials.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/realsense/_materials.urdf.xacro) | 机器人/相机参数化描述。 |
| [urdf/realsense/_usb_plug.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/realsense/_usb_plug.urdf.xacro) | 机器人/相机参数化描述。 |
| [urdf/ur7e_base.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/ur7e_base.urdf.xacro) | 机器人/相机参数化描述。 |
| [urdf/ur7e_d435.urdf.xacro](../../src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro) | 机器人/相机参数化描述。 |

## 测试与历史资料

本页的测试条目是源码覆盖目录，不表示已在本次环境运行。集成、故障注入、场景脚本可能启动进程或修改仿真对象；本次仅做静态文档分析。

- [README.md](../../src/ur7e_d435_gazebo/README.md)
- [THIRD_PARTY.md](../../src/ur7e_d435_gazebo/THIRD_PARTY.md)
- `docs/`：标定与装配等专题说明。
- `licenses/`：第三方许可证。
- `third_party/`：随包构建的第三方源码。
