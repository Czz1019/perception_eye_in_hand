# 新工程目录与模块职责说明

2026-11-04重构提示：本文保留此前结构/运行记录。当前默认已改为官方NvbloxNode组件，入口、配置及显示选择见 [重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md) 和 [最新操作说明](OPERATIONS.md)。此前inside-only默认、20字节点云及旧频率不能当作新版接口或结果；本轮未运行测试。

分析根目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。表中的相对路径均以该目录为根；目录和文件名用于对应实际源码，不表示它们都是默认启动的进程。

## 1. 目录总览

```text
nvblox_in_hand/
├── run.sh / env.sh / build.sh          启动、环境、构建
├── rviz.sh / check_sim.sh              显示与机器人状态检查
├── orbit_scan.sh / test.sh             可选扫描和离线测试
├── src/
│   ├── nvblox_in_hand/                 新的感知编排与薄适配功能包
│   │   ├── CMakeLists.txt / package.xml
│   │   ├── src/mapper_node.cpp
│   │   ├── include/nvblox_in_hand/esdf_view_policy.hpp
│   │   ├── scripts/robot_pose.py
│   │   ├── launch/bringup.launch.py
│   │   ├── launch/perception/mapping.launch.py
│   │   ├── launch/sensors/d435.launch.py
│   │   ├── config/nvblox/              算法和运行配置
│   │   ├── config/sensors/             D435 配置
│   │   ├── config/robot/               DDS 配置
│   │   └── rviz/in_hand.rviz
│   └── ur7e_d435_gazebo/               原机械臂/场景副本
│       ├── launch/ config/ urdf/ meshes/
│       ├── scripts/ src/ test/
│       ├── third_party/ licenses/ docs/
│       └── CMakeLists.txt / package.xml
├── upstream/                          官方 ROS/CUDA 源码快照
│   ├── nvblox_ros/nvblox_core/
│   ├── nvblox_ros_common/
│   ├── nvblox_msgs/
│   ├── nvblox_ros_python_utils/
│   └── nvblox_rviz_plugin/
├── tools/                             状态审计与显示回归
├── tests/                             新适配模块的离线测试
├── docs/                              架构、分析、操作和验证
├── provenance/                        来源清单和复制差异
├── artifacts/                         测试日志和运行报告
├── build*/ install*/ log/             生成产物
└── .test_runtime/                      测试用 Xvfb 解包，不参与仿真核心
```

## 2. 根目录各文件

| 文件 | 作用 | 需要注意 |
| --- | --- | --- |
| run.sh | 加载环境，选择安装模式，启动顶层 launch | 默认不启动扫描；自身不计算地图 |
| env.sh | 清理旧 overlay，加载 Humble/官方依赖/本工程，补 GXF 库路径及 RViz 兼容选项 | 用 source 才会影响当前监控终端；保留用户显式环境设置 |
| build.sh | 默认构建两个 src 包；--from-source 额外构建 upstream 包 | 两类构建分别写 build/install 与 build_source/install_source |
| rviz.sh | 单独打开本工程 RViz 配置 | 不启动 Gazebo/mapper；可指定实机时间模式 |
| check_sim.sh | 加载环境后运行 tools/check_simulation.py | 无运动，只核对反馈和 TF |
| orbit_scan.sh | 执行场景副本中的 orbit_scan.py | --check-only/--plan-only 与真正执行有不同含义 |
| test.sh | 编译主机侧策略测试，执行新 Python 测试及指定 FK/FCL 测试 | 不启动仿真；没有运行副本中所有历史测试 |
| COLCON_IGNORE | 防止父工程递归构建时发现同名场景包 | 本目录 build.sh 用显式 --base-paths src，仍可构建 |
| .gitignore | 忽略构建、安装、运行及本地测试依赖产物 | 不等于这些产物无调试价值 |
| LICENSE / THIRD_PARTY.md | 新代码和复制资产的许可证说明 | 不改变原资产版权和来源 |
| README.md | 新工程入口和文档导航 | 不代替具体模块分析 |

## 3. 感知功能包逐模块说明

功能包根目录：[src/nvblox_in_hand](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand)。

| 文件 | 输入/依赖 | 输出/职责 |
| --- | --- | --- |
| CMakeLists.txt | ament、nvblox_ros、CUDA、ROS 消息 | 编译 mapper_node，安装 Python、头文件、launch/config/RViz；不在默认模式重建核心 |
| package.xml | 构建/运行依赖清单 | 声明包接口和依赖，不会自动启动依赖里的节点 |
| src/mapper_node.cpp | 官方 NvbloxNode 及原生地图层 | InHandMapper、融合计数、可选 ESDF 原生体素视图 |
| include/.../esdf_view_policy.hpp | observed、内外标记、平方体素距离、分辨率和带宽 | 返回有效带内有符号米制距离，或无值；不执行融合 |
| scripts/robot_pose.py | CameraInfo 时间戳、TF | 相机 Pose、有限路径和 TF 健康信息 |
| launch/bringup.launch.py | 用户开关 | 场景、相机、感知、路径与显示的进程编排 |
| launch/perception/mapping.launch.py | 基础配置、自定义 YAML、launch 覆盖 | 官方节点参数、图像映射、后端选择和 mapper 退出联动 |
| launch/sensors/d435.launch.py | 物理 D435 与驱动包 | 可选 realsense2_camera_node；不是 Gazebo 传感器 |
| rviz/in_hand.rviz | Mesh、层、TF、描述、图像、路径 | 官方和标准显示配置、全景/俯视预设及鼠标工具 |

## 4. 配置文件的分工与优先级

| 配置 | 提供的信息 | 是否直接默认加载 |
| --- | --- | --- |
| config/nvblox/base.yaml | 2 cm、静态单相机、融合/ESDF/层频率、原生参数 | 是 |
| simulation.yaml | use_sim_time=true | 仿真分支加载 |
| realsense.yaml | use_sim_time=false | 非仿真分支加载 |
| fine.yaml | 1 cm 体素覆盖 | 只有指定 mapper_config 才加载 |
| comparison_3cm.yaml | 3 cm 体素覆盖 | 同上，用于公平比较的一部分条件 |
| esdf_2d.yaml / esdf_3d.yaml | ESDF 模式候选覆盖 | 不会自动随文件名切换；公开 esdf_mode 参数会最终覆盖它 |
| view.yaml | 附加 ESDF 视图参数候选 | 不默认单独加载；相关公开 launch 参数有更高优先级 |
| config/sensors/d435.yaml | 彩色/深度、分辨率、同步、关闭红外/IMU和驱动 TF | 只有 launch_camera=true 时由物理相机 launch 使用 |
| config/robot/cyclonedds_local.xml | DDS participant 发现设置 | env.sh 默认引用，显式 CYCLONEDDS_URI 优先 |

同一个字段出现在配置文件和公开 launch 参数时，应看最终覆盖规则，不能仅凭 YAML 文件名判断运行模式。当前直接使用 `esdf_mode:=2d`、`voxel_size:=0.01` 等参数最清楚。算法中未公开的细节参数可通过 mapper_config 设置。

## 5. 机器人与场景副本

根目录：[src/ur7e_d435_gazebo](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/ur7e_d435_gazebo)。

| 子模块/文件 | 功能 | 与感知的关系 |
| --- | --- | --- |
| launch/scene.launch.py | 加载描述/手眼、检查安装、启动场景和控制器 | 产生真实仿真输入，不直接调用 mapper |
| urdf/ur7e_base.urdf.xacro | 基础机器人、关节、visual/collision、ros2_control | 决定 FK、显示与仿真结构 |
| urdf/ur7e_d435.urdf.xacro | 组合机器人与相机、Gazebo 深度/彩色传感器 | 默认原始 RGB-D 输入来源 |
| urdf/d435_handeye_mount.urdf.xacro | 实测手眼与相机固定光学链 | 将 tool0 与两类光学 frame 正确联系 |
| 其他 mount/realsense xacro | 保留的安装模型和相机基础宏 | 并非同时生成多套相机或同时启动 |
| meshes/ | UR 和相机几何资产 | 用于显示/碰撞，不是从相机重建的地图 |
| config/controllers.yaml | 控制器类型、频率和关节顺序 | joint_states 和扫描命令消费者 |
| config/initial_positions.yaml | 仿真初始六关节 | 不是运行中自动回零指令 |
| config/ur7e_d435_handeye_20260907.yaml | 实测标定和来源说明 | 默认手眼；保留并不等于已证明实机安装无误 |
| config/collision_world_gazebo_tall.yml | 既有障碍物及几何 | 生成固定场景、扫描核验和碰撞检查 |
| config/gazebo_clock.yaml | Gazebo ROS 时钟发布配置 | 不能与物理计算步长或相机帧率混为一谈 |
| scripts/spawn_scene.py / gazebo_obstacle_utils.py | 创建障碍物和辅助转换 | 不向 TSDF/ESDF 注入真值 |
| scripts/mount_geometry.py | URDF 变换、给定关节下的 link FK、安装几何核查 | 状态审计和几何测试复用 |
| src/check_mount_meshes.cpp | 安装位置相关几何检查 | 场景启动前检查器 |
| scripts/orbit_geometry.py | FK、目标范围、IK/路径候选、碰撞封装 | 扫描设计层 |
| src/scan_collision_checker.cpp | 基于实际 mesh/基本体的 FCL 检查 | 扫描几何约束；不是地图查询器 |
| scripts/orbit_scan.py | 依赖检查、场景核对、规划/执行和采集协调 | 唯一可选运动链路之一，默认感知不会自动运行它 |
| scripts 的标定/导入/校正工具 | 保留原场景的辅助维护工具 | 不属于默认重建回调 |
| third_party/、licenses/、docs/ | 原插件源码、授权及历史说明 | 保留来源，不能将其历史状态当作新工程最新验收 |

复制包沿用原包名来保持 xacro 与资源引用兼容。env.sh 应让本目录安装结果在前；两个工作区的同名场景不应同时启动。

## 6. 官方源码快照按模块阅读

根目录：[upstream](/home/czz/perception_eye_in_hand/nvblox_in_hand/upstream)。这里保留算法层次；默认执行仍来自已安装库。

| 模块 | 主要工作 |
| --- | --- |
| nvblox_ros/src/lib/nvblox_node.cpp | ROS 节点生命周期、队列、tick、积分调用、输出与服务 |
| nvblox_ros/src/lib/node_params.cpp | 声明和读取 ROS 参数 |
| nvblox_ros/src/lib/mapper_initialization.cpp | 将参数组织到原生 Mapper/MultiMapper |
| nvblox_ros/src/lib/transformer.cpp | 按传感器 frame/time 查询 TF 或其他支持的位姿源 |
| nvblox_ros/src/lib/camera_cache.cpp | 原生相机相关缓存 |
| nvblox_ros/src/lib/conversions/ | Image、CameraInfo、Mesh、占据、切片、距离/梯度的消息转换 |
| nvblox_ros/src/lib/layer_publishing.cpp | 根据订阅者选择层，原生序列化和增量发布 |
| nvblox_ros 的 fuser/rosbag 模块 | 离线数据处理能力，默认在线启动不运行 fuser |
| nvblox_core/nvblox/src/mapper/ | 图层管理、深度/颜色融合和 Mesh/ESDF 更新入口 |
| nvblox_core/nvblox/src/integrators/ | TSDF、ESDF、视场、衰减与切片等 GPU 计算 |
| nvblox_core/nvblox/src/mesh/ | 三角网格提取和颜色关联 |
| nvblox_core/nvblox/include/nvblox/map/ | VoxelBlock、Layer、LayerCake、访问和内存管理接口 |
| nvblox_core/nvblox/src/map/ | 地图结构实现和更新块跟踪 |
| nvblox_core/nvblox/include/nvblox/serialization/ | 块序列化/流送接口，附加 ESDF 显示也复用此处 |
| nvblox_ros_common | CUDA/ROS 公共辅助 |
| nvblox_msgs | 原生 Mesh、VoxelBlockLayer、切片和 FilePath/ESDF 服务定义 |
| nvblox_rviz_plugin | 原生增量 Mesh/体素的 RViz display/visual |
| nvblox_ros_python_utils | 官方 Python/launch 工具，保留来源；本工程自有 launch 不依赖其全部示例逻辑 |

任务识别模型和示例工作流没有纳入运行链路；共享库中的通用动态/人物相关类型仍保留，避免为了裁剪而破坏官方框架和 ABI。

## 7. tools 和 tests

| 文件 | 验证内容 | 不证明什么 |
| --- | --- | --- |
| tools/check_simulation.py | 关节完整性/新鲜度、时钟、描述、发布者、同时间 FK/TF 残差 | 不证明物理标定正确或 Gazebo link 真值完全一致 |
| tools/test_rviz_material.py | 独立 ROS 域和虚拟显示内触发 Mesh 插件 | 不生成真实重建画面，不评价感知精度 |
| tests/esdf_view_policy_test.cpp | unknown/非法值剔除、距离单位和符号 | 不测试整套 CUDA ESDF 求解器 |
| tests/test_launch_contract.py | 输入/参数/开关与官方后端选择 | 不证明启动后所有硬件节点正常 |
| tests/test_robot_state_audit.py | 关节名称重排、缺值与位姿残差判断 | 不代替实际运行状态审计 |
| 场景 test/test_orbit_geometry.py | FK、目标光轴、FCL、场景拒绝及扫描依赖诊断 | 不代替轨迹执行与覆盖率验收 |

## 8. 生成目录与来源记录

- build、build_source：CMake/colcon 中间文件；不要作为主要代码阅读入口。
- install、install_source：实际启动使用的安装结果；源码修改后需重新构建才能同步配置和 Python 文件。
- log：构建/ROS 运行日志；不是业务算法。
- artifacts：离线计划、检查报告、构建证据和渲染回归。计划 JSON 不是运动完成证据。
- provenance/source_manifest.json：实际复制来源路径和哈希。
- provenance/local_changes.json：记录复制副本的局部修改，不能等同于所有后续新文件的 Git diff。
- .test_runtime：Xvfb 测试工具解包；运行正常仿真不要求它。

新增文件和实际调用链的深入解释见 [代码分析](CODE_ANALYSIS.md)。新工程与父工程如何接续，见 [整体工程关系](WHOLE_PROJECT_RELATIONSHIP.md)。
