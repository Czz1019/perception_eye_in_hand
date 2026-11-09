# ESDF sparse-v2 修改说明、官方代码对照与 Topic 操作手册

> **当前版本 0.4.0 / 接口 v4：** 主处理恢复为 workspaces 官方 ROS tick，Mesh 显示拆为模块，统计改用非阻塞 CUDA 事件；默认 ESDF 5 Hz，另有 30 Hz 目标配置。3 cm 体素与原输入/范围保持不变。修改和完整命令见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。已完成构建与离线协议检查，未启动新版仿真，未实测频率/延迟/几何质量。

> 后续构建修复更新：用户首次构建遇到的 Thrust 链接问题已修复，现已完成生产/可选工具的构建安装验证；仍未执行测试或运行新版仿真。当前操作以 [构建修复与仿真说明](/home/czz/perception_eye_in_hand/docs/ESDF_V2_BUILD_FIX_AND_SIMULATION_GUIDE.md) 为准。下文“本轮未构建”描述的是初次 sparse-v2 源码修改阶段。

日期：2026-10-27；本仓库接口版本 0.2.0 / `interface_revision=2`。

**本轮按要求只修改 `/home/czz/perception_eye_in_hand` 的代码与文档，未构建、未运行程序、未启动仿真、未执行测试。以下描述为源码实现和待验证预期，不是新的性能实测。旧 install_native 仍是此前二进制，必须在后续允许时重新构建。**

## 1. 用户指出的问题成立：上一版实时导出仍是补齐 ROI

上一版虽然使用了官方原生层读取模板，但实时发布流程仍然是：

```text
指定 ROI → 为 ROI 生成完整规则网格 → 查询每个位置的 ESDF
       → 未知填 1e6 → /esdf_3d 发布包括未知在内的全部位置
```

因此在没有观测信息的区域，也会存在规则排列的点。先前“只看 esdf_observed”可以隐藏显示问题，却没有纠正 `/esdf_3d` 的语义，也没有消除实时导出中的 ROI 体积扫描。这一部分需要实质性改代码。

应当区分两件事：

- nvblox 的地图本来就是分块体素结构，实际地图中的有效点仍位于规则体素中心。看到局部规则排列并不等于错误。
- **没有观测的位置也被补成点、点数由整个空盒子决定**，则是应用侧稠密导出/可视化方式造成的现象。它不应成为实时地图默认输出。

本次移除实时点云发布中的 dense AABB 转换，改为从实际分配的 ESDF 块读取有效体素；ROI 仅用于裁剪真实地图数据。

## 2. 对 workspaces 官方实现的逐部分参考

参考根目录：`/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox`。本轮只读取参考实现，没有修改该目录。

| 官方源码 | 核对到的逻辑 | 本项目如何采用 |
| --- | --- | --- |
| [layer_publishing.cpp](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:673) | 按订阅选择要序列化的地图层，Mesh 有消费者时才更新；按块流式发布 | 保留官方 `/mesh`；标准 Mesh 显示有消费者时才订阅它，关闭后释放内部订阅 |
| [EsdfVoxelFilter 定义](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:187) | 定义有效显示条件 `voxel.is_inside && voxel.observed` | `/esdf_3d` 明确采用这个筛选语义，排除未知和普通自由空间 |
| [publishMesh](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/layer_publishing.cpp:545) | 原生 Mesh 增量块、空块删除、clear 与新订阅者处理 | Marker 桥接按块 namespace/id 更新和删除，并补充显式新订阅者快照 |
| [mesh_conversions.cpp](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/conversions/mesh_conversions.cpp) | Mesh 顶点已在全局坐标，triangles 是块内顶点索引 | 桥接展开三角形；不再加一次块原点；快照复用官方 Marker 转换函数 |
| [esdf_and_gradients_conversions.cu](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/conversions/esdf_and_gradients_conversions.cu) | 显式查询 AABB 时生成规则网格、保留未知值，供距离查询服务使用 | 仍保留为按需服务，不再把它放进每帧可视化主路径 |
| [原生层 API](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/nvblox_core/nvblox/include/nvblox/map/layer.h) | `getAllBlockIndices()`、`getBlockAtIndex()` 访问实际地图块 | 新 CUDA exporter 遍历已有块、块内体素，直接读取 observed/is_inside/距离 |
| [官方 ESDF 更新输出分支](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_ros/src/lib/nvblox_node.cpp:783) | `static_esdf_pointcloud` 等切片输出受 `esdf_mode==2D` 限制 | 不把这些话题误写成当前 3D 模式一定发布的数据 |
| [官方视觉 SLAM Path](/home/czz/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:884) | `/visual_slam/tracking/slam_path` 来自视觉 SLAM 的位姿序列 | 本项目已有机械臂 pose，发布相同 ROS Path 类型，但明确标为 robot TF 来源 |

**参考版本中 EsdfVoxelFilter 只有定义，没有直接接线为独立的 3D 占用点云发布器。** 它的实际可视化主路径包含 Mesh、TSDF 等地图层以及二维模式的 ESDF 切片。不能仅凭这个过滤器的定义，就声称官方默认正在发布与本项目同名的 `/esdf_3d`。本次采用其筛选定义创建项目自己的三维可视化接口，同时复用实际的官方 Mesh/地图层路径。

官方照片中的表面 Mesh 与一个填满空间的三维距离点云是不同的显示对象。要对照“表面重建效果”，应查看 Mesh；要对照距离数值，应查看观测距离带或调用有界查询服务。

## 3. 新实时数据流

```text
深度图 + 匹配内参 + 图像时刻机械臂 TF
    → 官方 TSDF 融合 / 3D ESDF 更新
    → 实际 ESDF 块列表
    → 按块与 ROI 相交筛选
    → CUDA 每批最多 512 个已有块
    → 先检查 observed 和距离合法性，再计算体素坐标
    ├── observed && is_inside             → esdf_3d
    ├── observed && |distance| <= band    → esdf_observed
    └── observed 且距离有限               → esdf_distance（诊断）

官方 TSDF/Mesh 层 → 官方增量 mesh
                  → 标准 TRIANGLE_LIST Marker 桥接 → mesh_markers
图像时刻机械臂 TF → camera_pose → 有界 camera_path / slam_path 别名
```

### 3.1 与旧实现的实质区别

- 不为 ROI 预生成坐标网格，不使用 dense layer-to-grid 模板作为实时 exporter。
- 未分配区域没有 block，不会为它补点；已分配 block 内 `observed=false` 的单元也不输出。
- 坐标来自真实 block index 和块内 voxel index，而不是从一份预先建立的全空间查询点表填距离。
- GPU 临时空间按最多 512 个块分批复用；只为有订阅的输出申请相应压缩缓冲。
- 点云读回只包含符合条件的体素，不读回未知占位点，也不生成灰色未知点。
- 空地图/空 ROI 输出空点云；不会伪造一盒“没有信息”的点，也不会复用上一次结果。
- 仍按新地图版本与消费者需求导出，不靠重复发布旧地图提高 Hz。

成本约为：枚举已有块元数据 `O(B_total)`、遍历 ROI 内块槽位 `O(512 × B_ROI)`、读回有效输出 `O(N_output)`。不是“完全不访问未知”：实际分配块内仍需检查 observed 标志；也不是只更新变化块的增量 ESDF 点云。点云是当前 ROI 内有效体素的快照，顺序由 GPU 压缩决定，没有固定数组下标契约。

GPU scratch 分批有界不等于整个程序内存恒定。实际地图、CPU 输出和路径仍有各自容量；长期大范围扫描需要另测显存/内存增长。本次没有以调整体素精度或缩小输入换取上述结构改变。

## 4. 三类 ESDF 话题的语义

| Topic | 筛选 | 是否包含自由空间 | 是否包含未知点 |
| --- | --- | --- | --- |
| `/ur7e_nvblox/esdf_3d` | observed && is_inside | 普通外部自由空间不输出 | 不包含 |
| `/ur7e_nvblox/esdf_observed` | observed && abs(distance) ≤ 0.15 m | 包含靠近表面的已观测自由空间 | 不包含 |
| `/ur7e_nvblox/esdf_distance` | observed && finite(distance) | 包含；仅用于诊断完整已观测距离场 | 不包含 |

`is_inside` 是底层 ESDF 体素的内部状态，不是本项目重新做的一套概率占用建图，也不保证等同于封闭物体内部所有空间。传感器没有观测到的背面仍然未知。Mesh 则来自 TSDF 的表面提取，而不是把 ESDF 点连成三角形。

`esdf_distance` 可能仍显示许多空间中的规则体素点，但每个点都有有效观测距离，属于实际距离场，不是空白占位。不要用它代替表面 Mesh。

本轮点云均保持 `xyz/rgb/distance` 字段、20 字节/点、`base_link` 和对应源图像 stamp；未知点不发布，内部/障碍体素显示红色，其余距离由近到远着色。点数随实际观测、体素精度和 ROI 变化，不再固定为整个 ROI 的 217,800 个位置。

## 5. Mesh：增量原生接口与标准 RViz 显示

保留官方 `/ur7e_nvblox/mesh`，类型为 `nvblox_msgs/msg/Mesh`。Mesh 消费者存在时，由官方地图层发布模块更新并序列化真正的 TSDF 表面。

新增 `/ur7e_nvblox/mesh_markers`，类型为 `visualization_msgs/msg/MarkerArray`：

1. 有 Marker 消费者才创建内部 `/mesh` 订阅，触发官方 Mesh 需求；没有消费者时释放内部订阅。
2. 平稳运行时按增量 Mesh 块生成 TRIANGLE_LIST，块坐标作为 namespace、id=0。
3. 空三角形块发 DELETE；原生 clear 发 DELETEALL。
4. 新消费者出现、clear 或 reset 后，安排一次从当前实际 MeshLayer 生成的完整 ROI 快照，防止只收到后续增量而缺旧表面。除订阅数量外，按图层周期检查 endpoint GID，覆盖“连接替换但数量不变”的情况；不在每个 2 ms tick 查询图。
5. 快照使用原生 `serializeAllBlocks` 与官方 `markerMessageFromSerializedMesh`，只在需要重同步时执行；不是每次都发送完整 Mesh。
6. reset 时销毁旧内部订阅，并用 epoch 拒绝旧订阅的排队消息，减少旧 Mesh 跨代污染。

Mesh 视图按块与 ROI 相交裁剪，一个相交块中的三角形可能伸出精确 ROI 边界，最多涉及相交边界块；ESDF 点云则按体素中心严格裁剪。这两者不是完全相同的裁剪粒度。

标准 Marker 显示不加载 NVIDIA 自定义 Mesh 插件，可以绕开之前缺失 `rviz/glsl150/box.geom` 的插件资源路径。**这是本轮源码方案，尚未运行验证其在当前图形环境中的实际表现。** 默认配置提供标准 Mesh 开关，未默认开启以避免额外 Mesh 计算/传输。原生插件配置仍单独保留，资源完整时才选择。

目前配置 `use_color=false`，Mesh 是几何表面，不保证具有实景 RGB 纹理；本轮没有冒充已完成颜色流同步接入。首次完整快照可能增加处理延迟，Mesh 开启后的延迟需要重新测量。

## 6. slam_path 的实际来源和限制

新增 `/ur7e_nvblox/camera_path`，并默认提供 `/ur7e_nvblox/slam_path` 别名，均为 `nav_msgs/msg/Path`。路径来自成功深度融合所用的**图像时刻机器人 FK/手眼 TF**，不依赖每次 ESDF 发布；同时发布对应 `/camera_pose`。

- canonical 名称是 camera_path；slam_path 是便于查看的兼容名称，可用 `publish_slam_path_alias=false` 关闭。
- 不是视觉里程计估计，不含回环优化，不冒充官方 `/visual_slam/tracking/slam_path`。
- 最多保留 2,000 个 pose，按平移 ≥5 mm、旋转 ≥0.02 rad 或距离上次记录满 1 s 入队；路径消息发布上限 10 Hz。
- 每个 pose 保留自身图像 stamp，路径使用 global_frame。reset 时清空并发布空路径，epoch 增长。
- Path 发布使用 Reliable + Transient Local，晚加入的显示器可拿到最近路径。

如果需要真正的 cuVSLAM tracking/slam_path，应单独接入相应视觉 SLAM 节点与传感器流，再明确其 odom/map 和机械臂 base_link 的关系；本轮没有启动或新增一个假的 SLAM 估计器。

## 7. 可查看 Topic 清单

以下为**后续构建并启动 sparse-v2 后**的接口；本轮没有运行确认实际话题图。

| Topic | ROS 类型 | QoS / 查看方式 |
| --- | --- | --- |
| `/ur7e_nvblox/esdf_3d` | sensor_msgs/PointCloud2 | Reliable / Volatile / depth1；RViz PointCloud2，默认启用 |
| `/ur7e_nvblox/esdf_observed` | sensor_msgs/PointCloud2 | BestEffort / Volatile / depth1；距离带诊断 |
| `/ur7e_nvblox/esdf_distance` | sensor_msgs/PointCloud2 | BestEffort / Volatile / depth1；全部已观测距离，默认关闭 |
| `/ur7e_nvblox/mesh` | nvblox_msgs/Mesh | Reliable；官方增量协议，需官方插件或自有消费者 |
| `/ur7e_nvblox/mesh_markers` | visualization_msgs/MarkerArray | Reliable / Volatile / depth10；标准 RViz MarkerArray |
| `/ur7e_nvblox/camera_path` | nav_msgs/Path | Reliable / TransientLocal / depth1；标准 RViz Path，默认启用 |
| `/ur7e_nvblox/slam_path` | nav_msgs/Path | 同上；robot TF 路径的可关闭别名 |
| `/ur7e_nvblox/camera_pose` | geometry_msgs/PoseStamped | BestEffort；图像时刻相机 pose |
| `/ur7e_nvblox/map_epoch` | std_msgs/UInt64 | Reliable / TransientLocal；地图重置代际 |
| `/ur7e_nvblox/stats` | std_msgs/String JSON | 后端、接口版本、融合/ESDF/导出计数、块/点数与完成延迟 |
| `/ur7e_nvblox/tsdf_layer` | nvblox_msgs/VoxelBlockLayer | 继承官方；已知 TSDF 层，自定义插件协议 |
| `/ur7e_nvblox/tsdf_layer_marker` | visualization_msgs/Marker | 继承官方；标准 Marker 调试入口 |
| `/ur7e_nvblox/color_layer`、`color_layer_marker` | 原生层 / Marker | 继承官方；当前未接入 RGB 融合，不作为彩色重建完成证据 |
| `/esdf_input/depth`、`/esdf_input/camera_info` | Image / CameraInfo | 公共输入，保留图像时间与匹配内参 |
| `/tf`、`/tf_static` | tf2_msgs/TFMessage | 现有机械臂和相机 TF |

`static_esdf_pointcloud`、`static_map_slice` 等话题虽由基类创建，当前 3D 模式不执行其二维输出分支；有名字并不代表会有消息，不能推荐它们作为本轮 3D 主视图。

## 8. 本轮修改文件和预期结果

| 本仓库文件 | 改动 | 解决的问题 / 预期 |
| --- | --- | --- |
| [block_export.h](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.h)、[block_export.cu](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/block_export.cu) | 重写为已分配块遍历、按需多输出压缩、分批 scratch | 移除空 ROI 补点与密集可视化导出，未知不出现在点云中 |
| [native_mapper.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_mapper.cpp) | 新 ESDF 话题契约、路径/epoch、Mesh 需求与快照、统计接口 v2 | 可分别查看占用、距离、表面和轨迹；避免显示语义混淆 |
| [mesh_visualization.h](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/mesh_visualization.h) | 原生三角形转标准 Marker，坐标/索引检查、块删除 | 不依赖缺资源的自定义 Mesh 插件；不把 block 原点重复加到顶点 |
| [native_geometry_test.cpp](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/src/native_geometry_test.cpp) | 更新回归源码：原生距离参照、空地图、未知块、空 ROI、Mesh clear/delete | 为后续验证提供用例；本轮未执行，不能写“通过” |
| [gazebo.yaml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo.yaml) | 保持 3 cm、3D、原 ROI；图层显示目标设 5 Hz，增加路径配置 | 地图更新与显示解耦，不通过降精度制造提升 |
| [native.rviz](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/rviz/native.rviz) | 默认 occupied ESDF + camera_path；提供标准 Mesh、距离带、完整有效距离开关 | 默认不再看到未知盒子，按任务选择显示对象 |
| [旧 Python node.py](/home/czz/perception_eye_in_hand/src/ur7e_nvblox/ur7e_nvblox/node.py) | esdf_3d 发布前过滤未知和 d>0 | 避免兼容入口再把灰色未知盒子发布到同名话题；仍不是原生块路径 |
| [CMakeLists.txt](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/CMakeLists.txt)、[package.xml](/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/package.xml) | 显式加入 nav_msgs / visualization_msgs / nvblox_msgs，接口版本 0.2.0 | 构建声明与新接口一致 |
| [run_ur7e_native.sh](/home/czz/perception_eye_in_hand/run_ur7e_native.sh)、[run_ur7e_perception.sh](/home/czz/perception_eye_in_hand/run_ur7e_perception.sh) | 启动前检查源码是否比安装二进制新 | 防止实际仍运行旧版，却认为新语义已生效；统一入口在创建场景前检查 |

旧 Python 接口没有原生 is_inside 标志，所以采用 known && signed_distance≤0 的兼容筛选，零距离边界语义不能保证与原生标志完全相同。其内部仍保留 Torch 查询，不作为本轮 sparse-v2 原生结构的实现依据。推荐统一入口默认 C++ 后端。

## 9. 操作指令：供后续允许构建/运行时使用，本轮未执行

### 9.1 构建与统一启动

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_native.sh
# 如果场景尚未构建，再执行 ./build_ur7e_d435_scene.sh
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_ur7e_perception.sh gazebo_gui:=true
```

若使用现有同域场景，且没有重复 mapper/公共输入，用 `start_scene:=false`。不要与旧 Python mapper 同时启动。本轮没有触碰既有运行进程，也没有释放/占用端口；未来启动前仍应自行检查已有实例。

RViz Fixed Frame=`base_link`。默认查看 ESDF Occupied 与 Camera Path；要看表面，勾选 **Surface Mesh (enable to view)**；要看近障碍距离，勾选 **ESDF Distance Band**。不要同时打开所有视图再用其性能与关闭消费者的历史基线比较。

### 9.2 查看话题与指标

新终端使用相同 domain/RMW，source ROS；原生消息查看再 source 官方依赖：

```bash
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
source /opt/ros/humble/setup.bash
ros2 topic list -t
ros2 topic hz /ur7e_nvblox/esdf_3d --wall-time
# Ctrl+C 后分别运行下面命令
ros2 topic echo /ur7e_nvblox/stats --once
ros2 topic echo /ur7e_nvblox/camera_path --once --qos-durability transient_local
ros2 topic echo /ur7e_nvblox/slam_path --once --qos-durability transient_local
```

内部指标检查 `interface_revision=2`、`output_mode=allocated_blocks_sparse_v2`、`allocated_blocks`、`selected_blocks`、`visited_allocated_voxels` 和各类点数。`published_unknown_points=0` 是筛选规则保证的接口约束，不是本轮运行测得的计数。点数表示最近一次请求的相应输出，未订阅的输出可能为 0，不表示地图内绝对没有该类型体素。

Mesh 没有订阅者时按需不发布，`topic hz /mesh` 本身会创建消费者并触发 Mesh 工作。查看官方 Mesh 类型需要：

```bash
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
ros2 topic info /ur7e_nvblox/mesh --verbose
```

默认初始相机朝向可能不覆盖 ROI；如果更新在进行但点云为空，应手动将相机朝向工作区。保留原初始关节、手眼外参和工作范围，未为了制造画面而自动移动机械臂。既定测试观察姿态见 [统一启动说明](/home/czz/perception_eye_in_hand/docs/PERCEPTION_INTEGRATED_DESIGN_AND_STARTUP.md)。

### 9.3 清图与按需三维距离查询

```bash
ros2 service call /ur7e_nvblox/reset_map std_srvs/srv/Trigger '{}'
# Mesh 显示已开启时，可手动请求一次完整重同步
ros2 service call /ur7e_nvblox/republish_mesh std_srvs/srv/Trigger '{}'
ros2 service call /ur7e_nvblox/get_esdf_and_gradient nvblox_msgs/srv/EsdfAndGradients \
  "{update_esdf: false, visualize_esdf: false, use_aabb: true, frame_id: 'base_link', aabb_min_m: {x: -0.1, y: -0.1, z: 0.0}, aabb_size_m: {x: 0.2, y: 0.2, z: 0.2}}"
```

查询服务仍可以生成稠密 AABB 数组，因为调用者明确需要完整距离布局；这是按需数据接口，不再是默认显示点云。服务未知值默认 -1000，不能按“负数就是占用”判断；当前安装实现输出三维距离数组，没有附加梯度通道。

### 9.4 后续验收清单（未执行）

后续构建完成后，可单独执行更新的 CUDA 回归程序：

```bash
cd /home/czz/perception_eye_in_hand
ESDF_BUILD_GEOMETRY_TEST=ON ./build_ur7e_native.sh
ESDF_NATIVE_EXECUTABLE=native_geometry_test ./run_ur7e_native.sh
```

Python 兼容分支另在 `test_publish_versions.py` 中增加了 unknown/free padding 过滤用例，本轮同样未执行。旧的“13 项通过”只属于历史代码，不能替代这些修改后的回归验证。


1. 编译并检查新依赖/接口是否匹配当前安装的官方库。
2. 运行更新的 `native_geometry_test`，验证空地图和已分配但未知的块都不输出点、空 ROI 不扫描合成网格、原生距离与官方服务对应。
3. 检查 ESDF 三话题无未知占位、点数不固定为 ROI 体积单元数；分别核对 is_inside 与距离带。
4. 打开/关闭 Mesh，验证增量、空块删除、晚加入完整快照、reset 清除以及无消费者时不额外更新 Mesh。
5. 核对 camera_path 的坐标、每 pose 图像时间、容量、reset，以及明确的 robot_tf 来源。
6. 在相同 3 cm、640×360、同 ROI/姿态/消费者条件下重新测量 Hz、端到端延迟、GPU/CPU 与长期增长；Mesh 开/关分别报告。

## 10. 兼容性、预期和未完成部分

- `/esdf_3d` 从“完整 ROI（含未知）”改成“不定长占用有效体素快照”，是有意的破坏性语义修正。任何依赖固定长度、数组下标、未知哨兵或固定点顺序的规划程序必须改用显式查询服务，不能直接沿用旧假设。
- 统计字段不再用 query_cells 表示 ROI 栅格数，而报告实际块/槽位和输出数量。旧历史验证脚本如 `validate_recorded_interfaces.py` 假设 217,800 点，只用于旧归档，不能拿来验收 v2。
- 预期消除未知灰盒、减少空区域访问和消息体积，让 Mesh、距离场、轨迹各自有明确观察接口；**具体加速比和画面效果未验证**。
- 此前 2.4 ms 等结果属于 sparse-v2 之前的实现，不证明新代码已编译通过，也不能直接写成新版本性能。
- 未新增视觉 SLAM 引擎、RGB 融合、自体过滤或动态目标清除，未改造全部规划/真机消费者，未进行实机测试。
- 本次只进行了本地源码阅读、文件编辑及静态文本检查；严格没有构建、测试或启动。旧二进制/旧运行实例不会因为源码改变自动更新。
