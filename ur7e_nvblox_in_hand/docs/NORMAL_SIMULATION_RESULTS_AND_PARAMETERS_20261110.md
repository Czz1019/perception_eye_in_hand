# 正常配置的仿真效果、测试结果与配置参数说明

日期：2026-11-10。功能包：ur7e_nvblox_in_hand。本文只整理已经完成的正常时间匹配组，不增加测试、不修改源代码、不调整参数。人工时间偏移实验不属于正常应用配置，不纳入本文结果。

## 1. 结果来源与范围

结果来自已有正常组 /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/delay0ms。该组通过 0 偏移测试转发器转发 TF，其坐标与时间戳均未改变；不是再次采集或新运行。输入为约 87 秒实验室场景仿真扫描数据，640×480、深度和彩色约 29.95 Hz，图像与对应 CameraInfo 未发现缺失配对。

场景是工作台、试管架、瓶子、托盘和尺寸块；运动为既有缓慢横向扫描与停留。建图测试以 1 倍速回放，GUI 关闭，有实际 Mesh 和 ESDF 接收器。不是实机测试，也不是 RViz 渲染帧率测试。

## 2. 实际输出结果

| 指标 | 正常组结果 |
|---|---:|
| 深度接收回调 Hz | 29.96 |
| 彩色接收回调 Hz | 29.94 |
| 深度融合调用 Hz | 28.73 |
| 颜色融合调用 Hz | 28.79 |
| 3D ESDF 更新调用 Hz | 23.88 |
| ESDF 点云发布调用 Hz | 23.88 |
| Mesh 发布调用 Hz | 4.81 |
| 完整非空 ESDF 点云接收 Hz | 23.41 |
| 最后一帧导出 ESDF 点数 | 542,383 |
| 输入内参至点云接收 P50，ms | 96.84 |
| 输入内参至点云接收 P95，ms | 149.21 |

频率统计剔除回放前 10 秒，截止回放完成。融合和更新按代码现有计数计算，不是 CUDA 完成率；实际点云接收是监测订阅器收到完整非空消息的速率，不是 RViz FPS。延迟从同一深度时间戳的 CameraInfo 接收开始，到完整 ESDF 点云接收结束，不含相机曝光和屏幕绘制。该正常组属于之前的时间匹配检查运行，期间有离线检查并发，频率只说明该次实际运行，不替代严格的多轮性能基准。

![正常配置下实际保存的 Mesh](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/normal_effect_summary_20261110/normal_mesh.png)

左图为实际录制的 RGB，右图为保存 PLY 的三角面射线渲染，非 RViz 截图。RGB 使用彩色光学系，Mesh 按深度光学系观察，两者存在小视差，不能逐像素直接计算颜色误差。

![正常配置下实际 ESDF](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/normal_effect_summary_20261110/normal_esdf.png)

左侧为实际接收 ESDF 的桌面 ROI 三维显示，为可读性再绘制每 6 个中的 1 个；右侧为同一场在 z=0.125 m 的切片，切片没有额外抽稀。地图体素 1 cm，运行导出 stride=2，因此导出点典型间距为 2 cm。图中颜色是障碍距离，非表面 RGB；点包含观测自由空间，空白不等于可通行。没有人为补齐 unknown。

## 3. 当前质量能够说明什么

三个评估视角中，原始深度对程序化场景几何的误差 P95 约 3.06 mm；正常 Mesh 在该组有效像素上的 P95 约 16.66 mm。排除桌面后的物体表面 P95 约 55.22 mm，说明试管等细结构仍有明显误差。这里是相机视线方向的深度误差，不是全场景最近表面误差。

ESDF 本轮证明了持续生成与送达，没有重新完成距离真值误差、全图 unknown 覆盖、动态清除或抓取安全性验证。不能把 Mesh 的误差数字当作 ESDF 的距离误差。

## 4. 实际使用的配置文件与加载顺序

| 文件 | 用途 |
|---|---|
| [base.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/config/base.yaml) | 声明完整 mapper 基础配置；通用输入 QoS 默认 sensor_data。 |
| [realtime.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/config/realtime.yaml) | realtime 配置覆盖基础文件中的部分参数，目前为 1 cm、stride=2、40/40 Hz 融合门限、33.4 Hz ESDF 上限。 |
| [bringup.launch.py](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_nvblox_in_hand/launch/bringup.launch.py) | 组织相机/场景、mapper 与 RViz，处理命令行覆盖和输入话题。 |
| [本次运行参数快照 params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/delay0ms/params.yaml) | 节点实际生效值；检查本次结果时以此为准。 |
| [本次启动命令 command.txt](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/delay0ms/command.txt) | 本次 mapper 回放启动命令，未启动新场景或真实相机。 |
| [相机仿真 URDF](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/urdf/ur7e_d435.urdf.xacro) | 相机传感器 30 Hz、宽高参数、光学系、插件发布设置。 |
| [实验室场景配置](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/config/lab_scene.yaml) | 工作台及试管架等物体尺寸与位置。 |
| [实验室初始关节位置](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/config/lab_initial_positions.yaml) | 机械臂初始观察姿态。 |
| [手眼标定](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/src/ur7e_d435_gazebo/config/ur7e_d435_handeye_20260907.yaml) | tool0 与 camera_color_optical_frame 之间的标定定义；不得直接当成 tool0→camera_link 数值。 |

mapper 加载顺序：base.yaml → realtime.yaml（本次默认 profile）→ 可选 mapper_config → 启动覆盖。本文运行明确覆盖 input_qos=reliable；因此基础文件中的 sensor_data 不代表这次实际使用 Best Effort。配套仿真启动时也会自动匹配相机 QoS，外部源则应先检查其发布策略。

## 5. 参数值和含义

### 5.1 输入、时间和处理队列

| 参数/设置 | 本次值 | 含义 |
|---|---|
| 相机宽×高 / 相机帧率 | 640×480 / 30 Hz | 传感器设定，与 mapper 的融合上限分开。 |
| use_sim_time | true | 用 /clock 解释 ROS 时间戳；各输入应使用同一仿真时间基准。 |
| global_frame | base_link | 地图坐标系。 |
| input_qos | reliable | 图像和 CameraInfo 可靠订阅；发布端也必须兼容。 |
| input_queue_size | 3 | mapper 每路应用输入队列长度；不同于 DDS 历史深度。 |
| max_input_age_sec | 0.5 s | 输入超过年龄限制后丢弃，不是允许 pose 错配 0.5 秒。 |
| depth_hz / color_hz | 40 / 40 Hz | 按图像时间戳设置最短接纳间隔约 25 ms；不是把相机改为 40 Hz。 |
| min_depth_m / max_range_m | 0.1 / 3.0 m | mapper 过滤深度量程；不同于传感器的原始裁剪距离。 |
| depth_topic / depth_info_topic | /camera/depth/image_rect_raw；/camera/depth/camera_info | 深度及其内参输入。 |
| color_topic / color_info_topic | /camera/color/image_raw；/camera/color/camera_info | 彩色及其内参输入。实机通常是 /camera/camera 前缀，应按实际话题覆盖。 |

### 5.2 TSDF 几何融合

| 参数 | 本次值 | 含义 |
|---|---|
| voxel_size | 0.01 m | TSDF/ESDF 的空间离散尺度，不是保证所有表面误差小于 1 cm。 |
| truncation_vox | 4.0 | TSDF 截断距离参数，换算为 4×0.01=0.04 m。 |
| max_weight | 5.0 | 累计权重上限，不是只融合 5 帧。 |
| weighting_mode | inverse_square_tsdf_distance_penalty | 传给官方底层的融合加权模式，按观测距离等因素调整贡献。 |
| invalid_depth_decay_factor | -1.0 | 传给底层的无效深度衰减设置；不把无效深度当作有效表面。 |
| raycast_subsampling_factor | 4 | 候选块可见性计算的射线抽样，不是把实际深度输入降到 160×120，也不是 ESDF 显示 stride。 |
| workspace_bounds_type | unbounded | 未启用显式工作空间盒裁剪；已配置的 min/max 边界不应当作正在生效的 ROI。 |
| workspace_min/max_x,y,z | x[-2,2]、y[-2,2]、z[-0.5,2] m | 预留边界值，需要与对应 bounds 模式一起使用。 |

### 5.3 ESDF 更新与显示

| 参数 | 本次值 | 含义 |
|---|---|
| enable_esdf / enable_esdf_view | true / true | 分别启用场更新和可视化输出。 |
| esdf_mode | 3d | 更新三维距离场。 |
| esdf_hz | 33.4 Hz | 工作线程允许的更新上限，约 29.94 ms；不是实测更新频率。 |
| esdf_view_hz | 33.4 Hz | 可视化导出上限；相同/更快显示设置跟随新 ESDF 版本，不凭空生成新观测。 |
| esdf_view_mode | observed | 导出已观测 ESDF 体素，包含自由空间距离点。 |
| esdf_view_stride | 2 | 在 XYZ 上对原生体素索引抽样；显示间距约 2 cm，完整地图仍为 1 cm。 |
| esdf_view_band_m | 0.24 m | band 显示模式使用的距离带；当前 observed 模式不应将其理解为全图只显示 24 cm 距离带。 |
| esdf_max_distance_m | 2.0 m | 底层距离场传播/距离上限参数；饱和值不是实际测量到远处障碍的证明。 |
| esdf_min_weight | 0.1 | ESDF 使用 TSDF 观测的最低权重门限。 |
| esdf_max_site_vox | 2.0 | ESDF 种子判定的距离阈值参数，单位体素；不是安全膨胀半径。 |
| publish_esdf_distance_slice | true | 与二维切片输出有关；当前 3d 模式主要输出 esdf_points，不能据此说还单独重算了 2D ESDF。 |
| slice_min_height / slice_max_height / slice_height | 0 / 0.8 / 0.25 m | 2D ESDF 切片配置；当前三维更新不使用它们替代体积 ESDF。 |

33.4 Hz 是当前已有配置，本次没有修改它，也没有增加频率档位测试。即使上限更高，深度新观测约 30 Hz，工作线程导出/读回等任务也会降低实际更新次数。

### 5.4 Mesh、地图层及维护

| 参数 | 本次值 | 含义 |
|---|---|
| enable_depth / enable_color | true / true | 开启几何及颜色融合。 |
| enable_mesh / mesh_hz | true / 5 Hz | Mesh 输出开关及上限；需要实际订阅才触发对应导出。 |
| mesh_min_weight / mesh_weld_vertices | 0.1 / true | Mesh 提取权重门限和顶点焊接设置。 |
| enable_layers / layer_hz / layer_min_weight | true / 5 Hz / 0.1 | TSDF/颜色层显示设置；无对应订阅时不会据此强制每次导出。 |
| decay_hz / clearing_hz | 0 / 0 Hz | 本次关闭周期衰减及半径清理，静态重建可能保留运动机械臂历史表面。 |
| clearing_radius_m | -1.0 m | 未启用半径清理；启用时必须设置有效正半径。 |
| tsdf_decay_factor / tsdf_decayed_weight_threshold | 0.95 / 0.001 | 衰减算法预设，本次 decay_hz=0，因此不周期执行。 |
| publish_pose / publish_path | true / true | 发布按深度图时刻查询的相机 pose 和路径，路径不是 SLAM 估计。 |
| path_max_poses | 3000 | 路径缓存长度上限，路径输出在代码中约限为 5 Hz。 |
| max_query_points | 200000 | 单次 ESDF 查询服务的点数限制，不限制导出点云为 20 万点。 |
| service_timeout_sec | 20 s | 服务任务等待超时参数，不是图像匹配时间容差。 |

## 6. 主要输出话题

| 话题 | 消息 | 说明 |
|---|---|
| /ur7e_nvblox/esdf_points | sensor_msgs/msg/PointCloud2 | 三维 ESDF 导出点：xyz、距离 intensity、rgb、observed/inside/site；不是物体表面点云。 |
| /ur7e_nvblox/mesh | visualization_msgs/msg/MarkerArray | Python 包发布的 Mesh 显示消息，不是另一个 C++ 包的 nvblox_msgs/Mesh。 |
| /ur7e_nvblox/tsdf_layer | sensor_msgs/msg/PointCloud2 | 按需发布的 TSDF 层显示。 |
| /ur7e_nvblox/color_layer | sensor_msgs/msg/PointCloud2 | 按需发布的颜色层显示。 |
| /ur7e_nvblox/camera_pose | geometry_msgs/msg/PoseStamped | 成功深度融合对应的相机 pose，时间戳来自深度图。 |
| /ur7e_nvblox/camera_path | nav_msgs/msg/Path | 机械臂 TF 推导的相机路径。 |
| /ur7e_nvblox/metrics | std_msgs/msg/String | 约 1 Hz 诊断，内部 totals 用于计算融合/更新频率，不能用该话题自身 Hz 代替 ESDF Hz。 |

## 7. 按正常配置启动仿真查看

本次只提供运行说明，没有重新启动仿真。以下启动配套实验室场景和 RViz，不自动执行任何扫描，不含测试转发器和任何人为 TF 时间偏移。已有编译产物时直接运行；修改源码后先按项目 build.sh 编译。

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  scene_variant:=lab gazebo_gui:=true run_rviz:=true \
  camera_width:=640 camera_height:=480 \
  sim_camera_reliability:=reliable input_qos:=reliable \
  voxel_size:=0.01 truncation_vox:=4.0 \
  depth_hz:=40.0 color_hz:=40.0 \
  esdf_mode:=3d esdf_hz:=33.4 esdf_view_hz:=33.4 \
  esdf_view_stride:=2 mesh_hz:=5.0
```

另一个终端核对实际参数与输出：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox esdf_hz
ros2 param get /ur7e_nvblox esdf_view_hz
ros2 param get /ur7e_nvblox input_qos
ros2 topic echo /ur7e_nvblox/metrics --once
ros2 topic hz /ur7e_nvblox/esdf_points --wall-time
```

GUI/RViz 会增加负载，因此用户开启窗口后的实测 Hz 可能低于本文 GUI 关闭时的结果。端口被占用时先确认并关闭自己的上一组仿真，不要重复启动。

## 8. 文件与现阶段不足

- [正常组保存的 Mesh](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/delay0ms/mesh.ply)，可在 CloudCompare 中打开。
- [正常组完整地图](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/delay0ms/map.blox)，包含底层地图层。
- [正常组原始点云 CDR](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pose_time_test_20261110/delay0ms/last_received_esdf.cdr)，包含最后一次接收到的点云消息。
- [本说明的数据汇总](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/normal_effect_summary_20261110/normal_results.json)。

目前输入送达已接近 30 Hz，但完整 ESDF 输出仍约 23 Hz；表面细节损失和运动机械臂未自过滤的问题仍需处理。本文不把历史人为偏差组当成当前正常运行配置，也不把输出频率提升当成几何精度已经合格。
