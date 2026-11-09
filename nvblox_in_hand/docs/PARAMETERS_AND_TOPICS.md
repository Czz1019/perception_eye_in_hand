# 重构版参数、话题和输出契约

2026-11-05后续：查看三维点云使用 `./run_esdf.sh`，显式开启原生体素诊断导出；三维色带调整为−0.09～0.24m，二维保留0～1m。[发布条件、真实截图和新实测](ESDF_DENSE_COLOR_AND_PUBLISH_20261105.md)。

2026-11-05：realtime融合门限40Hz，相机30Hz，ESDF/三维诊断显示门限33.4Hz；距离配色方法参考workspaces二维，当前三维范围见顶部后续说明。实际调用32.78Hz、诊断发布29.44Hz不等于新地图频率，见 [最新四组对照](ESDF_334HZ_AND_COLOR_TEST_20261105.md)。

后续运行已验证：原1280×720结果见重构复测报告；当前默认640×480及实际3D数据见 [最新输入复测](INPUT640_3D_ESDF_TEST_20261104.md)。以下接口说明以当前代码为准，历史未测试说明指重构交付阶段。

适用2026-11-04示例主体重构。原理见 [代码分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md)，命令见 [操作说明](OPERATIONS.md)。下表为代码配置，实际频率见最新报告。

## 1. 默认与配置顺序

默认官方NvbloxNode组件、单D435、static_tsdf、机械臂图像时间TF、esdf_mode=3d、profile=realtime。默认RViz看官方Mesh，不启动项目三维点云适配。

顺序：nvblox_base → D435 specialization → robot/ur7e → profile → 时间源 → mapper_config → 显式数字CLI及固定输入/功能约束。数字CLI默认为空以继承配置；布尔开关、esdf_mode和global_frame仍最终覆盖配置。

## 2. 主要参数和开关

| 参数 | 默认/用途 |
| --- | --- |
| mapper_backend | official：官方组件；diagnostic：附加计数及GPU点云；observed_view为诊断旧别名 |
| profile | realtime：3cm、输入30Hz、深度/颜色融合限频40Hz、ESDF门限33.4Hz、Mesh/层5Hz；reference：继承参考base数值 |
| esdf_mode | 3d；可选2d原生高度投影，两者不是同一Mapper并行运行的两个场 |
| container_name / attach_to_container | nvblox_container / false；仅官方后端支持加载既有容器 |
| camera_width / camera_height | 640 / 480；控制本项目Gazebo及可选真实D435驱动，不会改变外部已运行相机或bag |
| start_scene / launch_camera | true / false；场景和真实相机互斥 |
| use_sim_time | true；真实D435使用false |
| enable_depth / enable_color | true / true；reference档也默认开启颜色 |
| enable_esdf / enable_mesh / enable_layers | true；保留计算/发布需求开关，Mesh/层有公共发布任务 |
| depth_hz / color_hz / esdf_hz / mesh_hz / layer_hz | 默认空，继承运行档；显式数字覆盖；mesh_hz是实际公共发布频率的别名，与layer_hz同时指定须一致 |
| voxel_size / max_range_m | 默认空；realtime为0.03m，本项目范围3m |
| decay_hz / clearing_hz / clearing_radius_m | 默认空；realtime关闭，reference继承2Hz/1Hz/7m |
| global_frame | base_link；robot光学TF来自实际手眼及运动学 |
| slice_height / slice_min_height / slice_max_height | 默认空，机器人配置0.25/0/0.8m，2d投影时使用 |
| publish_esdf_distance_slice | true；不能迫使本地3d模式输出二维点云 |
| mapper_config | 可选绝对路径YAML，适合覆盖细分算法参数 |
| run_rviz / rviz_mode | true / auto；auto按后端及ESDF模式选mesh、2d或diagnostic |
| publish_pose / publish_path | true / true；路径只是TF旁路，不是SLAM，也不是mapper输入中转 |
| rosbag / rosbag_args | 空；回放需关闭场景/真实驱动并使用仿真时间；默认回放录制/clock，不主动再造时钟 |
| enable_esdf_view / esdf_view_hz | 仅diagnostic有效，true / 33.4Hz；独立于核心ESDF门限 |
| esdf_view_mode | 仅diagnostic，observed默认，band或inside可选；surface为inside旧别名 |
| esdf_view_band_m | 仅diagnostic，0.12m，band裁剪阈值 |

本地update_mesh_rate_hz虽声明但未找到调度使用；Mesh在LayerPublisher中按订阅需求计算。mesh_hz控制真实publish_layer_rate_hz，0会停止公共发布；只关闭Mesh应使用enable_mesh=false，而非假设独立Mesh周期。

禁用层采用重映射，使普通消费者不再产生该层需求；不要手动订阅disabled命名空间。关闭enable_esdf只停周期更新，原生服务请求update_esdf=true仍可以更新。

## 3. 地图与状态输出

| 话题 | 类型 | 条件与含义 |
| --- | --- | --- |
| /nvblox_node/mesh | nvblox_msgs/msg/Mesh | 原生增量三角网格与RGB，订阅驱动发布 |
| /nvblox_node/tsdf_layer、color_layer | nvblox_msgs/msg/VoxelBlockLayer | 官方几何/颜色体素层；不是完整ESDF |
| /nvblox_node/tsdf_layer_marker、color_layer_marker | visualization_msgs/msg/Marker | 原生标准RViz体素显示 |
| /nvblox_node/static_esdf_pointcloud | sensor_msgs/msg/PointCloud2 | 仅2d模式有切片消息，intensity是距离；有advertise不等于有更新 |
| /nvblox_node/static_map_slice | nvblox_msgs/msg/DistanceMapSlice | 官方二维距离切片 |
| /nvblox_node/static_occupancy_grid | nav_msgs/msg/OccupancyGrid | 官方二维占据输出 |
| /nvblox_node/esdf_slice_bounds、workspace_bounds | visualization_msgs/msg/Marker | 官方边界诊断，不代表填充了整个空间 |
| /nvblox_in_hand/camera_pose | geometry_msgs/msg/PoseStamped | CameraInfo时刻的机器人光学相机TF |
| /nvblox_in_hand/camera_path | nav_msgs/msg/Path | 有限长度实际TF采样路径，不是SLAM估计 |
| /nvblox_in_hand/pose_health | std_msgs/msg/String | CameraInfo接收、TF解析和超时旁路统计，不是融合计数 |
| /nvblox_node/esdf_points | sensor_msgs/msg/PointCloud2 | 仅diagnostic：真实原生体素的选择集合；默认observed包含有效inside与outside |
| /nvblox_node/metrics | std_msgs/msg/String | 仅diagnostic：成功融合时间推进、ESDF入口调用、主机耗时与最近导出计数 |

Mesh/层通常Reliable/Volatile并按订阅需求更新，路径为Transient Local。默认官方后端以原生日志ros/update_esdf和save_rates/save_timings观察内部任务，不新增一个假统计话题。

## 4. 诊断点云格式与局限

PointCloud2的point_step=24：x/y/z FLOAT32在0/4/8，distance和intensity共享FLOAT32偏移12，rgb UINT32在16，observed/is_inside/is_site UINT8在20/21/22，23为保留字节。不能用旧脚本硬编码20字节解包新版消息。

所有输出点都已观测且距离有限。observed模式保留全部有效体素，band再按距离裁剪，inside仅保留inside。inside site可为零或负零距离，应读is_inside分类，不能仅用distance>=0作为自由判断。

没有输出的坐标不能自动解释为自由或unknown；点云没有完整unknown掩码，也没有梯度。精确几何表面看Mesh，规划地图应使用带清晰有效性契约的原生层/有界服务。点大小不改变体素精度。

## 5. 原生服务

`/nvblox_node/save_map`、load_map、save_ply、save_rates、save_timings使用nvblox_msgs/srv/FilePath；get_esdf_and_gradient使用EsdfAndGradients。保留原始服务语义。

本地get_esdf_and_gradient转换实际返回有界标量距离栅格及origin/voxel_size，没有明确独立xyz梯度通道，unknown采用配置默认值。调用可以触发ESDF更新、按请求清理形状和可选可视化，不应当无代价读取；不要连续请求大AABB衡量核心融合频率。
