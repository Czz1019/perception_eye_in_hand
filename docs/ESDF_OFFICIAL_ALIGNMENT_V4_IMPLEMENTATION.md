# 以 workspaces 官方 ROS 结构为主的感知优化实现（0.4.0 / 接口 v4）

后续操作更新：[RViz 鼠标交互修复与障碍物组多视角扫描](/home/czz/perception_eye_in_hand/docs/RVIZ_INTERACTION_AND_ORBIT_SCAN_GUIDE.md)。局部 view_sweep 仍保留，新增 orbit_scan 用于多方向、多高度采集；两者不能同时控制机械臂。

日期：2026-10-27。修改位于 `/home/czz/perception_eye_in_hand`。主要参考和实际链接依赖仍为 `/home/czz/workspaces/isaac_ros-dev` 中的官方 ROS 移植版；未修改该参考库。

本版已经完成程序修改、构建、安装和不启动 ROS/GPU 的 Mesh 协议检查。沿用此前“不运行仿真”的约束，没有启动新版 Gazebo、感知、RViz，也没有发送机械臂运动。因此本文说明已实现的结构和预期，不宣称已实测达到参考的频率或完整度。

## 1. 本次优化不是只调整 voxel_size

原来的应用虽然继承官方节点，但自己重写主循环、逐帧同步 GPU、管理 Mesh 重建和显示快照。这使代码组织与参考实现仍有明显区别。

本次将主处理权交回官方 `NvbloxNode::tick()`，应用层只增加必要的校验、相机轨迹、异步统计、稀疏三维 ESDF 显示和 Mesh 显示适配。深度融合、颜色融合、TF、服务处理、TSDF/ESDF 算法、Mesh 更新和原生图层发布均继续使用官方模块。

保持不变：3 cm 体素、640×360 公共深度输入、1280×720 RGB、base_link、导出中心 [0,0,0.5]、尺寸 [2,2,1.5] m、±15 cm 距离带、2 m ESDF 最大距离及既有手眼配置。没有用降精度或缩小工作区制造提速。

本次也没有虚构“已完成自体过滤、全视角轨迹或平面平滑”。这些仍需单独实现和验收，不能由主循环对齐替代。

## 2. 代码组织与实际复用关系

```text
perception_eye_in_hand/
├── run_ur7e_perception.sh                 统一入口，仍使用本项目
├── run_ur7e_native.sh                     构建校验、官方依赖环境、输入映射
├── src/ur7e_nvblox_native/
│   ├── src/native_mapper.cpp              官方节点薄适配：校验、轨迹、版本、导出
│   ├── src/mesh_view.h / mesh_view.cpp     独立显示模块：消费者、增量桥接、重连快照
│   ├── src/mesh_visualization.h            三角形/颜色/块删除协议转换
│   ├── src/gpu_timings.h                   有界 CUDA 事件池，非阻塞统计
│   ├── src/block_export.h / block_export.cu 原生块遍历，按需导出 ESDF
│   ├── src/mesh_protocol_test.cpp          不启动 ROS/GPU 的协议回归检查
│   ├── config/gazebo.yaml                  默认：参考型 5 Hz ESDF 调度
│   └── config/gazebo_fast_esdf.yaml        同算法、同精度的 30 Hz ESDF 目标
└── src/ur7e_d435_gazebo/
    ├── config/gazebo_clock.yaml            gazebo_ros_init 时钟发布目标 100 Hz
    └── launch/scene.launch.py              把时钟参数传给标准 Gazebo ROS 启动器
```

MeshView 是同一进程内的独立 C++ 模块，**不是额外进程或并行地图计算线程**。与官方 LayerPublisher 的职责划分类似，显示管理不再堆在主节点中。它与地图操作使用同一个 processing callback group，保证访问原生地图的串行一致性。

### 2.1 恢复官方主调度

新主循环的实际顺序：

1. 检查 ROS 时间回退，必要时清图。
2. 更新标准 Mesh 显示的订阅需求。
3. 直接调用 `NvbloxNode::tick()`。
4. 官方图层发布发生后，按需补齐标准显示快照。
5. 非阻塞查询 CUDA 测量事件。
6. 有新 ESDF 版本且有消费者时，导出实际已分配块中的有效体素。

第 3 步完全采用 workspaces 当前源码的执行顺序：服务请求 → 深度队列 → 颜色队列 → lidar（关闭）→ 衰减/清理 → ESDF → 订阅图层 → 调试输出和原生统计。

删除了应用自行维护的 ESDF token budget 和 steady-time 多阶段调度表，避免形成与参考不同的第二套执行策略。保留“没有新深度则不重复推进应用 ESDF 版本”的保护；显示不会通过重复发布旧版本冒充更新。

### 2.2 移除为了统计添加的逐帧 GPU 等待

旧版在每次有效深度处理后、每次 ESDF 处理后执行 `cuda_stream_->synchronize()`，以便记录完成时间。这会阻塞 CPU 处理组，限制主机与 GPU 工作重叠。

新版使用 `GpuTimings`：

- 在官方处理前后记录同一流的 CUDA event。
- `cudaEventQuery` 返回未完成时直接返回，不阻塞等待。
- 完成后使用 `cudaEventElapsedTime` 读取流上事件间隔。
- 事件复用，最多保留 64 个待测样本；超限只丢统计样本，不丢图像或计算任务。

这里删除的是**应用额外添加的强制同步**，没有修改官方底层必要的同步。ESDF 点云需要 GPU→CPU 数据时仍会等待实际结果；显式 reset_map 也保留同步屏障。不能把本次改动描述成“整个程序没有任何 GPU 同步”。

### 2.3 Mesh 保持官方增量更新，重连不再重建全地图

Mesh 几何更新仍由官方 `LayerPublisher::serializeAndpublishSubscribedLayers()` 在有消费者时调用 `Mapper::updateMesh()` 完成。

旧的标准显示重连路径另行调用 `updateMesh(kYes)`，强制更新整张地图。新版 MeshView 只在官方图层周期之后读取已经更新的 MeshLayer，并用官方 serializer 和 Marker 转换生成快照，不再为了显示重连再次强制全地图重建。

保留快照的原因：当前参考版本的原生 Mesh `clear`/新订阅者标志不宜被当作“历史所有块必然都在这一条消息中”的保证；静止图像、重连和地图已有历史区域时，标准 RViz 仍需明确的完整显示恢复。新版没有为了减负简单删除这个完整性路径。

原生 `/mesh` 增量数据、块删除、clear、消费者重连 GID、空地图和地图 epoch 都保留。无标准 Mesh 消费者时释放内部原生订阅，不为该显示额外计算 Mesh；其他原生 Mesh 消费者仍可独立触发官方更新。

### 2.4 深度校验补齐

深度进入官方模块前增加图像/CameraInfo 尺寸一致、焦距有限且为正的校验。时间戳、光学 frame 和过期帧检查继续保留。颜色仍使用自身内参和图像时间的 TF，没有拿深度内参冒充 RGB 内参。

## 3. 默认参数：对齐参考策略，保持项目精度

| 项目 | v3 | v4 默认 | 作用与边界 |
| --- | --- | --- | --- |
| 主循环 | 应用自定义调度 | 官方 NvbloxNode::tick | 收敛到参考处理结构 |
| tick_period_ms | 2 | 10 | 与参考基础参数一致，减少无必要唤醒 |
| TSDF 加权 | inverse_square | inverse_square_tsdf_distance_penalty | 与参考基础参数一致，减弱截断带外测量影响 |
| ESDF min_weight | 0.0001 | 0.1 | 与参考一致，减少弱观测过早进入距离场 |
| 队列上限 | 2 | 10 | 与参考一致，提高等待图像时间 TF 时的容纳能力；仍受 0.5 s 帧龄上限保护 |
| ESDF 目标 / 点云导出上限 | 30 / 30 Hz | 5 / 5 Hz | 采用参考的任务预算，减少传播/可视化争用 |
| 图层发布 / 颜色积分目标 | 5 / 5 Hz | 5 / 5 Hz | 保持一致 |
| 体素 / 深度 / ROI | 3 cm / 640×360 / 原范围 | 不变 | 保持精度与输入对照 |

**默认 ESDF 目标从 30 Hz 改为 5 Hz，是任务预算变化，不是已经测得的“30 Hz 性能优化结果”。** 几何融合和相机输入没有随之降到 5 Hz。需要较新距离场的场景应选择 30 Hz 目标配置，再验收实际延迟和吞吐。

以下为有意保留的项目差异：

- 深度积分参数仍为 1000 Hz，表示不额外限制原有约 30 Hz 相机；这不是 1000 Hz 的真实融合性能。避免严格限频边界导致合法相机帧跳过；参考对照应使用同一设置。
- ESDF site 阈值保留 1 voxel，没有把它作为画面平滑开关。3 cm 下照搬参考 2 voxel 会改变到 6 cm 的 site 范围，需距离语义实验后再决定。
- 保持静态地图不衰减、不按半径清图，以免通过删除旧障碍提升表面观感。机器人自体残影问题仍需专门过滤。
- 后台地图仍是 unbounded，现有 ROI 只裁剪导出和标准显示块，不声称已经限制了整个地图分配。
- RGB 继续启用，参考基础配置默认关闭；做性能对照必须对齐颜色开关和消费者。

因此本版不是机械复制一整份 YAML，而是在项目的位姿、精度和应用要求下复用参考代码及主要融合策略。

## 4. 为什么要处理 /clock

官方主循环用 ROS 时间判断 ESDF/图层处理是否到期。恢复它之后，低频 `/clock` 会成为实际调度上限之一，尤其是 30 Hz 配置。

本项目场景启动增加 `gazebo_params_file` 参数，默认读取 `config/gazebo_clock.yaml`，给标准 gazebo_ros_init 的 `/gazebo` 节点设置 `publish_rate: 100.0`。通过 Humble 安装的 `gzserver.launch.py` 支持的 `params_file` 参数传递。

这只提高仿真时钟消息发布目标，不改变 Gazebo 物理步长、相机 30 Hz、相机分辨率或仿真实时因子。是否生效及实际墙钟频率需运行后确认。本轮只确认了本机标准启动器支持该参数，安装的插件含 publish_rate 参数入口，并完成场景包构建。

使用 `start_scene:=false` 接已有场景时，新配置不会替换正在运行的 Gazebo。此时应检查已有 `/gazebo` 参数与 `/clock`，不能以为重启 mapper 就提高了场景时钟频率。

## 5. 话题保持 v3 名称，统计升级 v4

继续使用：

- `/ur7e_nvblox/mesh`：官方原生增量 Mesh。
- `/ur7e_nvblox/mesh_markers`：标准 RViz 彩色三角形视图。
- `/ur7e_nvblox/color_layer_marker`：官方颜色体素。
- `/ur7e_nvblox/esdf_occupied`：红色 inside 体素。
- `/ur7e_nvblox/esdf_distance_band`：已观测距离带，inside 红，自由距离黄→青→蓝。
- `/ur7e_nvblox/camera_pose`、`camera_path`：机械臂图像时刻 TF 位姿和绿色路径。
- `/ur7e_nvblox/stats`、`topic_info`、`map_epoch`：统计、接口语义和地图代际。

没有恢复重复 slam_path，也没有添加旧名称的大点云并行别名。全部距离点云仍默认不创建。

v4 `stats.map_latest` 新增 `scheduler=official_ros_tick`、`configured_esdf_hz`、`completed_map_version`、`pending_gpu_measurements`、`dropped_gpu_measurements`、`last_mesh_snapshot_ms`。

### 5.1 更新频率与完成时间的含义

| 字段 | 含义 |
| --- | --- |
| rates_hz.depth_integrated | 官方函数接受并提交有效深度融合的频率 |
| rates_hz.esdf_updated | 提交新 ESDF 版本的频率 |
| rates_hz.depth_completed / esdf_completed | 被 CUDA 事件轮询确认完成的测量样本频率 |
| map_version | 已提交的最新 ESDF 版本 |
| completed_map_version | 事件确认完成的最新 ESDF 版本；reset 后清零 |
| exported_map_version | 实际读回并发布的 ESDF 版本 |
| latency.depth_process_submit / esdf_process_submit | 主机调用耗时，不强制等待 GPU |
| latency.depth_gpu_elapsed / esdf_gpu_elapsed | CUDA 流上起止事件间隔，可能包含提交间隙，不是单独内核耗时总和 |
| latency.depth_completion_observed / esdf_completion_observed | 主机从开始调用到轮询观察完成的时间，含轮询延迟 |
| latency.queue_seen_to_esdf_completion_observed | 队列首见到主机观察 ESDF 完成，含排队和轮询延迟 |
| latency.queue_seen_to_publish | 队列首见到实际点云发布，不含相机曝光/进入队列之前的时间 |

旧 `depth_process_complete`、`esdf_complete`、`queue_seen_to_esdf` 不再以同名输出，避免把新的异步测量冒充旧的同步完成延迟。不同版本的这些延迟不能直接逐项比较。

事件轮询发生在主循环/统计回调，默认 tick=10 ms；处理阻塞时还可能更久。因此 completion_observed 是观察完成时间，不能解释成精确 GPU 完成时刻。如果 dropped_gpu_measurements>0，completed 计数只是被保留的测量样本数，不能当作所有任务的完成吞吐。

## 6. 完整操作指令

以下供用户下一步运行。本轮未执行这些仿真指令；只执行了构建和第 7 节的离线协议检查。

### 6.1 构建

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./build_ur7e_native.sh
./run_ur7e_native.sh --check-build
```

### 6.2 默认参考型任务预算（终端 A）

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_ur7e_perception.sh gazebo_gui:=true
```

若旧场景仍在运行，先按所属终端 Ctrl+C 正常关闭；只关 Gazebo 窗口可能留下服务器。端口占用时查看 `ss -ltnp 'sport = :11457'`，确认实例再处理。已有场景也可用 `start_scene:=false`，但需单独核对其时钟配置。

### 6.3 保留 30 Hz ESDF 目标的配置

不要与上一个 mapper 同时启动，改用：

```bash
./run_ur7e_perception.sh gazebo_gui:=true \
  mapper_config:=/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo_fast_esdf.yaml
```

两配置仅 tick 周期、ESDF 更新目标、导出上限不同；体素、输入、加权、置信门槛和范围相同。官方限频使用 ROS 时间且为严格间隔判断，实际 Hz 不保证等于配置值；还受输入、/clock 和 GPU 影响。

### 6.4 机械臂采集视角（终端 B）

场景和感知已运行后，沿用之前的观察姿态：

```bash
cd /home/czz/perception_eye_in_hand
./run_ur7e_view_sweep.sh --check-only
./run_ur7e_view_sweep.sh --seconds 20
```

仅用于当前仿真。小幅摆腕不等于完整环绕采集，不保证所有侧面/背面可见；不要把未观测区域当作自由空间。

### 6.5 查看真实状态（终端 C）

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
source /opt/ros/humble/setup.bash
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox static_mapper.projective_integrator_weighting_mode
ros2 param get /ur7e_nvblox static_mapper.esdf_integrator_min_weight
ros2 param get /gazebo publish_rate
ros2 topic echo /ur7e_nvblox/stats --once
ros2 topic echo /ur7e_nvblox/topic_info --once --qos-durability transient_local
```

预期配置分别为 0.03、inverse_square_tsdf_distance_penalty、0.1、100.0；这只是运行后应核对的值，本轮未做实时查询。

以下 hz 分别执行，Ctrl+C 后换下一条：

```bash
ros2 topic hz /clock --wall-time
ros2 topic hz /esdf_input/depth --wall-time
ros2 topic hz /ur7e_nvblox/esdf_distance_band --wall-time
ros2 topic hz /ur7e_nvblox/mesh_markers --wall-time
```

也可直接运行项目监视器，不创建第二个 mapper：

```bash
PYTHONPATH=/home/czz/perception_eye_in_hand/src/ur7e_nvblox python3 -B -m ur7e_nvblox.monitor --seconds 30
```

监视器新增 ESDF_submit/done 区分。点云 hz 和监视器本身都会创建点云消费者并触发导出，因此“只显示 Mesh”和“同时测点云”的负载不同。

### 6.6 Mesh 恢复与几何保存

```bash
ros2 service call /ur7e_nvblox/republish_mesh std_srvs/srv/Trigger '{}'
source /home/czz/workspaces/isaac_ros-dev/install/setup.bash
ros2 service call /ur7e_nvblox/save_ply nvblox_msgs/srv/FilePath '{file_path: /tmp/ur7e_v4_mesh.ply}'
```

republish_mesh 需要标准 Mesh 消费者。保存 PLY 是官方按需更新/导出，会增加瞬时开销；不要把保存窗口混入正常性能基线。

## 7. 已执行的验证与没有执行的验证

已执行：场景包与原生包编译/链接/安装；构建新旧检查；host-only Mesh 协议程序。协议检查覆盖全局坐标不重复加原点、三角形索引与 RGB 对齐、ROI 块过滤、clear/delete、缺失颜色回退和非法消息拒绝。

协议程序没有 `rclcpp::init()`，没有 Mapper 或 CUDA 调用，也没有机器人 I/O。用户可独立复核：

```bash
/home/czz/perception_eye_in_hand/install_native/ur7e_nvblox_native/lib/ur7e_nvblox_native/mesh_protocol_test
```

构建与检查证据存于 [artifacts/esdf_official_alignment_v4_20261027](/home/czz/perception_eye_in_hand/artifacts/esdf_official_alignment_v4_20261027)，其中保留修改前主节点和参数、构建日志及协议检查日志。

没有执行：新版 CUDA 事件池的设备运行验收、Gazebo/ROS/RViz 启动、真实更新频率、延迟、平面误差与多视角覆盖测量、物理机器人测试。本轮没有生成伪造的“实时效果图”或把 5/30 Hz 配置值称为实测。

## 8. 怎样证明与参考达到等同效果

本版是围绕已发现问题落实的代码与结构优化，不代表仅凭编译就已证明性能等同。后续需要使用同一深度/RGB/TF 输入、同精度、同范围、同颜色开关、同显示消费者条件，对官方原生节点和本项目分别回放/采集。

默认 5 Hz 只与参考 5 Hz 预算比较；30 Hz 配置应与同样 ESDF 更新目标的参考比较。GPU 应串行测量，避免双实例争用。

主要验收项：

- 深度融合提交/完成、ESDF 提交/完成、不同源时间戳的点云接收频率。
- 明确起点的墙钟延迟 P50/P95/P99，不能把 GPU 事件间隔与端到端时间混用。
- 同一墙面 Mesh 面积采样后的平面残差、对 Gazebo 真值的偏差、双层厚度。
- 可见区域覆盖、孔洞、离群点、低可信体素和 ESDF 未知率。
- CPU/GPU 负载、分配块、Mesh 三角形数和消息流量。

如果原始输入与参考相同、TSDF 参数一致后，官方 Mesh 与本项目 Mesh 的几何仍不同，应沿输入校验/丢帧、位姿、时序和数据转换继续定位；如果原生 Mesh 一致而 RViz 不同，应查显示桥接/缓存。后续的自体过滤、目标中心多视角和深度滤波分别做单项实验，不在没有证据时一次性叠加所有策略。

## 9. 官方来源对应

| 本版使用的逻辑 | workspaces 参考位置 |
| --- | --- |
| 主处理顺序 | nvblox_ros/src/lib/nvblox_node.cpp，NvbloxNode::tick |
| 图像时间 TF 和深度/颜色融合 | 同文件 processDepthImage / processColorImage |
| 图层按需和 Mesh 增量更新 | nvblox_ros/src/lib/layer_publishing.cpp |
| 快照序列化与 Marker 转换 | nvblox_core 的 layer_streamers；nvblox_ros/src/lib/conversions/mesh_conversions.cpp |
| TSDF distance penalty | nvblox_core/nvblox/include/nvblox/integrators/internal/impl/weighting_function_impl.h |
| ESDF 权重与 site 语义 | nvblox_core/nvblox/src/integrators/esdf_integrator.cu |
| 参考默认调度和权重配置 | nvblox_examples_bringup/config/nvblox/nvblox_base.yaml |

更详细的原因分析保留在 [表面质量分析](/home/czz/perception_eye_in_hand/docs/ESDF_SURFACE_QUALITY_ANALYSIS_AND_OPTIMIZATION.md)。该分析里的 v3 参数对照是优化前证据；当前参数与操作以本文为准。
