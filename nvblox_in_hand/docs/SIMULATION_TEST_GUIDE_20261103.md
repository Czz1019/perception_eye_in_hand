# 仿真测试与效果采集操作说明

2026-11-04重构提示：本文保留此前结构/运行记录。当前默认已改为官方NvbloxNode组件，入口、配置及显示选择见 [重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md) 和 [最新操作说明](OPERATIONS.md)。此前inside-only默认、20字节点云及旧频率不能当作新版接口或结果；本轮未运行测试。

日期：2026-11-03。适用工作区：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

本手册核对了当前源码入口，提供可执行的操作流程。本次编写文档没有启动仿真、移动机械臂或产生新的实测成绩。测试采用之前的观察姿态与小幅腕部转动，不执行环绕扫描。

## 1. 测试目标与固定条件

验证机器人状态、RGB-D 输入、Mesh、三维 ESDF 和相机路径是否正常；检查相机运动后的表面重影、未知区域和更新延迟。

| 条件 | 本轮统一设置 |
| --- | --- |
| 体素精度 | 0.02 m |
| 相机深度输入 | 场景原始 1280×720、标称 30 Hz |
| 最大融合距离 | 3 m |
| 地图坐标系 | base_link |
| 建图方式 | static_tsdf、图像时刻 TF |
| 深度/颜色调度目标 | 30 / 5 Hz |
| ESDF/地图发布调度目标 | 5 / 5 Hz |
| 自定义三维 ESDF 显示上限 | 1 Hz，距离带 0.12 m |
| 运动 | 移到已有观察姿态，wrist_1 小幅往复 |

目标频率不等于实际频率。ESDF 显示频率、ESDF 内部调用周期和 GPU 完成频率是三个不同指标。

## 2. 启动前准备

在旧场景、旧 mapper、旧扫描和旧 RViz 的原终端按 Ctrl+C，避免重复启动同名机器人和控制器。只关闭 Gazebo 窗口可能仍留下 gzserver。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
ss -ltnp 'sport = :11457'
```

若端口仍在监听，确认对应旧进程并在原终端停止。不要无差别清除所有 ROS 进程。

初次使用或源码发生变化后构建：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./test.sh
```

默认只构建新工作区的场景包和感知适配包，复用 workspaces 中已安装的官方依赖。`./test.sh` 是离线检查，不会执行扫描。构建成功不代表仿真效果已经验收。

## 3. 终端 A：启动场景和感知

推荐把 RViz 分开启动，便于判断问题来源：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/robot/cyclonedds_local.xml
mkdir -p artifacts/test_20261103
set -o pipefail
./run.sh gazebo_gui:=true run_rviz:=false \
  mapper_backend:=observed_view esdf_mode:=3d \
  enable_esdf:=true enable_esdf_view:=true \
  voxel_size:=0.02 max_range_m:=3.0 \
  depth_hz:=30.0 color_hz:=5.0 esdf_hz:=5.0 mesh_hz:=5.0 layer_hz:=5.0 \
  2>&1 | tee artifacts/test_20261103/launch.log
```

保持此终端运行，确认 Gazebo 未暂停，机器人、相机和控制器已就绪。默认启动不会自动移动机械臂。

## 4. 终端 B：检查并打开 RViz

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 pkg prefix nvblox_in_hand
ros2 pkg prefix ur7e_d435_gazebo
ros2 pkg prefix nvblox_ros
./check_sim.sh --timeout 30
./rviz.sh
```

前两个包应来自本目录 install；默认 nvblox_ros 来自 workspaces 的安装目录。不要再 source 父工程 install。

状态检查应返回 passed，且没有发送运动命令。它检查关节、时钟和同一时间的 FK/TF 一致性，不等于已经验证真实成像时间与 Gazebo 传感器真值一致。

RViz 设置：

| 内容 | Displays 中选择 | 话题 |
| --- | --- | --- |
| 表面重建 | Official incremental RGB Mesh | /nvblox_node/mesh |
| 原始 RGB | D435 RGB input | /camera/color/image_raw |
| 三维 ESDF | Observed native ESDF 3D band (optional readback) | /nvblox_node/esdf_native_band |
| 相机轨迹 | Robot camera path (not SLAM) | /nvblox_in_hand/camera_path |
| 真实尺寸颜色体素 | Official RGB voxel cubes (Humble safe) | /nvblox_node/color_layer_marker |

Fixed Frame 使用 base_link。选择 Move Camera 工具，左键拖动旋转、中键平移、滚轮缩放；Views 可选 Whole obstacle group 或 Top overview。

先只开 Mesh 和 RGB，确认障碍物在相机视场内，再勾选三维 ESDF。该显示默认未勾选，只有有订阅者时才读回发布。它显示已观测的近表面距离带，内部红色，外部按距离绿到蓝；未显示的位置不能直接判断为 unknown。

不要在 3d 模式下等待 Official 2D ESDF slice。该二维点云在本地版本的 2d 模式才发布。切换 2d 是独立实验，不能把它当作同时显示完整三维场的选项。

## 5. 终端 C：使用之前的观察姿态和腕部转动

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
../run_ur7e_view_sweep.sh --check-only
```

检查成功后，先移动到观察姿态并停住：

```bash
../run_ur7e_view_sweep.sh --seconds 0
```

等待地图稳定后，再执行小幅往复转动：

```bash
../run_ur7e_view_sweep.sh --seconds 30 --amplitude 0.06
```

脚本使用现有 Gazebo 位置控制器，平滑移到已有观察姿态，然后 wrist_1 在 ±0.06 rad 内往复转动，结束后回观察姿态保持。30 秒是往复部分的墙钟时长，不包含到达姿态及回位时间。该路径复用父工程脚本，不需要启动父工程 mapper。

不要同时运行 orbit_scan、其他关节命令程序或实机控制程序。此脚本针对当前固定仿真场景，并非任意新障碍环境的通用避障规划器。Ctrl+C 会停止发布，控制器保留最后目标。

## 6. 终端 D：采集频率、配置和诊断

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
mkdir -p artifacts/test_20261103
ros2 topic list -t > artifacts/test_20261103/topics.txt
ros2 param dump /nvblox_node > artifacts/test_20261103/mapper_parameters.yaml
ros2 topic echo /nvblox_in_hand/pose_health --once
ros2 topic echo /nvblox_node/metrics | tee artifacts/test_20261103/metrics.log
```

记录静止、转动和停稳三个阶段，每阶段至少记录 30 秒；开始采集后再从终端 C 执行转动。Ctrl+C 结束记录。

| 字段 | 含义和边界 |
| --- | --- |
| depth_integrated_wall_hz | 通过融合时间推进统计的深度墙钟频率 |
| color_integrated_wall_hz | 颜色融合墙钟频率 |
| esdf_cycles_wall_hz | 原生 ESDF 方法调用周期频率，不证明每次都有变化块或 GPU 已完成 |
| esdf_process_host_ms | 最近一次 ESDF 调用主机耗时，不是端到端延迟 |
| esdf_view_host_ms | 最近一次显示序列化/读回耗时；不发布时可能保留旧值 |
| esdf_view_points | 最近一次显示点数；不是全地图体素数，也不是覆盖率 |
| gpu_completion_measured | 当前为 false，没有测量 GPU 完成延迟 |

需要观察传感器或发布消息频率时，分别执行，Ctrl+C 后再换下一条：

```bash
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /nvblox_node/mesh
ros2 topic hz /nvblox_node/esdf_native_band
```

CLI 频率受订阅、传输和本机工具负载影响；订阅显示话题还会主动触发显示工作。性能基线阶段不要额外订阅三维 ESDF，优先使用 metrics 的墙钟频率，并记录 Gazebo 实时率。控制台 delays 是图像 ROS 时间戳到特定处理点的统计，不能冒充屏幕或 GPU 完成延迟。

## 7. 保存效果图和原生结果

使用系统截图工具 PrtSc，保存到 `/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/test_20261103/`。截图不是本手册预先生成的仿真结果，需要实际运行后采集。

建议保存：

1. `01_rgb_view.png`：相机确实看到目标的原始 RGB。
2. `02_mesh_static.png`：运动前静态 Mesh。
3. `03_mesh_moving.png`：运动中同一观察方向的 Mesh。
4. `04_esdf_moving.png`：运动中三维 ESDF 距离带。
5. `05_esdf_stopped.png`：停稳后同一视角，便于判断残影是否持续。
6. `06_metrics.png`：对应阶段的统计，保留测试条件与时间。

Mesh 和 ESDF 分别截图时使用相同 RViz 视角、缩放与工作范围。必要时用系统录屏保存转动全过程，静态截图不能体现更新频率。

测试完成后导出，避免把磁盘写入开销混入性能采样窗口：

```bash
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/test_20261103/scene_map.nvblx'}"
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/test_20261103/scene_mesh'}"
ros2 service call /nvblox_node/save_rates nvblox_msgs/srv/FilePath \
  "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/test_20261103/rates.txt'}"
ros2 service call /nvblox_node/save_timings nvblox_msgs/srv/FilePath \
  "{file_path: '/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/test_20261103/timings.txt'}"
```

各服务以 success 返回值及实际落盘文件为准。PLY 可能输出文件组合，不要假设传入前缀就是唯一成品文件名。

## 8. 公平比较实时性的方法

每组重启场景与 mapper，从空地图、相同初始姿态开始；固定上述精度、输入、范围和腕部轨迹。建议每组重复三次，分别记录启动预热、运动和稳定阶段，不能将运行几十秒的图与累计多分钟的大图直接比较。

| 组别 | 设置 | 定位目的 |
| --- | --- | --- |
| A | RViz 开启，关闭三维 ESDF 显示，保留 Mesh | 基础重建显示负载 |
| B | 同 A，开启三维 ESDF 显示 | 估算附加同步读回和 Marker 绘制代价 |
| C | 不启动 RViz，关闭附加 ESDF 显示 | 观察去掉可视化后的处理频率；也会失去部分按订阅触发的工作，不是纯显示差值 |
| D | official 后端，保持相同参数与 Mesh 订阅 | 核对薄适配的额外开销；该后端没有 metrics 和自定义距离带 |

C 组启动命令，先停止上一组：

```bash
./run.sh gazebo_gui:=true run_rviz:=false \
  mapper_backend:=observed_view enable_esdf_view:=false \
  voxel_size:=0.02 max_range_m:=3.0 \
  depth_hz:=30 color_hz:=5 esdf_hz:=5 mesh_hz:=5 layer_hz:=5
```

D 组启动命令，先停止上一组：

```bash
./run.sh gazebo_gui:=true run_rviz:=true mapper_backend:=official \
  voxel_size:=0.02 max_range_m:=3.0 \
  depth_hz:=30 color_hz:=5 esdf_hz:=5 mesh_hz:=5 layer_hz:=5
```

与 workspaces 比较时还需固定相机噪声、预处理、TSDF 截断和权重等参数，并尽量使用同一录制数据。只看到官方示例画面更流畅，不足以证明相同负载下算法更快。对比几何时核对内外参，对比吞吐时区分 ROS 时间和墙钟时间。

## 9. 常见故障

| 现象 | 检查与处理 |
| --- | --- |
| Gazebo master 已占用 | 停止旧场景，不重复启动相同端口 |
| 只有 parameter_events 和 rosout | mapper/场景未启动，或域号、DDS 配置不一致；先看终端 A 报错 |
| ESDF 没有显示 | observed_view 后端、3d 模式、enable_esdf/view 开启，RViz 勾选正确 Marker，Fixed Frame=base_link |
| ESDF 有发布但点数为零 | 看融合是否推进、有效深度是否在 3 m 内、TSDF 权重；不要只扩大显示点尺寸 |
| official 后端没有 metrics | 设计如此，使用其控制台统计及 save_rates/save_timings |
| RGB 有图但 Mesh 没颜色 | 开启 enable_color，核对颜色图、CameraInfo、光学 TF；不要将 TSDF 距离色当 RGB |
| RViz 闪退 | 用本目录 env.sh/rviz.sh 加载兼容材质；检查缺失 shader 日志，不从父工程混载环境 |
| 机器人显示姿态不对 | 停止运动后运行 check_sim；检查重复 joint_states/robot_description 和 TF 来源 |
| 运动时残影长尾 | 分别比较单帧、TSDF/Mesh、原生 ESDF 和显示；排查成像时间、外参、自身入图和历史表面 |
| 墙角空白 | 先区分未知与距离带过滤，再检查有效深度和可见性；不能直接填自由空间 |

更深入的原因见 [ESDF 未知与残影分析](/home/czz/perception_eye_in_hand/nvblox_in_hand/docs/ESDF_UNKNOWN_AND_MOTION_GHOST_ANALYSIS.md)。

## 10. 测试记录模板

| 项目 | 实测填写 |
| --- | --- |
| 日期、组别、代码版本、运行命令 | 待测试 |
| 输入分辨率、体素、范围、噪声配置 | 待核对 |
| Gazebo 实时率、墙钟运动时长 | 待记录 |
| 深度融合/颜色融合/ESDF 周期 Hz | 待记录 |
| 显示读回耗时、显示点数、订阅组合 | 待记录 |
| 静止与运动后表面差异、残影持续时间 | 待记录 |
| 墙角是否有有效深度及 observed 状态 | 待记录 |
| 截图、日志、原生地图位置 | 待记录 |

此前历史日志和离线验证不能填作本轮测试结果。当前 metrics 不足以给出严格 GPU 完成或端到端延迟，需要后续增加相应测量点。
