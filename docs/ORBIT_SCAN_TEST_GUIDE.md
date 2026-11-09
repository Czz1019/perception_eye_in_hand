# UR7e 障碍物组扫描：启动失败分析与完整仿真测试步骤

更新日期：2026-10-28。适用目录：`/home/czz/perception_eye_in_hand`，ROS 2 Humble、本项目 Gazebo 固定场景、原生 C++ 感知接口 v4。

## 1. 此次报错是什么意思

报错发生在 `orbit_scan.py` 的输入等待阶段，旧版等待 25 秒后把多项依赖统一报告为：

```text
Need running project Gazebo, robot_description, depth CameraInfo and native mapper stats
```

它表示“至少有一项条件未满足”，并不表示每一项都缺少。用户此次运行留下的 `artifacts/orbit_scan_20261027/last_run.json` 为 `failed`、`captures: []`；结合报错位置，可以确定程序没有进入扫描运动阶段。旧报告没有逐项记录，无法据此确定是哪一项输入缺失。

需要先启动完整感知，再运行扫描。`run_ur7e_orbit_scan.sh` 是运动与采集协调程序，不会自动启动 Gazebo、相机和 mapper。单独运行它，或仅启动 Gazebo 而未启动感知，都可能触发这条错误。

代码检查还发现一个通信隐患：修改前，扫描入口默认 `ROS_DOMAIN_ID=127`，统一感知入口没有设置默认域；后者在普通新终端中通常使用域 0。即使两边都在运行，也会互相看不到。这是可确认的配置差异，但旧报错没有证据证明它就是此次唯一原因。

### 本轮修正

| 修改位置 | 修改内容 | 解决的问题 |
| --- | --- | --- |
| 根目录 `run_ur7e_perception.sh` | 未显式设置时默认域 127、本机通信、Gazebo master 11457，与扫描入口一致 | 新终端启动默认值不同，导致输入不可见 |
| `src/ur7e_d435_gazebo/scripts/orbit_scan.py` | 每 5 秒列出缺失项，记录通信环境和逐项检查结果 | 不再只得到笼统错误 |
| 同上 | 新增 `--check-only`、`--wait-timeout`，默认等待 60 秒 | 可先做无运动预检查，允许正常启动等待 |
| 同上 | 检查相机信息和原生统计的新鲜度及统计来源，忽略非法 JSON | 避免把过期输入或不兼容统计当作就绪 |
| 同上 | 预检查写 `last_check.json`，正常扫描写 `last_run.json`，记录实际发送命令数量 | 诊断不会覆盖已有扫描报告；失败与实际采集可区分 |

显式设置的环境值仍优先。若原终端已经设置 `ROS_DOMAIN_ID=0`，新的默认值不会覆盖它，需要按下面指令明确设置 127。已经运行的旧进程不会随脚本更新自动更换域。

本轮没有修改 TSDF/ESDF 算法、体素尺寸、相机分辨率或扫描轨迹算法。启动配置和可诊断性修正不能当作重建性能提升或扫描成功的证据。

## 2. 正确启动顺序

```text
终端 A：完整感知入口
  ├─ Gazebo + 机器人/控制器/固定障碍物
  ├─ robot_state_publisher + joint_states + 相机 RGB-D
  ├─ 深度输入适配
  ├─ 原生 C++ mapper → Mesh / ESDF / 实际相机路径 / stats
  └─ RViz
终端 B：--check-only → 成功后执行 orbit_scan
终端 C：话题频率、统计、图像与 TF 检查
```

不要先单独启动 `run_ur7e_d435_scene.sh`，随后又用默认参数启动完整感知入口；这会重复启动 Gazebo。不要同时运行旧的 view sweep 或其他关节运动程序。

## 3. 构建（更新后执行一次）

在项目目录执行，构建结束后再启动：

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
./build_ur7e_native.sh
```

两个命令均应成功。若第一个失败，不应继续扫描；扫描碰撞检查器属于场景包。若第二个失败，感知启动入口会拒绝启动过期的 native 二进制。不要通过绕过构建检查运行旧程序。

依赖仍引用 `/home/czz/workspaces/isaac_ros-dev` 已有官方 ROS 移植库；本轮修正在 perception 项目中。无需为了此次问题重新修改 workspaces。

仅本轮 Python 诊断和根入口脚本变化不要求重新编译 C++；上述两步用于此前尚未成功构建或更新了其他源码的情况。

## 4. 终端 A：启动完整仿真与感知

先用原启动终端的 Ctrl+C 正常停止上一套项目仿真；然后在终端 A 执行：

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_ur7e_perception.sh gazebo_gui:=true
```

保持终端 A 运行，不要 Ctrl+C 后才去扫描。该命令默认同时启动场景、输入适配、mapper 和 RViz。Gazebo 必须处于播放状态，仿真时间应持续推进。启动界面出现并不等于所有节点已就绪。

如果没有启动 RViz 的需求，可使用同一入口加 `run_rviz:=false`；视觉验收仍建议打开 RViz。

### Gazebo master 已被占用

```bash
ss -ltnp 'sport = :11457'
```

先确认是否是之前的项目 `gzserver`。关闭 Gazebo 图形窗口不一定停止服务端。优先在原启动终端 Ctrl+C；必要时确认具体进程身份后对该 PID 使用普通 `kill PID`，不要无差别终止所有 Gazebo/ROS 进程。

若要复用**同一域、相同配置和默认障碍物**的已有本项目场景，可以用下面命令仅启动输入、感知和显示：

```bash
./run_ur7e_perception.sh start_scene:=false
```

这是上面完整启动命令的替代方式，不能两者同时运行。复用时还需确认 `/clock`、控制器、相机、URDF 和 Gazebo master 一致；不能用它跳过缺失场景问题。

## 5. 终端 B：先做无运动预检查

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_ur7e_orbit_scan.sh --check-only --wait-timeout 60
```

成功时出现 `"phase": "ready"`、`"motion_commands_sent": 0`。预检查不会规划新轨迹、创建关节命令发布者或移动机械臂；它会检查真实输入、相机视场、活动控制器及 Gazebo 模型位置和尺寸。Gazebo CLI 检查有独立超时，整个程序的耗时不只由输入等待参数决定。

输入检查内容：

| 条件 | 通过标准 | 常见缺失原因 |
| --- | --- | --- |
| `/joint_states` | 六个预期关节都有有限数值，最近 1 秒内更新 | 控制器未加载、机器人未生成、域不同 |
| `/clock (advancing)` | 至少两次时间前进，最近 1 秒内仍推进 | Gazebo 未启动、暂停、负载过高或域不同 |
| `/robot_description (Gazebo URDF)` | 已接收包含 gazebo_ros2_control 的机器人描述 | robot_state_publisher 未运行、错误 URDF、通信域不同 |
| `/camera/depth/camera_info` | 最近 3 秒收到相机信息；随后检查光学坐标系和视场 | 相机插件未工作、话题错误、暂停、域不同 |
| `/ur7e_nvblox/stats (native, fresh)` | 最近 3 秒收到合法原生统计，backend 为 official_ros_cpp，包含 totals | mapper 未启动/退出、只启动了场景、误用了旧 Python mapper |

后续还检查 `/controller_manager/list_controllers` 中 `forward_position_controller` 为 active 且类型正确；使用 `gz model` 核对机器人 `ur`、`sage_clean_cube1`、`sage_clean_cube2`、`sage_clean_sphere1` 的参考布局。场景被手工移动、缩放或使用其他模型时，不应强制绕过校验。

报告查看：

```bash
cat /home/czz/perception_eye_in_hand/artifacts/orbit_scan_20261027/last_check.json
```

`ready` 表示前置条件通过，不保证路径可达、深度图有效、已融合足够数据或重建质量达标。第 7 节仍需确认融合计数增长。

## 6. 终端 B：执行多视角扫描

仅在终端 A 持续运行、预检查成功且没有其他运动程序时执行：

```bash
cd /home/czz/perception_eye_in_hand
./run_ur7e_orbit_scan.sh
```

可选命令（各自独立使用）：

```bash
# 从当前关节姿态重新搜索；每个观察点至少停留 4 秒
./run_ur7e_orbit_scan.sh --replan --dwell 4

# 只生成离线计划，不创建 ROS 节点、不执行运动
./run_ur7e_orbit_scan.sh --plan-only

# 要求所有请求的方位与高度组合均有可用解；缺少则拒绝运动
./run_ur7e_orbit_scan.sh --require-full-orbit
```

扫描会重新验证缓存路径和当前姿态到起点的连接，然后才创建运动发布者。规划、路径验证阶段没有运动是正常现象。三个高度为 0.35、0.60、0.80 m，默认每层请求 6 个方位。当前默认场景的历史离线结果是 10/18 个观察点、约 240° 方位跨度，不能承诺完整 360° 或所有物体表面都重建。正常执行也可能因当前状态不同而重新规划、拒绝执行或中止。

观察点采用机械臂真实手眼关系，使相机 optical +Z 朝向障碍物组；转移过程不保证始终盯住目标。停稳后至少等待设定停留时间及 5 次新增深度融合。Ctrl+C 停止后续指令，控制器保留最后目标，不自动返回初始姿态。

扫描报告：

```bash
cat /home/czz/perception_eye_in_hand/artifacts/orbit_scan_20261027/last_run.json
```

检查 `result`、`motion_commands_sent`、`captures`、`integrated_frames`、`dwell_wall_seconds`。失败也会生成报告；文件存在本身不代表执行成功。`measured_surface_coverage: null` 表示没有测量完整表面覆盖率。

## 7. 终端 C：确认输入、实时频率和延迟

在每个新的监控终端先执行：

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source /opt/ros/humble/setup.bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
```

不需要 source 整个项目旧的 `install/setup.bash`。它可能引用未构建的历史包，造成无关的 local_setup.bash 缺失提示。对标准消息、图像、统计和 Mesh Marker 的查询，source Humble 即可；查询原生 `nvblox_msgs/Mesh` 自定义消息时额外需要官方依赖环境。

以下命令逐条执行；`timeout` 限定等待时间，避免不存在的话题一直等待。连续频率命令采样结束被 timeout 终止属于正常行为。

```bash
# 仿真时钟和机器人反馈（应持续收到）
timeout 12s ros2 topic hz /clock --wall-time
timeout 12s ros2 topic hz /joint_states --wall-time

# 相机标定与机器人描述：描述使用 Transient Local
ros2 topic info /camera/depth/camera_info --verbose
timeout 10s ros2 topic echo /camera/depth/camera_info --once --qos-reliability best_effort
timeout 10s ros2 topic echo /robot_description --once --qos-durability transient_local

# 先确认输入，再看输出
timeout 15s ros2 topic hz /camera/depth/image_rect_raw --wall-time
timeout 15s ros2 topic hz /esdf_input/depth --wall-time
timeout 15s ros2 topic hz /camera/color/image_raw --wall-time

# 最新原生统计；隔几秒再运行一次，确认 totals 中融合计数增长
timeout 10s ros2 topic echo /ur7e_nvblox/stats --once

# ESDF 话题采样（本版不使用旧 esdf_3d 名称）
timeout 20s ros2 topic hz /ur7e_nvblox/esdf_distance_band --wall-time
timeout 20s ros2 topic hz /ur7e_nvblox/esdf_occupied --wall-time
```

统计 JSON 的 `rates_hz` 使用墙钟窗口，可记录 `depth_integrated`、`depth_completed`、`color_integrated`、`esdf_updated`、`esdf_completed`、`esdf_exported`。`*_integrated/updated` 是提交阶段；`*_completed` 才是 CUDA 事件确认完成的计数。`map_latest.completed_map_version` 应随地图更新推进，不能只看 stats 自己的发布频率。

`latency` 中查看 `depth_gpu_elapsed`、`esdf_gpu_elapsed`、`depth_completion_observed`、`esdf_completion_observed`、`queue_seen_to_esdf_completion_observed` 的 mean/p95 等。GPU elapsed 是 CUDA 流事件时间；completion observed 包含主机轮询等待；queue_seen 是原生队列观测时刻，不等于相机曝光时刻，因此不能把它直接称为传感器到屏幕总延迟。没有样本时字段可能尚未出现。

默认配置是 **3 cm 体素、深度 640×360、RGB 1280×720、ESDF 目标 5 Hz、地图层目标 5 Hz**；深度源目标 30 Hz。配置频率不是实测承诺；较低仿真实时率、GPU 负载、话题订阅和处理耗时都会影响墙钟频率。深度处理配置的 1000 Hz 上限用于接收源帧，不表示传感器为 1000 Hz。使用 `--wall-time` 时，仿真时间与实际时间的比例也会影响观测值。

只做同精度高频对比时，先停止终端 A，使用同样环境重新启动：

```bash
./run_ur7e_perception.sh gazebo_gui:=true mapper_config:=/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/gazebo_fast_esdf.yaml
```

这是另一轮测试，不能与默认轮同时运行。该配置保持体素、分辨率与导出范围不变，将 ESDF/导出目标提高到 30 Hz；是否达到需要实测。每轮应从新地图开始、使用相同输入和扫描条件、等待稳定后采样，分别保存统计与画面，不把话题 hz 当作算法运行时长。

## 8. 实时画面与话题选择

RViz Fixed Frame 设为 `base_link`，加载本项目配置：

`/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/rviz/native.rviz`

鼠标需位于三维视图区，选择工具栏 Move Camera（快捷键 M），左键拖动旋转、中键平移、滚轮缩放。Views 中选择 Whole obstacle group 或 Top overview，避免只盯着原始相机中间区域。

| 话题 | RViz 类型 | 内容及颜色 |
| --- | --- | --- |
| `/ur7e_nvblox/mesh_markers` | MarkerArray | 融合表面，真实相机 RGB；建议首先看这个 |
| `/ur7e_nvblox/color_layer_marker` | Marker | 原生颜色体素层，RGB；需要有效彩色输入与 TF |
| `/ur7e_nvblox/esdf_occupied` | PointCloud2 | 已观测内部体素，红色 |
| `/ur7e_nvblox/esdf_distance_band` | PointCloud2 | 近表面距离带，内部红色、自由侧黄到青蓝 |
| `/ur7e_nvblox/camera_path` | Path | 图像时刻机械臂 TF 得到的实际相机路径，绿色 |
| `/ur7e_scan/planned_camera_path` | Path | 扫描程序发布的计划相机路径，青色 |
| `/ur7e_scan/view_markers` | MarkerArray | 计划观察方向，青色箭头 |
| `/camera/color/image_raw` | Image | 原始彩色视场，用于确认相机真正看到了什么 |
| `/ur7e_scan/status` | String（命令行 echo） | 输入等待、检查、规划、采集、结束或失败 |

路径/扫描标记使用 Reliable + Transient Local。扫描程序结束后，新订阅者不保证收到已退出进程的保留数据。实际相机路径由 mapper 持续提供，长度有上限；它是机械臂定位路径，不是视觉 SLAM。当前没有重复的 `/slam_path` 和 `/esdf_3d` 别名。

`/ur7e_nvblox/mesh` 是官方增量 nvblox_msgs/Mesh，不是普通 PointCloud2；标准 RViz 优先看 `/mesh_markers`，无需安装自定义 Mesh 显示插件。地图层和导出按订阅需求发布，测试对应话题的频率时应保持对应显示/订阅开启。

若彩色图像有内容而 Mesh 无颜色，查看 stats 的 `color_integrated`、`map_latest.color_state`、`color_encoding`、`color_age_ros_sec` 和 `color_integration_size`；不是把 ESDF 距离颜色误当成表面 RGB。空白区域可能是未观测/遮挡区域，不能用固定点阵补齐来伪造覆盖。

## 9. 按报错定位

| 现象 | 优先排查与处理 |
| --- | --- |
| 所有输入都缺失 | 终端 A 是否还活着；域是否都为 127；是否只执行了扫描；RMW/本机通信配置是否一致 |
| 只有 stats 缺失 | 终端 A 的 native 构建检查、启动日志、CUDA/依赖加载错误；仅有 Gazebo 不够 |
| clock 不推进 | Gazebo 是否暂停或 server 已退出；检查负载；不要通过删除该检查继续执行 |
| joint_states 缺失 | 机器人生成和控制器 spawner 日志；`/controller_manager/list_controllers` 服务是否存在 |
| CameraInfo 缺失 | 相机插件、话题名、仿真播放状态；检查原始深度图是否存在 |
| URDF 缺失 | robot_state_publisher 是否运行；查询时是否使用 Transient Local；是否在相同域 |
| controller not active | 控制器未加载/激活或启动失败，先检查终端 A 日志，不向错误控制器发指令 |
| Cannot verify Gazebo model | GAZEBO_MASTER_URI 是否与终端 A 一致，模型名/生成是否成功 |
| 模型 differs / 尺寸无法核对 | 场景已变更或不是默认场景；恢复参考场景或重新设计适用方案，不能只用 --replan 绕过 |
| Camera frame/FOV mismatch | 使用了不同相机配置/坐标系，需要核对输入及方案适用范围 |
| No sufficient new depth integration | 查看有效深度、图像时刻 TF、队列/过期帧计数及 stats 增长；不是只增加停留时间就能解决 |
| Another joint command publisher | 停止其他扫描/伺服程序；不要运行两套位置发布者 |

只延长 `--wait-timeout 120` 适合“确实仍在正常启动”的情况，无法修复域不一致、未启动 mapper 或插件失败。

## 10. 验收与本轮验证边界

建议按以下顺序验收：预检查 ready 且零运动命令 → 静止时深度/彩色融合计数增长 → 扫描出现实际 captures → 绿色实际路径覆盖多个观察方向 → Mesh 和近表面 ESDF 随新观测变化 → 保存频率、延迟和同场景画面。

本轮只进行了代码检查、离线诊断回归测试和语法/命令帮助验证，没有启动仿真、打开 RViz 或执行机械臂运动，没有测得新的重建效果、更新频率或端到端延迟。历史离线可达性结果不能代替本次运行验收。

7 项离线测试已通过（原有 5 项 FK/FCL 检查及新增 2 项输入诊断检查）。记录见 [测试日志](/home/czz/perception_eye_in_hand/artifacts/orbit_preflight_20261028/offline_tests.log) 和 [验证范围](/home/czz/perception_eye_in_hand/artifacts/orbit_preflight_20261028/verification.json)。
