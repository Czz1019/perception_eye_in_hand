# RViz 鼠标交互修复与障碍物组多视角扫描

日期：2026-10-28。修改仅在 perception_eye_in_hand。保留当前原生 nvblox 感知、体素精度、手眼配置和场景；不修改 workspaces。

遇到 `Need running project Gazebo...` 或第一次测试时，请先阅读[完整启动、无运动预检查与故障排查](/home/czz/perception_eye_in_hand/docs/ORBIT_SCAN_TEST_GUIDE.md)。扫描入口不会启动感知；必须先保持统一感知入口运行，再执行 `./run_ur7e_orbit_scan.sh --check-only`。

## 1. 为什么拖动鼠标不能改变 RViz 视角

此前项目的 `native.rviz` 配置虽然包含 Orbit View，却缺少 `Visualization Manager → Tools` 列表。视图控制器定义相机怎么运动，鼠标工具负责把拖动事件交给相机；只设置 Orbit 不足以保证加载 Move Camera 工具。

本次对照 workspaces 的 RViz 配置及本机 Humble 插件声明，补充：

- 默认首个工具为 `rviz_default_plugins/MoveCamera`。
- Interact、Select、FocusCamera、Measure 工具。
- Views 和 Tool Properties 面板。
- TF 变换插件，以及固定到 base_link 的 Orbit 目标。
- “Whole obstacle group”和“Top overview”两个保存视图。

默认焦点调整到障碍物组附近 `[0.4, 0.075, 0.3]`，观看距离 2.4 m。它只改变观察镜头，不移动机械臂，不修改地图，也不会让未观测区域凭空出现。

### 已打开窗口的立即操作

如果窗口已经有工具栏，点击 **Move Camera**，或尝试按 **M** 切换到该工具，然后在三维画面中按住左键拖动。滚轮缩放，中键拖动平移。鼠标要放在三维渲染区域，而不是左侧属性树或 RGB 图像面板上。

如果连 Move Camera 都没有，需要加载修复后的配置：

1. RViz 菜单 `File → Open Config`。
2. 打开 `/home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/rviz/native.rviz`。
3. 点击 Move Camera，在 Views 面板选择 Orbit；需要整体画面时，选择保存的 Whole obstacle group 或 Top overview。

也可以只重开 RViz，不重启感知和 Gazebo。在与当前应用相同的 ROS 域和 RMW 环境里：

```bash
cd /home/czz/perception_eye_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/src/ur7e_nvblox_native/config/cyclonedds_local.xml
./run_ur7e_native_rviz.sh
```

这会另开一个 RViz；可以先正常关闭旧 RViz。修复不依赖 NVIDIA 自定义 Mesh 插件，仍使用标准 MarkerArray 显示真实 RGB Mesh。

## 2. 新增机械臂整体观察指令

旧 `run_ur7e_view_sweep.sh` 只围绕一个姿态小幅摆腕，不能替代围绕目标的多视角采集。现在新增：

```bash
cd /home/czz/perception_eye_in_hand
./run_ur7e_orbit_scan.sh
```

前提是当前项目的 Gazebo 场景和原生感知已启动。该指令会从当前关节位置接入经过检查的扫描路线，依次到达不同方向、不同高度的观察姿态，停稳后等待新的深度融合，最后保留地图并停在最后一个观察姿态。

**这是当前固定仿真场景的可达多视角扫描，不承诺 UR7e 能绕整个障碍物组完成无缺口的 360° 运动，更不等于所有物体表面都已重建。** 不可达、碰撞受限或限定搜索预算内找不到连接的视角会跳过并报告。`--require-full-orbit` 可以要求所有请求的方位×高度组合都找到解，否则在运动前拒绝执行。

### 第一次使用先构建

```bash
cd /home/czz/perception_eye_in_hand
./build_ur7e_d435_scene.sh
```

新增的离线 FCL 碰撞工具由场景包构建。RViz 配置由根目录脚本直接从源码读取，不需要为鼠标修复重新编译 CUDA mapper。若要更新原生包安装目录中的 RViz 副本，可额外执行 `./build_ur7e_native.sh`。

### 只生成规划，不移动机械臂

```bash
./run_ur7e_orbit_scan.sh --plan-only
```

离线模式使用本项目默认 URDF、手眼配置、初始关节和固定障碍物配置，不启动 ROS 节点、Gazebo 或 GPU。规划文件默认保存到：

`/home/czz/perception_eye_in_hand/artifacts/orbit_scan_20261027/offline_plan.json`

此目录名沿用开始实现时的日期。文件是计划，不是运动执行证明。离线初始位置与当前机械臂不同没有关系：执行时会读取真实关节状态，另行检查连接；连接失败则不执行。

本轮默认场景的最终离线结果：**18 个请求组合中找到 10 个观察点，覆盖三个高度，方位跨度约 240°**。约 37° 方位附近未找到合适组合，其他方向也存在高度限制；不能按“完整 360°”描述这一结果。计划包含 21 个关节路点，按当前速度和默认停留时间推算，路线运动与停留约 7.4 分钟，另需加当前姿态接入、场景核对和路径复核时间；这不是实测执行时长。

### 常用指令

```bash
# 执行已保存的多视角计划；不存在或版本条件不匹配时会重新设计
./run_ur7e_orbit_scan.sh

# 从当前关节位置重新设计，再执行；每个观察点至少停留 4 秒
./run_ur7e_orbit_scan.sh --replan --dwell 4

# 每个高度增加到 8 个方位候选，搜索和扫描时间也会增加
./run_ur7e_orbit_scan.sh --replan --views 8

# 严格要求所有请求的方位×高度组合都找到解；有缺失时不发送运动命令
./run_ur7e_orbit_scan.sh --require-full-orbit
```

不要与旧的 view_sweep、性能测试运动或其他关节控制程序同时运行。程序会检测相同命令话题的其他发布者。Ctrl+C 会停止发送后续位置指令；位置控制器保留最后目标，不会自动回到起点。

默认 Gazebo master 为 `http://127.0.0.1:11457`。若当前场景使用另一个地址，应先设置相同的 GAZEBO_MASTER_URI；ROS 域、RMW 也须与场景一致。不能通过更改这些值把该脚本用于实机。

## 3. 怎么围绕目标设计视角

### 3.1 目标不是一个固定的中间点姿态

从当前三个障碍物的配置计算整体包围范围，忽略地下部分后，目标中心约为 `[0.4, 0.075, 0.25]` m。

默认每一圈 6 个方位候选，在 **0.35、0.60、0.80 m** 三个相机高度上设计观察点，共请求 18 个“方位×高度”组合。候选半径包括 0.35–0.75 m；结合视场、IK、碰撞与转移路线选择可用位置。

相机使用 optical +Z 指向目标区域，姿态来自当前 URDF 中真实的 tool0→camera_depth_optical_frame 变换，不把 tool0 的朝向当作相机朝向。IK 可以使用不同肘部构型初始化，但任何 seed 都不是直接发送的关节指令。

必要时允许相机绕 optical +Z 翻滚 180° 以寻找其他可行构型，观察方向仍指向目标。此时原始图像可能倒置，TF 融合使用真实姿态；这不是通过修改图像内容伪造朝向。

近处观察点可能只覆盖部分包围盒，较远和较高观察点用于整体覆盖。这正是多视角组合的作用。计划里的 `framed_bbox_corners` 仅表示包围盒角点落在保守视场中，**不是可见表面覆盖率**；没有把被遮挡的背面算作已经看到。

### 3.2 可达和转移检查

- 从当前真实 URDF 读取六个关节、硬/软限位、相机固定链与碰撞几何。
- 使用现有项目的 Gazebo mesh loader 和 FCL，加载实际碰撞 STL/box，不套用旧场景的一组关节角。
- 检查机器人与两个墙体、球体、地面的间隙；检查非相邻连杆自碰撞，普通相邻连杆按连接关系排除。相机与相邻腕部仍单独检查，固定安装在同一刚体的部分除外。
- 先尝试关节直线连接；失败时进行有时间预算的关节空间搜索。
- 也会尝试高位展开姿态作为构型切换的过渡，每个过渡点和每段路径均重新做碰撞检查，不直接盲目执行固定姿态。
- 路径检查对中点 FCL 距离减去运动界，必要时细分，不能仅检查观察点端点。
- 执行时对已缓存路径用当前模型重新检查，并核对各捕获点的相机 FK 是否匹配。

为了避免过度保守的全局球包络把本来无碰撞的 UR 臂姿态全部拒绝，最终检查使用实际碰撞网格。几何包围界只用于路径间隙的保守运动界，不代替碰撞几何。

规划仍是有限搜索，某个方向没找到解不等于已数学证明它绝对不可达。用户如果要求完整一圈，应使用严格选项查看是否满足，而不是强行忽略碰撞或把未找到解的区域补成地图。

### 3.3 执行前和执行中

执行模式限定 ROS_DOMAIN_ID=127，要求：推进中的 `/clock`、新鲜 `/joint_states`、含 gazebo_ros2_control 的机器人描述、当前项目的活动位置控制器，以及正在更新的原生感知统计。

还会用只读 `gz model` 查询核对 `ur` 的世界姿态和三个 `sage_clean_*` 障碍物的位置、朝向及尺寸。被用户移动/缩放的障碍物、不同相机视场、不同手眼或不同场景会导致拒绝执行或要求重规划。

只检查本项目这套已知固定场景，不是对任意新增物体的通用在线避障。它不连接实机，也不作为实机运动程序。世界真值仅用于扫描运动检查，不会写入 nvblox 或生成伪造的已观测体素。

执行采用平滑关节插值，指令速度上限约 0.12 rad/s，监测实际关节反馈和碰撞间隙。每个观察点默认至少停留 2.5 秒，并等待至少 5 次新的深度融合；无有效感知更新或反馈超时会中止。

转移过程使用关节空间连接，不保证相机在两个观察点之间始终盯住目标；规定朝向在观察点处满足。感知会继续更新，不自动清空地图。

## 4. 整体视角、颜色与话题

| 话题 | 内容 | 颜色/说明 |
| --- | --- | --- |
| /ur7e_scan/planned_camera_path | 已检查关节路线经 FK 采样得到的相机路径，nav_msgs/Path | 青色；不是实际执行记录 |
| /ur7e_scan/view_markers | 规划观察方向，MarkerArray | 青色箭头 |
| /ur7e_scan/status | 规划/检查/运动/采集/完成状态，JSON String | 包括部分覆盖和失败原因 |
| /ur7e_nvblox/camera_path | 实际接受深度帧对应的机械臂 TF 相机路径 | 绿色 |
| /ur7e_nvblox/mesh_markers | 原生融合后得到的表面 Mesh | 真实相机 RGB |

前三个话题由扫描进程在运行期间提供，采用 Reliable + Transient Local。进程退出后，不保证新打开的 RViz 还能收到其保留消息；已经收到的 RViz 会保留画面。实际相机路径和重建地图仍由 mapper 提供。

实际绿色轨迹受 mapper 的有限长度限制，长时间扫描可能截去早期路径点；青色计划路径包含整条计划路线。扫描尚未启动时，新增扫描话题可能没有发布者，这是可选扫描组件的正常状态。

程序执行后的 `last_run.json` 会记录实际捕获点、接收的融合帧数和停留时间。运行后可以查看：

```bash
cat /home/czz/perception_eye_in_hand/artifacts/orbit_scan_20261027/last_run.json
```

执行模式失败时也会生成该报告；检查 `result` 和 `captures`，不能用文件存在判断扫描完成。2026-10-28 新增的 `--check-only` 另写 `last_check.json`，不会覆盖扫描报告。`offline_plan.json` 中的候选数量不能冒充实际完成数量。即使 `reachable_scan_finished`，`measured_surface_coverage` 仍为 null，表示没有测得完整表面覆盖率。

## 5. 当前验证范围

本轮仅修改、构建和离线规划验证，不启动 Gazebo、RViz、感知节点或机器人运动。RViz 工具插件名称已对照本机 Humble 插件声明；配置已检查 MoveCamera 为首个工具且提供整体/俯视预设。

离线检查包括当前 URDF FK、FCL 碰撞、目标姿态与路径设计。运行时 Gazebo CLI 查询、控制器跟随、停稳融合和实际观察效果仍需用户启动后验收。不能把离线规划图或世界真值图称为真实重建效果。

五项离线回归检查已通过：当前相机 FK 与项目既有 FK 对照、look-at 光轴/旋转正交性、FCL 的无碰撞/故意放置阻挡物/关节限位拒绝、移动过的场景被拒绝，以及相机与相邻 wrist_2 的碰撞不会被普通相邻连杆豁免规则漏掉。日志及最终离线计划见 [验证目录](/home/czz/perception_eye_in_hand/artifacts/orbit_scan_20261027)。Gazebo CLI 的只读参数对照了本机帮助；未连接正在运行的场景验证其输出或执行扫描。

后续若当前机械臂和障碍物位置无法提供所有方向的视角，扩大搜索时间只能改善求解机会，不能扩大物理可达空间。可考虑调整障碍物组布局、机器人基座位置、分对象扫描或使用第二个相机；这些不在本轮自动修改范围内。

## 6. 文件说明

| 文件 | 修改内容 |
| --- | --- |
| src/ur7e_nvblox_native/rviz/native.rviz | 鼠标工具、Views 面板、整体视角、俯视、青色规划路径 |
| run_ur7e_orbit_scan.sh | 新的一条指令入口，使用现有仿真环境 |
| src/ur7e_d435_gazebo/scripts/orbit_scan.py | 离线/执行模式、场景核对、路径重验、停稳采集和状态 |
| src/ur7e_d435_gazebo/scripts/orbit_geometry.py | 当前 URDF 的 FK/IK、视角生成、FCL 接口与路径搜索 |
| src/ur7e_d435_gazebo/src/scan_collision_checker.cpp | 离线 FCL 实际几何查询，不启动 ROS/Gazebo |
| src/ur7e_d435_gazebo/test/test_orbit_geometry.py | FK、相机朝向、碰撞/限位、移动场景拒绝的离线检查 |

感知算法与 5/30 Hz 配置继续参见 [官方结构对齐实现说明](/home/czz/perception_eye_in_hand/docs/ESDF_OFFICIAL_ALIGNMENT_V4_IMPLEMENTATION.md)。
