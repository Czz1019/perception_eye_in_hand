# UR7e 在线感知规划

复用 ur7e_nvblox 的时间戳 TF/深度工具和 realtime_nvblox 的原生 GPU 后端，
通过新的原子 PlanningMap 消息将 ESDF 接入 storm_kit SAGE。原包代码不变。

```bash
./build_ur7e_perception_planning.sh
./run_ur7e_perception_planning.sh
# 已有且初始姿态相同的场景：start_scene:=false
# 按需开启 Gazebo 图形窗口：gazebo_gui:=true
```

默认只运行 Gazebo 服务端（headless）和 RViz，不启动 `gzclient`。
这会避开 Gazebo 图形界面加载在线模型列表时出现的 `libcurl (35)/(56)`、
`SSL routines::unexpected eof while reading` 网络错误，同时减少图形渲染负载。
相机仍由 Gazebo 服务端渲染，深度、TF、ESDF 和控制器继续工作。
显式开启 `gazebo_gui:=true` 后仍可能遇到外部模型服务器的网络错误；
本包没有关闭 TLS 证书校验，也没有修改系统网络配置。

whole_control 同一进程运行感知与规划：地图在进程内直接交接，GPU 操作串行，
ROS 接口同时发布无损压缩的完整地图用于观测/记录。系统 ROS 运行 Gazebo/RViz。仅连接 Gazebo 的
`/forward_position_controller/commands`，不要同时运行其他规划器或扫描控制器。

默认采用已验证的高初始观察姿态，不运行 reconstruction 扫描节点。
新包的场景启动器将深度配置为 320×180 / 10 Hz、RGB 为 320×180 / 1 Hz，
关闭未使用的插件点云输出，以减少渲染负载；原场景包文件不变。
## A→B 最小 demo

默认使用 `config/demo.yaml`：

```bash
./run_ur7e_perception_planning.sh
```

启动于高位 A（tool0 约 `[0.220, 0.149, 0.855]` m），先执行高位多视角观察，
每完成一个视角就检查当前到 B=`[0.5, 0, 0.45]` m 的整臂通道；通道满足地图净空后提前结束观察，直接到达 B。观察动作和到达动作均使用 SAGE 速度空间优化，
经执行碰撞检查后转换为 ros2_control 位置指令；demo 不使用局部雅可比控制器。

历史固定扫描基线见 [A→B 验证记录](validation/AB_DEMO.md)，其 389.5 s 是优化前的结果。
当前版本采用通道就绪即提前结束扫描、分档速度和保持相机观察方向的多段引导。
两轮已完成绕墙验证，结果及降频限制见 [扫描与绕墙优化记录](validation/FAST_WALL_DEMO.md)。

在已 source ROS 和本工作区的另一个终端发送目标：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.5, y: 0.0, z: 0.45}, orientation: {w: 1.0}}}'
```

`world` 目标经 TF 转到 `base_link`；也可直接指定 `base_link`。只跟踪 xyz，忽略目标姿态。
扫描中收到的目标会保留，在观察停靠点按最新目标重新检查通道；到达阶段再次发布目标会清除旧目标的引导。
新目标首先求解满足感知地图净空和自碰撞约束的关节姿态，并采样检查从当前关节位置到该姿态的
整臂引导路径。若终点可行但途中受阻，会将沿途净空加入求解，调整冗余的肘部、腕部姿态；
只有完整路径也通过密集采样检查，才为 SAGE 提供引导。SAGE 的预测使用同步减速的关节序列，
并惩罚偏离引导路径的候选动作，执行端仍逐段检查最新地图。
到达途中每秒在新地图上复查剩余引导；新观测使路径失效时停止旧指令并重新求解。
直接引导受阻时，搜索经地图检查的观察姿态路网，并在首个目标到达后或所有观察完成后使用有时间上限的双向 RRT。
路径搜索只提供 SAGE 引导，每条边和每次执行仍检查整臂净空。观测不足时继续剩余观察姿态；
无可行路径则停动，不保证任意 xyz 可达。

当前仿真外参的相机光轴为 tool0 +Z。demo 的 IK 和路径约束让光轴朝下（Z 分量不高于 -0.25），
避免抬腕看天空后全图无有效深度、继而触发地图超时的停滞；位置目标仍可调整其他冗余姿态。
这不是通用可见性证明；全图无有效深度仍不会刷新地图或清空未知空间。

主要状态：`SCANNING_*` / `SCAN_DWELL` 为建图准备，`REACHING` 为到达阶段，
`SOLVING_GUIDE` / `REPLANNING` 表示正在求解或更新引导，
`GOAL_REACHED` 为位置误差小于 1.2 cm 后保持；`GOAL_NOT_OBSERVED_OR_BLOCKED` 表示
终点或整臂引导路径尚未满足地图约束，`BLOCKED` 表示当前优化动作未通过检查。
`STALE_MAP` / `STALE_JOINT_STATE` / `STALE_PLAN` 会保持指令，输入恢复后重新检查。

### 感知设计与适用范围

- 相机全程在线融合；12 个观察姿态覆盖后上方、前方和俯视方向，每点停留至少 0.4 s 并等待至少 3 次新地图更新；通道已就绪时不要求走完 12 点。
  后退和高位伸肘观察用于避开相机近场盲区，检查前臂未来要经过的空间。
- 保存静态场景已观测自由空间，避免移开视线就丢失通道；深度无效或遮挡后方不清空。
- 表面融合距离为 3 m；自由空间射线使用当前 Gazebo 插件有效的 0.2～6 m 实测深度。
  3～6 m 的有效回波可以证明射线近段为空，但不会被截成 3 m 处的虚假表面，也不融合远处表面。
  无返回、超量程和自身过滤的像素仍为未知；`far_free_ray_pixels` 可检查远距离回波是否参与。
- 扫描动作使用 `demo_scan.yaml` 固定的已确认观察走廊，碰撞球外 7.5 cm 内作为启动先验，
  另保留初始姿态 12 cm 先验；先验以 `free_source=2` 明示，观测自由空间为 1。
  这是当前静态仿真布置下的初始化假设，**不是相机观测结果**，不能直接迁移到改动后的场景。
- 先验不会跟随用户目标或实际关节状态扩张，测得的障碍覆盖先验，其余未知区域仍禁止运动。
- 场景障碍真值只由仿真生成器和独立测试使用；运行中的感知、逆解和 SAGE 不读取墙/球坐标。
- 只允许从随包高位姿态启动。demo 中重置地图后必须重新启动整个场景，以重新完成建图准备。
- demo 查询网格为 x∈[-0.60,0.90]、y∈[-0.65,0.75]、z∈[-0.05,1.25] m，间距 2 cm；
  整臂碰撞球都必须位于有足够净空的网格内。原生 nvBlox 体素仍为 2.5 cm。

原小范围闭环仍可使用 `config:=<工作区>/src/ur7e_perception_planning/config/online.yaml`，
该模式不执行扫描，自动目标为初始末端偏移 `[-0.02,-0.03,0]` m。

常用状态/控制命令：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link}, pose: {position: {x: 0.22, y: 0.15, z: 0.86}, orientation: {w: 1.0}}}'
ros2 topic echo /perception_planning/status
ros2 service call /perception_planning/enable std_srvs/srv/SetBool '{data: false}'
```

当前 `/target_pose` 使用位置目标；消息中的姿态不会作为到达要求。

## 范围与地图语义

- 仅静态环境、在线深度融合、低速滚动规划；不承诺动态障碍物避障。
- 地图固定 base_link；PlanningMap 的数组按 [z,y,x] 排列，x 最快。`grid.map_arrays()`
  解码消息中的无损 zlib 数据，恢复原生距离、规划距离和自由空间来源三组数组。
- 同一消息内包含原生 nvBlox 距离、保守规划距离、自由空间来源、采集时间和地图版本。
- 未知空间和越界均阻止运动；已观测的深度射线前方作为自由空间证据。
- 相机近场与机器人背面不可见，初始机器人各碰撞球外 **0.12 m** 使用显式初始自由空间先验。
  该先验仅用于随包启动的已检查静态仿真姿态，不能直接用于未知真实环境。
  更换场景/初始姿态须重新确认净空，或将 initial_clearance_m 设为 0（可能无法起步）。
  新观测到的障碍始终覆盖先验。先验不会随机器人运动向未知区域扩展。
- 地面 z=0 为固定支撑平面；墙和球的位置不输入规划器。
- 深度去除机器人/相机/支架返回；无效像素不清除其后的空间。
- 相机/支架加入工具坐标系下的保守碰撞球；非邻接连杆使用球间自碰撞检查。
- SAGE 每轮冻结一份 GPU 地图；独立 CPU 执行器逐段检查最新地图，限制仿真时间下的关节速度。
- 原有不检查碰撞的局部修正关闭。旧 `online.yaml` 模式在距目标 10 cm 内优先使用阻尼雅可比修正，
  必须通过相同的整臂 ESDF、连续段采样、自碰撞和关节限位检查才可执行。
  修正可行时省去本轮 GPU 优化，给感知留出资源；修正受阻或目标更远时调用 SAGE。
  状态中的 `command_source` 和 `sage_optimizations` 可区分两条实际执行路径。
- 执行检查最小规划净空 1.5 cm，demo 路径要求 2 cm（旧 online 模式额外保留 1 cm 跟踪余量）；这两者之外，
  距离场本身已扣除体素和插值误差。状态/地图/规划过期时冻结最后一条已检查指令。

demo 从 0.08 rad/s 起步，按最近 8 帧的地图间隔/年龄及规划耗时逐级允许 0.16、0.24、0.30 rad/s；
延迟增大立即降档，地图过期速度为零。配置 0.30 是上限，旧 online.yaml 仍为 0.08。
规划目标周期 0.1 s、执行目标 50 Hz，均不代表实测频率。
静态场景允许图像/地图最多滞后 1.5 s 仿真时间，同时使用 3 s 墙钟地图看门狗；
运动过程中按采集时刻查询 TF。这一时间预算不适用于移动障碍物或高速运动。
规划距离另外扣除了网格插值误差余量；未知边界附近可能停止，需要更多观测。

日志位于 `~/nvblox_output/perception_planning/<时间>/`，含运行解析配置、状态、q_next、
q_cmd、q_actual、指令来源和碰撞检查结果。RViz 显示 ESDF、机械臂/负载碰撞球、目标和 SAGE 候选轨迹。
`/ur7e_nvblox/stats` 记录深度接收、融合与地图发布计数，以及排队、TF、自过滤、GPU 等待和自由场计算耗时。
映射线程保留短图像队列并选择已有精确时间 TF 的最新一帧；GPU 锁优先处理等待中的感知任务。
自由空间掩码不变时复用距离变换，变化时仅计算包围区域；新障碍仍立即使缓存失效。
地图发布为计算开始时刻计周期，RViz ESDF 限制 2 Hz、画面 15 FPS，规划快照不受可视化限频影响。

## 验证与故障注入

源 ROS 和工作区后，在 whole_control 中执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -B -m pytest -q src/ur7e_perception_planning/test
# 默认 A→B demo 的只读观测验收：
python -B src/ur7e_perception_planning/test/validate_demo.py --duration 600 \
  --output /tmp/perception_planning_demo.json
# 下面的历史故障脚本仅用于 online.yaml 模式，会暂停/恢复感知和重置地图：
python -B src/ur7e_perception_planning/test/validate_sim.py --exercise --faults \
  --output /tmp/perception_planning_validation.json
```

`/perception_planning/mapping_enabled`（SetBool）仅暂停/恢复地图更新，规划端的地图超时停动仍生效。
`/ur7e_nvblox/reset_map`（Trigger）重置地图；初始自由空间先验始终固定在已验证的初始姿态，
不会随当前机械臂姿态扩张。`/perception_planning/enable`（SetBool）控制规划执行。

墙内目标、视野外/未知区域会受阻；固定观察序列不等于通用主动探索。
有界路径搜索尚未完成通用目标集验收；没有动态障碍消退或任意远目标到达保证。
运行验证结果见 [validation/RESULTS.md](validation/RESULTS.md)。碰撞球近似和离散地图不构成硬件安全保证。
默认 headless 启动的修复及两遍复测见 [validation/HEADLESS_FIX.md](validation/HEADLESS_FIX.md)。
MarkerArray 的 QoS 修复见 [validation/MARKERS_QOS_FIX.md](validation/MARKERS_QOS_FIX.md)。
已完成事项与达到原规划包场景到达能力的后续任务见 [ToDoList.md](ToDoList.md)。
