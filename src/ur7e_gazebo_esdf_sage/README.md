# UR7e Gazebo：预扫描 ESDF + SAGE 模式 B

独立仿真包。预扫描融合 Gazebo 深度到 nvBlox TSDF/ESDF，冻结静态地图后，由实际关节反馈驱动 SAGE 滚动规划。**显式选择 `bounded_unknown` 局部地图模式后，已完成三个目标无碰撞到达及故障保护验证；默认 `strict` 模式仍会因近身未知区域拒绝起步。**局部模式保留未知标签、允许有界未知使用，不能将其结果解释成严格模式或实机安全保证。详见 [最新测试报告](validation/partial_map/REPORT.md)。未调用 RTDE、servoJ 或真实机器人 Action。

## 启动与操作

在 `/home/wqj/realtime_nvblox_ws`：

```bash
./build_ur7e_gazebo_esdf_sage.sh
./run_ur7e_gazebo_esdf_sage.sh
```

默认 Gazebo 后台运行。要打开 RViz，加 `run_rviz:=true`；要显示 Gazebo 窗口，加 `gazebo_gui:=true`。本机曾出现 RViz 与 CUDA 同时运行导致规划延迟的情况，因此完整闭环默认关闭可视化。过期优化结果会被丢弃并保持，不会继续执行。只启动场景等待手动开始，加 `auto_start:=false`。日志与地图默认保存在 `~/nvblox_output/gazebo_esdf_sage/<时间>/`；可传 `output:=/绝对路径`。

```bash
./ur7e_gazebo_esdf_sage_control.sh status
./ur7e_gazebo_esdf_sage_control.sh start
./ur7e_gazebo_esdf_sage_control.sh target 0.50 0.00 0.60
./ur7e_gazebo_esdf_sage_control.sh stop
./ur7e_gazebo_esdf_sage_control.sh invalidate
```

目标为 `base_link` 下的 `tool0` 位置，单位米；不约束末端姿态。也可在隔离 ROS 域发布 `/target_pose`（`geometry_msgs/PoseStamped`）。运动中可重设目标；停止或地图失效后保持停止，需重启并重新扫描。场景在冻结后必须保持静态，移动障碍后先 `invalidate`，再重启扫描。

启动与操作脚本使用 `UR7E_SIM_DOMAIN_ID=87`、`UR7E_SIM_GAZEBO_PORT=11387`，不继承终端里的 `ROS_DOMAIN_ID`。禁止使用默认真机域 0。普通 `ros2` 诊断命令需自行 `export ROS_DOMAIN_ID=87`。

## 补充覆盖扫描与停点处理

`config/scan_coverage.yaml` 提供 43 个停点（215 帧）的补扫方案，包含原有视角及两组近身环视。使用 `scan_config:=/home/wqj/realtime_nvblox_ws/src/ur7e_gazebo_esdf_sage/config/scan_coverage.yaml` 选择；默认方案仍为 16 个停点。`test/design_coverage_scan.py` 对补扫路径做独立工装几何核验，核验信息只用于扫描运动，不进入规划地图。

输入图像进入融合前仍须满足 1 秒的新鲜度限制。静态停点计算允许最长 5 秒处理时间，但接收结果时要求采集门和保持租约仍有效、关节反馈新鲜、从图像时刻起的反馈连续（间隔不超过 0.1 秒），且整个期间位置偏差小于 0.008 rad、速度不超过停稳阈值。运动、反馈缺失和超时结果会被拒绝。这不是动态场景的处理延迟许可。

本包将自滤波的相同射线/球相交判据批量计算，并将本进程 Python 线程切换间隔设为 1 ms，以减轻并发回调下大量小数组操作的调度开销；没有改变自滤波膨胀尺寸或自由空间判据。`test/test_self_filter.py` 验证 43 个姿态上的像素级一致性，`test/test_stationary.py` 验证延迟结果的接收和拒绝条件。

## 模式 B 的实现

`实际 q,dq → SAGE 优化 → 第一小步 q_next → 整臂/相机碰撞复核 → 200 Hz 位置指令 → Gazebo ros2_control → 下一次实际反馈`

- 地图冻结后，目标 IK、观察点连接图和有界 RRT 仅生成经过检查的关节引导路径，不直接下发该路径。
- 每个运动小步都重新读取实际关节位置与速度，运行真正的 CUDA SAGE 优化，仅执行约 0.1 s 的首步。独立检查当前反馈与命令位置到候选点的完整插值段；候选不安全时保持，不改为直接插值追踪引导路径。
- 扫描速度上限 0.25 rad/s，采用五次曲线停点；规划运动上限 0.15 rad/s。通过原有 `/forward_position_controller/commands` 传输。
- Gazebo 位置接口的瞬时速度包含物理子步重力漂移。此包以带时间戳的实际位置差分计算 `dq`，用于停稳判据和优化输入；不是预设零速度。
- 反馈丢失、跟踪误差、地图改变和碰撞检查失败不会继续推进未经检查的运动。反馈过期立即保持；持续 2 秒没有新反馈则锁定停止，短暂延迟恢复后继续。小步进度只随新反馈推进，时钟跳变不会追赶出一个大位置跳步。

这是**静态 ESDF 上的运动反馈闭环**。冻结后不在线更新环境地图，也不宣称具备动态障碍感知避障能力。

## 外参、碰撞和观测范围

`config/real_calibrated.urdf` 是 `/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf` 的标定快照。生成的 Gazebo 模型使用同一套机械臂关节变换、`tool0 → camera_link`、深度/彩色光学帧变换；实际传感器 `<pose>` 也按这些变换设置。仅更改消息 `frame_id` 不足以完成该适配。

碰撞模型包含 16 个原机械臂球和 6 个 D435i 外壳包络球；相机覆盖体包含 5 mm 膨胀。环境间隙 30 mm、自碰撞间隙 15 mm。未测量的支架、夹具和线缆尚未包含。

场景为本包独立测试工装：两块箱形墙和一个球体，定义于 `scripts/spawn_fixture`；不更改已有 Gazebo 场景包。深度自过滤另外覆盖固定底座，避免底座被融合成肩部附近的环境障碍；该过滤只丢弃像素，不提供自由空间证据。

规划地图明确区分：

- `source=1`：深度射线观测到的自由空间。
- `source=2`：根据可靠观测的顶部高度和明确声明的桌面结构假设，推断的上方自由空间。
- `source=0`：未知或被测量占据的空间；严格模式禁止通过，局部模式另行限制未知使用，实测障碍始终优先。

`config/tabletop.yaml` 声明：静态桌面场景，观测顶面上方无其他物体、悬挂物或上层结构；只在给定工作空间内应用此假设。可靠列要求朝上的连续深度面、水平单元四个子区均有采样、至少两个不同相机位姿支持；再收缩一格支持区域，使用相邻九格最高实测高度加 40 mm 余量。深度断裂、无效像素、侧面和缺乏支持的水平位置不会触发上方填充。这里是有前提的结构推断，不等于相机直接测量到了所有上方体素；参数尚需实机噪声与分辨率验证。

固定扫描扫掠通道只由独立的 `ScanPriorField` 用于预扫描运动检查。规划 `FieldBuilder` 从空地图开始，只接收深度和 native ESDF；冻结时丢弃扫描通道，重算距离并检查整臂起步状态。运行时规划不读取工装障碍真值，也不将扫描通道或机器人扫掠轨迹写入规划自由空间。`test/design_scan.py` 的工装真值仅用于离线核验扫描运动；`test/audit_tabletop.py` 只在运行之后审计结果。

新地图策略为 `depth_tabletop_columns_v1`。旧版带扫描通道自由空间的冻结地图会被拒绝加载，不能继续沿用旧版到达测试作为新策略的验证。

早期 43 停点补扫已完成 215 帧，可靠列增加到 4814，但冻结后起始状态仍被拒绝，未执行 SAGE 到达。详见 [补扫测试报告](validation/coverage/REPORT.md)，包括采集修正、失败轮次、实际间隙和本体不可观测问题。

## 本体实体与环境未知空间

根据用户确认的边界，冻结后单独生成 `robot_ownership.npz/json`。原始 `frozen_map.npz` 和 source 标签不改动；本体记录不是感知自由空间。仅使用真实标定 URDF 的七个封闭凸碰撞网格，在冻结时刻实际关节姿态下计算完整位于实体内部的网格单元。没有使用球模型的膨胀部分、仿真新增的相机外壳、安装区或扫描扫掠体积。

环境碰撞场允许查询上述实体内部，同时继续将外侧未知单元作为障碍；实测占据优先，不被本体记录覆盖。所有网格单元的八个角点都必须处于同一个 URDF 实体内部，边界单元宁可保留未知。场的距离仍扣除网格和插值余量。起步检查、引导路径、SAGE 距离查询和实时整臂检查使用同一个派生快照，自碰撞检查保持不变。

本体记录绑定冻结环境地图 UID、实际关节姿态、URDF 与网格文件 SHA256，含独立碰撞场 UID；记录不随未来候选姿态变化，也不累积历史运动空间。冻结后仍要求环境静态。`test/audit_ownership.py <输出目录>` 独立核对完整网格单元位于模型内部、外部未知未清除、实测障碍未覆盖，并事后检查与工装真值的交集。

`maps/<会话>/summary.json` 仍描述原始感知地图的检查结果；实际使用本体记录后的间隙见同目录 `robot_ownership.json` 及输出目录的 `ownership_audit.json`。运行事件同时记录环境地图 UID 和派生碰撞场 UID。

这项区分不能自动证明实体外侧及碰撞余量范围自由；若外侧覆盖不足，起步检查仍会拒绝规划。

`scan_oblique.yaml` 在 43 停点方案上增加低位斜视，合计 63 停点、315 帧。`test/design_oblique_scan.py` 可重新核验并生成该扫描配置。`tabletop.yaml` 的 `ray_evidence: raw_depth_before_returns` 保留原始有效深度返回点之前的射线空间（仍截断噪声与网格余量），自滤波后的深度才进入 TSDF 和顶部高度推断。不会将机器人表面作为桌面顶面，也不会清空其后方空间。此配置进入地图 identity，不能把不同射线策略的旧地图当作相同地图使用。

`scan_high_central.yaml` 将中位补扫替换为高位中心视角，仍为 63 停点、315 帧。最新实测使阻挡球从 14 个减少到 9 个，但未通过冻结后的整臂起步检查，SAGE 步数仍为 0。详见 [最新测试报告](validation/ownership/high_central_63_views/REPORT.md)；其中另外区分了本体内部边界体素的表示问题和实体外未知缺口，未扩大本体范围。

## 局部地图模式：区分障碍与未知

显式选择 `planning_policy:=bounded_unknown` 可使用局部地图规划策略；默认 `strict` 仍保留原先未知一律阻挡的行为。此策略调整未知空间的运动许可，**不是证明未知空间自由**，也不是原严格模式已通过。

```bash
./run_ur7e_gazebo_esdf_sage.sh planning_policy:=bounded_unknown scan_config:=/home/wqj/realtime_nvblox_ws/src/ur7e_gazebo_esdf_sage/config/scan_high_central.yaml
```

`partial_map.py` 分开处理三类信息：

- 感知证据：原始 `frozen_map.npz` 的 source 及桌面结构推断完全不变，未知仍为 0。
- 已测障碍：由 native ESDF 已知占据和已知距离生成障碍距离约束，整臂及相机保留 30 mm 环境间隙、自碰撞 15 mm 间隙。
- 不确定性：仅允许使用距离环境自由证据网格中心不超过 80 mm 的未知网格单元，远处未知及地图外仍阻挡。这个范围是实验性探索上限，既不是噪声界，也不是安全证明；20 mm 网格具有离散误差。URDF 内部单元自身不膨胀，不作为未知邻域距离的种子。

单独保存 `partial_policy.npz/json`，包含障碍距离、未知程度、准入掩膜及合成的运动可行性距离。合成距离不能称为感知 ESDF 的自由空间距离。目标点仍须在观测或桌面推断支持的空间内；IK、连接搜索、SAGE 和每步执行均使用同一可行性约束。SAGE 额外对各球中心及六个极点采样未知程度，增加代价；完整球及连续连接的硬检查仍独立进行，代价采样不替代这些检查。

该模式接受有限未知空间带来的残余风险，若未知区域藏有未观测物体，单靠当前静态地图不能保证不碰撞。Gazebo 是否实际无碰撞，须由运行后独立工装真值审计确认；真值不参与运行时规划。后续实机迁移需要另行处理观测盲区与执行保护，不能直接把仿真通过当作实机保证。

`test/audit_partial.py <输出目录>` 独立核对未知准入上限、占据优先、标签不变及实际运动中的障碍间隙、可行性间隙、未知采样比例。该比例是中心和极点的样本比例，不是整臂体积比例。

## 输出和复核

`status.json` 为当前状态；`events.jsonl` 记录各观察点、冻结地图、目标与到达事件；`execution.csv` 记录实际位置、命令、SAGE 首步和实际差分速度；`sage_feedback.jsonl` 实时记录，`sage_feedback.json` 在到达或退出时汇总，内容为每次成功执行小步使用的真实反馈时间戳。`maps/<扫描会话>/` 保存观测深度、相机位姿、冻结地图、native ESDF 和 `tabletop_evidence.npz`（顶部高度、加余量后的高度、可靠列及视点计数）。

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
~/miniconda3/envs/whole_control/bin/python -B -m pytest -q -p no:cacheprovider src/ur7e_gazebo_esdf_sage/test/test_offline.py src/ur7e_gazebo_esdf_sage/test/test_tabletop.py
# 对新策略的地图支持关系、实际扫描及保持状态进行审计：
~/miniconda3/envs/whole_control/bin/python -B src/ur7e_gazebo_esdf_sage/test/audit_tabletop.py /完整运行输出路径
# 完成运行后，对实际运动进行独立几何验收：
~/miniconda3/envs/whole_control/bin/python -B src/ur7e_gazebo_esdf_sage/test/validate_run.py /完整运行输出路径
```

`test/probe_runtime.py` 对实时 TF、控制器和投影深度进行只读检查。测试用故障服务 `test_feedback_loss` 仅让本节点模拟反馈丢失，不停止传感器或更改 Gazebo 控制器。

当前策略的实测与覆盖限制见 [validation/tabletop/REPORT.md](validation/tabletop/REPORT.md)。[旧报告](validation/REPORT.md) 保留历史证据，其中到达测试依赖已移除的扫描通道预填充。严格模式仍须通过地图起步检查；局部模式已完成下列墙间低位目标的到达验证：

```bash
./ur7e_gazebo_esdf_sage_control.sh target 0.6027275713 -0.0729611598 0.4420525377
```

`test/system_scenario.py <输出目录>` 会在已启动的隔离仿真上测试到达、换目标、未知目标拒绝和反馈故障，结束时故意保持锁定停止。该脚本不会启动真机。

## 第一阶段在线新增障碍层

`online_obstacles:=true` 在静态预扫描后持续接收深度；当前需要配合 `planning_policy:=bounded_unknown`。静态感知地图和局部未知准入范围保持冻结，新增障碍生成独立占据层与距离场，以距离的逐点最小值覆盖静态运动可行性场。新占据优先于旧自由；在线层不会扩大未知空间准入范围。

深度、关节插值、TF 和相机标定按图像时间对齐，自滤波后的新返回点用于占据，原始有效返回点之前的射线用于清除。离开视野或经过一定时间都不会自动清空障碍。当前使用 20 mm 网格、一次邻接膨胀和 40 mm 静态表面差异阈值；贴近已有表面的细小新增物尚不属于已验证能力。

SAGE 每轮采用不可变地图快照；执行首步前重新读取最新地图，并在控制过程中检查剩余段。地图超过 0.5 s 未更新或路径被新障碍阻挡时取消运动段并保持最后检查过的命令。地图恢复后重新求解。实际机械臂间隙不足仍触发锁定停止。这是仿真中的规划保护，不代表已经验证真实控制器的制动距离。

复现第一阶段实验：

```bash
./run_ur7e_gazebo_esdf_sage.sh output:=/绝对路径/输出目录 planning_policy:=bounded_unknown online_obstacles:=true hold_after_scan:=true scan_config:=/home/wqj/realtime_nvblox_ws/src/ur7e_gazebo_esdf_sage/config/scan_high_central.yaml
# 在另一终端 source ROS 和工作空间后执行：
ROS_DOMAIN_ID=87 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ~/miniconda3/envs/whole_control/bin/python -B src/ur7e_gazebo_esdf_sage/test/online_scenario.py /绝对路径/输出目录
```

测试进程单独向 Gazebo 添加静止方块，将其位置设为目标，核对阻挡保持，再移除方块、等待射线确认清空并恢复到达。方块真值只用于独立测试和事后审计，不传给感知或规划。此场景验证新增静止障碍的处理，不验证任意运动障碍预测或突然侵入时的制动性能。

`online/updates.jsonl` 保存更新时间、延迟和版本；`version_*.npz` 保存改变后的占据；`observation_*.npz` 保存深度及位姿证据。`test/audit_online.py` 复查每个成功执行首步所记录版本的整臂碰撞约束，并独立检查实际轨迹与静态工装、新增方块的间隙。

第一阶段实测结果见 [在线实验报告](validation/online_stage1/REPORT.md)：阻挡、射线清除、恢复到达与无碰撞审计通过；存在最长 70.28 s 更新中断并保持的稳定性问题，尚不能视为稳定实时动态避障系统。

## 第二阶段：低速横穿障碍

通过 `online_engine:=v2` 选择第二版在线感知，默认仍为 `v1`。新版使用独立进程接收深度、关节和 TF，执行时间对齐、机器人自滤波、在线占据更新及距离场计算，再通过带锁共享内存交换完整地图快照。几何计算另由单个工作进程完成；障碍端点按 4 倍降采样，清除射线保留 2 倍降采样分辨率。SAGE 和伺服回调不会与感知进程共享 Python GIL。静态底图和未知准入规则保持不变；仍只用有效自由射线清除占据，不使用时间衰减。

新版在线距离额外扣除 20 mm 运动余量；这是本阶段低速实验的显式参数，不是任意速度下的安全保证。地图仍在 0.5 s 后过期。过期时保持，恢复后保留并重新检查剩余引导；采样引导的折点只在完整替换连接通过碰撞检查时删除。运行时不接收测试方块的位置或速度，也不读取其仿真真值。

```bash
./run_ur7e_gazebo_esdf_sage.sh output:=/绝对路径/输出目录 planning_policy:=bounded_unknown online_obstacles:=true online_engine:=v2 hold_after_scan:=true scan_config:=/home/wqj/realtime_nvblox_ws/src/ur7e_gazebo_esdf_sage/config/scan_high_central.yaml
# source ROS 与工作空间后，在另一终端运行独立测试：
ROS_DOMAIN_ID=87 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ~/miniconda3/envs/whole_control/bin/python -B src/ur7e_gazebo_esdf_sage/test/moving_scenario.py /绝对路径/输出目录
```

测试方块边长 80 mm、速度 10 mm/s，由 Gazebo 平面速度插件驱动；真值里程计仅写入 `moving_truth.jsonl` 用于事后审计。先横穿原到达路径并检查绕行到达，再返回观察位置、堵塞目标、低速移开并检查恢复到达。测试进程在确认相机检测到方块后才发目标。

`audit_moving.py` 核对实际全臂/相机球与移动方块的逐时刻间隙、执行地图版本及更新连续性。`audit_crossing_route.py` 检查方块扫掠范围确实与不含新增障碍时的基线路径相交；该检查是空间相交，不是同时刻碰撞的反事实证明。`plot_moving.py` 生成运动与更新时间线。失败轮次也保留原始证据。

第二阶段完整结果与失败轮次见 [测试报告](validation/moving_stage2/REPORT.md)。


## 保留版本的感知与规划架构（2026-10-18）

本包保留“完整引导路径搜索 + SAGE 分段滚动优化”的已验证实现。连续局部优化实验另放在 `ur7e_gazebo_esdf_sage_continuous`，不覆盖本包运行代码。

感知链路：实机标定相机外参 → 静态多视角扫描 → TSDF/ESDF 与可靠桌面列上方自由假设 → URDF 本体实体处理、80 mm 有限未知准入 → 冻结静态可行性底图。在线 v2 用独立感知进程和几何工作进程持续处理深度，按图像时刻对齐关节/TF、自滤波、新增占据、有效射线清除与距离变换，再与静态底图逐点取最小值。在线层不会扩大静态准入，离开视野不按超时清除占据。地图年龄超过 0.5 秒保持。

规划链路：实际关节反馈与最终末端目标 → 多初值数值 IK → 尝试直接关节连接 → 以扫描关节姿态为候选节点的图搜索 → 必要时关节空间采样树搜索 → 检查并缩短完整引导路径 → SAGE 依次优化各引导点 → 最新地图整臂检查 → 200 Hz 插值发布关节位置命令。扫描姿态只是候选节点，连接仍按当前地图检查，不预填充扫描通道。

`planning.py:route()` 的完整搜索在 CPU 执行，预算约 30 秒；失败时保持并重试。SAGE 只在引导得到后开始运动优化。每轮重设朝向引导点的速度初值、`shift_steps=0`，执行完当前 0.1 秒小段后再优化。因此它是感知闭环与执行约束验证版本，并非持续最终目标优化的低延迟实现。地图更新、SAGE 计算、路径搜索和伺服频率是不同指标，200 Hz 发布不能解释为 200 Hz 规划。

执行保护：地图版本与新鲜度、反馈新鲜度、实际整臂环境/自碰撞间隙、命令到候选连接及运行段重检、跟踪误差。环境 30 mm / 自碰撞 15 mm，v2 新增障碍距离另扣 20 mm 低速余量。仿真障碍真值仅用于独立测试与审计，不进入规划。
