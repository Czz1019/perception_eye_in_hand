# 真机自动预扫描 → 静态 ESDF → SAGE 避障到达：任务交接

整理日期：2026-10-13。已核对当前源码、配置与验证记录；历史实测主要发生于 2026-09-27～29。本次只整理交接文档，没有连接设备或执行测试。设备当前是否在线、现场是否变化、示教器模式均未重新检查。

## 2026-10-13 后续更新：新增自动 ESDF 包

已新建 `src/ur7e_auto_esdf`：复用 8 个点和修复后的独立 RTDE 控制，自动停稳融合深度，保存可由真实静态规划包加载的 `frozen_map.npz`。28 项单元/共享控制、12 类故障测试及模拟 8 点/64 帧＋真实 CUDA 通过；实际 launch 回看与 TF 通过。**仍未进行新包真机测试，尚未接入 SAGE 目标规划/真机避障执行。** 后续优先读 `src/ur7e_auto_esdf/README.md`、`REAL_TEST_GUIDE.md`、`validation/RESULTS.md`。

以下是新增该包之前的交接基线；其中“自动扫描尚未接 ESDF”已由上述软件实现推进，P0 的硬件验证与 P1/P2/P3 仍未完成。原真实覆盖不足、模型间距问题和用户批准前不做真机测试的限制仍有效。

## 1. 最终目标与当前结论

最终目标：UR7e 搭载眼在手上的 D435i，自动依次到达观察位姿并停稳采集，建立 `base_link` 下的静态 ESDF；冻结并验收地图后，用 **SAGE** 生成通过整臂/自身碰撞检查的 A→B 轨迹，随后真机低速执行。

**现有能力已经分模块实现，但“自动预扫描 ESDF → SAGE → 真机避障到达”整条链路尚未接通和验收。**

目前分成两条线：

- `ur7e_real_static`：真实手动预扫描 → 静态 ESDF → SAGE 规划 → ROS 轨迹 Action 执行接口。真机验证止于 ESDF 扫描/冻结/预览；规划和执行仅完成软件/模拟控制器测试。
- `ur7e_auto_reconstruction`：RTDE 自动到点 → 停稳采集 → 彩色 TSDF 网格。**不计算 ESDF，不调用 SAGE**；可复用自动扫描与控制部分，但当前地图输出不能直接当作规划 ESDF。

彩色重建仅用于与实物比较、辅助检查感知，不作为最终规划输入。

## 2. 工作区和功能包分工

工作区：`/home/wqj/realtime_nvblox_ws`。ROS Humble，应用使用 Conda `whole_control`；`storm_kit` 已安装在该环境。参考控制实现：`/home/wqj/storm/examples/HIL_sage_rtde`。

| 功能包 | 当前职责 | 后续用途 |
|---|---|---|
| `src/realtime_nvblox` | nvBlox 基础实现 | 复用底层，保持源码不改 |
| `src/realtime_nvblox_2` | 他人的 TF 建图参考实现 | 已借鉴缓冲复用、计算/输出分离等，保持源码不改；其约 15 Hz 不代表本系统真机吞吐 |
| `src/ur7e_nvblox` | 深度＋CameraInfo＋图像时间 TF → ESDF | 已做查询/发布性能优化；单独使用缺少完整机器人自身过滤与规划闭环 |
| `src/ur7e_sage_planner` | 原独立 SAGE 仿真规划 | 规划能力参照；Gazebo 控制不是 RTDE 真机传输 |
| `src/ur7e_perception_planning` | Gazebo 在线 ESDF＋SAGE；含 `ur7e_perception_planning_core` | 复用 ESDF rollout、几何与地图基础；有特定 A→B/跨墙实测，未证明通用目标集稳定到达 |
| `src/ur7e_real_static` | 真机静态 ESDF、保守自由空间、SAGE、轨迹验证和 ROS Action 执行接口 | **最终系统的感知与规划主干** |
| `src/ur7e_real_reconstruction` | 手动真机彩色扫描/预览 | 保留，不覆盖 |
| `src/ur7e_auto_reconstruction` | 自动 8 点扫描、独立 RTDE、彩色重建 | **复用扫描调度与运动控制，不把彩色网格当作 ESDF** |

优先读 `ur7e_real_static/{README.md,ToDoList.md,REAL_TEST_GUIDE.md}`，以及自动包的 `validation/capture_fix/REPAIR_NOTES.md`。必要时新增一个薄的统一调度包，组合已有模块，不重写 SAGE/nvBlox，也不覆盖手动入口。

## 3. 硬件、标定与坐标约定

- 用户确认 UR7e IP：`192.168.56.100`；D435i 历史序列号：`349622072343`。
- 工厂校准：`/home/wqj/ur_arm/my_robot_calibration.yaml`，历史驱动校验通过。
- 已组合的校准 URDF：`/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf`。
- 当前预览用碰撞球：`/home/wqj/ur7e_real_setup/collision_spheres_preview.yaml`，**尚非通过真机规划验收的模型**。
- 历史真实 ESDF 配置：`/home/wqj/ur7e_real_setup/preview.yaml`。它写入旧测试输出目录，后续应复制配置并改为新的输出目录，不覆盖历史结果。
- 手眼外参原件：`src/ur7e_real_static/config/handeye_d435i.yaml`，是 `tool0 → camera_color_optical_frame`，平移单位 m：

```text
[[-0.706400704,  0.002414966, -0.707808034, -0.029901376],
 [ 0.707783955, -0.006516156, -0.706398905, -0.069185919],
 [-0.006318117, -0.999975854,  0.002893743, -0.012263443],
 [ 0,            0,            0,            1          ]]
```

对未对齐深度，应组合 `T_base_tool0 × T_tool0_color × T_color_depth`，不能把彩色光学外参直接用于深度光学图像。使用工厂校准模型、实际关节反馈以及图像采集时刻 TF，不依赖 SLAM，不以最新 TF 代替图像时间 TF。

历史输入话题：

- `/camera/camera/depth/image_rect_raw`、`/camera/camera/depth/camera_info`。
- 彩色辅助预览：`/camera/camera/color/image_raw`、相应 CameraInfo。
- 自动包只读 RTDE 输出 `/ur7e_auto/joint_states`，TF 为 `/ur7e_auto/tf`、`/ur7e_auto/tf_static`。
- 旧静态 ESDF 配置使用 `/ur7e_real_static/joint_states_sampled` 和默认 TF 通道。**接入自动扫描时必须统一这些输入与 TF remap；不能只改启动顺序。**

世界坐标固定 `base_link`。目标建议显式使用 `base_link`；`world` 只有存在经核实的 TF 时才可转换。当前真实静态规划目标是 tool0 的 xyz，PoseStamped 的 orientation 并非完整 6D 目标约束。

## 4. 已验证的真实 ESDF 状态与阻碍

**2026-10-13 补充：** 已新增实机标定 D435i 壳体 6 球包络及 `ur7e_real_static/config/real_d435i.yaml` 配置。
历史扫描结束姿态自碰撞间隙由 6.55 mm 提高到 17.28 mm，15 mm 阈值未降低；但旧 S01–S08 仍未通过该阈值，支架/工具/线缆仍缺几何，环境自由空间覆盖问题仍在。
完成 47 项离线回归与实际 SAGE/CUDA 几何一致性检查，未连接硬件。原配置、原模型和历史结果保留。
详见 [碰撞模型适配报告](src/ur7e_real_static/validation/camera_collision/REPORT.md)。下文为原扫描历史记录。

2026-09-27 手动扫描完成：**1233 帧、21 个观察姿态**；静态 ESDF 冻结、校验及 RViz 显示通过。用户确认该次扫描没有实物碰撞。报告中 21 张是记录关节角重绘图，并非逐姿态相机 RGB 原图。

- 地图：`log/real_first_scan_20260927/output/frozen_map.npz`。
- 网格：81×81×57、2.5 cm；观测自由体素 22,329，显式自由空间先验 0。
- 真实环境下历史短窗口约 4.23 Hz；有 CPU 降到约 400 MHz 和有效更新停顿记录，**没有持续 15 Hz 保证**。静态冻结后规划可避免持续依赖在线建图延迟。

当时真实目标轨迹预览未通过，主要有两项：

1. **起点整臂覆盖不足。** 17 个机器人/相机球中心均没有正的保守距离；观测自由区域最高约 z=0.55 m，而当时 tool0 高约 0.668 m。看到桌面物体，不等于机械臂全部链节与规划通道已经处于已观测自由空间。
2. **碰撞模型过近。** tool0 相机球与 wrist_2_link 球的模型表面间隙约 6.55 mm，小于 self_margin=15 mm。应核对真实安装几何、球包络和碰撞对，不直接降低阈值绕过。

历史相机/工具包络未完成实测；用户曾要求不关注夹爪，但如果实物仍安装且会扫过障碍，规划模型是否覆盖它仍需明确。用户“上方空旷、扫描可先不做避障”的授权不能解释成整幅未知 ESDF 都是自由空间。

相关证据：

- `src/ur7e_real_static/validation/FIRST_REAL_SCAN.md`
- `src/ur7e_real_static/report/2026-09-27_real_scan_report.md`
- `src/ur7e_real_static/validation/REAL_STATIC_OPTIMIZATION.md`

## 5. 自动扫描最近状态

观察点：`src/ur7e_auto_reconstruction/config/waypoints.yaml`，S01–S08。它们首先为彩色场景覆盖选定，**尚未证明能覆盖 ESDF 规划所需的整臂近场与全部候选通道**。

2026-09-29 第二轮真机已依次到达 8 点，用户反馈“无明显抖动”。保存 58 帧、8 组图像和彩色网格；S08 采集首帧后触发控制周期超时，最终 ERROR，不能算完整扫描通过。该轮没有获得额外的“无实物碰撞”确认。

随后按用户要求完成了**仅离线验证的修复**：

- 每点到达并停稳后，`servoStop → stopScript → disconnect`；再次连续核验静止后再采集。
- CAPTURING 阶段不运行 500 Hz 主机伺服、不持有 RTDEControl 连接；保留只读反馈、TF、漂移/速度/状态检查。
- 关门后重新核对实际起点，取得控制权执行下一段。原 40 ms 运动周期保护保持不变。
- 19 项控制测试、12 类协调器故障测试通过；模拟控制＋真实 CUDA 的 8 点闭环 DONE，共 67 帧。S08 人为持有 GIL 约 80.44 ms 后仍完成。
- **修复后未真机复测。** 具体主机停顿根因未由旧日志确定，仍不宣称运动线程硬实时。

默认自动扫描运动设置为 0.12 rad/s、0.18 rad/s²、500 Hz 目标频率；不等于障碍附近规划执行可直接使用这些速度。真实静态规划默认 0.05 rad/s、0.10 rad/s²，应分开配置。

报告与修复：

- `src/ur7e_auto_reconstruction/report/2026-09-29_real_auto_scan_test.md`
- `src/ur7e_auto_reconstruction/validation/capture_fix/REPAIR_NOTES.md`
- `src/ur7e_auto_reconstruction/validation/capture_fix/production_changes.patch`

## 6. SAGE 与执行接口的现状

`ur7e_real_static` 已实现冻结地图上的目标 IK、观测姿态路网/有界采样引导、实际 SAGE 优化、整臂/自碰撞检查、轨迹简化后的复查和低速五次轨迹参数化。不是仅显示 ESDF，也不是仅把 SAGE 名称接入。

关键源码：

- `src/ur7e_real_static/ur7e_real_static/capture.py`：输入、停稳、时间 TF、自过滤与建图。
- `field.py`：规划距离场、来源/未知处理、冻结与加载。
- `planning.py`：冻结地图上的 SAGE 规划。
- `trajectory.py`、`model.py`：路径检查、参数化、校准模型及碰撞球。
- `node.py`：状态机、目标、地图服务、Action 执行。
- `src/ur7e_perception_planning/ur7e_perception_planning_core/sage.py`：ESDF rollout/task 基础。
- `src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/{node.py,transport.py}`：扫描调度与 RTDE 控制复用入口。

真实静态包当前执行端是 ROS `FollowJointTrajectory`，默认 Action 为 `/scaled_joint_trajectory_controller/follow_joint_trajectory`；**并未接入自动包的 RTDE 执行器**。用户反馈原 ROS 控制有抖动，并提供了 RTDE 成功实践，因此建议优先实现经过验证轨迹的 RTDE 执行后端，但不能把“自动扫描 RTDE 已运行”当作“避障轨迹 RTDE 已验收”。

现有接口：

- `/target_pose`：仅生成目标轨迹预览，不自动执行。
- `/ur7e_real_static/start_scan`、`freeze_map`、`load_map`。
- `/ur7e_real_static/confirm_scene`、`execute`、`stop`、`invalidate`。
- `/ur7e_real_static/status`、`preview_trajectory`、`markers`。

默认 `execution_enabled=false`、`commissioned=false`。静态地图必须保持场景有效；场景或模型变化后不能沿用旧冻结地图。

独立 RTDE 模式要求与旧 UR ROS 控制驱动明确交接，不能同时控制；External Control 程序不需要启动，历史测试使用 Remote Control 模式。若仍选择 ROS Action 执行，则需先退出独立 RTDE 控制，再按 ROS 驱动流程启动 External Control。这两条流程不能混用。

## 7. 推荐后续任务顺序

### P0：统一自动预扫描 ESDF

- [ ] 以真实静态包的感知/规划为主干，复用自动包的 8 点调度与 RTDE；保留原手动和彩色重建入口。
- [ ] 为 ESDF 增加/对接逐点采集门、计数、会话/epoch、冻结完成确认；不要直接复用彩色重建服务的名称并假设数据语义相同。
- [ ] 接通独立 RTDE 关节反馈与校准 TF；统一相机时间、坐标系与模型指纹。
- [ ] 自动扫描结束后保存可查询的规划 ESDF、未知/来源、扫描姿态与配置；禁止把 `surface.npz` 网格当作 `frozen_map.npz`。

### P1：解决“地图可用于规划”的前置条件

- [ ] 校核实际碰撞包络及相机/腕部自碰撞模型。
- [ ] 检查扫描末端姿态这一真实 A 点的整臂自由空间、目标 B 的 IK、候选路径全臂覆盖；不满足则补扫描。
- [ ] 对不可见固定近场，如确需自由区，必须来自独立测量/确认且范围固定，保留来源；禁止将未知区整体放开或随运动自动扩张。
- [ ] 先在真实冻结地图上获得 PLAN_READY，并在 RViz 审核路径，执行仍禁用。

### P2：接入经过验证轨迹的 RTDE 执行

- [ ] 规划仍使用 SAGE；把最终检查和时间参数化后的轨迹接到执行器，不用未经检查的末端雅可比运动替代避障规划。
- [ ] 验证重采样/插值没有走出已经检查的路径；保留起点一致性、速度/加速度、反馈、跟踪、取消/停止与控制权互斥检查。
- [ ] 无硬件条件下测试全流程、地图不足/未知、目标不可达、场景失效、断流、取消和执行器故障。

### P3：用户批准后再做真机分阶段验收

- [ ] 先复测修复后的自动扫描，要求每点帧数达标、最终成功冻结 ESDF。
- [ ] 再做只规划预览，随后批准低速短距离 A→B，最后跨障碍绕行；记录实际轨迹、目标误差、净空、停止情况和现场观察。

**当前仍有效的用户限制：修复后先不进行真机测试，用户检查后决定。下一任务窗口不得因接到这份交接就自动连接/移动机械臂。**

## 8. 可复制给下一任务窗口的任务说明

> 请先阅读 `/home/wqj/realtime_nvblox_ws/HANDOFF_REAL_ESDF_SAGE.md`。我的最终目标是：UR7e＋D435i 眼在手自动预扫描真实静态场景，建立并冻结 base_link 下 ESDF，然后由 SAGE 规划并低速执行真实 A→B 避障到达。请以 ur7e_real_static 的 ESDF/SAGE 为主干，复用 ur7e_auto_reconstruction 已修复的自动扫描与 RTDE 控制，保留原手动和彩色重建功能。当前自动扫描只接彩色 TSDF，尚未接 ESDF；真实规划仍存在起点自由空间覆盖和相机/腕部球模型问题；现有静态包执行端还是 ROS Action，RTDE 规划执行尚需集成。请先完成软件连接和离线闭环验证，不启动真机测试、不发送运动指令，等待我检查后再决定实机验证。不要修改 realtime_nvblox 和 realtime_nvblox_2 原包。
