# 关键代码流程说明

[返回文档索引](README.md)

本文解释主要执行阶段及函数之间的关系。逐文件职责、完整符号和接口位置见各包代码说明。

## 1. 基础实时建图

阅读入口：[runtime.py](../src/realtime_nvblox/realtime_nvblox/runtime.py)、[realsense_source.py](../src/realtime_nvblox/realtime_nvblox/realsense_source.py)、[nvblox_backend.py](../src/realtime_nvblox/realtime_nvblox/nvblox_backend.py)。

| 阶段 | 代码 | 输入 → 处理 → 输出 |
| --- | --- | --- |
| 初始化 | `RealtimeNvbloxRuntime.start` | 启动采集，读标定，构造位姿/建图后端和工作线程 |
| 采集 | `RealSenseSource.start` 与各采集循环 | 视频/IMU → 带设备时间戳的数据结构 → 有界队列 |
| 帧选择 | `EmitterClassifier.is_on` | 优先用投射器 metadata，必要时用配置允许的回退分类，区分适合 VIO/深度的帧 |
| 跟踪 | `_vio_loop`、`CuVslamBackend.track` | 按时间处理 IMU/双目 → 连续位姿 → PoseBuffer |
| 对时 | `_pose_for`、`PoseBuffer.query` | 深度时间 → 插值/允许容差的位姿；robot 模式走 RobotPoseProvider |
| 融合 | `_mapping_loop` | 深度/颜色与对应位姿 → GPU TSDF/颜色 → 周期性 ESDF 与 Mesh 更新 |
| 距离查询 | `_esdf3d_loop`、`query_esdf_grid` | 规则网格与中心 → 原生 ESDF 查询 → 最新输出槽 |
| 发布/停止 | ROS 事件回调、`stop` | 打包结果，保存输出，通知线程结束并清理源 |

Mapper 各操作共享锁，因此独立查询线程主要解决调度与延迟隔离，不能视为多个线程无锁并发访问 GPU 地图。`robot_pose.py` 先插值工具位姿，再施加固定手眼变换，保留相机相对工具的偏置关系。

## 2. ROS 深度与图像时刻 TF 建图

阅读入口：[ur7e_nvblox/node.py](../src/ur7e_nvblox/ur7e_nvblox/node.py)、[depth.py](../src/ur7e_nvblox/ur7e_nvblox/depth.py)、[esdf.py](../src/ur7e_nvblox/ur7e_nvblox/esdf.py)。

`TfMapper._on_depth` 做输入接收与有效性筛选；`_worker` 用 CameraInfo 和图像时间查询变换，解码深度并融合。时钟回退/reset 切换会话，使旧计算结果失效。`_publisher_loop` 从最新结果组织显示消息；`_save_service` 与 `_reset_service` 提供外部控制入口。

`depth_meters` 显式处理 `step`、大小端和编码，深度统一为米；降采样时相机内参同步变化。`known_distances` 判断原生未知哨兵，不能把大幅未知值当作远离障碍。

`EfficientBackend.prepare_query` 将固定 Nx3 坐标搬到 GPU 并预分配输出；`query_prepared` 在每轮查询前先填充未知值，避免复用缓冲保留上轮结果。`observed_esdf_numpy` 通过 TSDF 的正权重挑选候选位置，实际距离仍查询 ESDF 层。

## 3. 彩色表面重建与手动真机采集

阅读入口：[surface.py](../src/ur7e_reconstruction/ur7e_reconstruction/surface.py)、[真机 inputs.py](../src/ur7e_real_reconstruction/ur7e_real_reconstruction/inputs.py)、[真机 node.py](../src/ur7e_real_reconstruction/ur7e_real_reconstruction/node.py)。

`SurfaceMapper.integrate` 分别使用深度和彩色的位姿，调用后端融合深度/颜色；`extract` 更新并提取 Mesh，再按 ROI 保留完整三角形。`surface_cloud` 显示融合后的顶点，不是单帧原始深度；`surface_markers` 用完整快照清理旧块；`save_ply` 原子保存几何与颜色。

真机 `Reconstruction` 的阶段如下：

```text
IDLE / FROZEN / ERROR
    └─ start_scan → STARTING → SCANNING
                                  └─ freeze_scan → FREEZING → FROZEN
```

`on_pair` 将同步器得到的 RGB-D 对加入小容量 pending 队列；`pick_ready` 从新到旧寻找关节和 TF 均能匹配图像时刻的输入。`JointHistory.stationary_at` 要求停稳，`model_matches` 核对 CPU FK 与 live TF。`ColorRectifier.decode` 根据彩色标定矫正图像，本体深度屏蔽后才调用 `mapper.integrate`。

`mapping_loop` 单独处理 start/freeze 命令，冻结时从最后完成融合的 mapper 再提取一次，避免保存旧显示快照。`record_view` 按关节视角分离阈值记录观测档案；不是每一帧都生成一个新的观察点目录。`export` 保存 `surface.npz`、PLY 和 metadata。该节点不取得机器人运动控制权。

## 4. 保守自由空间和冻结地图

阅读入口：[grid.py](../src/ur7e_perception_planning/ur7e_perception_planning_core/grid.py)、[field.py](../src/ur7e_real_static/ur7e_real_static/field.py)、[capture.py](../src/ur7e_real_static/ur7e_real_static/capture.py)。

1. 采集线程取得图像时刻关节、各 link TF 和相机变换，确认停稳和模型一致性。
2. 深度转换为米并过滤本体，过期输入不会因 GPU 初始化或等待重新变成有效输入。
3. nvBlox 更新原生 ESDF，按固定网格查询 `raw`，用未知哨兵得到 `known`。
4. `observed_free` 将网格中心投影到深度图，在回波前留余量、使用最小滤波，形成自由射线证据。
5. `FieldBuilder.update` 将观测证据累计为 source，原生近障碍格覆盖旧自由判断；计算到未知/占据边界的 EDT。
6. 最终距离取 EDT 扣离散化余量与已知原生距离的最小值；非自由格不能拥有正距离。
7. `FrozenMap.create` 校验尺寸、有限值、来源和未知约束，构造只读数组；`save/load` 原子保存并校验 metadata、模型 identity 与 SHA256。

自由集合不变时可以复用 EDT。在线通用 `conservative_field` 允许只对自由扩张短暂延后重算，但新障碍撤销已有自由时立即重算；保守性来自不夸大可用自由空间。真机静态 `FieldBuilder` 的逻辑与通用函数并非完全相同。

## 5. 真机静态目标规划与显式执行

阅读入口：[RealStatic](../src/ur7e_real_static/ur7e_real_static/node.py)、[execution_gate](../src/ur7e_real_static/ur7e_real_static/state.py)、[trajectory.py](../src/ur7e_real_static/ur7e_real_static/trajectory.py)。

`start_scan` 清理旧规划/冻结状态并增加 epoch；`freeze_map` 要求足够视角与有效帧、最新完整结果和一致的相机标定，保存后进入 `MAP_READY`。加载已有地图也不会自动确认当前环境未变化。

`on_goal` 要求地图已冻结、机器人停稳、目标坐标有效；异步规划线程把地图、实测起点和目标交给 `planning.plan`。路线搜索与 SAGE 完成后，`parameterize` 生成各节点速度/加速度为零的五次轨迹，`validate` 做整臂碰撞和运动约束检查。结果进入 `PLAN_READY`，发布预览并保存轨迹。

`execute` 是独立动作：先要求静止历史，再经过 `execution_gate` 和当前 identity 核查；允许的起点微小变化仍会重新参数化、重新校验，最后通过 FollowJointTrajectory Action 发送。`accepted/feedback/finished` 跟踪接收和终态；`cancel/watchdog` 处理失效、跟踪或反馈问题。运动中收到新目标会取消当前运动，要求停止确认后重新发目标，不静默替换正在执行的 Action。

## 6. 自动八点扫描：控制与采集互斥

阅读入口：[自动扫描 node.py](../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/node.py)、[trajectory.py](../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/trajectory.py)、[transport.py](../src/ur7e_auto_reconstruction/ur7e_auto_reconstruction/transport.py)、[自动 ESDF mapper](../src/ur7e_auto_esdf/ur7e_auto_esdf/mapper.py)。

| 步骤 | 主要函数 | 说明 |
| --- | --- | --- |
| 准备 | `Scanner.prepare`、`make_plan` | 从实时 RTDE 起点生成路线和相机轨迹图，保存 plan.json 与哈希 |
| 校验 | `start`、`verify_plan` | 要求 allow_motion、审核计划、有效状态，模型/路线/控制参数与实际起点均匹配 |
| 预热 | `run`、mapper `warmup` | 在创建 RTDEControl 前完成临时 GPU 预热；预热地图不计入扫描 |
| 取得控制 | `acquire_servo`、`RTDEControl` | 采集门须关闭，停稳核查后取得文件锁/独立控制接口，再从实测 q 起步 |
| 运动 | `Segment.sample`、`Servo._run` | 同步五次插值，受速度缩放、周期、反馈、跟踪误差与心跳限制 |
| 到点 | `settle`、`release_servo` | 连续停稳后 servoStop、stopScript、disconnect，确认控制已释放 |
| 采集 | `stationary_state`、mapper `capture` | 再次停稳后开门，lease 有效才采集，累计有效帧；没有高频伺服线程 |
| 下一点/结束 | `run` | 关门核对实际位置后进入下一段；最终冻结/保存并写终态结果 |

五次插值为 `s(u)=10u³−15u⁴+6u⁵`，`q=q0+s(u)(q1−q0)`。段时长同时按峰值速度和加速度约束选择，并预留时间缩放加速度预算；超期不采用跳点追赶。控制阶段与采集阶段允许的速度缩放状态不同：停止脚本后采集时缩放为零并不等于发生运动故障。

彩色版的 `GatedReconstruction` 扩展手动重建器；ESDF 版使用单独的 `EsdfMapper/capture`，复用 `transport` 和冻结地图工具。两者都不执行环境避障搜索，路线计划明确记录未完成环境/完整自身碰撞路径检查。

## 7. 仿真分段闭环与连续优化

阅读入口：[旧版 node.py](../src/ur7e_gazebo_esdf_sage/ur7e_gazebo_esdf_sage/node.py)、[continuous.py](../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/continuous.py)、[optimizer.py](../src/ur7e_gazebo_esdf_sage_continuous/ur7e_gazebo_esdf_sage_continuous/optimizer.py)。

两版都先建立标定模型、执行预扫描并冻结，再由明确策略生成碰撞检查快照。`ownership` 证明冻结姿态下完整体素在本体内；`partial_map` 提供有限未知准入。这些派生结果必须与原始环境观测证据分别解释。

旧版 `planning.route` 先找完整引导，然后逐小段用实际 q/dq 更新 SAGE，检查连接并等待执行。连续版通过 `continuous.reach` 直接以最终末端目标优化，`optimizer.Planner` 常驻进程最多保留一个未完成任务；`compute` 只在新目标 revision 时 reset，后续按 `shift_count` 平移预测分布。

优化结果最多取前四个动作。`checked_prefix` 用当前最新地图逐边检查，后面的边失败仍可保留先前有效前缀；安装新段时锁住指令锚点，防止检查到提交之间伺服推进造成跳变。新计划迟到、反馈/地图过期或当前段失效时保持，不无限外推旧动作。

`background_route` 在停滞时异步搜索；返回路径只作为 SAGE 的可选引导并经新图重检，不直接下发。在线障碍 `online/online_v2` 只由有效自由射线清除，不用 TTL 自动遗忘视野外占据；v2 将感知移到独立进程。仿真测试场景的障碍真值用于独立审计，不应通过测试脚本结构误判为运行感知输入。

## 8. 主要 ROS 接口速查

本表补充循环生成或配置指定的接口；分包文档保留声明表达式。以下节点私有路径按默认节点名展开，launch 重命名/namespace/remap 后应以实际 ROS 图为准。

| 节点/应用 | 主要输入 | 输出与操作 |
| --- | --- | --- |
| `ur7e_real_reconstruction` | 配置指定的深度、彩色、CameraInfo、关节与 TF | `~/mesh`、`~/surface_points`、`~/camera_pose`、`~/status`；Trigger `~/start_scan`、`~/freeze_scan` |
| `ur7e_real_static` | 配置指定深度/CameraInfo/关节/target_topic | `~/esdf`、`~/surface_esdf`、`~/preview_trajectory`、`~/markers`、`~/status` |
| `ur7e_real_static` 操作 | Trigger：`start_scan`、`freeze_map`、`load_map`、`execute`、`stop`、`invalidate` | 均在节点私有路径 `~/` 下；SetBool `~/confirm_scene` 确认/撤销静态场景 |
| `ur7e_real_static` 执行客户端 | `trajectory_action` 指定 Action 服务器 | `control_msgs/action/FollowJointTrajectory`，默认指向 scaled_joint_trajectory_controller |
| `ur7e_auto_esdf_scan` | RTDE 接收数据、mapper `capture_status` | Trigger `~/prepare`、`~/start`、`~/cancel`；`~/status`、`~/route`、`~/capture_lease` 与独立 joint_states |
| `ur7e_auto_esdf_mapper` | 独立关节/TF、相机输入、扫描器 lease | Trigger `~/start_scan`、`~/freeze_scan`、`~/warmup`；SetBool `~/capture`；`~/esdf`、`~/surface_esdf`、`~/capture_status` |
| `ur7e_auto_scan` | RTDE 接收数据、彩色 mapper 状态 | prepare/start/cancel 和 status/route/lease，私有路径对应 `/ur7e_auto_scan/` |
| 两种仿真闭环 `sim_esdf_sage` | `/target_pose`、mapper 反馈及内部地图快照 | `/forward_position_controller/commands`，`~/status`、`~/markers`，Trigger start/stop/invalidate 等 |

自动扫描服务返回“接受请求”不代表完成全部运动/采集；最终应检查 status 和会话终态文件。真实静态轨迹同样需要区分规划成功、Action 接受和执行成功。
