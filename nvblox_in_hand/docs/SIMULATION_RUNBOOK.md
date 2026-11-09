# 仿真运行手册：RViz 闪退修正与机械臂状态检查

2026-11-04重构提示：本文保留此前结构/运行记录。当前默认已改为官方NvbloxNode组件，入口、配置及显示选择见 [重构分析](WORKSPACES_EXAMPLE_REFACTOR_20261104.md) 和 [最新操作说明](OPERATIONS.md)。此前inside-only默认、20字节点云及旧频率不能当作新版接口或结果；本轮未运行测试。

2026-11-04补充：当前三维ESDF默认已改为 `/nvblox_node/esdf_points` 点云，使用 `./rviz.sh` 查看；Mesh使用 `./rviz.sh mesh`。本文保留早期闪退诊断，最新显示修改、实测频率和启动参数见 [ESDF点云优化说明](ESDF_POINTCLOUD_OPTIMIZATION_20261104.md)。不要加载历史artifacts中的方块配置来验证新版点云。

更新：2026-10-28。工程目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。本手册只针对本工程 Gazebo 仿真。

## 1. 本次日志给出的结论

两次启动都在 RViz 收到地图后出现：

```text
Unable to locate geometry program called rviz/glsl150/box.geom
terminate called after throwing an instance of 'Ogre::ItemIdentityException'
rviz2 ... exit code -6
```

本机 `/opt/ros/humble/share/rviz_rendering/ogre_media/materials` 只有 glsl120/scripts120，没有插件要求的 glsl150/scripts150。插件的 Mesh 和 VoxelBlockLayer 共用 visual 构造函数，即使只开启 Mesh，也会初始化体素材质并请求缺失 shader。所以仅关闭体素显示不能解决这次闪退。OpenGL 4.6 可用不代表对应 shader 资源已经安装。

日志中的 mapper 在 RViz 退出后仍继续更新；两个控制器均已成功激活，初始值也为本项目配置。没有证据表明是 mapper 异常或控制器激活失败导致 RViz 退出。退出前短暂看到的机械臂状态不足以证明 Gazebo 真实关节错误。

日志末尾的 `rcl_shutdown already called` 是用户 Ctrl+C 后 Python 节点重复清理的问题，已修正；Gazebo 的 -11 发生在 Ctrl+C 停机阶段，应与启动数秒后的 RViz -6 区分。此次没有据此修改物理参数或关节初值。

## 2. 本轮修正

- `env.sh` 检测缺少 GLSL150 资源时，启用上游插件自带的 `ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL`。统一入口与单独 rviz.sh 都会加载。
- Mesh 仍为原生增量三角网格、颜色和几何不变。降级影响插件体素显示：它改为点显示，不能用屏幕点大小判断真实体素精度。
- 新增 **Official RGB voxel cubes (Humble safe)** 和 **Official TSDF voxel cubes (Humble safe)** 标准 Marker 显示，保留原生体素立方体尺寸。需要看体素时优先用它们。
- RobotModel 明确采用 /robot_description、Reliable/Transient Local、无 TF Prefix、Update Interval=0，显示 visual 而非 collision；Alpha 从半透明改为不透明，便于看清关节。这是显示配置，不是关节角修正。
- 新增 `check_sim.sh`，只读取机器人描述、关节、时钟和 TF，对比同一时间戳的关节正运动学与各连杆/相机 TF，检测重复状态源；不发布关节命令。
- 修正 robot_pose.py 的 Ctrl+C 清理，使用 try_shutdown，避免重复 shutdown 报错。

RViz 的 RobotModel 通过 TF 更新各 link，并不直接从 joint_states 绘制，见 [Humble RobotModel 源码](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp)。因此应检查 joint_states → robot_state_publisher → TF → RViz 整条链，不应为了让外观好看而给 Gazebo 强制发送另一组角度。

## 3. 先停止旧运行

在旧扫描、旧感知和旧场景的原终端分别 Ctrl+C。不要同时启动父工程和本工程的同一机器人场景，不要开 joint_state_publisher_gui 来覆盖 Gazebo 的真实反馈。

检查旧服务端：

```bash
ss -ltnp 'sport = :11457'
```

若有监听，先确认 PID：

```bash
ps -p 这里填实际PID -o pid,ppid,comm,args
```

确认是已不用的旧 gzserver 后才使用 `kill 实际PID`。不要原样粘贴占位文字，不要无差别 pkill 全部 ROS/Gazebo 进程。只关闭 Gazebo 窗口不保证服务端停止。

## 4. 更新构建（一次）

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./build.sh
./test.sh
```

构建应显示 ur7e_d435_gazebo 和 nvblox_in_hand 两包完成。RViz 配置和 Python 节点是安装副本，修改后应重新构建，不能只加载旧 install。

本轮无需安装另一套 shader，也无需重新编译整个官方 CUDA 核心或改 workspaces。

## 5. 终端 A：先启动仿真和感知

第一次排查推荐暂时不自动打开 RViz，先确认真实反馈：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/config/robot/cyclonedds_local.xml
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run.sh gazebo_gui:=true run_rviz:=false
```

保持终端 A 运行。等待 Gazebo 生成机器人、激活 joint_state_broadcaster 和 forward_position_controller，处于播放状态。该命令默认不移动机械臂，也不会自动执行扫描。

初始关节配置按下列顺序为 `[0, -1.57, 1.1, -1.57, -1.57, 0]` rad：shoulder_pan、shoulder_lift、elbow、wrist_1、wrist_2、wrist_3。实际反馈可有数值误差。运行过轨迹后不应再要求它等于初值，也不要为了恢复画面直接发送该组位置。

## 6. 终端 B：确认机器人状态后打开 RViz

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
./check_sim.sh --timeout 30
```

通过时返回 `"result": "passed"`，并显示 `"motion_commands_sent": 0`。报告保存到：

`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/robot_state_check.json`

检查内容：

- 六个关节都有有限值，关节反馈和仿真时钟持续更新。
- /joint_states 与 /robot_description 各只有一个发布端点。
- 读取同一关节时间戳的肩、肘、腕、tool0、深度/颜色光学坐标系 TF，与实际 URDF 和关节值计算的 FK 对照。
- 平移差大于 2 mm 或旋转差大于 0.01 rad 时报告失败；运行前应让机械臂保持静止，避免快速运动中的采样插值影响判断。

这是关节/TF 链路一致性检查，不是独立测量 Gazebo link 真值，也不能自动证明每个 mesh 的 visual origin 正确。若检查通过而画面仍明显不同，应再对比视角、RobotModel 状态、URDF/mesh 与 Gazebo 的实际 link，而不是改角度掩盖现象。

通过后打开：

```bash
./rviz.sh
```

兼容设置检查：

```bash
echo "$ISAAC_ROS_NVBLOX_PLUGIN_FORCE_FALLBACK_MATERIAL"
```

在本机缺少 GLSL150 资源时应为 1。也可显式指定：

```bash
IN_HAND_RVIZ_MATERIAL_MODE=fallback ./rviz.sh
```

不要在当前缺少资源的环境使用 native 模式。fallback 是材质选择，与体素精度、深度融合、Mesh 和 ESDF 算法参数无关。

## 7. RViz 中如何看

Fixed Frame 选择 `base_link`。展开 **UR7e and D435**，确认 URDF 和各 link 的状态为正常；Description Topic 应为 /robot_description，TF Prefix 为空，Update Interval 为 0，Visual Enabled 开启，Collision Enabled 关闭。

选择 Move Camera（M），左键拖动旋转、中键平移、滚轮缩放。Views 选择 Whole obstacle group 或 Top overview。不同观察方向会让同一关节姿态看起来不同，先统一视角再对照 Gazebo。

推荐顺序：

1. UR7e and D435：观察机器人外形和各 link 变换。
2. Official incremental RGB Mesh：看重建表面；保持其开启才能请求官方 Mesh 更新。
3. D435 RGB input：确认相机实际看见障碍物。
4. Observed native ESDF 3D band：可选近表面三维距离带，内部红、外部绿到蓝，显示默认最多 1 Hz；不是 ESDF 计算频率。
5. Official RGB/TSDF voxel cubes (Humble safe)：需要看真实体素尺寸时选它们；默认关闭，避免额外开销。

3d 模式下官方 static_esdf_pointcloud 不发布二维切片，这是当前版本行为。需要二维投影时先停止本轮，再以 `esdf_mode:=2d` 启动，不能把缺少切片误判为没有三维 ESDF。

## 8. 终端 C：启动扫描

A 持续运行，B 的 RViz 正常后：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
./orbit_scan.sh --check-only
```

预检查 ready 后，由用户执行运动：

```bash
./orbit_scan.sh
```

可选重新规划并每视角至少停留 4 秒：

```bash
./orbit_scan.sh --replan --dwell 4
```

不能与其他关节命令发布者同时运行。默认固定场景只保证检查过的可达视角，不承诺完整 360°。退出扫描不会自动回起点；Ctrl+C 后控制器保留最后目标。报告位于 artifacts/orbit/。

## 9. 终端 D：输入、频率与状态

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
source ./env.sh
ros2 node list --no-daemon
ros2 topic info /joint_states --verbose
ros2 topic info /robot_description --verbose
timeout 10s ros2 topic echo /joint_states --once --qos-reliability best_effort
timeout 10s ros2 topic echo /nvblox_node/metrics --once
timeout 10s ros2 topic echo /nvblox_in_hand/pose_health --once
timeout 15s ros2 topic hz /camera/depth/image_rect_raw --wall-time
timeout 15s ros2 topic hz /nvblox_node/mesh --wall-time
```

频率查询逐条运行，timeout 停止采样正常。Metrics 的融合墙钟频率与官方控制台 ROS 时间频率不同；Mesh 是增量输出频率，不等于每秒融合帧数。更多服务、保存地图、1 cm 体素、官方后端和二维模式见 OPERATIONS.md。

## 10. 已确认正常后的一条指令启动

以后可直接：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
./run.sh gazebo_gui:=true
```

同样会自动使用材质兼容设置。但第一次排查仍推荐按 A/B/C/D 分步，以便分清场景、状态、显示和运动问题。

## 11. 退出顺序与验证边界

先停止扫描，再关闭 RViz，最后 Ctrl+C 停止终端 A。Gazebo 停机若仍出现 -11，应保留退出日志单独排查；它与这次 shader 缺失导致 RViz 启动闪退是两个阶段的问题。

本轮构建、离线检查与独立域离屏 Mesh 回归的结果记录在 artifacts/rviz_fix_20261028/。离屏测试中的三角网格仅用于触发显示插件，不是重建效果图。本轮没有启动 Gazebo、连接真实机械臂或执行扫描，因此不能宣称机械臂实际姿态与重建质量已经完成在线验收。

本轮验证结果：默认构建成功；13 项 Python 离线测试和 C++ 显示策略测试通过。虚拟 X 显示、独立 ROS 域 129 中，RViz 接收 48 条测试 Mesh，持续约 10 秒后主动正常退出（0），未再出现 box.geom 异常。测试依赖只解包在 .test_runtime/，没有安装到系统。该结果验证了材质兼容路径，不代替用户桌面上的完整场景验收。
