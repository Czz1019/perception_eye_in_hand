# UR7e clean SAGE 仿真规划与伺服功能包

复制并整理原 clean_SAGE 的规划、部署修正、伺服与记录代码，原目录不变。
仅通过 `whole_control` 环境调用已安装的 `storm_kit`；其它所需代码、配置、
规划 URDF、网格、碰撞球和权重都包含在本包中。Gazebo 场景由同工作空间的
`ur7e_d435_gazebo` 提供。

## 编译和启动

```bash
cd /home/wqj/realtime_nvblox_ws
./build_ur7e_sage_planner.sh
./run_ur7e_sage_planner.sh
```

默认同时启动 Gazebo、外侧 D435 高墙场景、RViz、规划和伺服。
若场景已经启动，使用 `./run_ur7e_sage_planner.sh start_scene:=false`。
关闭 RViz 可追加 `rviz:=false`，关闭 Gazebo GUI 可追加 `gazebo_gui:=false`。

标准 ROS 入口：

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/local_setup.bash
ros2 launch ur7e_sage_planner planning.launch.py
# 只运行规划/伺服，连接已启动的场景：
ros2 run ur7e_sage_planner sage_planner
# 不连接 Gazebo，只检查规划器初始化及一次规划：
ros2 run ur7e_sage_planner sage_planner --entry smoke
```

`sage_planner` 自动选择 `$HOME/miniconda3/envs/whole_control/bin/python`；
其它 conda 安装路径通过 `SAGE_CONDA_PREFIX` 指定。Gazebo/RViz 使用系统 ROS Python。
系统 ROS 依赖见 `package.xml`；PyTorch、CUDA、storm_kit 及其 Python 依赖沿用
已有 whole_control 环境，不会在构建过程中重新安装。

## 控制与配置

链路为 `SAGE_MPPI q_next → JointServoExecutor → GazeboBackend →
/forward_position_controller/commands → ros2_control`。
默认伺服 200 Hz、最大关节速度 0.6 rad/s、不启用加速度限制；规划周期为配置中的
0.05 s。保留 `execution_layer:=direct` 选项和原部署修正参数。

`config/ur7e_reacher_gazebo_tall_sage_clean.yml` 为规划配置，资源使用 `package://`
路径，在运行目录生成绝对路径副本后交给 storm_kit；可通过 `SAGE_TASK_FILE` 指定配置。
运行日志默认位于 `~/.ros/ur7e_sage_planner/<时间_进程号>/`，可用 `SAGE_RUN_DIR` 覆盖。
其中 `servo/servo_execution_log.csv` 包含 `q_next`、`q_cmd`、`q_actual`、`dq_actual`。
storm_kit 的临时控制器文件也写入该运行目录。

默认目标 `[0.4, -0.5, 0.4]`（world）。发送新目标：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.5, y: 0.0, z: 0.45}, orientation: {w: 1.0}}}"
```

保留 `/ee_pose`、候选轨迹、碰撞球、目标等 RViz 话题。场景包负责生成障碍物，
规划器默认不重复生成。连接空场景时可通过 `planner_args:='--spawn-obstacles'` 生成原高墙障碍物。
规划碰撞模型沿用原机械臂碰撞球，未增加相机和支架的碰撞球，也未接入 nvblox 动态地图。

## 验证与记录

在规划开始时运行以下命令，检查到达默认目标后再切换第二个目标（会发出运动目标）：

```bash
ros2 run ur7e_sage_planner verify_planning --retarget --output /tmp/sage_planning_check.json
```

检查两个目标均连续 10 帧小于 2 cm、实际关节发生运动、控制指令有限且持续发布。
完整的关节平滑性记录可单独运行，按 Ctrl-C 后输出 CSV、图像和报告：

```bash
ros2 run ur7e_sage_planner sage_planner --entry record --output-dir /tmp/sage_joint_records
```

2026-09-18 实测通过：

- colcon 构建、whole_control/CUDA 离线规划、Gazebo 与 RViz 启动。
- 默认目标和第二个目标均连续 10 帧小于 2 cm；验证结束时误差分别为
  1.76 mm、6.00 mm。实际关节运动幅度（六关节向量 L2）超过 2.46 rad。
- 收到 5865 条控制指令，发布频率中位数 199.77 Hz；CSV 验证最大指令速度
  0.6 rad/s、最大单步 0.003 rad，规划输出、伺服输出和关节反馈列均正常。
- 使用 `strace -f -e trace=openat,execve` 跟踪启动与子进程，STORM 目录中的成功
  文件读取全部位于 `storm_kit` 内，没有读取其 examples/content/weights。
- 原 clean_SAGE 目录全部 63 个文件的 SHA256 与迁移前一致，没有新增或删除文件。

摘要见 `validation/*.json`。完整启动日志、文件访问记录、伺服 CSV 和界面截图位于
`/home/wqj/realtime_nvblox_ws/log/sage_package_validation/`。
这是当前初始姿态和两个目标的功能验证，不是所有目标或轨迹的成功率评估。

退出检查：规划器正常退出；其后台优化进程由原有超时清理逻辑终止。
随后关闭 Gazebo 时 `gzserver` 返回 `-11`。这一退出异常仍存在于当前场景运行环境中，
未在本次规划功能包迁移中修改 Gazebo/相机插件；运行期间的规划、伺服和目标切换检查通过。
