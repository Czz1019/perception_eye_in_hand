# 自动真机扫描：首次测试说明

## 1. 使用边界

仅针对已经确认上方空旷、目标静止的场景。不使用 ESDF/SAGE 避障。8 个观察点来自离线 IK。2026-09-29 第二轮已在真机依次到达，但 S08 采集期间因控制周期超时而终止，完整扫描尚未通过；详见 [测试报告](report/2026-09-29_real_auto_scan_test.md)。

旧相机/手腕球体模型未通过完整自身间距检查，尚无经核实的完整自身碰撞保证。先审核进入段以及全部关节运动范围；保留示教器停止操作。软件停止服务不是安全认证的急停。

## 2. 相机与机器人准备

1. UR7e IP 为 `192.168.56.100`，使用本机 `/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf`。工厂校准来源 `/home/wqj/ur_arm/my_robot_calibration.yaml`；外参沿用已验证的 `tool0 → camera_color_optical_frame`。
2. D435i 驱动保持运行：默认彩色 `/camera/camera/color/image_raw`，深度 `/camera/camera/depth/image_rect_raw`，对应 CameraInfo。若尚未启动，可运行原包 `hardware/start_camera.sh`（已有驱动时不要重复启动）。
3. 示教器的 **External Control 保持停止**。本包使用独立 ur_rtde 上传脚本，不要求重新播放 External Control。
4. **退出电脑上的 UR ROS 控制驱动进程/launch**，并停止其他 SAGE/RTDE 控制程序。只停止示教器程序不足以完成交接；即使 ROS 运动控制器 inactive，本包也拒绝与 `ur_ros2_control_node` / `ros2_control_node` 同时运行。不要关闭相机驱动。
5. UR 处于可接受外部脚本的远程控制状态、上电且制动释放；正确配置现场负载/TCP及机器人安全设置。本包不会修改负载、安全参数、自动上电或解除保护停止。

自动包有自己的 RTDE 接收、JointState 和校准 robot_state_publisher，所以退出旧驱动后仍可获取重建所需 TF。TF 位于 `/ur7e_auto/tf*`；相机节点的全局 TF 不参与自动包的机器人位姿链。

## 3. 只读预览

```bash
cd /home/wqj/realtime_nvblox_ws
./run_ur7e_auto_reconstruction.sh
```

另一终端（工作目录仍为 `/home/wqj/realtime_nvblox_ws`）：

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/local_setup.bash
./ur7e_auto_scan_control.sh prepare
```

阅读返回目录的 `PREVIEW.md` 和 `route.png`，特别是橙色进入段。`plan.json` 包含实际起点关节角、各段相机路径、持续时间、模型哈希和参数。

2026-09-29 的只读快照中，相机高度约 **0.342 m**；进入 S01（0.700 m）标称约 **14.0 s**，腕一变化约 **51.5°**。这是关节插值曲线，不能当作纯竖直抬升。8 段标称总运动约 **94.1 s**，另加停稳、采集、GPU 初始化/导出和速度缩放。此数字只对应那次起点快照。

如果现场进入段不合适，先手动移到合适的高位起点，再重新预览。不要执行旧选点目录中的被拒绝参考抬升路径。

控制辅助脚本与启动脚本使用一致的干净 ROS 环境，避免继承旧 Gazebo/Conda 库路径；它调用的仍然是上表的标准 ROS 服务。

## 4. 显式执行

退出刚才的只读自动包，确认机器人仍停在审核过的起点。重新启动：

```bash
./run_ur7e_auto_reconstruction.sh \
  allow_motion:=true \
  approved_plan:=/absolute/path/to/reviewed/plan.json
```

上述命令只是允许后续开始，**不会自动运动**。等待相机/反馈正常后，在另一终端：

```bash
./ur7e_auto_scan_control.sh start
./ur7e_auto_scan_control.sh status
```

流程：核对预览 → 新建独立采集会话 → WARMING_UP（丢弃临时 CUDA 预热地图）→ 再校验实际起点 → 显式建立 RTDEControl → 检查新脚本与反馈 → 进入 S01 → 停稳 → STOPPING_AT_VIEW（停止伺服/脚本并断开控制）→ VERIFYING_STATIONARY（再次核验静止）→ 采集至少 8 帧 → 关采集门 → 从实际反馈重新取得控制权并执行下一段 → S08 采集结束 → 冻结导出地图 → DONE。

控制脚本停止后的静止采集阶段允许速度缩放为零，持续检查反馈是否新鲜、位置是否漂移及机器人安全状态。每段控制脚本启动后速度缩放仍为零则超时退出；运动中为零则停止。运动中的速度缩放渐变影响执行进度；没有“补赶遗漏轨迹”的瞬间跳点。运行后不自动返回起点。

## 5. 停止与故障

```bash
./ur7e_auto_scan_control.sh cancel
```

等待 `CANCELLED` 或 `ERROR`，检查反馈已经停稳。正常可响应情况下执行 `servoStop`、`stopScript`、断开控制；网络/线程失效还有控制端 10 Hz watchdog。伺服期间由 servoJ 更新 watchdog 所监视的输入寄存器，不在每次 servoJ 后再调用 kickWatchdog；显式 kick 只用于尚未伺服的脚本启动阶段。软件停止失效时使用示教器/现场硬件停止。

取消或故障后，程序尽力保存部分地图，并记录 `result.json`；只有导出成功后才有对应 `surface.npz`。采集帧不足、相机断流、TF 不匹配都会终止该轮，不会跳过未完成的点宣称扫描完成。修复原因后重新 prepare，从当前实际位置审核新进入段，不会自动续跑。

## 6. 核对结果

- `/ur7e_auto_scan/status` 必须为 DONE，`result.json` 中 8 个点各有至少 8 帧。
- `scan_*/observations/001…008/` 有 RGB、depth.npz、pose.json。
- `surface.ply` / `surface.npz` 为彩色重建，base_link 坐标系；没有 ESDF。
- RViz 观察表面颜色与几何；与现场比对。不可把可见表面之外区域视为已验证自由空间。
- `servo_trace.npz` 比较 command / actual / velocity、跟踪误差和控制周期。新增 `segment_index`；只在同一运动段内计算相邻周期，段间间隔是停止采集，不属于伺服漏周期。`result.json` 的 `motion_segments` 包含每段周期耗时及故障信息。
- 采集时状态应为 `servo_active=false`、`capture_stationary_guard=true`。S08 仍需至少 8 帧；最终须为 DONE。
- 本次采集阶段修复仅经过离线验证，须待用户检查后决定真机复测；每点脚本重启与再次停稳增加的耗时、接续平稳性尚需真机验证。

单独回看已保存地图，不连接 RTDE：

```bash
./run_ur7e_auto_reconstruction.sh run_controller:=false \
  load:=/absolute/path/to/scan_xxx/surface.npz
```

恢复旧手动扫描：退出自动包，再按原手动包流程启动 UR ROS 驱动及示教器 External Control；原包代码和历史文件保持不变。
