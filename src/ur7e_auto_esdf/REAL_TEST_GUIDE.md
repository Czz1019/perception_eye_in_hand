# 自动 ESDF 真机扫描说明

2026-10-13 已完成首次真机测试：第二轮 8 点/64 帧、DONE，用户反馈“平稳”；第一轮脚本启动超时尚待定位。详见[测试报告](report/2026-10-13_real_auto_esdf_test.md)。以下操作用于后续测试，每次仍须从实际起点重新预览。

## 1. 模式和控制权

1. 本机网络应能访问用户确认的 UR7e `192.168.56.100`。
2. 示教器设为 **Remote Control**；**不要启动 External Control**。本包由独立 ur_rtde 上传控制脚本。
3. 退出电脑上的旧 UR ROS 控制驱动/launch、旧自动扫描控制节点和其他 SAGE/RTDE 控制程序；只把 ROS controller 设 inactive 不够。与旧自动重建包共享 RTDE 控制锁。
4. 机器人上电、制动释放、状态正常，正确的负载/TCP/现场安全设置保持有效；本包不自动修改这些设置或解除保护停止。
5. 相机和安装位置须与既有标定一致；默认 URDF 是 `/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf`，碰撞球是该目录的 `collision_spheres_preview.yaml`。
6. 原观察点只针对已确认上方空旷的扫描路线；现场变化后需重新检查。不得把复用 8 点理解成已经进行障碍避让。

D435i 驱动需要独立运行。已有相机驱动不重复启动；需要时可按原 `src/ur7e_real_reconstruction/hardware/start_camera.sh` 启动，先检查其中设备设置。默认输入为 `/camera/camera/depth/image_rect_raw` 和 `/camera/camera/depth/camera_info`。

## 2. 只读预览

```bash
cd /home/wqj/realtime_nvblox_ws
./build_ur7e_auto_esdf.sh
./run_ur7e_auto_esdf.sh
```

另一个终端：

```bash
cd /home/wqj/realtime_nvblox_ws
./ur7e_auto_esdf_control.sh prepare
```

prepare 输出 `plan.json` 路径，并生成 `PREVIEW.md`、`route.png`、`arm_segments.png`。重点检查实际起点到 S01 的进入段；关节插值对应的相机路径不保证是直线。起点、模型、路线或控制参数变化后必须重新 prepare。

默认状态 PREVIEW_ONLY，不上传控制脚本。此时只有只读 RTDE/TF 和相机监听，不融合扫描数据。

## 3. 显式自动扫描

关闭上述预览 launch，确认机器人仍在审核过的起点，启动：

```bash
./run_ur7e_auto_esdf.sh allow_motion:=true approved_plan:=/absolute/path/to/plan.json
```

该命令允许后续启动，仍不会立即运动。另一个终端：

```bash
./ur7e_auto_esdf_control.sh start
./ur7e_auto_esdf_control.sh status
```

流程：新会话 → GPU 预热 → 实际起点复核 → 进入 S01 → 停稳 → 释放控制 → 再次确认静止 → 采集至少 8 个有效 ESDF → 关门 → 下一点 → S08 采集完成 → 保存冻结地图 → DONE。结束停在 S08，不自动返航。

默认扫描运动目标频率 500 Hz，最大速度 0.12 rad/s，加速度参数 0.18 rad/s²。采集上限 5 Hz，原生体素/规划查询间距均 2.5 cm，RViz 上限 1 Hz；这些不是实机持续完成率保证。

## 4. 停止和结果检查

```bash
./ur7e_auto_esdf_control.sh cancel
```

等待 CANCELLED 或 ERROR，再检查机械臂确已停止；软件停止不能代替现场硬件停止。取消/错误不自动续跑。帧数或视角不足时不伪造冻结地图：已写的逐点数据保留，部分地图导出可能被拒绝。重新扫描前退出该 launch、修复原因、重新启动并从当前实际姿态 prepare。

成功标准：

- 状态 DONE，`plans/run_*/result.json` 中 8 个点均至少 8 帧；`map` 指向存在且校验通过的 `frozen_map.npz`。
- RViz 原生 ESDF 表面可见，固定坐标系为 base_link；颜色是距离。
- `summary.json` 记录有效自由空间与当前整臂净空。该文件不是通用路径可达证明；规划尚未接入。
- 控制轨迹 `servo_trace.npz` 以 `segment_index` 区分运动段，采集间隔不计入伺服周期统计。

## 5. 后续接 SAGE

冻结格式复用 `ur7e_real_static`；加载时必须使用相同的 URDF、碰撞球、相机 frame、网格和建图参数，身份不匹配会被拒绝。本包自动包的 TF 在私有 `/ur7e_auto_esdf/tf*`，规划端未来接入时也需对应 remap。

旧静态包按其配置的 output_directory 读取 frozen_map.npz，不能把本包的输出根目录当成具体扫描会话目录。使用同一会话的 mapping.yaml 副本并设置具体会话目录，不覆盖历史地图；默认保持 execution_enabled=false。

开始真实规划之前，仍需处理整臂自由空间覆盖和相机/腕部自碰撞模型，不得直接降低安全余量或把未知区清空。
