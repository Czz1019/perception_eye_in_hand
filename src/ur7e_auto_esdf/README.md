# ur7e_auto_esdf：真机自动预扫描 ESDF

UR7e＋D435i 眼在手，**8 个观察点 → 独立 RTDE 运动 → 停稳采集深度 → TSDF / 原生 ESDF → 冻结规划 ESDF**。不调用 SAGE、不执行目标避障任务、不输出彩色 Mesh。

2026-10-13 已完成软件实现和离线验证：28 项单元/共享控制测试、12 类故障测试通过；模拟 RTDE＋合成深度＋真实 CUDA 完成 8 点/64 帧；保存地图可用 `ur7e_real_static.field.FrozenMap.load` 校验加载。实际 launch 的冻结回看、晚订阅点云、独立 TF 校验及退出通过。

同日真机第二轮完成 **8 点/64 帧、DONE**，地图重载校验通过，用户反馈“平稳”；含预热约 196 秒。第一轮 RTDE 脚本启动超时原因仍待定位。**扫描成功不代表可直接执行避障，planning_ready 仍为 false。** 见[真机测试报告](report/2026-10-13_real_auto_esdf_test.md)。

## 架构与复用

- 复用 `ur7e_auto_reconstruction` 原有 8 点完整关节目标与校准模型指纹。进入第一点的轨迹仍从实际起点重新生成。
- 共享原包 `transport.py` 的 RTDEReceive、独立 RTDEControl、伺服线程、控制权检查和保护，不改原包。
- 调度/轨迹校验从修复后的自动重建包移植：到点连续停稳后停止伺服、停止脚本、断开控制，再次核验静止后才采集；采集期间没有 500 Hz 伺服线程，下一段从实际反馈起步。
- ESDF 工作者从 `ur7e_real_static.capture` 适配，只增加采集门、lease、图像时间边界及逐点记录；保留图像时刻 TF、关节插值、自身过滤、有效深度和过期结果检查。
- 共享 `EfficientBackend`、`FieldBuilder`、`FrozenMap`。原生 ESDF 由 TSDF 更新；规划地图同时结合观测射线自由空间及未知边界，未知/越界不视为安全空间，不引入仿真先验。
- 建图与控制分进程；取得控制权前预热 GPU。预热使用丢弃的临时地图，不计入扫描。

原手动 ESDF、手动彩色重建、自动彩色重建入口均保留。

## 输出与坐标

所有地图固定在 `base_link`，无需 SLAM。用工厂校准模型及实际关节反馈计算相机位姿；深度相机外参包含 color→depth 内部变换。

| 文件/话题 | 内容 |
|---|---|
| `scan_*/frozen_map.npz` | 可供后续 SAGE 静态规划加载的保守距离场、来源/未知、模型身份与校验和 |
| `scan_*/native_esdf.npz` | 配置查询网格上的原生 nvBlox 距离，含 unknown 标记；用于显示/分析，不代替保守规划地图 |
| `scan_*/summary.json` | 帧数、视角、自由空间、当前整臂净空与自身模型间距；`planning_ready=false` |
| `scan_*/observations/` | 每个停靠点一份深度米值、图像内参、实际关节角、图像时刻相机位姿 |
| `plans/run_*/` | 预览、实际逐点结果、分段控制记录；成功要求 `result.json` 为 DONE |
| `/ur7e_auto_esdf_mapper/surface_esdf` | 原生近表面 ESDF，RViz 颜色表示距离，不是场景 RGB |
| `/ur7e_auto_esdf_mapper/esdf` | 保守规划自由空间距离点云 |
| `/ur7e_auto_esdf_mapper/capture_status` | 门控、有效帧、等待原因、计算耗时与初始化状态 |
| `/ur7e_auto_esdf_scan/status` | 控制状态；采集时 `servo_active=false` |
| `/ur7e_auto_esdf/joint_states`、`/ur7e_auto_esdf/tf*` | 独立 RTDE 反馈与校准 TF，不占用原包通道 |

输出根目录默认 `~/nvblox_output/ur7e_auto_esdf`。每次扫描创建新会话，旧数据不覆盖。每点要求至少 8 个有效完整 ESDF；默认全局至少 15 帧、3 个不同视角才允许冻结。

## 构建和运行

工作区根目录：

```bash
./build_ur7e_auto_esdf.sh
./run_ur7e_auto_esdf.sh
```

默认仅 RTDEReceive 接收反馈、显示预览，不创建控制接口。应用脚本使用 `whole_control`；相机驱动独立运行。本包不会自动启动 UR ROS 驱动或 RealSense 驱动。

显式扫描需先 `./ur7e_auto_esdf_control.sh prepare`，检查当前起点进入段，再按 [真机测试说明](REAL_TEST_GUIDE.md) 以审核后的 plan 启用运动、调用 start。

仅回看已保存地图，不连接 RTDE：

```bash
./run_ur7e_auto_esdf.sh run_controller:=false load:=/absolute/path/scan_xxx/frozen_map.npz
```

## 重要边界

8 个点已在此前彩色扫描中到达，但它们并非任意现场都合适，也不证明整臂近场/规划通道覆盖充分。保留用户此前确认的上方空旷扫描条件，不做环境避障或完整自身碰撞路径检查。冻结成功不等于可以直接执行避障；当前相机/腕部球模型问题仍需后续处理。

当前按独立 RTDE 模式设计：**示教器 Remote Control、External Control 程序保持停止、旧 UR ROS 控制驱动退出**。机器人还需处于正常可运动状态，相机输入和标定需匹配；仅切换远程模式不等于软件会自动开始运动。

详见 [实现与计划](ToDoList.md)、[离线验证](validation/RESULTS.md)。
