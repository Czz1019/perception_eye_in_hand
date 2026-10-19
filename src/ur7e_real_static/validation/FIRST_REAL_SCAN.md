# 第一次真实场景扫描与预览

2026-09-27。当前结果：**真实多视角扫描完成，1233 帧、21 个观察姿态已冻结为静态 ESDF，RViz 保留显示。当前姿态的规划前置检查不通过，尚未生成真实场景目标轨迹，更未执行真实运动。**

操作员在测试后明确确认：**本次测试全程没有发生实物碰撞**。下文的球模型间距未达到规划余量，不表示发生实体碰撞。

完整图文见 [真实扫描报告](../report/2026-09-27_real_scan_report.md)，包含 21 张由记录关节角重绘的姿态图和最终地图显示；本次未保存逐姿态相机原图。

## 设备和校准

- 机器人 IP `192.168.56.100`，用户确认即本次 UR7e。Dashboard 型号报告 UR5、PolyScope 5.21.2；驱动按用户指定 UR7e 加载。
- 电脑回连 IP `192.168.56.38`，External Control 脚本端口 `50002`。驱动就绪后按用户要求暂停，由用户在示教器启动 External Control；随后日志确认 reverse interface 已连接。
- 工厂校准 `/home/wqj/ur_arm/my_robot_calibration.yaml`，checksum `calib_16570280516586036686`。实际驱动报告 `Calibration checked successfully`。
- D435i SN `349622072343`，固件 5.15.1.55、USB 3.2。真实深度 640×480、Z16/ROS 16UC1、15 Hz；彩色临时保持启用以读取完整彩色光学 TF；未启用 IMU/对齐/驱动点云。
- 用户手眼外参为 tool0→color optical，使用设备读取的 color←depth 内参组成深度位姿；没有把彩色外参直接当作深度外参。
- 实测 `T_color_depth` 的平移为 `[0.014778125897, 0.000028658137, 0.000420058501]` m，四元数 xyzw 为 `[-0.000168599738, 0.002718785265, -0.006862367385, 0.999972743476]`。

组合模型在 `/home/wqj/ur7e_real_setup/calibrated_with_camera.urdf`。仅外部安装 TF tool0→camera_link 由新增静态发布器提供，相机内部链由 RealSense 发布，未给同一 child 增加第二个父节点。

## 实测检查

- 4 秒检查接收 1927 条关节消息和 49 帧深度；样本图像延迟 9.78 ms，0.2～3 m 内有效深度约 81%，中位深度 0.519 m。
- 以缓冲区内相同时间的实际关节反馈核对所有机械臂 link、tool0 和深度 optical TF；最大位置差约 0.015 mm，旋转差约 0.000059 rad。此项是软件模型/TF 一致性，不能解释为实际手眼测量精度。
- 原生 ESDF 点云实收 3593 点，frame_id=base_link，包含 xyz/rgb/distance；RViz 已目视确认机械臂与场景点云可见。点云颜色表示距离，不是场景 RGB。
- 没有自由区先验，`commissioned_free_boxes=[]`；只使用观测获得的自由空间。
- 按用户要求只建模机械臂与相机，不建模夹爪；相机球体依据本机安装的 D435i/D435 描述尺寸生成，支架/工具未做实测包络。夹爪进入画面仍可能进入地图，本配置不能作为真机执行验收模型。
- `execution_enabled=false`、`commissioned=false`，所有运动控制器始终 inactive；没有调用执行服务、发送轨迹或启动自动扫描运动。

## 更新停顿与本轮调整

首先按 15 Hz 上限采集，虽然生成了第一视角地图，但曾测得约 30 秒有效帧不增加，随后恢复。实际读到 CPU 频率间歇降为约 400 MHz；当时相机独立接收测量仍为 14.90 Hz，图像延迟 P50 20.53 ms / P95 22.76 ms。

本轮本机配置改为：

- 采集上限 5 Hz、显示上限 1 Hz；原生体素和查询网格仍为 2.5 cm。
- 驱动 RTDE/原始关节流保持 500 Hz，只增加扫描专用的 50 Hz 关节采样通道 `/ur7e_real_static/joint_states_sampled`。采样直接转发最新真实消息，保留原时间戳和数值，没有伪造反馈。
- 采样实测 50.01 Hz，延迟 P50 6.05 ms / P95 8.58 ms，相邻原时间戳最大间隔 24.1 ms。
- 图像超时仍为 0.5 秒，仍按图像时刻查询 TF、插值实际关节、要求停稳；过期结果拒绝提交。

调整后一次 29.53 秒窗口中有效帧从 154 增到 279，约 **4.23 Hz**；累计观测自由体素 2465、先验 0、视角 1。该窗口没有 capture_error，但此前已记录过期丢帧与停顿。
这只是初次运行中的短窗口观测，不是严格受控性能对比；**未解决电脑降频根因，不保证持续 5 Hz 或 15 Hz。**

## 多视角完成与冻结

用户完成手动预扫描并确认后，调用 `freeze_map` 成功，状态为 `MAP_READY`。有效帧数停止在 1233，观察姿态 21 个，默认 min_views=3 未降低。

- 规划地图：`log/real_first_scan_20260927/output/frozen_map.npz`，保存后以当前模型身份重新读入，校验通过。
- 地图 ID：`dbb20ff70a5e85fcf48f65806cf8dc0c2bc864b5a116930c9fbeb9688003e827`。
- 网格 81×81×57，分辨率 0.025 m；观测自由体素 22329，显式先验 0；所有距离有限，未知来源没有正距离。
- 保存的近表面显示快照：`output/frozen_surface_pointcloud.npz`，12036 点，含原始 PointCloud2 字节和字段元信息。它用于归档显示数据，不是可继续融合的 nvBlox 原生层，也不会由 load_map 自动恢复。
- 21 个观察姿态的相机位置范围：x 0.169～0.477 m、y -0.480～-0.436 m、z 0.458～0.612 m。视角数不等于整个工作区或任意规划通道已覆盖。
- 已在 RViz 确认融合后的范围比单视角扩大；颜色仍为距离。未用标尺/独立真值量测重建误差，不能仅凭融合完成认定手眼精度或无重影已经验收。

## 当前阻碍轨迹预览的两项问题

对当前实际关节姿态只读调用与规划器相同的起点检查（没有发布目标、没有动作请求）：

1. 17 个机械臂/相机包络球的中心查询均没有正的保守距离，不能证明整臂已处于有足够净空的观测自由空间。当前观测自由体素最高约 z=0.55 m，而 tool0 当前高度约 0.668 m。扫描主要覆盖相机朝向的下方区域，仍需针对机械臂附近和目标通道补足观测；不把整个未知区直接写为自由。
2. 相机 `tool0` 球与 `wrist_2_link` 球的模型表面间隙为 6.55 mm，低于 self_margin=15 mm。球模型基于近似外包形状，不能据此认定实物已经碰撞；需要核对/细化相机与腕部碰撞模型，而不是直接降低阈值通过检查。

本轮已经完成“真实扫描 → 静态 ESDF → RViz 感知预览”。“目标轨迹预览”尚未通过，需先处理上述前置条件，再设置明确目标。执行始终关闭；运动控制器再次核对仍 inactive。
扫描期间日志曾记录 External Control reverse 接口断开；关节反馈和 TF 仍可用于只读扫描。本轮没有重新启动机器人程序或激活运动控制器。

## 复现与证据

本机配置、使用说明：`/home/wqj/ur7e_real_setup/README.md`、`preview.yaml`。
完整运行证据和启动脚本在工作区 `log/real_first_scan_20260927/`：

- `driver.log`、`camera.log`、`mount.log`、`preview*.log`、`rviz.log`。
- `robot_driver.urdf`、`camera_calibration.json`、`sensor_checks.json`。
- `controllers_after_scan.txt`、`surface_check.json`、`depth_age.json`、`joint_sample_check.json`。
- `scan_monitor_15hz.json`、`scan_monitor_5hz_no_sampler.json`、`scan_monitor_sampled_startup.json`、`scan_monitor.json`。
- `frozen_checks.json`、`frozen_start_check.json`、`status_frozen.json`、`controllers_after_freeze.txt`、`after_scan_rviz.png`。
- `start_driver.sh`、`start_camera.sh`、`start_camera_mount.sh`、`sample_joint_states.py`；当前这些进程已在运行，不应重复启动。

本轮没有修改原感知/仿真包或工厂校准源文件，没有变更系统供电/CPU 策略。
