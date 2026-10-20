# UR7e 自动真机扫描重建

独立于 `ur7e_real_reconstruction` 的自动扫描入口。保留原手动扫描代码、启动脚本和历史结果。

## 真机测试状态（2026-09-29）

第二轮已自动到达 8 个观察点，用户反馈“无明显抖动”；保存 58 帧与彩色地图。最后一点采集期间触发 40 ms 控制周期保护，S08 仅 1 帧，最终 ERROR，**完整 8 点扫描尚未验收通过**。机械臂已停止。详见 [真机测试报告](report/2026-09-29_real_auto_scan_test.md)。

## 本次离线修复状态

针对 S08 停稳采集超时，流程改为每点先停止伺服、断开控制并再次核验静止，再采集 RGB-D；下一段重新核对实际起点并取得控制权。运动期间仍使用 500 Hz 目标频率与 40 ms 周期保护。采集期间保持 RTDEReceive/TF、位置漂移和安全状态检查，不创建高频伺服线程。

**按用户要求，本次未进行真机测试，也未重启真机控制程序。** 软件验证与待检查事项见 [修复说明](validation/capture_fix/REPAIR_NOTES.md)；原真机失败报告保留为历史记录。

## 实现

- 使用选定的 8 个眼在手上观察姿态，按既定顺序扫描；不使用 SAGE，也不计算 ESDF。
- 从实时关节反馈生成起点到 S01 的进入段；每段采用同步五次关节插值，端点速度/加速度为零。已有离线 IK 关节解，无需在执行时用雅可比追逐相机位姿。
- 独立 RTDE `servoJ` 控制线程，默认 500 Hz、0.12 rad/s、0.18 rad/s²、lookahead 0.1 s、gain 300。时间缩放保留一半加速度预算；延迟不触发跳点追赶。
- 到点误差小于 0.008 rad、速度小于 0.01 rad/s，并连续停稳 0.7 s 后，执行 servoStop / stopScript / disconnect，再连续核验静止 0.7 s 才打开采集门。默认每点至少融合 8 帧；关门后重新核验位置，从实际反馈起步执行下一段。
- 取得 RTDE 控制权前先预热 CUDA；预热地图丢弃，不计入真实数据。运动后仍保留感知心跳检查。
- 独立进程运行已有彩色 RGB-D TSDF 重建。继续校验图像时间的 TF、工厂校准模型、相机外参、原始颜色畸变、深度和关节静止条件，并做原来的机器人深度过滤。
- 每点保存 RGB、深度、实际关节角与相机位姿；最终输出彩色 `surface.ply`、`surface.npz` 和 RViz 网格。扫描结束停在最后一点，不自动返航。
- 提供只读 `prepare`、显式 `start` 和 `cancel`；反馈超时、过大跟踪误差、保护停止、循环超时、控制器心跳丢失、感知心跳丢失均终止运行。无自动故障复位/续跑。

## 范围

采用用户确认的“上方空旷”运行条件，**不做场景避障规划**。完整自身碰撞检查也未启用；旧观察点报告中的相机/手腕球体模型间距问题没有被本次实现解决。本包不宣称路径经过无碰撞验证。关节限位、平滑轨迹及 UR 硬件保护不能替代这一验证。

相机路径为关节插值对应的曲线，不保证直线。准备后检查实际进入段；起点变化超过 0.015 rad 或模型、路线、参数变化，执行会拒绝旧预览。

## 启动

在工作区根目录：

```bash
./build_ur7e_auto_reconstruction.sh
./run_ur7e_auto_reconstruction.sh
```

默认仅连接 **RTDEReceive**，不创建 RTDEControl、不上传脚本、不移动机械臂。D435i 相机驱动需已运行。

```bash
source /opt/ros/humble/setup.bash
source /home/wqj/realtime_nvblox_ws/install/local_setup.bash
./ur7e_auto_scan_control.sh prepare
```

返回 `plan.json` 所在目录，同时生成 `PREVIEW.md`、`route.png`。完整执行流程见 [真实测试说明](REAL_TEST_GUIDE.md)。

## ROS 接口与隔离

| 接口 | 用途 |
|---|---|
| `/ur7e_auto_scan/prepare` | 根据当前 RTDE 实测关节状态生成预览 |
| `/ur7e_auto_scan/start` | 只有启动时明确允许运动并提供审核后的预览文件才接受 |
| `/ur7e_auto_scan/cancel` | 停止控制，尽力冻结已采集部分；不自动重启 |
| `/ur7e_auto_scan/status` | PREVIEW_ONLY / WAIT_START / STARTING / WARMING_UP / ACQUIRING_CONTROL / MOVING / SETTLING / STOPPING_AT_VIEW / VERIFYING_STATIONARY / CAPTURING / EXPORTING / DONE / ERROR / CANCELLED |
| `/ur7e_auto_scan/route` | 相机路径预览 MarkerArray |
| `/ur7e_auto/joint_states` | RTDE 实测关节角；旧数据不会被重新标记为新反馈 |
| `/ur7e_auto/tf`、`/ur7e_auto/tf_static` | 自动包独立校准 TF，避免与原 UR 驱动、相机 TF 冲突 |
| `/ur7e_auto_mapper/mesh`、`surface_points` | RViz 彩色重建 |
| `/ur7e_auto_mapper/capture_status` | 当前点采集 epoch、有效帧数、相机/协调器状态 |

`start_scan`、`freeze_scan`、`capture`、`warmup` 是协调器使用的内部采集服务；运行期间不要手动调用。

## 文件与依赖

- `config/waypoints.yaml`：8 个完整精度关节目标，带模型哈希和离线选点来源。
- `config/control.yaml`：低速控制设置。
- `config/mapping.yaml`：独立重建输出目录和话题配置。
- `~/nvblox_output/ur7e_auto_reconstruction/scan_*/`：彩色地图、每点观测。
- `~/nvblox_output/ur7e_auto_reconstruction/plans/run_*/`：实际执行预览、点位结果、分段命令/实测/速度记录与周期诊断。`servo_trace.npz` 中 `segment_index` 标记运动段；统计周期时须排除段间采集间隔。

ROS Humble，`whole_control` 中的 `ur_rtde`、nvblox_torch、NumPy、SciPy、Pillow、OpenCV、Matplotlib，以及现有重建/模型功能包。脚本沿用本机校准模型路径。

参考用户已实践的 `storm/examples/HIL_sage_rtde` 的 RTDE 与控制线程分离方式；没有修改该实现。实时控制 API 与参数含义也核对了[UR 官方 servoJ 文档](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-scriptmanual/all_scripts/servoj_qavt0-008lookahead_time.htm)及本机安装的 `ur_rtde` API 文档。普通 Linux/Python 的 500 Hz 是目标频率，不是硬实时保证；实际数据记录于 `servo_trace.npz`。

原手动入口仍为：

```bash
./run_ur7e_real_reconstruction.sh
```
