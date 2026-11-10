# 1 cm 3D ESDF：stride=2 与 stride=3 的效果与实时性实测

2026-11-10。使用 `ur7e_nvblox_in_hand` 完成四轮实际 ROS 回放测试，顺序 **2→3、3→2**。本轮只通过运行参数修改 `esdf_view_stride`，没有改变融合算法、体素精度、Mesh 配置、系统网络设置或默认 stride=2。

主要结论：**stride3 明显减少点数、传输量和延迟，但本机本次测试没有提高实际接收频率。** 完整 ESDF 始终以 1 cm 计算，stride 仅影响显示点云。

## 1. 测试方法与公平性

先在工作台场景录制一整段已检查的机械臂定高左右扫描，再回放给全新的 mapper 实例。四轮使用同一个 rosbag、相同 1 倍回放速度和相同参数，唯一配置差异是 stride。运行后的 `params.yaml` 已逐项比较，差异确实只有 `esdf_view_stride`。

| 条件 | 设置 |
|---|---|
| 输入场景/位姿 | Gazebo 工作台、UR7e/D435、机械臂图像时间 TF |
| 分辨率 | RGB、深度均 640×480 |
| 体素/截断 | 1 cm / 4 体素即 4 cm |
| 深度/颜色融合上限 | 40 / 40 Hz |
| ESDF 更新/导出上限 | 33.4 / 33.4 Hz |
| Mesh 上限 | 5 Hz，四轮都启用 |
| 图层配置 | 保持原配置，无额外图层订阅 |
| 接收端 | 单独 ROS 节点，Reliable、KeepLast(depth=1)，raw CDR 回调 |
| 显示负载 | 未打开 RViz；包含实际 DDS 发送/接收，未包含 RViz 绘制 |
| 统计窗口 | 排除每轮回放前 10 秒，至回放结束；每轮约 85.9 秒 |

raw 接收避免在热路径逐点反序列化大点云和 Mesh。点云接收计数来自真正收到的非空 PointCloud2；启动时清空地图的空消息不计入。

录制数据长 **95.78 秒**，约 3.46 GB，包含 2274 帧深度、2223 帧颜色，平均约 **23.74/23.21 Hz**，并非稳定 30 Hz 输入。CameraInfo、TF、TF static、clock 同时录制。相同回放源不保证四轮节点实际收到或融合的帧数完全相同：ROS 传输和调度丢失仍可能发生，实际接收率另行统计。

首次录制因额外截图订阅的图像/内参匹配超时而提前停止，已保留为 `input_bag_failed_partial` 诊断。正式数据重新录制，扫描控制不再重复订阅大图像；运动反馈及场景检查保持不变。失败数据没有参与本报告。

## 2. 实际 ESDF 显示效果

![两种 stride 的实际接收点云与同地图截面](../artifacts/stride23_replay_20261110/esdf_stride2_vs_stride3.png)

上排为第一组回放**实际收到的最后一帧 ESDF 点云**，显示相同台面 ROI；下排为**同一份冻结地图**在 z=0.125 m 的截面，分别按 stride2、stride3 导出，隔离不同回放融合帧数的影响。

两组使用相同视角、点大小、空间范围及距离色标。图片是对真实数据的离线着色渲染，不是 RViz 窗口截图；统一 turbo 色标范围 -0.04～0.20 m，超出色标范围只裁剪颜色，不修改保存距离。

这里的点表示已观测体素到表面的距离，既包含已观测自由空间，也可能包含内部体素。它不是仅贴在物体表面的重建点云；红色自由空间不表示红色区域全部被占用。unknown 未填入显示点云。

### 对同一份冻结地图的精确子集检查

冻结地图完整 observed ESDF 为 **4309430 点**。

| 项目 | stride2 | stride3 |
|---|---:|---:|
| 显示间隔 | 2 cm | 3 cm |
| 导出点数 | **542040** | **157820** |
| 占完整 observed 点比例 | 12.58% | 3.66% |
| 理论密集网格保留比例 | 1/8 | 1/27 |
| 坐标、距离及标记与完整地图对应子集 | 完全一致 | 完全一致 |

stride3 点数约为 stride2 的 **29.12%**，减少约 **70.88%**。比例不严格等于 8/27，是稀疏观测边界和采样相位造成的。

两个 stride 都按原生全局体素索引选择已有 observed 体素；没有对选中距离重新计算、平滑或填充。两种采样的重合坐标使用同一原生距离。stride3 显示的细节更稀疏，管架附近的小尺度变化更容易在图上看不出来，但完整地图和服务查询仍是 1 cm 精度。

## 3. 实时性结果：两轮平均

以下使用统一稳态窗口。更新/发布来自节点累计计数；“实际接收”来自独立 Reliable 订阅器，两者明确区分。

| 指标 | stride2 | stride3 |
|---|---:|---:|
| 深度回调收到 | 17.87 Hz | 18.62 Hz |
| 深度融合调用 | 17.51 Hz | 18.36 Hz |
| 颜色融合调用 | 17.03 Hz | 18.71 Hz |
| **ESDF 更新调用** | **16.37 Hz** | **16.97 Hz** |
| ESDF 点云发布调用 | 16.37 Hz | 16.96 Hz |
| **实际非空点云接收** | **16.08 Hz** | **14.52 Hz** |
| Mesh 发布调用 | 4.88 Hz | 4.94 Hz |
| 输入至接收延迟中位数 | **81.80 ms** | **58.26 ms** |
| 输入至接收延迟 P95 | **135.33 ms** | **101.12 ms** |
| 实收点云载荷速率，不含协议开销 | 163.46 MB/s | 42.90 MB/s |

延迟列分别是两轮中位数/P95 的算术平均，不是把两轮样本混在一起重新计算的百分位，也不是统计置信区间。

### 每轮结果，避免平均数掩盖波动

| 执行顺序 | stride | ESDF更新调用 | 实际接收 | 延迟中位数 | 延迟P95 |
|---|---:|---:|---:|---:|---:|
| 第1轮 | 2 | 16.70 Hz | 16.37 Hz | 80.05 ms | 127.91 ms |
| 第2轮 | 3 | 17.21 Hz | 14.72 Hz | 57.89 ms | 101.59 ms |
| 第3轮 | 3 | 16.72 Hz | 14.31 Hz | 58.64 ms | 100.64 ms |
| 第4轮 | 2 | 16.05 Hz | 15.78 Hz | 83.55 ms | 142.75 ms |

两次 stride3 都表现为更短延迟，但实际接收 Hz 略低。两次 stride2 的接收帧间隔 P95 分别约 160.84/173.22 ms，stride3 约 187.30/193.00 ms，说明更低延迟与更连续的帧流不是同一指标；这次 stride3 的接收间隔更不均匀。

![第一组回放的更新调用、实收频率、延迟分布和点数](../artifacts/stride23_replay_20261110/realtime_stride2_vs_stride3.png)

图展示第一对回放；表格提供两对结果。频率曲线有窗口波动，不能用某个瞬时峰值称为稳定 30 Hz。

## 4. 延迟到底测的是什么

起点：接收某一深度时间戳的 CameraInfo。

终点：收到 ESDF 点云中引用同一深度时间戳的完整消息。

采用单机 `time.monotonic()`，不把仿真时间直接与系统时间相减。该指标包含回放输入传递、等待对应图像/TF、融合与 ESDF 调度、GPU导出、点云打包和 DDS 接收等路径的影响。CameraInfo 与深度图在 ROS 中并非绝对同时到达，因此它是**回放输入到点云接收的延迟代理**，不是物理相机曝光到显示的完整延迟。

它不包括真实传感器内部处理、相机驱动前段和 RViz 渲染耗时。只有能找到相同 CameraInfo 时间戳的非负样本参加统计；无匹配样本没有补零。启动前10秒和结束后的尾部消息不混入稳态窗口。

当前订阅器采用 Reliable + depth1，偏向保留最新消息；Reliable 不等于应用层一定逐帧收到每次发布。DDS/history/执行器与消息排队会影响实际接收。接收 Hz 是这个测试订阅器的实际结果，不能直接当作 RViz FPS。

## 5. 为什么 stride3 更省，却没有获得更高显示频率

实际链路为：

```text
RGB-D + 时间对应TF
  → 1cm TSDF/颜色融合
  → 完整1cm ESDF更新
  → GPU按stride选择显示体素
  → CPU点云打包、发布
  → DDS传输与订阅器接收
```

stride 主要降低最后几段的数据量，没有把完整 ESDF 改成 3 cm，也没有把 `esdf_hz=33.4` 改掉。

两轮末段主机计时均值再取平均：

| 阶段 | stride2 | stride3 |
|---|---:|---:|
| ESDF 提交 | 6.03 ms | 4.75 ms |
| ESDF 导出，含 GPU 等待 | 13.10 ms | 6.13 ms |
| 点云打包发布 | 23.91 ms | 5.23 ms |
| Mesh 打包发布 | 28.55 ms | 28.99 ms |

可以直接确认导出、打包负担下降，Mesh 打包开销基本不变。对同一冻结地图另做5次交替顺序的离线完整导出计时，stride2 平均约 **7.99 ms**，stride3 约 **3.18 ms**，也支持采样导出更轻这一点。这是无融合竞争时的导出耗时，不能替代运行中的整条链路延迟；各阶段有并发，表格耗时不能简单相加。

但“每次消息更快到达”不保证“每秒收到更多消息”。本轮 stride3 更新/发布调用略高，接收率却略低，差异发生在后续传递/接收阶段。大消息 history、DDS 传输、执行器调度与处理突发会影响这一结果；本轮没有包级跟踪，不能把全部差异认定为某一个具体底层缺陷。

本机读取到 socket rmem/wmem 上限为 212992 B，本轮未调整。它可能影响大消息突发传输，但仅凭这个上限也不足以完成因果定位。摄像头回放源为23～24 Hz，节点实际收到约18～19 Hz，说明输入接收和整个运行负载也限制了结果；不能仅靠 stride 就宣称恢复到30 Hz。

目前结果支持的选择是：

- 重视点密度和本次接收连续性，可保留 **stride2**。
- 重视降低带宽、较低消息延迟，且允许更稀疏显示，可用 **stride3**。
- 做碰撞距离查询仍应使用完整1cm地图服务，不能把降采样显示点云当作全分辨率地图。

stride 不修复前面测试中的错误距离、unknown 或重建错连，只减少显示点。本次没有重新宣称 ESDF 精度或抓取可用性已经达标。只有每种两轮，结论限定于当前机器、QoS、输入数据和负载。

## 6. 实际仿真查看命令

stride2：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11566
source ./env.sh
ros2 launch ur7e_nvblox_in_hand bringup.launch.py \
  scene_variant:=lab gazebo_gui:=true run_rviz:=true \
  camera_width:=640 camera_height:=480 \
  voxel_size:=0.01 truncation_vox:=4.0 \
  esdf_mode:=3d esdf_view_stride:=2
```

stride3 使用相同命令，把最后参数改为 `esdf_view_stride:=3`。两组不要同时启动。RViz 的 `/ur7e_nvblox/esdf_points` 选择 Reliable QoS；显示样式 Points。GUI/RViz 增加的负载不包含在本次回放测量里。

查看已经实收保存的数据：

```bash
CloudCompare /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/stride23_replay_20261110/stride2/received_esdf.ply
CloudCompare /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/stride23_replay_20261110/stride3/received_esdf.ply
```

CloudCompare 可选择 `distance` 标量场；两组使用相同距离色标和点大小。点云仅为观测体素显示，不是障碍物表面 Mesh。

## 7. 保存结果和复现边界

目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/stride23_replay_20261110/`。

- `input_bag/`、`input_summary.json`：四轮共用的正式输入。
- `stride2/`、`stride3/`、`stride3_repeat/`、`stride2_repeat/`：四轮实际配置、命令、地图、Mesh、metrics、接收日志和最终 CDR。
- 第一对目录中的 `received_esdf.ply`、`last_received_esdf.npy`：实际完整接收消息解码，不是离线查询补出来的点云。
- `common_stride2.npy`、`common_stride3.npy`、`common_export.json`：同一冻结地图的采样验证和独立导出计时。
- `comparison_summary.json`：稳态窗口、逐轮结果、两轮均值。
- `parameter_comparison.json`：四轮实际参数仅 stride 不同的检查。
- `record_inputs.py`、`record_motion.py`、`run_replays.py`、`run_repeats.py`、`monitor.py`、`export_common.py`、`analyze.py`：本轮测试代码。

首次失败的部分数据/日志单独保留，未混入正式结果。回放脚本拒绝覆盖已存在的轮次目录，避免误覆盖测试证据。测试已完成，未保留仿真、mapper、回放或运动进程；默认配置仍为 stride2。
