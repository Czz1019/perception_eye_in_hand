# 正常仿真输入下的各阶段实时性与瓶颈复测

日期：2026-11-10。功能包 ur7e_nvblox_in_hand。没有人为 TF 时间偏差，没有修改生产源代码或用户配置文件。使用测试目录中的只读观察包装器记录已有调用时间；不改变融合、更新、导出及发布调度。

## 1. 测试开始时发现的配置问题

当前 src/config/base.yaml 中 voxel_size=0.02，但 realtime.yaml 中仍为 0.01；realtime 覆盖基础配置。因此只改 base.yaml 不保证运行体素变为 2 cm。launch 默认还读取 install 中的安装副本，修改源文件后是否重新编译也会影响结果。应以 ros2 param get /ur7e_nvblox voxel_size 为准。

当前源基础配置另有 clearing_hz=1.0、clearing_radius_m=-1.0 的矛盾，直接读取会被节点验证拒绝。本次保留源文件，使用上一轮成功运行的正常参数快照 baseline_params.yaml 作为统一基准，并显式覆盖 voxel_size。运行后核对参数快照，两组只有 voxel_size 不同。

最初默认域136存在额外时钟/TF干扰，日志出现 TF_OLD_DATA 和空图，已停止该次回放并排除。正式测试改用预检无发布者的 ROS_DOMAIN_ID=159，没有停止用户已有的 Gazebo。失败记录保留在 failed_source_config_voxel001/、failed_domain136_voxel001/，不纳入表格或图像。

## 2. 正式测试条件

同一份约 87 秒可靠仿真 bag，RGB/Depth 640×480、实际输入约 29.95 Hz、同一机械臂缓慢扫描轨迹。分别从空地图建立 1 cm、2 cm 地图，各一轮，1 倍速回放，无 GUI/RViz，但实际订阅完整 ESDF 和 Mesh。两组使用同一观察包装器和监测器。

固定 depth_hz=color_hz=40、esdf_hz=esdf_view_hz=33.4、esdf_view_stride=2、mesh_hz=5、decay_hz=clearing_hz=0、input_qos=reliable、workspace_bounds_type=unbounded。truncation_vox=4 保留当前设置，对应物理截断 4 cm 与 8 cm；esdf_max_site_vox=2 对应尺度也随体素变化。不是保持物理截断带不变的纯内核基准。

统计剔除前10秒，截止回放完成。按现有 metrics 计数得到频率，阶段耗时从原有 record_time 观测；额外记录图像回调、进入融合的时刻和 TF 查询。打包/发布在独立线程，不能与工作线程耗时简单相加。测试观察本身有少量开销，表中不是未经观测包装的绝对性能上限。

## 3. 每一环节的实际频率

| 环节 | 1 cm | 2 cm |
|---|---:|---:|
| 深度接收回调 Hz | 29.96 | 29.96 |
| 彩色接收回调 Hz | 29.96 | 29.96 |
| 深度融合调用 Hz | 28.23 | 29.34 |
| 颜色融合调用 Hz | 28.15 | 29.43 |
| ESDF 更新调用 Hz | 23.80 | 28.50 |
| ESDF 发布调用 Hz | 23.78 | 28.50 |
| Mesh 发布调用 Hz | 4.80 | 4.97 |
| 完整非空 ESDF 实际接收 Hz | 23.23 | 26.26 |
| 实际接收载荷 MB/s | 250.24 | 37.99 |
| 最后一帧导出点数 | 542630.00 | 72633.00 |

![阶段频率与调用耗时](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pipeline_realtime_20261110/pipeline_rates_and_costs.png)

ESDF 更新数字为主机调用次数，不是 GPU 内核完成率；点云接收数字不等于 RViz FPS。地图中的距离更新与把完整地图搬到 ROS 是不同阶段。

## 4. 每阶段耗时与排队

| 主机调用，包含等待 | 1 cm 均值/P95，ms | 2 cm 均值/P95，ms |
|---|---:|---:|
| TF 查询 | 0.050 / 0.104 | 0.027 / 0.067 |
| 深度转换及提交 | 2.767 / 5.629 | 1.712 / 2.962 |
| 颜色转换及提交 | 4.658 / 10.522 | 1.348 / 2.397 |
| ESDF 更新提交 | 6.702 / 13.979 | 1.653 / 2.883 |
| ESDF 全量导出及 GPU 等待 | 10.415 / 19.573 | 1.424 / 2.053 |
| Mesh 生成/读回 | 32.894 / 45.722 | 7.724 / 10.465 |
| ESDF 打包发布，独立线程 | 20.598 / 31.885 | 2.294 / 3.751 |
| Mesh 打包发布，独立线程 | 27.635 / 43.090 | 9.588 / 13.785 |

| 延迟/排队 | 1 cm P50/P95，ms | 2 cm P50/P95，ms |
|---|---:|---:|
| 深度接收至进入融合 | 42.71 / 84.93 | 26.35 / 44.24 |
| 颜色接收至进入融合 | 44.40 / 84.38 | 26.55 / 45.17 |
| 同深度时间戳 CameraInfo 接收至完整 ESDF 接收 | 93.97 / 163.67 | 50.23 / 60.32 |

接收至融合包含排队、等待TF及线程调度等，不是只测队列容器。最后一项为回放输入至输出代理延迟，未包含真实曝光或 RViz 绘制。只统计匹配到同时间戳且非负的样本，匹配比例见 JSON。

## 5. 哪些帧未参与融合

| 稳定窗口计数变化 | 1 cm | 2 cm |
|---|---:|---:|
| depth_overwritten | 98 | 10 |
| color_overwritten | 102 | 3 |
| depth_stale | 0 | 0 |
| color_stale | 0 | 0 |
| depth_gate_skipped | 33 | 37 |
| color_gate_skipped | 35 | 37 |
| esdf_points_output_overwritten | 2 | 0 |
| clock_resets | 0 | 0 |

overwritten 是应用输入队列满后的覆盖；stale 是输入过期；gate_skipped 是图像时间戳间隔未通过当前 40 Hz 接纳门限；esdf_points_output_overwritten 是未发布的旧输出被更新版本覆盖。没有增长的计数不会被补成已测到的故障。TF失败尝试是重试次数，不等于丢图数量。

| TF 查询情况 | 1 cm | 2 cm |
|---|---:|---:|
| 稳定窗口失败尝试 | 7578 | 38804 |

## 6. 工作线程实际时间分布

![一秒内实际调用时间线](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pipeline_realtime_20261110/pipeline_actual_timeline.png)

时间线取回放第60～61秒。上轨是执行融合、ESDF和读回的工作线程，下轨是打包发布线程。色块为本次真实记录的主机调用区间，包含函数内部等待；空白含等待、调度和未记录的工作，不表示 GPU 一定空闲。两个轨道可以重叠，不能直接相加。

| 工作线程已记录阶段占墙钟比例 | 1 cm | 2 cm |
|---|---:|---:|
| depth_submit_ms | 7.8% | 5.0% |
| color_submit_ms | 13.1% | 4.0% |
| esdf_submit_ms | 15.9% | 4.7% |
| esdf_export_with_gpu_wait_ms | 24.6% | 4.0% |
| mesh_readback_ms | 15.8% | 3.8% |

![频率、延迟及点数随回放变化](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pipeline_realtime_20261110/pipeline_time_series.png)

## 7. 代码中导致频率无法直接达到上限的环节

1. 融合、ESDF更新、全量导出、Mesh读回共用工作线程。深度图更新可能合并进一次ESDF更新，不能期待融合调用与ESDF调用永远一一对应。
2. due() 按实际检查时刻加周期重新设期限，没有独立固定相位的ESDF线程；工作线程被阻塞时不补回错过的更新。dirty_depth 会保留到下一轮检查，不是一次未到期就丢弃地图变化。
3. native/export.cu 先 cudaDeviceSynchronize()，再遍历全部已分配块，分批回读计数和点数据并同步。该同步等待当前CUDA上下文先前提交的任务，导出耗时不是独立ESDF内核耗时。
4. backend.mesh() 更新颜色Mesh并把顶点/面/颜色搬回CPU。即使只5Hz，也会周期性阻塞工作线程。
5. 输出线程负责全量消息构造与DDS发布，共享CPU/内存资源。voxel_size不会减少640×480输入的像素数量；stride改变导出点数而非取消全层遍历。

以上为代码事实；各阶段对本轮损失的大小应结合实测表，不将未测量的CUDA内核、GIL或DDS分片过程强行指定为唯一根因。

## 7.1 本轮可以定位到的主要问题

**本轮确实观察到 2 cm 的改善：ESDF 更新调用 23.80→28.50 Hz，完整非空点云接收 23.23→26.26 Hz。** 这与“体素加倍完全没有提升”的现象不同；不能忽略配置覆盖或拿旧传输版本的日志直接比较。

**1 cm 主要压力在完整地图输出与 Mesh。** 工作线程 ESDF 更新提交约6.70ms、全量导出约10.41ms、Mesh生成/读回约32.89ms；独立线程点云打包发布约20.60ms。1cm 最后一帧约54.26万点、13.02MB，本轮实际接收载荷约250.24MB/s。工作线程上述导出和Mesh阶段分别占观测墙钟约24.6%、15.8%，并不只是底层ESDF更新在消耗时间。

**2 cm 的计算与搬运已明显变轻。** ESDF更新提交约1.65ms、导出约1.42ms、Mesh读回约7.72ms、点云打包发布约2.29ms。最后一帧72633点、约1.74MB，实际接收载荷约37.99MB/s。此时继续降低体素计算量，并不能直接消除输入时序等待和送达差异。

## 7.2 TF函数很快，不代表没有TF等待

| 针对成功融合的深度帧 | 1 cm | 2 cm |
|---|---:|---:|
| 接收至首次TF查询，均值 ms | 36.99 | 6.38 |
| 首次查询至成功，均值 ms | 8.90 | 19.79 |
| 首次查询至成功，P95 ms | 45.35 | 43.61 |
| 曾发生TF失败重试的融合帧比例 | 32.9% | 63.8% |

这些“首次至成功”的时间包含TF尚未可用、重试间隔和其他工作调度，不是lookup_transform函数执行时间。失败只记录成功/失败，未分类异常，不能把所有失败都指定为相同原因。输入bag已核对：关节状态约100Hz，而动态图TF约20Hz，图像约30Hz；按图像时刻插值常需等待后一个TF样本。

1cm 时工作线程处理较慢，很多帧到首次查询前已经排队约37ms，查询时TF往往已到；2cm 时首次查询更及时，平均约6.38ms，但平均重试等待约19.79ms。于是查找函数本身虽仅几十微秒，系统仍可能等待位姿数据。

本轮2cm 深度融合启动间隔中，约27.2%短于10ms、约57.9%超过40ms，说明30Hz观测在实际处理时不是均匀逐帧执行。集中处理的多帧变化与ESDF周期门限相遇时可能合并更新：本轮平均每次ESDF更新对应1.03次深度融合，1cm 为1.19次。该比值描述本次调用关系，不是每帧丢弃比例。

## 7.3 发布和接收仍有差别

2cm 的发布调用约28.50Hz，而监测器完整接收约26.26Hz；因此不能用topic hz把底层更新能力直接认定为26Hz。输出最新值队列、发布端与订阅端历史深度和调度都会影响观测。监测器可靠订阅历史深度为1，Reliable不等于无限保留每个版本。本轮没有逐消息DDS追踪，不能把这个差距指定为网络丢包。

## 7.4 后续优化的顺序

1. 先确认节点实际参数：体素值是否被realtime.yaml或安装副本覆盖；清理配置是否合法。
2. 对1cm，优先减少全量导出的同步/重复分配/复制与Mesh读回对工作线程的阻塞。采用正确CUDA流依赖或一致快照后再解耦，不能直接删除同步。
3. 对2cm，核对机械臂关节状态至TF的发布频率和时延。在状态频率足够的前提下改善TF时间覆盖，继续按图像历史时刻查询；不用最新TF替代来凑频率。
4. 将完整点云接收单独优化，检查输出队列、订阅历史、消息复制与DDS，而不是只提高esdf_hz参数。
5. 若需要精确区分CUDA内核、同步等待与CPU运行，再使用GPU事件/分析器；本次主机耗时与整卡nvidia-smi采样不足以作该结论。

以上只是由本轮证据得到的优化方向，本次未实施这些改动、未增加额外配置档位测试。

## 8. 如何核对你的实际配置

正常运行节点所在终端环境中执行：

```bash
ros2 param get /ur7e_nvblox voxel_size
ros2 param get /ur7e_nvblox truncation_vox
ros2 param get /ur7e_nvblox esdf_view_stride
ros2 param get /ur7e_nvblox esdf_hz
ros2 param get /ur7e_nvblox mesh_hz
ros2 param get /ur7e_nvblox clearing_hz
ros2 param get /ur7e_nvblox clearing_radius_m
```

本次使用域159，需要在检查本次复现时 source 当前项目 env.sh 并 export ROS_DOMAIN_ID=159；你的现有仿真使用自己的域，不要混用。本次回放完成后已经停止测试节点，因此这些查询需在节点运行期间进行。

## 9. 原始文件与限制

原始目录：/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/pipeline_realtime_20261110。voxel001/、voxel002/ 包含 params.yaml、command.txt、phase_trace.json、metrics_events.json、received_clouds.json、gpu.csv、map.blox、mesh.ply。pipeline_summary.json 是统计汇总；parameter_differences.json 验证两组仅 voxel_size 参数不同。

nvidia-smi 为整块GPU的低频采样，包含桌面和用户已有进程，不是 mapper 独占GPU的内核计时。每种体素一轮且顺序固定，因此小幅差异不宜当作可重复的绝对性能结论。本轮不做几何精度测试。
