# UR7e与D435i实机：测试记录、实时性结果与问题分析

日期：2026-11-11，Asia/Shanghai。工作区：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand`。本轮新增了30秒实机记录，并将此前“无重建、深度闪黑、测频无输出”三个问题的证据一起汇总。

## 1. 本轮结论

当前实机已经持续接收和融合新图像，3D ESDF更新与实际接收均接近29Hz。不能继续用之前TF断开时的0融合，或另一套大地图仿真中的约10Hz，作为当前实机的性能结果。

本轮没有发送机械臂运动指令、没有改运行参数、没有清图或重启相机；测量结束后成功保存彩色Mesh和原生地图。关节最大变化0.00486°，所以这是**静止实机连续更新测试**，不是手拖运动重建验收。

| 项目 | 本轮结果 |
|---|---:|
| 深度接收 | 29.55Hz |
| 颜色接收 | 29.28Hz |
| 深度融合调用 | 29.55Hz |
| 颜色融合调用 | 29.28Hz |
| ESDF更新调用 | 29.03Hz |
| ESDF点云发布调用 | 29.03Hz |
| ESDF实际接收 | 29.03Hz |
| Mesh发布调用 | 4.93Hz |
| 收到ESDF消息 | 871条，871个不同输入时间戳 |
| ESDF显示点 | 146,888–146,902 |
| 输入时间戳→ESDF接收年龄P50/P95 | 69.59/85.48ms |
| CameraInfo接收→ESDF接收代理P50/P95 | 35.56/51.47ms |
| ESDF接收间隔P50/P95/最大 | 31.92/52.70/104.00ms |

**平均频率接近30Hz，不等于每一帧都在33ms内到达。** 最大接收间隔约104ms，说明仍有瞬时抖动。P95是95%分位数，不是最大值或准确率。

## 2. 实际配置及测量口径

- 当前实际配置开始/结束一致，map_epoch保持0，没有检测到中途清图/换图；Mapper fatal error为空。
- 测量起始记录时间为2026-11-11 21:46:51，约30.002秒。原始metrics首尾计数窗口约29.001秒，因metrics每秒发布一次；两者不是同一分母。
- 相机和机械臂均为真实硬件，不播放rosbag、不使用仿真时钟；ROS域135。
- 体素0.005m，TSDF截断4体素=0.02m。
- 输入RGB/Depth为640×480、配置30Hz，真实话题前缀`/camera/camera`。
- 深度/颜色准入上限40Hz；ESDF更新/显示上限33.4Hz；Mesh上限5Hz。
- 3D ESDF、observed、stride2。原生5mm格点，显示采样间距约1cm。
- 输入范围0.2–2.05m，bounding_box为x[−0.1,1.1]、y[−0.6,0.6]、z[−0.05,1.05]m；边界按块选取，不是逐顶点精确裁剪。
- 衰减和清图均关闭；输入队列4帧、最大年龄0.5秒，input_qos=sensor_data。
- 位姿由每张图像时间戳查询机械臂TF，Depth和Color各用自己的相机模型。

原始生效配置：[params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/params.yaml)；完整计数与库路径：[result.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/result.json)。

本轮监测订阅既有metrics、Depth CameraInfo、抽样关节状态和RELIABLE原始ESDF点云，没有订阅整幅RGB/Depth。频率来自host计数或订阅端接收，**没有使用CUDA事件测GPU完成频率，也没有测RViz FPS**。

“输入时间戳年龄”使用消息携带的输入时间戳与本机接收时钟相减；CameraInfo代理使用同stamp的CameraInfo接收时刻与点云接收时刻相减。本轮871条都匹配到代理样本，但均不能直接等同相机曝光到屏幕的完整延迟。SDK时间与曝光语义及屏幕渲染未标定。

## 3. 三个10秒段，检查是否只是偶然瞬时高频

| 接收窗口 | 深度融合 | 颜色融合 | ESDF更新 | ESDF发布 | ESDF实际接收 | Mesh发布 | 输入年龄P95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 0–10秒 | 29.45 | 29.00 | 29.22 | 29.22 | 29.20 | 4.89 | 82.46ms |
| 10–20秒 | 29.77 | 29.77 | 29.22 | 29.22 | 29.20 | 5.00 | 86.53ms |
| 20–30秒 | 29.33 | 28.89 | 28.55 | 28.55 | 28.70 | 4.89 | 85.81ms |

表中频率单位Hz，段内host计数窗口各约9秒，接收窗口各10秒。个别段融合略高于接收，是计数窗口端点及前一窗口已排队图像造成的可能现象，不表示生成了额外相机帧。

三个接收窗口分别29.2、29.2、28.7Hz，说明本次不是只截取某一秒30Hz作结论。它仍只是一次30秒短测，没有多轮统计或长时间热稳态/大范围地图增长测试。

![实机分段结果](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/real_realtime_summary.png)

## 4. 各阶段耗时与数据损失

使用节点已有指标，以下为测量结束时最近200次调用的滚动统计。Mesh约5Hz，其200次窗口可覆盖本轮之前的历史；不能把各项视为同一段独立计时，更不能相加当作整帧延迟。

| 已有计时项 | 平均 | P95 |
|---|---:|---:|
| 深度提交/融合接口 / `depth_submit_ms` | 2.82ms | 5.56ms |
| 颜色提交/融合接口 / `color_submit_ms` | 2.65ms | 10.51ms |
| ESDF更新接口 / `esdf_submit_ms` | 5.23ms | 8.34ms |
| ESDF导出及GPU等待 / `esdf_export_with_gpu_wait_ms` | 3.21ms | 5.82ms |
| ESDF点云打包发布 / `esdf_points_pack_publish_ms` | 6.37ms | 9.24ms |
| Mesh更新及回读 / `mesh_readback_ms` | 17.44ms | 19.91ms |
| Mesh打包发布 / `mesh_pack_publish_ms` | 13.37ms | 17.04ms |

本轮深度、颜色队列覆盖增量均为0，深度过期增量也为0。计数窗口内深度接收与融合均约29.55Hz，颜色接收与融合均约29.28Hz，说明当前短窗口没有再出现此前大量等待TF/覆盖队列的问题。不能由此保证更大地图或运动时仍无丢帧。

每条ESDF约3.526MB；按29.03Hz估算，单路原始载荷约102MB/s。这不是实测网卡带宽，未计协议及内部拷贝，也不等于GPU处理量。

## 5. 与前面问题记录的联系

| 阶段 | 已有证据 | 当前解释 |
|---|---|---|
| 相机启动失败 | 前次日志加载了另一套环境的wrapper/SDK；用户反馈更新环境后恢复 | 不应继续把当时版本加载问题作为当前无建图的原因 |
| RGB和机器人可见但无Mesh | 手眼静态TF未设置域，默认0；相机/Mapper域135；图像时刻TF成功0/320、融合0 | 没有位姿时初始化成功也不能建图，不能误判GPU性能不足 |
| 深度闪黑 | 18秒已收320帧无全无效帧；10秒轻量可靠收296帧 | 本轮未复现全黑输入，显示范围与间歇问题需区分 |
| 测频无输出 | 普通topic hz固定BEST_EFFORT；此前大点云接收波动明显 | 当前采用RELIABLE raw接收确认实际有数据，不把无打印等同无发布 |
| 当前恢复后的记录 | 5mm配置，深度融合29.55Hz、ESDF更新/接收约29.03Hz | 已具备持续融合和输出条件；本轮未再替用户改TF或其他参数 |

前面TF故障诊断时实际Mapper体素曾为0.01m；本轮参数为0.005m。两者不是严格单变量A/B，不能将0→29Hz全部解释成体素优化；TF连通是之前完全无法融合的直接条件。

[前次无重建与闪黑诊断](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/docs/REAL_NO_MAP_DEPTH_FLICKER_DIAGNOSIS_20261111.md)保存了详细证据、图像和解决指令。本轮没有同步录制RViz闪黑瞬间，不能宣称显示闪黑问题已经修复。

### 为什么此前5mm仿真约10Hz，当前实机能接近30Hz

不是因为把Python换成了另一套核心。此前仿真扫描约637,691点、15.3MB/条、Mesh约33.8万三角面；本轮约146,902点、3.53MB/条、Mesh约10.3万三角面。地图范围内实际观测规模、场景结构、相机视角和GUI负载均不同。

此前仿真扫描末ESDF导出平均约34.90ms、打包约41.29ms；本轮约3.21ms与6.37ms。当前数据负载较小，整体才更容易跟上30Hz输入。此比较用于解释可能的负载差异，**不是同输入、同地图、同负载的严格性能A/B，也不能证明ROS/DDS、场景大小分别贡献了多少加速**。

全图同步/导出与完整Mesh发送的结构性成本仍存在；地图继续扩大后可能再次下降。即使维持5mm体素，静止近场与多视角大场景的频率也不能直接等同。

## 6. 保存地图与ESDF数据是什么

| 内容 | 保存结果 |
|---|---|
| Mesh | 65,130顶点，102,822三角面 |
| 原生ESDF地图 | 3,337块，1,171,455个observed体素 |
| 最后收到的ESDF点云 | 146,902点，frame=base_link |
| 显示点内部标记 | inside=19,327，site=6,625 |
| 距离数值范围 | -2.00…2.00m |
| 达到绝对距离约2m上限 | 58个显示采样点 |

原生observed体素与显示点数不同，显示经过stride2采样；unknown未作为空点填充到observed点云。inside为内部距离符号，不等同“所有这些点都是障碍物表面”。

58个点达到最大距离上限不等于58个已证明错误的点：本轮没有实机几何真值，也没有逐点核查传播父方向，因此不能判定其真实误差。之前仿真确认过少量距离高估，仍需作为后续正确性排查项，而不是用本轮较高频率宣称已解决。

本轮没有提供实机Mesh/ESDF误差MAE、P95或毫米级抓取精度，因为没有独立真值。频率、点数、体素精度和几何准确度是不同概念。

保存发生在频率测量之后，以免保存服务耗时进入测量窗口。Mesh和BLOX是顺序服务快照，最后点云是稍早收到的输出，三者不严格同一帧；点数不可强制逐字节相等。

## 7. 数据文件、CSV和复核

目录：`/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111`。

| 文件 | 用途 |
|---|---|
| [result.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/result.json) | 完整配置、频率、三段统计、延迟代理、保存结果 |
| [metrics_events.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/metrics_events.json) | 原始metrics及其接收时间 |
| [cloud_events.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/cloud_events.json) | 每条ESDF的接收时刻、输入stamp、点数、字节数及代理年龄 |
| [stage_rates.csv](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/stage_rates.csv) | 可直接在Excel导入的分段频率数据 |
| [timings.csv](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/timings.csv) | 原有阶段平均/P95耗时 |
| [params.yaml](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/params.yaml) | 节点实际参数转储 |
| [mesh.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/mesh.ply) | 彩色三角网格，可用CloudCompare查看 |
| [map.blox](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/map.blox) | 原生TSDF/Color/ESDF等层 |
| [last_esdf.cdr](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/last_esdf.cdr) | 最后实际收到的完整PointCloud2消息 |
| [esdf_points.ply](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/esdf_points.ply) | 实际显示点的xyz、距离、RGB及状态标记 |
| [esdf_points.npy](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/esdf_points.npy) | 同一点云的结构化数组 |
| [joint_samples.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/joint_samples.json) | 约10Hz抽样关节记录，用于判断本轮运动量 |
| [validation.json](/home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand/artifacts/real_realtime_record_20261111/validation.json) | 成功状态、配置一致性、数据字段与文件哈希核对 |

CloudCompare打开mesh.ply选择RGB；打开esdf_points.ply可选择distance标量场，单位m。PLY来自实际接收消息，没有人为补unknown或改变距离值。

## 8. 后续测试建议

1. 在相同5mm、输入分辨率、工作区及显示订阅条件下，记录“静止→操作员手拖→停止”三段，比较频率、时间戳年龄、队列覆盖和残影；本轮不能替代运动测试。
2. 持续5～10分钟并记录地图点数增长，验证负载增加及温度变化后的频率，不能只看当前30秒。
3. 用尺寸已知标块或独立参考点云评价Mesh几何、颜色边界和ESDF距离，分别报告覆盖和unknown；真实场景缺少真值时不编造精度数值。
4. 闪黑时同时保存原始深度和显示画面，确认是输入无效还是显示层；保持每个终端ROS域一致。

继续查看指令：

```bash
cd /home/czz/perception_eye_in_hand/ur7e_nvblox_in_hand
export ROS_DOMAIN_ID=135 ROS_LOCALHOST_ONLY=1
source ./env.sh
python3 tools/watch_realtime.py
# Ctrl+C后查看实际接收
python3 tools/watch_esdf_hz.py
```

本轮记录脚本固定输出目录且拒绝覆盖已有result.json；重新测试应使用新的输出目录。原始数据未覆盖前面的故障诊断结果，也没有将仿真数据冒充实机结果。
