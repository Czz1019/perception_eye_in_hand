# 三维ESDF实际输出实时性对比：九轮新测试

日期：2026-11-05。工程：`/home/czz/perception_eye_in_hand/nvblox_in_hand`。

本报告重新进行九轮仿真，以**三维ESDF输出**为主要对象，不用深度融合率、ESDF入口调用率或Mesh率代替实际三维点云结果。本轮每组均开启相同诊断导出和RViz；三维输出来自/nvblox_node/esdf_points，数据为原生observed ESDF体素。

## 1. 这次真正比较什么

| 指标 | 含义 | 能说明什么 / 不能说明什么 |
| --- | --- | --- |
| ESDF更新调用Hz | 官方ros/esdf累计计数差/墙钟时间 | 内部周期任务调用；可能没有新深度，不能当全新地图率 |
| 三维ESDF点云发布Hz | 实际点云接收间隔计算 | 已收到的可读距离点云流；可能重复关联同一深度帧 |
| 新深度关联输出Hz | 相邻点云header对应的最新深度时间戳向前推进次数/墙钟时间 | 输出是否跟随不同深度输入；不是逐体素几何变化率或每张输入都被单独输出 |
| 重复关联比例 | 相邻点云header相同的比例 | 发布了多少重复关联输入消息；不通过点排序或内容哈希判断 |
| 点云输入年龄P50/P95 | 回调时最近收到的/clock减点云header时间戳 | 包含输入等待/融合/ESDF/导出和接收等环节的近似数据年龄；受时钟与回调排队影响，不是纯核函数或屏幕端到端延迟 |
| 发布间隔P95 | 相邻点云接收墙钟间隔的95分位 | 输出流的间隔尾部；不是GPU计算耗时 |

当前诊断代码在processEsdf后把最新已融合深度时间戳写入点云header；发布条件只要求ESDF调用版本变化，不要求该深度时间戳变化，因此会有重复关联输入。这也是本次必须同时统计“发布”和“新深度关联”的原因。

本次直接记录整数纳秒header，不再从年龄反推时间戳。所有组都采集真正的PointCloud2消息，而非仅看配置或控制台调用次数。

## 2. 相同负载与测试配置

| 项目 | 设置 |
| --- | --- |
| 变量 | 深度/颜色融合门限30、33.4、40Hz；每档3次 |
| ESDF更新/导出显示门限 | 所有组均为33.4Hz / 33.4Hz |
| 输入/精度/范围 | RGB-D 640×480、约30Hz；0.03m体素、3m融合范围 |
| 机器人/位姿 | 同一0.82m高位静止相机，图像时间机械臂TF；无扫描运动 |
| 后端 | 相同diagnostic派生节点，内部调用同一个官方Mapper；每轮新建地图 |
| ESDF输出 | observed模式，inside与outside均保留；24字节点布局 |
| 显示与订阅 | 所有组同样开启3D ESDF RViz、RGB显示和Mesh记录订阅；Mesh调度5Hz |
| 过程 | 输入/Mesh/RViz预热、5秒静止基线、约20秒正式采样，结束后保存体素及网格 |
| 顺序 | 30→33.4→40→40→30→33.4→33.4→40→30 |
| 环境 | 独立domain129，Gazebo端口11459；同一场景持续运行 |

本轮与上一份九轮融合报告的关键差别是：**上一轮仅官方组件，本轮九组全部启用三维导出。**额外负载一致，不能把上一轮无导出融合数直接填入本轮表。

静态场景本身可能不发生几何变化，所以“关联新深度”仍不等于“几何必定变化”。两次点云之间可能合并多个融合输入，本指标也不是逐帧ESDF参与率。

## 3. 主要结果：三维ESDF输出实时性

频率为三轮均值；输入年龄和接收间隔分位数对每组三轮正式采样消息合并计算，不是将三个P95简单平均。

| 融合门限 | ESDF调用Hz | 3D点云发布Hz | 新深度关联输出Hz | 相邻重复关联 | 输入年龄P50 / P95 | 发布间隔P95 |
| --- | ---: | ---: | ---: | ---: | --- | ---: |
| 30Hz | 32.87 | 29.47 | 17.07 | 42.0% | 59 / 106ms | 40.30ms |
| 33.4Hz | 32.88 | 29.41 | 19.83 | 32.6% | 50 / 77ms | 40.26ms |
| 40Hz | 32.91 | 29.26 | 19.77 | 32.4% | 50 / 77ms | 40.39ms |

![三维ESDF实际输出实时性对比](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_realtime_repeats_20261105/esdf_realtime_comparison.png)

### 本轮应该怎样解读

- 融合门限30Hz时，三维消息发布29.47Hz，但关联新深度的输出只有17.07Hz；相邻重复关联约42.0%。
- 融合门限33.4Hz时，消息发布29.41Hz，新深度关联19.83Hz，输入年龄P95约77ms。
- 融合门限40Hz时，消息发布29.26Hz，新深度关联19.77Hz，输入年龄P95约77ms。

因此：**三组的显示消息都接近30Hz，不代表ESDF每秒跟随了30张不同深度图。**本次实际揭示的是不同融合门限下的关联输入推进与数据年龄差异。是否更实时应结合这些列，而非只看ESDF调用或topic hz。

对比30Hz融合组与40Hz组，新深度关联输出变化为17.07→19.77Hz，输入年龄P95为106→77ms。即使输入融合率已接近30Hz，当前调度/导出链仍没有证明达到30Hz的新深度关联输出。

本轮没有改算法去消除重复关联；结果如实保留。此处新深度关联仍是输出输入版本指标，不是不同几何地图计数或GPU逐帧完成率。


### 九轮逐轮值

| 顺序 | 融合门限/重复轮 | 点云发布Hz | 新深度关联输出Hz | 重复关联比例 | 输入年龄P50/P95 | 接收间隔P95 |
| --- | --- | ---: | ---: | ---: | --- | ---: |
| 1 | 30Hz / 1 | 29.30 | 17.44 | 40.5% | 60/102ms | 40.35ms |
| 2 | 33.4Hz / 1 | 29.32 | 19.94 | 32.0% | 50/77ms | 40.06ms |
| 3 | 40Hz / 1 | 29.18 | 19.62 | 32.8% | 50/83ms | 40.68ms |
| 4 | 40Hz / 2 | 29.27 | 19.86 | 32.1% | 50/80ms | 40.22ms |
| 5 | 30Hz / 2 | 29.64 | 17.18 | 42.0% | 59/103ms | 40.00ms |
| 6 | 33.4Hz / 2 | 29.33 | 19.62 | 33.1% | 50/77ms | 40.49ms |
| 7 | 33.4Hz / 3 | 29.58 | 19.92 | 32.7% | 50/77ms | 40.07ms |
| 8 | 40Hz / 3 | 29.32 | 19.81 | 32.4% | 49/76ms | 40.36ms |
| 9 | 30Hz / 3 | 29.46 | 16.60 | 43.7% | 60/108ms | 40.56ms |

### 各组三轮频率波动

| 融合门限 | 点云发布Hz：均值±样本标准差（范围） | 新深度关联输出Hz：均值±样本标准差（范围） |
| --- | --- | --- |
| 30Hz | 29.47±0.17（29.30～29.64） | 17.07±0.43（16.60～17.44） |
| 33.4Hz | 29.41±0.15（29.32～29.58） | 19.83±0.18（19.62～19.94） |
| 40Hz | 29.26±0.07（29.18～29.32） | 19.77±0.13（19.62～19.86） |

## 4. 输入融合率仅作为解释性数据

下表来自本轮有导出负载的同一九次测试，不是主要实时性结论。

| 融合门限 | 深度实际融合Hz | 颜色实际融合Hz | Mesh发布Hz | 每条3D点云点数范围 |
| --- | ---: | ---: | ---: | --- |
| 30Hz | 18.37 | 18.28 | 4.75 | 24184～24184 |
| 33.4Hz | 29.62 | 29.25 | 4.75 | 24184～24184 |
| 40Hz | 29.78 | 29.74 | 4.75 | 24184～24184 |

融合门限30Hz的严格最小间隔为33.333ms，正常33ms输入会被跳过；33.4和40Hz分别放宽到29.94和25ms，恢复更多输入。即使融合接近30Hz，ESDF周期与点云导出并非对每张新深度逐一触发，因此输出的新深度关联率仍可能更低。

## 5. 为什么仅提高调用/显示Hz不能代表ESDF更实时

1. 官方tick中的ESDF周期可以在没有新深度时调用；本轮三档均固定33.4Hz，调用率理应接近。
2. 诊断导出用esdf_调用计数当版本，未用新深度时间戳当唯一发布条件；所以topic hz可接近29～30，却有相邻重复关联输入。
3. 融合、ESDF周期、wall-clock显示门限和图像到达不同相位，某次导出可能重复旧输入，另一段又可能合并多张输入，不能把消息数当逐帧刷新数。
4. 原生导出还包含块扫描、CUDA同步/读回、PointCloud2组包和DDS传输；本轮测到了接收输出，未额外用CUDA事件拆分每段耗时。

源码：[ESDF时间戳与导出条件](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/mapper_node.cpp:67)、[GPU原生块导出](/home/czz/perception_eye_in_hand/nvblox_in_hand/src/nvblox_in_hand/src/esdf_pointcloud_exporter.cu:47)。

## 6. 与此前“ESDF门限30→33.4”的区别

本次固定ESDF/显示门限，单独观察融合准入对ESDF输出新鲜度的影响。此前另一次节拍测试为：

| 历史配置（融合均40） | ESDF调用Hz | 三维点云Hz | 新深度关联推进Hz |
| --- | ---: | ---: | ---: |
| ESDF30 / 显示30 | 24.62 | 24.58 | 19.91 |
| ESDF33.4 / 显示33.4 | 32.59 | 29.44 | 19.75 |

这张仅是**历史实验**，当时相机姿态、点数/运动条件与本轮不同，不能拼成同条件单因素对照。它说明提高ESDF/显示门限可以提高消息率，但不保证新观测推进率同幅提升。旧轮推进率由记录时钟/年龄反推，本轮则直接记录整数header。

## 7. 实际ESDF窗口

以下均来自本轮三维诊断输出，不是Mesh截图，也不是官方2D切片。

### 融合门限30Hz组

![三维ESDF 30Hz融合组](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_realtime_repeats_20261105/gate30p0_r1_measure.png)

### 融合门限33.4Hz组

![三维ESDF 33.4Hz融合组](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_realtime_repeats_20261105/gate33p4_r1_measure.png)

### 融合门限40Hz组

![三维ESDF 40Hz融合组](/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_realtime_repeats_20261105/gate40p0_r1_measure.png)

截图用于确认真实输出和显示，不用RViz右下角FPS代替ESDF频率；跨窗口相机视角与显示交互可能不同，不据此比较几何精度。

## 8. 已实施与下一步

本轮只增加测试采集和分析，不修改生产感知算法或默认参数。默认仍为融合40、ESDF33.4、诊断显示33.4；本轮三组仅通过CLI覆盖融合门限。记录的是三维消息接收，不再仅用更新调用代替实时性。

后续真正改善ESDF输出应考虑：按新深度/变化块版本安排更新和导出，避免空调用与重复关联发布；分离更新节拍和显示节拍，检查输入到达/处理相位；用逐帧关联及CUDA事件定位GPU完成、导出与传输耗时。只把重复消息删掉可以减少负载，但不会凭空提高新地图生成率，需要与更新调度一起评估。

本轮静止轨迹未量化运动残影和重建精度。unknown、自体污染和颜色混合仍是独立问题，不因点云发布接近30Hz而解决。

## 9. 复现命令

手动比较三组时，先停止旧场景/Mapper，每组仅启动一次；以下三条互选，ESDF与显示全部固定33.4Hz。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
./run_esdf.sh gazebo_gui:=true depth_hz:=30 color_hz:=30 esdf_hz:=33.4 esdf_view_hz:=33.4
```

```bash
./run_esdf.sh gazebo_gui:=true depth_hz:=33.4 color_hz:=33.4 esdf_hz:=33.4 esdf_view_hz:=33.4
```

```bash
./run_esdf.sh gazebo_gui:=true depth_hz:=40 color_hz:=40 esdf_hz:=33.4 esdf_view_hz:=33.4
```

新终端加载同样环境后：

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=127 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11457
source ./env.sh
ros2 topic hz /nvblox_node/esdf_points --wall-time
# Ctrl+C后，核对实际参数/现有状态
ros2 param get /nvblox_node integrate_depth_rate_hz
ros2 param get /nvblox_node update_esdf_rate_hz
ros2 param get /nvblox_node esdf_view_hz
ros2 topic echo /nvblox_node/metrics --once
```

单独topic hz不能提供重复关联率和年龄分布，要复现本文完整九轮采集，先停止手动场景、备份结果目录再执行下列命令；同名证据会覆盖。自动脚本包含进入高位动作，只用于本项目仿真。

```bash
cd /home/czz/perception_eye_in_hand/nvblox_in_hand
export ROS_DOMAIN_ID=129 ROS_LOCALHOST_ONLY=1 DISPLAY=:1
export GAZEBO_MASTER_URI=http://127.0.0.1:11459
source ./env.sh
/usr/bin/python3 artifacts/esdf_realtime_repeats_20261105/run_measurement.py
/usr/bin/python3 artifacts/esdf_realtime_repeats_20261105/analyze_fusion_counts.py
/usr/bin/python3 artifacts/esdf_realtime_repeats_20261105/analyze_esdf_realtime.py
MPLCONFIGDIR=/tmp/esdf-realtime-mpl /usr/bin/python3 artifacts/esdf_realtime_repeats_20261105/plot_esdf_realtime.py
```

DISPLAY以实际桌面为准。本次没有安装新依赖或重编官方核心，运行时核验组件参数、输入、单一发布器和原生点格式，脚本已做语法检查。

## 10. 原始证据与统计边界

目录：`/home/czz/perception_eye_in_hand/nvblox_in_hand/artifacts/esdf_realtime_repeats_20261105`。

| 文件 | 内容 |
| --- | --- |
| results.json、samples.json | 阶段、实际参数、输入、既有统计服务时间区间 |
| cloud_records.json | 每条三维ESDF的直接整数header、接收墙钟、最近/clock、点数与字节数 |
| depth_receipts.json | 同一观察节点的原始深度整数header及接收墙钟 |
| analysis.json、esdf_realtime_comparison.csv | 主要ESDF实时性数据与可导入表格 |
| fusion_counts.json | 同轮官方融合/ESDF调用计数，仅作解释性数据 |
| *_runtime_parameters.yaml、*_timings.txt、*_rates.txt | 生效配置及原有官方统计 |
| *_esdf.npy、mesh_*.ply | 原生ESDF快照和地图表面 |
| *_measure.png、*_rgb.png | 真实三维窗口和输入截图 |
| source_manifest.json | 本轮源码、配置与报告哈希 |

数据年龄采用回调时最近收到的仿真时钟，存在时钟接收与回调队列误差；无法当作精确曝光到屏幕延迟。另记录同一观察者原始深度消息到匹配ESDF消息的接收差及匹配率，保存在analysis.json，这种差值也不是传感器真实曝光到发布器延迟，可能受两个订阅的排队顺序影响。

本次没有相机屏幕同步、高速录像、逐输入GPU完成事件或内容真值变化标记，不能宣称已验证完整屏幕端到端实时性或每个体素的更新延迟。采用每档三次、约20秒短窗，不能替代长时间大场景稳定性/最坏延迟验证。
